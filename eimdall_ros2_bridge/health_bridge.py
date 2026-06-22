import json
from pathlib import Path
from typing import Any, Optional

import rclpy
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

from eimdall_ros2_bridge.msg import EimdallHealth, EimdallSensorStatus

_HEALTH_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

_DEFAULT_MAX_HEALTH_FILE_BYTES = 1_048_576  # 1 MiB


class HealthBridge(LifecycleNode):
    def __init__(self) -> None:
        super().__init__("eimdall_health_bridge")
        self.declare_parameter("health_path", "runtime_health.json")
        self.declare_parameter("publish_period_sec", 1.0)
        self.declare_parameter("max_file_bytes", _DEFAULT_MAX_HEALTH_FILE_BYTES)

        self._health_pub = None
        self._sensor_pub = None
        self._diag_pub = None
        self._timer = None
        self._health_path: Optional[Path] = None
        self._publish_period: float = 1.0
        self._max_file_bytes: int = _DEFAULT_MAX_HEALTH_FILE_BYTES
        self._last_raw: Optional[str] = None
        self._ticks: int = 0
        self._parse_errors: int = 0
        self._publishes: int = 0
        self._oversize_ticks: int = 0  # #688: consecutive ticks with oversized file

    # ── Lifecycle callbacks ─────────────────────────────────────────────────

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        raw_path = self.get_parameter("health_path").get_parameter_value().string_value
        try:
            self._health_path = Path(raw_path).resolve()
        except (ValueError, OSError) as exc:
            self.get_logger().error(f"invalid health_path '{raw_path}': {exc}")
            return TransitionCallbackReturn.FAILURE
        self._publish_period = (
            self.get_parameter("publish_period_sec").get_parameter_value().double_value
        )
        self._max_file_bytes = (
            self.get_parameter("max_file_bytes").get_parameter_value().integer_value
        )
        self._health_pub = self.create_lifecycle_publisher(EimdallHealth, "/eimdall/health", _HEALTH_QOS)
        self._sensor_pub = self.create_lifecycle_publisher(
            EimdallSensorStatus, "/eimdall/sensors/status", _HEALTH_QOS
        )
        self._diag_pub = self.create_lifecycle_publisher(DiagnosticArray, "/diagnostics", 10)  # #22
        self.get_logger().info(f"configured: health_path={self._health_path}")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self._timer = self.create_timer(self._publish_period, self._tick)
        self.get_logger().info("activated")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        if self._timer is not None:
            self._timer.cancel()
            self._timer = None
        self.get_logger().info("deactivated")
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        # #19: destroy publishers before releasing references
        for pub in (self._health_pub, self._sensor_pub, self._diag_pub):
            if pub is not None:
                self.destroy_publisher(pub)
        self._health_pub = None
        self._sensor_pub = None
        self._diag_pub = None
        self._last_raw = None
        self._ticks = 0
        self._parse_errors = 0
        self._publishes = 0
        self._oversize_ticks = 0  # #19: reset oversize counter
        self.get_logger().info("cleaned up")
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS

    # ── Timer callback ──────────────────────────────────────────────────────

    def _tick(self) -> None:
        self._ticks += 1
        if self._health_path is None or not self._health_path.exists():
            self._publish_diagnostics(ok=False, message="health file not found")
            return

        try:
            file_size = self._health_path.stat().st_size
            if file_size > self._max_file_bytes:
                self._oversize_ticks += 1
                # #688: after 3 consecutive oversize ticks, rotate the file to self-recover.
                _OVERSIZE_ROTATE_AFTER = 3
                self._publish_diagnostics(
                    ok=False,
                    message=(
                        f"health file too large ({file_size} B > {self._max_file_bytes} B), "
                        f"tick {self._oversize_ticks}/{_OVERSIZE_ROTATE_AFTER}"
                    ),
                )
                if self._oversize_ticks >= _OVERSIZE_ROTATE_AFTER:
                    backup = self._health_path.with_suffix(".oversize.bak")
                    try:
                        self._health_path.rename(backup)
                        self.get_logger().error(
                            "Health file rotated to %s after %d consecutive oversize ticks",
                            backup, self._oversize_ticks,
                        )
                    except OSError as rot_exc:
                        self.get_logger().error("Failed to rotate oversized health file: %s", rot_exc)
                    self._oversize_ticks = 0
                else:
                    self.get_logger().warning(
                        "health file exceeds limit (%d B) — skipping tick (backpressure)", file_size
                    )
                return
            self._oversize_ticks = 0  # reset on normal tick
            raw = self._health_path.read_text(encoding="utf-8")
        except OSError as exc:
            self._publish_diagnostics(ok=False, message=str(exc))
            self.get_logger().error(f"failed to read health file: {exc}")
            return

        if not raw.strip():
            self.get_logger().debug("health file is empty — skipping")
            return

        if raw == self._last_raw:
            return

        try:
            payload = json.loads(raw)
        except (json.JSONDecodeError, UnicodeDecodeError):
            self._parse_errors += 1
            self._publish_diagnostics(
                ok=False,
                message="JSON parse deferred until next health tick",
            )
            self.get_logger().warning(
                "health JSON is incomplete or invalid — keeping the last valid snapshot"
            )
            return

        self._publish_global_health(payload)
        self._publish_sensor_health(payload)
        self._last_raw = raw
        self._publishes += 1
        self._publish_diagnostics(ok=True, message=f"published snapshot #{self._publishes}")

    # ── Publishers ──────────────────────────────────────────────────────────

    def _publish_global_health(self, payload: dict[str, Any]) -> None:
        msg = EimdallHealth()
        msg.stamp = self.get_clock().now().to_msg()
        msg.uptime_s = float(payload.get("uptime_s") or 0.0)
        msg.configured_sensors = int(payload.get("configured_sensors") or 0)
        msg.active_sensors = int(payload.get("active_sensors") or 0)
        msg.total_reconnects = int(payload.get("total_reconnects") or 0)
        msg.total_consecutive_errors = int(payload.get("total_consecutive_errors") or 0)
        msg.total_lines_seen = int(payload.get("total_lines_seen") or 0)
        msg.total_parse_errors = int(payload.get("total_parse_errors") or 0)
        msg.total_processed_values = int(payload.get("total_processed_values") or 0)
        msg.total_anomaly_events = int(payload.get("total_anomaly_events") or 0)
        self._health_pub.publish(msg)

    def _publish_sensor_health(self, payload: dict[str, Any]) -> None:
        sensors = payload.get("sensors", [])
        if not isinstance(sensors, list):
            self.get_logger().warning("health payload has invalid 'sensors' field")
            return
        for sensor in sensors:
            if not isinstance(sensor, dict):
                continue
            msg = EimdallSensorStatus()
            msg.stamp = self.get_clock().now().to_msg()
            sensor_label = str(sensor.get("sensor_label") or sensor.get("sensor_id") or "")
            msg.sensor_id = str(sensor.get("sensor_id") or sensor_label)
            msg.family = str(sensor.get("family") or _family_from_label(sensor_label))
            msg.status = str(sensor.get("status") or _status_from_edge_sensor(sensor))
            msg.confidence_pct = float(sensor.get("confidence_pct") or _confidence_from_status(msg.status))
            msg.recent_readings = int(
                sensor.get("recent_readings") or sensor.get("processed_values") or 0
            )
            msg.last_reading_at_ms = int(
                sensor.get("last_reading_at_ms") or sensor.get("last_line_ts_ms") or 0
            )
            self._sensor_pub.publish(msg)

    def _publish_diagnostics(self, ok: bool, message: str) -> None:
        if self._diag_pub is None:
            return
        status = DiagnosticStatus()
        status.name = "eimdall_health_bridge"
        status.hardware_id = str(self._health_path or "")
        status.level = DiagnosticStatus.OK if ok else DiagnosticStatus.ERROR
        status.message = message
        status.values = [
            KeyValue(key="ticks", value=str(self._ticks)),
            KeyValue(key="snapshots_published", value=str(self._publishes)),
            KeyValue(key="parse_errors", value=str(self._parse_errors)),
        ]
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.status = [status]
        self._diag_pub.publish(arr)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HealthBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


def _family_from_label(sensor_label: str) -> str:
    if ":" in sensor_label:
        return sensor_label.split(":", 1)[0]
    if "-" in sensor_label:
        return sensor_label.split("-", 1)[0]
    return sensor_label


def _status_from_edge_sensor(sensor: dict[str, Any]) -> str:
    if int(sensor.get("consecutive_errors") or 0) > 0:
        return "warning"
    if int(sensor.get("parse_errors") or 0) > 0:
        return "warning"
    if int(sensor.get("processed_values") or 0) == 0:
        return "offline"
    return "ok"


def _confidence_from_status(status: str) -> float:
    return {
        "ok": 100.0,
        "warning": 70.0,
        "error": 40.0,
        "offline": 0.0,
    }.get(status, 50.0)
