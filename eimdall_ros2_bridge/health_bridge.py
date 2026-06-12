import json
import time
from pathlib import Path
from typing import Any, Optional

import rclpy
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

from eimdall_ros2_bridge.msg import EimdallHealth, EimdallSensorStatus

_HEALTH_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

_MAX_JSON_RETRIES = 3
_RETRY_DELAY_S = 0.05


class HealthBridge(LifecycleNode):
    def __init__(self) -> None:
        super().__init__("eimdall_health_bridge")
        self.declare_parameter("health_path", "runtime_health.json")
        self.declare_parameter("publish_period_sec", 1.0)

        self._health_pub = None
        self._sensor_pub = None
        self._diag_pub = None
        self._timer = None
        self._health_path: Optional[Path] = None
        self._publish_period: float = 1.0
        self._last_raw: Optional[str] = None
        self._ticks: int = 0
        self._parse_errors: int = 0
        self._publishes: int = 0

    # ── Lifecycle callbacks ─────────────────────────────────────────────────

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self._health_path = Path(
            self.get_parameter("health_path").get_parameter_value().string_value
        )
        self._publish_period = (
            self.get_parameter("publish_period_sec").get_parameter_value().double_value
        )
        self._health_pub = self.create_lifecycle_publisher(EimdallHealth, "/eimdall/health", _HEALTH_QOS)
        self._sensor_pub = self.create_lifecycle_publisher(
            EimdallSensorStatus, "/eimdall/sensors/status", _HEALTH_QOS
        )
        self._diag_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
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
        self._last_raw = None
        self._ticks = 0
        self._parse_errors = 0
        self._publishes = 0
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

        # Retry loop guards against partial writes / truncation mid-write
        payload: Optional[dict] = None
        for attempt in range(_MAX_JSON_RETRIES):
            try:
                payload = json.loads(raw)
                break
            except json.JSONDecodeError:
                if attempt < _MAX_JSON_RETRIES - 1:
                    time.sleep(_RETRY_DELAY_S)
                    try:
                        raw = self._health_path.read_text(encoding="utf-8")
                    except OSError:
                        break

        if payload is None:
            self._parse_errors += 1
            self._publish_diagnostics(ok=False, message="JSON parse failed after retries")
            self.get_logger().error(f"failed to parse health JSON after {_MAX_JSON_RETRIES} attempts")
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
            msg.sensor_id = str(sensor.get("sensor_id", ""))
            msg.family = str(sensor.get("family", ""))
            msg.status = str(sensor.get("status", ""))
            msg.confidence_pct = float(sensor.get("confidence_pct") or 0.0)
            msg.recent_readings = int(sensor.get("recent_readings") or 0)
            msg.last_reading_at_ms = int(sensor.get("last_reading_at_ms") or 0)
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
