import json
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

from eimdall_ros2_bridge.msg import EimdallAnomaly

_ANOMALY_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

_SEVERITY_MAP = {
    "info": 1,
    "low": 1,
    "warning": 2,
    "warn": 2,
    "medium": 2,
    "high": 3,
    "critical": 4,
    "error": 4,
}


_DEFAULT_MAX_ANOMALIES_PER_TICK = 200


class AnomalyBridge(LifecycleNode):
    def __init__(self) -> None:
        super().__init__("eimdall_anomaly_bridge")
        self.declare_parameter("anomaly_path", "runtime_anomalies.jsonl")
        self.declare_parameter("poll_period_sec", 0.5)
        self.declare_parameter("max_anomalies_per_tick", _DEFAULT_MAX_ANOMALIES_PER_TICK)

        self._pub = None
        self._diag_pub = None
        self._timer = None
        self._anomaly_path: Optional[Path] = None
        self._poll_period: float = 0.5
        self._max_anomalies_per_tick: int = _DEFAULT_MAX_ANOMALIES_PER_TICK
        self._offset: int = 0
        self._last_inode: Optional[int] = None
        self._events_published: int = 0
        self._errors: int = 0

    # ── Lifecycle callbacks ─────────────────────────────────────────────────

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        raw_path = self.get_parameter("anomaly_path").get_parameter_value().string_value
        try:
            self._anomaly_path = Path(raw_path).resolve()
        except (ValueError, OSError) as exc:
            self.get_logger().error(f"invalid anomaly_path '{raw_path}': {exc}")
            return TransitionCallbackReturn.FAILURE
        self._poll_period = (
            self.get_parameter("poll_period_sec").get_parameter_value().double_value
        )
        self._max_anomalies_per_tick = (
            self.get_parameter("max_anomalies_per_tick").get_parameter_value().integer_value
        )
        self._pub = self.create_lifecycle_publisher(EimdallAnomaly, "/eimdall/anomalies", _ANOMALY_QOS)
        self._diag_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.get_logger().info(f"configured: anomaly_path={self._anomaly_path}")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self._timer = self.create_timer(self._poll_period, self._tick)
        self.get_logger().info("activated")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        if self._timer is not None:
            self._timer.cancel()
            self._timer = None
        self.get_logger().info("deactivated")
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        # #20: destroy publishers before releasing references
        if self._pub is not None:
            self.destroy_publisher(self._pub)
            self._pub = None
        if self._diag_pub is not None:
            self.destroy_publisher(self._diag_pub)
            self._diag_pub = None
        self._offset = 0
        self._last_inode = None
        self._events_published = 0
        self._errors = 0
        self.get_logger().info("cleaned up")
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS

    # ── Timer callback ──────────────────────────────────────────────────────

    def _tick(self) -> None:
        if self._anomaly_path is None or not self._anomaly_path.exists():
            self._publish_diagnostics(ok=False, message="anomaly file not found")
            return

        try:
            stat = self._anomaly_path.stat()
            # Rotation detection: inode changed or file shrank
            if self._last_inode is not None and (
                stat.st_ino != self._last_inode or stat.st_size < self._offset
            ):
                self.get_logger().info("anomaly file rotated — resetting offset")
                self._offset = 0
            self._last_inode = stat.st_ino

            tick_count = 0
            with self._anomaly_path.open("r", encoding="utf-8") as fh:
                fh.seek(self._offset)
                while True:
                    if tick_count >= self._max_anomalies_per_tick:
                        self.get_logger().warning(
                            f"anomaly backlog: processed {tick_count} events this tick "
                            f"(limit={self._max_anomalies_per_tick}) — will resume next tick"
                        )
                        # Do NOT advance offset further; resume next tick.
                        break
                    line = fh.readline()
                    if not line:
                        break
                    self._offset = fh.tell()
                    tick_count += 1
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        payload = json.loads(line)
                        self._pub.publish(self._build_msg(payload))
                        self._events_published += 1
                    except Exception as exc:
                        self._errors += 1
                        self.get_logger().error(f"parse error: {exc}")

            self._publish_diagnostics(
                ok=True,
                message=f"published {self._events_published} total events (tick: {tick_count})",
            )

        except Exception as exc:
            self._errors += 1
            self._publish_diagnostics(ok=False, message=str(exc))
            self.get_logger().error(f"failed to read anomaly file: {exc}")

    # ── Helpers ─────────────────────────────────────────────────────────────

    def _build_msg(self, payload: dict) -> EimdallAnomaly:
        msg = EimdallAnomaly()
        msg.stamp = self.get_clock().now().to_msg()
        msg.event_id = str(payload.get("event_id", ""))
        msg.robot_id = str(payload.get("robot_id", ""))
        msg.tenant_id = str(payload.get("tenant_id", ""))
        msg.component = str(payload.get("component", ""))
        msg.severity = _severity_to_int(payload.get("severity", 0))
        msg.score = float(payload.get("score", 0.0))
        rc = payload.get("reason_codes", [])
        msg.reason_codes = [str(item) for item in rc] if isinstance(rc, list) else []
        return msg

    def _publish_diagnostics(self, ok: bool, message: str) -> None:
        if self._diag_pub is None:
            return
        status = DiagnosticStatus()
        status.name = "eimdall_anomaly_bridge"
        status.hardware_id = str(self._anomaly_path or "")
        status.level = DiagnosticStatus.OK if ok else DiagnosticStatus.ERROR
        status.message = message
        status.values = [
            KeyValue(key="events_published", value=str(self._events_published)),
            KeyValue(key="errors", value=str(self._errors)),
            KeyValue(key="byte_offset", value=str(self._offset)),
        ]
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.status = [status]
        self._diag_pub.publish(arr)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AnomalyBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


def _severity_to_int(value) -> int:
    if isinstance(value, str):
        value = value.strip().lower()
        if value in _SEVERITY_MAP:
            return _SEVERITY_MAP[value]
    try:
        return int(value)
    except (TypeError, ValueError):
        return 0
