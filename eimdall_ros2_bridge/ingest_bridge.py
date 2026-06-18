"""IngestBridge — ROS 2 sensor topics → Eimdall Edge ingest."""
import collections
import math
import os
import queue
import re
import stat
import threading
import time
from typing import Dict, Deque, List, Optional, Tuple

# #596: bounded sender queue — drops oldest entries under high load (newest-wins)
_INGEST_QUEUE_MAX = 500
# #644: maximum serialised payload size accepted per ingest call (64 KiB)
_MAX_PAYLOAD_BYTES = 65_536
# #644: maximum messages per second per sensor topic before dropping
_SENSOR_RATE_LIMIT = 200

_SAFE_ID = re.compile(r"^[a-zA-Z0-9_\-\.]{1,64}$")
_MAX_SCAN_SAMPLES = 1_000
_ERRORS_WINDOW_S = 300.0  # 5-minute sliding window for error rate reporting


def _sanitize_id(name: str) -> Optional[str]:
    """Return name if it contains only safe identifier characters, else None."""
    return name if _SAFE_ID.match(name) else None


def _finite(v: float) -> Optional[float]:
    """Return v if finite, else None (guards against NaN/Inf → invalid JSON)."""
    return v if math.isfinite(v) else None


def _safe_values(raw: Dict[str, float]) -> Optional[Dict[str, float]]:
    """Drop NaN/Inf entries; return None if no finite values remain."""
    out = {k: v for k, v in raw.items() if math.isfinite(v)}
    return out if out else None

import rclpy
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
from rclpy.subscription import Subscription
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from sensor_msgs.msg import BatteryState, Imu, JointState, LaserScan
from nav_msgs.msg import Odometry

from eimdall_ros2_bridge.edge_client import EdgeClient


class IngestBridge(LifecycleNode):
    """Subscribes to standard ROS 2 sensor topics and forwards readings to Eimdall Edge."""

    def __init__(self) -> None:
        super().__init__("eimdall_ingest_bridge")
        self.declare_parameter("robot_id", "robot-01")
        self.declare_parameter("bridge_id", "ros2-ingest")
        self.declare_parameter("edge_url", "http://127.0.0.1:8787")
        self.declare_parameter("token_file", "/etc/eimdall/eimdall-local-service.token")
        self.declare_parameter("ca_cert", "")
        self.declare_parameter("heartbeat_interval_s", 5.0)
        self.declare_parameter("imu_topic", "/imu/data")
        self.declare_parameter("battery_topic", "/battery_state")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("joint_states_topic", "/joint_states")
        # #644: enforce token file security checks (symlink, permissions, ownership)
        self.declare_parameter("strict_security", True)

        self._client: Optional[EdgeClient] = None
        self._robot_id: str = ""
        self._bridge_id: str = ""
        self._subs: List[Subscription] = []
        self._hb_timer = None
        self._diag_timer = None
        self._diag_pub = None
        self._start_ts: float = 0.0
        self._readings: int = 0
        self._errors: int = 0
        self._error_timestamps: List[float] = []  # for 5-min sliding window

        # #596: producer/consumer queue — ROS callbacks push here (non-blocking),
        # a dedicated sender thread drains it with synchronous HTTP calls.
        self._ingest_queue: queue.Queue[Optional[Tuple[str, str, dict]]] = queue.Queue(
            maxsize=_INGEST_QUEUE_MAX
        )
        self._sender_thread: Optional[threading.Thread] = None
        self._sender_stop = threading.Event()
        # #644: per-sensor sliding-window rate limiter (timestamps within 1-second window)
        self._sensor_rate: Dict[str, collections.deque] = {}

    # ── Lifecycle callbacks ─────────────────────────────────────────────────

    # ── Security helpers ────────────────────────────────────────────────────

    def _validate_token_file(self, token_file: str) -> None:
        """#644: refuse symlinked or world-readable token files."""
        if os.path.islink(token_file):
            raise RuntimeError(f"token_file is a symlink: {token_file}")
        try:
            st = os.stat(token_file)
        except OSError as exc:
            raise RuntimeError(f"cannot stat token_file '{token_file}': {exc}") from exc
        if st.st_mode & 0o077:
            raise RuntimeError(
                f"token_file '{token_file}' permissions too broad: "
                f"{oct(st.st_mode & 0o777)} — expected 0600"
            )
        if st.st_uid != os.getuid():
            raise RuntimeError(
                f"token_file '{token_file}' not owned by current user "
                f"(owner uid={st.st_uid}, current uid={os.getuid()})"
            )

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self._robot_id = self.get_parameter("robot_id").get_parameter_value().string_value
        self._bridge_id = self.get_parameter("bridge_id").get_parameter_value().string_value
        edge_url = self.get_parameter("edge_url").get_parameter_value().string_value
        token_file = self.get_parameter("token_file").get_parameter_value().string_value
        ca_cert = self.get_parameter("ca_cert").get_parameter_value().string_value or None
        strict = self.get_parameter("strict_security").get_parameter_value().bool_value

        # #644: validate token file security before reading it
        if strict:
            try:
                self._validate_token_file(token_file)
            except RuntimeError as exc:
                self.get_logger().error(f"token file security check failed: {exc}")
                return TransitionCallbackReturn.FAILURE

        try:
            self._client = EdgeClient(edge_url=edge_url, token_file=token_file, ca_cert=ca_cert)
        except (FileNotFoundError, ValueError) as exc:
            self.get_logger().error(f"configuration error: {exc}")
            return TransitionCallbackReturn.FAILURE

        self._diag_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.get_logger().info(
            f"configured: robot_id={self._robot_id} edge_url={edge_url}"
        )
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self._start_ts = time.monotonic()

        p = self.get_parameter
        self._subs = [
            self.create_subscription(
                Imu, p("imu_topic").get_parameter_value().string_value, self._on_imu, 10
            ),
            self.create_subscription(
                BatteryState,
                p("battery_topic").get_parameter_value().string_value,
                self._on_battery,
                10,
            ),
            self.create_subscription(
                Odometry, p("odom_topic").get_parameter_value().string_value, self._on_odom, 10
            ),
            self.create_subscription(
                LaserScan, p("scan_topic").get_parameter_value().string_value, self._on_scan, 10
            ),
            self.create_subscription(
                JointState,
                p("joint_states_topic").get_parameter_value().string_value,
                self._on_joint_states,
                10,
            ),
        ]

        hb_interval = (
            self.get_parameter("heartbeat_interval_s").get_parameter_value().double_value
        )
        self._hb_timer = self.create_timer(hb_interval, self._send_heartbeat)
        self._diag_timer = self.create_timer(5.0, self._publish_diagnostics)

        # #596: start dedicated sender thread so ROS callbacks are never blocked by HTTP
        self._sender_stop.clear()
        self._sender_thread = threading.Thread(
            target=self._send_loop, daemon=True, name="eimdall-ingest-sender"
        )
        self._sender_thread.start()

        self.get_logger().info("activated — listening on sensor topics")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        # Signal and join the sender thread before destroying subscriptions
        self._sender_stop.set()
        self._wake_sender()
        if self._sender_thread is not None:
            self._sender_thread.join(timeout=5.0)
            self._sender_thread = None

        for sub in self._subs:
            self.destroy_subscription(sub)
        self._subs = []
        for timer in (self._hb_timer, self._diag_timer):
            if timer is not None:
                timer.cancel()
        self._hb_timer = None
        self._diag_timer = None
        # #644: clear per-sensor rate buckets to free memory
        self._sensor_rate.clear()
        self.get_logger().info("deactivated")
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self._readings = 0
        self._errors = 0
        self.get_logger().info("cleaned up")
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS

    # ── Sensor callbacks ────────────────────────────────────────────────────

    def _on_imu(self, msg: Imu) -> None:
        a = msg.linear_acceleration
        g = msg.angular_velocity
        values = _safe_values({
            "accel_x_g": a.x / 9.81,
            "accel_y_g": a.y / 9.81,
            "accel_z_g": a.z / 9.81,
            "gyro_x_dps": math.degrees(g.x),
            "gyro_y_dps": math.degrees(g.y),
            "gyro_z_dps": math.degrees(g.z),
        })
        if values:
            self._ingest("imu", "imu", values)

    def _on_battery(self, msg: BatteryState) -> None:
        values = _safe_values({
            "voltage_v": msg.voltage,
            "current_a": msg.current,
            "soc_pct": msg.percentage * 100.0,
            "temp_c": msg.temperature,
        })
        if values:
            self._ingest("battery", "battery", values)

    def _on_odom(self, msg: Odometry) -> None:
        v = msg.twist.twist.linear
        values = _safe_values({
            "velocity_x_ms": v.x,
            "velocity_y_ms": v.y,
            "angular_z_rps": msg.twist.twist.angular.z,
        })
        if values:
            self._ingest("odom", "encoder", values)

    def _on_scan(self, msg: LaserScan) -> None:
        # Clamp to _MAX_SCAN_SAMPLES to prevent DoS via oversized range arrays
        samples = msg.ranges[:_MAX_SCAN_SAMPLES]
        valid = [r for r in samples if math.isfinite(r) and msg.range_min < r < msg.range_max]
        if not valid:
            return
        self._ingest("lidar", "lidar", {
            "scan_count": float(len(valid)),
            "min_distance_m": min(valid),
            "mean_distance_m": sum(valid) / len(valid),
        })

    def _on_joint_states(self, msg: JointState) -> None:
        for i, name in enumerate(msg.name):
            safe_name = _sanitize_id(name)
            if safe_name is None:
                continue
            raw: Dict[str, float] = {}
            if i < len(msg.velocity):
                raw["velocity_rps"] = msg.velocity[i]
            if i < len(msg.effort):
                raw["torque_nm"] = msg.effort[i]
            values = _safe_values(raw)
            if values:
                self._ingest(f"joint_{safe_name}", "motor", values)

    # ── Heartbeat & diagnostics ─────────────────────────────────────────────

    def _send_heartbeat(self) -> None:
        if self._client is None:
            return
        uptime = int(time.monotonic() - self._start_ts)
        now = time.monotonic()
        cutoff = now - _ERRORS_WINDOW_S
        self._error_timestamps = [t for t in self._error_timestamps if t > cutoff]
        errors_5m = len(self._error_timestamps)
        ok = self._client.heartbeat(
            robot_id=self._robot_id,
            bridge_id=self._bridge_id,
            uptime_s=uptime,
            errors_5m=errors_5m,
        )
        if not ok:
            self.get_logger().warning("heartbeat to Edge failed")

    def _publish_diagnostics(self) -> None:
        if self._diag_pub is None:
            return
        status = DiagnosticStatus()
        status.name = "eimdall_ingest_bridge"
        status.hardware_id = self._robot_id
        status.level = DiagnosticStatus.OK if self._errors == 0 else DiagnosticStatus.WARN
        status.message = "ingesting" if self._errors == 0 else f"{self._errors} ingest errors"
        status.values = [
            KeyValue(key="readings_ingested", value=str(self._readings)),
            KeyValue(key="ingest_errors", value=str(self._errors)),
            KeyValue(key="robot_id", value=self._robot_id),
            KeyValue(key="bridge_id", value=self._bridge_id),
        ]
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.status = [status]
        self._diag_pub.publish(arr)

    # ── Internal ────────────────────────────────────────────────────────────

    def _ingest(self, sensor_id: str, family: str, values: dict) -> None:
        """Push a reading onto the sender queue (non-blocking).

        #596: never blocks the ROS callback thread. If the queue is full, the
        oldest entry is dropped and a warning is emitted (newest-wins policy).
        #644: payload size and per-sensor rate limit enforced before queuing.
        """
        if self._client is None:
            return

        # #644: reject oversized payloads to prevent Edge bandwidth abuse
        payload_size = sum(len(str(v)) for v in values.values())
        if payload_size > _MAX_PAYLOAD_BYTES:
            self.get_logger().warning(
                f"ingest payload too large ({payload_size} B > {_MAX_PAYLOAD_BYTES} B) "
                f"from sensor {sensor_id} — dropping"
            )
            return

        # #644: per-sensor rate limit — prevents a misconfigured topic from flooding Edge
        now_ts = time.monotonic()
        if sensor_id not in self._sensor_rate:
            self._sensor_rate[sensor_id] = collections.deque()
        dq = self._sensor_rate[sensor_id]
        while dq and now_ts - dq[0] > 1.0:
            dq.popleft()
        if len(dq) >= _SENSOR_RATE_LIMIT:
            self.get_logger().warning(
                f"rate limit exceeded for sensor {sensor_id} ({_SENSOR_RATE_LIMIT} msg/s) — dropping"
            )
            return
        dq.append(now_ts)

        try:
            self._ingest_queue.put_nowait((sensor_id, family, values))
        except queue.Full:
            # Queue saturated — drop oldest entry and retry once
            try:
                self._ingest_queue.get_nowait()
            except queue.Empty:
                pass
            try:
                self._ingest_queue.put_nowait((sensor_id, family, values))
            except queue.Full:
                self.get_logger().warning(
                    f"ingest queue full — dropping reading from {sensor_id}"
                )

    def _wake_sender(self) -> None:
        try:
            self._ingest_queue.put_nowait(None)
        except queue.Full:
            try:
                self._ingest_queue.get_nowait()
            except queue.Empty:
                pass
            try:
                self._ingest_queue.put_nowait(None)
            except queue.Full:
                pass

    def _send_loop(self) -> None:
        """Sender thread: drains the ingest queue with synchronous HTTP calls."""
        while not self._sender_stop.is_set():
            try:
                item = self._ingest_queue.get(timeout=1.0)
            except queue.Empty:
                continue
            if item is None:  # sentinel — deactivating
                break
            sensor_id, family, values = item
            ok = self._client.ingest(  # type: ignore[union-attr]
                robot_id=self._robot_id,
                bridge_id=self._bridge_id,
                sensor_id=sensor_id,
                family=family,
                values=values,
            )
            if ok:
                self._readings += 1
            else:
                self._errors += 1
                self._error_timestamps.append(time.monotonic())


def main(args=None) -> None:
    rclpy.init(args=args)
    node = IngestBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
