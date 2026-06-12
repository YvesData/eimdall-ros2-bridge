"""IngestBridge — ROS 2 sensor topics → Eimdall Edge ingest."""
import math
import time
from typing import List, Optional

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

    # ── Lifecycle callbacks ─────────────────────────────────────────────────

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self._robot_id = self.get_parameter("robot_id").get_parameter_value().string_value
        self._bridge_id = self.get_parameter("bridge_id").get_parameter_value().string_value
        edge_url = self.get_parameter("edge_url").get_parameter_value().string_value
        token_file = self.get_parameter("token_file").get_parameter_value().string_value
        ca_cert = self.get_parameter("ca_cert").get_parameter_value().string_value or None

        try:
            self._client = EdgeClient(edge_url=edge_url, token_file=token_file, ca_cert=ca_cert)
        except FileNotFoundError as exc:
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
        self.get_logger().info("activated — listening on sensor topics")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        for sub in self._subs:
            self.destroy_subscription(sub)
        self._subs = []
        for timer in (self._hb_timer, self._diag_timer):
            if timer is not None:
                timer.cancel()
        self._hb_timer = None
        self._diag_timer = None
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
        self._ingest("imu", "imu", {
            "accel_x_g": a.x / 9.81,
            "accel_y_g": a.y / 9.81,
            "accel_z_g": a.z / 9.81,
            "gyro_x_dps": math.degrees(g.x),
            "gyro_y_dps": math.degrees(g.y),
            "gyro_z_dps": math.degrees(g.z),
        })

    def _on_battery(self, msg: BatteryState) -> None:
        self._ingest("battery", "battery", {
            "voltage_v": msg.voltage,
            "current_a": msg.current,
            "soc_pct": msg.percentage * 100.0,
            "temp_c": msg.temperature,
        })

    def _on_odom(self, msg: Odometry) -> None:
        v = msg.twist.twist.linear
        self._ingest("odom", "encoder", {
            "velocity_x_ms": v.x,
            "velocity_y_ms": v.y,
            "angular_z_rps": msg.twist.twist.angular.z,
        })

    def _on_scan(self, msg: LaserScan) -> None:
        valid = [r for r in msg.ranges if msg.range_min < r < msg.range_max]
        if not valid:
            return
        self._ingest("lidar", "lidar", {
            "scan_count": float(len(valid)),
            "min_distance_m": min(valid),
            "mean_distance_m": sum(valid) / len(valid),
        })

    def _on_joint_states(self, msg: JointState) -> None:
        for i, name in enumerate(msg.name):
            values: dict = {}
            if i < len(msg.velocity) and not math.isnan(msg.velocity[i]):
                values["velocity_rps"] = msg.velocity[i]
            if i < len(msg.effort) and not math.isnan(msg.effort[i]):
                values["torque_nm"] = msg.effort[i]
            if values:
                self._ingest(f"joint_{name}", "motor", values)

    # ── Heartbeat & diagnostics ─────────────────────────────────────────────

    def _send_heartbeat(self) -> None:
        if self._client is None:
            return
        uptime = int(time.monotonic() - self._start_ts)
        errors_snapshot = self._errors
        ok = self._client.heartbeat(
            robot_id=self._robot_id,
            bridge_id=self._bridge_id,
            uptime_s=uptime,
            errors_5m=errors_snapshot,
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
        if self._client is None:
            return
        ok = self._client.ingest(
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
