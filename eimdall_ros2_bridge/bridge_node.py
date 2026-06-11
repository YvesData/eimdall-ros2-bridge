"""Eimdall ROS 2 bridge node.

Subscribes to standard ROS 2 topics and forwards sensor data to the
Eimdall Edge local service running on the robot.

Supported topic → family mappings (auto-detected):
  /imu/data            → sensor_msgs/Imu       → imu
  /battery_state       → sensor_msgs/Battery   → battery
  /odom                → nav_msgs/Odometry     → encoder
  /scan                → sensor_msgs/LaserScan → lidar
  /joint_states        → sensor_msgs/JointState→ motor
  /cmd_vel_stamped     → custom proximity      → proximity
"""
from __future__ import annotations

import math
import os
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from sensor_msgs.msg import BatteryState, Imu, JointState, LaserScan
from nav_msgs.msg import Odometry

from .edge_client import EdgeClient


class EimdallBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("eimdall_bridge")

        self.declare_parameter("robot_id",   os.environ.get("EIMDALL_ROBOT_ID",   "robot-01"))
        self.declare_parameter("bridge_id",  os.environ.get("EIMDALL_BRIDGE_ID",  "ros2-bridge"))
        self.declare_parameter("edge_url",   os.environ.get("EIMDALL_EDGE_URL",   "https://127.0.0.1:8787"))
        self.declare_parameter("token_file", os.environ.get("EIMDALL_TOKEN_FILE", "/etc/eimdall/eimdall-local-service.token"))
        self.declare_parameter("ca_cert",    os.environ.get("EIMDALL_CA_CERT",    "/etc/eimdall/tls/edge-local-ca.crt"))
        self.declare_parameter("heartbeat_interval_s", 5.0)

        self._robot_id  = self.get_parameter("robot_id").value
        self._bridge_id = self.get_parameter("bridge_id").value
        self._start_ts  = time.monotonic()
        self._errors    = 0

        self._client = EdgeClient(
            edge_url   = self.get_parameter("edge_url").value,
            token_file = self.get_parameter("token_file").value,
            ca_cert    = self.get_parameter("ca_cert").value,
        )

        if not self._client.ping():
            self.get_logger().warning("Edge service unreachable — will retry on ingest")
        else:
            self.get_logger().info("Edge service reachable at %s", self.get_parameter("edge_url").value)

        self.create_subscription(Imu,          "/imu/data",      self._on_imu,          10)
        self.create_subscription(BatteryState, "/battery_state", self._on_battery,      10)
        self.create_subscription(Odometry,     "/odom",          self._on_odometry,     10)
        self.create_subscription(LaserScan,    "/scan",          self._on_scan,         10)
        self.create_subscription(JointState,   "/joint_states",  self._on_joint_states, 10)

        hb = self.get_parameter("heartbeat_interval_s").value
        self.create_timer(float(hb), self._send_heartbeat)

        self.get_logger().info(
            "Eimdall bridge started — robot_id=%s bridge_id=%s",
            self._robot_id, self._bridge_id,
        )

    # ── Sensor callbacks ────────────────────────────────────────────────────

    def _on_imu(self, msg: Imu) -> None:
        a = msg.linear_acceleration
        g = msg.angular_velocity
        self._ingest("imu", "imu", {
            "accel_x_g":   a.x / 9.81,
            "accel_y_g":   a.y / 9.81,
            "accel_z_g":   a.z / 9.81,
            "gyro_x_dps":  math.degrees(g.x),
            "gyro_y_dps":  math.degrees(g.y),
            "gyro_z_dps":  math.degrees(g.z),
        })

    def _on_battery(self, msg: BatteryState) -> None:
        self._ingest("battery", "battery", {
            "voltage_v":   msg.voltage,
            "current_a":   msg.current,
            "soc_pct":     msg.percentage * 100.0,
            "temp_c":      msg.temperature,
        })

    def _on_odometry(self, msg: Odometry) -> None:
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
            "scan_count":     float(len(valid)),
            "min_distance_m": min(valid),
            "mean_distance_m": sum(valid) / len(valid),
        })

    def _on_joint_states(self, msg: JointState) -> None:
        for i, name in enumerate(msg.name):
            values: dict = {}
            if i < len(msg.velocity):
                values["velocity_rps"] = msg.velocity[i]
            if i < len(msg.effort):
                values["torque_nm"] = msg.effort[i]
            if values:
                self._ingest(f"joint_{name}", "motor", values)

    # ── Heartbeat ───────────────────────────────────────────────────────────

    def _send_heartbeat(self) -> None:
        uptime = int(time.monotonic() - self._start_ts)
        ok = self._client.heartbeat(
            robot_id=self._robot_id,
            bridge_id=self._bridge_id,
            uptime_s=uptime,
            errors_5m=self._errors,
        )
        self._errors = 0
        if not ok:
            self.get_logger().warning("Heartbeat failed")

    # ── Internal ────────────────────────────────────────────────────────────

    def _ingest(self, sensor_id: str, family: str, values: dict) -> None:
        ok = self._client.ingest(
            robot_id=self._robot_id,
            bridge_id=self._bridge_id,
            sensor_id=sensor_id,
            family=family,
            values=values,
        )
        if not ok:
            self._errors += 1

    def destroy_node(self) -> None:
        self._send_heartbeat()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = EimdallBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
