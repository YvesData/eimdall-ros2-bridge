"""Patch ROS2 and message dependencies before any bridge module is imported.

This file is loaded automatically by pytest before test collection. It replaces
rclpy, rclpy.lifecycle, rclpy.qos, diagnostic_msgs.msg, and
eimdall_ros2_bridge.msg with lightweight in-process stubs so the bridge modules
can be unit-tested without a live ROS2 environment.
"""
from __future__ import annotations

import sys
import types
from unittest.mock import MagicMock


# ── TransitionCallbackReturn ──────────────────────────────────────────────────

class _TransitionCallbackReturn:
    SUCCESS = "SUCCESS"
    FAILURE = "FAILURE"


# ── FakeLifecycleNode ─────────────────────────────────────────────────────────

class FakeLifecycleNode:
    """Minimal LifecycleNode substitute that records publishers and timers."""

    def __init__(self, name: str) -> None:
        self._node_name = name
        self._params: dict = {}
        self._created_publishers: list = []
        self._logger = MagicMock()
        self._clock = MagicMock()
        self._clock.now.return_value.to_msg.return_value = MagicMock()

    # Parameters
    def declare_parameter(self, name: str, default) -> None:
        self._params[name] = default

    def get_parameter(self, name: str):
        default = self._params.get(name, "")
        pv = MagicMock()
        if isinstance(default, str):
            pv.string_value = default
            pv.double_value = 0.0
            pv.integer_value = 0
        elif isinstance(default, float):
            pv.string_value = ""
            pv.double_value = default
            pv.integer_value = int(default)
        elif isinstance(default, int):
            pv.string_value = ""
            pv.double_value = float(default)
            pv.integer_value = default
        else:
            pv.string_value = str(default)
            pv.double_value = 0.0
            pv.integer_value = 0
        wrapper = MagicMock()
        wrapper.get_parameter_value.return_value = pv
        return wrapper

    # Publishers & timers
    def create_lifecycle_publisher(self, msg_type, topic, qos):
        pub = MagicMock()
        self._created_publishers.append(pub)
        return pub

    def create_publisher(self, msg_type, topic, qos_or_depth):
        pub = MagicMock()
        self._created_publishers.append(pub)
        return pub

    def destroy_publisher(self, pub) -> None:
        pass

    def create_timer(self, period, callback):
        timer = MagicMock()
        timer.cancel = MagicMock()
        return timer

    # Accessors
    def get_clock(self):
        return self._clock

    def get_logger(self):
        return self._logger


# ── DiagnosticStatus (needs real class attrs) ─────────────────────────────────

class _DiagnosticStatus:
    OK = 0
    WARN = 1
    ERROR = 2

    def __init__(self):
        self.name = ""
        self.hardware_id = ""
        self.level = _DiagnosticStatus.OK
        self.message = ""
        self.values = []


# ── Install all mocks into sys.modules ────────────────────────────────────────

def _install():
    # rclpy root
    rclpy_mod = types.ModuleType("rclpy")
    rclpy_mod.init = MagicMock()
    rclpy_mod.spin = MagicMock()
    rclpy_mod.shutdown = MagicMock()
    sys.modules.setdefault("rclpy", rclpy_mod)

    # rclpy.lifecycle
    lc_mod = types.ModuleType("rclpy.lifecycle")
    lc_mod.LifecycleNode = FakeLifecycleNode
    lc_mod.State = MagicMock
    lc_mod.TransitionCallbackReturn = _TransitionCallbackReturn
    sys.modules.setdefault("rclpy.lifecycle", lc_mod)

    # rclpy.qos — each policy is a plain namespace with arbitrary attrs
    qos_mod = types.ModuleType("rclpy.qos")
    for _name in ("QoSProfile", "ReliabilityPolicy", "DurabilityPolicy", "HistoryPolicy"):
        setattr(qos_mod, _name, MagicMock())
    sys.modules.setdefault("rclpy.qos", qos_mod)

    # diagnostic_msgs.msg
    diag_mod = types.ModuleType("diagnostic_msgs")
    diag_msg_mod = types.ModuleType("diagnostic_msgs.msg")
    diag_msg_mod.DiagnosticStatus = _DiagnosticStatus
    diag_msg_mod.DiagnosticArray = MagicMock
    diag_msg_mod.KeyValue = MagicMock
    sys.modules.setdefault("diagnostic_msgs", diag_mod)
    sys.modules.setdefault("diagnostic_msgs.msg", diag_msg_mod)

    # eimdall_ros2_bridge.msg (ROS2-generated message classes)
    bridge_msg_mod = types.ModuleType("eimdall_ros2_bridge.msg")
    bridge_msg_mod.EimdallAnomaly = MagicMock
    bridge_msg_mod.EimdallHealth = MagicMock
    bridge_msg_mod.EimdallSensorStatus = MagicMock
    sys.modules.setdefault("eimdall_ros2_bridge.msg", bridge_msg_mod)


_install()
