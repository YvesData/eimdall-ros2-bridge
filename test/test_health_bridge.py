"""Unit tests for HealthBridge (no live ROS2 needed).

All rclpy / message-type imports are replaced by stubs in conftest.py.
"""
from __future__ import annotations

import json
from pathlib import Path
from unittest.mock import MagicMock

import pytest

# conftest.py has already installed sys.modules stubs before this import.
from eimdall_ros2_bridge.health_bridge import (  # noqa: E402
    HealthBridge,
    _family_from_label,
    _status_from_edge_sensor,
    _confidence_from_status,
)


# ── Helpers ───────────────────────────────────────────────────────────────────

def _configured_bridge(tmp_path: Path, health_file: str = "health.json") -> HealthBridge:
    bridge = HealthBridge()
    bridge._params["health_path"] = str(tmp_path / health_file)
    bridge._params["publish_period_sec"] = 0.5
    bridge._params["max_file_bytes"] = 1_048_576

    result = bridge.on_configure(MagicMock())
    assert result == "SUCCESS", f"on_configure failed: {result}"
    return bridge


def _write_health(path: Path, payload: dict) -> None:
    path.write_text(json.dumps(payload), encoding="utf-8")


_MINIMAL_HEALTH = {
    "uptime_s": 10.0,
    "configured_sensors": 2,
    "active_sensors": 2,
    "total_reconnects": 0,
    "total_consecutive_errors": 0,
    "total_lines_seen": 100,
    "total_parse_errors": 0,
    "total_processed_values": 50,
    "total_anomaly_events": 0,
    "sensors": [],
}


# ── Lifecycle ─────────────────────────────────────────────────────────────────

def test_configure_sets_health_path(tmp_path: Path) -> None:
    bridge = _configured_bridge(tmp_path)
    assert bridge._health_path is not None
    assert "health.json" in str(bridge._health_path)


def test_configure_creates_publishers(tmp_path: Path) -> None:
    bridge = _configured_bridge(tmp_path)
    assert bridge._health_pub is not None
    assert bridge._sensor_pub is not None
    assert bridge._diag_pub is not None


def test_cleanup_destroys_publishers(tmp_path: Path) -> None:
    bridge = _configured_bridge(tmp_path)
    bridge.on_cleanup(MagicMock())
    assert bridge._health_pub is None
    assert bridge._sensor_pub is None
    assert bridge._diag_pub is None


def test_cleanup_resets_counters(tmp_path: Path) -> None:
    bridge = _configured_bridge(tmp_path)
    bridge._ticks = 10
    bridge._publishes = 5
    bridge._parse_errors = 2
    bridge._oversize_ticks = 1
    bridge.on_cleanup(MagicMock())
    assert bridge._ticks == 0
    assert bridge._publishes == 0
    assert bridge._parse_errors == 0
    assert bridge._oversize_ticks == 0


def test_deactivate_cancels_timer(tmp_path: Path) -> None:
    bridge = _configured_bridge(tmp_path)
    bridge.on_activate(MagicMock())
    timer = bridge._timer
    bridge.on_deactivate(MagicMock())
    assert bridge._timer is None
    timer.cancel.assert_called_once()


# ── _tick: missing / empty file ───────────────────────────────────────────────

def test_tick_missing_file_no_publish(tmp_path: Path) -> None:
    bridge = _configured_bridge(tmp_path)
    bridge._tick()
    bridge._health_pub.publish.assert_not_called()


def test_tick_empty_file_skipped(tmp_path: Path) -> None:
    bridge = _configured_bridge(tmp_path)
    bridge._health_path.write_text("", encoding="utf-8")
    bridge._tick()
    bridge._health_pub.publish.assert_not_called()


# ── _tick: valid health snapshot ──────────────────────────────────────────────

def test_tick_publishes_health_snapshot(tmp_path: Path) -> None:
    bridge = _configured_bridge(tmp_path)
    _write_health(bridge._health_path, _MINIMAL_HEALTH)
    bridge._tick()
    bridge._health_pub.publish.assert_called_once()
    assert bridge._publishes == 1
    assert bridge._parse_errors == 0


def test_tick_publishes_sensor_status_per_sensor(tmp_path: Path) -> None:
    bridge = _configured_bridge(tmp_path)
    payload = dict(_MINIMAL_HEALTH)
    payload["sensors"] = [
        {"sensor_id": "lidar:0", "status": "ok", "processed_values": 10},
        {"sensor_id": "imu:0",   "status": "warning", "consecutive_errors": 1},
    ]
    _write_health(bridge._health_path, payload)
    bridge._tick()
    assert bridge._sensor_pub.publish.call_count == 2


# ── _tick: deduplication (same content) ──────────────────────────────────────

def test_tick_does_not_republish_same_content(tmp_path: Path) -> None:
    bridge = _configured_bridge(tmp_path)
    _write_health(bridge._health_path, _MINIMAL_HEALTH)
    bridge._tick()
    bridge._tick()  # same content — must not re-publish
    assert bridge._publishes == 1


def test_tick_republishes_after_content_change(tmp_path: Path) -> None:
    bridge = _configured_bridge(tmp_path)
    _write_health(bridge._health_path, _MINIMAL_HEALTH)
    bridge._tick()
    updated = dict(_MINIMAL_HEALTH, uptime_s=20.0)
    _write_health(bridge._health_path, updated)
    bridge._tick()
    assert bridge._publishes == 2


# ── _tick: parse error ────────────────────────────────────────────────────────

def test_tick_invalid_json_increments_parse_errors(tmp_path: Path) -> None:
    bridge = _configured_bridge(tmp_path)
    bridge._health_path.write_text("{not valid json", encoding="utf-8")
    bridge._tick()
    assert bridge._parse_errors == 1
    bridge._health_pub.publish.assert_not_called()


# ── _tick: oversize file ──────────────────────────────────────────────────────

def test_tick_oversize_file_no_publish(tmp_path: Path) -> None:
    bridge = _configured_bridge(tmp_path)
    bridge._max_file_bytes = 10  # 10-byte limit
    _write_health(bridge._health_path, _MINIMAL_HEALTH)  # >> 10 bytes
    bridge._tick()
    bridge._health_pub.publish.assert_not_called()
    assert bridge._oversize_ticks == 1


def test_tick_oversize_file_rotated_after_3_ticks(tmp_path: Path) -> None:
    bridge = _configured_bridge(tmp_path)
    bridge._max_file_bytes = 10
    _write_health(bridge._health_path, _MINIMAL_HEALTH)
    for _ in range(3):
        bridge._tick()
    # After 3 consecutive oversize ticks the file is renamed to *.oversize.bak.
    bak = bridge._health_path.with_suffix(".oversize.bak")
    assert bak.exists(), "expected health file to be rotated to .oversize.bak"
    # Counter resets after rotation.
    assert bridge._oversize_ticks == 0


# ── Pure helpers ──────────────────────────────────────────────────────────────

@pytest.mark.parametrize("label,expected", [
    ("lidar:front",  "lidar"),
    ("imu-0",        "imu"),
    ("camera",       "camera"),
    ("",             ""),
])
def test_family_from_label(label: str, expected: str) -> None:
    assert _family_from_label(label) == expected


@pytest.mark.parametrize("sensor,expected_status", [
    ({"consecutive_errors": 1},      "warning"),
    ({"parse_errors": 2},             "warning"),
    ({"processed_values": 0},         "offline"),
    ({"processed_values": 10},        "ok"),
])
def test_status_from_edge_sensor(sensor: dict, expected_status: str) -> None:
    assert _status_from_edge_sensor(sensor) == expected_status


@pytest.mark.parametrize("status,expected", [
    ("ok",      100.0),
    ("warning",  70.0),
    ("error",    40.0),
    ("offline",   0.0),
    ("unknown",  50.0),
])
def test_confidence_from_status(status: str, expected: float) -> None:
    assert _confidence_from_status(status) == expected
