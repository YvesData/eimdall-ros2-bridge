"""Unit tests for AnomalyBridge (no live ROS2 needed).

All rclpy / message-type imports are replaced by stubs in conftest.py.
"""
from __future__ import annotations

import json
import os
from pathlib import Path
from unittest.mock import MagicMock, call

import pytest

# conftest.py has already installed sys.modules stubs before this import.
from eimdall_ros2_bridge.anomaly_bridge import AnomalyBridge, _severity_to_int  # noqa: E402


# ── Helpers ───────────────────────────────────────────────────────────────────

def _configured_bridge(tmp_path: Path, anomaly_file: str = "anomalies.jsonl") -> AnomalyBridge:
    """Return an AnomalyBridge that has been successfully on_configure()d."""
    bridge = AnomalyBridge()
    # Override declared defaults so on_configure reads from tmp_path.
    bridge._params["anomaly_path"] = str(tmp_path / anomaly_file)
    bridge._params["poll_period_sec"] = 0.1
    bridge._params["max_anomalies_per_tick"] = 200

    result = bridge.on_configure(MagicMock())
    assert result == "SUCCESS", f"on_configure failed: {result}"
    return bridge


def _write_lines(path: Path, lines: list[str]) -> None:
    with open(path, "w") as fh:
        for line in lines:
            fh.write(line + "\n")


# ── Lifecycle ─────────────────────────────────────────────────────────────────

def test_configure_sets_anomaly_path(tmp_path: Path) -> None:
    bridge = _configured_bridge(tmp_path)
    assert bridge._anomaly_path is not None
    assert "anomalies.jsonl" in str(bridge._anomaly_path)


def test_configure_creates_publishers(tmp_path: Path) -> None:
    bridge = _configured_bridge(tmp_path)
    assert bridge._pub is not None
    assert bridge._diag_pub is not None


def test_cleanup_destroys_publishers(tmp_path: Path) -> None:
    bridge = _configured_bridge(tmp_path)
    bridge.on_cleanup(MagicMock())
    assert bridge._pub is None
    assert bridge._diag_pub is None


def test_cleanup_resets_counters(tmp_path: Path) -> None:
    bridge = _configured_bridge(tmp_path)
    bridge._events_published = 5
    bridge._errors = 3
    bridge._offset = 1024
    bridge.on_cleanup(MagicMock())
    assert bridge._events_published == 0
    assert bridge._errors == 0
    assert bridge._offset == 0


def test_deactivate_cancels_timer(tmp_path: Path) -> None:
    bridge = _configured_bridge(tmp_path)
    bridge.on_activate(MagicMock())
    timer = bridge._timer
    bridge.on_deactivate(MagicMock())
    assert bridge._timer is None
    timer.cancel.assert_called_once()


# ── _tick: missing file ───────────────────────────────────────────────────────

def test_tick_missing_file_no_publish(tmp_path: Path) -> None:
    bridge = _configured_bridge(tmp_path)
    # File does not exist — _pub.publish must NOT be called.
    bridge._tick()
    bridge._pub.publish.assert_not_called()


# ── _tick: valid anomalies ────────────────────────────────────────────────────

def test_tick_publishes_one_anomaly(tmp_path: Path) -> None:
    bridge = _configured_bridge(tmp_path)
    path = bridge._anomaly_path
    _write_lines(path, [
        json.dumps({"event_id": "e1", "robot_id": "r1", "severity": "high", "score": 0.9}),
    ])
    bridge._tick()
    bridge._pub.publish.assert_called_once()
    assert bridge._events_published == 1
    assert bridge._errors == 0


def test_tick_publishes_multiple_anomalies(tmp_path: Path) -> None:
    bridge = _configured_bridge(tmp_path)
    path = bridge._anomaly_path
    _write_lines(path, [
        json.dumps({"event_id": f"e{i}", "score": float(i)}) for i in range(5)
    ])
    bridge._tick()
    assert bridge._pub.publish.call_count == 5
    assert bridge._events_published == 5


def test_tick_advances_offset(tmp_path: Path) -> None:
    bridge = _configured_bridge(tmp_path)
    path = bridge._anomaly_path
    line = json.dumps({"event_id": "e1"})
    _write_lines(path, [line])
    bridge._tick()
    assert bridge._offset > 0
    prev_offset = bridge._offset

    # Calling tick again with no new content must not re-publish.
    prev_count = bridge._pub.publish.call_count
    bridge._tick()
    assert bridge._pub.publish.call_count == prev_count


# ── _tick: corrupted lines ────────────────────────────────────────────────────

def test_tick_corrupted_line_increments_errors(tmp_path: Path) -> None:
    bridge = _configured_bridge(tmp_path)
    path = bridge._anomaly_path
    _write_lines(path, [
        "not valid json {{",
        json.dumps({"event_id": "e2", "score": 0.5}),
    ])
    bridge._tick()
    assert bridge._errors == 1
    assert bridge._events_published == 1
    # Offset must have advanced past both lines.
    assert bridge._offset > 0


def test_tick_corrupted_line_still_advances_tick_count(tmp_path: Path) -> None:
    """tick_count is incremented before json.loads — a corrupt line still counts."""
    bridge = _configured_bridge(tmp_path)
    bridge._params["max_anomalies_per_tick"] = 1  # tight limit
    # Re-configure to pick up new max.
    bridge.on_cleanup(MagicMock())
    bridge.on_configure(MagicMock())
    path = bridge._anomaly_path
    # Write 2 lines; with limit=1 only the first is processed.
    _write_lines(path, [
        "bad json",
        json.dumps({"event_id": "e2"}),
    ])
    bridge._tick()
    # First line is corrupted → errors = 1, but tick_count hit the limit.
    assert bridge._errors == 1
    # Second line was NOT processed yet (backlog limit).
    assert bridge._events_published == 0


# ── _tick: file rotation ──────────────────────────────────────────────────────

def test_tick_detects_file_rotation(tmp_path: Path) -> None:
    bridge = _configured_bridge(tmp_path)
    path = bridge._anomaly_path

    _write_lines(path, [json.dumps({"event_id": "e1"})])
    bridge._tick()
    old_offset = bridge._offset
    assert old_offset > 0

    # Simulate rotation: replace the file (new inode, smaller size).
    path.unlink()
    _write_lines(path, [json.dumps({"event_id": "e2"})])
    bridge._last_inode = bridge._last_inode  # preserve stale inode
    # Force inode mismatch by reporting a fake old inode.
    bridge._last_inode = 999_999_999

    bridge._tick()
    # Offset must have been reset to 0 before reading the new file.
    # After tick, offset > 0 again (we read the new line).
    assert bridge._events_published == 2


# ── _severity_to_int ──────────────────────────────────────────────────────────

@pytest.mark.parametrize("value,expected", [
    ("info",     1),
    ("low",      1),
    ("warning",  2),
    ("warn",     2),
    ("medium",   2),
    ("high",     3),
    ("critical", 4),
    ("error",    4),
    ("HIGH",     3),   # case-insensitive
    (3,          3),   # numeric pass-through
    ("99",       99),  # numeric string
    ("unknown",  0),   # unmapped → 0
    (None,       0),   # None → 0
])
def test_severity_to_int(value, expected: int) -> None:
    assert _severity_to_int(value) == expected
