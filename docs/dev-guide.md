# Developer guide — eimdall_ros2_bridge

> Bridges a ROS 2 graph to the Eimdall Edge runtime, in both directions, via three managed
> lifecycle nodes. This guide targets experienced developers who are new to ROS 2 and to this
> codebase.

This package is a **public** repository (`github.com/YvesData/eimdall-ros2-bridge`), consumed as
a git submodule at `ros2/eimdall_ros2_bridge` in the main `eimdall` monorepo. It is
source-available, not open-source-licensed (see `LICENSE`: "© Eimdall / Data Agility. All rights
reserved.").

## 1. What this package does

Eimdall Edge (a separate Rust process, see the main repo's
[`docs/dev-guide/edge.md`](../../../../docs/dev-guide/edge.md)) writes local snapshot/log files
and exposes a local-only HTTP API. This package's job is to bridge that to a ROS 2 graph:

- **`health_bridge`** — polls a JSON snapshot file (`runtime_health.json`) written by Edge and
  republishes it as `/eimdall/health` (`EimdallHealth`) and `/eimdall/sensors/status`
  (`EimdallSensorStatus`, one message per sensor). Only republishes on content change.
- **`anomaly_bridge`** — tails an append-only JSONL file (`runtime_anomalies.jsonl`) by byte
  offset, republishing each new line as `/eimdall/anomalies` (`EimdallAnomaly`). Detects file
  rotation (inode change or size shrink) and resets its offset.
- **`ingest_bridge`** — the only node going the *other* direction: subscribes to standard ROS 2
  sensor topics (`sensor_msgs/Imu`, `sensor_msgs/BatteryState`, `nav_msgs/Odometry`,
  `sensor_msgs/LaserScan`, `sensor_msgs/JointState`) and forwards derived readings to Edge's local
  HTTP API at `/v1/local/bridge/ingest`, plus periodic heartbeats to
  `/v1/local/bridge/heartbeat`.

All three also publish `diagnostic_msgs/DiagnosticArray` on `/diagnostics` (standard ROS 2
convention, consumable by `rqt_robot_monitor`).

### Lifecycle nodes — read this before anything else

All three nodes subclass `rclpy.lifecycle.LifecycleNode`, **not** plain `Node`. If you've never
used ROS 2 managed lifecycle: a node goes through explicit states
(`unconfigured → inactive → active`) and only starts doing real work (creating timers,
subscriptions) in `on_activate`, not in `__init__`/`on_configure`. If you `ros2 run
eimdall_ros2_bridge health_bridge` directly, it will appear to hang doing nothing — it needs an
external `configure` then `activate` transition. The launch file (`launch/eimdall_bridge.launch.py`)
handles this automatically via a `_auto_start()` helper: it fires `TRANSITION_CONFIGURE` after a
1s delay, then registers an `OnStateTransition` handler that fires `TRANSITION_ACTIVATE` as soon
as the node reaches `inactive`. Manual lifecycle control:
```bash
ros2 lifecycle set /eimdall_health_bridge configure
ros2 lifecycle set /eimdall_health_bridge activate
```

### `edge_client.py` — the HTTP client shared by `ingest_bridge`

Minimal `urllib`-based client, not tied to `rclpy`. Two hardening points worth knowing:
- `_validate_edge_url()` **rejects any `edge_url` whose host isn't `localhost`/`127.0.0.1`/`::1`**
  — an explicit anti-SSRF/anti-token-exfiltration measure (Edge only ever runs locally). Passing a
  remote `edge_url` makes `on_configure` fail with `TransitionCallbackReturn.FAILURE`, not a
  silent remote connection attempt.
- Reads a bearer token from `token_file`, sent as header `X-Eimdall-Bridge-Token`. Optional CA
  cert for HTTPS.

`ingest_bridge.py` uses a producer/consumer pattern: ROS callbacks (`_on_imu`, `_on_battery`,
`_on_odom`, `_on_scan`, `_on_joint_states`) never call HTTP directly — they push onto a bounded
`queue.Queue(maxsize=500)`, and a dedicated daemon thread (`_send_loop`) drains it with blocking
HTTP calls, so a slow/unreachable Edge never blocks the ROS executor. It also enforces: a 64 KiB
payload-size cap, a 200 msg/s per-topic rate limit, sanitized joint names, NaN/Inf stripping, and
strict token-file permission checks (`_validate_token_file`, requires mode `0600` and
current-user ownership) — gated by the `strict_security` parameter (default `True`; set to
`false` if your token is mounted with different ownership in a container, but understand why
that check exists before disabling it).

## 2. Package layout

```text
eimdall_ros2_bridge/
├── package.xml, CMakeLists.txt      ament_cmake + ament_cmake_python build
├── setup.py, setup.cfg               console_scripts metadata (see gotcha #1 below)
├── eimdall_ros2_bridge/              Python package: the 3 nodes + edge_client.py
├── msg/                               3 custom message definitions
├── launch/eimdall_bridge.launch.py    Auto-configure + auto-activate all 3 nodes
├── config/bridge.yaml                 Reference params (see gotcha #3 — not actually wired up)
├── scripts/                           Thin wrappers installed as the ros2 run executables
├── hardware/esp32_mpu6050/sketch.ino  Arduino/ESP32 firmware, unrelated to the ROS build
├── monitoring/                        Grafana dashboard + OTel collector config (see gotcha #4)
└── test/                              pytest suite (rclpy stubbed out, see §4)
```

## 3. Custom messages (`msg/`)

**`EimdallAnomaly.msg`**
```
builtin_interfaces/Time stamp
string event_id
string robot_id
string tenant_id
string component
int32 severity        # 1=low, 2=medium, 3=high, 4=critical
float64 score          # 0.0–1.0
string[] reason_codes
```

**`EimdallHealth.msg`**
```
builtin_interfaces/Time stamp
float64 uptime_s
int32 configured_sensors
int32 active_sensors
int32 total_reconnects
int32 total_consecutive_errors
int64 total_lines_seen
int64 total_parse_errors
int64 total_processed_values
int64 total_anomaly_events
```
⚠️ The README's field table for `EimdallHealth` is currently stale — it lists only 7 fields and
omits `total_consecutive_errors`, `total_parse_errors`, `total_processed_values`, all three of
which exist in the `.msg` file and are populated by `health_bridge.py::_publish_global_health`.
Trust the `.msg` file over the README prose.

**`EimdallSensorStatus.msg`**
```
builtin_interfaces/Time stamp
string sensor_id
string family            # e.g. imu, encoder
string status              # ok | warning | error | offline
float32 confidence_pct
int64 recent_readings
int64 last_reading_at_ms
```

Messages are code-generated at build time (`rosidl_generate_interfaces()` in `CMakeLists.txt`)
into `eimdall_ros2_bridge.msg.EimdallHealth` etc. — they only exist after `colcon build`, which is
why `test/conftest.py` stubs the whole module out for plain-`pytest` runs (§4).

## 4. Build, run, test

**Build:**
```bash
cd ~/ros2_ws
colcon build --packages-select eimdall_ros2_bridge
source install/setup.bash
```

**Run:**
```bash
ros2 launch eimdall_ros2_bridge eimdall_bridge.launch.py \
  health_path:=/var/lib/eimdall/runtime_health.json \
  anomaly_path:=/var/lib/eimdall/runtime_anomalies.jsonl \
  robot_id:=my-robot-01 edge_url:=http://127.0.0.1:8787
ros2 topic echo /eimdall/health
```

**Tests — two separate paths, not the same suite:**

1. Pure Python, no ROS 2 install needed:
   ```bash
   python3 -m pytest -q
   ```
   Works because `test/conftest.py` injects fake `sys.modules` entries for `rclpy`,
   `rclpy.lifecycle`, `rclpy.qos`, `diagnostic_msgs.msg`, and `eimdall_ros2_bridge.msg` *before*
   the bridge modules are imported (a `FakeLifecycleNode` fakes `declare_parameter`,
   `get_parameter`, `create_lifecycle_publisher`, `create_timer`). Only `test_health_bridge.py`
   and `test_anomaly_bridge.py` exist — **`ingest_bridge.py` and `edge_client.py` have no unit
   tests at all** (the most complex node, with threading/queue/rate-limiting logic, is untested).

2. In a real ROS 2 Humble workspace: `colcon test --packages-select eimdall_ros2_bridge`.
   `CMakeLists.txt` has no `if(BUILD_TESTING: ...)` block wiring `ament_lint_auto_find_test_dependencies()`
   even though `package.xml` declares `ament_lint_auto`/`ament_lint_common` as `test_depend` — so
   `colcon test` currently has nothing registered to run for this package. The pytest suite is
   **not** hooked into `colcon test`; in CI it runs as a separate job that just does `pip install
   pytest && pytest -q` with no ROS install at all.

CI (`.github/workflows/ci.yml`): two jobs, `python` (setup-python 3.11, pytest) and `ros2`
(`ros-tooling/action-ros-ci` on Ubuntu 22.04 / Humble — builds only, per the point above).

## 5. Hardware (`hardware/esp32_mpu6050/sketch.ino`)

Arduino sketch for an ESP32 + MPU6050 over I²C (`Wire.begin(21, 22)`), sampling accel/gyro/temp at
~10 Hz and printing one JSON line per sample over serial at 115200 baud. Standalone — not
compiled or tested by this package's build/CI. The main `eimdall` repo has a companion runbook at
`docs/product/hardware-esp32-mpu6050.md` (wiring, Arduino IDE setup, WSL/`usbipd-win` steps to
attach the ESP32's USB-serial port) that reproduces this exact sketch — note that doc still calls
the project "Robovis" in places, an old internal codename, not a different project.

## 6. Monitoring assets (`monitoring/`) — describe Eimdall Central, not this bridge

`grafana-dashboard.json` and `otel-collector-config.yaml` are reference ops artifacts for the
broader Eimdall platform (fleet-level Prometheus metrics from Eimdall Central, OTel traces),
bundled here for convenience. Neither file's content matches what you might expect from a
"ROS2 bridge dashboard" — no per-bridge ingest-rate/parse-error/spool-depth panels exist, and none
of the three bridge nodes emit OTel telemetry directly.

## 7. Gotchas for new ROS 2 developers

1. **Dual/hybrid build system**: `package.xml` declares `ament_cmake` as build type, yet the repo
   also ships `setup.py`/`setup.cfg` with `console_scripts` entry points. Those entry points are
   **not** what gets installed as the `ros2 run`-able executables under `ament_cmake_python` — the
   real executables are installed explicitly in `CMakeLists.txt` via `install(PROGRAMS
   scripts/health_bridge scripts/anomaly_bridge scripts/ingest_bridge ...)`, where each
   `scripts/*` file is a 6-line wrapper (`from eimdall_ros2_bridge.health_bridge import main;
   main()`). If you come from pure Python packaging, you'll expect the `setup.py` entry_points to
   matter here; they're effectively vestigial.
2. Lifecycle nodes, not plain nodes — see §1.
3. `EimdallHealth.msg` vs. README mismatch — see §3.
4. `config/bridge.yaml` isn't actually consumed by the launch file — it builds parameters purely
   from `LaunchConfiguration` launch args, and two of the YAML's parameters
   (`max_file_bytes`, `max_anomalies_per_tick`) are left commented out as documentation-only
   examples. Treat it as a template for production deployments (e.g. `ros2 run ... --ros-args
   --params-file config/bridge.yaml`), not something `ros2 launch` picks up automatically.
5. Monitoring assets describe Eimdall Central, not this bridge — see §6.
6. Numbered comments scattered through the code (`#596`, `#644`, `#688`, `#19`-`#24`...) are
   issue/PR references from the originating tracker — useful for `git blame` archaeology, but
   meaningless without that tracker. The git log (`M60`-`M62` tagged commits) shows these
   correspond to a series of milestone-tagged security/robustness hardening passes.
7. `edge_url` is hard-restricted to loopback — see §1 (`edge_client.py`). Passing a remote host
   fails node configuration rather than silently connecting.
8. Token file must be `0600` and owned by the running user when `strict_security:=true` (the
   default) on `ingest_bridge` — will fail configuration in containerized setups where the token
   is mounted with different ownership, unless `strict_security:=false` is explicitly set.
9. Test coverage gap: only `health_bridge` and `anomaly_bridge` have unit tests; `ingest_bridge.py`
   and `edge_client.py` have none.
10. A few French passages remain in the README's "Tests" section — an inconsistency worth
    cleaning up, since the rest of the README/docstrings/comments are English (this repo is
    public and should stay English-only per project convention).
