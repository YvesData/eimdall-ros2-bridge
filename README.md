# eimdall_ros2_bridge

ROS 2 bridge for the [Eimdall](https://github.com/YvesData/eimdall) edge runtime.

Three managed lifecycle nodes connect the Eimdall Edge process to any ROS 2 system:

- **`health_bridge`** — polls `runtime_health.json` and publishes typed health messages
- **`anomaly_bridge`** — tails `runtime_anomalies.jsonl` incrementally and publishes anomaly events
- **`ingest_bridge`** — subscribes to standard sensor topics and forwards readings to the Edge HTTP API

---

## Architecture

```text
Eimdall Edge runtime
  │
  ├── runtime_health.json       (snapshot, overwritten every second)
  └── runtime_anomalies.jsonl   (append-only event log)
         │
         │  health_bridge  ──►  /eimdall/health             EimdallHealth
         │                 ──►  /eimdall/sensors/status      EimdallSensorStatus
         │
         │  anomaly_bridge ──►  /eimdall/anomalies           EimdallAnomaly
         │
         │  ingest_bridge  ◄──  /imu/data                    sensor_msgs/Imu
                          ◄──  /battery_state               sensor_msgs/BatteryState
                          ◄──  /odom                         nav_msgs/Odometry
                          ◄──  /scan                         sensor_msgs/LaserScan
                          ◄──  /joint_states                 sensor_msgs/JointState
                          ──►  Edge HTTP API  /v1/local/bridge/ingest

All nodes publish to /diagnostics  (diagnostic_msgs/DiagnosticArray)
```

---

## Custom message types

### `EimdallAnomaly`

| Field | Type | Description |
| --- | --- | --- |
| `stamp` | `builtin_interfaces/Time` | ROS 2 timestamp |
| `event_id` | `string` | Unique event identifier |
| `robot_id` | `string` | Source robot |
| `tenant_id` | `string` | Tenant identifier |
| `component` | `string` | Component that triggered the anomaly |
| `severity` | `int32` | 1=low · 2=medium · 3=high · 4=critical |
| `score` | `float64` | ML anomaly score 0.0–1.0 |
| `reason_codes` | `string[]` | List of contributing reason codes |

### `EimdallHealth`

| Field | Type | Description |
| --- | --- | --- |
| `stamp` | `builtin_interfaces/Time` | ROS 2 timestamp |
| `uptime_s` | `float64` | Edge process uptime in seconds |
| `configured_sensors` | `int32` | Total sensors configured |
| `active_sensors` | `int32` | Sensors currently active |
| `total_reconnects` | `int32` | Cumulative reconnect count |
| `total_lines_seen` | `int64` | Total JSONL lines processed |
| `total_anomaly_events` | `int64` | Total anomalies emitted |

### `EimdallSensorStatus`

| Field | Type | Description |
| --- | --- | --- |
| `stamp` | `builtin_interfaces/Time` | ROS 2 timestamp |
| `sensor_id` | `string` | Sensor identifier |
| `family` | `string` | Sensor family (imu, encoder, …) |
| `status` | `string` | ok · warning · error · offline |
| `confidence_pct` | `float32` | Confidence percentage 0–100 |
| `recent_readings` | `int64` | Readings in the last window |
| `last_reading_at_ms` | `int64` | Unix timestamp of last reading |

---

## Published topics

| Topic | Type | QoS | Description |
| --- | --- | --- | --- |
| `/eimdall/anomalies` | `EimdallAnomaly` | RELIABLE · TRANSIENT_LOCAL · depth 100 | One message per anomaly event |
| `/eimdall/health` | `EimdallHealth` | RELIABLE · VOLATILE · depth 10 | Global health snapshot, published on change |
| `/eimdall/sensors/status` | `EimdallSensorStatus` | RELIABLE · VOLATILE · depth 10 | Per-sensor status, one message per sensor |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | Default | Node health for rqt_robot_monitor |

---

## Nodes

### `health_bridge`

Polls `runtime_health.json` at a configurable rate. Only publishes when file content changes. Guards against partial writes with a JSON-parse retry loop (`_MAX_JSON_RETRIES=3`, 50 ms between attempts).

| Parameter | Default | Description |
| --- | --- | --- |
| `health_path` | `runtime_health.json` | Path to the Eimdall runtime health snapshot |
| `publish_period_sec` | `1.0` | Polling period in seconds |

---

### `anomaly_bridge`

Reads `runtime_anomalies.jsonl` incrementally using a byte offset. Detects file rotation (inode change or file shrink) and resets the offset automatically.

| Parameter | Default | Description |
| --- | --- | --- |
| `anomaly_path` | `runtime_anomalies.jsonl` | Path to the Eimdall anomaly JSONL file |
| `poll_period_sec` | `0.5` | Polling period in seconds |

---

### `ingest_bridge`

Subscribes to standard ROS 2 sensor topics and calls the Eimdall Edge local HTTP API. Requires a valid bridge token file. Sends heartbeats to the Edge on a configurable interval.

| Parameter | Default | Description |
| --- | --- | --- |
| `robot_id` | `robot-01` | Robot identifier |
| `bridge_id` | `ros2-ingest` | Bridge source identifier |
| `edge_url` | `http://127.0.0.1:8787` | Eimdall Edge local service URL |
| `token_file` | `/etc/eimdall/eimdall-local-service.token` | Path to the bridge token file |
| `ca_cert` | `` | CA certificate for HTTPS (leave empty for plain HTTP) |
| `heartbeat_interval_s` | `5.0` | Heartbeat interval in seconds |
| `imu_topic` | `/imu/data` | IMU topic |
| `battery_topic` | `/battery_state` | Battery state topic |
| `odom_topic` | `/odom` | Odometry topic |
| `scan_topic` | `/scan` | Laser scan topic |
| `joint_states_topic` | `/joint_states` | Joint states topic |

---

## Requirements

- ROS 2 Humble or later
- Python 3.10+
- `diagnostic_msgs`, `sensor_msgs`, `nav_msgs`, `lifecycle_msgs`

---

## Build & install

```bash
cd ~/ros2_ws
colcon build --packages-select eimdall_ros2_bridge
source install/setup.bash
```

---

## Usage

### Launch all nodes (auto configure + activate)

```bash
ros2 launch eimdall_ros2_bridge eimdall_bridge.launch.py \
  health_path:=/var/lib/eimdall/runtime_health.json \
  anomaly_path:=/var/lib/eimdall/runtime_anomalies.jsonl \
  robot_id:=my-robot-01 \
  edge_url:=http://127.0.0.1:8787
```

### Run nodes individually

```bash
# Health bridge
ros2 run eimdall_ros2_bridge health_bridge \
  --ros-args -p health_path:=/var/lib/eimdall/runtime_health.json

# Anomaly bridge
ros2 run eimdall_ros2_bridge anomaly_bridge \
  --ros-args -p anomaly_path:=/var/lib/eimdall/runtime_anomalies.jsonl

# Ingest bridge
ros2 run eimdall_ros2_bridge ingest_bridge \
  --ros-args -p robot_id:=my-robot-01 -p edge_url:=http://127.0.0.1:8787
```

### Manage lifecycle manually

```bash
# List nodes in unconfigured state
ros2 lifecycle list /eimdall_health_bridge

# Configure
ros2 lifecycle set /eimdall_health_bridge configure

# Activate
ros2 lifecycle set /eimdall_health_bridge activate

# Deactivate for maintenance
ros2 lifecycle set /eimdall_health_bridge deactivate
```

### Inspect topics

```bash
ros2 topic echo /eimdall/health
ros2 topic echo /eimdall/anomalies
ros2 topic echo /diagnostics
```

---

## Integration with Eimdall Edge

The Eimdall Edge runtime writes to the paths configured in
`EIMDALL_RUNTIME_HEALTH_PATH` and `EIMDALL_ANOMALY_PATH`. Point the bridge
parameters to the same paths.

Typical robot deployment:

```text
/var/lib/eimdall/
  ├── runtime_health.json       ← written by eimdall-edge service
  └── runtime_anomalies.jsonl   ← written by eimdall-edge service

/etc/eimdall/
  └── eimdall-local-service.token  ← required by ingest_bridge
```

---

## Hardware integration

### ESP32 + MPU6050 (`hardware/esp32_mpu6050/`)

A minimal Arduino sketch for an **ESP32** wired to an **MPU6050** IMU (I²C on pins 21/22). The sketch reads accelerometer, gyroscope and temperature at 10 Hz and prints one JSON line per sample on the serial port at 115 200 baud:

```json
{"ts_ms":1234,"ax_g":0.0012,"ay_g":-0.0034,"az_g":1.0001,"gx_dps":0.123,"gy_dps":-0.045,"gz_dps":0.002,"temp_c":28.50}
```

The Eimdall Edge runtime can read this stream directly when configured with a serial sensor source pointing to the device's USB port (e.g. `/dev/ttyUSB0`). No additional driver or middleware is required.

**Dependencies (Arduino Library Manager):**
- `Adafruit MPU6050`
- `Adafruit Unified Sensor`

This sketch is intentionally minimal — it only handles the sensor read loop. TLS, authentication and data forwarding are handled by the Edge runtime, not the microcontroller.

---

## Monitoring

Configuration files for integrating Eimdall into a standard observability stack are in `monitoring/`.

### Grafana dashboard (`monitoring/grafana-dashboard.json`)

A ready-to-import Grafana dashboard (schema v38, tested on Grafana 10+) showing:

- Fleet health overview — active robots, anomaly rate, sensor coverage
- Per-robot health score timeline
- Anomaly events by severity
- Edge runtime metrics — ingest rate, parse errors, spool depth

**Import:** Grafana → Dashboards → Import → upload `grafana-dashboard.json`. Select a Prometheus datasource connected to the Eimdall Central metrics endpoint (default port `9091`).

### OpenTelemetry Collector (`monitoring/otel-collector-config.yaml`)

A reference OTel Collector configuration that:

- Scrapes Eimdall Central's Prometheus endpoint (supports mTLS + bearer token)
- Receives traces pushed by Eimdall Central via OTLP gRPC/HTTP
- Re-exports to any compatible backend — Grafana Tempo, Jaeger, Datadog, or others (commented stanzas included)

```bash
docker run --rm \
  -v $(pwd)/monitoring/otel-collector-config.yaml:/etc/otelcol/config.yaml \
  -v /etc/eimdall/certs:/etc/otelcol/certs \
  -v /etc/eimdall/eimdall-token:/etc/otelcol/eimdall-token \
  -p 4317:4317 -p 4318:4318 \
  otel/opentelemetry-collector-contrib:latest
```

Replace `EIMDALL_HOST` and `GRAFANA_TEMPO_HOST` placeholders with your actual hostnames before use.

---

## Tests

Depuis la racine du package :

```bash
python3 -m compileall -q eimdall_ros2_bridge launch scripts
python3 -m pytest -q
```

Dans un workspace ROS 2 Humble :

```bash
colcon test --packages-select eimdall_ros2_bridge
colcon test-result --verbose
```

Ces commandes sont également exécutées par le workflow CI du dépôt.

---

## Repository layout

```text
eimdall_ros2_bridge/
├── eimdall_ros2_bridge/    Python package — lifecycle nodes + edge client
├── msg/                    Custom ROS 2 message definitions
├── launch/                 Launch file with auto configure+activate
├── config/                 Parameter YAML for all three nodes
├── hardware/
│   └── esp32_mpu6050/      Arduino sketch — ESP32 + MPU6050 → Edge serial
└── monitoring/
    ├── grafana-dashboard.json       Grafana fleet dashboard
    └── otel-collector-config.yaml  OTel Collector for metrics + traces
```

---

## License

Proprietary — © Eimdall / Data Agility. All rights reserved.
