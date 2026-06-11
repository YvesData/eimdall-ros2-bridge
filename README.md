# eimdall_ros2_bridge

ROS 2 bridge for the [Eimdall](https://robovis.io) Edge runtime.

Subscribes to standard ROS 2 topics and forwards sensor data to the Eimdall Edge local service running on the robot. No dependency on Eimdall binaries in your ROS workspace — communication is done over a local HTTPS endpoint.

**MIT licensed. No demo required.**

---

## Requirements

| | |
|---|---|
| ROS 2 | Humble, Iron or Jazzy |
| Python | 3.10+ |
| Eimdall Edge | v0.9+ running on the robot ([request access](https://robovis.io/#cta)) |

---

## Install

```bash
cd ~/ros2_ws/src
git clone https://github.com/YvesData/eimdall-ros2-bridge.git

cd ~/ros2_ws
colcon build --packages-select eimdall_ros2_bridge
source install/setup.bash
```

---

## Usage

```bash
ros2 launch eimdall_ros2_bridge bridge.launch.py \
  robot_id:=robot-01 \
  edge_url:=https://127.0.0.1:8787 \
  token_file:=/etc/eimdall/eimdall-local-service.token
```

Or with a config file:

```bash
ros2 launch eimdall_ros2_bridge bridge.launch.py \
  --ros-args --params-file config/bridge.yaml
```

---

## Topic mapping

The bridge subscribes to these topics automatically if they exist:

| Topic | Message type | Eimdall family |
|-------|-------------|----------------|
| `/imu/data` | `sensor_msgs/Imu` | `imu` |
| `/battery_state` | `sensor_msgs/BatteryState` | `battery` |
| `/odom` | `nav_msgs/Odometry` | `encoder` |
| `/scan` | `sensor_msgs/LaserScan` | `lidar` |
| `/joint_states` | `sensor_msgs/JointState` | `motor` |

> **Custom topics** — remap any topic to one of the above using standard ROS 2 remapping:
> ```bash
> ros2 launch eimdall_ros2_bridge bridge.launch.py \
>   robot_id:=robot-01 \
>   --ros-args -r /imu/data:=/my_robot/imu/raw
> ```

---

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `robot_id` | `robot-01` | Robot identifier in your Eimdall tenant |
| `bridge_id` | `ros2-bridge` | Bridge instance name |
| `edge_url` | `https://127.0.0.1:8787` | Eimdall Edge local service URL |
| `token_file` | `/etc/eimdall/eimdall-local-service.token` | Path to the Edge token file |
| `ca_cert` | `/etc/eimdall/tls/edge-local-ca.crt` | Path to the Edge CA certificate |
| `heartbeat_interval_s` | `5.0` | Heartbeat interval (seconds) |

---

## How it works

```
Your robot sensors
      │
      ▼ ROS 2 topics
eimdall_ros2_bridge  ──HTTPS──▶  Eimdall Edge (127.0.0.1:8787)
                                        │
                                        ▼  DailyReport (protobuf)
                                 Eimdall Central
                                        │
                                        ▼
                               Dashboard + LLM insights
```

The bridge sends individual sensor readings to the Edge runtime via its local HTTPS API. The Edge runtime handles aggregation, anomaly detection, spooling and upload to Central — no internet connection required on the robot.

---

## Eimdall Edge

The Edge binary is distributed by DATA AGILITY after a demo.  
→ [Request access at robovis.io](https://robovis.io/#cta)

---

## License

MIT — see [LICENSE](LICENSE).
