import launch
import launch.actions
import launch.events
import launch_ros.actions
import launch_ros.event_handlers
import launch_ros.events.lifecycle
import lifecycle_msgs.msg
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def _auto_start(node: launch_ros.actions.LifecycleNode) -> list:
    """Returns event handlers that configure then activate a lifecycle node."""
    configure = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )
    activate = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
        )
    )
    return [
        # Send configure after 1 s to let the node finish initialising
        launch.actions.TimerAction(period=1.0, actions=[configure]),
        # Activate as soon as configure completes (node reaches 'inactive')
        launch.actions.RegisterEventHandler(
            launch_ros.event_handlers.OnStateTransition(
                target_lifecycle_node=node,
                goal_state="inactive",
                entities=[activate],
            )
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    # ── Launch arguments ────────────────────────────────────────────────────
    args = [
        DeclareLaunchArgument("health_path", default_value="runtime_health.json"),
        DeclareLaunchArgument("anomaly_path", default_value="runtime_anomalies.jsonl"),
        DeclareLaunchArgument("publish_period_sec", default_value="1.0"),
        DeclareLaunchArgument("poll_period_sec", default_value="0.5"),
        DeclareLaunchArgument("robot_id", default_value="robot-01"),
        DeclareLaunchArgument("bridge_id", default_value="ros2-ingest"),
        DeclareLaunchArgument("edge_url", default_value="http://127.0.0.1:8787"),
        DeclareLaunchArgument(
            "token_file",
            default_value="/etc/eimdall/eimdall-local-service.token",
        ),
        DeclareLaunchArgument("ca_cert", default_value=""),
        DeclareLaunchArgument("heartbeat_interval_s", default_value="5.0"),
        DeclareLaunchArgument("imu_topic", default_value="/imu/data"),
        DeclareLaunchArgument("battery_topic", default_value="/battery_state"),
        DeclareLaunchArgument("odom_topic", default_value="/odom"),
        DeclareLaunchArgument("scan_topic", default_value="/scan"),
        DeclareLaunchArgument("joint_states_topic", default_value="/joint_states"),
    ]

    # ── Lifecycle nodes ─────────────────────────────────────────────────────
    health_bridge = launch_ros.actions.LifecycleNode(
        package="eimdall_ros2_bridge",
        executable="health_bridge",
        name="eimdall_health_bridge",
        output="screen",
        parameters=[{
            "health_path": LaunchConfiguration("health_path"),
            "publish_period_sec": LaunchConfiguration("publish_period_sec"),
        }],
    )

    anomaly_bridge = launch_ros.actions.LifecycleNode(
        package="eimdall_ros2_bridge",
        executable="anomaly_bridge",
        name="eimdall_anomaly_bridge",
        output="screen",
        parameters=[{
            "anomaly_path": LaunchConfiguration("anomaly_path"),
            "poll_period_sec": LaunchConfiguration("poll_period_sec"),
        }],
    )

    ingest_bridge = launch_ros.actions.LifecycleNode(
        package="eimdall_ros2_bridge",
        executable="ingest_bridge",
        name="eimdall_ingest_bridge",
        output="screen",
        parameters=[{
            "robot_id": LaunchConfiguration("robot_id"),
            "bridge_id": LaunchConfiguration("bridge_id"),
            "edge_url": LaunchConfiguration("edge_url"),
            "token_file": LaunchConfiguration("token_file"),
            "ca_cert": LaunchConfiguration("ca_cert"),
            "heartbeat_interval_s": LaunchConfiguration("heartbeat_interval_s"),
            "imu_topic": LaunchConfiguration("imu_topic"),
            "battery_topic": LaunchConfiguration("battery_topic"),
            "odom_topic": LaunchConfiguration("odom_topic"),
            "scan_topic": LaunchConfiguration("scan_topic"),
            "joint_states_topic": LaunchConfiguration("joint_states_topic"),
        }],
    )

    return LaunchDescription(
        args
        + [health_bridge, anomaly_bridge, ingest_bridge]
        + _auto_start(health_bridge)
        + _auto_start(anomaly_bridge)
        + _auto_start(ingest_bridge)
    )
