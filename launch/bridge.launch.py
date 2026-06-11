from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument("robot_id",   default_value="robot-01",
                              description="Unique robot identifier in your Eimdall tenant"),
        DeclareLaunchArgument("bridge_id",  default_value="ros2-bridge",
                              description="Bridge instance name"),
        DeclareLaunchArgument("edge_url",   default_value="https://127.0.0.1:8787",
                              description="Eimdall Edge local service URL"),
        DeclareLaunchArgument("token_file", default_value="/etc/eimdall/eimdall-local-service.token",
                              description="Path to the Edge local service token file"),
        DeclareLaunchArgument("ca_cert",    default_value="/etc/eimdall/tls/edge-local-ca.crt",
                              description="Path to the Edge CA certificate (leave empty to disable verification)"),
        DeclareLaunchArgument("heartbeat_interval_s", default_value="5.0",
                              description="Heartbeat interval in seconds"),

        Node(
            package="eimdall_ros2_bridge",
            executable="bridge",
            name="eimdall_bridge",
            output="screen",
            parameters=[{
                "robot_id":              LaunchConfiguration("robot_id"),
                "bridge_id":             LaunchConfiguration("bridge_id"),
                "edge_url":              LaunchConfiguration("edge_url"),
                "token_file":            LaunchConfiguration("token_file"),
                "ca_cert":               LaunchConfiguration("ca_cert"),
                "heartbeat_interval_s":  LaunchConfiguration("heartbeat_interval_s"),
            }],
        ),
    ])
