import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "husky",
            default_value="true",
            description="Husky A200 driver (Clearpath stack)",
        ),
        DeclareLaunchArgument(
            "realsense",
            default_value="true",
            description="RealSense camera",
        ),
        DeclareLaunchArgument(
            "camera_tower",
            default_value="false",
            description="Three-camera image combiner",
        ),
        DeclareLaunchArgument(
            "zenoh_bridge",
            default_value="true",
            description="zenoh_bridge_node + aggregator_node",
        ),
        DeclareLaunchArgument(
            "skeleton",
            default_value="false",
            description="skeleton_detection rtmo_node (consumes the RealSense RGBD topic)",
        ),
        DeclareLaunchArgument(
            "state_machine",
            default_value="true",
            description="Husky robot status and mode publisher",
        ),
        DeclareLaunchArgument(
            "foxglove",
            default_value="false",
            description="Dev: foxglove_bridge on ws://<orin-ip>:8765",
        ),
        DeclareLaunchArgument(
            "sensors",
            default_value="false",
            description="VLP-16 + Fixposition (needs the Autoware workspace sourced)",
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(REPO_ROOT, "launch/husky.launch.py")),
            condition=IfCondition(LaunchConfiguration("husky")),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(REPO_ROOT, "launch/realsense.launch.py")),
            launch_arguments={"config_file": os.path.join(REPO_ROOT, "config/realsense.yaml")}.items(),
            condition=IfCondition(LaunchConfiguration("realsense")),
        ),
        Node(
            package="three_camera_combiner",
            executable="camera_combiner_node",
            name="three_camera_combiner",
            parameters=[os.path.join(REPO_ROOT, "config/camera_combiner.yaml")],
            output="screen",
            condition=IfCondition(LaunchConfiguration("camera_tower")),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(REPO_ROOT, "modules/zenoh_bridge/launch/bringup.launch.py")),
            launch_arguments={"device_config": os.path.join(REPO_ROOT, "config/zenoh_bridge.yaml")}.items(),
            condition=IfCondition(LaunchConfiguration("zenoh_bridge")),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(REPO_ROOT, "modules/skeleton_detection/launch/skeleton_detection_bringup.launch.py")
            ),
            launch_arguments={
                "config": os.path.join(REPO_ROOT, "config/skeleton_detection.yaml"),
                "models_dir": os.path.join(REPO_ROOT, "modules/skeleton_detection/models"),
            }.items(),
            condition=IfCondition(LaunchConfiguration("skeleton")),
        ),
        Node(
            package="state_machine",
            executable="state_machine_node",
            name="state_machine",
            output="screen",
            condition=IfCondition(LaunchConfiguration("state_machine")),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(REPO_ROOT, "launch/sensors.launch.py")),
            condition=IfCondition(LaunchConfiguration("sensors")),
        ),
        # Drops past a 10 MB per-client buffer so a slow link can't back up the bridge.
        # Clients can publish and set params: development only.
        Node(
            package="foxglove_bridge",
            executable="foxglove_bridge",
            name="foxglove_bridge",
            parameters=[{
                "port": 8765,
                "address": "0.0.0.0",
                "topic_whitelist": [".*"],
                "send_buffer_limit": 10000000,
            }],
            output="screen",
            condition=IfCondition(LaunchConfiguration("foxglove")),
        ),
    ])
