"""RealSense camera, color stream remapped to zenoh_bridge's camera topic."""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_CONFIG = os.path.join(REPO_ROOT, "config", "realsense.yaml")

REALSENSE_DEFAULT_COLOR_TOPIC = "/camera/camera/color/image_raw"


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "image_topic",
            default_value="/device/camera/front/image_raw",
            description="Color image topic (must match zenoh_bridge's camera topic)",
        ),
        DeclareLaunchArgument(
            "config_file",
            default_value=DEFAULT_CONFIG,
            description="realsense2_camera_node parameter YAML",
        ),
        GroupAction([
            SetRemap(src=REALSENSE_DEFAULT_COLOR_TOPIC, dst=LaunchConfiguration("image_topic")),
            Node(
                package="realsense2_camera",
                executable="realsense2_camera_node",
                name="camera",
                namespace="camera",
                parameters=[LaunchConfiguration("config_file")],
                output="screen",
            ),
        ]),
    ])
