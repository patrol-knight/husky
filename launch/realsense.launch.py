"""RealSense camera bring-up for the Husky.

Launches realsense2_camera_node directly (not realsense2_camera's rs_launch.py
convenience wrapper) so every tuning knob -- resolution, fps, serial_no --
lives in one ROS parameter YAML (config/realsense.yaml) instead of a pile of
individual launch arguments.

Runs against the Orin's native ROS 2 install (no Docker), so the
realsense2_camera package must be installed and sourced on the host.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# This repo's launch/ and config/ are plain source-tree directories, not an
# ament package share dir -- both are direct children of the repo root.
REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_CONFIG = os.path.join(REPO_ROOT, "config", "realsense.yaml")


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=DEFAULT_CONFIG,
            description="ROS parameter YAML for realsense2_camera_node (resolution/fps/"
                        "serial_no) -- defaults to config/realsense.yaml",
        ),
        Node(
            package="realsense2_camera",
            executable="realsense2_camera_node",
            name="camera",
            namespace="camera",
            parameters=[LaunchConfiguration("config_file")],
            output="screen",
        ),
    ])
