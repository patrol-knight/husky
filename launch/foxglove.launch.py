"""Foxglove bridge for remote monitoring/debugging (battery, speed, IMU,
...) from the Foxglove app.

Development only -- off by default in main.launch.py. Connect the Foxglove
app to ws://<orin-ip>:8765.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_CONFIG = os.path.join(REPO_ROOT, "config", "foxglove.yaml")


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=DEFAULT_CONFIG,
            description="foxglove_bridge parameters (port, topic whitelist) -- defaults to "
                        "config/foxglove.yaml",
        ),
        Node(
            package="foxglove_bridge",
            executable="foxglove_bridge",
            name="foxglove_bridge",
            parameters=[LaunchConfiguration("config_file")],
            output="screen",
        ),
    ])
