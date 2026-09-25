"""Top-level entry point for the Husky.

Brings up each of this repo's components, each toggled by its own
name:=true/false launch argument. Runs against the Orin's native ROS 2
install (no Docker).

Typical usage:

  ros2 launch launch/main.launch.py
  ros2 launch launch/main.launch.py realsense:=false

launch/ and config/ are plain source-tree directories (this repo has no
compiled nodes of its own, just orchestration), not an ament package share
dir -- editing a launch file or config value takes effect immediately, no
colcon build/install step needed.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REALSENSE_LAUNCH = os.path.join(REPO_ROOT, "launch", "realsense.launch.py")
REALSENSE_CONFIG = os.path.join(REPO_ROOT, "config", "realsense.yaml")


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "realsense",
            default_value="true",
            description="Bring up the RealSense camera",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(REALSENSE_LAUNCH),
            launch_arguments={"config_file": REALSENSE_CONFIG}.items(),
            condition=IfCondition(LaunchConfiguration("realsense")),
        ),
    ])
