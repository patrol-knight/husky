"""Top-level entry point for the Husky.

Brings up each of this repo's components, each toggled by its own
name:=true/false launch argument. Runs against the Orin's native ROS 2
install (no Docker).

Typical usage (after ./scripts/build.sh):

  source install/setup.bash
  ros2 launch launch/main.launch.py
  ros2 launch launch/main.launch.py realsense:=false zenoh_bridge:=true
  ros2 launch launch/main.launch.py sensors:=true   # Autoware workspace sourced too
  ros2 launch launch/main.launch.py joystick:=true foxglove:=true   # development tools

launch/ and config/ are plain source-tree directories (this repo has no
compiled nodes of its own, just orchestration), not an ament package share
dir -- editing a launch file or config value takes effect immediately, no
rebuild needed. Only modules/zenoh_bridge's packages are compiled.
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
# zenoh_bridge's own bringup launches zenoh_bridge_node, plus aggregator_node
# because config/zenoh_bridge.yaml sets mode: robot.
ZENOH_BRIDGE_LAUNCH = os.path.join(REPO_ROOT, "modules", "zenoh_bridge", "launch", "bringup.launch.py")
ZENOH_BRIDGE_CONFIG = os.path.join(REPO_ROOT, "config", "zenoh_bridge.yaml")
# Temporary VLP-16 + Fixposition bring-up, to be replaced by Autoware's own
# sensor launch. Needs the Autoware workspace sourced, so it's off by default.
SENSORS_LAUNCH = os.path.join(REPO_ROOT, "launch", "sensors.launch.py")
# Development tools, off by default.
JOYSTICK_LAUNCH = os.path.join(REPO_ROOT, "launch", "joystick.launch.py")
FOXGLOVE_LAUNCH = os.path.join(REPO_ROOT, "launch", "foxglove.launch.py")


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "realsense",
            default_value="true",
            description="Bring up the RealSense camera",
        ),
        DeclareLaunchArgument(
            "zenoh_bridge",
            default_value="true",
            description="Bring up zenoh_bridge_node + aggregator_node",
        ),
        DeclareLaunchArgument(
            "sensors",
            default_value="false",
            description="Bring up the VLP-16 lidar + Fixposition GNSS/IMU (needs the "
                        "Autoware workspace sourced)",
        ),
        DeclareLaunchArgument(
            "joystick",
            default_value="false",
            description="Development: drive the Husky with a controller plugged into the Orin",
        ),
        DeclareLaunchArgument(
            "foxglove",
            default_value="false",
            description="Development: foxglove_bridge on ws://<orin-ip>:8765 for remote monitoring",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(REALSENSE_LAUNCH),
            launch_arguments={"config_file": REALSENSE_CONFIG}.items(),
            condition=IfCondition(LaunchConfiguration("realsense")),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ZENOH_BRIDGE_LAUNCH),
            launch_arguments={"device_config": ZENOH_BRIDGE_CONFIG}.items(),
            condition=IfCondition(LaunchConfiguration("zenoh_bridge")),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(SENSORS_LAUNCH),
            condition=IfCondition(LaunchConfiguration("sensors")),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(JOYSTICK_LAUNCH),
            condition=IfCondition(LaunchConfiguration("joystick")),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(FOXGLOVE_LAUNCH),
            condition=IfCondition(LaunchConfiguration("foxglove")),
        ),
    ])
