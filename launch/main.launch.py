"""Top-level entry point for the Husky.

Brings up each of this repo's components, each toggled by its own
name:=true/false launch argument. Runs against the Orin's native ROS 2
install (no Docker).

Typical usage (after ./scripts/build.sh):

  source install/setup.bash
  ros2 launch launch/main.launch.py
  ros2 launch launch/main.launch.py realsense:=false zenoh_bridge:=true
  ros2 launch launch/main.launch.py husky:=false    # everything but the robot base
  ros2 launch launch/main.launch.py sensors:=true   # Autoware workspace sourced too
  ros2 launch launch/main.launch.py joystick:=true foxglove:=true   # development tools

launch/ and config/ are plain source-tree directories (this repo has no
compiled nodes of its own, just orchestration), not an ament package share
dir -- editing a launch file or config value takes effect immediately, no
rebuild needed. Only modules/zenoh_bridge's packages are compiled.

To drive from a PC instead of the Orin, run modules/joystick's
docker-compose.yml there and keep joystick:=false here.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
# Husky A200 driver (official Clearpath stack, generated from config/husky.yaml).
HUSKY_LAUNCH = os.path.join(REPO_ROOT, "launch", "husky.launch.py")
REALSENSE_LAUNCH = os.path.join(REPO_ROOT, "launch", "realsense.launch.py")
REALSENSE_CONFIG = os.path.join(REPO_ROOT, "config", "realsense.yaml")
# zenoh_bridge's own bringup launches zenoh_bridge_node, plus aggregator_node
# because config/zenoh_bridge.yaml sets mode: robot.
ZENOH_BRIDGE_LAUNCH = os.path.join(REPO_ROOT, "modules", "zenoh_bridge", "launch", "bringup.launch.py")
ZENOH_BRIDGE_CONFIG = os.path.join(REPO_ROOT, "config", "zenoh_bridge.yaml")
# Temporary VLP-16 + Fixposition bring-up, to be replaced by Autoware's own
# sensor launch. Needs the Autoware workspace sourced, so it's off by default.
SENSORS_LAUNCH = os.path.join(REPO_ROOT, "launch", "sensors.launch.py")
# Xbox joystick teleop (modules/joystick owns the mapping and speeds), off by
# default. Feeds twist_mux's joystick input (priority 10): pre-empts plain
# /husky/cmd_vel, not RC teleop or the e-stop. Runs under /husky/joystick --
# the Clearpath stack already has its own idle /husky/joy_node.
JOYSTICK_LAUNCH = os.path.join(REPO_ROOT, "modules", "joystick", "launch", "joystick.launch.py")


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "husky",
            default_value="true",
            description="Bring up the Husky A200 driver (Clearpath stack: MCU, controllers, "
                        "twist_mux, EKF, battery)",
        ),
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
            PythonLaunchDescriptionSource(HUSKY_LAUNCH),
            condition=IfCondition(LaunchConfiguration("husky")),
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
            launch_arguments={
                "namespace": "/husky/joystick",
                "cmd_vel_topic": "/husky/joy_teleop/cmd_vel",
            }.items(),
            condition=IfCondition(LaunchConfiguration("joystick")),
        ),
        # WebSocket server for the Foxglove app: ws://<orin-ip>:8765. Only
        # topics a Foxglove panel subscribes to are sent, and the per-client
        # send buffer drops messages past 10 MB rather than queueing, so a
        # slow wireless link can't back up the bridge. Connected clients can
        # also publish topics and set parameters -- development use only.
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
