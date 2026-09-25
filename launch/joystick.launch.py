"""Joystick teleop for the Husky, for a controller plugged into the Orin.

Development/fallback control only -- off by default in main.launch.py.
Publishes to twist_mux's joystick input (/husky/joy_teleop/cmd_vel,
priority 10), so it pre-empts plain /husky/cmd_vel but not RC teleop or
the e-stop.

Runs under /husky/joystick rather than /husky: the Clearpath platform stack
already has its own /husky/joy_node + /husky/teleop_twist_joy_node, bound
to a fixed /dev/input/ps4 device that doesn't exist for this controller.

This drives the robot. Only enable it with the e-stop within reach.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_CONFIG = os.path.join(REPO_ROOT, "config", "joystick.yaml")


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=DEFAULT_CONFIG,
            description="joy_node + teleop_twist_joy_node parameters (device, axis/button "
                        "mapping, speeds) -- defaults to config/joystick.yaml",
        ),
        Node(
            package="joy_linux",
            executable="joy_linux_node",
            name="joy_node",
            namespace="/husky/joystick",
            parameters=[LaunchConfiguration("config_file")],
            output="screen",
        ),
        Node(
            package="teleop_twist_joy",
            executable="teleop_node",
            name="teleop_twist_joy_node",
            namespace="/husky/joystick",
            parameters=[LaunchConfiguration("config_file")],
            remappings=[("cmd_vel", "/husky/joy_teleop/cmd_vel")],
            output="screen",
        ),
    ])
