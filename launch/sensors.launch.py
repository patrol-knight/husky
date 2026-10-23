"""Temporary sensor bring-up for the Husky: VLP-16 lidar + Fixposition
GNSS/IMU.

Stand-in for Autoware's own sensor launch -- replace this whole file (and
its sensors:= switch in main.launch.py) with Autoware's launch once that is
wired up. Launches the same two upstream launch files as the old
sensor-driver shell script, but under ros2 launch, so Ctrl-C shuts every
node down cleanly.

Both driver packages (nebula_velodyne, fixposition_driver_ros2) come from
the Autoware workspace, which must be sourced before launching:

  source /opt/ros/humble/setup.bash
  source <autoware>/install/setup.bash
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_VLP16_CONFIG = os.path.join(REPO_ROOT, "config", "vlp16.param.yaml")


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "vlp16_config",
            default_value=DEFAULT_VLP16_CONFIG,
            description="nebula_velodyne parameter YAML for the VLP-16 -- defaults to "
                        "config/vlp16.param.yaml",
        ),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("nebula_velodyne"), "launch", "velodyne_launch_all_hw.xml"])),
            launch_arguments={
                "sensor_model": "VLP16",
                "config_file": LaunchConfiguration("vlp16_config"),
            }.items(),
        ),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("fixposition_driver_ros2"), "launch", "node.launch"])),
        ),
    ])
