"""Temporary VLP-16 + Fixposition bring-up, until Autoware's sensor launch replaces it.
Needs the Autoware workspace sourced."""

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
            description="nebula_velodyne parameter YAML for the VLP-16",
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
