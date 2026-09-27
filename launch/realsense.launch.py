"""RealSense camera bring-up for the Husky.

Launches realsense2_camera_node directly (not realsense2_camera's rs_launch.py
convenience wrapper) so every tuning knob -- resolution, fps, serial_no --
lives in one ROS parameter YAML (config/realsense.yaml) instead of a pile of
individual launch arguments.

Runs against the Orin's native ROS 2 install (no Docker), so the
realsense2_camera package must be installed and sourced on the host.

The color stream is remapped onto the fixed topic zenoh_bridge's camera
gateway subscribes to (kCameraImageTopic in modules/zenoh_bridge).
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_CONFIG = os.path.join(REPO_ROOT, "config", "realsense.yaml")

# realsense2_camera_node's default color topic (name=camera, namespace=camera).
REALSENSE_DEFAULT_COLOR_TOPIC = "/camera/camera/color/image_raw"


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "image_topic",
            default_value="/device/camera/front/image_raw",
            description="Topic the RealSense color stream is remapped to -- must match "
                        "zenoh_bridge's camera gateway topic",
        ),
        DeclareLaunchArgument(
            "config_file",
            default_value=DEFAULT_CONFIG,
            description="ROS parameter YAML for realsense2_camera_node (resolution/fps/"
                        "serial_no) -- defaults to config/realsense.yaml",
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
