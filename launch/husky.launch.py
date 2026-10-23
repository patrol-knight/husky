"""Husky A200 driver: the official Clearpath ROS 2 stack, run natively.

Every launch, this regenerates the Clearpath setup from config/husky.yaml
into .clearpath/ (gitignored) and then includes the generated
platform/launch/platform-service.launch.py -- the same file Clearpath's own
clearpath-platform.service runs. That brings up ros2_control_node with the
A200Hardware interface (opens /dev/clearpath/prolific to talk to the MCU),
the diff-drive controller, twist_mux, the EKF, and the battery/diagnostics
nodes, all under /husky.

Runs the same generators as Clearpath's clearpath-robot.service, minus
setup.bash (RMW is set here instead, ROS_DOMAIN_ID comes from the shell),
the discovery server (robot.yaml uses simple discovery) and vcan (the A200
talks over serial, not CAN). Nothing is installed into systemd, /etc or
another user -- never run `ros2 run clearpath_robot install` on the Orin,
it adds boot services that would start a second driver fighting over the
serial port.
"""

import os
import shutil
import subprocess

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import (
    LogInfo,
    GroupAction,
    OpaqueFunction,
    DeclareLaunchArgument,
    SetEnvironmentVariable,
    IncludeLaunchDescription,
)

from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_ROBOT_CONFIG = os.path.join(REPO_ROOT, "config", "husky.yaml")

SETUP_PATH = os.path.join(REPO_ROOT, ".clearpath") + os.sep
RMW_IMPLEMENTATION = "rmw_fastrtps_cpp"
SERIAL_DEVICE = "/dev/clearpath/prolific"

GENERATORS = [
    ("clearpath_generator_common", "generate_description"),
    ("clearpath_generator_common", "generate_semantic_description"),
    ("clearpath_generator_robot",  "generate_param"),
    ("clearpath_generator_robot",  "generate_launch"),
]


def _generate_and_launch(context, *args, **kwargs):
    robot_config = LaunchConfiguration("robot_config").perform(context)

    shutil.rmtree(SETUP_PATH, ignore_errors=True)
    os.makedirs(SETUP_PATH)
    shutil.copyfile(robot_config, os.path.join(SETUP_PATH, "robot.yaml"))

    env = dict(os.environ, RMW_IMPLEMENTATION=RMW_IMPLEMENTATION)
    for package, executable in GENERATORS:
        command = [os.path.join(get_package_prefix(package), "lib", package, executable), "-s", SETUP_PATH]
        result = subprocess.run(command, env=env, capture_output=True, text=True)
        if result.returncode != 0:
            raise RuntimeError(
                f"Clearpath generator {executable} failed on {robot_config}:\n"
                f"{result.stdout}{result.stderr}")

    platform_launch = os.path.join(SETUP_PATH, "platform", "launch", "platform-service.launch.py")
    actions = []
    if not os.path.exists(SERIAL_DEVICE):
        actions.append(LogInfo(msg=(
            f"[husky] WARNING: {SERIAL_DEVICE} not found -- the Husky driver will fail to start. "
            "Check the robot's USB-serial cable, or run scripts/setup_serial.sh once on this host.")))
    return actions + [
        GroupAction([
            SetEnvironmentVariable("RMW_IMPLEMENTATION", RMW_IMPLEMENTATION),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(platform_launch)),
        ]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "robot_config",
            default_value=DEFAULT_ROBOT_CONFIG,
            description="Clearpath robot.yaml for this Husky -- defaults to config/husky.yaml",
        ),
        OpaqueFunction(function=_generate_and_launch),
    ])
