#!/bin/bash

cd "$(dirname "$0")"
source install/setup.bash

ros2 launch launch/main.launch.py \
    husky:=true \
    realsense:=true \
    zenoh_bridge:=true \
    joystick:=false \
    foxglove:=false \
    sensors:=false \
    "$@"
