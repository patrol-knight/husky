#!/bin/bash

cd "$(dirname "$0")"
source install/setup.bash

ros2 launch launch/main.launch.py \
    husky:=true \
    realsense:=true \
    zenoh_bridge:=true \
    skeleton:=false \
    state_machine:=true \
    foxglove:=false \
    sensors:=false \
    "$@"
