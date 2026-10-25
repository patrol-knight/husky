#!/bin/bash

# record_battery.sh
# Record Husky battery test data
cd "$(dirname "$0")/.."
source install/setup.bash
mkdir -p data
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
BAG_NAME="data/battery_test_${TIMESTAMP}"

echo "Starting battery test recording..."
echo "Bag name: ${BAG_NAME}"

ros2 bag record \
  -o "${BAG_NAME}" \
  /husky/platform/bms/state \
  /husky/platform/mcu/status/power \
  /husky/platform/odom \
  /husky/platform/cmd_vel_unstamped \
  /husky/platform/motors/left/temperature \
  /husky/platform/motors/right/temperature \
  /husky/platform/driver/left/temperature \
  /husky/platform/driver/right/temperature \
  /husky/platform/emergency_stop \
  /husky/diagnostics \
  /husky/platform/mcu/status \
  /husky/platform/joint_states