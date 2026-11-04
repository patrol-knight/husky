# Husky
Bringup for the Clearpath Husky A200 on an NVIDIA Jetson Orin NX (ROS 2 Humble).

## Build
```bash
git submodule update --init --recursive
./scripts/build.sh --deps               # first time: install dependencies, then build
./scripts/build.sh zenoh_bridge         # later: rebuild one group (clearpath | zenoh_bridge | skeleton | state_machine)
```
`--deps zenoh_bridge` runs `modules/zenoh_bridge/scripts/install.sh` (apt packages, libnice 0.1.23, `patrolknight_msgs`), so run it from a real terminal: it prompts for `sudo`. `--deps skeleton` downloads the skeleton_detection model weights into `modules/skeleton_detection/models/`.

## Launch
```bash
./launch.sh                           # default: husky + realsense + zenoh_bridge + state_machine
./launch.sh husky:=false state_machine:=false  # camera + zenoh_bridge only (WebRTC test)
./launch.sh realsense:=false foxglove:=true
./launch.sh state_machine:=false       # disable the state machine for one run
```
`launch.sh` sources `install/setup.bash` and runs `launch/main.launch.py`. Any `name:=value` argument you pass overrides the defaults below.

| Argument       | Default | Description |
| -------------- | ------- | ----------- |
| `husky`        | `true`  | Husky A200 driver (Clearpath stack: MCU, controllers, battery) |
| `realsense`    | `true`  | RealSense camera (`config/realsense.yaml`) |
| `zenoh_bridge` | `true`  | `zenoh_bridge_node` + `aggregator_node` (`config/zenoh_bridge.yaml`; set `zenoh.connect` to the mission manager's `zenohd`) |
| `skeleton`     | `false` | `skeleton_detection` node (`config/skeleton_detection.yaml`), consumes the RealSense RGBD topic (needs `realsense:=true`) |
| `state_machine` | `true` | Publishes `/robot_status` and `/robot_mode` from Autoware status, Husky odometry, and joystick commands |
| `foxglove`     | `false` | Development: `foxglove_bridge` on `ws://<orin-ip>:8765` for remote monitoring |
| `sensors`      | `false` | VLP-16 lidar + Fixposition GNSS/IMU (needs the Autoware workspace sourced) |

For Mission Manager to drive the robot, Autoware has to run with `launch_api:=false` (`run_autoware.sh` does), otherwise every goal on `/planning/mission_planning/goal` is routed twice. `aggregator_node` reads `/localization/kinematic_state` (pose), `/husky/platform/odom/filtered` (speed), `/husky/platform/bms/state` and `/robot_status`; goals and cancels go to `/planning/mission_planning/goal` and `/husky/cancel`.
