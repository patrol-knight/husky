# Husky
Bringup for the Clearpath Husky A200 on an NVIDIA Jetson Orin NX (ROS 2 Humble).

## Build
```bash
git submodule update --init --recursive
source /opt/ros/humble/setup.bash
./scripts/build.sh --deps               # first time: install dependencies, then build
./scripts/build.sh zenoh_bridge         # later: rebuild one group (clearpath | zenoh_bridge)
```
`--deps zenoh_bridge` runs `modules/zenoh_bridge/scripts/install.sh` (apt packages, libnice 0.1.23, `patrolknight_msgs`), so run it from a real terminal: it prompts for `sudo`.

## Launch
```bash
./launch.sh                           # default: husky + realsense + zenoh_bridge
./launch.sh husky:=false              # camera + zenoh_bridge only (WebRTC test)
./launch.sh realsense:=false foxglove:=true
```
`launch.sh` sources `install/setup.bash` and runs `launch/main.launch.py`. Any `name:=value` argument you pass overrides the defaults below.

| Argument       | Default | Description |
| -------------- | ------- | ----------- |
| `husky`        | `true`  | Husky A200 driver (Clearpath stack: MCU, controllers, battery) |
| `realsense`    | `true`  | RealSense camera (`config/realsense.yaml`) |
| `zenoh_bridge` | `true`  | `zenoh_bridge_node` + `aggregator_node` (`config/zenoh_bridge.yaml`; set `zenoh.connect` to the mission manager's `zenohd`) |
| `foxglove`     | `false` | Development: `foxglove_bridge` on `ws://<orin-ip>:8765` for remote monitoring |
| `sensors`      | `false` | VLP-16 lidar + Fixposition GNSS/IMU (needs the Autoware workspace sourced) |
