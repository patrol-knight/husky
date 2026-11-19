# Husky
Bringup for the Clearpath Husky A200 on an NVIDIA Jetson Orin NX (ROS 2 Humble).

## Build
```bash
git submodule update --init --recursive
./scripts/build.sh --deps               # first time: install dependencies
./scripts/build.sh [submodule]         # later: rebuild all or specific one group 
```
`--deps zenoh_bridge` runs `modules/zenoh_bridge/scripts/install.sh` (apt packages, libnice 0.1.23, `patrolknight_msgs`), so run it from a real terminal: it prompts for `sudo`. `--deps skeleton` downloads the skeleton_detection model weights into `modules/skeleton_detection/models/`.

## Launch
```bash
./launch.sh                           
./launch.sh realsense:=false foxglove:=true
```
`launch.sh` sources `install/setup.bash` and runs `launch/main.launch.py`. Any `name:=value` argument you pass overrides the defaults below.

To use the three-camera tower, build it with `./scripts/build.sh camera_tower`,
then run `./launch.sh camera_tower:=true realsense:=false`. Both cameras publish
the same image topic, so enable one at a time. Set the camera devices in
`config/camera_combiner.yaml`. Changes to that file take effect on the next launch
without rebuilding.

| Argument       | Default | Description |
| -------------- | ------- | ----------- |
| `husky`        | `true`  | Husky A200 driver (Clearpath stack: MCU, controllers, battery) |
| `realsense`    | `true`  | RealSense camera (`config/realsense.yaml`) |
| `camera_tower` | `false` | Three-camera image combiner from [modules/camera_tower](modules/camera_tower/README.md), configured by `config/camera_combiner.yaml`; publishes `/device/camera/front/image_raw` |
| `zenoh_bridge` | `true`  | `zenoh_bridge_node` + `aggregator_node` (`config/zenoh_bridge.yaml`; set `zenoh.connect` to the mission manager's `zenohd`) |
| `skeleton`     | `false` | `skeleton_detection` node (`config/skeleton_detection.yaml`), consumes the RealSense RGBD topic (needs `realsense:=true`) |
| `state_machine` | `true` | `/robot/status` (state, mode, reason, goal) and the `/robot/navigate` action, from Autoware and Clearpath topics; see [modules/state_machine](modules/state_machine/README.md) |
| `foxglove`     | `false` | Development: `foxglove_bridge` on `ws://<orin-ip>:8765` for remote monitoring |
| `sensors`      | `false` | VLP-16 lidar + Fixposition GNSS/IMU (needs the Autoware workspace sourced) |

