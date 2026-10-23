# Husky
Bringup for the Clearpath Husky A200 on an NVIDIA Jetson Orin NX (ROS 2 Humble).

## Launch
```bash
./launch.sh                           # default: husky + realsense
./launch.sh realsense:=false foxglove:=true
```
`launch.sh` sources `install/setup.bash` and runs `launch/main.launch.py`. Any `name:=value` argument you pass overrides the defaults below.

| Argument       | Default | Description |
| -------------- | ------- | ----------- |
| `husky`        | `true`  | Husky A200 driver (Clearpath stack: MCU, controllers, battery) |
| `realsense`    | `true`  | RealSense camera (`config/realsense.yaml`) |
| `zenoh_bridge` | `false` | `zenoh_bridge_node` + `aggregator_node` (`config/zenoh_bridge.yaml`) |
| `joystick`     | `false` | Development: drive the Husky with a controller plugged into the Orin. **Not working yet**: the Xbox controller firmware can't be installed on the Orin |
| `foxglove`     | `false` | Development: `foxglove_bridge` on `ws://<orin-ip>:8765` for remote monitoring |
| `sensors`      | `false` | VLP-16 lidar + Fixposition GNSS/IMU (needs the Autoware workspace sourced) |
