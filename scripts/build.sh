#!/usr/bin/env bash
# Builds modules/zenoh_bridge's packages into a single merged install space
# at the repo root. This repo's own launch/ and config/ are plain
# directories (no compiled nodes, nothing for colcon to build).
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

if [[ -z "${ROS_DISTRO:-}" ]]; then
  echo "ROS_DISTRO is not set. Source ROS 2 Humble first, e.g.:" >&2
  echo "  source /opt/ros/humble/setup.bash" >&2
  exit 1
fi

colcon build --merge-install \
  --paths modules/zenoh_bridge/src/* \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

echo "==> done. Next:"
echo "  source install/setup.bash"
echo "  ros2 launch launch/main.launch.py"
