#!/usr/bin/env bash
# Builds the compiled parts of this repo's submodules into one merged install
# space at the repo root (build/, install/, log/). This repo's own launch/ and
# config/ are plain directories -- nothing for colcon to build there.
#
# Groups:
#   clearpath      the Husky driver packages Clearpath only ships for amd64:
#                  clearpath_hardware_interfaces (A200Hardware + battery
#                  nodes), clearpath_generator_robot, clearpath_diagnostics.
#                  The rest of the Clearpath stack comes from apt
#                  (ros-humble-clearpath-common).
#   zenoh_bridge   zenoh_bridge, aggregator, zenoh_bridge_msgs.
#   skeleton       skeleton_detection.
#   state_machine  Husky robot status, control mode and navigation action.
#   camera_tower   Three-camera image combiner.
#
# patrolknight_msgs (modules/patrolknight_msgs, the shared messages) is built
# with the groups that depend on it, so it is pinned with the other submodules.
#
# Building one group leaves the other's already-installed packages in place.
#
# Usage:
#   ./scripts/build.sh                     # all groups
#   ./scripts/build.sh clearpath           # one group
#   ./scripts/build.sh --deps clearpath    # rosdep-install its dependencies first
#   ./scripts/build.sh --deps zenoh_bridge # run zenoh_bridge's install.sh first
#   ./scripts/build.sh --deps skeleton     # download skeleton_detection's models first
#   ./scripts/build.sh state_machine       # rebuild the state machine only
#   ./scripts/build.sh camera_tower        # rebuild the camera combiner only
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

declare -A GROUP_PATHS=(
  [clearpath]="modules/clearpath_robot/clearpath_hardware_interfaces modules/clearpath_robot/clearpath_generator_robot modules/clearpath_robot/clearpath_diagnostics"
  [zenoh_bridge]="modules/zenoh_bridge/src/aggregator modules/zenoh_bridge/src/zenoh_bridge modules/zenoh_bridge/src/zenoh_bridge_msgs"
  [skeleton]="modules/skeleton_detection"
  [state_machine]="modules/state_machine"
  [camera_tower]="modules/camera_tower"
)
ALL_GROUPS=(clearpath zenoh_bridge skeleton state_machine camera_tower)

usage() {
  echo "Usage: $(basename "$0") [--deps] [clearpath] [zenoh_bridge] [skeleton] [state_machine] [camera_tower]" >&2
  exit 1
}

deps=false
groups=()
for arg in "$@"; do
  case "$arg" in
    --deps) deps=true ;;
    -h|--help) usage ;;
    *)
      [[ -n "${GROUP_PATHS[$arg]:-}" ]] || { echo "Unknown group '$arg' (known: ${ALL_GROUPS[*]})" >&2; exit 1; }
      groups+=("$arg")
      ;;
  esac
done
[[ ${#groups[@]} -gt 0 ]] || groups=("${ALL_GROUPS[@]}")

if [[ -z "${ROS_DISTRO:-}" ]]; then
  echo "ROS_DISTRO is not set. Source ROS 2 Humble first, e.g.:" >&2
  echo "  source /opt/ros/humble/setup.bash" >&2
  exit 1
fi

if $deps && [[ " ${groups[*]} " == *" zenoh_bridge "* ]]; then
  echo "==> zenoh_bridge install.sh"
  modules/zenoh_bridge/scripts/install.sh
fi

if $deps && [[ " ${groups[*]} " == *" skeleton "* ]]; then
  echo "==> skeleton_detection models"
  python3 modules/skeleton_detection/docker/fetch_models.py --models-dir modules/skeleton_detection/models
fi

paths=()
if [[ " ${groups[*]} " =~ " "(zenoh_bridge|skeleton|state_machine)" " ]]; then
  [[ -f modules/patrolknight_msgs/package.xml ]] || { echo "Missing modules/patrolknight_msgs -- run: git submodule update --init --recursive" >&2; exit 1; }
  paths+=(modules/patrolknight_msgs)
fi
for group in "${groups[@]}"; do
  for path in ${GROUP_PATHS[$group]}; do
    [[ -d "$path" ]] || { echo "Missing $path -- run: git submodule update --init --recursive" >&2; exit 1; }
    paths+=("$path")
  done
done

if $deps; then
  echo "==> rosdep install for: ${groups[*]}"
  rosdep update
  rosdep install --from-paths "${paths[@]}" --ignore-src -r -y \
    || echo "==> rosdep reported unresolved keys (see above) -- continuing"
fi

echo "==> building: ${groups[*]}"
colcon build --merge-install \
  --paths "${paths[@]}" \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

echo "==> done. Next:"
echo "  source install/setup.bash"
echo "  ./launch.sh"
