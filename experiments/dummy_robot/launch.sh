#!/usr/bin/env bash
# Choose which of this stack's docker-compose containers to run, mirroring
# ros2 launch's own name:=true/false argument style (same interface as the
# iot repo's launch.sh), and provide matching `build`/`down` commands.
#
#   ./launch.sh up                                  # defaults: dummy_robot + zenoh_bridge + realsense
#   ./launch.sh up realsense:=false                 # no RealSense plugged in
#   ./launch.sh build                               # build images only, don't start
#   ./launch.sh down                                # stop + remove everything, whatever was up
#
# Backed by docker-compose.yml's profiles -- every service there has a
# `profiles:` entry, so each node here maps to exactly one --profile and
# nothing starts by accident. zenohd is never started here -- see
# docker-compose.yml's header for where it comes from.
set -euo pipefail

STACK_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
COMPOSE=(docker compose -f "$STACK_DIR/docker-compose.yml" --project-directory "$STACK_DIR")

# node name -> compose profile name (docker-compose.yml)
declare -A NODE_PROFILE=(
  [dummy_robot]="dummy_robot"
  [zenoh_bridge]="zenoh_bridge"
  [realsense]="realsense"
)
ALL_PROFILES=(dummy_robot zenoh_bridge realsense)

usage() {
  cat >&2 <<USAGE
Usage:
  $(basename "$0") up [<node>:=<true|false> ...]
  $(basename "$0") build [<node>:=<true|false> ...]
  $(basename "$0") down

Known nodes: ${!NODE_PROFILE[*]} (default: all true)
USAGE
}

# Parses <node>:=<true|false> args into the global `selected` array (node
# names whose value ended up true) and `profile_flags` array (their
# --profile flags for docker compose). Shared by up/build so their node
# selection logic can't drift apart.
parse_node_args() {
  declare -A value=(
    [dummy_robot]="true"
    [zenoh_bridge]="true"
    [realsense]="true"
  )
  for arg in "$@"; do
    case "$arg" in
      *:=*)
        name="${arg%%:=*}"
        val="${arg#*:=}"
        if [[ -z "${NODE_PROFILE[$name]:-}" ]]; then
          echo "Unknown node '$name'. Known nodes: ${!NODE_PROFILE[*]}" >&2
          exit 1
        fi
        if [[ "$val" != "true" && "$val" != "false" ]]; then
          echo "Invalid value for '$name': '$val' (expected true or false)" >&2
          exit 1
        fi
        value[$name]="$val"
        ;;
      *)
        echo "Malformed argument '$arg', expected '<node>:=<true|false>'" >&2
        exit 1
        ;;
    esac
  done

  profile_flags=()
  selected=()
  for node in "${!value[@]}"; do
    if [[ "${value[$node]}" == "true" ]]; then
      profile_flags+=(--profile "${NODE_PROFILE[$node]}")
      selected+=("$node")
    fi
  done

  if [[ ${#selected[@]} -eq 0 ]]; then
    echo "No nodes selected -- nothing to do." >&2
    exit 1
  fi
}

cmd_up() {
  parse_node_args "$@"
  echo "==> starting: ${selected[*]}"
  "${COMPOSE[@]}" "${profile_flags[@]}" up --build
}

cmd_build() {
  parse_node_args "$@"
  echo "==> building: ${selected[*]}"
  "${COMPOSE[@]}" "${profile_flags[@]}" build
}

cmd_down() {
  profile_flags=()
  for profile in "${ALL_PROFILES[@]}"; do
    profile_flags+=(--profile "$profile")
  done
  echo "==> stopping and removing all containers/networks from docker-compose.yml"
  "${COMPOSE[@]}" "${profile_flags[@]}" down --remove-orphans
}

case "${1:-}" in
  up)
    shift
    cmd_up "$@"
    ;;
  build)
    shift
    cmd_build "$@"
    ;;
  down)
    cmd_down
    ;;
  *)
    usage
    exit 1
    ;;
esac
