#!/usr/bin/env bash
# Start/build/stop the dummy robot containers.
#
#   ./launch.sh up [<node>:=false ...]   # dummy_robot, state_machine, zenoh_bridge, realsense
#   ./launch.sh build
#   ./launch.sh down
set -euo pipefail

STACK_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
COMPOSE=(docker compose -f "$STACK_DIR/docker-compose.yml" --project-directory "$STACK_DIR")
BRIDGE_CONFIG="$STACK_DIR/config/zenoh_bridge.yaml"

# node name -> compose profile name (docker-compose.yml)
declare -A NODE_PROFILE=(
  [dummy_robot]="dummy_robot"
  [state_machine]="state_machine"
  [zenoh_bridge]="zenoh_bridge"
  [realsense]="realsense"
)
ALL_PROFILES=(dummy_robot state_machine zenoh_bridge realsense)

usage() {
  cat >&2 <<USAGE
Usage:
  $(basename "$0") up [<node>:=<true|false> ...]
  $(basename "$0") build [<node>:=<true|false> ...]
  $(basename "$0") down

Known nodes: ${!NODE_PROFILE[*]} (default: all true)
USAGE
}

# Sets globals `selected` and `profile_flags`.
parse_node_args() {
  declare -A value=(
    [dummy_robot]="true"
    [state_machine]="true"
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

# LAN IPv4: route to zenohd, else default route, else first non-Docker address.
detect_lan_ip() {
  local target ip_addr
  target="$(sed -n 's|^[[:space:]]*zenoh\.connect:.*tcp/\([0-9.]*\):.*|\1|p' "$BRIDGE_CONFIG" | head -n1)"
  [[ -z "$target" || "$target" == 127.* ]] && target="1.1.1.1"
  ip_addr="$(ip -4 route get "$target" 2>/dev/null | sed -n 's/.* src \([0-9.]*\).*/\1/p' | head -n1)"
  if [[ -z "$ip_addr" ]]; then
    ip_addr="$(ip -4 -o addr show scope global \
      | awk '$2 !~ /^(docker|br-|veth)/ {split($4, a, "/"); print a[1]; exit}')"
  fi
  echo "$ip_addr"
}

# Exports WEBRTC_ANNOUNCED_IP (env wins, else detected) for the browser to reach the camera.
check_webrtc_address() {
  if [[ -z "${WEBRTC_ANNOUNCED_IP:-}" ]] && command -v ip >/dev/null; then
    export WEBRTC_ANNOUNCED_IP="$(detect_lan_ip)"
  fi
  if [[ -z "${WEBRTC_ANNOUNCED_IP:-}" ]]; then
    echo "WARNING: no LAN address found: camera video only reaches a browser on this machine." >&2
  else
    echo "==> camera video: browsers connect to $WEBRTC_ANNOUNCED_IP:${WEBRTC_ICE_PORT:-50000}/udp"
  fi
}

cmd_up() {
  parse_node_args "$@"
  if [[ " ${selected[*]} " == *" zenoh_bridge "* ]]; then
    check_webrtc_address
  fi
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
