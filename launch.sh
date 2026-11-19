#!/bin/bash

cd "$(dirname "$0")"
source install/setup.bash

# Follow Mission Manager's clock: the zenoh.connect host also serves NTP.
sync_clock() {
    local server line file=/etc/chrony/sources.d/patrolknight.sources
    server="$(sed -n 's|^[[:space:]]*zenoh\.connect:.*tcp/\([0-9.]*\):.*|\1|p' config/zenoh_bridge.yaml | head -n1)"
    if [[ -z "$server" ]] || ! command -v chronyc >/dev/null; then
        echo "WARNING: clock not following Mission Manager (no chrony, or no zenoh.connect address)" >&2
        return
    fi
    line="server $server iburst prefer"
    if [[ "$(cat "$file" 2>/dev/null)" != "$line" ]]; then
        echo "==> clock: following $server"
        echo "$line" | sudo tee "$file" >/dev/null && sudo chronyc reload sources >/dev/null
    fi
}
sync_clock

ros2 launch launch/main.launch.py \
    husky:=true \
    realsense:=false \
    camera_tower:=true \
    zenoh_bridge:=true \
    skeleton:=false \
    state_machine:=true \
    foxglove:=false \
    sensors:=false \
    "$@"
