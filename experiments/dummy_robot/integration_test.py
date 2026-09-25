#!/usr/bin/env python3
"""End-to-end check of the dummy robot stack's Zenoh dataflow, from the
backend's side of zenohd:

  1. robot state (dummy_robot -> aggregator_node -> zenoh_bridge_node)
     reaches Zenoh
  2. a navigate_to_goal command sent over Zenoh is accepted, relayed to
     /dummy_robot/goal, and reported succeeded once the dummy arrives
  3. the robot's reported position actually ends up at that goal

Needs the stack up (see docker-compose.yml) and `pip install eclipse-zenoh`.
zenohd is wherever you run it -- pass its address, default is this machine:

  python3 integration_test.py
  python3 integration_test.py tcp/192.168.1.10:7447
"""

import json
import math
import sys
import threading
import time

import zenoh

DEVICE_ID = "husky-sim-01"  # config/zenoh_bridge.yaml
# mission-1's first patrol waypoint -- in-lanelet, a few meters from the
# dummy's starting pose (config/dummy_robot.yaml), so the route is short
# but real.
GOAL_X, GOAL_Y = -54.818426363850335, 66.03214713795371
ARRIVAL_TOLERANCE_M = 1.0
STATE_TIMEOUT_SEC = 15
NAV_TIMEOUT_SEC = 60


def check(name: str, condition: bool) -> bool:
    print(f"[{'PASS' if condition else 'FAIL'}] {name}")
    return condition


def main() -> int:
    router = sys.argv[1] if len(sys.argv) > 1 else "tcp/localhost:7447"
    print(f"zenohd: {router}, device_id: {DEVICE_ID}")

    config = zenoh.Config()
    config.insert_json5("mode", json.dumps("client"))
    config.insert_json5("connect/endpoints", json.dumps([router]))
    session = zenoh.open(config)

    states: list[dict] = []
    events: list[dict] = []
    first_state = threading.Event()

    def on_state(sample):
        states.append(json.loads(bytes(sample.payload)))
        first_state.set()

    state_sub = session.declare_subscriber(f"devices/{DEVICE_ID}/state", on_state)
    events_sub = session.declare_subscriber(
        f"devices/{DEVICE_ID}/events", lambda s: events.append(json.loads(bytes(s.payload))))

    ok = True
    ok &= check("robot state reaches Zenoh", first_state.wait(STATE_TIMEOUT_SEC))
    if not ok:
        session.close()
        print("\nSOME FAILED")
        return 1
    print(f"       first state: {states[-1]}")

    nav_cmd = {"command": "navigate_to_goal", "request_id": "it-nav-1", "x": GOAL_X, "y": GOAL_Y}
    session.put(f"devices/{DEVICE_ID}/commands", json.dumps(nav_cmd).encode())

    deadline = time.monotonic() + NAV_TIMEOUT_SEC
    while time.monotonic() < deadline:
        statuses = [e["status"] for e in events if e.get("request_id") == "it-nav-1"]
        if "succeeded" in statuses or "failed" in statuses:
            break
        time.sleep(0.5)

    statuses = [e["status"] for e in events if e.get("request_id") == "it-nav-1"]
    ok &= check("navigate_to_goal accepted", "accepted" in statuses)
    ok &= check("navigate_to_goal running", "running" in statuses)
    ok &= check("navigate_to_goal succeeded", "succeeded" in statuses)

    time.sleep(1.0)  # let a post-arrival state sample come through
    last = states[-1]
    distance = math.hypot(last["pose"]["x"] - GOAL_X, last["pose"]["y"] - GOAL_Y)
    ok &= check(f"robot state position reached the goal ({distance:.2f} m away)",
                distance <= ARRIVAL_TOLERANCE_M)

    state_sub.undeclare()
    events_sub.undeclare()
    session.close()

    print()
    print("ALL PASSED" if ok else "SOME FAILED")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
