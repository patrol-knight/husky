# Robot status design

Status: implemented 2026-11-04 on `fix/state-machine` (`patrolknight_msgs`, `state_machine`, `autoware`) and `feat/husky-sm` (husky, `zenoh_bridge`, mission manager). Verified against `dummy_robot`, not yet on the Husky. Remove this file once it is merged and the [open items](#to-fix) are done.

How it works now lives in the READMEs:

- `modules/state_machine/README.md`: `/robot/status` (state, mode, reason, goal_id) and the `/robot/navigate` action.
- `modules/zenoh_bridge/README.md`: the aggregator and the navigation relay.
- `experiments/dummy_robot/README.md`: the fake Autoware and Clearpath topics.

## Why it changed

| Before | Problem | Now |
| --- | --- | --- |
| Autoware `robot_state` → `state_machine` → aggregator `status_map` → mission manager | Three vocabularies (`navigate` → `navigating`, `ready` → `idle` → "Ready") | One vocabulary, passed through unchanged |
| `reach` published once, `ready` 0.2 s later | Missed between ticks; the aggregator held `arrived`; mission manager guessed arrival within 2 m, which completed close waypoints early | `arrived` lasts; a goal ends on its own action result |
| `/robot_mode` from `joy_teleop` only | RC and marker driving not seen, with or without Autoware | `manual` from all three `twist_mux` inputs, `estop` from Clearpath |
| Autoware `control_mode` | Reads `MANUAL` after every arrival; absent without Autoware | `mode` from Clearpath topics, `auto` from the operation mode |
| No reason, no goal id | UI could only say "not ready"; results unmatched to goals | `reason` and `goal_id` |

Principles: one vocabulary; one owner per fact; states are states and events are results; who drives is a platform fact, reported without Autoware; the rules are a pure, unit-tested core.

## Mission manager

- Reads `status.state` / `mode` / `reason` / `goal_id`. Can start a mission when `state` is `ready`, `navigating` or `arrived` and `mode` is `none` or `auto`.
- A waypoint or investigation goal ends on its command result (matched by `request_id`): `succeeded` reaches it; `goal_dropped` and `state_machine_unavailable` send it again, as does no `running` within `navigation_timeout_sec` (10 s); `preempted` is ignored; any other `failed` aborts autonomy without a cancel, except the route failures below.
- `route_rejected` / `route_aborted` (or `state: failure`) pause a running mission without a cancel: Autoware keeps retrying while the operator drives the robot back onto the lanes with RC, then Resume (once `ready`) re-sends the same waypoint, or Cancel clears the failure. An investigation is aborted instead.
- `manual` aborts without a cancel (the route is already cleared); `notready` and `estop` abort with one (an e-stop leaves the route set).
- The Start hint uses `reason`; the mode pill shows for `auto`, `manual` and `estop`.

## Merge order

Each layer reads what the previous one publishes, so merge together, in order: `patrolknight_msgs`, `state_machine`, `zenoh_bridge`, mission manager, husky (submodule bumps), then `autoware` (removes `robot_state` and the RViz `RobotStatusPanel` / `RobotModePanel`; a panel for `/robot/status` would need RViz started with the husky workspace sourced).

## Verifying on the Husky

| Case | Expect in mission manager |
| --- | --- |
| `./launch.sh` only, robot still | Not ready, "Autoware is not running", no mode pill |
| `./launch.sh` only, RC driving | Not ready + Manual |
| E-stop pressed | E-stop pill, Start blocked |
| Autoware up, localized | Ready |
| Mission | Navigating + Auto → Arrived → next waypoint, also with waypoints < 2 m apart |
| Joystick or RC mid-mission | Ready + Manual, mission aborted (`goal_manual_takeover`) |
| Goal off the map | Failure, mission aborted (`goal_route_rejected`) |
| Cancel | Ready, robot stopped |
| `state_machine` killed | Not ready, "No status from the robot" within 2 s |

## To fix

### Readiness is latched

Kept from autoware `948c4bef`. After readiness is first established, a stale LiDAR / IMU / wheel odometry, lost localization or Autoware stopping keeps reporting `ready` / `navigating`; mission manager doesn't show it or abort. Fix: check health continuously in `StatusCore::status`, abort the active goal with the health reason and publish `/husky/cancel`; whatever topic made the latch necessary gets a longer timeout or leaves the check.

### Battery

`/husky/platform/bms/state` doesn't come from a BMS: the A200 has none. Clearpath's `battery_state_estimator` averages the last 30 MCU voltage readings and looks the voltage up in a 12 V lead-acid table, so the percentage drops while driving (voltage sag under load) and comes back when the robot stops.

Measured at rest on 2026-11-04 (full battery): 2.4 Hz (the 30-reading average spans 12.5 s); voltage 25.25–26.11 V, stdev 0.26 V; `current` 1.1–2.4 A and real (not NaN); `percentage` stuck at 1.0 (the table tops out at 12.7 V per block). The table is steep: 0.1 V of pack voltage is about 6 %.

Driven with RC on 2026-11-04 (full battery, 240 s: 30 s still, 110 s driving, 100 s still):

| | Still | Driving |
| --- | --- | --- |
| `current` | 1.6 A | 4.5–9 A (10 s means), 25 A peak turning in place |
| `voltage` | 25.6–25.7 V | 24.7–25.2 V |
| `percentage` | 100 % | 56–80 %, back to 100 % within 10–20 s of stopping |

Fitting V = E − R·I over all samples: R ≈ 0.10 Ω (r = −0.80), E ≈ 25.75 V. `V + I·R` cuts the voltage spread from 0.46 V to 0.28 V, the same as the measurement noise at rest, so the sag is almost all I·R.

Implemented as `BatteryFilter` in the aggregator (`battery_resistance_ohm: 0.10`, `battery_series_blocks: 2` in `sources.yaml`): compensate (`V + R·I`), average over 30 s, look it up in the same SLA table, and only let it go down unless it stays at least 5 % higher for 60 s (charging). Still to check on the Husky below about 80 %, where the table isn't saturated: that the percentage stays flat while driving, and whether R needs tuning.

## Open questions

1. **Why was health latched in `948c4bef`?** Needed to fix the latch. To agree with the teammate.
2. **E-stop release mid-route.** Does the vehicle interface re-enable autonomy, or does the route need a new goal? Decides whether mission manager should pause rather than abort on `estop`.
3. **The adaptor's retry.** A rejected goal is retried once per second and, on success, cleared without driving. With goal results, retrying is mission manager's call; the retry could be turned off for goals from `/robot/navigate`.
