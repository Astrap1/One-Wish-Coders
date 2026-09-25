# Project execution guide

This repository is building a Gazebo and ROS 2 demonstration of an autonomous air-cushion logistics vehicle operating in a simulated tidal-mangrove corridor. The objective is a credible, repeatable demonstration of route planning, obstacle response and safety-driven fallback—not a claim of field-validated hovercraft physics.

## Operating assumptions

- Target stack: **ROS 2 Jazzy**, **Gazebo Harmonic** and `ros_gz`.
- The live demonstration must work from one documented launch path owned by the vehicle simulation and integration lead.
- The safety supervisor is the only component permitted to publish `/cmd_vel`.
- Topic and message changes must be made with the matching update to `docs/INTERFACES.md`.
- Do not commit local PDFs, generated `build/`, `install/`, `log/`, recordings or experiment outputs.

See `docs/ARCHITECTURE.md` and `docs/INTERFACES.md` for the current system boundary and contracts.

## Vehicle configuration and mobility modes

The simulated vehicle has an inflatable air-cushion skirt, a lift fan, rear propulsion fans and wheels. The mobility model must demonstrate understandable control behaviour without claiming validated propeller, skirt or hovercraft physics.

The vehicle controller uses three internal mobility modes:

- **WHEEL** — used on firm ground. The lift fan is off or idling, the vehicle rests on its wheels, and the wheels execute the safety-approved motion command.
- **TRANSITION** — used while changing between wheel and hover operation. Horizontal motion stops, the lift fan ramps up or down, and the controller waits until the vehicle is either hover-ready or settled onto its wheels.
- **HOVER** — used over mud and shallow water. The lift fan maintains the simulated air cushion, the wheels are unloaded or ignored, and the rear propulsion fans execute the safety-approved forward and turning command.

These mobility modes are separate from Person 2's safety states and must not replace or rename `CRUISE`, `CAUTION`, `HOLD` or `RETURN`. They do not introduce new external commands or topics. The existing command authority remains:

1. Person 1 publishes route and motion proposals through `/planned_path` and `/cmd_vel_proposed`; autonomy does not directly command wheel speeds or fan RPM.
2. Person 2 applies the existing safety and mission logic and remains the only publisher of `/cmd_vel`.
3. Person 4 consumes the safety-approved `/cmd_vel` and owns the internal wheel, lift-fan and propulsion-fan control, including command limits, fan ramping, hover-ready checks and mobility-mode transitions.

A wheel-to-hover transition must stop horizontal motion, ramp the lift fan and wait for hover-ready status before propulsion begins. A hover-to-wheel transition must stop horizontal motion, reduce lift in a controlled way and confirm that the vehicle has settled onto its wheels before wheel motion begins. The safety-approved command remains authoritative in every mode. Any future topic or message change requires the matching update to `docs/INTERFACES.md`.

### Shared terrain-cost semantics

The terrain cost map supplies the shared route and mobility interpretation:

- `0`--`19`: firm shore; the controller uses **WHEEL** mode.
- `20`--`59`: mud or shallow water; the controller uses **HOVER** mode.
- `60`--`89`: elevated-risk mud or shallow water; the controller uses **HOVER** mode with conservative speed/energy assumptions.
- `90`--`100`: no-go terrain.
- `-1`: unknown terrain; treated as no-go.

Autonomy plans with these costs and Safety estimates the return energy/time using
the same bands. A path that crosses a `90+` or `-1` cell is invalid.

## Team roles

| Person | Role | Owns | Does not own |
| --- | --- | --- | --- |
| 1 | Autonomy lead | Global route planner, local obstacle response, replanning and proposed motion commands. | The final safety decision or complete-system launch. |
| 2 | Safety and mission lead | Mission progression, return-margin logic, state machine and safety-approved commands. | Gazebo vehicle physics or visual world assets. |
| 3 | Environment lead | Tidal corridor world, terrain zones, mangrove roots, debris, delivery area and tide-state visuals. | Navigation and safety policies. |
| 4 | Vehicle simulation and integration lead | Vehicle SDF model, sensors, mobility abstraction, ROS-Gazebo bridge, packages and one-command launch. | Mission-policy thresholds or dashboard narrative. |
| 5 | Operator, evaluation and demo lead | Operator display, scenario runner, fault injection controls, metrics, comparison visuals and presentation story. | Direct vehicle control outside the documented operator controls. |

## Role 1: Autonomy implementation status

- Implemented `global_planner`, which consumes `/terrain_costmap`, `/odom`, `/mission_goal` and `/terrain_state` and publishes `/planned_path` only.
- Implemented a ROS-independent, eight-connected A* core that minimises distance and terrain risk.
- Agreed planner interpretation of `/terrain_costmap`: `0`--`19` firm-wheel terrain, `20`--`59` hover terrain, `60`--`89` elevated-risk hover terrain, `90`--`100` no-go, and `-1` unknown/no-go.
- The planner replans after cost-map, goal or terrain-state updates, refuses mismatched frames, and publishes an empty path when a previously valid route becomes unsafe.
- Implemented `path_follower`, which consumes `/planned_path` and `/odom` and publishes forward and turning proposals on `/cmd_vel_proposed` at 10 Hz.
- The follower uses lookahead steering, slows near the goal, stops to correct large heading errors, and proposes zero motion for empty paths, stale odometry or mismatched frames.
- Added eleven ROS-independent planner/follower tests and two ROS topic integration tests.
- Verified the autonomy package builds in WSL and both `global_planner` and `path_follower` start successfully.

### Path follower implementation details

`path_follower` is split into a ROS-independent control core and a ROS 2 wrapper. The wrapper receives `/planned_path` and `/odom`, runs the controller at 10 Hz and publishes only `geometry_msgs/Twist` proposals on `/cmd_vel_proposed`. It sets `linear.x` for forward speed and `angular.z` for turning; all other `Twist` fields remain zero.

For each control update, the follower:

1. starts its search at the last reached portion of the path so progress does not move backwards;
2. selects the first waypoint at least the lookahead distance from the current pose, or the final waypoint when none is farther away;
3. calculates the shortest signed heading error between vehicle yaw and the selected waypoint;
4. applies proportional turning and clamps the result to the angular-speed limit;
5. stops forward motion when the heading error is large, otherwise reduces speed according to heading error and remaining goal distance; and
6. publishes zero forward and angular motion inside the goal tolerance.

Default path-follower parameters are:

| Parameter | Default | Purpose |
| --- | ---: | --- |
| `control_rate_hz` | `10.0` | Keep proposed commands fresh for Safety. |
| `max_linear_speed` | `0.8 m/s` | Bound the maximum forward proposal. |
| `max_angular_speed` | `1.0 rad/s` | Bound the maximum turning proposal. |
| `lookahead_distance` | `0.75 m` | Select a smoother target ahead on the path. |
| `goal_tolerance` | `0.25 m` | Stop when the vehicle is close enough to the final waypoint. |
| `heading_gain` | `1.5` | Convert heading error into turning speed. |
| `slow_down_distance` | `1.0 m` | Reduce forward speed near the goal. |
| `rotate_in_place_angle` | `0.7 rad` | Stop forward motion while correcting a large heading error. |
| `odom_timeout` | `0.5 s` | Stop proposing motion when localisation is stale. |

The follower proposes a zero command when the path is empty, odometry is missing or stale, frames do not match, or the goal is reached. The global planner publishes an empty path once when a new terrain map invalidates a previously published route. This prevents continued tracking of a stale route.

The follower is independent of `WHEEL`, `TRANSITION` and `HOVER` actuator behaviour. Person 2 retains final command authority and remains the only publisher of `/cmd_vel`; Person 4 translates the approved body-motion command into wheel, lift-fan and propulsion-fan behaviour.

LiDAR-based local obstacle response, live simulated-odometry tuning and Safety-requested return-path handling remain the next Role 1 milestones. Autonomy does not publish `/cmd_vel`.

## Three-day build plan

This plan assumes roughly 2.5–3 focused build days. If less time is available, preserve the milestone gates and reduce visual polish rather than skipping integration or safety behaviour.

### Stage 0 — Contract and environment lock

**Timing:** first 2–3 hours.
**Goal:** make every workstream independently productive without waiting for final assets.

All five people agree on the fixed mission:

```text
launch → cross shore/mud/shallow water → avoid roots or debris
→ deliver payload → return safely
```

They also confirm the three required scenarios:

1. Normal delivery and return.
2. Obstacle-induced reroute.
3. Rising tide changes terrain risk, forcing a replan, hold or return.

Work during this stage:

- **Person 1:** confirms planner inputs and `/cmd_vel_proposed` output.
- **Person 2:** writes initial safety thresholds and the transition table for `CRUISE`, `CAUTION`, `HOLD` and `RETURN`.
- **Person 3:** defines the world layout, start/goal coordinates, terrain zones and obstacle spawn locations.
- **Person 4:** confirms the ROS/Gazebo environment, creates the common launch path and validates the package dependency graph.
- **Person 5:** defines the metric schema, scenario checklist, Foxglove layout, dashboard fields and camera/storyboard requirements.

**Exit gate:** the topic contract in `docs/INTERFACES.md` is accepted, the expected demo host is known, and every role can run against mocked or placeholder inputs.

### Stage 1 — Minimal end-to-end spine

**Timing:** remainder of Day 1.
**Goal:** a crude but complete mission loop before visual detail.

- **Person 4:** creates a simple controllable vehicle, basic Gazebo world and `/cmd_vel` bridge. It must publish usable `/odom`, `/scan`, `/imu`, `/terrain_state` and `/vehicle_health`, even if some values are initially simple constants.
- **Person 3:** produces a blockout world with recognisable shore, mud, shallow-water and obstacle regions. Visual quality is secondary at this stage.
- **Person 1:** develops the planner against a static terrain grid and mocked obstacle feed; publishes `/planned_path` and `/cmd_vel_proposed`.
- **Person 2:** implements a minimal safety supervisor that passes safe proposed commands through and holds on a mocked critical fault.
- **Person 5:** builds an initial RViz/Foxglove view or dashboard mock fed from the agreed topics; creates the run-result record format.

**Exit gate:** one command launches Gazebo and the ROS graph; the vehicle can be manually moved through the world; telemetry is visible; a mocked mission goal produces a proposed route and a safety state.

### Stage 2 — First autonomous delivery

**Timing:** Day 2 morning.
**Goal:** complete the core autonomous mission in a static, low-risk world.

- **Person 1:** connects real simulated odometry and LiDAR to the planner, then produces a route to the delivery point and a return route.
- **Person 2:** connects safety to live vehicle health and verifies that only safety-approved commands reach the vehicle.
- **Person 3:** keeps terrain geometry stable and adds collision-relevant roots/debris needed by the first route.
- **Person 4:** integrates sensor frames, Gazebo bridge and launch configuration; resolves transforms, topic names and controller issues.
- **Person 5:** exposes planned path, pose, LiDAR obstacles, battery and safety state; records baseline metrics for the normal mission.

**Exit gate:** from a selected mission goal, the vehicle autonomously reaches the delivery point and returns in a static world without manual steering.

### Stage 3 — Mission credibility and fault response

**Timing:** Day 2 afternoon through Day 3 morning.
**Goal:** demonstrate why the system is useful in a tidal corridor rather than merely following waypoints.

- **Person 3:** adds final terrain materials, mangrove roots, debris, tide-risk zones and clearly visible delivery/return landmarks. Its tide manager must publish the updated terrain state and cost map.
- **Person 4:** implements or tunes the transparent mobility model: terrain-dependent speed, limited turning/braking, battery consumption and vehicle-health inputs. Keep these assumptions documented rather than presenting them as validated physics.
- **Person 1:** adds local obstacle avoidance and repeated route assessment against `/terrain_costmap` for the blocked-route and rising-tide scenarios.
- **Person 2:** adds return-reserve and tide-risk handling. A rising tide must transition from `CRUISE` through `CAUTION` to a visible replan, `HOLD` or `RETURN` with a clear rationale.
- **Person 5:** implements deterministic scenario triggers, result logging and the Foxglove mission layout. It also prepares a simple capability comparison graphic for wheeled rover, boat and air-cushion vehicle profiles.

**Exit gate:** all three required scenarios run from the common launcher:

1. normal delivery and return;
2. obstacle detected, route replanned and mission completed; and
3. rising tide updates the terrain cost map and causes a visible, safety-driven replan, hold or return.

### Stage 4 — Demonstration hardening

**Timing:** final half-day.
**Goal:** reliability, clarity and a backup plan.

- **Person 4:** freezes the launch path and protects a stable demo branch/tag. Only integration fixes enter after this point.
- **Person 1 and Person 2:** tune only issues that prevent the three scenarios from completing consistently; do not introduce new autonomy features.
- **Person 3:** makes small visual fixes that improve legibility from the demo camera; do not change route geometry without rerunning scenarios.
- **Person 5:** records a clean backup video of every scenario, finalises captions/metrics and runs the presentation sequence with the operator view.

**Exit gate:** each required scenario succeeds repeatedly from a clean launch, its outcome is visible on the operator interface, and a recorded fallback demonstration is available.

## Integration cadence

- Run an integration check at the end of Stage 0, twice during Stage 1, at the Stage 2 exit gate and before every Stage 3/4 demo run.
- Person 4 coordinates these checks; each owner fixes failures in their own package.
- Do not wait for final graphics before connecting live topics.
- New features after the Stage 3 exit gate require agreement from Person 4 and Person 5 that they do not endanger demo reliability or story clarity.

## Definition of done

A workstream change is complete only when it has a documented input/output contract, can run from the shared launch flow, and contributes to at least one of the three demo scenarios. The demonstrated claim remains:

> The team demonstrates autonomous route planning, obstacle response and safety-driven fallback under explicitly defined simulated terrain, tide and vehicle-health conditions.
