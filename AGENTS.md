# Project execution guide

This repository is building a Gazebo and ROS 2 demonstration of an autonomous air-cushion logistics vehicle operating in a simulated tidal-mangrove corridor. The objective is a credible, repeatable demonstration of route planning, obstacle response and safety-driven fallback—not a claim of field-validated hovercraft physics.

## Operating assumptions

- Target stack: **ROS 2 Jazzy**, **Gazebo Harmonic** and `ros_gz`.
- The live demonstration must work from one documented launch path owned by the vehicle simulation and integration lead.
- The safety supervisor is the only component permitted to publish `/cmd_vel`.
- Topic and message changes must be made with the matching update to `docs/INTERFACES.md`.
- Do not commit local PDFs, generated `build/`, `install/`, `log/`, recordings or experiment outputs.

See `docs/ARCHITECTURE.md` and `docs/INTERFACES.md` for the current system boundary and contracts.

## Team roles

| Person | Role | Owns | Does not own |
| --- | --- | --- | --- |
| 1 | Autonomy lead | Global route planner, local obstacle response, replanning and proposed motion commands. | The final safety decision or complete-system launch. |
| 2 | Safety and mission lead | Mission progression, return-margin logic, state machine and safety-approved commands. | Gazebo vehicle physics or visual world assets. |
| 3 | Environment lead | Tidal corridor world, terrain zones, mangrove roots, debris, delivery area and tide-state visuals. | Navigation and safety policies. |
| 4 | Vehicle simulation and integration lead | Vehicle SDF model, sensors, mobility abstraction, ROS-Gazebo bridge, packages and one-command launch. | Mission-policy thresholds or dashboard narrative. |
| 5 | Operator, evaluation and demo lead | Operator display, scenario runner, fault injection controls, metrics, comparison visuals and presentation story. | Direct vehicle control outside the documented operator controls. |

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
3. Unsafe condition causing a hold or return.

Work during this stage:

- **Person 1:** confirms planner inputs and `/cmd_vel_proposed` output.
- **Person 2:** writes initial safety thresholds and the transition table for `CRUISE`, `CAUTION`, `HOLD` and `RETURN`.
- **Person 3:** defines the world layout, start/goal coordinates, terrain zones and obstacle spawn locations.
- **Person 4:** confirms the ROS/Gazebo environment, creates the common launch path and validates the package dependency graph.
- **Person 5:** defines the metric schema, scenario checklist, dashboard fields and camera/storyboard requirements.

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

- **Person 3:** adds final terrain materials, mangrove roots, debris, tide-risk zones and clearly visible delivery/return landmarks.
- **Person 4:** implements or tunes the transparent mobility model: terrain-dependent speed, limited turning/braking, battery consumption and vehicle-health inputs. Keep these assumptions documented rather than presenting them as validated physics.
- **Person 1:** adds local obstacle avoidance and route replanning for the blocked-route scenario.
- **Person 2:** adds return-reserve, tide-risk and fault handling. At least one condition must transition from `CRUISE` through `CAUTION` to `HOLD` or `RETURN` with a clear rationale.
- **Person 5:** implements deterministic scenario triggers, result logging and the live mission overlay. It also prepares a simple capability comparison graphic for wheeled rover, boat and air-cushion vehicle profiles.

**Exit gate:** all three required scenarios run from the common launcher:

1. normal delivery and return;
2. obstacle detected, route replanned and mission completed; and
3. an unsafe condition causes a visible, safety-driven hold or return.

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
