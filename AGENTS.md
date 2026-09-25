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

The simulated vehicle is a **hovercraft-dominant amphibious vehicle with a complete retractable tracked undercarriage and controlled air-cushion load sharing** (Vehicle Version 2, below). It has an inflatable, segmented air-cushion skirt, a lift fan, two rear ducted propulsion fans with rudders and two retractable rubber tracks. Its working size is 2.5 m long, 1.5 m wide and 1.5 m high overall, which matches the planning footprint Autonomy already uses; Person 4's final collision geometry remains authoritative. The mobility model must demonstrate understandable control behaviour without claiming validated propeller, skirt, track or hovercraft physics.

The vehicle controller uses three internal mobility modes:

- **HOVER** — the default, used over mud and shallow water. The tracks are retracted into the hull, the lift fan carries the full weight on the air cushion, and the rear propulsion fans execute the safety-approved forward and turning command.
- **TRACK** — used on firm land and on slopes. The tracks are deployed and execute the safety-approved command as a skid-steer drive. The lift fan provides a controlled share of the vehicle's weight (the *cushion load share*): about 0–20% on firm, level ground and up to about 60% on soft ground or slopes, lowering the tracks' ground pressure while they keep traction and braking. The tracks are never fully unloaded in TRACK mode.
- **TRANSITION** — used while changing between TRACK and HOVER. Horizontal motion stops, the tracks deploy or retract, the lift fan ramps up or down, and the controller waits until the vehicle is either hover-ready or settled and loaded on its tracks.

Mode selection is internal to Person 4's controller (`vehicle_mobility_node`, `gear: tracks`). It reads the `/terrain_costmap` bands from the vehicle's position to 2.5 m ahead along its heading, so it changes mode *before* reaching new terrain. It switches to TRACK on firm land (cost `0`--`19`, after a 3 s dwell) or on a climbing or side slope steeper than the hover slope limit (8°, sustained for 1.5 s so edges and bumps don't trigger it). It switches to HOVER when hover terrain (cost `20`--`89`) lies ahead. Two slope rules use the tracks where a hovercraft would slide. When the vehicle is stopped for 5 s on a slope steeper than 2°, it parks on its tracks. When it is asked to pivot in place on such a slope, it pivots on its tracks. Neither applies over water, where the surface is level. It never lifts off while tilted.

These mobility modes are separate from Person 2's safety states and must not replace or rename `CRUISE`, `CAUTION`, `HOLD` or `RETURN`. They do not introduce new external commands or topics. The existing command authority remains:

1. Person 1 publishes route and motion proposals through `/planned_path` and `/cmd_vel_proposed`; autonomy does not directly command track speeds, fan RPM or cushion load share.
2. Person 2 applies the existing safety and mission logic and remains the only publisher of `/cmd_vel`.
3. Person 4 consumes the safety-approved `/cmd_vel` and owns the internal track, lift-fan and propulsion-fan control, including command limits, fan ramping, cushion load sharing, hover-ready checks and mobility-mode transitions.

A track-to-hover transition must stop horizontal motion, ramp the lift fan, wait for hover-ready status, then retract the tracks before propulsion begins. A hover-to-track transition must stop horizontal motion, deploy the tracks, reduce lift in a controlled way to the target load share and confirm that the vehicle has settled onto its tracks before track motion begins. The safety-approved command remains authoritative in every mode. Any future topic or message change requires the matching update to `docs/INTERFACES.md`.

Safety's ground-mode return-energy parameters are `track_cost_max`, `track_energy_percent_per_m` and `track_nominal_speed_mps`. The common launch sets Safety's `track_mode_enabled` to true for `vehicle:=v2` and false for the Version 1 fallback, which runs `hover_only`.

### Shared terrain-cost semantics

The terrain cost map supplies the shared route and mobility interpretation:

- `0`--`19`: firm shore; the controller uses **TRACK** mode.
- `20`--`59`: mud or shallow water; the controller uses **HOVER** mode (TRACK with a high cushion load share on slopes steeper than the hover limit).
- `60`--`89`: elevated-risk mud or shallow water; the controller uses **HOVER** mode with conservative speed/energy assumptions.
- `90`--`100`: no-go terrain.
- `-1`: unknown terrain; treated as no-go.

Autonomy plans with these costs and Safety estimates the return energy/time using
the same bands. A path that crosses a `90+` or `-1` cell is invalid.

### Vehicle Version 2 (the demo vehicle)

Version 2 is built and is the default vehicle of the common launch (`vehicle:=v2`). Its Blender source is `assets/vehicle_blender/version_2/build_vehicle.py`. The pipeline is the same as Version 1's: Blender script → `link_frames.json` → `gen_description_v2.py` → `models/hovercraft_v2/model.sdf` and `urdf/hovercraft_v2.urdf`. It uses the same frame names (`base_link`, `lidar_link`, `camera_link`, `imu_link`) and the same public topics as Version 1. Version 1 is frozen in `assets/vehicle_blender/version_1/` (1.2 × 0.7 m, 25 kg, four wheels on swing-up legs) and stays available as the fallback with `vehicle:=v1`.

Version 2 sizing. These are stated design assumptions, not validated data. The geometric values are the built model's, checked in `assets/vehicle_blender/version_2/renders/dimensions.txt`:

| Quantity | Working value | Basis |
| --- | --- | --- |
| Overall size | 2.5 m L × 1.5 m W × 1.5 m H | Team working dimensions; the height includes the LiDAR mast. |
| Design mass | ≈ 300 kg including a 30 kg payload | Composite hull and skirt ≈ 70 kg, track undercarriage ≈ 70 kg, fans and motors ≈ 50 kg, 10 kWh battery ≈ 65 kg, electronics ≈ 15 kg. |
| Cushion pressure | ≈ 0.81 kPa over ≈ 3.6 m² of cushion | Within the usual 0.5–1.5 kPa range for light hovercraft. |
| Lift power | ≈ 2–3 kW electrical | Air escaping under an 8 m skirt perimeter with a 6 mm effective gap, 50% fan-and-motor efficiency. |
| Propulsion | Two reversible ducted fans, ≈ 200 N each, 80% reverse thrust | Thrust-to-weight ≈ 0.14. Reverse thrust of 320 N brakes the vehicle and holds it on slopes up to ≈ 6°. |
| Tracks | Two inboard tracks, 0.28 m wide, 1.40 m ground contact, 0.84 m gauge | Length-to-gauge ≈ 1.67, inside the 1.0–1.8 range where skid steering works well. Ground pressure ≈ 3.75 kPa with no cushion support and ≈ 1.5 kPa at 60% cushion load share. A standing person exerts roughly 15–25 kPa. They retract 0.25 m into hull wells. |
| Slope limits (working) | HOVER climbs ≤ 8°; TRACK ≤ 20° (15° ramp tested) | Above 8° it climbs on the tracks with cushion assistance. When stopped or pivoting on slopes above 2° it uses the tracks, because the cushion slides sideways. |
| Height and stability | Hull top 0.95 m, centre of mass 0.46 m above ground | Keeps centre-of-mass height to beam ≈ 0.31 so a compartmented skirt stays roll-stable on the cushion. The rest of the 1.5 m is the sensor mast. |

Conclusion: the dimensions are physically plausible for an electric, 300 kg-class craft, provided the hull stays low and the 1.5 m height is mostly the mast. Air-cushion-assisted tracked vehicles have been studied for soft terrain such as marsh and snow, but performance in Singapore mangrove mud needs physical testing.

Agreed Version 2 design decisions:

- **Demo vehicle.** Version 2 is the vehicle for the final demonstration, and Version 1 is the fallback (`vehicle:=v1`).
- **Payload.** Rated at 30 kg in a sealed box. It can carry up to about 80 kg at reduced performance: cushion pressure ≈ 1.0 kPa, lift power up about 26%, thrust-to-weight ≈ 0.12.
- **Tracks.** Two inboard tracks under the hull, inside the skirt footprint, retracting vertically into hull wells. The overall width stays 1.5 m.
- **Propulsion.** Two rear reversible ducted fans with rudders.
- **LiDAR.** On a mast at the vehicle's centre, directly above `base_link`, so its horizontal position matches the odometry position that Autonomy's obstacle projection assumes. The lift fan is offset forward.
- **Style.** The same as Version 1: olive hull, black skirt, orange payload box.

Simulation risk retired: Gazebo Harmonic's `TrackController`/`TrackedVehicle` work with DART and the Bullet collision detector, which the air-cushion ray casts need. They also work with the corridor's heightmap. The tracks drive at the commanded speed, pivot in place without drift and climb a 15° ramp. The `hover::AirCushion` plugin gained a `lift_share` input for TRACK-mode load sharing, reversible thrust and a yaw reserve. Version 1 is unchanged by these additions.

## Team roles

| Person | Role | Owns | Does not own |
| --- | --- | --- | --- |
| 1 | Autonomy lead | Global route planner, local obstacle response, replanning and proposed motion commands. | The final safety decision or complete-system launch. |
| 2 | Safety and mission lead | Mission progression, return-margin logic, state machine and safety-approved commands. | Gazebo vehicle physics or visual world assets. |
| 3 | Environment lead | Tidal corridor world, terrain zones, mangrove roots, debris, delivery area and tide-state visuals. | Navigation and safety policies. |
| 4 | Vehicle simulation and integration lead | Vehicle SDF model, sensors, mobility abstraction, ROS-Gazebo bridge, packages and one-command launch. | Mission-policy thresholds or dashboard narrative. |
| 5 | Operator, evaluation and demo lead | Operator display, scenario runner, fault injection controls, metrics, comparison visuals and presentation story. | Direct vehicle control outside the documented operator controls. |

## Role 1: Autonomy implementation status

- Implemented `global_planner`, which consumes `/terrain_costmap`, `/odom`, `/mission_goal`, `/terrain_state`, `/scan`, `/safety_status` and reset scenario events. It publishes the active `/planned_path`, Safety's `/return_path` and lifecycle `/mission_event` messages.
- Implemented a ROS-independent, eight-connected A* core that minimises distance and terrain risk.
- Agreed planner interpretation of `/terrain_costmap`: `0`--`19` firm-ground (TRACK) terrain, `20`--`59` hover terrain, `60`--`89` elevated-risk hover terrain, `90`--`100` no-go, and `-1` unknown/no-go.
- The planner replans after cost-map, goal or terrain-state updates, refuses mismatched frames, and publishes an empty path when a previously valid route becomes unsafe.
- Added LiDAR obstacle projection: valid `/scan` returns become inflated blocked cells in an internal planning overlay, and a changed scan triggers route reassessment without modifying Simulation's terrain map.
- Implemented `path_follower`, which consumes `/planned_path` and `/odom` and publishes forward and turning proposals on `/cmd_vel_proposed` at 10 Hz.
- The follower uses lookahead steering, slows near the goal, stops to correct large heading errors, and proposes zero motion for empty paths, stale odometry or mismatched frames.
- Added seventeen ROS-independent planner, follower and LiDAR tests and five ROS topic integration tests.
- Restored the autonomy package's `ament_python` build-tool declaration and verified the full eight-package workspace build.
- Verified live simulated odometry and LiDAR integration. The full launcher completed an autonomous delivery and HOME return while Safety remained the only `/cmd_vel` publisher.

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

The follower is independent of `TRACK` (Version 1: `WHEEL`), `TRANSITION` and `HOVER` actuator behaviour. Person 2 retains final command authority and remains the only publisher of `/cmd_vel`; Person 4 translates the approved body-motion command into wheel, lift-fan and propulsion-fan behaviour.

### LiDAR obstacle response implementation details

The global planner keeps Simulation's `/terrain_costmap` unchanged as its base map. Each `/scan` update is converted from polar range measurements into world coordinates using the vehicle pose from `/odom`, then into terrain-grid cells. Invalid, infinite, too-near and over-range measurements are ignored. Detected cells are expanded by the configured safety radius and marked no-go only in an internal copy used by A*.

Each accepted scan replaces the previous dynamic obstacle set. When that set changes, the planner immediately rechecks the route. It publishes a detour when one exists and publishes an empty `/planned_path` if the obstacle removes every safe route. A later clear scan removes the temporary cells and allows the direct terrain route to return.

Default LiDAR parameters are:

| Parameter | Default | Purpose |
| --- | ---: | --- |
| `obstacle_inflation_radius_m` | `1.7 m` | Cover the estimated half-diagonal of the vehicle plus about 0.25 m clearance. |
| `obstacle_max_range_m` | `8.0 m` | Ignore detections beyond the useful local planning distance. |

For the first integration slice, the LiDAR is assumed to be located at the odometry position and aligned with the vehicle's forward direction. The 1.7 m circular inflation is based on an approximately 2.5 m by 1.5 m vehicle footprint plus about 0.25 m clearance. Person 4's final collision geometry and sensor-frame transform must replace these assumptions when available.

### Return mission implementation details

During outbound travel, the planner publishes a prospective route from the current pose to HOME on `/return_path` so Safety can calculate return energy and time before allowing departure. A `return_required=true` message on `/safety_status` then latches return mode. The planner immediately invalidates the outbound route, publishes a fresh HOME route on `/return_path`, and publishes the same route on `/planned_path` for the path follower.

Every terrain cost-map update forces a newly stamped route publication, even when A* selects the same cells. A return-only 2 Hz refresh prevents callback ordering from making Safety treat that route as old, without resetting the path follower. Later `return_required=false` messages do not cancel the latch. The existing `reset` scenario event clears the mission and routes, then Autonomy publishes `mission_reset`.

Autonomy publishes `delivery_confirmed` when odometry reaches the outbound path endpoint and `mission_complete` when it reaches the HOME path endpoint. Empty active and return paths are published at completion so the path follower proposes a stop while Safety resets to `PRELAUNCH`.

Default return parameters are:

| Parameter | Default | Purpose |
| --- | ---: | --- |
| `home_x_m` | `0.0 m` | HOME position in the configured frame. |
| `home_y_m` | `0.0 m` | HOME position in the configured frame. |
| `home_frame` | `map` | Frame containing HOME. |
| `goal_event_tolerance_m` | `0.3 m` | Distance from a path endpoint that triggers a mission event. |
| `return_path_refresh_rate_hz` | `2.0 Hz` | Keep Safety's validated return route fresh without resetting the follower. |

The HOME values must match Person 2's safety parameters. The final sensor transform and Person 3's changing tide map still need integration. Autonomy does not publish `/cmd_vel`.

## Role 4: Vehicle simulation and integration status

- **Version 2 delivered on `main` and is the launch default.** It includes the Blender model, SDF/URDF, the mobility controller (`gear: tracks`), a bridge configuration and the test worlds described in `docs/VEHICLE_SIMULATION.md`. Launch with `ros2 launch tidal_vehicle_bringup sim.launch.py`, or add `vehicle:=v1` for the fallback.
- **Gazebo physics verified:** all 19 Version 2 checks pass (`tools/vehicle_tests/analyze.py`):
  - hover gap 5.05 cm held with the tracks retracted;
  - 2.0 m/s and 0.46 rad/s command tracking in HOVER;
  - 1.00 m/s and a 0.51 rad/s pivot with 5 mm drift in TRACK;
  - the 15° ramp climbed on the tracks;
  - exactly 60% cushion load share while resting on the tracks;
  - the full water → mud → bank sequence: hover, stop, deploy the tracks, lower the lift to 40%, then climb a 12° bank on the tracks.

  The Version 1 regression checks are unchanged.
- **ROS 2 integration verified** (headless, integration world with `tide:=false`). An autonomous goal across water and mud produced `delivery_confirmed` then `mission_complete`. Version 2 started on its tracks at HOME, switched to HOVER 2.5 m before the water, and returned to TRACK at HOME. Safety remained the sole `/cmd_vel` publisher. The Version 1 fallback completes the same mission.
- **Tidal corridor:** Version 2 spawns at HOME, parks on its tracks, crosses the channel in HOVER and reaches delivery. Open issues are listed below.

Open integration items:

1. **Corridor incline.** The corridor terrain is tilted 4.6° (`fixed_terrain` pitch 0.08 rad). On the return leg Version 2 creeps downhill in TRACK mode even with its tracks stopped, and it takes about 3 m to stop from cruise before a pivot. The track/skirt contact with the heightmap needs investigating. Version 1 does not reach the delivery point on this terrain at all. A flatter shore (less than about 2°) would suit both vehicles.
2. **Tide timing.** `scenario_defaults.yaml` floods the corridor in 30 s of sim time, so a normal delivery cannot finish before the tide cuts off HOME. Scenario 1 needs the tide held, or a slower rise, with the rising tide triggered for scenario 3.
3. **Rendering load.** The mangrove and rock meshes are heavy for the LiDAR and camera. Check the real-time factor on the demo laptop's GPU.
4. **Sensor transform.** Autonomy should use the Version 2 LiDAR transform (centre mast, 1.44 m) once the shared TF work lands. Until then its "LiDAR at odometry position" assumption holds for Version 2.

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
