# Project execution guide

This repository is building a Gazebo and ROS 2 demonstration of an autonomous air-cushion logistics vehicle operating in a simulated tidal-mangrove corridor. The objective is a credible, repeatable demonstration of route planning, obstacle response and safety-driven fallback—not a claim of field-validated hovercraft physics.

## Operating assumptions

- Target stack: **ROS 2 Jazzy**, **Gazebo Harmonic** and `ros_gz`.
- The live demonstration must work from one documented launch path owned by the vehicle simulation and integration lead.
- The shared demo launch is `ros2 launch tidal_vehicle_bringup sim.launch.py`; it defaults to the tidal corridor with its dynamic tide manager. Use `headless:=true` for the WSL2/Foxglove demonstration path.
- The safety supervisor is the only component permitted to publish `/cmd_vel`.
- Topic and message changes must be made with the matching update to `docs/INTERFACES.md`.
- Do not commit local PDFs, generated `build/`, `install/`, `log/`, recordings or experiment outputs.

See `docs/ARCHITECTURE.md` and `docs/INTERFACES.md` for the current system boundary and contracts.

## Open requests between workstreams (agents: read before editing a listed package)

Each request below names an owner. **If you are a coding agent working in that owner's package, tell your user about the matching request before making unrelated changes there.** Apply it only if your user agrees. When it's done, or the owner declines it, delete the request here in the same commit and say which in the commit message. The numbers are the Version 2 simulation's stated assumptions (Person 4), not measured data.

### To Person 1 (Autonomy), `tidal_vehicle_autonomy`

1. **Obstacle inflation for Version 2.** `global_planner` still uses its 0.75 m default for `obstacle_inflation_radius_m`, which was sized for Version 1 (1.2 × 0.7 m), and `sim.launch.py` passes no value. Version 2 is 2.5 × 1.5 m, so its half-diagonal is √(1.25² + 0.75²) ≈ 1.46 m. **Request: 1.7 m** (half-diagonal plus 0.25 m clearance). With 0.5 m cells that is 4 cells. Check the corridor's narrowest passages still have a route. If none remains, 1.5 m (half-diagonal only) is the floor. Person 4 can pass the value per vehicle from `sim.launch.py` if you prefer to keep your default.
2. **LiDAR transform: no change needed.** The Version 2 LiDAR sits on a centre mast directly above `base_link` (x = 0, y = 0, z = 1.44 m), and `/scan` is published in `lidar_link`, which is axis-aligned with `base_link`. Your "LiDAR at the odometry position, facing forward" assumption is therefore exact in the horizontal plane. The mast height is already handled in `lidar_scan_node` (Person 4), which filters ground and self returns. Using TF instead is optional.
3. **Turning capability (information).** With the rudders and puff ports, Version 2 is designed to reach the follower's 1.0 rad/s limit in HOVER, including a pivot in place (see *Turning aids* below). This still has to be verified in Gazebo (Role 4, open item 8). Braking uses 320 N of reverse thrust (≈ 1 m/s²), so stopping from 0.8 m/s takes well under 1 m, far inside the 8 m obstacle range. Raising `max_linear_speed` to about 1.5 m/s would still stop within about 2 m.

### To Person 2 (Safety), `tidal_vehicle_safety`

Safety's per-metre energy figures should match the battery drain that `vehicle_mobility_node` actually reports on `/vehicle_health` (`config/vehicle_mobility_v2.yaml`: 10 kWh pack; idle 150 W; lift fan 2.5 kW; thrust 15 W per N; thrust = 50·v + 25·v² N; tracks 800 W).

4. **Hover return energy.** At your `hover_nominal_speed_mps` of 0.7 m/s the vehicle draws 150 + 2500 + 15 × 47 ≈ 3.36 kW, which is 1.33 Wh/m, or **0.0133 % per metre**. At the follower's 0.8 m/s it is 0.0121 %/m. `hover_energy_percent_per_m` is currently 0.05, about 3.8× the simulated drain. That makes Safety return or caution much earlier than the battery requires. **Request: 0.0133**, or about 0.02 if you want margin in this figure as well as in `return_margin_percent`. Your choice.
5. **Hovering in place still drains the battery.** The lift fan keeps running while stopped. A `HOLD` in HOVER costs about 2.65 kW, which is **0.44 % per minute**. If HOLD waits for a tide window, the return estimate should include that time.
6. **Track parameters are inactive for now.** Version 2 is hover-first: it deploys its tracks only for a sustained forward climb steeper than 15° on firm land, and `sim.launch.py` sets `track_mode_enabled: false` for both vehicles. If track mode is enabled later, use the Version 2 track figures: 150 + 0.6 × 2500 + 800 ≈ 2.45 kW at 1.0 m/s, which is **≈ 0.0068 %/m**, with `track_nominal_speed_mps` 1.0.

## Vehicle configuration and mobility modes

The demonstration vehicle is **Version 2**: a hovercraft-dominant amphibious vehicle with a complete retractable tracked undercarriage and controlled air-cushion load sharing (see *Vehicle Version 2* below). It is the default vehicle of the common launch. **Version 1**, the earlier 1.2 m vehicle with a retractable wheeled undercarriage, remains available as the fallback with `vehicle:=v1`. The mobility model must demonstrate understandable control behaviour without claiming validated propeller, skirt, wheel, track or hovercraft physics.

The vehicle controller uses three internal mobility modes:

- **HOVER** — the default, used over mud and shallow water. The tracks are retracted into the hull, the lift fan carries the full weight on the air cushion, and the rear propulsion fans execute the safety-approved forward and turning command.
- **TRACK** — used on firm land and on slopes. The tracks are deployed and execute the safety-approved command as a skid-steer drive. The lift fan provides a controlled share of the vehicle's weight (the *cushion load share*): about 0–20% on firm, level ground and up to about 60% on soft ground or slopes, lowering the tracks' ground pressure while they keep traction and braking. The tracks are never fully unloaded in TRACK mode.
- **TRANSITION** — used while changing between TRACK and HOVER. Horizontal motion stops, the tracks deploy or retract, the lift fan ramps up or down, and the controller waits until the vehicle is either hover-ready or settled and loaded on its tracks.

Mode selection is internal to Person 4's controller (`vehicle_mobility_node`, `gear: tracks`, `mode_policy: terrain_auto`). Version 2 is **hover-first**: it hovers across firm shore, mud and water, with its tracks retracted once the cushion is ready. It switches to TRACK only for a sustained (> 1.5 s) forward climb steeper than 15° on firm land. There the cushion carries 60% of the load while the tracks provide traction, and it returns to HOVER when the climb ends. Side tilt, stopping, pivoting, descending, mud and water do not deploy the tracks. The cost bands below still define route risk and Safety's energy estimates. See `docs/VEHICLE_SIMULATION.md` (*Mode selection*).

These mobility modes are separate from Person 2's safety states and must not replace or rename `CRUISE`, `CAUTION`, `HOLD` or `RETURN`. They do not introduce new external commands or topics. The existing command authority remains:

1. Person 1 publishes route and motion proposals through `/planned_path` and `/cmd_vel_proposed`; autonomy does not directly command track speeds, fan RPM or cushion load share.
2. Person 2 applies the existing safety and mission logic and remains the only publisher of `/cmd_vel`.
3. Person 4 consumes the safety-approved `/cmd_vel` and owns the internal track, lift-fan and propulsion-fan control, including command limits, fan ramping, cushion load sharing, hover-ready checks and mobility-mode transitions.

A track-to-hover transition must stop horizontal motion, ramp the lift fan, wait for hover-ready status, then retract the tracks before propulsion begins. A hover-to-track transition must stop horizontal motion, deploy the tracks, reduce lift in a controlled way to the target load share and confirm that the vehicle has settled onto its tracks before track motion begins. The safety-approved command remains authoritative in every mode. Any future topic or message change requires the matching update to `docs/INTERFACES.md`.

Safety's ground-mode return-energy parameters are `track_cost_max`, `track_energy_percent_per_m` and `track_nominal_speed_mps`. The common launch currently sets Safety's `track_mode_enabled` to false for both vehicles, because Version 2 is hover-first (see *Open requests between workstreams*, item 6, for the Version 2 track figures).

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
| Turning aids | Two slipstream rudders (±25°); four puff ports (0.24 × 0.12 m side vents, bow and stern), each with a sliding shutter that shows when it is open | A puff port gives ≈ 37 N (2·Cd·p·A at 0.81 kPa, Cd 0.8), so a bow–stern pair gives a ≈ 55 N·m yaw couple at any speed. Each open vent bleeds ≈ 0.85 m³/s of cushion air. The lift fan is assumed to have that flow margin. |
| Tracks | Two inboard tracks, 0.28 m wide, 1.40 m ground contact, 0.84 m gauge | Length-to-gauge ≈ 1.67, inside the 1.0–1.8 range where skid steering works well. Ground pressure ≈ 3.75 kPa with no cushion support and ≈ 1.5 kPa at 60% cushion load share. A standing person exerts roughly 15–25 kPa. They retract 0.25 m into hull wells. |
| Slope limits (working) | HOVER climbs ≤ 8°; TRACK ≤ 20° (15° ramp tested) | Above 8° it climbs on the tracks with cushion assistance. When stopped or pivoting on slopes above 2° it uses the tracks, because the cushion slides sideways. |
| Height and stability | Hull top 0.95 m, centre of mass 0.46 m above ground | Keeps centre-of-mass height to beam ≈ 0.31 so a compartmented skirt stays roll-stable on the cushion. The rest of the 1.5 m is the sensor mast. |

Conclusion: the dimensions are physically plausible for an electric, 300 kg-class craft, provided the hull stays low and the 1.5 m height is mostly the mast. Air-cushion-assisted tracked vehicles have been studied for soft terrain such as marsh and snow, but performance in Singapore mangrove mud needs physical testing.

Agreed Version 2 design decisions:

- **Demo vehicle.** Version 2 is the vehicle for the final demonstration, and Version 1 is the fallback (`vehicle:=v1`).
- **Payload.** Rated at 30 kg in a sealed box. It can carry up to about 80 kg at reduced performance: cushion pressure ≈ 1.0 kPa, lift power up about 26%, thrust-to-weight ≈ 0.12.
- **Tracks.** Two inboard tracks under the hull, inside the skirt footprint, retracting vertically into hull wells. The overall width stays 1.5 m.
- **Propulsion.** Two rear reversible ducted fans with rudders.
- **Turning aids.** In HOVER, the controller inside `hover::AirCushion` shares the yaw moment it needs in this order: the rudders first (effective at speed), then the puff ports (effective at any speed, including a pivot in place), then differential fan thrust. Spare puff-port force damps sideways drift. The aids are off in TRACK mode, where the tracks steer. They are internal to Person 4's vehicle model and add no ROS topics.
- **LiDAR.** On a mast at the vehicle's centre, directly above `base_link`, so its horizontal position matches the odometry position that Autonomy's obstacle projection assumes. The lift fan is offset forward.
- **Style.** The same as Version 1: olive hull, black skirt, orange payload box. The hull topsides flare out over the skirt and sit on a bolted skirt-attachment flange, so the hull and skirt read as one craft. The skirt is a neoprene bag with 96 overlapping, curved fingers.
- **Retraction.** In HOVER mode both tracks slide 0.25 m straight up into the hull wells, leaving their lowest point 0.22 m above the skirt bottom, so the cushion alone carries the vehicle. `renders/retract_comparison.png` shows both states, and the Gazebo check measures the retracted joint position (0.250 m) before propulsion starts.

Simulation risk retired: Gazebo Harmonic's `TrackController`/`TrackedVehicle` work with DART and the Bullet collision detector, which the air-cushion ray casts need. They also work on the corridor's inclined `demo_terrain` collision surface. The tracks drive at the commanded speed, pivot in place without drift and climb a 15° ramp. The `hover::AirCushion` plugin gained a `lift_share` input for TRACK-mode load sharing, reversible thrust and a yaw reserve. Version 1 is unchanged by these additions.

## Team roles

| Person | Role | Owns | Does not own |
| --- | --- | --- | --- |
| 1 | Autonomy lead | Global route planner, local obstacle response, replanning and proposed motion commands. | The final safety decision or complete-system launch. |
| 2 | Safety and mission lead | Mission progression, return-margin logic, state machine and safety-approved commands. | Gazebo vehicle physics or visual world assets. |
| 3 | Environment lead | Tidal corridor world, terrain zones, mangrove roots, debris, delivery area and tide-state visuals. | Navigation and safety policies. |
| 4 | Vehicle simulation and integration lead | Vehicle SDF model, sensors, mobility abstraction, ROS-Gazebo bridge, packages and one-command launch. | Mission-policy thresholds or dashboard narrative. |
| 5 | Operator, evaluation and demo lead | Operator display, scenario runner, fault injection controls, metrics, comparison visuals and presentation story. | Direct vehicle control outside the documented operator controls. |

### Remote-control fallback

The dashboard may expose a circular fallback remote-control panel for operator demonstrations and recovery requests. It is a safety-gated request interface, not a direct actuator controller. Directional requests (`forward`, `reverse`, `left`, `right`) and `STOP` must be represented as operator intent and routed through the Safety supervisor; the dashboard must never publish `/cmd_vel` directly. Safety remains the only `/cmd_vel` publisher and may reject, clamp, hold or override every request. The fallback is for cases where autonomous planning or the normal goal workflow is unavailable, and it must be clearly labelled as such, logged, resettable and tested against the same emergency-stop and return rules as autonomous operation.

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
| `obstacle_inflation_radius_m` | `0.75 m` | Cover the Version 1 vehicle's half-diagonal plus a small clearance; recalibrate for Version 2. |
| `obstacle_max_range_m` | `8.0 m` | Ignore detections beyond the useful local planning distance. |

For the first integration slice, the LiDAR is assumed to be located at the odometry position and aligned with the vehicle's forward direction. The 0.75 m circular inflation is based on Version 1's approximately 1.2 m by 0.7 m footprint plus a small clearance. Person 4's final collision geometry and sensor-frame transform must replace these assumptions when Version 2 is available.

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

The HOME values must match Person 2's safety parameters. The changing map-frame tide map is integrated through the shared launcher; Person 1 and Person 4 should tune the final sensor transform and route geometry against the corridor before the demo. Autonomy does not publish `/cmd_vel`.

## Role 4: Vehicle simulation and integration status

- **Version 2 delivered on `main` and is the launch default.** It includes the Blender model (visually polished to Version 1's standard, with the hull blended into the skirt), SDF/URDF, the mobility controller (`gear: tracks`), a bridge configuration and the test worlds described in `docs/VEHICLE_SIMULATION.md`. Launch with `ros2 launch tidal_vehicle_bringup sim.launch.py`, or add `vehicle:=v1` for the fallback.
- **Gazebo physics verified:** all 19 Version 2 checks pass (`tools/vehicle_tests/analyze.py`):
  - hover gap 5.05 cm held with the tracks retracted;
  - 2.0 m/s and 0.46 rad/s command tracking in HOVER;
  - 1.00 m/s and a 0.51 rad/s pivot with 5 mm drift in TRACK;
  - the 15° ramp climbed on the tracks;
  - exactly 60% cushion load share while resting on the tracks;
  - the full water → mud → bank sequence: hover, stop, deploy the tracks, lower the lift to 40%, then climb a 12° bank on the tracks.

  The Version 1 regression checks are unchanged.
- **ROS 2 integration verified** (headless, integration world with `tide:=false`). An autonomous goal across water and mud produced `delivery_confirmed` then `mission_complete` in 82 s. Version 2 now starts in HOVER at HOME and remains there across ordinary shore, mud and water; it deploys tracks only for a sustained forward firm-land climb above 15°. Safety remains the sole `/cmd_vel` publisher. The Version 1 fallback completes the same mission.
- **Tidal corridor** (the default world): it runs through `sim.launch.py` with DART/Bullet physics, sensors, buoyancy (Version 1 only) and terrain zones. The launch spawns the vehicle at map-frame HOME, and the tide manager is the sole dynamic `/terrain_state` and `/terrain_costmap` publisher. On the earlier heightmap collision Version 2 reached the delivery point. On the current `demo_terrain` surface it drives from HOME on its tracks but stalls at the channel edge (item 1 below), so the corridor mission is not yet verified end to end. `vehicle_tests/integration_test` remains the static regression world (`world:=vehicle_tests/integration_test tide:=false`).

Open integration items:

1. **Corridor support surface.** The simplified collision shore is deliberately flat. `TerrainZones`, the visual water sheet and `tide_manager` start together at `z=0.02 m` and rise by `0.55 m` over 30 simulated seconds. The rendered sheet covers the shared 80 x 60 m map for legibility, while the narrower `TerrainZones` channel remains the authoritative physical support and navigation-risk corridor. This prevents a sharp vertical water step at the channel entrance from overturning a hovering vehicle. Version 2 has no Gazebo buoyancy in this world: its air cushion is the authoritative water-support abstraction.
2. **Tide timing.** The default automatic rise is suitable for a short rising-tide demo. For a longer normal-delivery recording, use the existing `tide_hold` scenario event before issuing the goal, then trigger `tide_resume` or `tide_rise` for the rising-tide scenario.
3. **Rendering load.** The mangrove and rock meshes are heavy for the LiDAR and camera. With software rendering in a CPU-only container, the full ROS corridor stack ran at a real-time factor of 0.001–0.6 (the integration world runs near real time), so the corridor mission could not be timed there. Check the real-time factor on the demo laptop's GPU.
4. **Sensor transform.** Resolved: the Version 2 LiDAR is directly above `base_link`, so Autonomy's "LiDAR at odometry position" assumption is exact horizontally (*Open requests*, item 2).
5. Confirm the complete corridor mission repeatedly from the shared headless launch: camera, LiDAR, map-frame odometry, tide updates, replan and Safety fallback must all be visible in Foxglove.
6. Tune the corridor map rectangles, HOME/delivery coordinates and obstacle inflation so that the generated path matches the visibly safe route through the world.
7. **Obstacle inflation and Safety energy figures.** Sent to Person 1 and Person 2 as *Open requests between workstreams*, items 1 and 4–6. Once they land, re-check the corridor route and the return timing.
8. **Rudders and puff ports (new, not yet run in Gazebo).** Build the workspace, run all five Version 2 tests including the new `v2_turn_test`, and check that the 19 earlier checks still pass. Then rerun the integration-world mission.
9. Add Person 5's Foxglove layout: 3D scene, `/camera/image_raw`, planned and return paths, terrain-cost map, battery, safety reason and tide-window fields.

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

## Current tidal map setup

The active environment is launched from `autonomous_tidal_vehicle_ws` with `tidal_vehicle_simulation`. The map currently contains:

- An approximately 80 m x 80 m inclined terrain heightmap with mudflat materials.
- A visual-only water surface that rises in parallel with the terrain. It extends beyond the terrain edges so its perimeter is not visible and has no collision geometry. The configured rise is 2.4 m over 30 seconds, leaving higher land visible at high tide.
- Nineteen Mangrovetree GLB-derived mangroves, 7-15 m tall, placed in a dense irregular layout. Their trunks, roots and branches have collision meshes for physics and LiDAR; foliage is represented by the visual GLB.
- Eighteen Rock.glb instances placed as scattered obstacles. The rock visuals are enlarged and each has a larger primitive collision shape for stable physics and LiDAR detection.
- The former red reference box has been removed.

Water remains visual-only, so it can overlap the tree and rock collision objects without colliding with them. Do not remove collision elements from tree or rock models: Gazebo ray sensors detect collision shapes, not visual meshes.

### Run the current map

```bash
cd /home/tiffy/One-Wish-Coders/autonomous_tidal_vehicle_ws
source /opt/ros/jazzy/setup.bash
pkill -f '^gz sim server$' || true
colcon build --symlink-install --packages-select tidal_vehicle_simulation
source install/setup.bash
ros2 launch tidal_vehicle_simulation environment.launch.py
```

For a quick headless validation, replace the final command with:

```bash
timeout 12s ros2 launch tidal_vehicle_simulation environment.launch.py
```

The authoritative world is `src/tidal_vehicle_simulation/worlds/tidal_corridor.sdf`. Environment assets are under `src/tidal_vehicle_simulation/models/`, including `mangrove_imported/` and `rock_imported/`.
