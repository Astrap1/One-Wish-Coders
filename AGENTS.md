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

2. **LiDAR transform: no change needed.** The Version 2 LiDAR sits on a centre mast directly above `base_link` (x = 0, y = 0, z = 1.44 m), and `/scan` is published in `lidar_link`, which is axis-aligned with `base_link`. Your "LiDAR at the odometry position, facing forward" assumption is therefore exact in the horizontal plane. The mast height is already handled in `lidar_scan_node` (Person 4), which filters ground and self returns. Using TF instead is optional.
3. **Turning capability (information, verified 2026-09-26).** With the rudders and puff ports on, Version 2 turns at 0.85 rad/s at 1.5 m/s (fans only: 0.37) and pivots in place at 0.81 rad/s (fans only: 0.57), so your 0.45 rad/s follower limit can be raised if you want. Three clean integration-world missions completed in 68–70 s, with your target-cell arrival fix. Braking uses 320 N of reverse thrust (about 1 m/s²), so stopping from 0.8 m/s takes well under 1 m.

### To Person 2 (Safety), `tidal_vehicle_safety`

Safety's per-metre energy figures should match the battery drain that `vehicle_mobility_node` actually reports on `/vehicle_health` (`config/vehicle_mobility_v2.yaml`: 10 kWh pack; idle 150 W; lift fan 2.5 kW; thrust 15 W per N; thrust = 50·v + 25·v² N; tracks 800 W).

4. **Hover return energy.** At your `hover_nominal_speed_mps` of 0.7 m/s the vehicle draws 150 + 2500 + 15 × 47 ≈ 3.36 kW, which is 1.33 Wh/m, or **0.0133 % per metre**. At the follower's 0.8 m/s it is 0.0121 %/m. `hover_energy_percent_per_m` is currently 0.05, about 3.8× the simulated drain. That makes Safety return or caution much earlier than the battery requires. **Request: 0.0133**, or about 0.02 if you want margin in this figure as well as in `return_margin_percent`. Your choice.
5. **Hovering in place still drains the battery.** The lift fan keeps running while stopped. A `HOLD` in HOVER costs about 2.65 kW, which is **0.44 % per minute**. If HOLD waits for a tide window, the return estimate should include that time.
6. **Track parameters are inactive for now.** Version 2 is hover-first: it deploys its tracks only for a sustained forward climb at or above 14° on firm land (a guard band for the physical 15° bank), and `sim.launch.py` sets `track_mode_enabled: false` for both vehicles. If track mode is enabled later, use the Version 2 track figures: 150 + 0.6 × 2500 + 800 ≈ 2.45 kW at 1.0 m/s, which is **≈ 0.0068 %/m**, with `track_nominal_speed_mps` 1.0.

### Version 3 requests (plan ahead: not needed for the Version 2 demo)

Version 3 is being designed for 30 km/h cruise and 50 km/h max over open water (see *Vehicle Version 3*). These requests prepare each workstream for it. **None of them may slow or destabilise the Version 2 demo.** Keep Version 2 behaviour unchanged by default, for example behind a parameter or `vehicle:=v3`. All speed limits follow the split cost bands in *Shared terrain-cost semantics*.

**All workstreams**
7. **Cost-band split (agreed).** `20`--`29` = open, surveyed water where fast travel is allowed; `30`--`59` = mud, shallow water, roots and debris. Band checks at `19`, `60` and `90` are unchanged. Don't reuse `20`--`29` for anything else.

**NOTICE to Person 1 (Autonomy): changes made in your package on 2026-09-26 by Person 4, at the team's request.** The split cost band now works end to end. Please review them. They are **off by default**: Versions 1 and 2 behave exactly as before, and only `vehicle:=v3` switches them on (`sim.launch.py`, `VEHICLES["v3"]["follower"]`).
8. **Done: speed by zone.** `path_follower` has a new parameter, `zone_speed_limits_mps` (`[firm, open water, mud/roots/debris, elevated]`, `[0.0]` = off). When it's on, the follower subscribes to `/terrain_costmap`, samples the planned path from the vehicle out to its stopping distance (1 s reaction plus braking at `brake_decel_mps2`), and caps `max_linear_speed` at the slowest zone on that stretch, so it slows *before* a slower zone. New helpers are in `path_follower_core.py` (`zone_speed_limit`, `path_points_ahead`, `stopping_reach`), with tests in `test/test_speed_limits.py`.
9. **Done: slow for curves.** `lateral_accel_limit_mps2` (0 = off) caps speed at √(a/κ), using the pure-pursuit curvature κ = 2 sin(error) / lookahead (`curvature_speed_limit`). Version 3 uses a conservative 0.7 m/s² after corridor steering tests.

**NOTICE to Person 2 (Safety): changes made in your package on 2026-09-26 by Person 4, at the team's request.** Please review them. They are **off by default** and switched on only for `vehicle:=v3` (`VEHICLES["v3"]["safety"]`).
11. **Done: zone speed limits.** `safety_supervisor` has new parameters, `zone_speed_limits_mps` (`[0.0]` = off) and `brake_decel_mps2`. When they're on, it subscribes to `/odom` and clamps both autonomous and remote `/cmd_vel` to the slowest zone between the vehicle and its stopping distance (`zone_limits.py`, tests in `test/test_zone_limits.py`). For Version 3 the launch also raises your state limits: CRUISE 13.9 m/s, CAUTION 2.8 m/s, RETURN 8.3 m/s. **Still yours:** HOLD when a LiDAR obstacle is closer than the stopping distance at the current speed.
12. **Hybrid energy: still yours.** `/vehicle_health.fuel_percent` now exists: Version 3 reports diesel left, and Versions 1 and 2 report −1. On Version 3 the generator keeps `battery_percent` near 100%, so return estimates should use fuel. The first figure is about 2.5 L/h at 30 km/h on a 30 L tank; see `config/vehicle_mobility_v3.yaml` for the power model.

**Person 3 (Environment), `tidal_vehicle_simulation` worlds and `tide_manager`**
13. **Mark open, surveyed water.** Publish cost `20`--`29` only for water that is surveyed and free of roots and debris. Keep other water at `30`--`59`. Rising tide or new debris must move cells back to `30`+.
14. **Optional:** a long open-water stretch (≥ 150 m) so a Version 3 demo can show speed. Person 4 will build a separate Version 3 speed test world regardless.
18. **Done: conservative corridor obstacle rasterization (2026-09-26).** The physical radii already matched Gazebo's tree and rock collision proxies, but the 1 m map previously marked a cell only when its centre was inside a proxy. `tide_manager` now uses an exact circle-versus-cell test, so every grid cell touched by a collision proxy is cost `100`. Person 1 separately inflates those static no-go cells by the selected vehicle clearance before A*. A live V3 rerun produced a valid 107-cell HOME-to-delivery route before the dynamic overlay, confirming both fixes. V3's separate scan faults were subsequently fixed with inclusive body bounds, a tested 1.85 m radial self mask and a higher obstacle-height cutoff that rejects terrain/slope returns while retaining the mapped rock and mangrove collision geometry.

**Person 5 (Operator), `tidal_vehicle_operator`**
15. **Fuel and speed display.** `VehicleHealth.fuel_percent` now exists (Version 3: diesel left; Versions 1 and 2: −1, meaning no fuel tank). Please feed it to the dashboard's Fuel card, and show speed against the current zone limit.
16. **Extra cameras and collisions on the dashboard.** Version 3 now publishes `/camera/rear/image_raw`, `/camera/left/image_raw` and `/camera/right/image_raw` (with `cameras:=all`) and `/vehicle/collision` (item 17). All are in `docs/INTERFACES.md`. Please show them, for example with a camera selector and a collision alert.

**Collision detection (Version 3; new topic)**
17. **`/vehicle/collision` exists** (`tidal_vehicle_interfaces/Collision`, published by Person 4's `collision_monitor` on Version 3). It reports the source (`contact` or `imu`), the part (`hull`, `skirt`, `track_left`, `track_right`, or `unknown` for IMU), the side (front, rear, left or right), what was hit if known, and a strength. Ground contact is filtered out, and each hit is reported at most once per second. **Person 2:** please HOLD on a collision (then RETURN if the vehicle is still healthy), with a reason such as "collision: skirt front-left". **Person 1** (optional): mark the hit location as blocked in the planning overlay. **Person 5:** see item 16.

## Vehicle configuration and mobility modes

The demonstration vehicle is **Version 2**: a hovercraft-dominant amphibious vehicle with a complete retractable tracked undercarriage and controlled air-cushion load sharing (see *Vehicle Version 2* below). It is the default vehicle of the common launch. **Version 1**, the earlier 1.2 m vehicle with a retractable wheeled undercarriage, remains available as the fallback with `vehicle:=v1`. The mobility model must demonstrate understandable control behaviour without claiming validated propeller, skirt, wheel, track or hovercraft physics.

The vehicle controller uses three internal mobility modes:

- **HOVER** — the default, used over mud and shallow water. The tracks are retracted into the hull, the lift fan carries the full weight on the air cushion, and the rear propulsion fans execute the safety-approved forward and turning command.
- **TRACK** — used on firm land and on slopes. The tracks are deployed and execute the safety-approved command as a skid-steer drive. The lift fan provides a controlled share of the vehicle's weight (the *cushion load share*): about 0–20% on firm, level ground and up to about 60% on soft ground or slopes, lowering the tracks' ground pressure while they keep traction and braking. The tracks are never fully unloaded in TRACK mode.
- **TRANSITION** — used while changing between TRACK and HOVER. Horizontal motion stops, the tracks deploy or retract, the lift fan ramps up or down, and the controller waits until the vehicle is either hover-ready or settled and loaded on its tracks.

Mode selection is internal to Person 4's controller (`vehicle_mobility_node`, `gear: tracks`, `mode_policy: terrain_auto`). Version 2 is **hover-first**: it hovers across firm shore, mud and water, with its tracks retracted once the cushion is ready. It switches to TRACK only for a sustained (> 1.5 s) forward climb at or above 14° on firm land. This 1° guard band prepares it for the physical 15° bank. There the cushion carries 60% of the load while the tracks provide traction, and it returns to HOVER when the climb ends. Side tilt, stopping, pivoting, descending, mud and water do not deploy the tracks. The cost bands below still define route risk and Safety's energy estimates. See `docs/VEHICLE_SIMULATION.md` (*Mode selection*).

These mobility modes are separate from Person 2's safety states and must not replace or rename `CRUISE`, `CAUTION`, `HOLD` or `RETURN`. They do not introduce new external commands or topics. The existing command authority remains:

1. Person 1 publishes route and motion proposals through `/planned_path` and `/cmd_vel_proposed`; autonomy does not directly command track speeds, fan RPM or cushion load share.
2. Person 2 applies the existing safety and mission logic and remains the only publisher of `/cmd_vel`.
3. Person 4 consumes the safety-approved `/cmd_vel` and owns the internal track, lift-fan and propulsion-fan control, including command limits, fan ramping, cushion load sharing, hover-ready checks and mobility-mode transitions.

A track-to-hover transition must stop horizontal motion, ramp the lift fan, wait for hover-ready status, then retract the tracks before propulsion begins. A hover-to-track transition must stop horizontal motion, deploy the tracks, reduce lift in a controlled way to the target load share and confirm that the vehicle has settled onto its tracks before track motion begins. The safety-approved command remains authoritative in every mode. Any future topic or message change requires the matching update to `docs/INTERFACES.md`.

Safety's ground-mode return-energy parameters are `track_cost_max`, `track_energy_percent_per_m` and `track_nominal_speed_mps`. The common launch currently sets Safety's `track_mode_enabled` to false for both vehicles, because Version 2 is hover-first (see *Open requests between workstreams*, item 6, for the Version 2 track figures).

### Shared terrain-cost semantics

The terrain cost map supplies the shared route and mobility interpretation.

> **Change (agreed 2026-09-26): the `20`--`59` band is split.** `20`--`29` now means *open, surveyed water* where fast travel is allowed; `30`--`59` keeps the old meaning. This was added for Version 3's higher speeds. It is backwards compatible: Autonomy's and Safety's existing band checks (`≤ 19`, `≥ 60`, `≥ 90`) are unchanged. Versions 1 and 2 travel far below every speed limit. Current maps publish water at `30`, so nothing is fast-travel until Person 3 marks open-water cells `20`--`29`. **Every workstream: read this before changing cost values or speed logic.**

| Cost | Meaning | Mobility | Speed limit (Version 3; V1/V2 are slower anyway) |
| --- | --- | --- | --- |
| `0`--`19` | Firm shore | HOVER (Version 2 is hover-first; tracks only for a steep firm-land climb) | 15 km/h (4.2 m/s) |
| `20`--`29` | **Open, surveyed water**: no roots or debris | HOVER | Cruise 30 km/h (8.3 m/s), max 50 km/h (13.9 m/s) |
| `30`--`59` | Mud, shallow water, and root or debris zones | HOVER | 10 km/h (2.8 m/s) |
| `60`--`89` | Elevated-risk mud or shallow water | Conservative HOVER | 10 km/h, with conservative energy assumptions |
| `90`--`100` | No-go | None | – |
| `-1` | Unknown, treated as no-go | None | – |

Autonomy plans with these costs, and Safety estimates the return energy and time using the same bands. A path that crosses a `90+` or `-1` cell is invalid. The speed limits are layered: Autonomy proposes speeds within them, Safety enforces them, and Person 4's vehicle controller caps the command as a last line of defence. See *Vehicle Version 3* and the Version 3 open requests.

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

### Vehicle Version 3 (built and tested; not the demo vehicle yet)

Version 3 is Version 2's design scaled up for **higher speed, sharper turning and more payload**. It is built (`vehicle:=v3`, `cameras:=all` for the extra cameras) and passes its Gazebo and ROS checks (`docs/VEHICLE_V3_PLAN.md`, *Status*). **Version 2 stays the demo vehicle and the launch default** until the team switches after corridor runs. Nobody should switch the demo to Version 3 before then. The plan is `docs/VEHICLE_V3_PLAN.md`. The numbers below come from `tools/vehicle_sizing/v3_sizing.py`; its report is `docs/VEHICLE_V3_SIZING.md`, and you should rerun it after changing any input. They are first-order design estimates, not validated data.

Agreed Version 3 decisions (2026-09-26):

- **Speed targets:** cruise 30 km/h (8.3 m/s) and max 50 km/h (13.9 m/s), only over open, surveyed water (cost `20`--`29`). In root and debris zones and on mud it keeps to about 10 km/h (2.8 m/s). See *Shared terrain-cost semantics*.
- **Power: series hybrid.** A diesel engine (about 20 kW) only drives a generator. The lift fan, thrust fans, tracks and electronics are all electric, with a 5 kWh buffer battery for bursts. This keeps the fast, reversible electric fan control that autonomy and braking rely on. Estimated fuel use is about 2.5 L/h at 30 km/h, giving about 12 h on a 30 L tank. The all-electric alternative would last about 2.8 h.
- **Size and payload:** 3.0 × 1.8 m footprint, 100 kg rated payload, about 530 kg total. Cushion pressure is about 1.0 kPa, similar to Version 2.
- **Faster:** bigger ducted fans (2 × 0.7 m, about 10 kW each; static thrust about 940 N, 18% of weight, enough to get over the water "hump" at about 11 km/h). Less drag from a faired bow and an enclosed payload. **The skirt design stays as in Version 2.**
- **Turning:** rudders and puff ports carried over from Version 2 and sized up. At speed the turn radius is large (about 70 m at 30 km/h, about 200 m at 50 km/h), so routes must slow down before curves.
- **Tracks:** kept, capped at about 15 km/h. Speed comes from hovering.
- **Track deployment:** Version 3 remains hover-first, but deploys onto TRACK after a sustained forward firm-ground climb of **8° or more**. This is intentionally earlier than Version 2's 14° threshold: V3's stated hover-climb margin ends at 8°, while its tracks are intended to climb through 20°. Water, mud, level ground, descent, pivots and side tilt remain HOVER conditions.
- **Braking** from 50 km/h takes about 67 m with reverse thrust, beyond the 30 m LiDAR range. This is why high speed is only allowed in open, surveyed water.
- **Cameras:** the front camera plus **rear, left and right** cameras for a full view around the vehicle, at the same low-load profile as Version 2's (320 × 240, 5 Hz). They can be switched on or off at launch. They must not slow the LiDAR or the front camera, and are off by default if they do. **No top-down camera was fitted**: any mount above the LiDAR would sit in its beams, so a bird's-eye view is left for software stitching of the four cameras later.
- **Collision detection:** Gazebo contact sensors on the hull, skirt and tracks report what was hit and where. An IMU impact (jolt) check backs them up and would also work on a real vehicle. This adds one new topic (see the Version 3 requests).
- **Launch:** `vehicle:=v3` (and `cameras:=front|all`). Same frame names and public topics as Version 2, plus `fuel_percent`, `/vehicle/collision` and the extra camera topics (`docs/INTERFACES.md`).

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
- The planner rebuilds the active route after a new goal, an actual terrain-cost change, a relevant confirmed obstacle or a Safety return request. It preserves active-path geometry across odometry movement, unchanged cost-map refreshes and duplicate terrain-state telemetry so the follower is not repeatedly reset onto equally good A* variants. Odometry movement of at least half a map cell still refreshes Safety's prospective return path, and mission completion is checked on every odometry update.
- Added LiDAR obstacle projection: raw `/scan` hit cells are temporally confirmed before footprint inflation, preventing overlapping inflated cells from confirming scan ghosts. A hit already inside the private static clearance map is not overlaid again, so surveyed rocks and trunk proxies are visible without being double-inflated into a closed corridor. New unmapped obstacles replan only when they affect the outbound or return route; unrelated removals retain the existing safe detour, while a fully cleared overlay or blocked mission triggers route recovery. Simulation's terrain map remains unchanged.
- Implemented `path_follower`, which consumes `/planned_path` and `/odom` and publishes forward and turning proposals on `/cmd_vel_proposed` at 10 Hz.
- The follower treats A* output as a safe corridor rather than a sequence of grid centres that must be touched. It projects the vehicle onto that corridor and interpolates a target ahead along its arc length; Version 3 scales that preview with measured speed. It slows near the goal, stops to correct large heading errors, and proposes zero motion for empty paths, stale odometry or mismatched frames. A timestamp-only refresh of identical path geometry preserves follower progress instead of resetting it to the first waypoint.
- Added 39 ROS-independent planner, follower, motion-limit and LiDAR tests and eight ROS topic integration tests (47 Autonomy tests total).
- The shared launcher selects LiDAR inflation by vehicle: `0.75 m` for Version 1, `1.7 m` for Version 2 and `2.0 m` for Version 3. The standalone planner default remains the Version 1 value.
- Static cost `90+` and unknown cells are now inflated by that same vehicle-specific radius in Autonomy's private planning map. This converts Simulation's physical occupancy map into vehicle-centre configuration space without modifying `/terrain_costmap`; LiDAR hits remain a separate temporary overlay.
- Version 3 now uses speed-aware 8--30 m sensing, a sensor/clearance-limited 6.55 m/s proposal ceiling, a 3.0 m minimum/1.8 s speed-scaled lookahead, and a stopping-distance preview that brakes for upcoming route curvature before a sharp turn. At a 0.45 rad route-heading error while moving it also proposes bounded 0.8 m/s reverse thrust to arrest momentum before recovering; Safety still owns the final `/cmd_vel` and can clamp this proposal. Moving yaw is limited by 0.7 m/s² lateral acceleration and measured yaw-rate damping, with an 8 m final slowdown and a 0.3 m mission-event tolerance. Its LiDAR conversion retains only 0.30--1.30 m terrain protrusions, so rocks and trunk collision geometry remain visible while the embankment and visual-only canopy cannot seal the route. These V3-only settings reduce grid-cell cornering and counter continuing yaw after a turn; Versions 1 and 2 are unchanged. In the final static integration run before this retune it published `delivery_confirmed`, returned automatically and published `mission_complete`; outbound maximum path error was 0.52 m, and it settled stationary 0.22 m from the HOME grid endpoint.
- A live V3 corridor regression after this retune produced no collision, no LiDAR self return (nearest finite return 2.03 m, outside the 1.85 m body mask), peak yaw rate 0.27 rad/s and peak lateral acceleration 0.50 m/s². It stayed controlled through the first north-side detour, then Safety held because LiDAR reported low geometry outside the Environment map's existing static collision proxies and A* could no longer make a valid return route. **Open integration issue for Person 3 and Person 1:** audit the physical mangrove/root collision footprints against the published cost map. Do not suppress those unmapped hits in Autonomy; once Environment publishes the matching no-go cells, Autonomy's static-clearance filter prevents them from being double-inflated.
- Hardened the `global_planner` and `path_follower` entry points against common ROS shutdown races. Standalone shutdown is clean; one final WSL full-launch run still required launch to kill `global_planner` and Gazebo after mission completion, so shared-launch teardown remains an integration issue.
- Restored the autonomy package's `ament_python` build-tool declaration and verified the full eight-package workspace build.
- Verified live simulated odometry and LiDAR integration. The full launcher completed an autonomous delivery and HOME return while Safety remained the only `/cmd_vel` publisher. A later Version 2 static-world regression confirmed the `1.7 m` inflation override, Safety in `CRUISE`, and movement from approximately `x=0.00 m` to `x=5.32 m`; the newly added supervised remote-control path did not interfere with autonomous proposals.

### Path follower implementation details

`path_follower` is split into a ROS-independent control core and a ROS 2 wrapper. The wrapper receives `/planned_path` and `/odom`, runs the controller at 10 Hz and publishes only `geometry_msgs/Twist` proposals on `/cmd_vel_proposed`. It sets `linear.x` for forward speed and `angular.z` for turning; all other `Twist` fields remain zero.

For each control update, the follower:

1. starts its search at the last reached portion of the path so progress does not move backwards;
2. projects the vehicle onto the remaining route and interpolates a target one lookahead distance ahead along the route, or selects the final waypoint when the remaining route is shorter;
3. calculates the shortest signed heading error between vehicle yaw and the selected waypoint;
4. applies proportional turning and clamps the result to the angular-speed limit;
5. stops forward motion when the heading error is large, otherwise reduces speed according to heading error and remaining goal distance; and
6. publishes zero forward and angular motion inside the goal tolerance.

Default path-follower parameters are:

| Parameter | Default | Purpose |
| --- | ---: | --- |
| `control_rate_hz` | `10.0` | Keep proposed commands fresh for Safety. |
| `max_linear_speed` | `0.8 m/s` | Bound the maximum forward proposal. |
| `max_angular_speed` | `0.45 rad/s` | Bound the maximum turning proposal. |
| `lookahead_distance` | `0.75 m` | Select a smoother target ahead on the path. |
| `goal_tolerance` | `0.25 m` | Stop when the vehicle is close enough to the final waypoint. |
| `heading_gain` | `0.9` | Convert heading error into turning speed. |
| `slow_down_distance` | `1.0 m` | Reduce forward speed near the goal. |
| `rotate_in_place_angle` | `1.05 rad` | Stop forward motion while correcting a large heading error. |
| `odom_timeout` | `0.5 s` | Stop proposing motion when localisation is stale. |

The follower proposes a zero command when the path is empty, odometry is missing or stale, frames do not match, or the goal is reached. The global planner publishes an empty path once when a new terrain map invalidates a previously published route. This prevents continued tracking of a stale route. Repeated publications with the same frame and waypoint geometry are treated as freshness updates and do not reset the follower's progress index.

Version 3 launch overrides are: `max_linear_speed=13.9`, `max_angular_speed=0.45`, `lookahead_distance=3.0`, `lookahead_time_s=1.8`, `heading_gain=0.5`, `rotate_in_place_angle=0.70`, `yaw_rate_damping=0.6`, `corner_preview_sample_distance_m=3.0`, `slow_down_distance=8.0`, `lateral_accel_limit_mps2=0.7`, `obstacle_detection_range_m=30.0` and `obstacle_clearance_m=2.0`. The last two cap the actual autonomous proposal at about 6.55 m/s under the 1 s reaction/1.0 m/s² braking assumptions. The corner preview checks route curvature throughout the current reaction-plus-braking horizon and lowers the proposal early enough to reach the curve-speed limit before the bend; moving yaw is then capped so `v × |yaw_rate| <= 0.7 m/s²`. Yaw damping subtracts measured rotational velocity from the proposal so the larger V3 body settles instead of continuing through the desired heading.
The follower is independent of `TRACK` (Version 1: `WHEEL`), `TRANSITION` and `HOVER` actuator behaviour. Person 2 retains final command authority and remains the only publisher of `/cmd_vel`; Person 4 translates the approved body-motion command into wheel, lift-fan and propulsion-fan behaviour.

### LiDAR obstacle response implementation details

The global planner keeps Simulation's `/terrain_costmap` unchanged as its base map. It first inflates all static no-go and unknown cells by the configured vehicle-clearance radius in a private A* map. Each `/scan` update is then converted from polar range measurements into world coordinates using the vehicle pose from `/odom`, then into terrain-grid cells. Invalid, infinite, too-near and over-range measurements are ignored. Detected cells are expanded by the same configured radius and marked no-go only in that internal planning copy.

Dynamic raw hit cells use temporal hysteresis before footprint inflation. A hit must appear in two consecutive scans before it is added, while five consecutive misses are required before it is removed. Brief LiDAR dropouts therefore retain the existing detour without letting overlapping inflated footprints confirm a moving scan artifact. A newly confirmed cell runs A* immediately only if it intersects the active path or prospective return path. Unrelated removals do not reset a safe detour; clearing the whole overlay or removing cells while no route exists triggers route recovery. If obstacles remove every safe route, the planner publishes an empty `/planned_path`.

Default LiDAR parameters are:

| Parameter | Default | Purpose |
| --- | ---: | --- |
| `obstacle_inflation_radius_m` | `0.75 m` | Static and LiDAR no-go clearance; shared launch uses `1.7 m` for V2 and `2.0 m` for V3. |
| `obstacle_min_range_m` | `8.0 m` | Minimum useful planning look-ahead, even at low speed. |
| `obstacle_max_range_m` | `8.0 m` | Ignore detections beyond the useful local planning distance. |
| `obstacle_brake_decel_mps2` | `1.0 m/s²` | Braking assumption for speed-aware scan range. |
| `obstacle_reaction_time_s` | `1.0 s` | Reaction allowance added before braking. |
| `obstacle_confirmation_scans` | `2` | Consecutive detections required before a cell becomes blocked. |
| `obstacle_clear_scans` | `5` | Consecutive misses required before a blocked cell is removed. |

The LiDAR is assumed to be located at the odometry position and aligned with the vehicle's forward direction; that horizontal transform is exact for Versions 2 and 3. The 0.75 m circular inflation is based on Version 1's footprint. The shared launcher uses 1.7 m for Version 2 and 2.0 m for Version 3. On V3 the scan range grows from 8 m to at most 30 m as stopping distance increases, including the 2 m clearance.

### Return mission implementation details

During outbound travel, the planner publishes a prospective route from the current pose to HOME on `/return_path` so Safety can calculate return energy and time before allowing departure. A `return_required=true` message on `/safety_status` then latches return mode. The planner immediately invalidates the outbound route, publishes a fresh HOME route on `/return_path`, and publishes the same route on `/planned_path` for the path follower.

Every terrain cost-map update forces a newly stamped route publication. If the map contents are unchanged, the planner republishes the existing cells without running A* or resetting follower progress; an actual cost change rebuilds the route. A return-only 2 Hz refresh prevents callback ordering from making Safety treat that route as old. Later `return_required=false` messages do not cancel the latch. The existing `reset` scenario event clears the mission and routes, then Autonomy publishes `mission_reset`.

Autonomy publishes `delivery_confirmed` when odometry reaches the outbound path endpoint and `mission_complete` when it reaches the HOME path endpoint. Empty active and return paths are published at completion so the path follower proposes a stop while Safety resets to `PRELAUNCH`.

Default return parameters are:

| Parameter | Default | Purpose |
| --- | ---: | --- |
| `home_x_m` | `0.0 m` | HOME position in the configured frame. |
| `home_y_m` | `0.0 m` | HOME position in the configured frame. |
| `home_frame` | `map` | Frame containing HOME. |
| `goal_event_tolerance_m` | `2.0 m` | Delivery/HOME zone radius that triggers a mission event. |
| `return_path_refresh_rate_hz` | `2.0 Hz` | Keep Safety's validated return route fresh without resetting the follower. |

The shared Version 3 launch overrides `goal_event_tolerance_m` to `0.3 m`; together with its 8 m final slowdown this prevents mission events from clearing the route while the larger craft is still travelling quickly.

The HOME values must match Person 2's safety parameters. The changing map-frame tide map is integrated through the shared launcher; Person 1 and Person 4 should tune the final sensor transform and route geometry against the corridor before the demo. Autonomy does not publish `/cmd_vel`.

## Role 4: Vehicle simulation and integration status

- **Version 3 built (2026-09-26), not the demo vehicle.** `vehicle:=v3` (`cameras:=all` adds the rear, left and right cameras). It passes all its Gazebo checks: 50 km/h, braking from 50 km/h in 58 m, turn radius 9 m at 10 km/h and 77 m at 30 km/h, the 15° ramp on the tracks, and the Version 2 scenarios. The collision test reports "skirt front hit wall". Full ROS missions in the integration world complete with zero false collisions and with the zone speed limits held. Details: `docs/VEHICLE_V3_PLAN.md` (*Status*) and `docs/VEHICLE_SIMULATION.md` (*Version 3*). The corrected corridor base map yields a valid 107-cell route, and V3's false near-field self returns are now removed by inclusive body bounds plus a 1.85 m radial self mask. A repeat corridor mission is still required after the steering retune.
- **Version 2 delivered on `main` and is the launch default.** It includes the Blender model (visually polished to Version 1's standard, with the hull blended into the skirt), SDF/URDF, the mobility controller (`gear: tracks`), a bridge configuration and the test worlds described in `docs/VEHICLE_SIMULATION.md`. Launch with `ros2 launch tidal_vehicle_bringup sim.launch.py`, or add `vehicle:=v1` for the fallback.
- **Gazebo physics verified:** all 19 Version 2 checks pass (`tools/vehicle_tests/analyze.py`):
  - hover gap 5.05 cm held with the tracks retracted;
  - 2.0 m/s and 0.46 rad/s command tracking in HOVER;
  - 1.00 m/s and a 0.51 rad/s pivot with 5 mm drift in TRACK;
  - the 15° ramp climbed on the tracks;
  - exactly 60% cushion load share while resting on the tracks;
  - the full water → mud → bank sequence: hover, stop, deploy the tracks, lower the lift to 40%, then climb a 12° bank on the tracks.

  The Version 1 regression checks are unchanged.
- **ROS 2 integration verified** (headless, integration world with `tide:=false`). An autonomous goal across water and mud produced `delivery_confirmed` then `mission_complete` in 82 s. Version 2 now starts in HOVER at HOME and remains there across ordinary shore, mud and water; it deploys tracks only for a sustained forward firm-land climb at or above 14° (a guard band for the 15° bank). Safety remains the sole `/cmd_vel` publisher. The Version 1 fallback completes the same mission.
- **Tidal corridor** (the default world): it runs through `sim.launch.py` with DART/Bullet physics, sensors and terrain zones. HOME is on the first z = 0 m bank of a 120 m corridor. The 96 m tidal valley occupies 80% of that footprint, mirrored 15° bank sections lead into long gentle lower slopes and a narrow z = -3 m centre, and the delivery marker is on the opposite bank at `(104, 0)`. Physical cushion support, the visible sheet and the ROS tide manager start at z = -2.80 m as a narrow central channel and rise 2.80 m over 180 simulated seconds until water reaches both bank crests. Water stays traversable in HOVER mode; tide changes route cost and obstacle-clearance assumptions rather than becoming a generic closure. The corridor mission must still be verified end to end; `vehicle_tests/integration_test` remains the static regression world (`world:=vehicle_tests/integration_test tide:=false`).

Open integration items:

1. **Valley mission validation.** Confirm that Version 2 transitions smoothly from HOME down the 15° slope, rides the `TerrainZones` water surface across the basin, climbs the opposite 15° slope and reaches the delivery marker at `(104, 0)` without contacting the basin floor.
2. **Tide timing validation.** The synchronized rise is now 2.80 m over 180 simulated seconds, from z = -2.80 m to the z = 0 bank crests. Confirm on the demo laptop that a normal delivery crosses while fresh terrain costs and routes are published. Use `tide_hold` for a static baseline, then `tide_resume` or `tide_rise` to demonstrate changing-terrain replanning. A blocked route, energy limit or vehicle fault—not water alone—demonstrates Safety fallback. `tide_hold`, `tide_resume`, and `tide_reset` are runtime-verified to keep the visible surface, physics-side level, `/terrain_state`, and `/terrain_costmap` under the same scenario control; laptop mission timing remains to be measured.
3. **Terrain rendering optimisation (not done yet).** The mangrove meshes remain heavy for the camera; rocks already use low-cost native ellipsoids and both obstacle types retain simple collision proxies for physics and LiDAR. Before the final demo, create a low-poly visual/LOD mangrove variant while retaining the existing simple root/trunk collision proxies, reduce shadow-casting foliage, and benchmark real-time factor with the front camera enabled. With software rendering in a CPU-only container, the full ROS corridor stack ran at a real-time factor of 0.001–0.6 (the integration world runs near real time), so the corridor mission could not be timed there. Check the real-time factor on the demo laptop's GPU.
4. **Sensor transform.** Resolved: the Version 2 LiDAR is directly above `base_link`, so Autonomy's "LiDAR at odometry position" assumption is exact horizontally (*Open requests*, item 2).
5. Confirm the complete corridor mission repeatedly from the shared headless launch: camera, LiDAR, map-frame odometry, tide updates, replan and Safety fallback must all be visible in Foxglove.
6. Tune the corridor map rectangles, HOME/delivery coordinates and sensor-return geometry so that the generated path matches the visibly safe route through the world.
7. **Version 2 obstacle inflation and route clearance.** The footprint correction is complete: the shared launch uses `0.75 m` inflation for Version 1 and `1.7 m` for Version 2. Revalidate HOME and return-route clearance in the newly expanded valley with the current collision proxies and LiDAR filtering. If near-terrain returns block HOME, align the spawn, HOME and grid origin or remove unintended returns; do not restore the undersized Version 1 radius.
8. **Safety return energy (Person 2).** The energy and HOLD-drain work in *Open requests between workstreams*, items 4–6, remains open. Re-check corridor return timing after it lands.
9. **Rudders and puff ports: verified and on (2026-09-26).** All 30 Version 2 checks pass. The rudders fade in above 15% thrust and slew at most 1.5 rad/s, which removed the low-speed chatter that led to them being switched off. Three clean autonomous missions completed. The demo sensor profile and the 400 yaw gain, previously hand-edited into `model.sdf`, now come from `gen_description_v2.py`. **Don't hand-edit generated `model.sdf` files; change the generator instead.**
10. Add Person 5's Foxglove layout: 3D scene, `/camera/image_raw`, planned and return paths, terrain-cost map, battery, safety reason and tide-window fields.
11. **Version 3 corridor stability.** Keep V3's 30–50 km/h configuration confined to its isolated open-water test world. Build a conservative corridor-speed profile, validate that it stays within map bounds through the obstacle detour and 15° embankment, then reassess whether it is appropriate for a demo.
12. **Done: Version 3 LiDAR self-filter (2026-09-26).** The closest false returns from the live corridor run, `(-1.56, -0.11, -0.42)` and `(-0.99, 1.27, -0.43)` in `lidar_link`, are rejected by an inclusive, tested 1.85 m circular footprint envelope around V3's centre mast. The test keeps a point at 1.86 m, proving genuine external obstacles are retained. Autonomy continues to receive all non-self returns and must not suppress external obstacles to compensate.

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
3. Rising tide changes terrain cost and obstacle clearance, forcing repeated route assessment and replanning while water remains traversable in HOVER mode.

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
- **Person 2:** adds return-reserve and explicit-hazard handling. Rising tide must produce a visible replan without a generic water closure; a blocked route, energy limit or vehicle fault must produce `HOLD` or `RETURN` with a clear rationale.
- **Person 5:** implements deterministic scenario triggers, result logging and the Foxglove mission layout. It also prepares a simple capability comparison graphic for wheeled rover, boat and air-cushion vehicle profiles.

**Exit gate:** all three required scenarios run from the common launcher:

1. normal delivery and return;
2. obstacle detected, route replanned and mission completed; and
3. rising tide updates the terrain cost map and causes a visible replan while water remains traversable; a separate blocked-route, energy or vehicle-fault condition demonstrates safety-driven hold or return.

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

- A 120 m corridor spanning x = -12–108 m, with a 96 m tidal valley from x = 4–100 m (80%) and only 24 m of combined dry banks (20%). Mirrored 15° bank sections lead into long 2.7° lower slopes and a narrow z = -3 m centre. HOME remains `(0, 0)` and the delivery marker is `(104, 0)`.
- A 96 m-long level water sheet spanning x = 4–100 m and the full 60 m map width. It starts at z = -2.80 m as a roughly 12.5 m-wide central channel (about 10% of the corridor area), then rises 2.80 m over 180 simulated seconds and expands to the full 80% tidal footprint only at bank height. It remains traversable in HOVER mode.
- Twelve Mangrovetree GLB-derived mangroves, 7–15 m tall, arranged fully inside the dry and wet margins. Their roots and trunks retain collision geometry for physics and LiDAR.
- Twenty-four low-cost native Gazebo ellipsoid rocks arranged in irregular bank and channel-edge clusters, with primitive collision shapes for physics and LiDAR and a broad navigable route around y = 0.
- A visual-only green delivery pad on the far high ground.

The water sheet itself is visual-only so it can pass through trees and rocks. The 120 x 60 `/terrain_costmap` matches the same profile: high ground and dry 15-degree bank faces are firm (cost 10), lower dry slopes are mud (cost 45+), and cells become water (cost 55+) when the rising surface reaches their terrain height, and all known tree and rock collision footprints are marked no-go (cost 100). Physical water support and drag come from the matching `hover::TerrainZones` surface used by `hover::AirCushion`; this is what keeps the hovercraft above the rising water. Do not remove collision elements from tree or rock models: Gazebo ray sensors detect collision shapes, not visual meshes.

### Run the current map

```bash
cd ~/One-Wish-Coders/autonomous_tidal_vehicle_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to tidal_vehicle_bringup
source install/setup.bash
ros2 launch tidal_vehicle_bringup sim.launch.py headless:=true dashboard:=true
```

For a quick headless validation, replace the final command with:

```bash
timeout 12s ros2 launch tidal_vehicle_bringup sim.launch.py headless:=true
```

The authoritative world is `src/tidal_vehicle_simulation/worlds/tidal_corridor.sdf`. Environment assets are under `src/tidal_vehicle_simulation/models/`, including `mangrove_imported/` and `rock_imported/`.
