# Vehicle simulation (Person 4: vehicle simulation and integration)

This describes the air-cushion vehicle: model, physics abstraction, sensors, ROS interface and the one-command launch. **Version 2** (tracked, below) is the demo vehicle and the launch default. Sections 1–6 describe the shared pipeline and **Version 1** (wheeled), which is the fallback with `vehicle:=v1`. The public topics are in [`INTERFACES.md`](INTERFACES.md), and the design decisions are in `AGENTS.md` under "Vehicle Version 2".

## Version 2: tracked, hovercraft-dominant (the demo vehicle)

```bash
ros2 launch tidal_vehicle_bringup sim.launch.py                        # Version 2 in the tidal corridor
ros2 launch tidal_vehicle_bringup sim.launch.py world:=vehicle_tests/integration_test tide:=false
ros2 launch tidal_vehicle_bringup sim.launch.py vehicle:=v1            # Version 1 fallback
```

**Vehicle:**
- 2.5 m long, 1.5 m wide, 1.5 m high; 300 kg including a 30 kg payload box.
- Bag skirt with segmented fingers.
- Lift fan offset forward, so the 16-channel LiDAR sits on a centre mast directly above `base_link` at 1.44 m.
- Two reversible ducted fans (200 N each, 80% reverse) with rudders in their slipstream (±25° commanded).
- Four puff ports: louvred side vents (0.24 × 0.12 m) at the bow and stern on both sides, fed with cushion air. Each vent has a sliding shutter (`puff_<bow|stern>_<left|right>` links). It slides 0.26 m along the hull, bow shutters aft and stern shutters forward, in proportion to the vent force. So Gazebo, RViz and Foxglove show which vents are firing. The shutters are visual only: the thrust is computed in the plugin, and they stay inside the 1.5 m footprint and the LiDAR self-filter box. See `renders/puff_port_open.png`.
- Two inboard rubber tracks (0.28 m wide, 1.40 m contact, 0.84 m gauge) on prismatic joints. They retract 0.25 m into hull wells in HOVER.

**Rebuild the model:**
```bash
blender --background --python assets/vehicle_blender/version_2/build_vehicle.py
python3 autonomous_tidal_vehicle_ws/src/tidal_vehicle_description/scripts/gen_description_v2.py
```
The Blender script also runs with the pip `bpy` 4.2 module (`python3 .../build_vehicle.py`). Its spec checks are in `renders/dimensions.txt`, and all of them pass.

**Simulation model (stated assumptions, not validated physics):**
- `hover::AirCushion` gains are scaled from Version 1 to 300 kg: same cushion natural frequency and damping ratio, 5 cm hover gap, drag sized for a top speed of about 3 m/s.
- New plugin inputs:
  - `lift_share` (0–1): in TRACK mode the cushion carries that share of the weight and the tracks carry the rest. While a share is active the propulsion fans idle, because the tracks propel and brake; a fan speed loop holding zero would fight them.
  - `reverse_thrust_fraction` 0.8, which brakes and holds the vehicle on slopes up to about 6°.
  - `yaw_reserve_fraction` 0.3, which keeps some steering while braking.
  - Turning aids (`rudder_control`, `puff_port_*`). In HOVER the yaw-rate controller asks for a yaw moment and shares it out in this order:
    1. **Rudders.** The plugin sets both rudder angles on `rudder_left_cmd`/`rudder_right_cmd` (side force 0.55·T·sin δ at the stern). They are free while the fans push forward, strongest at speed and useless at zero thrust. So they fade in between 60 N and 120 N of forward thrust (`rudder_min_thrust`, 15% of full thrust) and slew at no more than 1.5 rad/s (`rudder_rate`). Without this, at low speed they swung from stop to stop chasing small heading errors, which made the vehicle wobble.
    2. **Puff ports.** Opening a bow vent on one side and a stern vent on the other makes a pure yaw couple at any speed, including a pivot in place. Each vent gives 2·Cd·p·A ≈ 37 N at the 0.81 kPa cushion pressure (Cd 0.8), so a pair gives ≈ 55 N·m. Force scales with cushion pressure and follows a 0.15 s valve lag. Spare vent force damps sideways drift (150 N per m/s of slip).
    3. **Differential fan thrust** supplies the rest, with the same speed-first limits as before.

    The moment the rudders and vents actually deliver is subtracted, so the total never exceeds what the controller asked for. The aids are off in TRACK load sharing (the tracks steer). `/model/hovercraft_v2/turn_aids` (`gz.msgs.Boolean`, a Gazebo-only topic) switches them off for A/B tests.

    Stated assumption: each open vent bleeds about 0.85 m³/s of cushion air, similar to the skirt's own leakage. The lift fan is assumed to have the flow margin for one open pair, and no lift loss is modelled. Version 1 doesn't set these parameters and is unchanged.
- Gazebo's `TrackController` and `TrackedVehicle` drive the tracks from `/vehicle/cmd_vel_tracks`.
- `hover::TerrainZones` water only counts where the water surface is above the measured ground. A water zone can also rise over time (`<rise>`, `<rise_duration>`).
- The tidal corridor uses no world-wide Gazebo buoyancy plane. It has two z = 0 m banks around a 96 m-wide tidal valley, which is 80% of the 120 m corridor footprint. Short mirrored 15° bank sections lead into long 2.7° lower slopes and a narrow z = -3 m centre. The synchronized `TerrainZones`, visible water surface and ROS tide manager start at z = -2.80 m and rise 2.80 m over 180 simulated seconds, expanding from a small central channel to the bank crests. Air-cushion support and water drag come from `TerrainZones`, while solid collision ground remains beneath the water. Water remains traversable in HOVER mode; the rising tide updates cost and obstacle-clearance assumptions rather than acting as a generic corridor closure.

**Mode control** (`vehicle_mobility_node`, `config/vehicle_mobility_v2.yaml`, `gear: tracks`, `mode_policy: terrain_auto`):

| From → to | Sequence (horizontal motion is zero throughout) | Ready when | Fault on timeout (12 s) |
|---|---|---|---|
| TRACK → HOVER | full lift → wait for `hover_state == HOVER` → retract the tracks | measured track joints at 0.25 m | `hover_not_ready` / `track_deployment_fault` |
| HOVER → TRACK | deploy the tracks while hovering → lower the lift to the TRACK load share | skirt gap at the on-track height (≤ 3.8 cm), or vertical speed below 3 cm/s in `LOAD_SHARE` | `track_deployment_fault` / `track_settle_timeout` |

**Mode selection:**
- Version 2 is hover-first across firm shore, mud and water: its tracks retract once the cushion is ready, and the fans hold the configured gap over the active support surface.
- TRACK is selected only for a sustained (>1.5 s), forward climb at or above 14° on firm land. This 1° guard band deploys the tracks before the physical 15° bank can stall the hovercraft. Side tilt, stopping, pivoting, descending, mud and water do not deploy it.
- In that exceptional TRACK mode, the cushion carries 60% of the load while the tracks provide climbing traction. It returns to HOVER when the climb no longer applies.

**Tests:**
```bash
python3 autonomous_tidal_vehicle_ws/src/tidal_vehicle_simulation/scripts/gen_test_worlds_v2.py --no-render-sensors --out /tmp/v2worlds
for t in "v2_hover_test 62" "v2_track_test 48" "v2_load_share_test 18" "v2_transition_test 63" "v2_turn_test 96"; do
  WORLD_DIR=/tmp/v2worlds tools/vehicle_tests/run_test.sh $t; done
python3 tools/vehicle_tests/analyze.py        # v2_* rows in results/acceptance.md
```
All 30 Version 2 checks pass (ROS 2 Jazzy, Gazebo 8.15, WSL, 2026-09-26) with the turning aids on:

| Test | Result |
|---|---|
| Hover gap with the tracks retracted | 5.05 cm (4.4–5.5) |
| HOVER tracking | 2.006 m/s at 2.0; 0.51 rad/s at 0.5 |
| TRACK drive | 1.000 m/s; pivot 0.51 rad/s with 6 mm drift |
| 15° ramp | climbed on the tracks |
| Load share | 0.600 at 0.6 commanded, resting on the tracks (gap 2.7–3.0 cm) |
| Water → mud → bank | hover across; stop; deploy; 40% share; climb a 12° bank on the tracks |
| Turn at 1.5 m/s, 1.0 rad/s asked | fans only 0.37 rad/s → with rudders and puff ports **0.85 rad/s**; rudders at 25° |
| Pivot in place, 0.8 rad/s asked | fans only 0.57 rad/s → with puff ports **0.81 rad/s** (37 N per vent, shutters open 0.26 m, 0.12 m drift) |
| Stability | max roll/pitch 0.2°; hover gap 4.3–5.6 cm while turning |

The earlier fans-only setting (turning aids off, the version on `main` before this change) also passes the 19 non-turning checks.

With the full ROS stack in the integration world, an autonomous goal across the water channel produced `delivery_confirmed` then `mission_complete` in 87 s. Open items in the tidal corridor are listed in `AGENTS.md` (Role 4).

**Test-host notes:**
- Headless rendering needs EGL (`libegl1`, `libegl-mesa0`).
- conda/RoboStack builds of gz-sim 8.10 lack `Model::SetStatic`. Build with `-DTIDAL_BUILD_TIDE_VISUAL=OFF` there.
- The apt Gazebo that ROS 2 Jazzy installs has `Model::SetStatic`, so none of this applies on the WSL laptop.

## Version 3: high-speed series hybrid (in development)

```bash
ros2 launch tidal_vehicle_bringup sim.launch.py vehicle:=v3                     # front camera only
ros2 launch tidal_vehicle_bringup sim.launch.py vehicle:=v3 cameras:=all        # + rear, left, right
```

Version 2 scaled up: 3.0 × 1.8 × 1.9 m, 530 kg including a 100 kg payload, series hybrid (a 20 kW diesel generator, 30 L of fuel, a 5 kWh buffer battery), 2 × 0.7 m ducted fans (940 N), rudders and puff ports, retractable tracks (15 km/h), a LiDAR mast raised to 1.82 m, and four cameras. Decisions: `AGENTS.md`, *Vehicle Version 3*. Plan, status and results: [`VEHICLE_V3_PLAN.md`](VEHICLE_V3_PLAN.md). Sizing: `tools/vehicle_sizing/v3_sizing.py`.

**Model pipeline:** `assets/vehicle_blender/version_3/build_vehicle.py` → `gen_description_v3.py` → `models/hovercraft_v3/`, `urdf/hovercraft_v3.urdf`.

**What is new in the simulation:**
- **High-speed drag** in `hover::AirCushion`, off unless set (Version 2 keeps its demo glide drag). It has four terms:
  - `glide_constant_drag`: skirt and spray drag, 104 N;
  - `water_hump_drag` / `water_hump_speed`: the over-water wave hump, 261 N peak at 3.0 m/s;
  - `thrust_falloff_speed`: fan thrust falls linearly to zero at 31.9 m/s;
  - air drag through `glide_quadratic_drag` (0.5 ρ CdA).

  Sideways, the skirt resists up to 10% of the weight (`glide_lateral_factor` 5).
- **Skirt collision** is the stiff round bag (box plus bag ends), not the 11 cm of flexible fingers. The bow overhang can therefore ride up a 15° bank, and debris under about 11 cm passes under the skirt.
- **Zone speed limits** (split cost bands, `AGENTS.md`) in `vehicle_mobility_node` (`zone_speed_limits_mps`, `brake_decel_mps2`), the path follower and Safety. Each looks ahead as far as it needs to stop.
- **Series-hybrid energy model:** the generator follows demand up to `genset_max_w` and tops up the battery. `/vehicle_health.fuel_percent` is reported (−1 on Versions 1 and 2).
- **Collision detection:** contact sensors on the hull, skirt and track collisions (Gazebo's Contact system is loaded by the model) plus an IMU jolt check feed `collision_monitor_node` → `/vehicle/collision`. Ground contact (vertical normals) and the vehicle's own parts are filtered out.
- **V3 track policy:** V3 also remains hover-first, but deploys tracks after a sustained forward firm-ground climb of **8° or more**. Its stated hover-climb limit is 8° and its stated tracked limit is 20°, so this earlier threshold avoids waiting until the 15° bank has already stalled hover propulsion. Water, mud, level ground, descent, pivoting and side tilt remain HOVER conditions.
- **Cameras:** rear, left and right on the mast (320 × 240, 5 Hz), bridged only with `cameras:=all`. Gazebo renders a camera only while it is subscribed: real-time factor 0.94 with the front camera, 0.78 with all four.

**Tests:**
```bash
python3 autonomous_tidal_vehicle_ws/src/tidal_vehicle_simulation/scripts/gen_test_worlds_v3.py --no-render-sensors --out /tmp/v3worlds
for t in "v3_speed_test 222" "v3_hover_test 62" "v3_track_test 48" "v3_load_share_test 18"          "v3_transition_test 63" "v3_turn_test 96"; do WORLD_DIR=/tmp/v3worlds tools/vehicle_tests/run_test.sh $t; done
python3 tools/vehicle_tests/analyze.py        # v3_* rows in results/acceptance.md
tools/vehicle_tests/run_collision_test.sh     # bow into a wall -> /vehicle/collision
```

## 1. Where things live

| Path | What |
|---|---|
| `assets/vehicle_blender/version_1/build_vehicle.py` | Procedural Blender model for **Version 1** (frozen; see `assets/vehicle_blender/README.md`). It writes the `.blend`, preview renders, one mesh per part and `link_frames.json`. |
| `tidal_vehicle_description/models/hovercraft/` | Gazebo model: `model.sdf`, meshes, `link_frames.json` |
| `tidal_vehicle_description/urdf/hovercraft.urdf` | The same vehicle for ROS (`robot_state_publisher`, RViz, Foxglove) |
| `tidal_vehicle_description/scripts/gen_description.py` | Builds `model.sdf` and the URDF from `link_frames.json`, so they can't drift apart |
| `tidal_vehicle_simulation/plugins/` | C++ Gazebo systems: `hover::AirCushion`, `hover::TerrainZones`, `hover::ScriptedCommands` → `libtidal_vehicle_plugins.so` |
| `tidal_vehicle_simulation/scripts/vehicle_mobility_node.py` | Safety-approved `/cmd_vel` → fans or wheels, TRACK/TRANSITION/HOVER switching, `/vehicle_health` and placeholder `/terrain_state` for static test worlds only |
| `tidal_vehicle_simulation/scripts/lidar_scan_node.py` | `/points` → `/scan` |
| `tidal_vehicle_simulation/scripts/terrain_costmap_node.py` | Static `/terrain_costmap` fallback for explicit vehicle-only test worlds |
| `tidal_vehicle_simulation/worlds/vehicle_tests/` | Vehicle test worlds (generated by `scripts/gen_test_worlds.py`). The tidal-corridor demo world belongs to the environment workstream. |
| `tidal_vehicle_bringup/launch/sim.launch.py` | The one-command launch |
| `tools/vehicle_tests/` | Headless test runner, analysis and A/B clip recorder. Output goes to `results/`, which git ignores. |
| `assets/vehicle_blender/version_2/build_vehicle.py` | Procedural Blender model for **Version 2** (tracked). It writes meshes and `link_frames.json` to `models/hovercraft_v2/`. |
| `tidal_vehicle_description/models/hovercraft_v2/`, `urdf/hovercraft_v2.urdf` | Version 2 Gazebo model and URDF, generated by `scripts/gen_description_v2.py` |
| `tidal_vehicle_simulation/config/vehicle_mobility_v2.yaml` | Version 2 mobility, LiDAR and battery parameters |
| `tidal_vehicle_bringup/config/ros_gz_bridge_v2.yaml` | Version 2 Gazebo ⇄ ROS topic map |
| `tidal_vehicle_simulation/scripts/gen_test_worlds_v2.py` | Version 2 test worlds (`worlds/vehicle_tests/v2_*.sdf`) |

## 2. Development setup (agreed team workflow)

- **Code lives in WSL:** Ubuntu 24.04 under WSL2, repo cloned to `~/One-Wish-Coders`. Don't work under `C:\...`.
- **VS Code:** edit through the Remote-WSL window. Its terminal, Python, git and ROS tools run inside WSL.
- **Gazebo and ROS 2:** both run in WSL. Gazebo's GUI appears through WSLg.
- **Foxglove Desktop** runs on Windows and connects to `ws://localhost:8765`. The launch file starts `foxglove_bridge` on `0.0.0.0:8765`, and WSL2 forwards `localhost` to it by default.
- **Line endings:** `.gitattributes` forces LF for scripts, so files edited on Windows don't break `bash`/`python` in WSL.

```bash
sudo apt install ros-jazzy-desktop ros-jazzy-ros-gz ros-jazzy-foxglove-bridge
source /opt/ros/jazzy/setup.bash
cd ~/One-Wish-Coders/autonomous_tidal_vehicle_ws
rosdep install --from-paths src -y --ignore-src
colcon build --symlink-install
source install/setup.bash
ros2 launch tidal_vehicle_bringup sim.launch.py            # Gazebo GUI + bridge + Foxglove bridge
ros2 launch tidal_vehicle_bringup sim.launch.py rviz:=true world:=vehicle_tests/sensor_test
```

The common launch includes Safety, which is the only allowed `/cmd_vel` publisher. Use the scripted vehicle-test worlds for direct actuator and physics checks.

**GPU check in WSL.** Run `glxinfo -B`; it must list the D3D12/vendor GPU, not
`llvmpipe`. On the NVIDIA WSL2 demo host, launch with `gpu:=nvidia` to select
the D3D12 NVIDIA adapter:

```bash
ros2 launch tidal_vehicle_bringup sim.launch.py headless:=true foxglove:=true dashboard:=true gpu:=nvidia
```

`LIBGL_ALWAYS_SOFTWARE=1` is only a fallback and is unsuitable for the live demo.

### Blender on Windows

The model is code, so nobody hand-exports meshes. Run the script with Windows Blender, pointed at the repo inside WSL. It writes straight into the WSL repo; there's nothing to copy:

```powershell
& "C:\Program Files\Blender Foundation\Blender 5.2\blender.exe" --background `
  --python \\wsl.localhost\Ubuntu-24.04\home\<you>\One-Wish-Coders\assets\vehicle_blender\version_1\build_vehicle.py
```

Then regenerate the description in WSL:

```bash
python3 autonomous_tidal_vehicle_ws/src/tidal_vehicle_description/scripts/gen_description.py
```

Blender 5.x can't export COLLADA, so it writes only `.glb`. The generator then uses `.glb` for Gazebo and the URDF. The committed meshes were made with Blender 4.2 and include both formats. If you regenerate with 5.x, check that the mesh orientation looks right once in Gazebo and RViz.

## 3. The vehicle

Section 2 of the build plan, plus the team's changes:

- **Dimensions:** hull/skirt footprint 1.2 × 0.7 m. Overall width is 0.99 m (agreed) because the wheels sit outboard.
- **Mass:** 25 kg including a 4 kg payload.
- **Heights:** in hover mode the hull top is at 0.44 m and the LiDAR at 0.69 m. The LiDAR mast was raised 0.10 m so the rear fan ducts no longer block the lower beams; self-returns in the rear ±30° sector dropped from 26 % to 12 %, and `/scan` filters out the rest.
- **Mobility:**
  - One lift fan.
  - Two rear thrust fans, 30 N each.
  - Two rudders in the fan slipstream.
  - Four 0.30 m balloon wheels on swing-up legs with coil-over suspension (8 cm travel). Fronts fold back, rears fold forward.
- **Payload:** sealed box, strapped to the deck.
- **Sensors:**
  - 16-channel 3D LiDAR, 0.3–30 m, 10 Hz
  - front camera
  - IMU, 50 Hz
  - GNSS
  - 4 downward range sensors

## 4. Physics abstraction (stated, not validated)

Everything here is a transparent, stated model. None of it is validated physics.

- **Air cushion.** Each skirt corner gets a preloaded spring-damper: `F = m·g/4 + k·(h_t − h) − c·ḣ` with h_t = 4 cm, k = 1500 N/m and c = 130 N·s/m.
  - The gap `h` comes from physics ray casts down from each corner. Over water it's measured to the water surface.
  - Above h_t the lift vents away exponentially. The lift fan spins up over 1.5 s.
- **Glide drag.** While hovering: linear + quadratic in speed. Sideways it is 3× higher (skirt walls, fins) and 1.3× higher over water.
- **Fans and rudders.** Thrust acts at the fan positions. The rudders' side force is 0.55·T·sin δ.
- **Hover velocity control.** `/cmd_vel` in HOVER mode becomes fan thrust inside the plugin: feed-forward drag, PI on speed, P on yaw rate.
- **Mud.** Friction is μ = 0.08. When the vehicle isn't on its cushion there is also a sinkage/rolling resistance `F = −(150·v + 0.25·m·g·tanh(v/0.05))`, so wheels bog down and the cushion glides over.
- **Skirt contact.** The collision box stops 5 cm above the skirt bottom (flexible fingers) and has friction 0.2. In hover mode, debris up to about 7–9 cm passes under the skirt; taller obstacles block and must be avoided using the LiDAR.
- **Water.** Gazebo's graded buoyancy applies to the hull and skirt, plus hull drag when off-cushion.
- **Odometry** is ground truth.
- **Battery** uses a simple power model: 20 W idle, 300 W lift fan, 7 W per N of thrust, 60 W for the wheels, from a 500 Wh pack.

## 5. Mode switching (`vehicle_mobility_node`)

The public modes are `TRACK`, `TRANSITION` and `HOVER`. Version 1 implements TRACK commands with its four wheels; Version 2 will use tracks.

- **`hover_only` (default):** starts in TRANSITION, enables the lift fan, retracts the wheels and waits for an explicit `HOVER` readiness message from the Gazebo plugin before allowing fan propulsion.
- **`terrain_auto`:** starts in TRACK, switches through TRANSITION to HOVER on MUD or WATER, and changes back toward TRACK after the configured firm-terrain dwell.
- Horizontal wheel and fan commands stay zero throughout TRANSITION.
- A transition that does not become ready before the timeout publishes `hover_not_ready` or `wheel_settle_timeout` through `/vehicle_health.fault`. Safety responds with HOLD.
- The current HOVER-to-TRACK readiness check uses plugin hover-off state plus a settling delay. Measured suspension load or ground-gear contact remains required before `terrain_auto` becomes the demo default.

## 6. Tests (headless Gazebo, `tools/vehicle_tests/`)

```bash
cmake -S tools/vehicle_tests/gz_sink -B tools/vehicle_tests/gz_sink/build && cmake --build tools/vehicle_tests/gz_sink/build
for t in "empty_test 18" "gap_hold_test 72" "hover_drive_test 45" "transition_test 30" \
         "debris_test 16" "cmd_vel_test 30" "mud_ab_test 22"; do tools/vehicle_tests/run_test.sh $t; done
python3 tools/vehicle_tests/analyze.py                  # -> results/acceptance.md, results/plots/
tools/vehicle_tests/record_ab.sh                        # Checkpoint 2 A/B clip -> results/mud_ab_test/
```

Latest run: every check passes.

| Test | Result |
|---|---|
| Legs fold/unfold; hull settles on skirt and lifts back | pass |
| Hover gap held 60 s | 4.01 cm ± 0.45 cm (spec ±1.5 cm) |
| Top speed / turn radius | 2.87 m/s / 1.0 m |
| Water → mud → firm bank | crosses, max tilt 5.8°, no sinking |
| 7 cm branch vs 20 cm log | branch passes under the skirt; log blocks |
| `/cmd_vel` in HOVER | speed error 0.03 m/s, yaw-rate error 0.001 rad/s, stops within 5 s |
| Mud A/B | wheels stop 0.74 m into the mud; hover crosses 8 m and stops past the finish |

These Gazebo physics tests ran on Gazebo Harmonic 8.15 in a cloud container with software rendering.

The complete ROS 2 side was subsequently verified in WSL with the default unscripted integration world:

- all eight packages build and all 42 registered tests pass;
- the bridge, state publisher, LiDAR converter, mobility controller, static cost map, planner, follower and Safety node start from the common launch;
- `/scan` connects to Autonomy with best-effort sensor QoS;
- Safety is the sole `/cmd_vel` publisher;
- a live autonomous goal produced `delivery_confirmed` followed by `mission_complete` after the HOME return; and
- all launch processes exit cleanly.
