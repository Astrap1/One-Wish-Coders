# ROS 2 interface contract

These are the initial contracts between workstreams. Topic names and message types may evolve, but update this file in the same change whenever they do.

| Topic | Publisher | Consumer | Initial type | Purpose |
| --- | --- | --- | --- | --- |
| `/scan` | Simulation (`lidar_scan_node`) | Autonomy | `sensor_msgs/LaserScan` | Best-effort sensor QoS. Near-field obstacle sensing used for the planner's temporary obstacle overlay. 2D scan in `lidar_link`, 720 bins, derived from `/points` (closest return per bearing between ~7 cm and 1.2 m above ground; the vehicle's own body is filtered out). 10 Hz. |
| `/imu` | Simulation | Autonomy | `sensor_msgs/Imu` | Orientation and motion sensing. Frame `imu_link`, 50 Hz. |
| `/odom` | Simulation (Gazebo OdometryPublisher) | Autonomy, Safety, Operator | `nav_msgs/Odometry` | Vehicle position and velocity. Ground truth (idealised), `header.frame_id: map`, `child_frame_id: base_link`, 50 Hz. Valid in both hover and ground mode. |
| `/terrain_state` | Simulation | Autonomy, Safety | `tidal_vehicle_interfaces/TerrainState` | Tide and traversability estimate in `map`. In the default tidal-corridor launch, `tide_manager.py` is the sole publisher. The vehicle's 10 Hz low-tide placeholder is enabled only with `tide:=false` for static test worlds. |
| `/terrain_costmap` | Simulation | Autonomy, Safety, Operator | `nav_msgs/OccupancyGrid` | Current map-frame terrain-risk map. In the default tidal-corridor launch, `tide_manager.py` publishes it from the same channel/mud rectangles as Gazebo `TerrainZones`; `tide:=false` selects the static test-world map. |
| `/vehicle_health` | Simulation (`vehicle_mobility_node`) | Safety, Operator | `tidal_vehicle_interfaces/VehicleHealth` | Raw battery, mobility, link and payload health at 10 Hz. Battery uses a stated power model per vehicle (`config/vehicle_mobility_v2.yaml` for Version 2, `vehicle_mobility.yaml` for Version 1); mobility, link, payload and fault remain runtime fault-injection parameters. From 2026-09-26 it also carries `fuel_percent`: diesel left, 0–100, on the Version 3 series hybrid; −1 means no fuel tank (Versions 1 and 2). |
| `/mission_goal` | Operator | Autonomy, Safety | `geometry_msgs/PoseStamped` | Requested delivery point. The browser dashboard's **Dispatch demo delivery** control publishes the fixed demo goal `(104 m, 0 m)` in the `map` frame; HOME is `(0 m, 0 m)`, placing the two points on opposite firm banks. A new goal is accepted as a retarget request only during `OUTBOUND`. |
| `/planned_path` | Autonomy | Path follower, Operator, Safety | `nav_msgs/Path` | Active proposed route: outbound normally, HOME route while return is latched. |
| `/return_path` | Autonomy | Safety, Operator | `nav_msgs/Path` | Prospective route from the current pose to HOME during outbound travel; freshly republished and activated when return is required. |
| `/cmd_vel_proposed` | Autonomy | Safety | `geometry_msgs/Twist` | Motion proposal before safety approval. |
| `/operator_remote_enabled` | Operator dashboard | Safety | `std_msgs/Bool` | Explicit control-source toggle. `true` selects supervised remote input and suppresses autonomous proposals; `false` returns command authority to Autonomy. Published reliably with transient-local durability so Safety receives the active selection after a restart. |
| `/operator_cmd_vel` | Operator dashboard | Safety | `geometry_msgs/Twist` | Hold-to-run remote driving request. Safety is the only `/cmd_vel` publisher and caps this input to 0.8 m/s linear and 0.6 rad/s angular. The dashboard publishes at 10 Hz while a direction is held and sends zero on release; stale input stops the vehicle. |
| `/cmd_vel` | Safety | Simulation (`vehicle_mobility_node`) | `geometry_msgs/Twist` | Safety-approved motion command: `linear.x` (m/s, ≤ 2.5 for Version 2; ≤ 13.9 for Version 3, capped by the zone speed limits) and `angular.z` (rad/s, ≤ 1.0). Both tracked versions normally route it to hover fans: Version 2 deploys tracks for a sustained firm-land forward climb at or above 14°; Version 3 deploys at or above 8° to remain below its hover-climb limit. Version 1 routes TRACK mode to wheels. Held at zero during TRANSITION. A command older than 0.5 s means stop. |
| `/safety_status` | Safety | Autonomy, Operator, Evaluation | `tidal_vehicle_interfaces/SafetyStatus` | State, rationale, return requirement and Safety-calculated return energy, margin and ETA. Autonomy must act on `return_required=true`. |
| `/mission_event` | Autonomy | Safety, Operator, Evaluation | `std_msgs/String` | Latest lifecycle event used by Safety for its internal mission phase. Reliable, transient-local QoS lets late-starting consumers recover the current lifecycle transition. |
| `/mission_event_history` | Autonomy | Operator | `std_msgs/String` (JSON) | Reliable, transient-local snapshot of the most recent 20 lifecycle events. The browser dashboard uses it to restore its mission-event timeline after a restart. |
| `/scenario_event` | Evaluation | Simulation, Safety, Autonomy | `std_msgs/String` | Controlled fault or scenario event. `reset` and `tide_reset` both begin a fresh, low-tide mission run; `tide_hold`, `tide_resume`, and `tide_rise` affect tide progression only. |
| `/points` | Simulation | Autonomy, Operator | `sensor_msgs/PointCloud2` | 16-channel 3D LiDAR, frame `lidar_link`, 10 Hz, 0.3–30 m. |
| `/camera/image_raw`, `/camera/camera_info` | Simulation | Operator | `sensor_msgs/Image`, `CameraInfo` | Front camera, frame `camera_link`, 320×240 at 5 Hz (demo profile). |
| `/camera/{rear,left,right}/image_raw`, `/camera/{rear,left,right}/camera_info` | Simulation (Version 3, with `cameras:=all`) | Operator | `sensor_msgs/Image`, `CameraInfo` | Mast cameras below the LiDAR, tilted 15° down, 320×240 at 5 Hz. Frames `camera_rear_link`, `camera_left_link`, `camera_right_link`. Only rendered while bridged. |
| `/gps/fix` | Simulation | Operator | `sensor_msgs/NavSatFix` | Simulated GNSS (world origin at Sungei Buloh, 1.4466 N 103.7300 E), 10 Hz. |
| `/tf`, `/tf_static` | Simulation, `robot_state_publisher` | All | `tf2_msgs/TFMessage` | `map → base_link` from odometry; `base_link →` every vehicle part and sensor from the URDF and `/joint_states`. |
| `/joint_states` | Simulation | `robot_state_publisher`, `vehicle_mobility_node` | `sensor_msgs/JointState` | Version 2: track retract joints, fans, rudders, puff-port shutters (the mobility node reads the track positions). Version 1: legs, suspension, wheels, fans, rudders. |
| `/robot_description` | `robot_state_publisher` | Operator (RViz / Foxglove) | `std_msgs/String` | Vehicle URDF (meshes as `package://tidal_vehicle_description/...`). |
| `/vehicle/mode` | Simulation (`vehicle_mobility_node`) | Operator, Evaluation | `std_msgs/String` | Public mobility mode: `TRACK`, `TRANSITION` or `HOVER`. |
| `/vehicle/collision` | Simulation (`collision_monitor`, Version 3) | Safety, Operator, Evaluation | `tidal_vehicle_interfaces/Collision` | A detected hit. `source` is `contact` (contact sensors on the hull, skirt or tracks; ground contact is filtered out) or `imu` (a horizontal jolt above 6 m/s²). Also gives `part`, `side` (front, rear, left, right), `other` (the model hit, if known) and `strength`. Each hit is reported at most once per second. |
| `/clock` | Simulation | All | `rosgraph_msgs/Clock` | Simulation time; every node runs with `use_sim_time: true`. |

Topics under `/vehicle/*` other than `/vehicle/mode` are internal to the vehicle simulation (mobility node ⇄ Gazebo) and are not a cross-workstream contract. They are listed in `tidal_vehicle_bringup/config/ros_gz_bridge_v2.yaml` (Version 2, which adds `/vehicle/cmd_vel_tracks`, `/vehicle/tracks_cmd` and `/vehicle/lift_share`) and `ros_gz_bridge.yaml` (Version 1).

## Frames

```text
map ──(/odom, /tf: ground-truth pose)──► base_link ──► lidar_link, camera_link, imu_link,
                                                       skirt, payload_box, fans, rudders,
                                                       track_left, track_right        (Version 2)
                                                       wheel_leg_* ► wheel_shock_* ► wheel_*  (Version 1)
```

`/odom`, `/terrain_costmap`, `/mission_goal` and `/planned_path` all use `map`. Odometry is ground truth, so there is no separate `odom` frame. `base_link` is the hull at its centre. On Version 2 it is 0.50 m above ground on the tracks and 0.52 m when hovering; the LiDAR sits on the centre mast directly above it (`lidar_link` at 1.44 m on the tracks). On Version 1 it is 0.30 m above ground on wheels and 0.24 m when hovering. Frame names match `tidal_vehicle_description/config/sensors.yaml`.

## Safety states

`CRUISE`, `CAUTION`, `HOLD` and `RETURN` are the required state names. The safety supervisor must publish a human-readable reason with every state change.

## Terrain-cost semantics

`/terrain_costmap` uses one interpretation across Simulation, Autonomy and
Safety:

| Cost | Meaning | Mobility used by vehicle controller |
| --- | --- | --- |
| `0`--`19` | Firm shore | `HOVER`; Version 2 may select `TRACK` for a sustained measured forward climb at or above 14°, Version 3 at or above 8° (Version 1: `WHEEL`) |
| `20`--`29` | Open, surveyed water (no roots or debris): fast travel allowed | `HOVER` |
| `30`--`59` | Mud, shallow water, root or debris zones | `HOVER` |
| `60`--`89` | Elevated-risk mud or shallow water | Conservative `HOVER` |
| `90`--`100` | No-go | None |
| `-1` | Unknown/no-go | None |

Safety samples these bands along `/return_path` using the conservative HOVER
energy/speed profile. A 2D cost map cannot know whether a firm segment has the
rare steep forward climb that deploys tracks (≥14° for Version 2, ≥8° for Version 3).

The `20`--`29` / `30`--`59` split was added on 2026-09-26 for Version 3's
zone speed limits. The limits are: firm shore 15 km/h; open surveyed water
cruise 30 km/h, max 50 km/h; mud, shallow water, roots and debris 10 km/h;
elevated risk 10 km/h (see `AGENTS.md`, *Shared terrain-cost semantics*). Band
checks at `19`, `60` and `90` are unchanged, so existing consumers keep working.
Water is published at `30` until the environment workstream marks surveyed
open water `20`--`29`.

## Vehicle-fault semantics

`/vehicle_health.fault` is empty during normal operation. The vehicle
controller publishes a non-empty fault only for an unsafe failed condition,
including `transition_timeout`, `hover_not_ready`, `lift_fan_fault`,
`track_settle_timeout` and `track_deployment_fault` (Version 1:
`wheel_settle_timeout` and `wheel_deployment_fault`). Safety treats any
non-empty fault as `HOLD`. Normal internal `TRANSITION` activity is not a fault
and does not create a new external topic or safety state.

Safety keeps the following **internal-only** mission phases; they are not extra
public safety states: `PRELAUNCH`, `OUTBOUND`, `DELIVERED`, and `RETURNING`.
Autonomy publishes `/mission_event` at each meaningful lifecycle transition:

| Event | Safety effect |
| --- | --- |
| `delivery_confirmed` | Set internal phase to `DELIVERED`, then request the return route. |
| `mission_complete` | Stop the vehicle and reset to `PRELAUNCH`. |
| `mission_reset` | Stop the vehicle and reset to `PRELAUNCH`. |

Autonomy emits `delivery_confirmed` when odometry reaches the outbound path endpoint and `mission_complete` when it reaches the HOME path endpoint. On completion or reset, it publishes empty active paths so the path follower proposes a stop.

An operator may replace `/mission_goal` while the mission is `OUTBOUND`.
Safety temporarily withholds autonomous motion until Autonomy publishes the
fresh outbound route; Autonomy then replans from the vehicle's current pose.
Once delivery is confirmed, or any return is latched, later delivery-goal
updates are rejected and the HOME route remains authoritative.

`/mission_event` is reliable and transient-local, retaining the most recent
lifecycle transition for late-starting Safety and operator consumers. Autonomy
also publishes a reliable, transient-local `/mission_event_history` JSON
snapshot containing the latest 20 events for the browser dashboard; this is a
small demo-facing retained timeline, not a general event database.

When Safety sets `return_required=true`, Autonomy must stop proposing the
outbound route and publish a freshly stamped `/return_path` to HOME. The same
HOME route is published on `/planned_path` as the path follower's active route.
Safety accepts the return route only when it arrives after the return request,
ends within the configured HOME tolerance, and remains valid against the latest
terrain cost map. Until then, Safety publishes a zero command. Every terrain
cost-map update requires a fresh path publication even when A* selects the same
cells. While returning, Autonomy refreshes `/return_path` without republishing
`/planned_path`, preventing callback ordering from resetting freshness or path
follower progress. A later false `return_required` value does not clear the latched return;
the `reset` or `tide_reset` scenario event clears it.

## Control-source selection

Autonomous mode is the default: Safety evaluates and gates
`/cmd_vel_proposed`. When `/operator_remote_enabled` is true, Safety ignores
Autonomy's proposed motion and instead gates `/operator_cmd_vel`. Remote mode
intentionally makes automatic route, tide, return-margin and `RETURN`/`HOLD`
policy decisions advisory to the operator; it does not allow a direct vehicle
command path. Stale vehicle telemetry, a non-empty vehicle-controller fault,
or a stale hold-to-run remote command still publish zero `/cmd_vel`.

The dashboard's explicit **Abort mission and return home** action publishes
`operator_abort`, disables remote mode and restores the normal autonomous
return policy.

Safety evaluates telemetry, command and route freshness using ROS time, which is Gazebo simulation time in the common launch.

`VehicleHealth` is raw simulation telemetry. Safety calculates the estimated
return energy, return margin and return ETA from `/return_path`,
`/terrain_costmap`, battery and mobility health, then publishes those derived
values in `/safety_status`.

`/scenario_event` is reserved for deterministic evaluation controls. The
supported mission values are `operator_abort` (request a controlled return) and
`reset` or `tide_reset` (stop, return to low tide, and reset Safety and
Autonomy for a new mission). `tide_rise`, `tide_hold`, and `tide_resume`
control tide progression only. These synchronously control the rendered water,
physics-side water level, `/terrain_state`, and `/terrain_costmap`.
Simulation-specific fault
injection remains owned by Evaluation and Simulation and should be reflected in
`/vehicle_health` or `/terrain_state`.

## Rising-tide update rule

The simulation publishes a new terrain state and cost map whenever the simulated tide changes. Autonomy must re-evaluate the outbound and return route from the current pose; it must not assume that the route accepted at launch remains safe. Safety uses the same updated terrain information to determine whether a return remains feasible.

## Terrain cost-map encoding

`/terrain_costmap` uses standard `nav_msgs/OccupancyGrid` row-major layout in the `map` frame for the first integration slice. Its cell values are semantic traversal costs, not probability of occupancy:

- `0`--`89`: traversable, with larger values representing increasing terrain risk.
- `90`--`100`: no-go terrain or obstacle.
- `-1`: unknown terrain, treated as no-go by autonomy.

Until the common TF tree is integrated, `/terrain_costmap`, `/odom`, `/mission_goal` and `/planned_path` must have matching frame IDs. The global planner publishes `/planned_path`; the path follower combines it with `/odom` and publishes `/cmd_vel_proposed`. The safety supervisor remains the only publisher of `/cmd_vel`.

## LiDAR obstacle update rule

Autonomy treats `/terrain_costmap` as the Simulation-owned base map. Known static tree and rock collision footprints are published there as cost `100` no-go cells. Autonomy must not republish or modify that source map. Valid finite `/scan` returns within the sensor minimum range and Autonomy's configured maximum range are projected into base-map cells, inflated by the configured safety radius and overlaid as temporary no-go cells for route planning.

Each accepted scan replaces the previous temporary obstacle set. A changed set causes immediate route reassessment; a clear scan removes prior LiDAR cells. If the overlay blocks every route, the global planner publishes an empty `/planned_path` to stop the path follower's previous proposal.

Until the common TF tree and final sensor mounting are integrated, Autonomy assumes the LiDAR origin matches the odometry position and its zero angle points along the vehicle's forward axis. The initial 1.7 m inflation radius represents an estimated 2.5 m by 1.5 m footprint plus about 0.25 m clearance. Person 4 owns the final collision geometry, sensor pose and frame publication; Person 1 must use those values once available.
