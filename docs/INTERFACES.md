# ROS 2 interface contract

These are the initial contracts between workstreams. Topic names and message types may evolve, but update this file in the same change whenever they do.

| Topic | Publisher | Consumer | Initial type | Purpose |
| --- | --- | --- | --- | --- |
| `/scan` | Simulation (`lidar_scan_node`) | Autonomy | `sensor_msgs/LaserScan` | Best-effort sensor QoS. Near-field obstacle sensing used for the planner's temporary obstacle overlay. 2D scan in `lidar_link`, 720 bins, derived from `/points` (closest return per bearing between ~7 cm and 1.2 m above ground; the vehicle's own body is filtered out). 10 Hz. |
| `/imu` | Simulation | Autonomy | `sensor_msgs/Imu` | Orientation and motion sensing. Frame `imu_link`, 50 Hz. |
| `/odom` | Simulation (Gazebo OdometryPublisher) | Autonomy, Safety, Operator | `nav_msgs/Odometry` | Vehicle position and velocity. Ground truth (idealised), `header.frame_id: map`, `child_frame_id: base_link`, 50 Hz. Valid in both hover and ground mode. |
| `/terrain_state` | Simulation | Autonomy, Safety | `tidal_vehicle_interfaces/TerrainState` | Tide and traversability estimate. Person 3's `tide_manager.py` publishes this for the corridor. The vehicle test launch uses a 10 Hz low-tide placeholder; disable it whenever the tide manager runs so there is one publisher. |
| `/terrain_costmap` | Simulation | Autonomy, Safety, Operator | `nav_msgs/OccupancyGrid` | Current terrain-risk map. The default vehicle integration world uses a static map; Person 3's tide manager supplies the changing map when the corridor is integrated. |
| `/vehicle_health` | Simulation (`vehicle_mobility_node`) | Safety, Operator | `tidal_vehicle_interfaces/VehicleHealth` | Raw battery, mobility, link and payload health at 10 Hz. Battery uses a stated Version 1 power model; mobility, link, payload and fault remain runtime fault-injection parameters. |
| `/mission_goal` | Operator | Autonomy, Safety | `geometry_msgs/PoseStamped` | Requested delivery point. |
| `/planned_path` | Autonomy | Path follower, Operator, Safety | `nav_msgs/Path` | Active proposed route: outbound normally, HOME route while return is latched. |
| `/return_path` | Autonomy | Safety, Operator | `nav_msgs/Path` | Prospective route from the current pose to HOME during outbound travel; freshly republished and activated when return is required. |
| `/cmd_vel_proposed` | Autonomy | Safety | `geometry_msgs/Twist` | Motion proposal before safety approval. |
| `/cmd_vel` | Safety | Simulation (`vehicle_mobility_node`) | `geometry_msgs/Twist` | Safety-approved motion command: `linear.x` (m/s, ≤ 2.5) and `angular.z` (rad/s, ≤ 1.0). Routed to the fans in HOVER mode or the Version 1 wheels in TRACK mode. A command older than 0.5 s means stop. |
| `/safety_status` | Safety | Autonomy, Operator, Evaluation | `tidal_vehicle_interfaces/SafetyStatus` | State, rationale, return requirement and Safety-calculated return energy, margin and ETA. Autonomy must act on `return_required=true`. |
| `/mission_event` | Autonomy | Safety, Operator, Evaluation | `std_msgs/String` | Explicit lifecycle event used by Safety for its internal mission phase. |
| `/scenario_event` | Evaluation | Simulation, Safety, Autonomy | `std_msgs/String` | Controlled fault or scenario event; Autonomy consumes the existing `reset` value. |
| `/points` | Simulation | Autonomy, Operator | `sensor_msgs/PointCloud2` | 16-channel 3D LiDAR, frame `lidar_link`, 10 Hz, 0.3–30 m. |
| `/camera/image_raw`, `/camera/camera_info` | Simulation | Operator | `sensor_msgs/Image`, `CameraInfo` | Front camera, frame `camera_link`, 640×480 at 15 Hz. |
| `/gps/fix` | Simulation | Operator | `sensor_msgs/NavSatFix` | Simulated GNSS (world origin at Sungei Buloh, 1.4466 N 103.7300 E), 10 Hz. |
| `/tf`, `/tf_static` | Simulation, `robot_state_publisher` | All | `tf2_msgs/TFMessage` | `map → base_link` from odometry; `base_link →` every vehicle part and sensor from the URDF and `/joint_states`. |
| `/joint_states` | Simulation | `robot_state_publisher` | `sensor_msgs/JointState` | Legs, suspension, wheels, fans, rudders. |
| `/robot_description` | `robot_state_publisher` | Operator (RViz / Foxglove) | `std_msgs/String` | Vehicle URDF (meshes as `package://tidal_vehicle_description/...`). |
| `/vehicle/mode` | Simulation (`vehicle_mobility_node`) | Operator, Evaluation | `std_msgs/String` | Public mobility mode: `TRACK`, `TRANSITION` or `HOVER`. |
| `/clock` | Simulation | All | `rosgraph_msgs/Clock` | Simulation time; every node runs with `use_sim_time: true`. |

Topics under `/vehicle/*` other than `/vehicle/mode` are internal to the vehicle simulation (mobility node ⇄ Gazebo) and are not a cross-workstream contract. They are listed in `tidal_vehicle_bringup/config/ros_gz_bridge.yaml`.

## Frames

```text
map ──(/odom, /tf: ground-truth pose)──► base_link ──► lidar_link, camera_link, imu_link,
                                                       skirt, payload_box, fans, rudders,
                                                       wheel_leg_* ► wheel_shock_* ► wheel_*
```

`/odom`, `/terrain_costmap`, `/mission_goal` and `/planned_path` all use `map`. Odometry is ground truth, so there is no separate `odom` frame. `base_link` is the hull at its centre: 0.30 m above ground on wheels, 0.24 m when hovering. Frame names match `tidal_vehicle_description/config/sensors.yaml`.

## Safety states

`CRUISE`, `CAUTION`, `HOLD` and `RETURN` are the required state names. The safety supervisor must publish a human-readable reason with every state change.

## Terrain-cost semantics

`/terrain_costmap` uses one interpretation across Simulation, Autonomy and
Safety:

| Cost | Meaning | Mobility used by vehicle controller |
| --- | --- | --- |
| `0`--`19` | Firm shore | `TRACK` (Version 1: `WHEEL`) |
| `20`--`59` | Mud or shallow water | `HOVER` |
| `60`--`89` | Elevated-risk mud or shallow water | Conservative `HOVER` |
| `90`--`100` | No-go | None |
| `-1` | Unknown/no-go | None |

Safety samples these bands along `/return_path`: track (Version 1: wheel) and
hover segments use different declared energy/speed assumptions, and each
ground--hover mode change adds transition energy and time to the return ETA.

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
the existing `reset` scenario event clears it.

Safety evaluates telemetry, command and route freshness using ROS time, which is Gazebo simulation time in the common launch.

`VehicleHealth` is raw simulation telemetry. Safety calculates the estimated
return energy, return margin and return ETA from `/return_path`,
`/terrain_costmap`, battery and mobility health, then publishes those derived
values in `/safety_status`.

`/scenario_event` is reserved for deterministic evaluation controls. The
initial supported values are `operator_abort` (request a controlled return) and
`reset` (stop and reset the safety supervisor). Simulation-specific fault
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

Autonomy treats `/terrain_costmap` as the Simulation-owned base map. It must not republish or modify that source map. Valid finite `/scan` returns within the sensor minimum range and Autonomy's configured maximum range are projected into base-map cells, inflated by the configured safety radius and overlaid as temporary no-go cells for route planning.

Each accepted scan replaces the previous temporary obstacle set. A changed set causes immediate route reassessment; a clear scan removes prior LiDAR cells. If the overlay blocks every route, the global planner publishes an empty `/planned_path` to stop the path follower's previous proposal.

Until the common TF tree and final sensor mounting are integrated, Autonomy assumes the LiDAR origin matches the odometry position and its zero angle points along the vehicle's forward axis. The initial 1.7 m inflation radius represents an estimated 2.5 m by 1.5 m footprint plus about 0.25 m clearance. Person 4 owns the final collision geometry, sensor pose and frame publication; Person 1 must use those values once available.
