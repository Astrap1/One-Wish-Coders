# ROS 2 interface contract

These are the initial contracts between workstreams. Topic names and message types may evolve, but update this file in the same change whenever they do.

| Topic | Publisher | Consumer | Initial type | Purpose |
| --- | --- | --- | --- | --- |
| `/scan` | Simulation | Autonomy | `sensor_msgs/LaserScan` | Near-field obstacle sensing used for the planner's temporary obstacle overlay. |
| `/imu` | Simulation | Autonomy | `sensor_msgs/Imu` | Orientation and motion sensing. |
| `/odom` | Simulation | Autonomy, Safety, Operator | `nav_msgs/Odometry` | Vehicle position and velocity. |
| `/terrain_state` | Simulation | Autonomy, Safety | `tidal_vehicle_interfaces/TerrainState` | Tide and traversability estimate. |
| `/terrain_costmap` | Simulation | Autonomy, Safety, Operator | `nav_msgs/OccupancyGrid` | Current terrain-risk map after the simulated tide update. |
| `/vehicle_health` | Simulation | Safety, Operator | `tidal_vehicle_interfaces/VehicleHealth` | Raw battery, mobility, link and payload health. |
| `/mission_goal` | Operator | Autonomy, Safety | `geometry_msgs/PoseStamped` | Requested delivery point. |
| `/planned_path` | Autonomy | Operator, Safety | `nav_msgs/Path` | Proposed route. |
| `/return_path` | Autonomy | Safety, Operator | `nav_msgs/Path` | Fresh route from current pose to the fixed HOME zone. |
| `/cmd_vel_proposed` | Autonomy | Safety | `geometry_msgs/Twist` | Motion proposal before safety approval. |
| `/cmd_vel` | Safety | Simulation | `geometry_msgs/Twist` | Safety-approved motion command. |
| `/safety_status` | Safety | Autonomy, Operator, Evaluation | `tidal_vehicle_interfaces/SafetyStatus` | State, rationale, return requirement and Safety-calculated return energy, margin and ETA. Autonomy must act on `return_required=true`. |
| `/mission_event` | Autonomy | Safety, Operator, Evaluation | `std_msgs/String` | Explicit lifecycle event used by Safety for its internal mission phase. |
| `/scenario_event` | Evaluation | Simulation, Safety | `std_msgs/String` | Controlled fault or scenario event. |

## Safety states

`CRUISE`, `CAUTION`, `HOLD` and `RETURN` are the required state names. The safety supervisor must publish a human-readable reason with every state change.

## Terrain-cost semantics

`/terrain_costmap` uses one interpretation across Simulation, Autonomy and
Safety:

| Cost | Meaning | Mobility used by vehicle controller |
| --- | --- | --- |
| `0`--`19` | Firm shore | `WHEEL` |
| `20`--`59` | Mud or shallow water | `HOVER` |
| `60`--`89` | Elevated-risk mud or shallow water | Conservative `HOVER` |
| `90`--`100` | No-go | None |
| `-1` | Unknown/no-go | None |

Safety samples these bands along `/return_path`: wheel and hover segments use
different declared energy/speed assumptions, and each wheel--hover mode change
adds transition energy and time to the return ETA.

## Vehicle-fault semantics

`/vehicle_health.fault` is empty during normal operation. The vehicle
controller publishes a non-empty fault only for an unsafe failed condition,
including `transition_timeout`, `hover_not_ready`, `lift_fan_fault`,
`wheel_settle_timeout` and `wheel_deployment_fault`. Safety treats any
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

When Safety sets `return_required=true`, Autonomy must stop proposing the
outbound route and publish a freshly stamped `/return_path` to HOME. Safety
accepts that route only when it arrives after the return request, ends within
the configured HOME tolerance, and remains valid against the latest terrain
cost map. Until then, Safety publishes a zero command.

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
