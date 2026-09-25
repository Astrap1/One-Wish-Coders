# ROS 2 interface contract

These are the initial contracts between workstreams. Topic names and message types may evolve, but update this file in the same change whenever they do.

| Topic | Publisher | Consumer | Initial type | Purpose |
| --- | --- | --- | --- | --- |
| `/scan` | Simulation | Autonomy | `sensor_msgs/LaserScan` | Near-field obstacle sensing. |
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

Until the common TF tree is integrated, `/terrain_costmap`, `/odom` and `/mission_goal` must have matching frame IDs. The global planner will publish only `/planned_path`; `/cmd_vel_proposed` remains a later path-following component.
