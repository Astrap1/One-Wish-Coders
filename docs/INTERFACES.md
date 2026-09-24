# ROS 2 interface contract

These are the initial contracts between workstreams. Topic names and message types may evolve, but update this file in the same change whenever they do.

| Topic | Publisher | Consumer | Initial type | Purpose |
| --- | --- | --- | --- | --- |
| `/scan` | Simulation | Autonomy | `sensor_msgs/LaserScan` | Near-field obstacle sensing. |
| `/imu` | Simulation | Autonomy | `sensor_msgs/Imu` | Orientation and motion sensing. |
| `/odom` | Simulation | Autonomy, Safety, Operator | `nav_msgs/Odometry` | Vehicle position and velocity. |
| `/terrain_state` | Simulation | Autonomy, Safety | `tidal_vehicle_interfaces/TerrainState` | Tide and traversability estimate. |
| `/terrain_costmap` | Simulation | Autonomy, Operator | `nav_msgs/OccupancyGrid` | Current terrain-risk map after the simulated tide update. |
| `/vehicle_health` | Simulation | Safety, Operator | `tidal_vehicle_interfaces/VehicleHealth` | Battery, mobility, link and payload health. |
| `/mission_goal` | Operator | Autonomy, Safety | `geometry_msgs/PoseStamped` | Requested delivery point. |
| `/planned_path` | Autonomy | Operator, Safety | `nav_msgs/Path` | Proposed route. |
| `/cmd_vel_proposed` | Autonomy | Safety | `geometry_msgs/Twist` | Motion proposal before safety approval. |
| `/cmd_vel` | Safety | Simulation | `geometry_msgs/Twist` | Safety-approved motion command. |
| `/safety_status` | Safety | Operator, Evaluation | `tidal_vehicle_interfaces/SafetyStatus` | State, rationale and return requirement. |
| `/scenario_event` | Evaluation | Simulation, Safety | `std_msgs/String` | Controlled fault or scenario event. |

## Safety states

`CRUISE`, `CAUTION`, `HOLD` and `RETURN` are the required state names. The safety supervisor must publish a human-readable reason with every state change.

## Rising-tide update rule

The simulation publishes a new terrain state and cost map whenever the simulated tide changes. Autonomy must re-evaluate the outbound and return route from the current pose; it must not assume that the route accepted at launch remains safe. Safety uses the same updated terrain information to determine whether a return remains feasible.

## Terrain cost-map encoding

`/terrain_costmap` uses standard `nav_msgs/OccupancyGrid` row-major layout in the `map` frame for the first integration slice. Its cell values are semantic traversal costs, not probability of occupancy:

- `0`--`89`: traversable, with larger values representing increasing terrain risk.
- `90`--`100`: no-go terrain or obstacle.
- `-1`: unknown terrain, treated as no-go by autonomy.

Until the common TF tree is integrated, `/terrain_costmap`, `/odom` and `/mission_goal` must have matching frame IDs. The global planner will publish only `/planned_path`; `/cmd_vel_proposed` remains a later path-following component.
