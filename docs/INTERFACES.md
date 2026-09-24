# ROS 2 interface contract

These are the initial contracts between workstreams. Topic names and message types may evolve, but update this file in the same change whenever they do.

| Topic | Publisher | Consumer | Initial type | Purpose |
| --- | --- | --- | --- | --- |
| `/scan` | Simulation | Autonomy | `sensor_msgs/LaserScan` | Near-field obstacle sensing. |
| `/imu` | Simulation | Autonomy | `sensor_msgs/Imu` | Orientation and motion sensing. |
| `/odom` | Simulation | Autonomy, Safety, Operator | `nav_msgs/Odometry` | Vehicle position and velocity. |
| `/terrain_state` | Simulation | Autonomy, Safety | `tidal_vehicle_interfaces/TerrainState` | Tide and traversability estimate. |
| `/vehicle_health` | Simulation | Safety, Operator | `tidal_vehicle_interfaces/VehicleHealth` | Battery, mobility, link and payload health. |
| `/mission_goal` | Operator | Autonomy, Safety | `geometry_msgs/PoseStamped` | Requested delivery point. |
| `/planned_path` | Autonomy | Operator, Safety | `nav_msgs/Path` | Proposed route. |
| `/cmd_vel_proposed` | Autonomy | Safety | `geometry_msgs/Twist` | Motion proposal before safety approval. |
| `/cmd_vel` | Safety | Simulation | `geometry_msgs/Twist` | Safety-approved motion command. |
| `/safety_status` | Safety | Operator, Evaluation | `tidal_vehicle_interfaces/SafetyStatus` | State, rationale and return requirement. |
| `/scenario_event` | Evaluation | Simulation, Safety | `std_msgs/String` | Controlled fault or scenario event. |

## Safety states

`CRUISE`, `CAUTION`, `HOLD` and `RETURN` are the required state names. The safety supervisor must publish a human-readable reason with every state change.
