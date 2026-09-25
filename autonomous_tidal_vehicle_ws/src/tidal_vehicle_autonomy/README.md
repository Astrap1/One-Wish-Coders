# tidal_vehicle_autonomy

Owns route proposals, path following, local obstacle response and delivery mission logic. It publishes proposals only and must not bypass the safety supervisor.

## Runnable components

### global_planner

The global planner listens to /terrain_costmap, /odom, /mission_goal, /terrain_state, /scan, /safety_status and reset scenario events. It publishes the active /planned_path, Safety's /return_path and lifecycle /mission_event messages.

For the first integration slice, map, odometry and goal must use the same frame, normally map. The node refuses to mix frames until the common transform tree is available.

Terrain cost-map encoding is fixed as follows:

- 0 through 89: traversable; higher values carry higher route cost.
- 90 through 100: no-go terrain or obstacle.
- -1: unknown and no-go.

If replanning makes a previously published route unsafe, the planner publishes an empty path to invalidate it.

LiDAR measurements are projected into the terrain grid using the vehicle pose from /odom. Valid detections are inflated and added to an internal planning copy of the terrain map. A changed obstacle set triggers replanning; an obstacle-free scan removes the prior temporary cells. The Simulation-owned /terrain_costmap is never modified.

For the initial integration, the vehicle footprint is estimated as 2.5 m long by 1.5 m wide, and the LiDAR is assumed to be at the odometry position and aligned with the vehicle's forward axis. Use Person 4's final collision geometry and sensor transform when they are ready.

Global-planner LiDAR parameters:

| Parameter | Default | Meaning |
| --- | ---: | --- |
| obstacle_inflation_radius_m | 0.75 m | Version 1 vehicle half-diagonal plus a small clearance; recalibrate for Version 2. |
| obstacle_max_range_m | 8.0 m | Farthest scan return used by local planning. |

Return behavior:

- `return_required=true` latches return mode and replaces the outbound route with a fresh route to HOME;
- the return route is published on both /return_path for Safety and /planned_path for the follower;
- every terrain cost-map update republishes a newly stamped route, including when the selected cells are unchanged;
- /return_path is refreshed at 2 Hz while returning so Safety cannot see a stale route because of callback ordering;
- reaching the delivery endpoint publishes `delivery_confirmed`;
- reaching HOME publishes `mission_complete` and empty routes;
- the existing `reset` scenario event clears the latch and publishes `mission_reset`.

Global-planner return parameters:

| Parameter | Default | Meaning |
| --- | ---: | --- |
| home_x_m | 0.0 m | HOME x coordinate. Must match Safety. |
| home_y_m | 0.0 m | HOME y coordinate. Must match Safety. |
| home_frame | map | Frame containing HOME. |
| goal_event_tolerance_m | 0.3 m | Endpoint distance that triggers a mission event. |
| return_path_refresh_rate_hz | 2.0 Hz | Return-only refresh rate for Safety's freshness check. |

### path_follower

The path follower listens to /planned_path and /odom, then publishes forward and turning proposals on /cmd_vel_proposed at 10 Hz. Person 2's safety supervisor remains the only publisher of /cmd_vel.

The controller:

- selects a lookahead point in front of the vehicle;
- turns toward that point;
- stops forward motion while the heading error is large;
- slows near the final goal;
- stops when the goal is reached;
- publishes zero motion for an empty path, stale odometry or mismatched frames.

Run the components after building and sourcing the workspace:

    ros2 run tidal_vehicle_autonomy global_planner
    ros2 run tidal_vehicle_autonomy path_follower

Path-follower parameters:

| Parameter | Default | Meaning |
| --- | ---: | --- |
| control_rate_hz | 10.0 | Proposed-command publication rate. |
| max_linear_speed | 0.8 m/s | Maximum forward proposal. |
| max_angular_speed | 1.0 rad/s | Maximum turning proposal. |
| lookahead_distance | 0.75 m | Distance used to select the tracking point. |
| goal_tolerance | 0.25 m | Distance at which the goal counts as reached. |
| heading_gain | 1.5 | Proportional turning gain. |
| slow_down_distance | 1.0 m | Distance over which forward speed reduces near the goal. |
| rotate_in_place_angle | 0.7 rad | Heading error that stops forward motion while turning. |
| odom_timeout | 0.5 s | Maximum age of odometry before proposing a stop. |

## Remaining milestones

1. Replace the initial LiDAR pose assumption with Person 4's final sensor transform.
2. Tune LiDAR inflation, usable range and path-following parameters against the integrated simulated vehicle.
3. Verify the HOME parameters against Person 4's final world and Person 2's launch configuration.

## Safety-return integration

Autonomy subscribes to /safety_status. When return_required is true, it latches the request, stops proposing the outbound route, and publishes a freshly computed, stamped /return_path from the current pose to HOME. The same route becomes the active /planned_path. Both outbound and return paths are recomputed and republished whenever the terrain cost map changes. Autonomy does not clear a safety return request in response to a later false status; the reset scenario and resulting mission_reset start a new outbound mission.
