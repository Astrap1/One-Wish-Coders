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

LiDAR measurements are projected into the terrain grid using the vehicle pose from /odom. Raw hit cells must be detected in two consecutive scans before they are footprint-inflated and added to an internal planning copy; five misses clear a hit. New cells replan only when they intersect the active outbound/return route. Unrelated removals retain the current safe detour, while a cleared overlay or blocked mission triggers route recovery. The Simulation-owned /terrain_costmap is never modified.

The active route remains geometrically stable while the vehicle follows it. Odometry movement updates Safety's prospective return path without rebuilding the follower's active path, and repeated cost maps with unchanged geometry and costs refresh route timestamps without rerunning A*. An actual terrain-cost change, a relevant confirmed obstacle, a new goal or a Safety return request still produces a fresh active route.

For the initial integration, the vehicle footprint is estimated as 2.5 m long by 1.5 m wide, and the LiDAR is assumed to be at the odometry position and aligned with the vehicle's forward axis. Use Person 4's final collision geometry and sensor transform when they are ready.

Global-planner LiDAR parameters:

| Parameter | Default | Meaning |
| --- | ---: | --- |
| obstacle_inflation_radius_m | 0.75 m | Standalone/V1 default; shared launch uses 1.7 m for V2 and 2.0 m for V3. |
| obstacle_min_range_m | 8.0 m | Minimum useful local look-ahead. |
| obstacle_max_range_m | 8.0 m | Farthest scan return used by local planning. |
| obstacle_brake_decel_mps2 | 1.0 m/s² | Deceleration used to grow scan range with stopping distance. |
| obstacle_reaction_time_s | 1.0 s | Reaction distance allowed before braking. |
| obstacle_confirmation_scans | 2 | Consecutive detections required before a cell becomes blocked. |
| obstacle_clear_scans | 5 | Consecutive misses required before a blocked cell is removed. |

For `vehicle:=v3`, the planner range grows from 8 m to the 30 m sensor limit using 1 s reaction, 1.0 m/s² braking and 2.0 m clearance. The follower uses the remaining 28 m as its usable stopping range, giving an effective autonomous ceiling of about 6.55 m/s (23.6 km/h). Versions 1 and 2 keep their existing behavior.

Return behavior:

- `return_required=true` latches return mode and replaces the outbound route with a fresh route to HOME;
- the return route is published on both /return_path for Safety and /planned_path for the follower;
- every terrain cost-map update republishes a newly stamped route; an unchanged map preserves the selected cells while a changed map reruns A*;
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
| goal_event_tolerance_m | 2.0 m | Delivery/HOME zone radius that triggers a mission event. |
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
| zone_speed_limits_mps | [0.0] | Four terrain-band limits; disabled by default. |
| brake_decel_mps2 | 1.0 m/s² | Shared zone/sensor stopping assumption. |
| lateral_accel_limit_mps2 | 0.0 | Curve and moving-yaw limit; disabled by default. |
| lookahead_time_s | 0.0 s | Speed-scaled preview; disabled by default. |
| reaction_time_s | 1.0 s | Reaction time included in stopping reach. |
| obstacle_detection_range_m | 0.0 m | Sensor-based speed ceiling; disabled by default. |
| obstacle_clearance_m | 0.0 m | Clearance subtracted from usable detection range. |

V3 overrides these with a 1.5 s preview, 1.0 m/s² lateral limit, 0.62 rad/s yaw ceiling, 8 m final slowdown, 30 m detection and 2 m clearance. While moving, yaw is additionally capped so `speed × |yaw_rate|` stays within the lateral-acceleration limit.

## Remaining milestones

1. Re-run the V3 corridor mission after Person 3 expands the cost-100 footprints to match the actual mangrove-root and rock collision/LiDAR extents.
2. Verify the final 0.3 m delivery/HOME event tolerance in that corrected corridor; the static V3 integration mission already completes.
3. Keep the sensor-safe speed and braking assumptions aligned if Person 4 changes LiDAR range, footprint or braking performance.

## Safety-return integration

Autonomy subscribes to /safety_status. When return_required is true, it latches the request, stops proposing the outbound route, and publishes a freshly computed, stamped /return_path from the current pose to HOME. The same route becomes the active /planned_path. Both outbound and return paths are recomputed and republished whenever the terrain cost map changes. Autonomy does not clear a safety return request in response to a later false status; the reset scenario and resulting mission_reset start a new outbound mission.
