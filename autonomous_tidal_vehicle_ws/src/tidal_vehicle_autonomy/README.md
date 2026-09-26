# tidal_vehicle_autonomy

Owns route proposals, path following, local obstacle response and delivery mission logic. It publishes proposals only and must not bypass the safety supervisor.

## Runnable components

### global_planner

By default the global planner listens to /terrain_costmap, /odom, /mission_goal, /terrain_state, /scan, /safety_status and reset scenario events. It publishes the active /planned_path, Safety's /return_path and lifecycle /mission_event messages.

For the first integration slice, map, odometry and goal must use the same frame, normally map. The node refuses to mix frames until the common transform tree is available.

Terrain cost-map encoding is fixed as follows:

- 0 through 89: traversable; higher values carry higher route cost.
- 90 through 100: no-go terrain or obstacle.
- -1: unknown and no-go.

If replanning makes a previously published route unsafe, the planner publishes an empty path to invalidate it.

The Simulation-owned /terrain_costmap is never modified. Before A*, its static no-go and unknown cells are inflated in a private planning copy by the selected vehicle's clearance radius. LiDAR measurements are projected into the terrain grid using the vehicle pose from /odom. A scan return already inside that static exclusion area is not added again: mapped rocks and trunks remain visible, but are not double-inflated. Unmapped raw hit cells must be detected in two consecutive scans before they are inflated by the same radius and added as a temporary overlay; five misses clear a hit. New cells replan only when they intersect the active outbound/return route. Unrelated removals retain the current safe detour, while a cleared overlay or blocked mission triggers route recovery.

The active route remains geometrically stable while the vehicle follows it. Odometry movement updates Safety's prospective return path without rebuilding the follower's active path, and repeated cost maps with unchanged geometry and costs refresh route timestamps without rerunning A*. An actual terrain-cost change, a relevant confirmed obstacle, a new goal or a Safety return request still produces a fresh active route.

For the initial integration, the vehicle footprint is estimated as 2.5 m long by 1.5 m wide, and the LiDAR is assumed to be at the odometry position and aligned with the vehicle's forward axis. Use Person 4's final collision geometry and sensor transform when they are ready.

Global-planner LiDAR parameters:

| Parameter | Default | Meaning |
| --- | ---: | --- |
| obstacle_inflation_radius_m | 0.75 m | Static and LiDAR no-go clearance; shared launch uses 1.7 m for V2 and 2.0 m for V3. |
| obstacle_min_range_m | 8.0 m | Minimum useful local look-ahead. |
| obstacle_max_range_m | 8.0 m | Farthest scan return used by local planning. |
| obstacle_brake_decel_mps2 | 1.0 m/s² | Deceleration used to grow scan range with stopping distance. |
| obstacle_reaction_time_s | 1.0 s | Reaction distance allowed before braking. |
| obstacle_confirmation_scans | 2 | Consecutive detections required before a cell becomes blocked. |
| obstacle_clear_scans | 5 | Consecutive misses required before a blocked cell is removed. |

For `vehicle:=v3`, the current tuning profile sets `lidar_only_navigation:=true`: Autonomy does not subscribe to, sample or assign route cost from `/terrain_costmap`. It uses a fixed 1 m bounded planning grid (`[-12, 108]` m x `[-30, 30]` m in `map`) solely to express confirmed, inflated LiDAR returns as no-go cells. Unseen space is treated as free, so this is deliberately capped at 1.2 m/s while steering and vehicle PID tuning are underway. V3 uses the full 30 m LiDAR horizon at every speed, with 2 m clearance, two-scan confirmation and a 1.0 m/s² braking assumption. The path follower's terrain-zone speed limits are also disabled in this profile; it continues to limit speed for sensor range, curve geometry and yaw behaviour. In V3, Safety and the dashboard use the cost map as an operator reference only. Versions 1 and 2 keep their existing behavior.

LiDAR-only planner parameters:

| Parameter | Default | Meaning |
| --- | ---: | --- |
| lidar_only_navigation | false | Use a bounded blank grid plus confirmed LiDAR obstacles instead of `/terrain_costmap`. |
| lidar_only_bounds_m | `[-12, 108, -30, 30]` m | `[min_x, max_x, min_y, max_y]` extent of that grid. Bounds must align exactly with the resolution. |
| lidar_only_resolution_m | 1.0 m | Cell size of the LiDAR-only planning grid. |

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

- projects the vehicle onto the safe route corridor and interpolates a target
  ahead along it, using a speed-scaled lookahead on Version 3;
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
| max_angular_speed | 0.45 rad/s | Maximum turning proposal. |
| lookahead_distance | 0.75 m | Distance used to select the tracking point. |
| goal_tolerance | 0.25 m | Distance at which the goal counts as reached. |
| heading_gain | 0.9 | Proportional turning gain. |
| slow_down_distance | 1.0 m | Distance over which forward speed reduces near the goal. |
| rotate_in_place_angle | 1.05 rad | Heading error that stops forward motion while turning. |
| odom_timeout | 0.5 s | Maximum age of odometry before proposing a stop. |
| zone_speed_limits_mps | [0.0] | Four terrain-band limits; disabled by default. |
| brake_decel_mps2 | 1.0 m/s² | Shared zone/sensor stopping assumption. |
| lateral_accel_limit_mps2 | 0.0 | Curve and moving-yaw limit; disabled by default. |
| yaw_rate_damping | 0.0 | Measured-yaw feedback used to settle turns; disabled by default. |
| corner_preview_sample_distance_m | 0.0 m | Route-curvature preview used to brake before turns; disabled by default. |
| lookahead_time_s | 0.0 s | Speed-scaled preview; disabled by default. |
| reaction_time_s | 1.0 s | Reaction time included in stopping reach. |
| obstacle_detection_range_m | 0.0 m | Sensor-based speed ceiling; disabled by default. |
| obstacle_clearance_m | 0.0 m | Clearance subtracted from usable detection range. |

V3's current tuning profile has a deliberately conservative 1.2 m/s forward and 0.30 rad/s yaw ceiling, 2.0 m lookahead with 1.5 s speed scaling, 4.0 m corner preview, a 0.35 m/s² lateral limit, 0.8 measured-yaw damping, 5 m final slowdown, 30 m detection and 2 m clearance. It brakes before upcoming sharp turns and, at a 0.35 rad route-heading error while still moving, proposes up to 0.45 m/s reverse thrust to arrest momentum before the recovery turn. It continues enforcing the curve limit while turning, and stops forward motion for heading errors of 0.60 rad or more. Safety remains the only `/cmd_vel` publisher and may clamp that proposal.

## Remaining milestones

1. Re-run the V3 corridor mission using the LiDAR-only, conservative steering profile, recording path error, yaw overshoot, obstacle detours and collisions before increasing its 1.2 m/s cap.
2. Verify the final 0.3 m delivery/HOME event tolerance in the corrected corridor; the static V3 integration mission already completes.
3. Keep the sensor-safe speed and braking assumptions aligned if Person 4 changes LiDAR range, footprint or braking performance.

## Safety-return integration

Autonomy subscribes to /safety_status. When return_required is true, it latches the request, stops proposing the outbound route, and publishes a freshly computed, stamped /return_path from the current pose to HOME. The same route becomes the active /planned_path. Both outbound and return paths are recomputed and republished whenever the terrain cost map changes. Autonomy does not clear a safety return request in response to a later false status; the reset scenario and resulting mission_reset start a new outbound mission.
