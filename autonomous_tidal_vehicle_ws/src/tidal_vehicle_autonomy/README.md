# tidal_vehicle_autonomy

Owns route proposals, path following, local obstacle response and delivery mission logic. It publishes proposals only and must not bypass the safety supervisor.

## Runnable components

### global_planner

The global planner listens to /terrain_costmap, /odom, /mission_goal and /terrain_state, then publishes a terrain-aware /planned_path.

For the first integration slice, map, odometry and goal must use the same frame, normally map. The node refuses to mix frames until the common transform tree is available.

Terrain cost-map encoding is fixed as follows:

- 0 through 89: traversable; higher values carry higher route cost.
- 90 through 100: no-go terrain or obstacle.
- -1: unknown and no-go.

If replanning makes a previously published route unsafe, the planner publishes an empty path to invalidate it.

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

1. Consume /scan and create a local obstacle representation.
2. Replan around an injected obstacle.
3. Respect the Safety return instruction while publishing current outbound and return routes against terrain state.
4. Tune path-following parameters against the integrated simulated vehicle.

## Safety-return integration

Autonomy subscribes to /safety_status. When return_required is true, it must latch the request, stop proposing the outbound route, and publish a freshly computed, stamped /return_path from the current pose to HOME. Recalculate and republish both outbound and return paths whenever the terrain cost map changes. Autonomy must not clear a safety return request; only mission_reset starts a new outbound mission.
