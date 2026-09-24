# `tidal_vehicle_autonomy`

Owns the route proposal, local obstacle response and delivery mission logic. It publishes proposals only; it must not bypass the safety supervisor.

## First runnable component

`global_planner` listens to `/terrain_costmap`, `/odom`, `/mission_goal` and `/terrain_state`, then publishes a terrain-aware `/planned_path`. It publishes no velocity commands.

For the first integration slice, map, odometry and goal must use the same frame (normally `map`). The node deliberately refuses to mix frames until the common transform tree is available.

Terrain cost-map encoding is fixed as follows:

- `0`--`89`: traversable; higher values carry higher route cost.
- `90`--`100`: no-go terrain or obstacle.

## Initial milestones

1. Load a semantic terrain-cost map and publish a route to a mission goal.
2. Consume `/scan` and create a local obstacle representation.
3. Replan around an injected obstacle.
4. Respect the return-energy constraint provided by vehicle health and terrain state.
