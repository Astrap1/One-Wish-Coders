# `tidal_vehicle_autonomy`

Owns the route proposal, local obstacle response and delivery mission logic. It publishes proposals only; it must not bypass the safety supervisor.

## Initial milestones

1. Load a semantic terrain-cost map and publish a route to a mission goal.
2. Consume `/scan` and create a local obstacle representation.
3. Replan around an injected obstacle.
4. Respect the return-energy constraint provided by vehicle health and terrain state.
