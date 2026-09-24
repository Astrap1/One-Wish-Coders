# `tidal_vehicle_simulation`

Owns the Gazebo tidal-corridor environment and the transparent abstractions that turn tide, terrain and vehicle condition into simulation inputs.

## Deliverables

- `worlds/` — firm shore, mudflat, shallow-water and mangrove corridor scenes.
- `models/` — roots, debris, delivery zone and other reusable world assets.
- `plugins/` — terrain cost, tide-state and mobility-health plugins.
- `config/` — repeatable scenario parameters.
- `launch/` — standalone simulation launch entry points.

The first complete world must provide `/scan`, `/imu`, `/odom`, `/terrain_state` and `/vehicle_health` through the agreed interface contract.
