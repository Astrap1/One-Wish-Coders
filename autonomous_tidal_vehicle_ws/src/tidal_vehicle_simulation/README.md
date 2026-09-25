# `tidal_vehicle_simulation`

Owns the Gazebo tidal-corridor environment and the transparent abstractions that turn tide, terrain and vehicle condition into simulation inputs.

## Vehicle simulation (Person 4)

See [`docs/VEHICLE_SIMULATION.md`](../../../docs/VEHICLE_SIMULATION.md).

- `plugins/` — `libtidal_vehicle_plugins.so`, C++ Gazebo systems:
  - `hover::AirCushion` — cushion lift, glide drag, fan thrust, rudders, hover velocity control, mud resistance
  - `hover::TerrainZones` — declares mud and water zones
  - `hover::ScriptedCommands` — timed test commands
- `scripts/vehicle_mobility_node.py` — `/cmd_vel` → fans or wheels, TRACK/TRANSITION/HOVER switching, `/vehicle_health`, placeholder `/terrain_state`
- `scripts/lidar_scan_node.py` — `/points` → best-effort `/scan`
- `scripts/terrain_costmap_node.py` — static `/terrain_costmap` for the default vehicle integration world
- `scripts/gen_test_worlds.py` → `worlds/vehicle_tests/*.sdf` — vehicle test worlds
- `worlds/vehicle_tests/integration_test.sdf` — unscripted world used by the common ROS launch
- `config/vehicle_mobility.yaml` — mobility and battery assumptions

## Environment (Person 3)

- `worlds/` — firm shore, mudflat, shallow-water and mangrove corridor scenes. The world's `<world name>` must equal its file name.
- `models/` — roots, debris, delivery zone and other reusable world assets.
- Mud and water zones: add a `hover::TerrainZones` plugin to the world (see `worlds/vehicle_tests/transition_test.sdf`) so the vehicle's cushion and mud physics know where they are. Use Gazebo's `Buoyancy` system for water.

The first complete world must provide `/scan`, `/imu`, `/odom`, `/terrain_state` and `/vehicle_health` through the agreed interface contract. The vehicle stack already provides all of these, with a placeholder `/terrain_state`.
