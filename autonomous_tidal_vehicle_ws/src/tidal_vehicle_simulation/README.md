# `tidal_vehicle_simulation`

Owns the Gazebo tidal-corridor environment and the transparent abstractions that turn tide, terrain and vehicle condition into simulation inputs.

## Vehicle simulation (Person 4)

See [`docs/VEHICLE_SIMULATION.md`](../../../docs/VEHICLE_SIMULATION.md).

- `plugins/` — `libtidal_vehicle_plugins.so`, C++ Gazebo systems:
  - `hover::AirCushion` — cushion lift, glide drag, fan thrust, rudders, hover velocity control, mud resistance
  - `hover::TerrainZones` — declares mud and water zones
  - `hover::ScriptedCommands` — timed test commands
- `scripts/vehicle_mobility_node.py` — `/cmd_vel` → fans or wheels, TRACK/TRANSITION/HOVER switching, `/vehicle_health`, and a low-tide placeholder `/terrain_state` only for static test worlds
- `scripts/lidar_scan_node.py` — `/points` → best-effort `/scan`
- `scripts/terrain_costmap_node.py` — static `/terrain_costmap` fallback for explicit vehicle test worlds
- `scripts/tide_manager.py` — changing, map-frame `/terrain_state` and `/terrain_costmap` for the tidal corridor
- `scripts/gen_test_worlds.py` → `worlds/vehicle_tests/*.sdf` — vehicle test worlds
- `worlds/vehicle_tests/integration_test.sdf` — unscripted world used by the common ROS launch
- `config/vehicle_mobility.yaml` — mobility and battery assumptions
- `config/vehicle_mobility_v2.yaml` — Version 2 track/hover settings. Its
  LiDAR height gate rejects the ground mesh and retains raised roots, trunks
  and debris for the planner's dynamic-obstacle overlay.

## Environment (Person 3)

- `worlds/` — firm shore, mudflat, shallow-water and mangrove corridor scenes. The world's `<world name>` must equal its file name.
- `models/` — roots, debris, delivery zone and other reusable world assets.
- Mud and water zones: add a `hover::TerrainZones` plugin to the world (see `worlds/vehicle_tests/transition_test.sdf`) so the vehicle's cushion and mud physics know where they are. Use Gazebo's `Buoyancy` system for water.

`worlds/tidal_corridor.sdf` is the complete demo world: it includes the
hovercraft, DART/Bullet physics, sensors and terrain-zone plugin. Its
channel/mud rectangles and tide profile exactly match the tide manager cost map.
The tide manager, not the vehicle placeholder, provides `/terrain_state` and
`/terrain_costmap`. The `TerrainZones` water level is a physical air-cushion
surface: while in HOVER mode, the vehicle holds its configured skirt gap over
the rising water rather than passively floating like a boat.

For the live corridor, `models/terrain_demo/` retains the authored terrain mesh
as a visual and uses an inclined box as the physics collision surface. The
terrain zones and cost map—not that collision surface—represent firm shore, mud
and shallow-water traversal behaviour. This is a deliberate performance
abstraction for repeatable DART/Bullet demonstrations.
