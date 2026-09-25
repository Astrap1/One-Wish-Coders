# `tidal_vehicle_bringup`

This package is owned by the integration lead. It provides the one-command system launch.

```bash
ros2 launch tidal_vehicle_bringup sim.launch.py headless:=true
```

This launches Vehicle Version 2 (`vehicle:=v2`, tracked) in the tidal corridor by default. Add `vehicle:=v1` for the wheeled Version 1 fallback. The launch spawns the chosen vehicle at HOME, and turns Safety's TRACK energy profile on for Version 2. In the corridor, its Gazebo terrain zones, moving water
visual and `tide_manager` provide the authoritative, changing map-frame terrain
state and cost map. The launch then starts the ROS-Gazebo bridge, robot state
publisher, LiDAR conversion, vehicle mobility, global planner, path follower,
safety supervisor and Foxglove bridge.

For a static vehicle-only test world, explicitly select it and disable the tide
manager. The vehicle's low-tide placeholder terrain state and static cost map are
then enabled:

```bash
ros2 launch tidal_vehicle_bringup sim.launch.py world:=vehicle_tests/integration_test tide:=false
```

The default integration world has no scripted vehicle commands. The complete command chain is `/cmd_vel_proposed` from Autonomy, approval on `/cmd_vel` from Safety, then internal track, wheel or fan commands from the mobility node. Safety must remain the only `/cmd_vel` publisher.

`config/ros_gz_bridge_v2.yaml` (Version 2) and `config/ros_gz_bridge.yaml` (Version 1) are the complete Gazebo ⇄ ROS topic maps. Keep them in step with `docs/INTERFACES.md`. Keep full-system launch logic here rather than duplicating it across workstreams.
