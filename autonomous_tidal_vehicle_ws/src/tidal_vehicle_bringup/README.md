# `tidal_vehicle_bringup`

This package is owned by the integration lead. It provides the one-command system launch.

```bash
ros2 launch tidal_vehicle_bringup sim.launch.py headless:=true
```

This launches the tidal corridor by default: its Gazebo terrain zones, moving water
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

The default integration world has no scripted vehicle commands. The complete command chain is `/cmd_vel_proposed` from Autonomy, approval on `/cmd_vel` from Safety, then internal wheel or fan commands from the mobility node. Safety must remain the only `/cmd_vel` publisher.

`config/ros_gz_bridge.yaml` is the complete Gazebo ⇄ ROS topic map. Keep it in step with `docs/INTERFACES.md`. Keep full-system launch logic here rather than duplicating it across workstreams.
