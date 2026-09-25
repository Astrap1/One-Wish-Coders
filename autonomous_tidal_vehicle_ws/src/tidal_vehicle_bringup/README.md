# `tidal_vehicle_bringup`

This package is owned by the integration lead. It provides the one-command system launch.

```bash
ros2 launch tidal_vehicle_bringup sim.launch.py world:=vehicle_tests/integration_test mode_policy:=hover_only
```

The launch starts Gazebo, the ROS-Gazebo bridge, robot state publisher, LiDAR conversion, vehicle mobility, the temporary static terrain map, global planner, path follower, safety supervisor, Foxglove bridge and optional RViz.

The default integration world has no scripted vehicle commands. The complete command chain is `/cmd_vel_proposed` from Autonomy, approval on `/cmd_vel` from Safety, then internal wheel or fan commands from the mobility node. Safety must remain the only `/cmd_vel` publisher.

`config/ros_gz_bridge.yaml` is the complete Gazebo ⇄ ROS topic map. Keep it in step with `docs/INTERFACES.md`. Keep full-system launch logic here rather than duplicating it across workstreams.
