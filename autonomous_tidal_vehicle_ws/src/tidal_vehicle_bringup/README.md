# `tidal_vehicle_bringup`

This package is owned by the integration lead. It provides the one-command system launch.

```bash
ros2 launch tidal_vehicle_bringup sim.launch.py [world:=vehicle_tests/transition_test] [mode_policy:=hover_only] [rviz:=true] [foxglove:=true] [headless:=false]
```

- `launch/sim.launch.py` starts Gazebo, `ros_gz_bridge`, `robot_state_publisher`, the LiDAR scan node, the vehicle mobility node, `foxglove_bridge` (port 8765) and, optionally, RViz.
- `config/ros_gz_bridge.yaml` is the complete Gazebo ⇄ ROS topic map. Keep it in step with `docs/INTERFACES.md`.

Keep launch logic here; do not duplicate full-system launch files across workstreams. Autonomy and safety nodes are added to `sim.launch.py` by their owners once their entry points are ready.
