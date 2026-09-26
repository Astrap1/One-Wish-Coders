# `tidal_vehicle_bringup`

This package is owned by the integration lead. It provides the one-command system launch.

```bash
ros2 launch tidal_vehicle_bringup sim.launch.py headless:=true foxglove:=true dashboard:=true gpu:=nvidia
```

This launches Vehicle Version 2 (`vehicle:=v2`, tracked) in the tidal corridor by default. Add `vehicle:=v1` for the wheeled Version 1 fallback. The launch spawns the chosen vehicle at HOME, and turns Safety's TRACK energy profile on for Version 2. In the corridor, its Gazebo terrain zones, moving water
visual and `tide_manager` provide the authoritative, changing map-frame terrain
state and cost map. The launch then starts the ROS-Gazebo bridge, robot state
publisher, LiDAR conversion, vehicle mobility, global planner, path follower,
safety supervisor and Foxglove bridge.

Keep `tide:=true` (the default) for every bank-to-bank tidal-corridor demo.
`tide:=false` starts the small legacy static integration map for the dedicated
`vehicle_tests/integration_test` world; it does not cover the corridor's
delivery point at `(104, 0)` and therefore cannot plan that mission.

For a static vehicle-only test world, explicitly select it and disable the tide
manager. The vehicle's low-tide placeholder terrain state and static cost map are
then enabled:

```bash
ros2 launch tidal_vehicle_bringup sim.launch.py world:=vehicle_tests/integration_test tide:=false
```

Start the browser operator dashboard alongside the normal shared launch:

```bash
ros2 launch tidal_vehicle_bringup sim.launch.py dashboard:=true
```

Open `http://localhost:8000`. The **Remote control** toggle selects supervised
manual driving. The dashboard publishes only `/operator_cmd_vel`; the safety
supervisor remains the sole publisher of `/cmd_vel`. Switching back to
Autonomous control immediately releases manual authority.
The **Dispatch demo delivery** button publishes the documented `(104 m, 0 m)`
map-frame mission goal: from HOME `(0 m, 0 m)` on the western firm bank to the
delivery pad on the eastern firm bank, across the tidal corridor.

On the NVIDIA WSL2 demo laptop, add `gpu:=nvidia`. This selects the WSLg D3D12
adapter instead of Mesa's `llvmpipe` software renderer; verify it with
`glxinfo -B` before a demo.

The default integration world has no scripted vehicle commands. The complete command chain is `/cmd_vel_proposed` from Autonomy, approval on `/cmd_vel` from Safety, then internal track, wheel or fan commands from the mobility node. Safety must remain the only `/cmd_vel` publisher.

`config/ros_gz_bridge_v2.yaml` (Version 2) and `config/ros_gz_bridge.yaml` (Version 1) are the complete Gazebo ⇄ ROS topic maps. Keep them in step with `docs/INTERFACES.md`. Keep full-system launch logic here rather than duplicating it across workstreams.
