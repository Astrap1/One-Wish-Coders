# Autonomous Tidal Vehicle ROS 2 workspace

This is a `colcon` workspace for the Gazebo simulation and supporting ROS 2 packages.

## Intended setup

The initial configuration targets ROS 2 Jazzy and Gazebo Harmonic. The team will run it on Ubuntu 24.04 under WSL2, with Gazebo developed through WSLg and Foxglove Desktop connected through Foxglove Bridge.

```bash
cd autonomous_tidal_vehicle_ws
colcon build --symlink-install
source install/setup.bash
```

The vehicle model, Gazebo plugins, ROS bridge and one-command launch are in place (see `../docs/VEHICLE_SIMULATION.md`):

```bash
ros2 launch tidal_vehicle_bringup sim.launch.py
```

## Package map

| Package | Role |
| --- | --- |
| `tidal_vehicle_description` | Vehicle description, sensors and visual assets. |
| `tidal_vehicle_simulation` | Gazebo world, models and terrain/tide abstractions. |
| `tidal_vehicle_interfaces` | Shared ROS 2 messages for health, terrain and safety. |
| `tidal_vehicle_autonomy` | Perception, planning, avoidance and mission decisions. |
| `tidal_vehicle_safety` | Safety authority and fallback state machine. |
| `tidal_vehicle_operator` | Operator dashboard and visualisation configuration. |
| `tidal_vehicle_evaluation` | Scenario control, fault injection and metrics. |
| `tidal_vehicle_bringup` | Complete-system launch and shared configuration. |
