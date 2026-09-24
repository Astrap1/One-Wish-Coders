# Build decisions

## Defaults adopted for the initial skeleton

- **Middleware:** ROS 2 Jazzy.
- **Simulator:** Gazebo Harmonic, bridged through `ros_gz`.
- **Primary language:** Python for autonomy, safety, evaluation and operator tooling; SDF/URDF and YAML for simulation assets.
- **Physics scope:** A transparent mobility and terrain-risk abstraction, not validated computational fluid dynamics.
- **Navigation scope:** A known semantic terrain map plus simulated LiDAR for local obstacle response. SLAM is a future extension, not a critical-path dependency.

## Decisions to make before implementation expands

1. Which host environment will run the live demo: native Ubuntu, dual boot, WSL2 with graphics support, or a lab machine?
2. Will the operator interface be RViz/Foxglove-first, or should the team build a custom browser dashboard?
3. Which unsafe condition is the primary judged scenario: rising tide, low battery, skirt degradation or communications loss?
4. Is the vehicle’s final project name available, or should the repository remain name-neutral until branding is decided?
