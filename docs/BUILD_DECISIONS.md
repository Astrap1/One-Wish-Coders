# Build decisions

## Decisions adopted for the initial build

- **Middleware:** ROS 2 Jazzy.
- **Simulator:** Gazebo Harmonic, bridged through `ros_gz`.
- **Demo host:** Ubuntu 24.04 under WSL2. Keep the workspace in the Linux filesystem; use WSLg for Gazebo development.
- **Operator view:** Foxglove Desktop, connected through Foxglove Bridge. A custom web dashboard is a future enhancement, not a critical-path dependency.
- **Primary language:** Python for autonomy, safety, evaluation and operator tooling; SDF/URDF and YAML for simulation assets.
- **Physics scope:** A transparent mobility and terrain-risk abstraction, not validated computational fluid dynamics.
- **Navigation scope:** A known semantic terrain map plus simulated LiDAR for local obstacle response. SLAM is a future extension, not a critical-path dependency.
- **Headline unsafe scenario:** Rising tide. The simulated water level changes terrain traversability and route cost over time, requiring repeated route assessment and replanning.

## Remaining decisions

1. Is the vehicle’s final project name available, or should the repository remain name-neutral until branding is decided?
2. What is the team’s exact WSL2/Ubuntu setup on the demo laptop, including GPU driver verification?
