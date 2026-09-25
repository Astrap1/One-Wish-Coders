# `tidal_vehicle_description`

The air-cushion vehicle's description. See [`docs/VEHICLE_SIMULATION.md`](../../../docs/VEHICLE_SIMULATION.md).

- `models/hovercraft/` — Gazebo model (`model.sdf`, `model.config`), Blender-exported meshes and `link_frames.json`
- `urdf/hovercraft.urdf` — ROS robot description with the same links and joints (for `robot_state_publisher`, RViz and Foxglove)
- `scripts/gen_description.py` — regenerates both from `link_frames.json`. Run it after `assets/vehicle_blender/build_vehicle.py`. Never edit the generated files by hand.
- `config/sensors.yaml` — sensor frames, topics and rates
- `rviz/hovercraft.rviz` — RViz view (Fixed Frame `map`)

Frames: `base_link` (hull), `lidar_link`, `camera_link`, `imu_link`, plus the moving parts (`wheel_leg_*`, `wheel_shock_*`, `wheel_*`, fans, rudders).
