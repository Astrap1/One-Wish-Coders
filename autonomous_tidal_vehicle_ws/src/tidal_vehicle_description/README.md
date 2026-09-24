# `tidal_vehicle_description`

Owns the SDF/URDF representation of the air-cushion vehicle, its collision/visual meshes, sensor mounts and RViz display configuration.

## Deliverables

- `urdf/` — Xacro/URDF source when a ROS robot description is needed.
- `models/` — Gazebo model folders and `model.sdf` files.
- `meshes/` — Blender-exported visual and collision meshes.
- `config/sensors.yaml` — Shared sensor-frame and update-rate assumptions.
- `rviz/` — Operator-facing visualisation configuration.

The vehicle should expose a hull, skirt, protected payload pod, LiDAR, IMU and camera mount. Detailed aerodynamic modelling is outside this package's initial scope.
