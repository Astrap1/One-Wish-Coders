# Vehicle Blender models

Each vehicle design generation lives in its own `version_N/` folder. The Blender
script in a version folder is the single source of that design's geometry: it
writes the `.blend`, preview renders and the per-link meshes plus
`link_frames.json` that `tidal_vehicle_description/scripts/gen_description.py`
turns into the Gazebo SDF and ROS URDF.

| Version | Folder | Concept | Status |
| --- | --- | --- | --- |
| 1 | `version_1/` | 1.2 × 0.7 m air-cushion vehicle (0.99 m overall width), 25 kg, one lift fan, two rear thrust fans with rudders, four 0.30 m balloon wheels on swing-up legs with coil-over suspension. | Frozen reference. Its generated model is `tidal_vehicle_description/models/hovercraft/`, which the launch file still uses until Version 2 replaces it. Gazebo acceptance results are summarised in `docs/VEHICLE_SIMULATION.md`. |
| 2 | `version_2/` (not yet created) | Hovercraft-dominant amphibious vehicle with a complete retractable tracked undercarriage and controlled air-cushion load sharing, about 2.5 m long, 1.5 m wide and 1.5 m high. | In design. See `AGENTS.md` → *Vehicle Version 2*. |

Rebuild Version 1 (Blender 4.2 writes both `.dae` and `.glb`; see `docs/VEHICLE_SIMULATION.md` §2 for Blender on Windows):

```bash
blender --background --python assets/vehicle_blender/version_1/build_vehicle.py
python3 autonomous_tidal_vehicle_ws/src/tidal_vehicle_description/scripts/gen_description.py
```

Do not edit a frozen version in place. Start a new `version_N/` folder instead.
