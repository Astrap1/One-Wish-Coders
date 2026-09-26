# Vehicle Blender models

Each vehicle design generation lives in its own `version_N/` folder. The Blender
script in a version folder is the single source of that design's geometry: it
writes the `.blend`, preview renders and the per-link meshes plus
`link_frames.json` that `tidal_vehicle_description/scripts/gen_description.py`
turns into the Gazebo SDF and ROS URDF.

| Version | Folder | Concept | Status |
| --- | --- | --- | --- |
| 1 | `version_1/` | 1.2 × 0.7 m air-cushion vehicle (0.99 m overall width), 25 kg, one lift fan, two rear thrust fans with rudders, four 0.30 m balloon wheels on swing-up legs with coil-over suspension. | Frozen. It is the fallback (`vehicle:=v1`); its generated model is `tidal_vehicle_description/models/hovercraft/`. |
| 2 | `version_2/` | Hovercraft-dominant amphibious vehicle with a complete retractable tracked undercarriage and controlled air-cushion load sharing: 2.5 × 1.5 × 1.5 m, 300 kg including a 30 kg payload, two inboard tracks, reversible ducted fans and a centre LiDAR mast. | The demo vehicle and launch default. Its generated model is `tidal_vehicle_description/models/hovercraft_v2/`. See `AGENTS.md` → *Vehicle Version 2* and `docs/VEHICLE_SIMULATION.md`. |
| 3 | `version_3/` | Version 2 scaled up for speed: 3.0 × 1.8 × 1.9 m, 530 kg including a 100 kg payload, series-hybrid (diesel generator) power, 2 × 0.7 m ducted fans, rudders and puff ports, retractable tracks, four cameras and contact sensors. Built by stretching Version 2's hull 1.2× in plan; round parts are rebuilt at size. | In development. `vehicle:=v3`; Version 2 stays the demo vehicle. Its generated model is `tidal_vehicle_description/models/hovercraft_v3/`. See `docs/VEHICLE_V3_PLAN.md`. |

Rebuild Version 3 (same tools as Version 2):

```bash
blender --background --python assets/vehicle_blender/version_3/build_vehicle.py
python3 autonomous_tidal_vehicle_ws/src/tidal_vehicle_description/scripts/gen_description_v3.py
```

Version 3 heavy-load presentation variant (visual only: about 350 kg of strapped deck cargo, woodland camouflage and generic military markings; `-- --no-camo` keeps the olive paint). It reuses `build_vehicle.py` but never writes the simulation meshes, SDF or URDF, so the Gazebo model keeps its 100 kg payload. Outputs `version_3/hovercraft_v3_heavy_load.blend` and `version_3/renders/heavy_load/` (including `load_log.txt`: load masses, overload figures and LiDAR clearance):

```bash
blender --background --python assets/vehicle_blender/version_3/build_heavy_load.py
```

Rebuild Version 2 (Blender 4.2+, or the pip `bpy` 4.2 module with `python3`):

```bash
blender --background --python assets/vehicle_blender/version_2/build_vehicle.py
python3 autonomous_tidal_vehicle_ws/src/tidal_vehicle_description/scripts/gen_description_v2.py
```

Rebuild Version 1 (Blender 4.2 writes both `.dae` and `.glb`; see `docs/VEHICLE_SIMULATION.md` §2 for Blender on Windows):

```bash
blender --background --python assets/vehicle_blender/version_1/build_vehicle.py
python3 autonomous_tidal_vehicle_ws/src/tidal_vehicle_description/scripts/gen_description.py
```

Do not edit a frozen version in place. Start a new `version_N/` folder instead.
