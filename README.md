# Autonomous Tidal-Corridor Logistics Vehicle

> An autonomous air-cushion logistics vehicle concept for delivering critical supplies across tidal mudflats, shallow water and mangrove terrain.

This project responds to the Singapore Defence Tech Hackathon **Green Corridor** challenge. It explores how an unmanned air-cushion vehicle could move a small protected payload—such as water, batteries or medical supplies—through short tidal crossings that are difficult or unsafe for personnel, wheeled vehicles and boats.

## The problem

Mangrove and tidal terrain can combine soft mud, shallow water, submerged obstacles, dense roots and rapidly changing access conditions. Wheeled platforms can bog down in mud; boats can ground before reaching shore; sending people across on foot exposes them to physical strain, tide risk and unnecessary danger while carrying supplies.

Our near-term goal is to demonstrate the **decision-making problem** in simulation: can a vehicle safely plan, execute, reassess and, if required, abort a logistics mission in a tidal-mangrove corridor?

The project is not claiming that an operational hovercraft has already been validated in live mangroves. Durability, payload capacity, noise, salt-water resistance and real terrain performance require physical prototyping and field trials.

## Proposed approach

We model a compact autonomous air-cushion vehicle that will use retractable tracks over firm shore, then transition to air-cushion travel over wet mud and shallow water while carrying a protected payload. Rather than relying on perfect autonomy, the design combines route planning, local sensing, high-level operator supervision and an independent safety layer. The current simulated Vehicle Version 1 is intentionally hover-only; tracked operation and TRACK--HOVER transitions are Version 2 work, not a current demo claim.

The vehicle receives a delivery goal, terrain information, tide state and a return-energy reserve. It plans a low-risk route, detects nearby obstacles with simulated LiDAR, replans when necessary and returns or holds when the mission is no longer safe.

## Mission architecture

1. An operator selects a destination, confirms the payload and approves launch.
2. The mission manager checks whether a safe outbound route and battery reserve for return exist.
3. A terrain-aware planner chooses a route using terrain cost, no-go zones, tide state and return-energy requirements.
4. LiDAR and IMU data support local obstacle awareness while the vehicle follows the approved route.
5. At the delivery point, the system confirms payload arrival and starts its return journey.
6. If a route is blocked or the risk changes, the system replans, holds in a safe location or returns to launch.

The independent safety supervisor controls four states:

- **CRUISE** — continue along the approved route.
- **CAUTION** — slow down and reassess after detecting elevated risk.
- **HOLD** — stop safely when proceeding is uncertain or unsafe.
- **RETURN** — abort delivery or depart after delivery using a safe route to launch.

Safety overrides navigation when battery reserve, tide progression, obstacle clearance, communications or vehicle health falls outside safe limits.

## Simulation and software architecture

The prototype uses **Gazebo** for the 3D environment and **ROS 2** for subsystem integration.

```text
Gazebo tidal corridor
  ├─ firm shore, soft mud, shallow water, mangrove roots and debris
  ├─ changing terrain-risk/tide zones
  └─ vehicle model: hull, skirt, cargo pod, lift/propulsion fans, sensors and planned retractable tracks

ROS 2 autonomy
  ├─ local perception: LiDAR → obstacle map
  ├─ terrain-aware global planner
  ├─ local avoidance and route replanning
  ├─ mission manager: delivery, tide window and return route
  ├─ safety supervisor: CRUISE / CAUTION / HOLD / RETURN
  └─ vehicle controller: approved speed and heading commands

Operator and evaluation
  ├─ dashboard: map, outbound/return route, health, tide, payload and safety state
  ├─ scenario runner: faults and repeatable mission conditions
  └─ metrics: completion, duration, replans, clearance and return margin
```

The simulator will model tide as changing terrain risk and traversability, rather than attempting full computational fluid dynamics. Vehicle mobility will similarly use a stated engineering abstraction: Version 1 demonstrates air-cushion travel over mud and shallow water; Version 2 will add tracked firm-shore travel and stopped, controlled transitions between modes. It will model terrain-dependent speed, battery consumption and simulated degradation without claiming validated hovercraft physics. This keeps the demo technically honest while making the mission behaviour credible and testable.

## Planned demonstration

The final demo will run the same vehicle through three repeatable scenarios:

1. **Normal delivery:** Deliver cargo across the tidal corridor and return safely.
2. **Blocked route:** Detect debris or dense roots, create a local obstacle map and replan around the hazard.
3. **Rising tide:** Water levels change the terrain-cost map during the mission, requiring a visible replan, hold or safety-driven return.

We will show a 3D vehicle-camera view and a mission console with outbound and
return routes, LiDAR obstacles, battery, Safety-calculated return energy/margin/ETA,
tide risk, payload status and safety state. We may also compare terrain capability
profiles for a wheeled rover, boat and air-cushion vehicle to communicate the
operational niche; this comparison is not a claim that three complete autonomous
systems have been built.

## Team workstreams

| Workstream | Ownership |
| --- | --- |
| Autonomy | Terrain-aware routing, local obstacle avoidance, replanning and command generation. |
| Safety and mission | State machine, battery/tide return logic, vehicle-health monitoring and fallback decisions. |
| Environment | 3D tidal-mangrove world, terrain zones, roots, debris, delivery area and tide-state changes. |
| Vehicle simulation and integration | Vehicle SDF model, sensors, mobility abstraction, ROS-Gazebo bridge, packages and launch files. |
| Operator, evaluation and demonstration | Dashboard, logging, scenario controls, metrics, baseline comparison and demonstration narrative. |

## Suggested repository layout

```text
assets/                         # Vehicle meshes, textures and reference media
autonomous_tidal_vehicle_ws/
  src/
    vehicle_description/        # SDF/URDF, vehicle assets, sensor definitions
    tidal_simulation/           # Gazebo worlds and terrain/tide/mobility plugins
    autonomy/                   # Perception, planning, avoidance and mission management
    safety/                     # Health monitoring and safety state machine
    operator_interface/         # Dashboard, RViz configuration and camera views
    evaluation/                 # Scenarios, fault injection and metrics
docs/                           # Architecture, experiment plan and presentation material
```

## Path to an operational solution

If the simulated mission logic is successful, further development would include physical vehicle prototypes; skirt, propulsion and payload testing in mud, sand and shallow water; salt-water environmental ruggedisation; degraded-navigation sensing; launch and recovery procedures; noise and safety studies; and staged field validation with relevant stakeholders.

## References

- [Team concept write-up](https://docs.google.com/document/d/16XhY4t8aE2lHvgW-lWGe5dFWkDKtkL9ntcomOaFqlUk/edit?tab=t.0)
- [Green Corridor challenge brief](01_Green_Corridor.pdf)
- [Hackathon introduction](00_Introduction%20%281%29.pdf)

## Status

Concept and simulation plan. The Gazebo environment, vehicle model, autonomy stack and operator interface are under development.
