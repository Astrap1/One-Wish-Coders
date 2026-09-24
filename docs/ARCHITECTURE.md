# Architecture

The project separates simulation, mission decisions, safety authority and operator evidence so that each can be developed and tested independently.

```text
Gazebo environment and vehicle
        │ sensors, odometry, terrain state, vehicle health
        ▼
Autonomy: perception → planner → mission manager
        │ proposed motion commands and mission intent
        ▼
Safety supervisor
        │ approved command, hold or return override
        ▼
Vehicle controller and Gazebo bridge

Operator interface and evaluation consume telemetry from every layer.
```

## Authority model

The planner proposes a path. The mission manager decides whether delivery is still worthwhile. The safety supervisor has final authority: it can reduce speed, command a hold or force a return when the vehicle no longer has a safe operating margin.

## Simulation boundary

The simulator models a 3D tidal corridor, sensor observations, terrain cost, changing tide risk, battery use and vehicle degradation. It does not claim to provide validated hydrodynamic, mud-penetration or skirt-durability physics. Those require later physical testing.

## Workstream boundaries

| Package group | Owner | Responsibility |
| --- | --- | --- |
| `tidal_vehicle_description`, `tidal_vehicle_simulation`, `tidal_vehicle_bringup` | Vehicle simulation and integration | Vehicle model, world, sensors, Gazebo bridge and complete launch. |
| `tidal_vehicle_autonomy` | Autonomy | Perception, planning, avoidance and delivery mission logic. |
| `tidal_vehicle_safety` | Safety and mission | Vehicle health interpretation and safety-state authority. |
| `tidal_vehicle_operator`, `tidal_vehicle_evaluation` | Operator, evaluation and demonstration | Console, scenarios, fault injection, recording and metrics. |
