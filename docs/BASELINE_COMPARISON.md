# Baseline Capability Comparison

This comparison is a qualitative baseline for the simulated tidal-mangrove mission. It is a planning and presentation aid, not a field-performance claim. Numeric speed, energy and payload values must come from the same controlled scenario runs before they are reported as measured results.

## Mission context

```text
launch → cross shore/mud/shallow water → avoid roots or debris
→ deliver payload → return safely
```

| Capability | Wheeled rover | Small boat | Simulated air-cushion vehicle |
| --- | --- | --- | --- |
| Firm shore | Strong | Poor without a launch/recovery point | Strong in wheel mode |
| Mud | High sinkage risk; route may become impassable | Not applicable | Intended operating band in hover mode |
| Shallow water | Limited; requires firm crossing | Strong | Strong in hover mode, subject to the simulated terrain-cost model |
| Mixed shore-to-water transition | Difficult and slow | Requires water access | Core mission advantage; represented as a mobility-mode transition |
| Roots/debris | Ground collision and entanglement risk | Floating/submerged obstacle risk | LiDAR obstacle response and safety hold/return are demonstrated |
| Rising tide | Reduces traversable shore/mud area | May improve flotation route but can create new hazards | Updates terrain state/cost map and can force replan, hold or return |
| Payload delivery | Practical on firm access routes | Practical from water access | Sealed payload box in the simulated vehicle model |
| Return safety | Battery and terrain-dependent | Battery, current and access-dependent | Safety supervisor calculates return energy, margin and ETA |
| Control authority | Vehicle/controller dependent | Vehicle/controller dependent | Safety supervisor is the only `/cmd_vel` publisher |
| Evidence in this project | Comparison baseline only | Comparison baseline only | Full ROS/Gazebo demonstration target |

## What the project actually demonstrates

The air-cushion profile is the implemented demonstration system. Its comparison advantage is operational flexibility across the shore/mud/shallow-water boundary, not a claim of superior real-world speed or endurance.

The required evidence is:

1. Normal delivery and return.
2. Obstacle-induced route replanning.
3. Rising-tide terrain updates that cause a visible safety-driven replan, hold or return.

For each profile, do not claim a capability unless the same scenario definition, goal, payload condition and success criteria were used. Record `duration_s`, `replans`, `minimum_obstacle_clearance_m`, `safety_interventions`, `final_return_margin_percent` and completion status for the implemented vehicle.

## Recommended presentation graphic

Use a three-column slide with one row per mission concern:

```text
                 WHEELED ROVER       SMALL BOAT       AIR-CUSHION VEHICLE
Firm shore       strong              weak             strong
Mud              risk of sinkage     n/a              hover mode
Shallow water    limited             strong           hover mode
Transition       difficult           launch point     core mission case
Rising tide      route loss           changing hazard  replan / hold / return
Evidence         baseline            baseline         live simulation
```

Label the first two columns “qualitative baseline” and the last column “simulated demonstration.” Add a footnote: “Relative capability descriptions are mission assumptions; no field-validated vehicle-performance comparison is claimed.”

## Follow-up measurement plan

When the common launcher and scenarios are stable, run each required scenario repeatedly and export a result table. The comparison should then show measured values only for the air-cushion simulation, with rover and boat remaining clearly marked as qualitative reference profiles unless equivalent models are implemented.
