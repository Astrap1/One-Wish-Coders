# Person 5 — Operator, Evaluation and Demo Context

## Role

Person 5 owns the operator-facing experience and the evidence that makes the demonstration credible:

- operator display and mission visibility;
- deterministic scenario triggers and evaluation;
- fault-injection controls owned by Evaluation;
- run-result logging and metrics;
- Foxglove layout and dashboard presentation;
- comparison visuals, captions and the presentation story.

Person 5 does **not** directly control the vehicle or publish `/cmd_vel`. The Safety supervisor is the only component allowed to publish `/cmd_vel`; the operator requests goals or scenario events and observes the resulting safety decision.

## Demonstration claim

The project demonstrates autonomous route planning, obstacle response and safety-driven fallback in explicitly defined simulated terrain, tide and vehicle-health conditions. It does not claim field-validated hovercraft, hydrodynamic or mud-penetration physics.

The fixed mission story is:

```text
launch → cross shore/mud/shallow water → avoid roots or debris
→ deliver payload → return safely
```

## Required scenarios

Every demo run should cover these three scenarios:

1. **Normal delivery and return** — select a goal, follow the planned route, deliver the payload and return to HOME.
2. **Obstacle-induced reroute** — introduce an obstacle or LiDAR blockage, show route reassessment/replanning, and complete or safely recover the mission.
3. **Rising tide** — update terrain/tide state and cost map so the route becomes riskier; show a visible replan, `CAUTION`, `HOLD` or `RETURN` decision with a human-readable rationale.

For each run record:

| Metric | Meaning |
| --- | --- |
| completion | Whether the scenario met its success condition |
| duration_s | Start-to-completion or start-to-safe-stop duration |
| replans | Number of route changes after the initial plan |
| minimum_obstacle_clearance_m | Closest valid obstacle distance |
| safety_interventions | Safety holds, speed reductions or forced returns |
| final_return_margin_percent | Safety-calculated reserve at the end |
| final_state | `CRUISE`, `CAUTION`, `HOLD` or `RETURN` at the end |
| reason | The latest safety/operator explanation |

The evaluation package already defines `ScenarioResult` with the first six core measurements and a `summarize_scenario_result()` formatter. Extend that package for run storage, deterministic triggers and report export rather than adding ad-hoc logging to the dashboard.

## ROS topics to display or evaluate

| Topic | Type | Person 5 use |
| --- | --- | --- |
| `/odom` | `nav_msgs/Odometry` | Vehicle pose, velocity and mission trace |
| `/scan` | `sensor_msgs/LaserScan` | Obstacle visualization and clearance metric |
| `/imu` | `sensor_msgs/Imu` | Motion/orientation diagnostics |
| `/terrain_state` | `tidal_vehicle_interfaces/TerrainState` | Tide phase, water level, tide risk and corridor safety |
| `/terrain_costmap` | `nav_msgs/OccupancyGrid` | Terrain-risk visualization and rising-tide evidence |
| `/vehicle_health` | `tidal_vehicle_interfaces/VehicleHealth` | Battery, mobility, link, payload and fault display |
| `/mission_goal` | `geometry_msgs/PoseStamped` | Operator goal-selection output |
| `/planned_path` | `nav_msgs/Path` | Active outbound or approved return route |
| `/return_path` | `nav_msgs/Path` | Fresh route to HOME and return evidence |
| `/safety_status` | `tidal_vehicle_interfaces/SafetyStatus` | Primary safety state, rationale, return requirement, energy, margin and ETA |
| `/mission_event` | `std_msgs/String` | Delivery, completion and reset lifecycle evidence |
| `/scenario_event` | `std_msgs/String` | Deterministic evaluation controls such as `operator_abort` and `reset` |

Public safety states are only `CRUISE`, `CAUTION`, `HOLD` and `RETURN`. Internal phases such as `PRELAUNCH`, `OUTBOUND`, `DELIVERED` and `RETURNING` are not extra public safety states.

## Current operator surfaces

### Browser dashboard

The browser dashboard is served by `tidal_vehicle_operator` at:

```text
http://localhost:8000
```

Start it from WSL after sourcing ROS and the workspace:

```bash
source /opt/ros/jazzy/setup.bash
source /home/gayle/One-Wish-Coders/autonomous_tidal_vehicle_ws/install/setup.bash
ros2 run tidal_vehicle_operator browser_dashboard
```

The dashboard polls `/api/state` once per second. It currently shows only the requested operator metrics:

- fuel and available energy;
- combined fuel/energy reserve;
- current vehicle speed;
- current vehicle coordinates;
- a map of the current location with planned and return paths;
- safety state and rationale;
- tide level, tide phase and tide risk.

Fuel is displayed as `N/A` until the vehicle-health contract provides a dedicated fuel field. Battery percentage is used for available energy, and Safety's return-margin percentage is used for the combined reserve.

The dashboard is an operator summary, not the source of safety authority. If live terrain or health topics are absent, it should visibly report that telemetry is unavailable rather than implying a successful mission.

### Foxglove

Run the ROS bridge in WSL:

```bash
source /opt/ros/jazzy/setup.bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

Connect Foxglove Desktop on Windows to:

```text
ws://localhost:8765
```

The Foxglove layout should include, at minimum, vehicle pose, `/planned_path`, `/return_path`, LiDAR obstacles, terrain cost map, battery/health, tide state/risk/water level, `/safety_status` and mission events. Keep the layout legible at demo scale and make the safety rationale visible without opening a terminal.

## Operator workflow

1. Start the documented simulation/integration launch path owned by Person 4.
2. Start or verify Foxglove bridge and open the Foxglove layout.
3. Open the browser dashboard at `http://localhost:8000`.
4. Confirm telemetry is live: pose, terrain state, vehicle health and safety state should update.
5. Select or publish a `/mission_goal`; do not publish `/cmd_vel` from the operator UI.
6. Capture a baseline run before triggering faults.
7. Trigger only deterministic Evaluation events for the relevant scenario.
8. Record the scenario result, screenshots/recording and safety rationale.
9. Reset through the documented `reset` event and verify the system returns to a clean prelaunch state.

## Evidence checklist

Before calling a scenario successful, verify:

- route and vehicle pose are visible together;
- obstacle or terrain-risk change is visible before the route changes;
- the safety state changes with a readable reason;
- a return request produces a fresh `/return_path` to HOME;
- the final return margin is recorded;
- the dashboard and Foxglove agree on the state;
- no operator-only shortcut bypasses Safety;
- the run can be repeated from a clean reset.

For the final presentation, prepare one clean recording of each required scenario and a compact comparison graphic for wheeled rover, boat and air-cushion vehicle profiles. Present the comparison as capability trade-offs for this simulated mission, not as an unsupported real-world performance claim.

## Current gaps and next actions

- The complete one-command launch path and final demo branch are still Person 4 integration responsibilities; use the shared launcher once it exists rather than inventing a parallel launch script.
- The Evaluation package currently contains result data structures and summary formatting; add deterministic scenario orchestration, event logging and report persistence.
- Add the goal-selection and visible emergency-abort workflow required by the operator package README. An abort should publish the agreed `/scenario_event` value (`operator_abort`), not a direct velocity command.
- Build and save a Foxglove layout for the three scenarios, including the terrain/tide view and safety rationale.
- Add automated checks that compare dashboard/evaluation values against the message contracts in `docs/INTERFACES.md`.
- Do not commit generated `build/`, `install/`, `log/`, recordings, PDFs or experiment outputs.

## Safe boundaries

- Do not change topic names or message fields without updating `docs/INTERFACES.md` in the same change.
- Do not add a new public safety state for presentation convenience.
- Do not make the dashboard or operator UI publish `/cmd_vel`.
- Do not describe simulated mobility assumptions as validated physics.
- Keep operator controls deterministic, auditable and resettable.
