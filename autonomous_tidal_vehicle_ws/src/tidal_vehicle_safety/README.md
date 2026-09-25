# `tidal_vehicle_safety`

Owns the authoritative safety state machine. It consumes proposed motion commands and health/terrain telemetry, then publishes the only motion command that the simulation may execute.

## Required transitions

| Trigger | Expected state |
| --- | --- |
| Normal mission margin | `CRUISE` |
| Elevated terrain, obstacle or health risk | `CAUTION` |
| Insufficient confidence or safe clearance | `HOLD` |
| Insufficient return reserve, worsening tide or severe fault | `RETURN` |

Every state transition must include a human-readable reason in `/safety_status`.

## Internal mission phase

Safety uses a separate internal phase to distinguish an outbound mission from a
return. The phase is intentionally not an additional public safety state:

```text
PRELAUNCH → OUTBOUND → DELIVERED → RETURNING → PRELAUNCH
```

`/mission_event` provides the explicit `delivery_confirmed`,
`mission_complete`, and `mission_reset` lifecycle events. Delivery confirmation
immediately requests a route home. If Safety requests a return for any reason,
it holds the vehicle until Autonomy has published a fresh `/return_path` that
ends within the configured HOME tolerance.

## Return estimate and safety precedence

Safety—not simulation—calculates the estimated return energy, return margin and
return ETA. It samples `/return_path` at 0.5 m intervals against the shared
terrain-cost bands: tracks on firm shore (`0`--`19`), hover over mud/shallow
water (`20`--`59`) and conservative hover in elevated-risk cells (`60`--`89`).
Every TRACK--HOVER transition adds declared transition time and energy. Safety
then adds the larger of an 8 percentage-point buffer or 20% contingency. These
are transparent simulation assumptions, not field-validated vehicle-performance
claims.

The current Gazebo vehicle is **Version 1**, which has wheels and is deliberately
launched with `hover_only`; it does not demonstrate tracked travel or mode
transitions. Accordingly, `track_mode_enabled: false` is the default safety
configuration and firm-shore route samples use the hover energy/speed profile.
Enable TRACK estimation only together with the Version 2 tracked vehicle after
its Gazebo transitions are validated.

The supervisor applies the following precedence:

1. `HOLD` for stale telemetry, a non-traversable corridor, invalid return path,
   exhausted return margin, critical mobility degradation or any non-empty
   vehicle-controller fault.
2. `RETURN` when the tide/return time window, return margin, mobility health or
   communications link no longer supports continuing outbound.
3. `CAUTION` for elevated tide, a narrowing return margin or degraded mobility.
4. `CRUISE` only with fresh telemetry, current paths and normal operating margin.

`/safety_status` is reliable, transient-local and published at 10 Hz. Its
`return_required` flag is a latched instruction to Autonomy; Autonomy must
replace its outbound route with a new `/return_path` to HOME.

## Configuration and launch

Thresholds and command caps are in `config/safety_params.yaml`. Launch the
supervisor after sourcing the ROS 2 workspace:

```bash
ros2 run tidal_vehicle_safety safety_supervisor --ros-args \
  --params-file $(ros2 pkg prefix tidal_vehicle_safety)/share/tidal_vehicle_safety/config/safety_params.yaml
```

Only this node may publish `/cmd_vel`. The launch configuration must not bridge
or run another controller that publishes that topic.
