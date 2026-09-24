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
