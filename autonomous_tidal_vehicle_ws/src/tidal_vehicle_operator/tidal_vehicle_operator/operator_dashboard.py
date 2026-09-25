"""Helper utilities for the operator dashboard and live mission summary."""

from __future__ import annotations


def build_dashboard_payload(
    *,
    mission_state: str,
    return_required: bool,
    planned_route_points: int,
    return_route_points: int,
    battery_percent: float,
    return_margin_percent: float,
    reason: str,
    telemetry: dict[str, object] | None = None,
) -> dict[str, object]:
    """Return structured mission state for both log and browser dashboards."""
    status = "RETURN" if return_required else mission_state
    return {
        "mission_state": mission_state,
        "return_required": return_required,
        "planned_route_points": planned_route_points,
        "return_route_points": return_route_points,
        "battery_percent": float(battery_percent),
        "return_margin_percent": float(return_margin_percent),
        "reason": reason,
        "telemetry": telemetry or {},
        "summary": (
            f"Mission state: {status}. "
            f"Battery {battery_percent:.1f}%, return margin {return_margin_percent:.1f}%. "
            f"Outbound route points={planned_route_points}, return route points={return_route_points}. "
            f"Reason: {reason}"
        ),
    }


def summarize_dashboard(
    *,
    mission_state: str,
    return_required: bool,
    planned_route_points: int,
    return_route_points: int,
    battery_percent: float,
    return_margin_percent: float,
    reason: str,
) -> str:
    """Return a compact human-readable summary for operator display."""
    return build_dashboard_payload(
        mission_state=mission_state,
        return_required=return_required,
        planned_route_points=planned_route_points,
        return_route_points=return_route_points,
        battery_percent=battery_percent,
        return_margin_percent=return_margin_percent,
        reason=reason,
    )["summary"]
