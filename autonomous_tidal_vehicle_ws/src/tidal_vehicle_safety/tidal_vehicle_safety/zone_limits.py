"""Zone speed limits from the split terrain-cost bands (AGENTS.md, *Shared
terrain-cost semantics*). Added by Person 4 for Version 3; Safety only uses
them when `zone_speed_limits_mps` is set (sim.launch.py does so for v3)."""
from __future__ import annotations

from math import cos, sin
from typing import Callable, Optional, Sequence

BANDS = ((0, 19), (20, 29), (30, 59), (60, 89))


def zone_speed_limit(costs: Sequence[int], limits: Sequence[float]) -> Optional[float]:
    """Lowest limit (m/s) of the sampled cells. limits = [firm 0-19, open
    surveyed water 20-29, mud/shallow water/roots/debris 30-59, elevated risk
    60-89]. No-go and unknown cells are left to the route checks. None when
    nothing matched or the limits are off."""
    if len(limits) != 4:
        return None
    out = None
    for cost in costs:
        for (low, high), limit in zip(BANDS, limits):
            if low <= cost <= high:
                out = limit if out is None else min(out, limit)
    return out


def zone_limit_ahead(
    cost_at: Callable[[float, float], Optional[int]],
    x: float, y: float, yaw: float, speed: float,
    limits: Sequence[float], brake_decel: float,
    step_m: float = 0.5, minimum_m: float = 2.0,
) -> Optional[float]:
    """Zone limit from the vehicle out to its stopping distance (1 s reaction
    plus braking) along its heading, so it slows before a slower zone."""
    reach = max(minimum_m, speed + speed * speed / (2.0 * max(brake_decel, 0.1)))
    costs = []
    for k in range(int(reach / step_m) + 1):
        d = k * step_m
        cost = cost_at(x + d * cos(yaw), y + d * sin(yaw))
        if cost is not None:
            costs.append(cost)
    return zone_speed_limit(costs, limits)
