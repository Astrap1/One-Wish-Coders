"""Zone speed limits (split cost bands, AGENTS.md). Added by Person 4 for Version 3."""
from tidal_vehicle_safety.return_estimator import GridCostmap
from tidal_vehicle_safety.zone_limits import zone_limit_ahead, zone_speed_limit

LIMITS = [4.2, 13.9, 2.8, 2.8]          # firm, open surveyed water, mud/roots/debris, elevated


def _strip(costs):
    """A 1-cell-high map along +x, 1 m per cell, starting at x = 0."""
    return GridCostmap(width=len(costs), height=1, resolution_m=1.0,
                       origin_x_m=0.0, origin_y_m=-0.5, data=costs)


def test_zone_speed_limit_bands() -> None:
    assert zone_speed_limit([25, 22], LIMITS) == 13.9
    assert zone_speed_limit([25, 40], LIMITS) == 2.8
    assert zone_speed_limit([10], LIMITS) == 4.2
    assert zone_speed_limit([100, -1], LIMITS) is None
    assert zone_speed_limit([25], [0.0]) is None


def test_open_water_allows_speed() -> None:
    grid = _strip([25] * 200)
    assert zone_limit_ahead(grid.cost_at, 0.0, 0.0, 0.0, 8.0, LIMITS, 1.5) == 13.9


def test_slows_before_a_root_zone_within_stopping_distance() -> None:
    # roots start 30 m ahead; at 13.9 m/s the vehicle needs ~78 m to stop
    grid = _strip([25] * 30 + [40] * 170)
    assert zone_limit_ahead(grid.cost_at, 0.0, 0.0, 0.0, 13.9, LIMITS, 1.5) == 2.8
    # at walking pace the same roots are beyond the 2 m reach
    assert zone_limit_ahead(grid.cost_at, 0.0, 0.0, 0.0, 0.5, LIMITS, 1.5) == 13.9
