from math import inf, nan, pi

import pytest

from tidal_vehicle_autonomy.lidar_obstacle_core import (
    ObstaclePersistenceFilter,
    associate_obstacle_hits,
    inflate_no_go_cells,
    inflate_obstacle_cells,
    obstacle_cells_from_scan,
    obstacle_change_requires_replan,
    overlay_obstacles,
    path_cells_ahead,
)
from tidal_vehicle_autonomy.planner_core import GridCostMap, plan_path


def _safe_map(width: int = 7, height: int = 7) -> GridCostMap:
    return GridCostMap(width=width, height=height, costs=[0] * (width * height))


def test_projects_forward_return_into_map_cell() -> None:
    obstacles = obstacle_cells_from_scan(
        _safe_map(),
        robot_x=2.5,
        robot_y=2.5,
        robot_yaw=0.0,
        ranges=[2.0],
        angle_min=0.0,
        angle_increment=0.0,
        range_min=0.1,
        range_max=10.0,
        inflation_radius=0.0,
    )

    assert obstacles == {(4, 2)}


def test_vehicle_yaw_rotates_scan_into_map_frame() -> None:
    obstacles = obstacle_cells_from_scan(
        _safe_map(),
        robot_x=2.5,
        robot_y=2.5,
        robot_yaw=pi / 2.0,
        ranges=[2.0],
        angle_min=0.0,
        angle_increment=0.0,
        range_min=0.1,
        range_max=10.0,
        inflation_radius=0.0,
    )

    assert obstacles == {(2, 4)}


def test_ignores_invalid_and_out_of_range_returns() -> None:
    obstacles = obstacle_cells_from_scan(
        _safe_map(),
        robot_x=1.5,
        robot_y=1.5,
        robot_yaw=0.0,
        ranges=[nan, inf, 0.05, 11.0, 2.0],
        angle_min=0.0,
        angle_increment=0.0,
        range_min=0.1,
        range_max=10.0,
        inflation_radius=0.0,
    )

    assert obstacles == {(3, 1)}


def test_inflates_obstacle_without_blocking_the_robot_cell() -> None:
    obstacles = obstacle_cells_from_scan(
        _safe_map(width=3, height=3),
        robot_x=0.5,
        robot_y=0.5,
        robot_yaw=0.0,
        ranges=[1.0],
        angle_min=0.0,
        angle_increment=0.0,
        range_min=0.1,
        range_max=10.0,
        inflation_radius=1.0,
    )

    assert obstacles == {(1, 0), (2, 0), (1, 1)}


def test_overlay_does_not_modify_base_costmap() -> None:
    base = GridCostMap(width=3, height=1, costs=[0, 20, 0])

    overlay = overlay_obstacles(base, {(1, 0)})

    assert list(base.costs) == [0, 20, 0]
    assert overlay.cost_at((1, 0)) == 90
    assert not overlay.traversable((1, 0))


def test_static_no_go_inflation_accounts_for_the_whole_blocked_cell() -> None:
    costs = [0] * 35
    costs[2 * 7 + 3] = 100
    base = GridCostMap(width=7, height=5, costs=costs, resolution=1.0)

    inflated = inflate_no_go_cells(base, clearance_radius=1.0)

    assert base.traversable((3, 2)) is False
    assert inflated.traversable((2, 1)) is False
    assert inflated.traversable((4, 3)) is False
    assert inflated.traversable((1, 2)) is True
    assert list(base.costs).count(100) == 1


def test_static_no_go_inflation_rejects_invalid_clearance() -> None:
    with pytest.raises(ValueError):
        inflate_no_go_cells(_safe_map(), clearance_radius=nan)


def test_planner_detours_around_lidar_obstacle() -> None:
    base = GridCostMap(width=5, height=3, costs=[0] * 15)
    overlay = overlay_obstacles(base, {(2, 1)})

    path = plan_path(overlay, (0, 1), (4, 1))

    assert path is not None
    assert (2, 1) not in path
    assert any(y != 1 for _, y in path)


def test_obstacle_filter_requires_consecutive_detections() -> None:
    obstacle_filter = ObstaclePersistenceFilter(
        confirmation_scans=2,
        clear_scans=3,
    )

    assert obstacle_filter.update({(2, 1)}) == frozenset()
    assert obstacle_filter.update(set()) == frozenset()
    assert obstacle_filter.update({(2, 1)}) == frozenset()
    assert obstacle_filter.update({(2, 1)}) == {(2, 1)}


def test_obstacle_filter_retains_confirmed_cell_through_brief_dropouts() -> None:
    obstacle_filter = ObstaclePersistenceFilter(
        confirmation_scans=2,
        clear_scans=3,
    )

    obstacle_filter.update({(2, 1)})
    assert obstacle_filter.update({(2, 1)}) == {(2, 1)}
    assert obstacle_filter.update(set()) == {(2, 1)}
    assert obstacle_filter.update(set()) == {(2, 1)}
    assert obstacle_filter.update({(2, 1)}) == {(2, 1)}
    assert obstacle_filter.update(set()) == {(2, 1)}
    assert obstacle_filter.update(set()) == {(2, 1)}
    assert obstacle_filter.update(set()) == frozenset()


def test_moving_single_scan_hits_do_not_confirm_through_inflation_overlap() -> None:
    obstacle_filter = ObstaclePersistenceFilter(
        confirmation_scans=2,
        clear_scans=3,
    )

    assert obstacle_filter.update({(2, 2)}) == frozenset()
    assert obstacle_filter.update({(3, 2)}) == frozenset()


def test_inflates_confirmed_hit_centres_after_filtering() -> None:
    obstacle_filter = ObstaclePersistenceFilter(confirmation_scans=2, clear_scans=3)
    obstacle_filter.update({(3, 3)})
    confirmed = obstacle_filter.update({(3, 3)})

    blocked = inflate_obstacle_cells(_safe_map(), confirmed, 1.0)

    assert blocked == {(3, 3), (2, 3), (4, 3), (3, 2), (3, 4)}


def test_associates_a_shifted_surface_return_with_confirmed_obstacle() -> None:
    associated = associate_obstacle_hits(
        observed={(6, 5), (10, 1)},
        confirmed={(5, 5)},
        resolution_m=1.0,
        association_radius_m=1.5,
    )

    assert associated == {(5, 5), (10, 1)}


def test_path_cells_ahead_excludes_the_travelled_prefix() -> None:
    path = [(0, 0), (1, 0), (2, 0), (3, 0), (4, 0)]

    assert path_cells_ahead(path, (3, 0)) == ((3, 0), (4, 0))


def test_removed_obstacle_does_not_reset_an_existing_safe_detour() -> None:
    path = [(0, 0), (1, 0), (2, 0)]

    assert not obstacle_change_requires_replan(
        previous={(4, 4), (5, 5)},
        current={(5, 5)},
        active_path=path,
    )


def test_obstacle_change_replans_for_route_risk_or_route_recovery() -> None:
    path = [(0, 0), (1, 0), (2, 0)]

    assert obstacle_change_requires_replan(set(), {(1, 0)}, path)
    assert not obstacle_change_requires_replan(set(), {(4, 4)}, path)
    assert obstacle_change_requires_replan({(4, 4)}, set(), path)
    assert obstacle_change_requires_replan({(4, 4)}, set(), None)


def test_return_route_is_also_protected_from_new_obstacles() -> None:
    assert obstacle_change_requires_replan(
        set(), {(3, 0)}, [(0, 0), (1, 0)], [(2, 0), (3, 0)]
    )
