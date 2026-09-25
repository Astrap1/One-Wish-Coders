from math import inf, nan, pi

from tidal_vehicle_autonomy.lidar_obstacle_core import (
    obstacle_cells_from_scan,
    overlay_obstacles,
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


def test_inflates_obstacle_within_map_bounds() -> None:
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

    assert obstacles == {(0, 0), (1, 0), (2, 0), (1, 1)}


def test_overlay_does_not_modify_base_costmap() -> None:
    base = GridCostMap(width=3, height=1, costs=[0, 20, 0])

    overlay = overlay_obstacles(base, {(1, 0)})

    assert list(base.costs) == [0, 20, 0]
    assert overlay.cost_at((1, 0)) == 90
    assert not overlay.traversable((1, 0))


def test_planner_detours_around_lidar_obstacle() -> None:
    base = GridCostMap(width=5, height=3, costs=[0] * 15)
    overlay = overlay_obstacles(base, {(2, 1)})

    path = plan_path(overlay, (0, 1), (4, 1))

    assert path is not None
    assert (2, 1) not in path
    assert any(y != 1 for _, y in path)
