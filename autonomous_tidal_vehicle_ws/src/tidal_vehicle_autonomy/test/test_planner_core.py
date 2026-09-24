from tidal_vehicle_autonomy.planner_core import GridCostMap, plan_path


def test_plans_direct_route_on_safe_terrain() -> None:
    costmap = GridCostMap(width=5, height=1, costs=[0, 0, 0, 0, 0])

    assert plan_path(costmap, (0, 0), (4, 0)) == [(0, 0), (1, 0), (2, 0), (3, 0), (4, 0)]


def test_prefers_low_risk_detour_over_short_high_risk_route() -> None:
    costmap = GridCostMap(
        width=5,
        height=3,
        costs=[
            0, 0, 0, 0, 0,
            0, 80, 80, 80, 0,
            0, 0, 0, 0, 0,
        ],
    )

    path = plan_path(costmap, (0, 1), (4, 1))

    assert path is not None
    assert any(y != 1 for _, y in path)
    assert all(costmap.cost_at(cell) < 80 for cell in path)


def test_unknown_and_no_go_cells_are_not_traversed() -> None:
    costmap = GridCostMap(width=3, height=1, costs=[0, -1, 0])

    assert plan_path(costmap, (0, 0), (2, 0)) is None


def test_does_not_cut_through_blocked_diagonal_corner() -> None:
    costmap = GridCostMap(width=2, height=2, costs=[0, 100, 100, 0])

    assert plan_path(costmap, (0, 0), (1, 1)) is None


def test_converts_between_grid_and_world_coordinates() -> None:
    costmap = GridCostMap(width=2, height=2, costs=[0, 0, 0, 0], resolution=0.5, origin_x=-1.0, origin_y=2.0)

    assert costmap.grid_from_world(-0.26, 2.74) == (1, 1)
    assert costmap.world_from_grid((1, 1)) == (-0.25, 2.75)
