from math import isclose

from tidal_vehicle_safety.return_estimator import (
    GridCostmap,
    ReturnEstimatorConfig,
    estimate_return,
    path_ends_at_home,
)


def _config() -> ReturnEstimatorConfig:
    return ReturnEstimatorConfig(
        sample_spacing_m=0.5,
        no_go_cost=100,
        base_energy_percent_per_m=1.0,
        terrain_cost_energy_weight=0.5,
        mobility_degradation_weight=0.7,
        nominal_speed_mps=1.0,
        minimum_buffer_percent=8.0,
        contingency_ratio=0.2,
    )


def _grid(data=None) -> GridCostmap:
    return GridCostmap(
        width=10,
        height=10,
        resolution_m=1.0,
        origin_x_m=0.0,
        origin_y_m=0.0,
        data=[0] * 100 if data is None else data,
    )


def test_estimate_includes_minimum_buffer_and_margin():
    estimate = estimate_return([(1.0, 1.0), (3.0, 1.0)], _grid(), 50.0, 100.0, _config())

    assert estimate.valid
    assert isclose(estimate.route_length_m, 2.0)
    assert isclose(estimate.estimated_energy_percent, 10.0)
    assert isclose(estimate.margin_percent, 40.0)
    assert isclose(estimate.eta_s, 2.0)


def test_estimate_rejects_unknown_or_no_go_cells():
    unknown = [0] * 100
    unknown[1 * 10 + 2] = -1
    blocked = [0] * 100
    blocked[1 * 10 + 2] = 100

    assert not estimate_return([(1.0, 1.0), (3.0, 1.0)], _grid(unknown), 50.0, 100.0, _config()).valid
    assert not estimate_return([(1.0, 1.0), (3.0, 1.0)], _grid(blocked), 50.0, 100.0, _config()).valid


def test_home_validation_uses_the_final_path_pose():
    assert path_ends_at_home([(1.0, 1.0), (2.0, 2.0)], (2.5, 2.0), 0.6)
    assert not path_ends_at_home([(1.0, 1.0), (2.0, 2.0)], (5.0, 5.0), 2.0)
