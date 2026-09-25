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
        no_go_cost=90,
        wheel_cost_max=19,
        elevated_hover_cost_min=60,
        wheel_energy_percent_per_m=1.0,
        hover_energy_percent_per_m=2.0,
        wheel_nominal_speed_mps=1.0,
        hover_nominal_speed_mps=0.5,
        elevated_hover_energy_multiplier=1.25,
        elevated_hover_speed_multiplier=0.7,
        mode_transition_time_s=5.0,
        mode_transition_energy_percent=1.0,
        terrain_cost_energy_weight=0.5,
        mobility_degradation_weight=0.7,
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
    blocked[1 * 10 + 2] = 90

    assert not estimate_return([(1.0, 1.0), (3.0, 1.0)], _grid(unknown), 50.0, 100.0, _config()).valid
    assert not estimate_return([(1.0, 1.0), (3.0, 1.0)], _grid(blocked), 50.0, 100.0, _config()).valid


def test_mixed_wheel_hover_route_accounts_for_transition_cost_and_time():
    terrain = [0] * 100
    for column in range(1, 10):
        terrain[column] = 20

    estimate = estimate_return(
        [(0.1, 0.5), (2.9, 0.5)], _grid(terrain), 80.0, 100.0, _config()
    )

    assert estimate.valid
    assert estimate.mode_transition_count == 1
    assert estimate.eta_s > 5.0
    assert estimate.estimated_energy_percent > 8.0


def test_elevated_hover_band_costs_more_than_normal_hover():
    normal = [20] * 100
    elevated = [60] * 100

    normal_estimate = estimate_return([(1.0, 1.0), (3.0, 1.0)], _grid(normal), 80.0, 100.0, _config())
    elevated_estimate = estimate_return([(1.0, 1.0), (3.0, 1.0)], _grid(elevated), 80.0, 100.0, _config())

    assert elevated_estimate.estimated_energy_percent > normal_estimate.estimated_energy_percent
    assert elevated_estimate.eta_s > normal_estimate.eta_s


def test_home_validation_uses_the_final_path_pose():
    assert path_ends_at_home([(1.0, 1.0), (2.0, 2.0)], (2.5, 2.0), 0.6)
    assert not path_ends_at_home([(1.0, 1.0), (2.0, 2.0)], (5.0, 5.0), 2.0)
