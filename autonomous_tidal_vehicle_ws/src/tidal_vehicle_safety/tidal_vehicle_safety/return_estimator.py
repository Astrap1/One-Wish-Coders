"""Pure terrain-aware energy and time estimation for a route to HOME."""

from __future__ import annotations

from dataclasses import dataclass
from math import floor, hypot
from typing import Sequence


Point2 = tuple[float, float]


@dataclass(frozen=True)
class GridCostmap:
    """Minimal OccupancyGrid representation independent of ROS messages."""

    width: int
    height: int
    resolution_m: float
    origin_x_m: float
    origin_y_m: float
    data: Sequence[int]

    def cost_at(self, x_m: float, y_m: float) -> int | None:
        """Return the cost at a map coordinate, or None when outside the grid."""
        column = floor((x_m - self.origin_x_m) / self.resolution_m)
        row = floor((y_m - self.origin_y_m) / self.resolution_m)
        if column < 0 or row < 0 or column >= self.width or row >= self.height:
            return None
        return self.data[row * self.width + column]


@dataclass(frozen=True)
class ReturnEstimatorConfig:
    sample_spacing_m: float
    no_go_cost: int
    track_mode_enabled: bool
    track_cost_max: int
    elevated_hover_cost_min: int
    track_energy_percent_per_m: float
    hover_energy_percent_per_m: float
    track_nominal_speed_mps: float
    hover_nominal_speed_mps: float
    elevated_hover_energy_multiplier: float
    elevated_hover_speed_multiplier: float
    mode_transition_time_s: float
    mode_transition_energy_percent: float
    terrain_cost_energy_weight: float
    mobility_degradation_weight: float
    minimum_buffer_percent: float
    contingency_ratio: float


@dataclass(frozen=True)
class ReturnEstimate:
    valid: bool
    reason: str
    route_length_m: float = 0.0
    estimated_energy_percent: float = float("nan")
    margin_percent: float = float("nan")
    eta_s: float = float("nan")
    mode_transition_count: int = 0


def path_ends_at_home(path: Sequence[Point2], home: Point2, tolerance_m: float) -> bool:
    """Check whether the final route pose ends within the configured HOME zone."""
    if not path:
        return False
    end_x, end_y = path[-1]
    return hypot(end_x - home[0], end_y - home[1]) <= tolerance_m


def estimate_return(
    path: Sequence[Point2],
    costmap: GridCostmap,
    battery_percent: float,
    mobility_health_percent: float,
    config: ReturnEstimatorConfig,
) -> ReturnEstimate:
    """Estimate return energy and ETA using route samples and terrain costs.

    Occupancy values below zero are unknown and values at/above ``no_go_cost``
    are not traversable. The calculation is a declared simulation abstraction,
    not a physical vehicle model.
    """
    if not path:
        return ReturnEstimate(False, "Return route is empty.")
    if costmap.width <= 0 or costmap.height <= 0 or costmap.resolution_m <= 0:
        return ReturnEstimate(False, "Terrain cost map is invalid.")
    if not 0.0 < mobility_health_percent <= 100.0:
        return ReturnEstimate(False, "Mobility health cannot support an estimate.")
    if (
        config.sample_spacing_m <= 0
        or config.track_nominal_speed_mps <= 0
        or config.hover_nominal_speed_mps <= 0
    ):
        return ReturnEstimate(False, "Return-estimator configuration is invalid.")

    terrain_multiplier_base = 1.0 + config.mobility_degradation_weight * (
        1.0 - mobility_health_percent / 100.0
    )
    total_length_m = 0.0
    raw_energy_percent = 0.0
    eta_s = 0.0
    transition_count = 0
    previous_mode: str | None = None

    for start, end in _segments(path):
        segment_x = end[0] - start[0]
        segment_y = end[1] - start[1]
        segment_length = hypot(segment_x, segment_y)
        if segment_length == 0:
            continue
        samples = max(1, int((segment_length + config.sample_spacing_m - 1e-9) / config.sample_spacing_m))
        sample_length = segment_length / samples
        for index in range(samples):
            fraction = (index + 0.5) / samples
            x_m = start[0] + segment_x * fraction
            y_m = start[1] + segment_y * fraction
            cost = costmap.cost_at(x_m, y_m)
            if cost is None:
                return ReturnEstimate(False, "Return route leaves the terrain cost map.")
            if cost < 0:
                return ReturnEstimate(False, "Return route crosses unknown terrain.")
            if cost >= config.no_go_cost:
                return ReturnEstimate(False, "Return route crosses a no-go terrain cell.")

            mode = _mode_for_cost(cost, config)
            if previous_mode is not None and mode != previous_mode:
                raw_energy_percent += config.mode_transition_energy_percent
                eta_s += config.mode_transition_time_s
                transition_count += 1
            previous_mode = mode

            terrain_multiplier = terrain_multiplier_base * (
                1.0 + config.terrain_cost_energy_weight * min(cost, 100) / 100.0
            )
            energy_per_m, speed_mps = _mode_rates(mode, cost, config)
            raw_energy_percent += sample_length * energy_per_m * terrain_multiplier
            eta_s += sample_length / (speed_mps / terrain_multiplier)
            total_length_m += sample_length

    if len(path) == 1:
        cost = costmap.cost_at(*path[0])
        if cost is None or cost < 0 or cost >= config.no_go_cost:
            return ReturnEstimate(False, "HOME point is not traversable.")

    buffer_percent = max(
        config.minimum_buffer_percent,
        raw_energy_percent * config.contingency_ratio,
    )
    estimated_energy_percent = raw_energy_percent + buffer_percent
    return ReturnEstimate(
        True,
        "Return route is valid.",
        total_length_m,
        estimated_energy_percent,
        battery_percent - estimated_energy_percent,
        eta_s,
        transition_count,
    )


def estimate_lidar_return(
    path: Sequence[Point2],
    battery_percent: float,
    mobility_health_percent: float,
    config: ReturnEstimatorConfig,
) -> ReturnEstimate:
    """Estimate a LiDAR-only return without sampling terrain-map values.

    This deliberately uses the conservative hover profile for every segment.
    Obstacle validation belongs to Autonomy's confirmed LiDAR overlay; a
    terrain-cost map remains a display/reference input, not a route veto.
    """
    if not path:
        return ReturnEstimate(False, "Return route is empty.")
    if not 0.0 < mobility_health_percent <= 100.0:
        return ReturnEstimate(False, "Mobility health cannot support an estimate.")
    if config.hover_nominal_speed_mps <= 0.0:
        return ReturnEstimate(False, "Return-estimator configuration is invalid.")

    terrain_multiplier = 1.0 + config.mobility_degradation_weight * (
        1.0 - mobility_health_percent / 100.0
    )
    route_length_m = sum(
        hypot(end[0] - start[0], end[1] - start[1])
        for start, end in _segments(path)
    )
    raw_energy_percent = (
        route_length_m * config.hover_energy_percent_per_m * terrain_multiplier
    )
    eta_s = route_length_m / (config.hover_nominal_speed_mps / terrain_multiplier)
    buffer_percent = max(
        config.minimum_buffer_percent,
        raw_energy_percent * config.contingency_ratio,
    )
    estimated_energy_percent = raw_energy_percent + buffer_percent
    return ReturnEstimate(
        True,
        "LiDAR return route is valid.",
        route_length_m,
        estimated_energy_percent,
        battery_percent - estimated_energy_percent,
        eta_s,
    )


def _segments(path: Sequence[Point2]) -> list[tuple[Point2, Point2]]:
    return list(zip(path, path[1:]))


def _mode_for_cost(cost: int, config: ReturnEstimatorConfig) -> str:
    """Select the declared return-estimation mode for a cost-map cell.

    Version 1 of the Gazebo vehicle is intentionally hover-only, so it must
    estimate firm-shore segments with the hover profile. Enable TRACK only when
    the Version 2 tracked vehicle and its transitions are validated.
    """
    if config.track_mode_enabled and cost <= config.track_cost_max:
        return "TRACK"
    return "HOVER"


def _mode_rates(
    mode: str, cost: int, config: ReturnEstimatorConfig
) -> tuple[float, float]:
    if mode == "TRACK":
        return config.track_energy_percent_per_m, config.track_nominal_speed_mps

    energy_per_m = config.hover_energy_percent_per_m
    speed_mps = config.hover_nominal_speed_mps
    if cost >= config.elevated_hover_cost_min:
        energy_per_m *= config.elevated_hover_energy_multiplier
        speed_mps *= config.elevated_hover_speed_multiplier
    return energy_per_m, speed_mps
