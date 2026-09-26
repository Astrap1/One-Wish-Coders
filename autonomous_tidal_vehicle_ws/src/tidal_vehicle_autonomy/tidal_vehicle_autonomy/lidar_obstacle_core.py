"""ROS-independent LiDAR projection and planning-grid obstacle overlay."""

from __future__ import annotations

from math import ceil, cos, hypot, isfinite, sin
from typing import Iterable, Sequence

from .planner_core import GridCell, GridCostMap


class ObstaclePersistenceFilter:
    """Confirm detections and retain obstacles through brief scan dropouts."""

    def __init__(self, confirmation_scans: int, clear_scans: int) -> None:
        if confirmation_scans <= 0 or clear_scans <= 0:
            raise ValueError("Obstacle persistence scan counts must be positive")
        self.confirmation_scans = confirmation_scans
        self.clear_scans = clear_scans
        self._active: set[GridCell] = set()
        self._detection_counts: dict[GridCell, int] = {}
        self._miss_counts: dict[GridCell, int] = {}

    @property
    def active(self) -> frozenset[GridCell]:
        return frozenset(self._active)

    def update(self, observed: Iterable[GridCell]) -> frozenset[GridCell]:
        """Return stable cells after applying consecutive hit/miss thresholds."""
        observed_cells = set(observed)

        # Unconfirmed detections must be consecutive. A one-scan return is
        # discarded rather than being allowed to accumulate over time.
        for cell in set(self._detection_counts) - observed_cells:
            self._detection_counts.pop(cell, None)

        for cell in observed_cells:
            self._miss_counts.pop(cell, None)
            if cell in self._active:
                continue
            count = self._detection_counts.get(cell, 0) + 1
            if count >= self.confirmation_scans:
                self._active.add(cell)
                self._detection_counts.pop(cell, None)
            else:
                self._detection_counts[cell] = count

        for cell in self._active - observed_cells:
            misses = self._miss_counts.get(cell, 0) + 1
            if misses >= self.clear_scans:
                self._active.remove(cell)
                self._miss_counts.pop(cell, None)
            else:
                self._miss_counts[cell] = misses

        return self.active

    def reset(self) -> None:
        self._active.clear()
        self._detection_counts.clear()
        self._miss_counts.clear()


def obstacle_cells_from_scan(
    costmap: GridCostMap,
    robot_x: float,
    robot_y: float,
    robot_yaw: float,
    ranges: Sequence[float],
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    inflation_radius: float,
) -> frozenset[GridCell]:
    """Project valid LiDAR returns into inflated cost-map obstacle cells."""
    _validate_scan_geometry(
        robot_x,
        robot_y,
        robot_yaw,
        angle_min,
        angle_increment,
        range_min,
        range_max,
        inflation_radius,
    )

    obstacles: set[GridCell] = set()
    for index, measured_range in enumerate(ranges):
        distance = float(measured_range)
        if not isfinite(distance) or distance < range_min or distance > range_max:
            continue

        beam_angle = robot_yaw + angle_min + index * angle_increment
        hit_x = robot_x + distance * cos(beam_angle)
        hit_y = robot_y + distance * sin(beam_angle)
        hit_cell = costmap.grid_from_world(hit_x, hit_y)
        if not costmap.in_bounds(hit_cell):
            continue
        obstacles.update(_inflated_cells(costmap, hit_cell, inflation_radius))

    # The cell containing the sensor is occupied by the vehicle itself and
    # must remain a valid A* start. Coarse-grid inflation can otherwise reach
    # back into this cell even when the detected obstacle is outside the
    # configured vehicle clearance.
    obstacles.discard(costmap.grid_from_world(robot_x, robot_y))
    return frozenset(obstacles)


def overlay_obstacles(
    base_costmap: GridCostMap,
    obstacles: Iterable[GridCell],
) -> GridCostMap:
    """Return a cost map with dynamic obstacle cells marked as no-go."""
    costs = list(base_costmap.costs)
    for cell in obstacles:
        if not base_costmap.in_bounds(cell):
            continue
        x, y = cell
        costs[y * base_costmap.width + x] = base_costmap.blocked_cost

    return GridCostMap(
        width=base_costmap.width,
        height=base_costmap.height,
        costs=costs,
        resolution=base_costmap.resolution,
        origin_x=base_costmap.origin_x,
        origin_y=base_costmap.origin_y,
        blocked_cost=base_costmap.blocked_cost,
    )


def _inflated_cells(
    costmap: GridCostMap,
    centre: GridCell,
    inflation_radius: float,
) -> set[GridCell]:
    radius_in_cells = ceil(inflation_radius / costmap.resolution)
    inflated: set[GridCell] = set()
    for offset_y in range(-radius_in_cells, radius_in_cells + 1):
        for offset_x in range(-radius_in_cells, radius_in_cells + 1):
            if hypot(offset_x, offset_y) * costmap.resolution > inflation_radius:
                continue
            cell = (centre[0] + offset_x, centre[1] + offset_y)
            if costmap.in_bounds(cell):
                inflated.add(cell)
    return inflated


def _validate_scan_geometry(
    robot_x: float,
    robot_y: float,
    robot_yaw: float,
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    inflation_radius: float,
) -> None:
    finite_values = (
        robot_x,
        robot_y,
        robot_yaw,
        angle_min,
        angle_increment,
        range_min,
        range_max,
        inflation_radius,
    )
    if not all(isfinite(value) for value in finite_values):
        raise ValueError("LiDAR geometry values must be finite")
    if range_min < 0.0 or range_max <= range_min:
        raise ValueError("LiDAR range limits are invalid")
    if inflation_radius < 0.0:
        raise ValueError("Obstacle inflation radius must not be negative")
