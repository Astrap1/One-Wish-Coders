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

    hits: set[GridCell] = set()
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
        hits.add(hit_cell)

    # The cell containing the sensor is occupied by the vehicle itself and
    # must remain a valid A* start. Coarse-grid inflation can otherwise reach
    # back into this cell even when the detected obstacle is outside the
    # configured vehicle clearance.
    return inflate_obstacle_cells(
        costmap,
        hits,
        inflation_radius,
        excluded_cells={costmap.grid_from_world(robot_x, robot_y)},
    )


def inflate_obstacle_cells(
    costmap: GridCostMap,
    centres: Iterable[GridCell],
    inflation_radius: float,
    excluded_cells: Iterable[GridCell] = (),
) -> frozenset[GridCell]:
    """Inflate confirmed hit centres after temporal filtering."""
    if not isfinite(inflation_radius) or inflation_radius < 0.0:
        raise ValueError("Obstacle inflation radius must be finite and non-negative")
    inflated: set[GridCell] = set()
    for centre in centres:
        if costmap.in_bounds(centre):
            inflated.update(_inflated_cells(costmap, centre, inflation_radius))
    inflated.difference_update(excluded_cells)
    return frozenset(inflated)


def associate_obstacle_hits(
    observed: Iterable[GridCell],
    confirmed: Iterable[GridCell],
    *,
    resolution_m: float,
    association_radius_m: float,
) -> frozenset[GridCell]:
    """Keep successive views of one static obstacle at one grid location.

    A LiDAR normally hits a different surface cell of a trunk or rock as the
    vehicle moves around it.  Associating a new surface cell with a nearby
    confirmed centre prevents that harmless shift from repeatedly changing the
    inflated overlay and causing A* to choose a fresh detour.
    """
    if resolution_m <= 0.0:
        raise ValueError("resolution_m must be positive")
    if association_radius_m < 0.0:
        raise ValueError("association_radius_m must be non-negative")

    confirmed_cells = frozenset(confirmed)
    if association_radius_m == 0.0 or not confirmed_cells:
        return frozenset(observed)

    max_distance_cells = association_radius_m / resolution_m
    associated: set[GridCell] = set()
    for candidate in observed:
        match = min(
            confirmed_cells,
            key=lambda existing: (
                hypot(candidate[0] - existing[0], candidate[1] - existing[1]),
                existing,
            ),
        )
        if (
            hypot(candidate[0] - match[0], candidate[1] - match[1])
            <= max_distance_cells
        ):
            associated.add(match)
        else:
            associated.add(candidate)
    return frozenset(associated)


def path_cells_ahead(
    path: Iterable[GridCell], current_cell: GridCell
) -> tuple[GridCell, ...]:
    """Return the untravelled suffix of a route nearest to the vehicle.

    An obstacle that appears on the already-traversed prefix cannot endanger
    the current command.  Ignoring that prefix prevents a disappearing or
    shifting rearward return from needlessly resetting the forward route.
    """
    cells = tuple(path)
    if not cells:
        return ()
    start_index = min(
        range(len(cells)),
        key=lambda index: (
            hypot(cells[index][0] - current_cell[0], cells[index][1] - current_cell[1]),
            -index,
        ),
    )
    return cells[start_index:]


def obstacle_change_requires_replan(
    previous: Iterable[GridCell],
    current: Iterable[GridCell],
    active_path: Iterable[GridCell] | None,
    return_path: Iterable[GridCell] | None = None,
) -> bool:
    """Return whether an overlay change can invalidate or restore a route.

    Removals that do not clear the overlay cannot make the current route
    unsafe, so retain the existing detour instead of making the follower chase
    every equally valid A* alternative. When no active route exists, any
    removal may reopen one and is assessed immediately.
    """
    previous_cells = set(previous)
    current_cells = set(current)
    active_cells = None if active_path is None else set(active_path)
    return_cells = None if return_path is None else set(return_path)

    removed = previous_cells - current_cells
    if removed and (active_cells is None or not current_cells):
        return True

    added = current_cells - previous_cells
    if not added:
        return False
    if active_cells is None:
        return True
    if added.intersection(active_cells):
        return True
    return return_cells is not None and bool(added.intersection(return_cells))


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


def inflate_no_go_cells(
    base_costmap: GridCostMap,
    clearance_radius: float,
) -> GridCostMap:
    """Inflate static no-go cells by vehicle-centre clearance for A*."""
    if not isfinite(clearance_radius) or clearance_radius < 0.0:
        raise ValueError("No-go clearance radius must be finite and non-negative")
    if clearance_radius == 0.0:
        return base_costmap

    blocked_cells = {
        (x, y)
        for y in range(base_costmap.height)
        for x in range(base_costmap.width)
        if not base_costmap.traversable((x, y))
    }
    inflated: set[GridCell] = set()
    search_cells = ceil(
        clearance_radius / base_costmap.resolution + 0.5
    )
    half_cell = base_costmap.resolution * 0.5
    for centre_x, centre_y in blocked_cells:
        for offset_y in range(-search_cells, search_cells + 1):
            for offset_x in range(-search_cells, search_cells + 1):
                nearest_dx = max(
                    abs(offset_x) * base_costmap.resolution - half_cell,
                    0.0,
                )
                nearest_dy = max(
                    abs(offset_y) * base_costmap.resolution - half_cell,
                    0.0,
                )
                if hypot(nearest_dx, nearest_dy) > clearance_radius:
                    continue
                cell = (centre_x + offset_x, centre_y + offset_y)
                if base_costmap.in_bounds(cell):
                    inflated.add(cell)

    return overlay_obstacles(base_costmap, inflated)


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
