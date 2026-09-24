"""Small, ROS-independent terrain-cost A* planner used by the autonomy node."""

from __future__ import annotations

from dataclasses import dataclass
from heapq import heappop, heappush
from math import hypot
from typing import Sequence

GridCell = tuple[int, int]


@dataclass(frozen=True)
class GridCostMap:
    """A row-major OccupancyGrid-style terrain-risk map.

    Cells in the range 0--89 are traversable and cost progressively more to
    cross. ``-1`` (unknown) and values at or above ``blocked_cost`` are no-go.
    """

    width: int
    height: int
    costs: Sequence[int]
    resolution: float = 1.0
    origin_x: float = 0.0
    origin_y: float = 0.0
    blocked_cost: int = 90

    def __post_init__(self) -> None:
        if self.width <= 0 or self.height <= 0:
            raise ValueError("Grid dimensions must be positive")
        if len(self.costs) != self.width * self.height:
            raise ValueError("Cost data length does not match grid dimensions")
        if self.resolution <= 0.0:
            raise ValueError("Grid resolution must be positive")

    def in_bounds(self, cell: GridCell) -> bool:
        x, y = cell
        return 0 <= x < self.width and 0 <= y < self.height

    def cost_at(self, cell: GridCell) -> int:
        if not self.in_bounds(cell):
            raise IndexError(f"Cell {cell} is outside the grid")
        x, y = cell
        return int(self.costs[y * self.width + x])

    def traversable(self, cell: GridCell) -> bool:
        if not self.in_bounds(cell):
            return False
        cost = self.cost_at(cell)
        return 0 <= cost < self.blocked_cost

    def grid_from_world(self, x: float, y: float) -> GridCell:
        return (
            int((x - self.origin_x) // self.resolution),
            int((y - self.origin_y) // self.resolution),
        )

    def world_from_grid(self, cell: GridCell) -> tuple[float, float]:
        x, y = cell
        if not self.in_bounds(cell):
            raise IndexError(f"Cell {cell} is outside the grid")
        return (
            self.origin_x + (x + 0.5) * self.resolution,
            self.origin_y + (y + 0.5) * self.resolution,
        )


def plan_path(costmap: GridCostMap, start: GridCell, goal: GridCell) -> list[GridCell] | None:
    """Return a lowest-risk path, or ``None`` when no safe path exists."""
    if not costmap.traversable(start) or not costmap.traversable(goal):
        return None

    frontier: list[tuple[float, int, GridCell]] = []
    heappush(frontier, (_heuristic(start, goal), 0, start))
    came_from: dict[GridCell, GridCell | None] = {start: None}
    path_cost: dict[GridCell, float] = {start: 0.0}
    sequence = 0

    while frontier:
        _, _, current = heappop(frontier)
        if current == goal:
            return _reconstruct_path(came_from, goal)

        for neighbour, distance in _neighbours(costmap, current):
            terrain_multiplier = 1.0 + costmap.cost_at(neighbour) / 100.0
            candidate_cost = path_cost[current] + distance * terrain_multiplier
            if candidate_cost >= path_cost.get(neighbour, float("inf")):
                continue

            path_cost[neighbour] = candidate_cost
            came_from[neighbour] = current
            sequence += 1
            priority = candidate_cost + _heuristic(neighbour, goal)
            heappush(frontier, (priority, sequence, neighbour))

    return None


def _neighbours(costmap: GridCostMap, cell: GridCell) -> list[tuple[GridCell, float]]:
    x, y = cell
    neighbours: list[tuple[GridCell, float]] = []
    for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)):
        neighbour = (x + dx, y + dy)
        if not costmap.traversable(neighbour):
            continue
        if dx and dy:
            if not costmap.traversable((x + dx, y)) or not costmap.traversable((x, y + dy)):
                continue
            neighbours.append((neighbour, hypot(dx, dy)))
        else:
            neighbours.append((neighbour, 1.0))
    return neighbours


def _heuristic(first: GridCell, second: GridCell) -> float:
    return hypot(first[0] - second[0], first[1] - second[1])


def _reconstruct_path(came_from: dict[GridCell, GridCell | None], goal: GridCell) -> list[GridCell]:
    path = [goal]
    while came_from[path[-1]] is not None:
        path.append(came_from[path[-1]])
    path.reverse()
    return path
