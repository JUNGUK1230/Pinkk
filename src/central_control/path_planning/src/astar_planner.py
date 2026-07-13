"""Two-dimensional A* search on a binary occupancy grid."""

from dataclasses import dataclass
import heapq
import math
from typing import Iterable

import numpy as np

GridPoint = tuple[int, int]


@dataclass(frozen=True)
class AStarResult:
    """A* result containing a start-to-goal path and its movement cost."""

    path: list[GridPoint]
    total_cost: float
    success: bool


class AStarPlanner:
    """Plan a shortest collision-free path using 4- or 8-connected motion."""

    def __init__(
        self,
        grid: np.ndarray,
        allow_diagonal: bool = True,
        prevent_corner_cutting: bool = True,
        obstacle_threshold: int = 50,
    ) -> None:
        if grid.ndim != 2:
            raise ValueError("grid must be a two-dimensional array")
        self.grid = grid
        self.height, self.width = grid.shape
        self.allow_diagonal = allow_diagonal
        self.prevent_corner_cutting = prevent_corner_cutting
        self.obstacle_threshold = obstacle_threshold

    def plan(self, start: GridPoint, goal: GridPoint) -> AStarResult:
        """Run A* from start to goal; return success=False when no path exists."""
        self._validate_endpoint("start", start)
        self._validate_endpoint("goal", goal)

        if start == goal:
            return AStarResult([start], 0.0, True)

        # The heap may contain stale entries; g_score remains the authoritative cost.
        open_heap: list[tuple[float, float, GridPoint]] = []
        heapq.heappush(open_heap, (self._heuristic(start, goal), 0.0, start))
        came_from: dict[GridPoint, GridPoint] = {}
        g_score: dict[GridPoint, float] = {start: 0.0}
        closed: set[GridPoint] = set()

        while open_heap:
            _, queued_cost, current = heapq.heappop(open_heap)
            if current in closed or queued_cost > g_score[current]:
                continue
            if current == goal:
                path = self._reconstruct_path(came_from, current)
                return AStarResult(path, g_score[current], True)

            closed.add(current)
            for neighbor, move_cost in self._neighbors(current):
                tentative_cost = g_score[current] + move_cost
                if tentative_cost >= g_score.get(neighbor, math.inf):
                    continue
                came_from[neighbor] = current
                g_score[neighbor] = tentative_cost
                priority = tentative_cost + self._heuristic(neighbor, goal)
                heapq.heappush(open_heap, (priority, tentative_cost, neighbor))

        return AStarResult([], math.inf, False)

    def _neighbors(self, point: GridPoint) -> Iterable[tuple[GridPoint, float]]:
        x, y = point
        motions = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0)]
        if self.allow_diagonal:
            diagonal_cost = math.sqrt(2.0)
            motions.extend(
                [(1, 1, diagonal_cost), (1, -1, diagonal_cost),
                 (-1, 1, diagonal_cost), (-1, -1, diagonal_cost)]
            )

        for dx, dy, cost in motions:
            neighbor = (x + dx, y + dy)
            if not self._in_bounds(neighbor) or self._is_obstacle(neighbor):
                continue
            if dx != 0 and dy != 0 and self.prevent_corner_cutting:
                # Both side cells must be free, otherwise the diagonal clips a corner.
                if self._is_obstacle((x + dx, y)) or self._is_obstacle((x, y + dy)):
                    continue
            yield neighbor, cost

    def _validate_endpoint(self, name: str, point: GridPoint) -> None:
        if not self._in_bounds(point):
            raise ValueError(f"{name} {point} is outside the grid")
        if self._is_obstacle(point):
            raise ValueError(f"{name} {point} is an obstacle")

    def _in_bounds(self, point: GridPoint) -> bool:
        x, y = point
        return 0 <= x < self.width and 0 <= y < self.height

    def _is_obstacle(self, point: GridPoint) -> bool:
        x, y = point
        return bool(self.grid[y, x] >= self.obstacle_threshold)

    @staticmethod
    def _heuristic(first: GridPoint, second: GridPoint) -> float:
        return math.hypot(first[0] - second[0], first[1] - second[1])

    @staticmethod
    def _reconstruct_path(
        came_from: dict[GridPoint, GridPoint], current: GridPoint
    ) -> list[GridPoint]:
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path
