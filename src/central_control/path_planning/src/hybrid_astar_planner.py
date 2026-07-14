"""Heading-aware Hybrid A* planner using a kinematic bicycle model."""

from dataclasses import dataclass
import heapq
import itertools
import math
import time
from typing import Iterable

import numpy as np


@dataclass(frozen=True)
class HybridState:
    """Continuous vehicle pose stored by Hybrid A*."""

    x_cm: float
    y_cm: float
    yaw_rad: float
    direction: int
    steer_rad: float = 0.0


@dataclass(frozen=True)
class HybridAStarResult:
    """Hybrid A* path and search diagnostics."""

    path: list[HybridState]
    total_cost: float
    success: bool
    expanded_nodes: int
    message: str


class HybridAStarPlanner:
    """Search over position, heading, steering, and forward/reverse motion."""

    def __init__(
        self,
        grid: np.ndarray,
        resolution_cm: float = 1.0,
        wheelbase_cm: float = 8.0,
        motion_step_cm: float = 3.0,
        yaw_resolution_deg: float = 10.0,
        steer_set_deg: tuple[float, ...] = (-30.0, 0.0, 30.0),
        allow_reverse: bool = True,
        obstacle_threshold: int = 50,
        timeout_sec: float = 5.0,
        goal_tolerance_cm: float = 4.0,
        goal_yaw_tolerance_deg: float = 15.0,
        reverse_penalty: float = 1.5,
        gear_switch_penalty: float = 5.0,
        steer_penalty: float = 0.2,
        steer_change_penalty: float = 0.5,
    ) -> None:
        if grid.ndim != 2:
            raise ValueError("grid must be a two-dimensional array")
        if resolution_cm <= 0 or wheelbase_cm <= 0 or motion_step_cm <= 0:
            raise ValueError("resolution, wheelbase, and motion step must be positive")
        if yaw_resolution_deg <= 0 or timeout_sec <= 0:
            raise ValueError("yaw resolution and timeout must be positive")
        if not steer_set_deg:
            raise ValueError("steer_set_deg must contain at least one steering angle")

        self.grid = grid
        self.height, self.width = grid.shape
        self.resolution_cm = resolution_cm
        self.wheelbase_cm = wheelbase_cm
        self.motion_step_cm = motion_step_cm
        self.yaw_resolution_rad = math.radians(yaw_resolution_deg)
        self.steer_set_rad = tuple(math.radians(value) for value in steer_set_deg)
        self.allow_reverse = allow_reverse
        self.obstacle_threshold = obstacle_threshold
        self.timeout_sec = timeout_sec
        self.goal_tolerance_cm = goal_tolerance_cm
        self.goal_yaw_tolerance_rad = math.radians(goal_yaw_tolerance_deg)
        self.reverse_penalty = reverse_penalty
        self.gear_switch_penalty = gear_switch_penalty
        self.steer_penalty = steer_penalty
        self.steer_change_penalty = steer_change_penalty
        self.yaw_bin_count = max(1, round(2.0 * math.pi / self.yaw_resolution_rad))

    def plan(
        self,
        start: tuple[float, float, float],
        goal: tuple[float, float, float],
    ) -> HybridAStarResult:
        """Plan from a continuous start pose to a position-and-heading goal pose."""
        start_state = HybridState(start[0], start[1], self._normalize_yaw(start[2]), 1)
        goal_yaw = self._normalize_yaw(goal[2])
        self._validate_pose("start", start_state.x_cm, start_state.y_cm)
        self._validate_pose("goal", goal[0], goal[1])

        start_key = self._state_key(start_state)
        counter = itertools.count()
        open_heap: list[tuple[float, int, tuple[int, int, int, int, int]]] = []
        heapq.heappush(
            open_heap,
            (self._heuristic(start_state, goal), next(counter), start_key),
        )
        states = {start_key: start_state}
        parents: dict[tuple[int, int, int, int, int], tuple[int, int, int, int, int]] = {}
        g_score = {start_key: 0.0}
        closed: set[tuple[int, int, int, int, int]] = set()
        started_at = time.monotonic()
        expanded_nodes = 0

        while open_heap:
            if time.monotonic() - started_at > self.timeout_sec:
                return HybridAStarResult(
                    [], math.inf, False, expanded_nodes, "timeout"
                )

            _, _, current_key = heapq.heappop(open_heap)
            if current_key in closed:
                continue
            current = states[current_key]
            if self._is_goal(current, goal[0], goal[1], goal_yaw):
                return HybridAStarResult(
                    self._reconstruct(states, parents, current_key),
                    g_score[current_key],
                    True,
                    expanded_nodes,
                    "goal reached",
                )

            closed.add(current_key)
            expanded_nodes += 1
            for neighbor, primitive_cost in self._neighbors(current):
                neighbor_key = self._state_key(neighbor)
                tentative_cost = g_score[current_key] + primitive_cost
                if tentative_cost >= g_score.get(neighbor_key, math.inf):
                    continue
                states[neighbor_key] = neighbor
                parents[neighbor_key] = current_key
                g_score[neighbor_key] = tentative_cost
                priority = tentative_cost + self._heuristic(neighbor, goal)
                heapq.heappush(open_heap, (priority, next(counter), neighbor_key))

        return HybridAStarResult([], math.inf, False, expanded_nodes, "open set exhausted")

    def _neighbors(self, state: HybridState) -> Iterable[tuple[HybridState, float]]:
        directions = (1, -1) if self.allow_reverse else (1,)
        for direction in directions:
            for steer_rad in self.steer_set_rad:
                neighbor = self._simulate_motion(state, steer_rad, direction)
                if neighbor is None:
                    continue
                cost = self.motion_step_cm
                if direction < 0:
                    cost *= self.reverse_penalty
                if direction != state.direction:
                    cost += self.gear_switch_penalty
                max_steer = max(abs(value) for value in self.steer_set_rad) or 1.0
                cost += self.steer_penalty * abs(steer_rad) / max_steer
                cost += self.steer_change_penalty * abs(steer_rad - state.steer_rad) / max_steer
                yield neighbor, cost

    def _simulate_motion(
        self, state: HybridState, steer_rad: float, direction: int
    ) -> HybridState | None:
        """Integrate one bicycle primitive and reject it on any sampled collision."""
        # Sample at most every half grid cell so a primitive cannot jump through a wall.
        collision_step = max(0.25, self.resolution_cm * 0.5)
        sample_count = max(1, math.ceil(self.motion_step_cm / collision_step))
        signed_distance = direction * self.motion_step_cm / sample_count
        x_cm, y_cm, yaw_rad = state.x_cm, state.y_cm, state.yaw_rad

        for _ in range(sample_count):
            next_yaw = yaw_rad + signed_distance / self.wheelbase_cm * math.tan(steer_rad)
            midpoint_yaw = yaw_rad + 0.5 * (next_yaw - yaw_rad)
            x_cm += signed_distance * math.cos(midpoint_yaw)
            y_cm += signed_distance * math.sin(midpoint_yaw)
            yaw_rad = self._normalize_yaw(next_yaw)
            if self._is_collision(x_cm, y_cm):
                return None
        return HybridState(x_cm, y_cm, yaw_rad, direction, steer_rad)

    def _state_key(self, state: HybridState) -> tuple[int, int, int, int, int]:
        x_bin = round(state.x_cm / self.resolution_cm)
        y_bin = round(state.y_cm / self.resolution_cm)
        yaw_bin = round((state.yaw_rad + math.pi) / self.yaw_resolution_rad) % self.yaw_bin_count
        steer_bin = min(
            range(len(self.steer_set_rad)),
            key=lambda index: abs(self.steer_set_rad[index] - state.steer_rad),
        )
        return x_bin, y_bin, yaw_bin, state.direction, steer_bin

    def _heuristic(
        self, state: HybridState, goal: tuple[float, float, float]
    ) -> float:
        distance = math.hypot(goal[0] - state.x_cm, goal[1] - state.y_cm)
        yaw_error = abs(self._angle_difference(goal[2], state.yaw_rad))
        # A mild heading term guides search without dominating obstacle detours.
        return 1.1 * distance + 0.2 * self.wheelbase_cm * yaw_error

    def _is_goal(
        self, state: HybridState, goal_x: float, goal_y: float, goal_yaw: float
    ) -> bool:
        return (
            math.hypot(goal_x - state.x_cm, goal_y - state.y_cm)
            <= self.goal_tolerance_cm
            and abs(self._angle_difference(goal_yaw, state.yaw_rad))
            <= self.goal_yaw_tolerance_rad
        )

    def _validate_pose(self, label: str, x_cm: float, y_cm: float) -> None:
        if self._is_collision(x_cm, y_cm):
            raise ValueError(f"{label} pose ({x_cm:.2f}, {y_cm:.2f}) is invalid or occupied")

    def _is_collision(self, x_cm: float, y_cm: float) -> bool:
        gx = round(x_cm / self.resolution_cm)
        gy = round(y_cm / self.resolution_cm)
        if not (0 <= gx < self.width and 0 <= gy < self.height):
            return True
        return bool(self.grid[gy, gx] >= self.obstacle_threshold)

    @staticmethod
    def _reconstruct(
        states: dict[tuple[int, int, int, int, int], HybridState],
        parents: dict[
            tuple[int, int, int, int, int], tuple[int, int, int, int, int]
        ],
        current_key: tuple[int, int, int, int, int],
    ) -> list[HybridState]:
        path = [states[current_key]]
        while current_key in parents:
            current_key = parents[current_key]
            path.append(states[current_key])
        path.reverse()
        return path

    @staticmethod
    def _normalize_yaw(yaw_rad: float) -> float:
        return (yaw_rad + math.pi) % (2.0 * math.pi) - math.pi

    @staticmethod
    def _angle_difference(first: float, second: float) -> float:
        return (first - second + math.pi) % (2.0 * math.pi) - math.pi
