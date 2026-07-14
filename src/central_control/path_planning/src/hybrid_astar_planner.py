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
        vehicle_length_cm: float = 12.0,
        vehicle_width_cm: float = 11.0,
        rear_overhang_cm: float | None = None,
        motion_step_cm: float = 3.0,
        path_output_step_cm: float = 0.5,
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
        if (
            resolution_cm <= 0
            or wheelbase_cm <= 0
            or vehicle_length_cm <= 0
            or vehicle_width_cm <= 0
            or motion_step_cm <= 0
            or path_output_step_cm <= 0
        ):
            raise ValueError(
                "resolution, vehicle dimensions, wheelbase, motion step, and "
                "path output step must be positive"
            )
        if vehicle_length_cm < wheelbase_cm:
            raise ValueError("vehicle length must not be shorter than wheelbase")
        if yaw_resolution_deg <= 0 or timeout_sec <= 0:
            raise ValueError("yaw resolution and timeout must be positive")
        if not steer_set_deg:
            raise ValueError("steer_set_deg must contain at least one steering angle")

        self.grid = grid
        self.height, self.width = grid.shape
        self.resolution_cm = resolution_cm
        self.wheelbase_cm = wheelbase_cm
        self.vehicle_length_cm = vehicle_length_cm
        self.vehicle_width_cm = vehicle_width_cm
        self.rear_overhang_cm = (
            (vehicle_length_cm - wheelbase_cm) * 0.5
            if rear_overhang_cm is None
            else rear_overhang_cm
        )
        if not 0.0 <= self.rear_overhang_cm < self.vehicle_length_cm:
            raise ValueError(
                "rear_overhang_cm must be zero or positive and shorter than the vehicle"
            )
        self.front_extent_cm = self.vehicle_length_cm - self.rear_overhang_cm
        self.motion_step_cm = motion_step_cm
        self.path_output_step_cm = path_output_step_cm
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
        # Cache occupied grid offsets for each discrete heading. Collision checks
        # then use NumPy indexing instead of rebuilding a rotated polygon for every
        # motion sample expanded by the search.
        self._footprint_offsets = self._build_footprint_offsets()

    def plan(
        self,
        start: tuple[float, float, float],
        goal: tuple[float, float, float],
    ) -> HybridAStarResult:
        """Plan from a continuous start pose to a position-and-heading goal pose."""
        start_state = HybridState(start[0], start[1], self._normalize_yaw(start[2]), 1)
        goal_yaw = self._normalize_yaw(goal[2])
        self._validate_pose(
            "start", start_state.x_cm, start_state.y_cm, start_state.yaw_rad
        )
        self._validate_pose("goal", goal[0], goal[1], goal_yaw)

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
                sparse_path = self._reconstruct(states, parents, current_key)
                return HybridAStarResult(
                    self._densify_path(sparse_path),
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
            if self._is_collision(x_cm, y_cm, yaw_rad):
                return None
        return HybridState(x_cm, y_cm, yaw_rad, direction, steer_rad)

    def _state_key(self, state: HybridState) -> tuple[int, int, int, int, int]:
        x_bin = round(state.x_cm / self.resolution_cm)
        y_bin = round(state.y_cm / self.resolution_cm)
        yaw_bin = self._yaw_bin(state.yaw_rad)
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

    def _validate_pose(
        self, label: str, x_cm: float, y_cm: float, yaw_rad: float
    ) -> None:
        if self._is_collision(x_cm, y_cm, yaw_rad):
            raise ValueError(
                f"{label} pose ({x_cm:.2f}, {y_cm:.2f}, "
                f"{math.degrees(yaw_rad):.1f} deg) has an invalid vehicle footprint"
            )

    def is_pose_collision(self, x_cm: float, y_cm: float, yaw_rad: float) -> bool:
        """Return whether the rotated rectangular vehicle footprint collides."""
        return self._is_collision(x_cm, y_cm, self._normalize_yaw(yaw_rad))

    def find_nearest_valid_pose(
        self,
        x_cm: float,
        y_cm: float,
        yaw_rad: float,
        max_radius_cm: float = 30.0,
    ) -> tuple[float, float]:
        """Find the closest grid-aligned position valid for a fixed heading."""
        if max_radius_cm < 0:
            raise ValueError("max_radius_cm must be zero or positive")

        origin_x = min(max(round(x_cm / self.resolution_cm), 0), self.width - 1)
        origin_y = min(max(round(y_cm / self.resolution_cm), 0), self.height - 1)
        max_radius_cells = math.ceil(max_radius_cm / self.resolution_cm)
        offsets = [
            (dx * dx + dy * dy, dy, dx)
            for dy in range(-max_radius_cells, max_radius_cells + 1)
            for dx in range(-max_radius_cells, max_radius_cells + 1)
            if dx * dx + dy * dy <= max_radius_cells * max_radius_cells
        ]
        offsets.sort()

        for _, dy, dx in offsets:
            candidate_x = origin_x + dx
            candidate_y = origin_y + dy
            if not (0 <= candidate_x < self.width and 0 <= candidate_y < self.height):
                continue
            candidate_x_cm = candidate_x * self.resolution_cm
            candidate_y_cm = candidate_y * self.resolution_cm
            if not self._is_collision(candidate_x_cm, candidate_y_cm, yaw_rad):
                return candidate_x_cm, candidate_y_cm

        raise ValueError(
            f"No footprint-valid pose found within {max_radius_cm:.1f} cm of "
            f"({x_cm:.2f}, {y_cm:.2f})"
        )

    def _is_collision(self, x_cm: float, y_cm: float, yaw_rad: float) -> bool:
        """Check every grid cell covered by the rotated vehicle rectangle."""
        gx = round(x_cm / self.resolution_cm)
        gy = round(y_cm / self.resolution_cm)
        if not (0 <= gx < self.width and 0 <= gy < self.height):
            return True

        offset_y, offset_x = self._footprint_offsets[self._yaw_bin(yaw_rad)]
        footprint_x = gx + offset_x
        footprint_y = gy + offset_y
        if (
            np.any(footprint_x < 0)
            or np.any(footprint_x >= self.width)
            or np.any(footprint_y < 0)
            or np.any(footprint_y >= self.height)
        ):
            return True
        return bool(
            np.any(self.grid[footprint_y, footprint_x] >= self.obstacle_threshold)
        )

    def _build_footprint_offsets(self) -> tuple[tuple[np.ndarray, np.ndarray], ...]:
        """Precompute conservative footprint cells for every heading bin."""
        # Half a cell of padding includes cells touched by the rectangle boundary,
        # preventing a narrow obstacle from disappearing due to center sampling.
        padding_cm = 0.5 * self.resolution_cm
        rear_limit = -self.rear_overhang_cm - padding_cm
        front_limit = self.front_extent_cm + padding_cm
        half_width = 0.5 * self.vehicle_width_cm + padding_cm
        bound_cells = math.ceil(
            max(abs(rear_limit), abs(front_limit), half_width) / self.resolution_cm
        )
        caches: list[tuple[np.ndarray, np.ndarray]] = []

        for yaw_bin in range(self.yaw_bin_count):
            yaw_rad = yaw_bin * self.yaw_resolution_rad - math.pi
            cos_yaw = math.cos(yaw_rad)
            sin_yaw = math.sin(yaw_rad)
            offsets: list[tuple[int, int]] = []
            for dy in range(-bound_cells, bound_cells + 1):
                for dx in range(-bound_cells, bound_cells + 1):
                    world_x = dx * self.resolution_cm
                    world_y = dy * self.resolution_cm
                    longitudinal = world_x * cos_yaw + world_y * sin_yaw
                    lateral = -world_x * sin_yaw + world_y * cos_yaw
                    if (
                        rear_limit <= longitudinal <= front_limit
                        and abs(lateral) <= half_width
                    ):
                        offsets.append((dy, dx))
            offset_array = np.asarray(offsets, dtype=np.int32)
            caches.append((offset_array[:, 0], offset_array[:, 1]))

        return tuple(caches)

    def _densify_path(self, sparse_path: list[HybridState]) -> list[HybridState]:
        """Resample each motion primitive with the same bicycle-model integration.

        Search nodes remain spaced by ``motion_step_cm`` to keep Hybrid A* fast.
        Only the successful output is expanded, so controllers receive gradual
        position and heading changes without increasing the search state count.
        """
        if len(sparse_path) < 2:
            return sparse_path.copy()

        # Match the collision-check integration steps. Selecting points from this
        # exact trajectory avoids straight-line interpolation across curved motion.
        integration_step_cm = max(0.25, self.resolution_cm * 0.5)
        sample_count = max(1, math.ceil(self.motion_step_cm / integration_step_cm))
        dense_path = [sparse_path[0]]

        for parent, child in zip(sparse_path, sparse_path[1:]):
            signed_step = child.direction * self.motion_step_cm / sample_count
            x_cm = parent.x_cm
            y_cm = parent.y_cm
            yaw_rad = parent.yaw_rad
            next_output_distance = self.path_output_step_cm

            for sample_index in range(1, sample_count + 1):
                next_yaw = (
                    yaw_rad
                    + signed_step / self.wheelbase_cm * math.tan(child.steer_rad)
                )
                midpoint_yaw = yaw_rad + 0.5 * (next_yaw - yaw_rad)
                x_cm += signed_step * math.cos(midpoint_yaw)
                y_cm += signed_step * math.sin(midpoint_yaw)
                yaw_rad = self._normalize_yaw(next_yaw)
                traveled_cm = sample_index * abs(signed_step)
                is_endpoint = sample_index == sample_count

                if traveled_cm + 1e-9 < next_output_distance and not is_endpoint:
                    continue

                # Use the stored endpoint exactly to prevent numerical drift between
                # consecutive primitives after repeated floating-point integration.
                if is_endpoint:
                    output_state = child
                else:
                    output_state = HybridState(
                        x_cm,
                        y_cm,
                        yaw_rad,
                        child.direction,
                        child.steer_rad,
                    )
                dense_path.append(output_state)
                while next_output_distance <= traveled_cm + 1e-9:
                    next_output_distance += self.path_output_step_cm

        return dense_path

    def _yaw_bin(self, yaw_rad: float) -> int:
        return (
            round((self._normalize_yaw(yaw_rad) + math.pi) / self.yaw_resolution_rad)
            % self.yaw_bin_count
        )

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
