"""Heading-aware Hybrid A* planner using a kinematic bicycle model."""

from dataclasses import dataclass
import heapq
import itertools
import math
import time
from typing import Iterable, Protocol

import numpy as np

if __package__:
    from .path_smoothing import (
        PathSmoothingStats,
        calculate_path_smoothing_metrics,
        smooth_hybrid_path,
    )
    from .reeds_shepp import ReedsSheppPath, ReedsSheppPlanner
else:
    from path_smoothing import (
        PathSmoothingStats,
        calculate_path_smoothing_metrics,
        smooth_hybrid_path,
    )
    from reeds_shepp import ReedsSheppPath, ReedsSheppPlanner


class PathPose(Protocol):
    """Footprint 검사에 필요한 최소 경로 pose 인터페이스."""

    x_cm: float
    y_cm: float
    yaw_rad: float


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
    smoothing_stats: PathSmoothingStats | None = None


class HybridAStarPlanner:
    """Search over position, heading, steering, and forward/reverse motion."""

    def __init__(
        self,
        grid: np.ndarray,
        resolution_cm: float = 1.0,
        wheelbase_cm: float = 8.0,
        vehicle_length_cm: float = 12.0,
        vehicle_width_cm: float = 8.0,
        rear_overhang_cm: float | None = None,
        minimum_turning_radius_cm: float | None = None,
        motion_step_cm: float = 3.0,
        path_output_step_cm: float = 0.5,
        yaw_resolution_deg: float = 10.0,
        steer_set_deg: tuple[float, ...] = (
            -30.0,
            -20.0,
            -10.0,
            0.0,
            10.0,
            20.0,
            30.0,
        ),
        max_steer_change_deg: float = 10.0,
        allow_reverse: bool = True,
        obstacle_threshold: int = 50,
        timeout_sec: float = 5.0,
        goal_tolerance_cm: float = 4.0,
        goal_yaw_tolerance_deg: float = 15.0,
        reverse_penalty: float = 1.5,
        gear_switch_penalty: float = 5.0,
        steer_penalty: float = 0.2,
        steer_change_penalty: float = 0.5,
        analytic_expansion_enabled: bool = True,
        analytic_expansion_distance_cm: float = 30.0,
        analytic_turning_radius_margin_cm: float = 4.0,
        path_smoothing_enabled: bool = True,
        smoothing_knot_spacing_cm: float = 3.0,
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
        if minimum_turning_radius_cm is not None and minimum_turning_radius_cm <= 0:
            raise ValueError("minimum turning radius must be positive")
        if yaw_resolution_deg <= 0 or max_steer_change_deg <= 0 or timeout_sec <= 0:
            raise ValueError(
                "yaw resolution, maximum steering change, and timeout must be positive"
            )
        if not steer_set_deg:
            raise ValueError("steer_set_deg must contain at least one steering angle")
        if analytic_expansion_distance_cm <= 0:
            raise ValueError("analytic expansion distance must be positive")
        if analytic_turning_radius_margin_cm < 0:
            raise ValueError("analytic turning radius margin must not be negative")
        if smoothing_knot_spacing_cm <= 0:
            raise ValueError("smoothing knot spacing must be positive")

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
        self.max_steer_change_rad = math.radians(max_steer_change_deg)
        self.allow_reverse = allow_reverse
        self.obstacle_threshold = obstacle_threshold
        self.timeout_sec = timeout_sec
        self.goal_tolerance_cm = goal_tolerance_cm
        self.goal_yaw_tolerance_rad = math.radians(goal_yaw_tolerance_deg)
        self.reverse_penalty = reverse_penalty
        self.gear_switch_penalty = gear_switch_penalty
        self.steer_penalty = steer_penalty
        self.steer_change_penalty = steer_change_penalty
        self.analytic_expansion_enabled = analytic_expansion_enabled
        self.analytic_expansion_distance_cm = analytic_expansion_distance_cm
        self.analytic_turning_radius_margin_cm = analytic_turning_radius_margin_cm
        self.path_smoothing_enabled = path_smoothing_enabled
        self.smoothing_knot_spacing_cm = smoothing_knot_spacing_cm
        self.max_abs_steer_rad = max(abs(value) for value in self.steer_set_rad)
        if self.analytic_expansion_enabled and self.max_abs_steer_rad <= 1e-12:
            raise ValueError(
                "analytic expansion requires at least one non-zero steering angle"
            )
        kinematic_turning_radius_cm = (
            self.wheelbase_cm / math.tan(self.max_abs_steer_rad)
            if self.max_abs_steer_rad > 1e-12
            else math.inf
        )
        self.minimum_turning_radius_cm = max(
            kinematic_turning_radius_cm,
            minimum_turning_radius_cm or 0.0,
        )
        self.analytic_turning_radius_cm = (
            self.minimum_turning_radius_cm
            + self.analytic_turning_radius_margin_cm
        )
        self.analytic_steer_rad = math.atan(
            self.wheelbase_cm / self.analytic_turning_radius_cm
        )
        self.reeds_shepp_planner = (
            ReedsSheppPlanner(
                self.analytic_turning_radius_cm,
                step_size_cm=self.path_output_step_cm,
            )
            if self.analytic_expansion_enabled
            else None
        )
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

            analytic_path = self.try_analytic_expansion(current, goal)
            if analytic_path is not None:
                sparse_path = self._reconstruct(states, parents, current_key)
                raw_hybrid_path = self._join_analytic_path(sparse_path, analytic_path)
                hybrid_path, smoothing_stats = self.smooth_path_with_fallback(
                    raw_hybrid_path
                )
                return HybridAStarResult(
                    hybrid_path,
                    g_score[current_key]
                    + self._analytic_path_cost(current, analytic_path),
                    True,
                    expanded_nodes,
                    "goal reached by Reeds-Shepp analytic expansion; "
                    + smoothing_stats.status,
                    smoothing_stats,
                )

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
                # A hard steering-rate constraint prevents an instantaneous jump
                # such as -30 to +30 degrees at a primitive boundary. This is
                # separate from the soft steer-change cost below.
                if (
                    abs(steer_rad - state.steer_rad)
                    > self.max_steer_change_rad + 1e-12
                ):
                    continue
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

    def first_path_collision_index(
        self,
        poses: Iterable[PathPose],
    ) -> int | None:
        """Return the first footprint-colliding pose index, or None when safe.

        Reeds-Shepp paths are sampled every 0.5 cm before this check. Reusing
        this planner's footprint model keeps vehicle dimensions, map bounds,
        heading discretization, and obstacle thresholds identical to Hybrid A*.
        """
        for index, pose in enumerate(poses):
            if self.is_pose_collision(pose.x_cm, pose.y_cm, pose.yaw_rad):
                return index
        return None

    def is_path_collision_free(self, poses: Iterable[PathPose]) -> bool:
        """Return whether every sampled pose has a valid vehicle footprint."""
        return self.first_path_collision_index(poses) is None

    def try_analytic_expansion(
        self,
        current: HybridState,
        goal: tuple[float, float, float],
    ) -> ReedsSheppPath | None:
        """Return the shortest collision-free Reeds-Shepp goal connection.

        Candidates are attempted only near the goal. A rejected candidate has
        no effect on the open set, so normal Hybrid A* expansion remains the
        fallback whenever every analytic path intersects an obstacle.
        """
        if self.reeds_shepp_planner is None:
            return None
        if (
            math.hypot(goal[0] - current.x_cm, goal[1] - current.y_cm)
            > self.analytic_expansion_distance_cm
        ):
            return None

        start = (current.x_cm, current.y_cm, current.yaw_rad)
        normalized_goal = (goal[0], goal[1], self._normalize_yaw(goal[2]))
        for path in self.reeds_shepp_planner.plan_candidates(start, normalized_goal):
            if not self.allow_reverse and any(
                segment.direction < 0 for segment in path.segments
            ):
                continue
            if self.is_path_collision_free(path.poses):
                return path
        return None

    def _join_analytic_path(
        self,
        sparse_path: list[HybridState],
        analytic_path: ReedsSheppPath,
    ) -> list[HybridState]:
        """Densify the searched prefix and append continuous analytic poses."""
        path = self._densify_path(sparse_path)
        for pose in analytic_path.poses[1:]:
            if pose.segment_mode == "L":
                steer_rad = self.analytic_steer_rad
            elif pose.segment_mode == "R":
                steer_rad = -self.analytic_steer_rad
            else:
                steer_rad = 0.0
            path.append(
                HybridState(
                    pose.x_cm,
                    pose.y_cm,
                    pose.yaw_rad,
                    pose.direction,
                    steer_rad,
                )
            )
        return path

    def _analytic_path_cost(
        self,
        current: HybridState,
        path: ReedsSheppPath,
    ) -> float:
        """Apply Hybrid A* reverse, gear, and steering costs to an analytic path."""
        cost = 0.0
        previous_direction = current.direction
        previous_steer = current.steer_rad
        max_steer = self.max_abs_steer_rad or 1.0

        for segment in path.segments:
            distance_cm = abs(segment.length_cm)
            direction = segment.direction
            if segment.mode == "L":
                steer_rad = self.analytic_steer_rad
            elif segment.mode == "R":
                steer_rad = -self.analytic_steer_rad
            else:
                steer_rad = 0.0

            cost += distance_cm * (self.reverse_penalty if direction < 0 else 1.0)
            if direction != previous_direction:
                cost += self.gear_switch_penalty
            primitive_count = distance_cm / self.motion_step_cm
            cost += primitive_count * self.steer_penalty * abs(steer_rad) / max_steer
            cost += (
                self.steer_change_penalty
                * abs(steer_rad - previous_steer)
                / max_steer
            )
            previous_direction = direction
            previous_steer = steer_rad
        return cost

    def smooth_path_with_fallback(
        self,
        raw_path: list[HybridState],
    ) -> tuple[list[HybridState], PathSmoothingStats]:
        """Use smoothing only when every kinematic and collision check passes."""
        if not self.path_smoothing_enabled or len(raw_path) < 3:
            return raw_path, self._make_smoothing_stats(
                raw_path,
                raw_path,
                attempted=False,
                accepted=False,
                status="raw path retained (smoothing disabled)",
            )
        try:
            smoothed_poses = smooth_hybrid_path(
                raw_path,
                wheelbase_cm=self.wheelbase_cm,
                output_step_cm=self.path_output_step_cm,
                knot_spacing_cm=self.smoothing_knot_spacing_cm,
            )
        except ValueError as error:
            return raw_path, self._make_smoothing_stats(
                raw_path,
                raw_path,
                attempted=True,
                accepted=False,
                status=f"raw path retained (smoothing error: {error})",
            )

        smoothed_path = [
            HybridState(
                pose.x_cm,
                pose.y_cm,
                pose.yaw_rad,
                pose.direction,
                pose.steer_rad,
            )
            for pose in smoothed_poses
        ]
        if not smoothed_path:
            return raw_path, self._make_smoothing_stats(
                raw_path,
                raw_path,
                attempted=True,
                accepted=False,
                status="raw path retained (empty smoothing result)",
            )

        start_error = math.hypot(
            smoothed_path[0].x_cm - raw_path[0].x_cm,
            smoothed_path[0].y_cm - raw_path[0].y_cm,
        )
        goal_error = math.hypot(
            smoothed_path[-1].x_cm - raw_path[-1].x_cm,
            smoothed_path[-1].y_cm - raw_path[-1].y_cm,
        )
        start_yaw_error = abs(
            self._angle_difference(smoothed_path[0].yaw_rad, raw_path[0].yaw_rad)
        )
        goal_yaw_error = abs(
            self._angle_difference(smoothed_path[-1].yaw_rad, raw_path[-1].yaw_rad)
        )
        if max(start_error, goal_error, start_yaw_error, goal_yaw_error) > 1e-6:
            return raw_path, self._make_smoothing_stats(
                raw_path,
                raw_path,
                attempted=True,
                accepted=False,
                status="raw path retained (endpoint changed by smoothing)",
                candidate_path=smoothed_path,
            )

        if any(
            abs(state.steer_rad) > self.max_abs_steer_rad + 1e-6
            for state in smoothed_path
        ):
            return raw_path, self._make_smoothing_stats(
                raw_path,
                raw_path,
                attempted=True,
                accepted=False,
                status="raw path retained (steering limit exceeded)",
                candidate_path=smoothed_path,
            )

        for first, second in zip(smoothed_path, smoothed_path[1:]):
            distance_cm = math.hypot(
                second.x_cm - first.x_cm,
                second.y_cm - first.y_cm,
            )
            if distance_cm > self.path_output_step_cm + 1e-6:
                return raw_path, self._make_smoothing_stats(
                    raw_path,
                    raw_path,
                    attempted=True,
                    accepted=False,
                    status="raw path retained (output spacing exceeded)",
                    candidate_path=smoothed_path,
                )
            if first.direction != second.direction:
                continue
            allowed_change = (
                self.max_steer_change_rad
                * distance_cm
                / self.motion_step_cm
                + math.radians(0.1)
            )
            if abs(second.steer_rad - first.steer_rad) > allowed_change:
                return raw_path, self._make_smoothing_stats(
                    raw_path,
                    raw_path,
                    attempted=True,
                    accepted=False,
                    status="raw path retained (steering rate exceeded)",
                    candidate_path=smoothed_path,
                )

        collision_index = self.first_path_collision_index(smoothed_path)
        if collision_index is not None:
            return (
                raw_path,
                self._make_smoothing_stats(
                    raw_path,
                    raw_path,
                    attempted=True,
                    accepted=False,
                    status=(
                        "raw path retained "
                        f"(smoothed collision at pose {collision_index})"
                    ),
                    candidate_path=smoothed_path,
                ),
            )
        return smoothed_path, self._make_smoothing_stats(
            raw_path,
            smoothed_path,
            attempted=True,
            accepted=True,
            status="curvature-smoothed path accepted",
            candidate_path=smoothed_path,
        )

    def _make_smoothing_stats(
        self,
        raw_path: list[HybridState],
        final_path: list[HybridState],
        attempted: bool,
        accepted: bool,
        status: str,
        candidate_path: list[HybridState] | None = None,
    ) -> PathSmoothingStats:
        return PathSmoothingStats(
            attempted=attempted,
            accepted=accepted,
            status=status,
            knot_spacing_cm=self.smoothing_knot_spacing_cm,
            turning_radius_cm=self.analytic_turning_radius_cm,
            raw=calculate_path_smoothing_metrics(raw_path),
            candidate=(
                calculate_path_smoothing_metrics(candidate_path)
                if candidate_path is not None
                else None
            ),
            final=calculate_path_smoothing_metrics(final_path),
        )

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
