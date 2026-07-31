"""차동구동 차량의 고정 경로 추종을 위한 작은 nonlinear MPC core."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

import numpy as np
from scipy.optimize import Bounds, LinearConstraint, NonlinearConstraint, minimize


def normalize_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


@dataclass(frozen=True)
class VehicleState:
    x_m: float
    y_m: float
    yaw_rad: float


@dataclass(frozen=True)
class ReferencePoint:
    x_m: float
    y_m: float
    yaw_rad: float
    direction: int


@dataclass(frozen=True)
class MpcLimits:
    dt_sec: float = 0.25
    horizon_steps: int = 10
    forward_speed_mps: float = 0.06
    reverse_speed_mps: float = 0.02
    max_forward_speed_mps: float = 0.08
    max_reverse_speed_mps: float = 0.03
    max_acceleration_mps2: float = 0.12
    max_curvature_1pm: float = 7.0
    max_curvature_rate_1pmps: float = 10.0
    max_angular_speed_radps: float = 0.35
    straight_curvature_threshold_1pm: float = 0.35
    straight_max_curvature_1pm: float = 3.0
    max_tracking_yaw_error_rad: float = math.radians(25.0)
    pose_timeout_sec: float = 0.6
    goal_position_tolerance_m: float = 0.025
    goal_yaw_tolerance_rad: float = math.radians(8.0)
    gear_position_tolerance_m: float = 0.01
    nearest_forward_window: int = 140
    nearest_backward_window: int = 4
    curvature_smoothing_points: int = 5
    straight_lookahead_points: int = 4
    straight_history_points: int = 12
    straight_end_guard_points: int = 30
    solver_max_iterations: int = 45
    solver_ftol: float = 1e-5

    def validate(self) -> None:
        positive_values = (
            self.dt_sec,
            self.forward_speed_mps,
            self.reverse_speed_mps,
            self.max_forward_speed_mps,
            self.max_reverse_speed_mps,
            self.max_acceleration_mps2,
            self.max_curvature_1pm,
            self.max_curvature_rate_1pmps,
            self.max_angular_speed_radps,
            self.straight_curvature_threshold_1pm,
            self.straight_max_curvature_1pm,
            self.max_tracking_yaw_error_rad,
            self.pose_timeout_sec,
            self.goal_position_tolerance_m,
            self.goal_yaw_tolerance_rad,
            self.gear_position_tolerance_m,
            self.solver_ftol,
        )
        if any(not math.isfinite(value) or value <= 0.0 for value in positive_values):
            raise ValueError("all MPC limits must be positive and finite")
        if self.horizon_steps < 2:
            raise ValueError("MPC horizon_steps must be at least 2")
        if self.nearest_forward_window < 1 or self.nearest_backward_window < 0:
            raise ValueError("MPC nearest-point windows are invalid")
        if (
            self.curvature_smoothing_points < 1
            or self.curvature_smoothing_points % 2 == 0
        ):
            raise ValueError(
                "curvature_smoothing_points must be a positive odd integer"
            )
        if (
            self.straight_lookahead_points < 1
            or self.straight_history_points < 1
            or self.straight_end_guard_points < 0
        ):
            raise ValueError("MPC straight-section point counts are invalid")
        if self.solver_max_iterations < 1:
            raise ValueError("MPC solver_max_iterations must be positive")
        if self.forward_speed_mps > self.max_forward_speed_mps:
            raise ValueError("forward reference speed exceeds its limit")
        if self.reverse_speed_mps > self.max_reverse_speed_mps:
            raise ValueError("reverse reference speed exceeds its limit")


@dataclass(frozen=True)
class MpcWeights:
    position: float = 200.0
    yaw: float = 2.0
    terminal_position: float = 500.0
    terminal_yaw: float = 5.0
    speed: float = 50.0
    curvature: float = 0.08
    speed_rate: float = 1.0
    curvature_rate: float = 0.04

    def validate(self) -> None:
        values = (
            self.position,
            self.yaw,
            self.terminal_position,
            self.terminal_yaw,
            self.speed,
            self.curvature,
            self.speed_rate,
            self.curvature_rate,
        )
        if any(not math.isfinite(value) or value < 0.0 for value in values):
            raise ValueError("MPC weights must be finite and non-negative")


@dataclass(frozen=True)
class MpcCommand:
    linear_mps: float
    angular_radps: float
    curvature_1pm: float
    progress_index: int
    status: str
    solve_time_sec: float
    cost: float


class DifferentialDriveMpc:
    """Signed speed와 curvature를 최적화해 제자리 회전을 구조적으로 막는다."""

    def __init__(
        self,
        limits: MpcLimits | None = None,
        weights: MpcWeights | None = None,
    ) -> None:
        self.limits = limits or MpcLimits()
        self.weights = weights or MpcWeights()
        self.limits.validate()
        self.weights.validate()
        self.path: tuple[ReferencePoint, ...] = ()
        self._arc_length = np.empty(0, dtype=np.float64)
        self._reference_curvature = np.empty(0, dtype=np.float64)
        self.progress_index = 0
        self.last_speed_mps = 0.0
        self.last_curvature_1pm = 0.0
        self._warm_start: np.ndarray | None = None

    def set_path(self, points: Sequence[ReferencePoint]) -> None:
        if len(points) < 2:
            raise ValueError("MPC path needs at least two points")
        normalized: list[ReferencePoint] = []
        for point in points:
            if point.direction not in (-1, 1):
                raise ValueError("MPC path direction must be -1 or 1")
            values = (point.x_m, point.y_m, point.yaw_rad)
            if any(not math.isfinite(value) for value in values):
                raise ValueError("MPC path contains non-finite values")
            normalized.append(
                ReferencePoint(
                    float(point.x_m),
                    float(point.y_m),
                    normalize_angle(float(point.yaw_rad)),
                    int(point.direction),
                )
            )
        self.path = tuple(normalized)
        arc_length = [0.0]
        for first, second in zip(self.path, self.path[1:]):
            arc_length.append(
                arc_length[-1]
                + math.hypot(second.x_m - first.x_m, second.y_m - first.y_m)
            )
        self._arc_length = np.asarray(arc_length, dtype=np.float64)
        curvature = np.zeros(len(self.path), dtype=np.float64)
        for index in range(len(self.path) - 1):
            if self.path[index].direction != self.path[index + 1].direction:
                continue
            distance = self._arc_length[index + 1] - self._arc_length[index]
            if distance <= 1e-12:
                continue
            curvature[index] = (
                self.path[index].direction
                * normalize_angle(
                    self.path[index + 1].yaw_rad - self.path[index].yaw_rad
                )
                / distance
            )
        block_start = 0
        while block_start < len(self.path):
            block_end = block_start
            while (
                block_end + 1 < len(self.path)
                and self.path[block_end + 1].direction
                == self.path[block_start].direction
            ):
                block_end += 1
            block = curvature[block_start : block_end + 1]
            kernel = np.ones(
                self.limits.curvature_smoothing_points,
                dtype=np.float64,
            )
            numerator = np.convolve(block, kernel, mode="same")
            denominator = np.convolve(np.ones_like(block), kernel, mode="same")
            curvature[block_start : block_end + 1] = numerator / denominator
            block_start = block_end + 1
        self._reference_curvature = np.clip(
            curvature,
            -self.limits.max_curvature_1pm,
            self.limits.max_curvature_1pm,
        )
        self.progress_index = 0
        self.last_speed_mps = 0.0
        self.last_curvature_1pm = 0.0
        self._warm_start = None

    def clear_path(self) -> None:
        self.path = ()
        self._arc_length = np.empty(0, dtype=np.float64)
        self._reference_curvature = np.empty(0, dtype=np.float64)
        self.progress_index = 0
        self.last_speed_mps = 0.0
        self.last_curvature_1pm = 0.0
        self._warm_start = None

    def stop(self, status: str) -> MpcCommand:
        self.last_speed_mps = 0.0
        self.last_curvature_1pm = 0.0
        self._warm_start = None
        return MpcCommand(0.0, 0.0, 0.0, self.progress_index, status, 0.0, 0.0)

    def advance_gear_segment(self) -> bool:
        """현재 direction block 다음 점으로 이동하고 제어 상태를 초기화한다."""
        if not self.path:
            return False
        end = self._segment_end(self.progress_index)
        if end >= len(self.path) - 1:
            return False
        self.progress_index = end + 1
        self.last_speed_mps = 0.0
        self.last_curvature_1pm = 0.0
        self._warm_start = None
        return True

    def command(self, state: VehicleState) -> MpcCommand:
        if not self.path:
            return self.stop("NO_PATH")
        if any(
            not math.isfinite(value)
            for value in (state.x_m, state.y_m, state.yaw_rad)
        ):
            return self.stop("INVALID_POSE")

        self.progress_index = self._nearest_index(state)
        segment_end = self._segment_end(self.progress_index)
        endpoint = self.path[segment_end]
        endpoint_distance = math.hypot(
            endpoint.x_m - state.x_m,
            endpoint.y_m - state.y_m,
        )
        endpoint_yaw_error = abs(normalize_angle(endpoint.yaw_rad - state.yaw_rad))
        is_final_segment = segment_end == len(self.path) - 1

        if (
            endpoint_distance <= self.limits.goal_position_tolerance_m
            and endpoint_yaw_error <= self.limits.goal_yaw_tolerance_rad
            and is_final_segment
        ):
            return self.stop("GOAL_REACHED")
        if (
            endpoint_distance <= self.limits.gear_position_tolerance_m
            and not is_final_segment
        ):
            return self.stop("GEAR_CHANGE_REQUIRED")

        references = self._reference_horizon(self.progress_index, segment_end)
        direction = self.path[self.progress_index].direction
        curvature_limit = self._curvature_limit(
            references,
            self.progress_index,
            segment_end,
            direction,
        )
        tracking_yaw_error = abs(
            normalize_angle(
                self.path[self.progress_index].yaw_rad - state.yaw_rad
            )
        )
        if (
            curvature_limit < self.limits.max_curvature_1pm
            and tracking_yaw_error > self.limits.max_tracking_yaw_error_rad
        ):
            return self.stop("HEADING_ERROR_TOO_LARGE")
        self.last_curvature_1pm = float(
            np.clip(
                self.last_curvature_1pm,
                -curvature_limit,
                curvature_limit,
            )
        )
        initial = self._initial_guess(direction, references)
        bounds = self._bounds(direction, curvature_limit)
        rate_constraint = self._rate_constraint()
        angular_constraint = NonlinearConstraint(
            self._angular_speed_margin,
            0.0,
            np.inf,
        )

        import time

        started = time.perf_counter()
        result = minimize(
            self._objective,
            initial,
            args=(state, references, direction),
            method="SLSQP",
            bounds=bounds,
            constraints=(rate_constraint, angular_constraint),
            options={
                "maxiter": self.limits.solver_max_iterations,
                "ftol": self.limits.solver_ftol,
                "disp": False,
            },
        )
        solve_time = time.perf_counter() - started
        if not result.success or not np.all(np.isfinite(result.x)):
            return self.stop(f"SOLVER_FAILED: {result.message}")

        controls = np.asarray(result.x, dtype=np.float64).reshape(-1, 2)
        speed = float(controls[0, 0])
        curvature = float(controls[0, 1])
        angular = speed * curvature
        if abs(angular) > self.limits.max_angular_speed_radps + 1e-6:
            return self.stop("SOLVER_ANGULAR_LIMIT")

        self.last_speed_mps = speed
        self.last_curvature_1pm = curvature
        shifted = np.vstack((controls[1:], controls[-1]))
        self._warm_start = shifted.reshape(-1)
        return MpcCommand(
            linear_mps=speed,
            angular_radps=angular,
            curvature_1pm=curvature,
            progress_index=self.progress_index,
            status="TRACKING",
            solve_time_sec=solve_time,
            cost=float(result.fun),
        )

    def _nearest_index(self, state: VehicleState) -> int:
        # 기어가 바뀐 직후에는 cusp 양쪽 점이 거의 같은 위치에 있다.
        # 검색 범위가 이전 direction 구간까지 넘어가면 전진 마지막점을 다시
        # 선택해 후진이 영원히 시작되지 않으므로 현재 segment 안으로 제한한다.
        segment_start = self._segment_start(self.progress_index)
        start = max(
            segment_start,
            self.progress_index - self.limits.nearest_backward_window,
        )
        stop = min(
            self._segment_end(self.progress_index) + 1,
            self.progress_index + self.limits.nearest_forward_window + 1,
        )
        return min(
            range(start, stop),
            key=lambda index: (
                (self.path[index].x_m - state.x_m) ** 2
                + (self.path[index].y_m - state.y_m) ** 2
            ),
        )

    def _segment_start(self, start: int) -> int:
        direction = self.path[start].direction
        index = start
        while index > 0 and self.path[index - 1].direction == direction:
            index -= 1
        return index

    def _segment_end(self, start: int) -> int:
        direction = self.path[start].direction
        index = start
        while index + 1 < len(self.path) and self.path[index + 1].direction == direction:
            index += 1
        return index

    def _reference_horizon(
        self,
        start: int,
        segment_end: int,
    ) -> tuple[tuple[ReferencePoint, float], ...]:
        direction = self.path[start].direction
        reference_speed = (
            self.limits.forward_speed_mps
            if direction > 0
            else self.limits.reverse_speed_mps
        )
        start_s = self._arc_length[start]
        references: list[tuple[ReferencePoint, float]] = []
        for step in range(1, self.limits.horizon_steps + 1):
            target_s = min(
                start_s + reference_speed * self.limits.dt_sec * step,
                self._arc_length[segment_end],
            )
            references.append(self._interpolate_reference(target_s, start, segment_end))
        return tuple(references)

    def _interpolate_reference(
        self,
        target_s: float,
        start: int,
        segment_end: int,
    ) -> tuple[ReferencePoint, float]:
        upper = int(
            np.searchsorted(
                self._arc_length[start : segment_end + 1],
                target_s,
                side="left",
            )
        ) + start
        upper = min(max(upper, start), segment_end)
        if upper == start:
            return self.path[start], float(self._reference_curvature[start])
        lower = upper - 1
        span = self._arc_length[upper] - self._arc_length[lower]
        if span <= 1e-12:
            return self.path[upper], float(self._reference_curvature[upper])
        ratio = (target_s - self._arc_length[lower]) / span
        first, second = self.path[lower], self.path[upper]
        return (
            ReferencePoint(
                x_m=first.x_m + ratio * (second.x_m - first.x_m),
                y_m=first.y_m + ratio * (second.y_m - first.y_m),
                yaw_rad=normalize_angle(
                    first.yaw_rad
                    + ratio * normalize_angle(second.yaw_rad - first.yaw_rad)
                ),
                direction=first.direction,
            ),
            float(
                self._reference_curvature[lower]
                + ratio
                * (
                    self._reference_curvature[upper]
                    - self._reference_curvature[lower]
                )
            ),
        )

    def _initial_guess(
        self,
        direction: int,
        references: Sequence[tuple[ReferencePoint, float]],
    ) -> np.ndarray:
        if self._warm_start is not None:
            controls = self._warm_start.copy().reshape(-1, 2)
        else:
            speed = (
                self.limits.forward_speed_mps
                if direction > 0
                else -self.limits.reverse_speed_mps
            )
            controls = np.tile(
                (speed, self.last_curvature_1pm),
                (self.limits.horizon_steps, 1),
            )
        speed_delta = self.limits.max_acceleration_mps2 * self.limits.dt_sec
        curvature_delta = (
            self.limits.max_curvature_rate_1pmps * self.limits.dt_sec
        )
        previous_speed = self.last_speed_mps
        previous_curvature = self.last_curvature_1pm
        for index, (_, reference_curvature) in enumerate(references):
            controls[index, 0] = np.clip(
                controls[index, 0],
                previous_speed - speed_delta,
                previous_speed + speed_delta,
            )
            controls[index, 1] = np.clip(
                reference_curvature,
                previous_curvature - curvature_delta,
                previous_curvature + curvature_delta,
            )
            previous_speed = controls[index, 0]
            previous_curvature = controls[index, 1]
        return controls.reshape(-1)

    def _curvature_limit(
        self,
        references: Sequence[tuple[ReferencePoint, float]],
        progress_index: int,
        segment_end: int,
        direction: int,
    ) -> float:
        # 마지막 후진 주차 곡선은 기존 최대 곡률을 그대로 사용한다.
        if (
            direction < 0
            or segment_end - progress_index < self.limits.straight_end_guard_points
        ):
            return self.limits.max_curvature_1pm
        near_references = references[
            : min(self.limits.straight_lookahead_points, len(references))
        ]
        recent_start = max(
            0,
            progress_index - self.limits.straight_history_points,
        )
        recent_curvature = self._reference_curvature[
            recent_start : progress_index + 1
        ]
        if (
            near_references
            and all(
            abs(reference_curvature)
            <= self.limits.straight_curvature_threshold_1pm
            for _, reference_curvature in near_references
            )
            and np.all(
                np.abs(recent_curvature)
                <= self.limits.straight_curvature_threshold_1pm
            )
        ):
            return min(
                self.limits.straight_max_curvature_1pm,
                self.limits.max_curvature_1pm,
            )
        return self.limits.max_curvature_1pm

    def _bounds(self, direction: int, curvature_limit: float) -> Bounds:
        if direction > 0:
            speed_lower, speed_upper = 0.0, self.limits.max_forward_speed_mps
        else:
            speed_lower, speed_upper = -self.limits.max_reverse_speed_mps, 0.0
        lower = np.tile(
            (speed_lower, -curvature_limit),
            self.limits.horizon_steps,
        )
        upper = np.tile(
            (speed_upper, curvature_limit),
            self.limits.horizon_steps,
        )
        return Bounds(lower, upper)

    def _rate_constraint(self) -> LinearConstraint:
        horizon = self.limits.horizon_steps
        matrix = np.zeros((2 * horizon, 2 * horizon), dtype=np.float64)
        lower = np.empty(2 * horizon, dtype=np.float64)
        upper = np.empty(2 * horizon, dtype=np.float64)
        speed_delta = self.limits.max_acceleration_mps2 * self.limits.dt_sec
        curvature_delta = (
            self.limits.max_curvature_rate_1pmps * self.limits.dt_sec
        )
        for step in range(horizon):
            speed_row = 2 * step
            curvature_row = speed_row + 1
            matrix[speed_row, 2 * step] = 1.0
            matrix[curvature_row, 2 * step + 1] = 1.0
            if step == 0:
                lower[speed_row] = self.last_speed_mps - speed_delta
                upper[speed_row] = self.last_speed_mps + speed_delta
                lower[curvature_row] = self.last_curvature_1pm - curvature_delta
                upper[curvature_row] = self.last_curvature_1pm + curvature_delta
            else:
                matrix[speed_row, 2 * (step - 1)] = -1.0
                matrix[curvature_row, 2 * (step - 1) + 1] = -1.0
                lower[speed_row], upper[speed_row] = -speed_delta, speed_delta
                lower[curvature_row], upper[curvature_row] = (
                    -curvature_delta,
                    curvature_delta,
                )
        return LinearConstraint(matrix, lower, upper)

    def _angular_speed_margin(self, flat_controls: np.ndarray) -> np.ndarray:
        controls = np.asarray(flat_controls).reshape(-1, 2)
        angular = controls[:, 0] * controls[:, 1]
        return self.limits.max_angular_speed_radps**2 - angular**2

    def _objective(
        self,
        flat_controls: np.ndarray,
        state: VehicleState,
        references: Sequence[tuple[ReferencePoint, float]],
        direction: int,
    ) -> float:
        controls = np.asarray(flat_controls, dtype=np.float64).reshape(-1, 2)
        x, y, yaw = state.x_m, state.y_m, state.yaw_rad
        reference_speed = (
            self.limits.forward_speed_mps
            if direction > 0
            else -self.limits.reverse_speed_mps
        )
        cost = 0.0
        previous_speed = self.last_speed_mps
        previous_curvature = self.last_curvature_1pm
        for index, ((speed, curvature), reference_data) in enumerate(
            zip(controls, references)
        ):
            reference, reference_curvature = reference_data
            angular = speed * curvature
            x += self.limits.dt_sec * speed * math.cos(yaw)
            y += self.limits.dt_sec * speed * math.sin(yaw)
            yaw = normalize_angle(yaw + self.limits.dt_sec * angular)
            position_error = (x - reference.x_m) ** 2 + (y - reference.y_m) ** 2
            yaw_error = normalize_angle(yaw - reference.yaw_rad)
            terminal = index == len(references) - 1
            cost += (
                (self.weights.terminal_position if terminal else self.weights.position)
                * position_error
                + (self.weights.terminal_yaw if terminal else self.weights.yaw)
                * yaw_error**2
                + self.weights.speed * (speed - reference_speed) ** 2
                + self.weights.curvature
                * (curvature - reference_curvature) ** 2
                + self.weights.speed_rate * (speed - previous_speed) ** 2
                + self.weights.curvature_rate
                * (curvature - previous_curvature) ** 2
            )
            previous_speed = speed
            previous_curvature = curvature
        return float(cost)
