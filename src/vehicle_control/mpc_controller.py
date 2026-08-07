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
    # Pose와 reference path가 나타내는 제어점(차량 중심)이 차동구동
    # 회전 중심(rear axle)보다 앞에 있는 거리다.
    control_point_offset_m: float = 0.04
    # PinkyPro differential-drive physical geometry.
    wheel_radius_m: float = 0.027
    wheel_separation_m: float = 0.0961
    max_wheel_angular_speed_radps: float = 0.105 / 0.027
    dt_sec: float = 0.25
    horizon_steps: int = 10
    forward_speed_mps: float = 0.06
    reverse_speed_mps: float = 0.02
    max_forward_speed_mps: float = 0.08
    max_reverse_speed_mps: float = 0.03
    max_acceleration_mps2: float = 0.12
    max_curvature_1pm: float = 1.0 / 0.12
    max_curvature_rate_1pmps: float = 10.0
    max_angular_speed_radps: float = 0.40
    straight_curvature_threshold_1pm: float = 0.35
    straight_max_curvature_1pm: float = 3.0
    cross_track_feedback_gain_1pm2: float = 15.0
    heading_feedback_gain_1pmprad: float = 4.0
    heading_feedback_deadband_rad: float = math.radians(2.0)
    cross_track_deadband_m: float = 0.003
    reverse_cross_track_deadband_m: float = 0.006
    reverse_heading_feedback_deadband_rad: float = math.radians(2.0)
    reverse_cross_track_gain_scale: float = 1.20
    reverse_heading_gain_scale: float = 1.15
    cross_track_slowdown_start_m: float = 0.01
    cross_track_slowdown_full_m: float = 0.04
    minimum_tracking_speed_scale: float = 0.35
    max_tracking_yaw_error_rad: float = math.radians(25.0)
    heading_recovery_full_curvature_error_rad: float = math.radians(45.0)
    heading_recovery_speed_scale: float = 0.60
    pose_timeout_sec: float = 0.6
    goal_position_tolerance_m: float = 0.03
    goal_yaw_tolerance_rad: float = math.radians(8.0)
    gear_position_tolerance_m: float = 0.01
    gear_fallback_position_tolerance_m: float = 0.04
    gear_stall_speed_threshold_mps: float = 0.003
    gear_fallback_max_segment_length_m: float = 0.15
    gear_passed_endpoint_lateral_tolerance_m: float = 0.06
    gear_transition_end_guard_points: int = 20
    nearest_forward_window: int = 140
    nearest_backward_window: int = 4
    steering_preview_points: int = 6
    steering_preview_weight: float = 0.30
    steering_rejoin_preview_points: int = 12
    steering_rejoin_full_error_m: float = 0.03
    steering_rejoin_preview_weight: float = 0.65
    curve_feedforward_preview_points: int = 14
    curve_feedforward_gain: float = 0.55
    curve_feedforward_deadband_1pm: float = 0.5
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
            self.wheel_radius_m,
            self.wheel_separation_m,
            self.max_wheel_angular_speed_radps,
            self.max_curvature_1pm,
            self.max_curvature_rate_1pmps,
            self.max_angular_speed_radps,
            self.straight_curvature_threshold_1pm,
            self.straight_max_curvature_1pm,
            self.cross_track_deadband_m,
            self.reverse_cross_track_deadband_m,
            self.reverse_cross_track_gain_scale,
            self.reverse_heading_gain_scale,
            self.cross_track_slowdown_start_m,
            self.cross_track_slowdown_full_m,
            self.minimum_tracking_speed_scale,
            self.max_tracking_yaw_error_rad,
            self.heading_recovery_full_curvature_error_rad,
            self.heading_recovery_speed_scale,
            self.steering_rejoin_full_error_m,
            self.pose_timeout_sec,
            self.goal_position_tolerance_m,
            self.goal_yaw_tolerance_rad,
            self.gear_position_tolerance_m,
            self.gear_fallback_position_tolerance_m,
            self.gear_stall_speed_threshold_mps,
            self.gear_fallback_max_segment_length_m,
            self.gear_passed_endpoint_lateral_tolerance_m,
            self.solver_ftol,
        )
        if any(not math.isfinite(value) or value <= 0.0 for value in positive_values):
            raise ValueError("all MPC limits must be positive and finite")
        if (
            not math.isfinite(self.cross_track_feedback_gain_1pm2)
            or self.cross_track_feedback_gain_1pm2 < 0.0
        ):
            raise ValueError("cross-track feedback gain must be finite and non-negative")
        if (
            not math.isfinite(self.heading_feedback_gain_1pmprad)
            or self.heading_feedback_gain_1pmprad < 0.0
        ):
            raise ValueError("heading feedback gain must be finite and non-negative")
        if (
            not math.isfinite(self.heading_feedback_deadband_rad)
            or self.heading_feedback_deadband_rad < 0.0
        ):
            raise ValueError("heading feedback deadband must be finite and non-negative")
        if (
            not math.isfinite(self.reverse_heading_feedback_deadband_rad)
            or self.reverse_heading_feedback_deadband_rad < 0.0
        ):
            raise ValueError(
                "reverse heading feedback deadband must be finite and non-negative"
            )
        if (
            not math.isfinite(self.control_point_offset_m)
            or self.control_point_offset_m < 0.0
        ):
            raise ValueError("control point offset must be finite and non-negative")
        if self.cross_track_slowdown_full_m <= self.cross_track_slowdown_start_m:
            raise ValueError("cross-track slowdown range is invalid")
        if self.minimum_tracking_speed_scale > 1.0:
            raise ValueError("minimum tracking speed scale must not exceed one")
        if self.heading_recovery_speed_scale > 1.0:
            raise ValueError("heading recovery speed scale must not exceed one")
        if self.horizon_steps < 2:
            raise ValueError("MPC horizon_steps must be at least 2")
        if self.nearest_forward_window < 1 or self.nearest_backward_window < 0:
            raise ValueError("MPC nearest-point windows are invalid")
        if self.steering_preview_points < 1:
            raise ValueError("steering preview points must be positive")
        if self.steering_rejoin_preview_points < self.steering_preview_points:
            raise ValueError(
                "rejoin preview points must not be below normal preview points"
            )
        if (
            not math.isfinite(self.steering_preview_weight)
            or not 0.0 <= self.steering_preview_weight <= 1.0
        ):
            raise ValueError("steering preview weight must be between zero and one")
        if (
            not math.isfinite(self.steering_rejoin_preview_weight)
            or not self.steering_preview_weight
            <= self.steering_rejoin_preview_weight
            <= 1.0
        ):
            raise ValueError(
                "rejoin preview weight must be between normal weight and one"
            )
        if self.steering_rejoin_full_error_m <= self.cross_track_deadband_m:
            raise ValueError(
                "rejoin full error must exceed cross-track deadband"
            )
        if self.curve_feedforward_preview_points < 1:
            raise ValueError("curve feed-forward preview points must be positive")
        if (
            not math.isfinite(self.curve_feedforward_gain)
            or not 0.0 <= self.curve_feedforward_gain <= 1.0
        ):
            raise ValueError("curve feed-forward gain must be between zero and one")
        if (
            not math.isfinite(self.curve_feedforward_deadband_1pm)
            or self.curve_feedforward_deadband_1pm < 0.0
        ):
            raise ValueError(
                "curve feed-forward deadband must be finite and non-negative"
            )
        if (
            self.heading_recovery_full_curvature_error_rad
            < self.max_tracking_yaw_error_rad
        ):
            raise ValueError(
                "full-curvature heading error must not be below recovery threshold"
            )
        if self.gear_transition_end_guard_points < 1:
            raise ValueError("gear transition end guard must be positive")
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
        if (
            self.gear_fallback_position_tolerance_m
            < self.gear_position_tolerance_m
        ):
            raise ValueError(
                "gear fallback tolerance must not be smaller than primary tolerance"
            )


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
    """차량 중심 pose에서 signed speed와 curvature를 최적화한다."""

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
        self._progress_initialized = False
        self.last_speed_mps = 0.0
        self.last_curvature_1pm = 0.0
        self._warm_start: np.ndarray | None = None

    def propagate_state(
        self,
        state: VehicleState,
        linear_mps: float,
        angular_radps: float,
        dt_sec: float | None = None,
    ) -> VehicleState:
        """Rear-axle twist로 차량 중심 pose를 한 step 전진시킨다."""
        dt = self.limits.dt_sec if dt_sec is None else float(dt_sec)
        offset = self.limits.control_point_offset_m
        rear_x = state.x_m - offset * math.cos(state.yaw_rad)
        rear_y = state.y_m - offset * math.sin(state.yaw_rad)
        next_yaw = normalize_angle(state.yaw_rad + dt * angular_radps)
        return VehicleState(
            x_m=rear_x
            + dt * linear_mps * math.cos(state.yaw_rad)
            + offset * math.cos(next_yaw),
            y_m=rear_y
            + dt * linear_mps * math.sin(state.yaw_rad)
            + offset * math.sin(next_yaw),
            yaw_rad=next_yaw,
        )

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
        # 입력 경로는 차량 중심 궤적이지만 cmd_vel curvature는 차동구동
        # 회전 중심의 이동거리 ds_rear에 대해 d(yaw)/ds_rear로 정의된다.
        # 중심 궤적의 arc length를 그대로 쓰면 특히 후진 곡률이 틀어진다.
        offset = self.limits.control_point_offset_m
        rear_points = [
            (
                point.x_m - offset * math.cos(point.yaw_rad),
                point.y_m - offset * math.sin(point.yaw_rad),
            )
            for point in self.path
        ]
        curvature = np.zeros(len(self.path), dtype=np.float64)
        for index in range(len(self.path) - 1):
            if self.path[index].direction != self.path[index + 1].direction:
                continue
            distance = math.hypot(
                rear_points[index + 1][0] - rear_points[index][0],
                rear_points[index + 1][1] - rear_points[index][1],
            )
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
        self._reference_curvature[
            np.abs(self._reference_curvature)
            < self.limits.curve_feedforward_deadband_1pm
        ] = 0.0
        self.progress_index = 0
        self._progress_initialized = False
        self.last_speed_mps = 0.0
        self.last_curvature_1pm = 0.0
        self._warm_start = None

    def clear_path(self) -> None:
        self.path = ()
        self._arc_length = np.empty(0, dtype=np.float64)
        self._reference_curvature = np.empty(0, dtype=np.float64)
        self.progress_index = 0
        self._progress_initialized = False
        self.last_speed_mps = 0.0
        self.last_curvature_1pm = 0.0
        self._warm_start = None

    def restore_progress(self, progress_index: int) -> None:
        """동일 경로 재설정 뒤 저장한 진행도를 복원한다."""
        if not self.path:
            raise ValueError("Cannot restore progress without an MPC path")
        if not 0 <= progress_index < len(self.path):
            raise ValueError("MPC progress index is outside the path")
        self.progress_index = progress_index
        self._progress_initialized = True

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

        if not self._progress_initialized:
            # MPC를 경로 중간에서 재시작해도 제한된 forward window의 끝점을
            # 쫓지 않도록 첫 제어에서 현재 direction 구간 전체에 정렬한다.
            # 이후에는 progress가 경로를 건너뛰지 않도록 기존 window를 사용한다.
            self.progress_index = self._initial_nearest_index(state)
            self._progress_initialized = True
        else:
            self.progress_index = self._nearest_index(state)
        segment_end = self._segment_end(self.progress_index)
        endpoint = self.path[segment_end]
        endpoint_distance = math.hypot(
            endpoint.x_m - state.x_m,
            endpoint.y_m - state.y_m,
        )
        endpoint_yaw_error = abs(normalize_angle(endpoint.yaw_rad - state.yaw_rad))
        is_final_segment = segment_end == len(self.path) - 1
        near_segment_end = (
            segment_end - self.progress_index
            <= self.limits.gear_transition_end_guard_points
        )
        direction = self.path[self.progress_index].direction
        endpoint_heading = self._segment_tangent_yaw(segment_end)
        endpoint_error_x = state.x_m - endpoint.x_m
        endpoint_error_y = state.y_m - endpoint.y_m
        passed_endpoint_distance = direction * (
            endpoint_error_x * math.cos(endpoint_heading)
            + endpoint_error_y * math.sin(endpoint_heading)
        )
        endpoint_lateral_error = abs(
            -math.sin(endpoint_heading) * endpoint_error_x
            + math.cos(endpoint_heading) * endpoint_error_y
        )

        if (
            endpoint_distance <= self.limits.goal_position_tolerance_m
            and endpoint_yaw_error <= self.limits.goal_yaw_tolerance_rad
            and is_final_segment
        ):
            return self.stop("GOAL_REACHED")
        if (
            endpoint_distance <= self.limits.gear_position_tolerance_m
            and not is_final_segment
            and near_segment_end
        ):
            return self.stop("GEAR_CHANGE_REQUIRED")
        if (
            not is_final_segment
            and near_segment_end
            and passed_endpoint_distance >= 0.0
        ):
            if (
                endpoint_lateral_error
                <= self.limits.gear_passed_endpoint_lateral_tolerance_m
            ):
                # 전환점을 한 번 지나가면 원형 거리 오차는 다시 커진다.
                # 이때 기존 로직으로는 현재 기어 명령이 계속 유지되므로,
                # endpoint 접선 평면 통과를 별도로 검출해 다음 기어로 넘긴다.
                return self.stop("GEAR_CHANGE_REQUIRED")
            # 전환점을 크게 벗어나 통과했으면 잘못된 자세로 다음 기어를
            # 시작하지 않고 정지한다. 특히 첫 후진의 무한 후진을 막는다.
            return self.stop("GEAR_ENDPOINT_PASSED_OFF_PATH")

        references = self._reference_horizon(self.progress_index, segment_end)
        reference_yaw = self._segment_tangent_yaw(self.progress_index)
        curvature_limit = self._curvature_limit(
            references,
            self.progress_index,
            segment_end,
            direction,
        )
        signed_tracking_yaw_error = normalize_angle(
            reference_yaw - state.yaw_rad
        )
        heading_recovery = (
            abs(signed_tracking_yaw_error)
            > self.limits.max_tracking_yaw_error_rad
        )
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
        anticipated_curvature = self._anticipated_curvature(
            self.progress_index,
            segment_end,
        )
        # 최적화 horizon을 늘려 계산 주기를 늦추지 않고, 가까워지는 곡선의
        # reference curvature의 미래 증가분을 현재 명령에 점진적으로 더한다.
        # 직선에서는 현재·미래가 모두 0이므로 불필요한 조향이 생기지 않고,
        # 횡오차 복귀를 위해 MPC가 만든 조향도 약화시키지 않는다.
        curvature += self.limits.curve_feedforward_gain * (
            anticipated_curvature
            - float(self._reference_curvature[self.progress_index])
        )
        nearest = self.path[self.progress_index]
        nearest_yaw = reference_yaw
        offset = self.limits.control_point_offset_m
        state_rear_x = state.x_m - offset * math.cos(state.yaw_rad)
        state_rear_y = state.y_m - offset * math.sin(state.yaw_rad)
        nearest_rear_x = nearest.x_m - offset * math.cos(nearest_yaw)
        nearest_rear_y = nearest.y_m - offset * math.sin(nearest_yaw)
        error_x = state_rear_x - nearest_rear_x
        error_y = state_rear_y - nearest_rear_y
        current_cross_track_error = (
            -math.sin(nearest_yaw) * error_x
            + math.cos(nearest_yaw) * error_y
        )
        active_cross_track_deadband = (
            self.limits.reverse_cross_track_deadband_m
            if direction < 0
            else self.limits.cross_track_deadband_m
        )

        # 정상 추종에서는 가까운 경로만 보지만, 이탈량이 커질수록 더 앞쪽
        # 경로 접선을 향하도록 preview 거리와 비중을 연속적으로 늘린다.
        # 따라서 가장 가까운 한 점으로 급히 꺾지 않고 진행 방향을 유지한
        # 채 경로에 비스듬히 합류한다.
        rejoin_ratio = float(
            np.clip(
                (
                    abs(current_cross_track_error)
                    - active_cross_track_deadband
                )
                / (
                    self.limits.steering_rejoin_full_error_m
                    - active_cross_track_deadband
                ),
                0.0,
                1.0,
            )
        )
        dynamic_preview_points = int(
            round(
                self.limits.steering_preview_points
                + rejoin_ratio
                * (
                    self.limits.steering_rejoin_preview_points
                    - self.limits.steering_preview_points
                )
            )
        )
        preview_end = min(
            self.progress_index + dynamic_preview_points,
            segment_end,
        )

        # 현재 차량을 미래 경로점에 직접 비교하면 아직 주행하지 않은 종방향
        # 거리까지 횡오차로 섞인다. 대신 첫 MPC 곡률을 유지했을 때 차량이
        # preview 거리 뒤에 도달할 위치를 예측하고 같은 시점의 경로와 비교한다.
        # 따라서 직선 복귀는 과격하지 않고, 코너는 진입 전에 미리 반응한다.
        preview = self.path[preview_end]
        preview_yaw = self._segment_tangent_yaw(preview_end)
        preview_distance = 0.0
        previous_rear_x = nearest_rear_x
        previous_rear_y = nearest_rear_y
        for preview_index in range(self.progress_index + 1, preview_end + 1):
            point = self.path[preview_index]
            point_rear_x = point.x_m - offset * math.cos(point.yaw_rad)
            point_rear_y = point.y_m - offset * math.sin(point.yaw_rad)
            preview_distance += math.hypot(
                point_rear_x - previous_rear_x,
                point_rear_y - previous_rear_y,
            )
            previous_rear_x = point_rear_x
            previous_rear_y = point_rear_y
        signed_preview_distance = direction * preview_distance
        predicted_yaw = normalize_angle(
            state.yaw_rad + signed_preview_distance * curvature
        )
        if abs(curvature) <= 1e-9:
            predicted_rear_x = (
                state_rear_x
                + signed_preview_distance * math.cos(state.yaw_rad)
            )
            predicted_rear_y = (
                state_rear_y
                + signed_preview_distance * math.sin(state.yaw_rad)
            )
        else:
            predicted_rear_x = state_rear_x + (
                math.sin(predicted_yaw) - math.sin(state.yaw_rad)
            ) / curvature
            predicted_rear_y = state_rear_y - (
                math.cos(predicted_yaw) - math.cos(state.yaw_rad)
            ) / curvature
        preview_rear_x = preview.x_m - offset * math.cos(preview_yaw)
        preview_rear_y = preview.y_m - offset * math.sin(preview_yaw)
        preview_error_x = predicted_rear_x - preview_rear_x
        preview_error_y = predicted_rear_y - preview_rear_y
        predicted_cross_track_error = (
            -math.sin(preview_yaw) * preview_error_x
            + math.cos(preview_yaw) * preview_error_y
        )
        preview_weight = (
            self.limits.steering_preview_weight
            + rejoin_ratio
            * (
                self.limits.steering_rejoin_preview_weight
                - self.limits.steering_preview_weight
            )
        )
        cross_track_error = (
            (1.0 - preview_weight) * current_cross_track_error
            + preview_weight * predicted_cross_track_error
        )
        # 상단 카메라 중심 검출의 mm 단위 흔들림을 그대로 고 gain에 넣으면
        # 조향 부호가 매 주기 바뀌며 직선에서 S자로 주행한다. 작은 오차는
        # 무시하고 deadband를 넘은 실제 이탈분에만 연속적으로 보정한다.
        effective_cross_track_error = math.copysign(
            max(
                0.0,
                abs(cross_track_error)
                - active_cross_track_deadband,
            ),
            cross_track_error,
        )
        feedback = (
            -self.limits.cross_track_feedback_gain_1pm2
            * (
                self.limits.reverse_cross_track_gain_scale
                if direction < 0
                else 1.0
            )
            * effective_cross_track_error
        )
        # 경로에 닿는 위치만 맞추면 차량이 접선과 비스듬한 상태로 선을
        # 통과한 뒤 반대 조향하며 S자가 된다. 진행 방향을 고려한 heading
        # feedback으로 합류 순간에 경로 접선과 나란해지도록 감쇠한다.
        tracking_heading_error = normalize_angle(nearest_yaw - state.yaw_rad)
        active_heading_deadband = (
            self.limits.reverse_heading_feedback_deadband_rad
            if direction < 0
            else self.limits.heading_feedback_deadband_rad
        )
        effective_heading_error = math.copysign(
            max(
                0.0,
                abs(tracking_heading_error)
                - active_heading_deadband,
            ),
            tracking_heading_error,
        )
        feedback += (
            direction
            * self.limits.heading_feedback_gain_1pmprad
            * (
                self.limits.reverse_heading_gain_scale
                if direction < 0
                else 1.0
            )
            * effective_heading_error
        )
        curvature_delta = (
            self.limits.max_curvature_rate_1pmps * self.limits.dt_sec
        )
        effective_limit = curvature_limit
        if abs(speed) > 1e-9:
            effective_limit = min(
                effective_limit,
                self.limits.max_angular_speed_radps / abs(speed),
            )
        curvature = float(
            np.clip(
                curvature + feedback,
                max(-effective_limit, self.last_curvature_1pm - curvature_delta),
                min(effective_limit, self.last_curvature_1pm + curvature_delta),
            )
        )
        absolute_cross_track_error = abs(cross_track_error)
        if absolute_cross_track_error > self.limits.cross_track_slowdown_start_m:
            slowdown_ratio = min(
                1.0,
                (
                    absolute_cross_track_error
                    - self.limits.cross_track_slowdown_start_m
                )
                / (
                    self.limits.cross_track_slowdown_full_m
                    - self.limits.cross_track_slowdown_start_m
                ),
            )
            speed_scale = 1.0 - slowdown_ratio * (
                1.0 - self.limits.minimum_tracking_speed_scale
            )
            speed *= speed_scale
        if heading_recovery:
            # 큰 yaw 오차에서 정지하면 차동구동 차량은 스스로 복구할 수 없다.
            # 다만 기준속도로 계속 진행하면 좁은 벽 근처에서 최대 곡률로
            # 회복하기 전에 차체가 벽 쪽으로 더 이동한다. 횡오차 복귀용
            # 최소 속도로 감속하면서 최대 곡률을 유지한다. 장애물 정지는
            # follower가 이 명령을 발행하기 전에 기존대로 적용한다.
            speed_delta = (
                self.limits.max_acceleration_mps2 * self.limits.dt_sec
            )
            nominal_speed = (
                self.limits.forward_speed_mps
                if direction > 0
                else self.limits.reverse_speed_mps
            )
            target_recovery_speed = (
                nominal_speed * self.limits.heading_recovery_speed_scale
            )
            current_speed_magnitude = abs(self.last_speed_mps)
            if current_speed_magnitude > target_recovery_speed:
                recovery_speed_magnitude = max(
                    target_recovery_speed,
                    current_speed_magnitude - speed_delta,
                )
            else:
                recovery_speed_magnitude = min(
                    target_recovery_speed,
                    current_speed_magnitude + speed_delta,
                )
            speed = math.copysign(recovery_speed_magnitude, direction)
            recovery_scale = min(
                1.0,
                abs(signed_tracking_yaw_error)
                / self.limits.heading_recovery_full_curvature_error_rad,
            )
            recovery_limit = (
                self.limits.max_curvature_1pm * recovery_scale
            )
            if abs(speed) > 1e-9:
                recovery_limit = min(
                    recovery_limit,
                    self.limits.max_angular_speed_radps / abs(speed),
                )
            curvature = math.copysign(
                recovery_limit,
                signed_tracking_yaw_error * speed,
            )
        if (
            abs(speed) <= self.limits.gear_stall_speed_threshold_mps
            and endpoint_distance
            > self.limits.gear_fallback_position_tolerance_m
        ):
            # 짧고 급한 주차 곡선에서 SLSQP가 경로 중간의 0속도 local
            # minimum에 고정되는 것을 막는다. 저속으로만 진행시키고 기존
            # curvature/feedback은 유지해 다음 주기에 다시 최적화한다.
            reference_speed = (
                self.limits.forward_speed_mps
                if direction > 0
                else -self.limits.reverse_speed_mps
            )
            speed = (
                reference_speed
                * self.limits.minimum_tracking_speed_scale
            )
        if (
            not is_final_segment
            and near_segment_end
            and (
                self._arc_length[segment_end]
                - self._arc_length[self._segment_start(self.progress_index)]
                <= self.limits.gear_fallback_max_segment_length_m
            )
            and endpoint_distance
            <= self.limits.gear_fallback_position_tolerance_m
            and abs(speed) <= self.limits.gear_stall_speed_threshold_mps
        ):
            # 위치 노이즈나 비홀로노믹 종점 수렴 때문에 기본 전환 반경 바로
            # 밖에서 0속도 해에 굳으면, 충분히 가까울 때만 후진 구간으로 넘긴다.
            return self.stop("GEAR_CHANGE_REQUIRED")
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

    def _initial_nearest_index(self, state: VehicleState) -> int:
        segment_end = self._segment_end(self.progress_index)
        return min(
            range(self.progress_index, segment_end + 1),
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

    def _segment_tangent_yaw(self, index: int) -> float:
        """기준점과 다음 좌표로 차량이 맞춰야 할 실제 헤딩을 계산한다."""
        point = self.path[index]
        segment_start = self._segment_start(index)
        segment_end = self._segment_end(index)
        neighbour_index: int | None = None
        motion_x = 0.0
        motion_y = 0.0

        # 같은 기어 구간의 다음 좌표를 우선 사용한다. 중복 좌표가 있으면
        # 실제 방향을 정의할 수 있는 첫 좌표까지 건너뛴다.
        for candidate_index in range(index + 1, segment_end + 1):
            candidate = self.path[candidate_index]
            motion_x = candidate.x_m - point.x_m
            motion_y = candidate.y_m - point.y_m
            if math.hypot(motion_x, motion_y) > 1e-9:
                neighbour_index = candidate_index
                break

        # 구간 마지막점에서는 직전 좌표→현재 좌표의 접선을 사용한다.
        if neighbour_index is None:
            for candidate_index in range(index - 1, segment_start - 1, -1):
                candidate = self.path[candidate_index]
                motion_x = point.x_m - candidate.x_m
                motion_y = point.y_m - candidate.y_m
                if math.hypot(motion_x, motion_y) > 1e-9:
                    neighbour_index = candidate_index
                    break

        if neighbour_index is None:
            return point.yaw_rad

        tangent_yaw = math.atan2(motion_y, motion_x)
        if point.direction < 0:
            # 후진 경로의 좌표 진행 방향은 차량 전면 헤딩과 반대다.
            tangent_yaw += math.pi
        return normalize_angle(tangent_yaw)

    def _anticipated_curvature(self, start: int, segment_end: int) -> float:
        """가까운 미래 곡률을 거리 가중 평균해 코너 선행 조향을 만든다."""
        stop = min(
            segment_end,
            start + self.limits.curve_feedforward_preview_points,
        )
        values = self._reference_curvature[start : stop + 1]
        if len(values) == 0:
            return 0.0
        # 현재점의 비중이 가장 크고 먼 점은 작게 반영된다. 곡선이 preview
        # 끝에 처음 보일 때는 약하게, 가까워질수록 자연스럽게 강해진다.
        weights = np.linspace(1.0, 0.2, len(values), dtype=np.float64)
        return float(np.dot(values, weights) / np.sum(weights))

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
        linear = controls[:, 0]
        angular = linear * controls[:, 1]
        half_track = self.limits.wheel_separation_m * 0.5
        left_wheel_radps = (
            linear - angular * half_track
        ) / self.limits.wheel_radius_m
        right_wheel_radps = (
            linear + angular * half_track
        ) / self.limits.wheel_radius_m
        maximum_wheel_speed_squared = (
            self.limits.max_wheel_angular_speed_radps**2
        )
        return np.concatenate(
            (
                self.limits.max_angular_speed_radps**2 - angular**2,
                maximum_wheel_speed_squared - left_wheel_radps**2,
                maximum_wheel_speed_squared - right_wheel_radps**2,
            )
        )

    def _objective(
        self,
        flat_controls: np.ndarray,
        state: VehicleState,
        references: Sequence[tuple[ReferencePoint, float]],
        direction: int,
    ) -> float:
        controls = np.asarray(flat_controls, dtype=np.float64).reshape(-1, 2)
        offset = self.limits.control_point_offset_m
        yaw = state.yaw_rad
        # 최적화의 운동 방정식은 cmd_vel의 기준점인 rear axle에서 계산한다.
        # 외부 pose/path와 완료·횡오차 판정은 계속 차량 중심 기준이다.
        x = state.x_m - offset * math.cos(yaw)
        y = state.y_m - offset * math.sin(yaw)
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
            reference_x = reference.x_m - offset * math.cos(reference.yaw_rad)
            reference_y = reference.y_m - offset * math.sin(reference.yaw_rad)
            position_error = (x - reference_x) ** 2 + (y - reference_y) ** 2
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
