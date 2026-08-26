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
class _PathProjection:
    """현재 rear axle을 전진 기준 경로에 연속 투영한 결과다."""

    lower_index: int
    ratio: float
    rear_s_m: float
    rear_x_m: float
    rear_y_m: float
    center_x_m: float
    center_y_m: float
    center_tangent_yaw_rad: float
    body_yaw_rad: float
    center_cross_track_m: float
    rear_cross_track_m: float


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
    parking_realign_speed_mps: float = 0.035
    first_forward_endpoint_slowdown_distance_m: float = 0.12
    parking_realign_endpoint_slowdown_distance_m: float = 0.06
    first_forward_endpoint_min_speed_mps: float = 0.024
    forward_gear_endpoint_min_speed_mps: float = 0.018
    reverse_speed_mps: float = 0.02
    max_forward_speed_mps: float = 0.08
    max_reverse_speed_mps: float = 0.03
    max_acceleration_mps2: float = 0.12
    forward_max_acceleration_mps2: float = 0.08
    max_curvature_1pm: float = 1.0 / 0.12
    max_curvature_rate_1pmps: float = 10.0
    straight_curvature_rate_1pmps: float = 6.0
    curvature_rate_speed_reduction: float = 0.25
    max_angular_speed_radps: float = 0.40
    max_lateral_acceleration_mps2: float = 0.015
    straight_curvature_threshold_1pm: float = 0.35
    straight_max_curvature_1pm: float = 3.0
    full_curvature_path_threshold_1pm: float = 6.0
    cross_track_feedback_gain_1pm2: float = 15.0
    forward_cross_track_gain_scale: float = 1.0
    forward_converging_cross_track_gain_scale: float = 0.25
    forward_straight_heading_gain_scale: float = 1.8
    forward_rejoin_lookahead_m: float = 0.13
    forward_rejoin_max_heading_rad: float = math.radians(15.0)
    forward_curve_rejoin_lookahead_m: float = 0.15
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
    path_start_max_distance_m: float = 0.15
    goal_position_tolerance_m: float = 0.03
    goal_yaw_tolerance_rad: float = math.radians(8.0)
    gear_position_tolerance_m: float = 0.01
    gear_fallback_position_tolerance_m: float = 0.04
    gear_stall_speed_threshold_mps: float = 0.003
    gear_fallback_max_segment_length_m: float = 0.15
    gear_transition_end_guard_points: int = 20
    nearest_forward_window: int = 12
    nearest_backward_window: int = 4
    # 후진은 기존 가까운 preview를 유지하고 전진에서만 조금 더 앞의
    # 경로 접선을 함께 사용해 한 점을 쫓는 조향을 줄인다.
    steering_preview_points: int = 6
    steering_preview_weight: float = 0.30
    forward_steering_preview_points: int = 6
    forward_steering_preview_weight: float = 0.30
    steering_rejoin_preview_points: int = 12
    steering_rejoin_full_error_m: float = 0.03
    steering_rejoin_preview_weight: float = 0.65
    curve_feedforward_preview_points: int = 14
    reverse_curve_feedforward_preview_points: int = 14
    curve_feedforward_gain: float = 0.55
    forward_large_curve_feedforward_gain: float = 0.42
    curve_feedforward_deadband_1pm: float = 0.5
    curvature_smoothing_points: int = 5
    straight_lookahead_points: int = 4
    straight_history_points: int = 12
    solver_max_iterations: int = 45
    solver_ftol: float = 1e-5

    def validate(self) -> None:
        positive_values = (
            self.dt_sec,
            self.forward_speed_mps,
            self.parking_realign_speed_mps,
            self.first_forward_endpoint_slowdown_distance_m,
            self.parking_realign_endpoint_slowdown_distance_m,
            self.first_forward_endpoint_min_speed_mps,
            self.forward_gear_endpoint_min_speed_mps,
            self.reverse_speed_mps,
            self.max_forward_speed_mps,
            self.max_reverse_speed_mps,
            self.max_acceleration_mps2,
            self.forward_max_acceleration_mps2,
            self.wheel_radius_m,
            self.wheel_separation_m,
            self.max_wheel_angular_speed_radps,
            self.max_curvature_1pm,
            self.max_curvature_rate_1pmps,
            self.straight_curvature_rate_1pmps,
            self.max_angular_speed_radps,
            self.max_lateral_acceleration_mps2,
            self.straight_curvature_threshold_1pm,
            self.straight_max_curvature_1pm,
            self.full_curvature_path_threshold_1pm,
            self.cross_track_deadband_m,
            self.reverse_cross_track_deadband_m,
            self.reverse_cross_track_gain_scale,
            self.reverse_heading_gain_scale,
            self.forward_cross_track_gain_scale,
            self.forward_converging_cross_track_gain_scale,
            self.forward_straight_heading_gain_scale,
            self.forward_rejoin_lookahead_m,
            self.forward_rejoin_max_heading_rad,
            self.forward_curve_rejoin_lookahead_m,
            self.cross_track_slowdown_start_m,
            self.cross_track_slowdown_full_m,
            self.minimum_tracking_speed_scale,
            self.max_tracking_yaw_error_rad,
            self.heading_recovery_full_curvature_error_rad,
            self.heading_recovery_speed_scale,
            self.steering_rejoin_full_error_m,
            self.pose_timeout_sec,
            self.path_start_max_distance_m,
            self.goal_position_tolerance_m,
            self.goal_yaw_tolerance_rad,
            self.gear_position_tolerance_m,
            self.gear_fallback_position_tolerance_m,
            self.gear_stall_speed_threshold_mps,
            self.gear_fallback_max_segment_length_m,
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
            self.forward_converging_cross_track_gain_scale
            > self.forward_cross_track_gain_scale
        ):
            raise ValueError(
                "converging cross-track gain must not exceed forward gain"
            )
        if self.forward_straight_heading_gain_scale < 1.0:
            raise ValueError("straight heading gain scale must be at least one")
        if (
            not math.isfinite(self.heading_feedback_gain_1pmprad)
            or self.heading_feedback_gain_1pmprad < 0.0
        ):
            raise ValueError("heading feedback gain must be finite and non-negative")
        if self.straight_curvature_rate_1pmps > self.max_curvature_rate_1pmps:
            raise ValueError(
                "straight curvature rate must not exceed maximum curvature rate"
            )
        if not 0.0 <= self.curvature_rate_speed_reduction < 1.0:
            raise ValueError("curvature rate speed reduction must be in [0, 1)")
        if (
            self.full_curvature_path_threshold_1pm
            <= self.straight_curvature_threshold_1pm
            or self.straight_max_curvature_1pm > self.max_curvature_1pm
        ):
            raise ValueError("adaptive path-curvature range is invalid")
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
        if self.forward_steering_preview_points < 1:
            raise ValueError("forward steering preview points must be positive")
        if self.steering_rejoin_preview_points < max(
            self.steering_preview_points,
            self.forward_steering_preview_points,
        ):
            raise ValueError(
                "rejoin preview points must not be below normal preview points"
            )
        if (
            not math.isfinite(self.steering_preview_weight)
            or not 0.0 <= self.steering_preview_weight <= 1.0
        ):
            raise ValueError("steering preview weight must be between zero and one")
        if (
            not math.isfinite(self.forward_steering_preview_weight)
            or not 0.0 <= self.forward_steering_preview_weight <= 1.0
        ):
            raise ValueError(
                "forward steering preview weight must be between zero and one"
            )
        if (
            not math.isfinite(self.steering_rejoin_preview_weight)
            or not max(
                self.steering_preview_weight,
                self.forward_steering_preview_weight,
            )
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
        if self.reverse_curve_feedforward_preview_points < 1:
            raise ValueError(
                "reverse curve feed-forward preview points must be positive"
            )
        if (
            not math.isfinite(self.curve_feedforward_gain)
            or not 0.0 <= self.curve_feedforward_gain <= 1.0
        ):
            raise ValueError("curve feed-forward gain must be between zero and one")
        if (
            not math.isfinite(self.forward_large_curve_feedforward_gain)
            or not 0.0 <= self.forward_large_curve_feedforward_gain <= 1.0
        ):
            raise ValueError(
                "forward large-curve feed-forward gain must be between zero and one"
            )
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
        ):
            raise ValueError("MPC straight-section point counts are invalid")
        if self.solver_max_iterations < 1:
            raise ValueError("MPC solver_max_iterations must be positive")
        if self.forward_speed_mps > self.max_forward_speed_mps:
            raise ValueError("forward reference speed exceeds its limit")
        if self.parking_realign_speed_mps > self.max_forward_speed_mps:
            raise ValueError("parking realign speed exceeds forward limit")
        if (
            self.first_forward_endpoint_slowdown_distance_m
            <= self.gear_position_tolerance_m
        ):
            raise ValueError(
                "first-forward slowdown distance must exceed gear tolerance"
            )
        if (
            self.parking_realign_endpoint_slowdown_distance_m
            <= self.gear_position_tolerance_m
        ):
            raise ValueError(
                "parking-realign slowdown distance must exceed gear tolerance"
            )
        if (
            self.first_forward_endpoint_min_speed_mps
            > self.forward_speed_mps
        ):
            raise ValueError(
                "first-forward endpoint speed exceeds forward speed"
            )
        if (
            self.forward_gear_endpoint_min_speed_mps
            > self.parking_realign_speed_mps
        ):
            raise ValueError(
                "forward gear endpoint speed exceeds parking realign speed"
            )
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
    predicted_states: tuple[VehicleState, ...] = ()
    predicted_controls: tuple[tuple[float, float], ...] = ()
    reference_horizon: tuple[ReferencePoint, ...] = ()
    reference_curvatures_1pm: tuple[float, ...] = ()
    cross_track_error_m: float = 0.0
    heading_error_rad: float = 0.0
    speed_limit_mps: float = 0.0
    curvature_limit_1pm: float = 0.0
    angular_speed_limit_radps: float = 0.0


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
        self._rear_points = np.empty((0, 2), dtype=np.float64)
        self._rear_arc_length = np.empty(0, dtype=np.float64)
        self._reference_curvature = np.empty(0, dtype=np.float64)
        self.progress_index = 0
        self._progress_initialized = False
        self.last_speed_mps = 0.0
        self.last_curvature_1pm = 0.0
        self._warm_start: np.ndarray | None = None
        self._reset_forward_rejoin()

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
        self._straighten_short_initial_forward_connector(normalized)
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
        self._rear_points = np.asarray(rear_points, dtype=np.float64)
        rear_arc_length = [0.0]
        for first, second in zip(rear_points, rear_points[1:]):
            rear_arc_length.append(
                rear_arc_length[-1]
                + math.hypot(second[0] - first[0], second[1] - first[1])
            )
        self._rear_arc_length = np.asarray(rear_arc_length, dtype=np.float64)
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
        self._reset_forward_rejoin()

    def _straighten_short_initial_forward_connector(
        self,
        points: list[ReferencePoint],
    ) -> None:
        """Remove a sub-centimetre start kink before an otherwise straight run.

        The live route starts at the detected vehicle centre.  A small camera
        position error can therefore place only point 0 slightly off the fixed
        straight lane.  Using point 0 -> point 1 as a tangent turns that
        sub-millimetre lateral error into a large heading command.  Internally
        project only that first point onto the continuation of the verified
        straight line.  Reverse segments and genuine initial curves are left
        unchanged.
        """
        required_points = max(4, self.limits.straight_lookahead_points + 1)
        stop = min(len(points), required_points)
        if stop < 4 or points[0].direction < 0:
            return
        if any(point.direction != 1 for point in points[:stop]):
            return

        stable_dx = points[stop - 1].x_m - points[1].x_m
        stable_dy = points[stop - 1].y_m - points[1].y_m
        if math.hypot(stable_dx, stable_dy) <= 1e-9:
            return
        stable_yaw = math.atan2(stable_dy, stable_dx)
        angular_tolerance = max(
            self.limits.heading_feedback_deadband_rad,
            math.radians(1.5),
        )

        stable_distances: list[float] = []
        for first, second in zip(points[1 : stop - 1], points[2:stop]):
            dx = second.x_m - first.x_m
            dy = second.y_m - first.y_m
            distance = math.hypot(dx, dy)
            if distance <= 1e-9:
                return
            segment_yaw = math.atan2(dy, dx)
            if abs(normalize_angle(segment_yaw - stable_yaw)) > angular_tolerance:
                return
            stable_distances.append(distance)
        if any(
            abs(normalize_angle(point.yaw_rad - stable_yaw)) > angular_tolerance
            for point in points[1:stop]
        ):
            return
        if (
            abs(normalize_angle(points[0].yaw_rad - stable_yaw))
            > angular_tolerance
        ):
            return

        first_distance = math.hypot(
            points[1].x_m - points[0].x_m,
            points[1].y_m - points[0].y_m,
        )
        nominal_distance = float(np.median(stable_distances))
        if first_distance <= 1e-9 or first_distance > max(0.015, 2.5 * nominal_distance):
            return

        points[0] = ReferencePoint(
            points[1].x_m - first_distance * math.cos(stable_yaw),
            points[1].y_m - first_distance * math.sin(stable_yaw),
            normalize_angle(stable_yaw),
            1,
        )

    def clear_path(self) -> None:
        self.path = ()
        self._arc_length = np.empty(0, dtype=np.float64)
        self._rear_points = np.empty((0, 2), dtype=np.float64)
        self._rear_arc_length = np.empty(0, dtype=np.float64)
        self._reference_curvature = np.empty(0, dtype=np.float64)
        self.progress_index = 0
        self._progress_initialized = False
        self.last_speed_mps = 0.0
        self.last_curvature_1pm = 0.0
        self._warm_start = None
        self._reset_forward_rejoin()

    def restore_progress(self, progress_index: int) -> None:
        """동일 경로 재설정 뒤 저장한 진행도를 복원한다."""
        if not self.path:
            raise ValueError("Cannot restore progress without an MPC path")
        if not 0 <= progress_index < len(self.path):
            raise ValueError("MPC progress index is outside the path")
        self.progress_index = progress_index
        self._progress_initialized = True
        self._reset_forward_rejoin()

    def stop(self, status: str, *, clear_rejoin: bool = False) -> MpcCommand:
        self.last_speed_mps = 0.0
        self.last_curvature_1pm = 0.0
        self._warm_start = None
        # Pose/scan timeout처럼 잠깐 멈춘 경우에는 latch된 합류 곡선을
        # 보존한다. 재개할 때 매번 새 목표를 만들면 다시 moving target이 된다.
        if clear_rejoin:
            self._reset_forward_rejoin()
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
        self._reset_forward_rejoin()
        return True

    def _reset_forward_rejoin(self) -> None:
        self._forward_rejoin_active = False
        self._forward_rejoin_start_s_m = 0.0
        self._forward_rejoin_length_m = 0.0
        self._forward_rejoin_offset_m = 0.0
        self._forward_rejoin_target_offset_m = 0.0
        self._forward_rejoin_slope = 0.0

    def command(self, state: VehicleState) -> MpcCommand:
        if not self.path:
            return self.stop("NO_PATH", clear_rejoin=True)
        if any(
            not math.isfinite(value)
            for value in (state.x_m, state.y_m, state.yaw_rad)
        ):
            return self.stop("INVALID_POSE")

        if not self._progress_initialized:
            # 새 고정 경로는 현재 endpoint에서 시작하므로 첫 검색도 제한된
            # forward window 안에서만 수행한다.
            self.progress_index = self._initial_nearest_index(state)
            initial_reference = self.path[self.progress_index]
            initial_distance = math.hypot(
                initial_reference.x_m - state.x_m,
                initial_reference.y_m - state.y_m,
            )
            if initial_distance > self.limits.path_start_max_distance_m:
                return self.stop(
                    f"PATH_START_TOO_FAR_{initial_distance:.3f}m",
                    clear_rejoin=True,
                )
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
        remaining_segment_arc_m = max(
            0.0,
            self._arc_length[segment_end]
            - self._arc_length[self.progress_index],
        )
        direction = self.path[self.progress_index].direction
        endpoint_heading = self._segment_tangent_yaw(segment_end)
        endpoint_error_x = state.x_m - endpoint.x_m
        endpoint_error_y = state.y_m - endpoint.y_m
        passed_endpoint_distance = direction * (
            endpoint_error_x * math.cos(endpoint_heading)
            + endpoint_error_y * math.sin(endpoint_heading)
        )
        if (
            endpoint_distance <= self.limits.goal_position_tolerance_m
            and endpoint_yaw_error <= self.limits.goal_yaw_tolerance_rad
            and is_final_segment
        ):
            return self.stop("GOAL_REACHED", clear_rejoin=True)
        if (
            not is_final_segment
            and remaining_segment_arc_m
            <= self.limits.gear_position_tolerance_m + 1e-9
        ):
            # 마지막 약 1cm까지 같은 direction 구간을 순서대로 진행했다면
            # optimizer가 cusp 직전에 0속도로 수렴해도 다음 기어로 넘긴다.
            # 이 판정은 차량 위치/yaw 오차를 사용하지 않는다.
            return self.stop("GEAR_CHANGE_REQUIRED", clear_rejoin=True)
        if (
            not is_final_segment
            and near_segment_end
            and passed_endpoint_distance >= 0.0
        ):
            # 전환점의 접선 평면을 통과한 뒤에는 원형 거리와 횡오차가
            # 다시 커질 수 있다. 구간 끝 guard 안까지 순서대로 진행한
            # 사실을 전환 기준으로 삼고 위치/yaw 오차로 전환을 막지 않는다.
            return self.stop("GEAR_CHANGE_REQUIRED", clear_rejoin=True)

        segment_start = self._segment_start(self.progress_index)
        # 후진-전진-후진으로 구성된 3-point 주차에서 가운데 전진은
        # 이동 구간이 아니라 최종 후진축에 자세를 맞추는 정렬 구간이다.
        parking_realign = (
            direction > 0
            and segment_start > 0
            and segment_end + 1 < len(self.path)
            and self.path[segment_start - 1].direction < 0
            and self.path[segment_end + 1].direction < 0
        )
        segment_reference_speed_mps = (
            self.limits.parking_realign_speed_mps
            if parking_realign
            else None
        )
        forward_endpoint_slowdown_m: float | None = None
        forward_endpoint_base_speed_mps: float | None = None
        forward_endpoint_min_speed_mps: float | None = None
        # 첫 후진과 최종 후진 사이의 재전진은 구간 전체가 정렬 동작이다.
        # 시작부터 실제 경로 선분/접선을 직접 따라 최종 후진축에 합류한다.
        forward_endpoint_alignment = parking_realign
        if direction > 0 and not is_final_segment:
            if segment_start == 0:
                forward_endpoint_slowdown_m = (
                    self.limits.first_forward_endpoint_slowdown_distance_m
                )
                forward_endpoint_base_speed_mps = self.limits.forward_speed_mps
                forward_endpoint_min_speed_mps = (
                    self.limits.first_forward_endpoint_min_speed_mps
                )
            elif parking_realign:
                forward_endpoint_slowdown_m = (
                    self.limits.parking_realign_endpoint_slowdown_distance_m
                )
                forward_endpoint_base_speed_mps = (
                    self.limits.parking_realign_speed_mps
                )
                forward_endpoint_min_speed_mps = (
                    self.limits.forward_gear_endpoint_min_speed_mps
                )
        if (
            forward_endpoint_slowdown_m is not None
            and forward_endpoint_base_speed_mps is not None
            and forward_endpoint_min_speed_mps is not None
        ):
            # 첫 전진과 짧은 재전진은 끝점 접근 속도만 각각 낮춘다. 재전진
            # 마지막의 작은 위치 보정이 한 주기에 큰 횡이동으로 나타나는 것을
            # 줄이면서 후진 구간의 속도와 조향 계산은 그대로 보존한다.
            remaining_forward_m = max(
                0.0,
                self._arc_length[segment_end]
                - self._arc_length[self.progress_index],
            )
            slowdown_distance_m = forward_endpoint_slowdown_m
            if remaining_forward_m < slowdown_distance_m:
                # 마지막 짧은 구간은 새 합류곡선을 만들지 않고 현재 path
                # 선분과 접선을 직접 따라 전환점까지 자세를 정렬한다.
                forward_endpoint_alignment = (
                    forward_endpoint_alignment
                    or remaining_forward_m
                    <= min(0.08, slowdown_distance_m)
                )
                tolerance_m = self.limits.gear_position_tolerance_m
                ratio = float(
                    np.clip(
                        (remaining_forward_m - tolerance_m)
                        / max(slowdown_distance_m - tolerance_m, 1e-6),
                        0.0,
                        1.0,
                    )
                )
                smooth_ratio = ratio * ratio * (3.0 - 2.0 * ratio)
                segment_reference_speed_mps = (
                    forward_endpoint_min_speed_mps
                    + smooth_ratio
                    * (
                        forward_endpoint_base_speed_mps
                        - forward_endpoint_min_speed_mps
                    )
                )
        projection = (
            self._forward_path_projection(state)
            if direction > 0
            else None
        )
        # 고정 경로의 x/y는 rear axle 궤적을 차량 중심으로 옮긴 좌표다.
        # 전진에서는 최근접 vertex가 아니라 rear path의 선분 위에 연속
        # 투영해 reference 시간축과 CTE가 0.5cm 샘플 경계마다 튀지 않게 한다.
        if projection is not None:
            horizon_start_s_m = projection.rear_s_m
            path_tangent_yaw = projection.center_tangent_yaw_rad
            reference_yaw = projection.body_yaw_rad
            current_tracking_error = projection.center_cross_track_m
        else:
            horizon_start_s_m = None
            path_tangent_yaw = self._segment_tangent_yaw(self.progress_index)
            reference_yaw = self.path[self.progress_index].yaw_rad
            current_tracking_error = self._current_cross_track_error(
                state,
                self.path[self.progress_index],
                path_tangent_yaw,
                direction,
            )
        base_references = self._reference_horizon(
            self.progress_index,
            segment_end,
            start_s_m=horizon_start_s_m,
            apply_forward_rejoin=False,
            reference_speed_mps=segment_reference_speed_mps,
        )
        path_curvature_limit = self._curvature_limit(
            base_references,
            self.progress_index,
            segment_end,
            direction,
        )
        curve_ratio = self._curve_ratio_from_limit(path_curvature_limit)
        forward_straight_tracking = (
            direction > 0
            and path_curvature_limit
            <= self.limits.straight_max_curvature_1pm + 1e-9
        )
        if direction > 0 and projection is not None:
            if forward_endpoint_alignment:
                # 이전에 시작한 긴 합류곡선도 종점 직전에는 끝내고, 남은
                # 실제 경로의 선과 yaw를 직접 추종한다.
                self._reset_forward_rejoin()
            else:
                self._update_forward_rejoin(
                    state,
                    projection,
                    segment_start,
                    segment_end,
                    forward_straight_tracking,
                )
            rejoin_reference_speed_mps = segment_reference_speed_mps
            if self._forward_rejoin_active:
                rejoin_reference_speed_mps = (
                    (
                        segment_reference_speed_mps
                        or self.limits.forward_speed_mps
                    )
                    * self._tracking_speed_scale(
                        abs(current_tracking_error)
                    )
                )
            references = self._reference_horizon(
                self.progress_index,
                segment_end,
                start_s_m=projection.rear_s_m,
                apply_forward_rejoin=True,
                reference_speed_mps=rejoin_reference_speed_mps,
            )
            if self._forward_rejoin_active:
                current_rejoin_reference, _ = self._forward_rejoin_reference(
                    projection.rear_s_m,
                    segment_start,
                    segment_end,
                )
                tracking_reference_yaw = current_rejoin_reference.yaw_rad
            else:
                tracking_reference_yaw = reference_yaw
        else:
            self._reset_forward_rejoin()
            references = base_references
            tracking_reference_yaw = reference_yaw
            rejoin_reference_speed_mps = None
        signed_tracking_yaw_error = normalize_angle(
            tracking_reference_yaw - state.yaw_rad
        )
        heading_recovery = (
            abs(signed_tracking_yaw_error)
            > self.limits.max_tracking_yaw_error_rad
        )
        planned_recovery_scale = (
            self._heading_recovery_ratio(signed_tracking_yaw_error)
            if heading_recovery
            else 0.0
        )
        # 경로 구간이 바뀌어 목표 곡률 한계가 낮아져도 이전 명령을 즉시
        # 잘라 조향을 튀게 하지 않는다. solver에는 직전 명령까지 허용하고
        # 최종 출력에서 적응형 변화율로 새 한계까지 부드럽게 내린다.
        solver_curvature_limit = max(
            path_curvature_limit,
            min(self.limits.max_curvature_1pm, abs(self.last_curvature_1pm)),
        )
        nominal_rate_speed = (
            (
                rejoin_reference_speed_mps
                or self.limits.forward_speed_mps
            )
            if direction > 0
            else self.limits.reverse_speed_mps
        )
        planned_curvature_rate = self._adaptive_curvature_rate(
            curve_ratio,
            nominal_rate_speed,
            direction,
            planned_recovery_scale,
        )
        initial = self._initial_guess(
            direction,
            references,
            planned_curvature_rate,
        )
        bounds = self._bounds(direction, solver_curvature_limit)
        rate_constraint = self._rate_constraint(
            direction,
            planned_curvature_rate,
        )
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
            args=(state, references, direction, rejoin_reference_speed_mps),
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
            (
                self.limits.curve_feedforward_preview_points
                if direction > 0
                else self.limits.reverse_curve_feedforward_preview_points
            ),
        )
        # 최적화 horizon을 늘려 계산 주기를 늦추지 않고, 가까워지는 곡선의
        # reference curvature의 미래 증가분을 현재 명령에 점진적으로 더한다.
        # 직선에서는 현재·미래가 모두 0이므로 불필요한 조향이 생기지 않고,
        # 횡오차 복귀를 위해 MPC가 만든 조향도 약화시키지 않는다.
        if not (direction > 0 and self._forward_rejoin_active):
            current_reference_curvature = float(
                self._reference_curvature[self.progress_index]
            )
            feedforward_gain = self.limits.curve_feedforward_gain
            if direction > 0:
                # 전진 코너 진입처럼 미래 곡률이 현재보다 같은 방향으로
                # 커질 때만 큰 곡선 gain을 사용한다. 코너 출구나 S자 전환에서
                # 반대 조향을 증폭하지 않으며 후진은 기본 gain을 유지한다.
                same_turn_direction = (
                    anticipated_curvature * current_reference_curvature >= 0.0
                )
                entering_curve = (
                    abs(anticipated_curvature)
                    > abs(current_reference_curvature) + 1e-9
                )
                if same_turn_direction and entering_curve:
                    feedforward_gain += curve_ratio * (
                        self.limits.forward_large_curve_feedforward_gain
                        - self.limits.curve_feedforward_gain
                    )
            curvature += feedforward_gain * (
                anticipated_curvature
                - current_reference_curvature
            )
        nearest = self.path[self.progress_index]
        nearest_yaw = reference_yaw
        offset = self.limits.control_point_offset_m
        state_rear_x = state.x_m - offset * math.cos(state.yaw_rad)
        state_rear_y = state.y_m - offset * math.sin(state.yaw_rad)
        if projection is not None:
            # 전진은 horizon과 같은 연속 투영점을 preview 시작점으로 쓴다.
            # vertex 중심점과 보간 yaw를 섞으면 샘플 경계마다 수 mm의 가짜
            # preview 거리와 횡오차가 생겨 합류 뒤 조향이 다시 흔들린다.
            nearest_rear_x = projection.rear_x_m
            nearest_rear_y = projection.rear_y_m
        else:
            nearest_rear_x = nearest.x_m - offset * math.cos(nearest_yaw)
            nearest_rear_y = nearest.y_m - offset * math.sin(nearest_yaw)
        # 중앙 카메라 pose와 고정 경로는 모두 차량 중심 기준이다. 전진
        # 횡오차까지 rear axle로 옮기면 heading 오차가 있는 순간 offset의
        # 횡성분을 실제 경로 이탈로 잘못 더하게 된다. 후진은 이미 실차에서
        # 검증된 기존 rear-axle 보정을 유지하고 전진만 중심끼리 비교한다.
        current_cross_track_error = (
            projection.center_cross_track_m
            if projection is not None
            else self._current_cross_track_error(
                state,
                nearest,
                path_tangent_yaw,
                direction,
            )
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
        base_preview_points = (
            self.limits.forward_steering_preview_points
            if direction > 0
            else self.limits.steering_preview_points
        )
        base_preview_weight = (
            self.limits.forward_steering_preview_weight
            if direction > 0
            else self.limits.steering_preview_weight
        )
        dynamic_preview_points = int(
            round(
                base_preview_points
                + rejoin_ratio
                * (
                    self.limits.steering_rejoin_preview_points
                    - base_preview_points
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
        preview_tangent_yaw = self._segment_tangent_yaw(preview_end)
        preview_yaw = preview.yaw_rad
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
        if direction > 0:
            predicted_x = predicted_rear_x + offset * math.cos(predicted_yaw)
            predicted_y = predicted_rear_y + offset * math.sin(predicted_yaw)
            preview_error_x = predicted_x - preview.x_m
            preview_error_y = predicted_y - preview.y_m
        else:
            preview_error_x = predicted_rear_x - preview_rear_x
            preview_error_y = predicted_rear_y - preview_rear_y
        predicted_cross_track_error = (
            -math.sin(preview_tangent_yaw) * preview_error_x
            + math.cos(preview_tangent_yaw) * preview_error_y
        )
        preview_weight = (
            base_preview_weight
            + rejoin_ratio
            * (
                self.limits.steering_rejoin_preview_weight
                - base_preview_weight
            )
        )
        cross_track_error = (
            (1.0 - preview_weight) * current_cross_track_error
            + preview_weight * predicted_cross_track_error
        )
        if forward_endpoint_alignment:
            # 종점 하나를 향하는 preview 보정 대신 연속 선분 투영 오차를
            # 사용해 마지막까지 경로 위에 붙은 상태로 이동한다.
            cross_track_error = current_cross_track_error
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
        # 긴 전진 직선에서는 휠 편차로 누적되는 횡오차를 일찍 닫는다.
        # 주차 전환점과 곡선까지 같은 gain을 쓰면 cusp 진입 자세가 오히려
        # 틀어질 수 있으므로 직선 곡률 제한이 활성화된 동안에만 강화한다.
        forward_cross_track_scale = 1.0
        forward_heading_scale = 1.0
        if direction > 0 and rejoin_reference_speed_mps is not None:
            # 재전진 정렬과 virtual rejoin에서 optimizer가 사용한 기준속도를
            # 실제 출력도 넘지 않게 해 예측과 실차 이동을 일치시킨다.
            speed = math.copysign(
                min(abs(speed), rejoin_reference_speed_mps),
                speed,
            )
        if direction > 0 and self._forward_rejoin_active:
            # 합류 곡선의 x/y/yaw/curvature를 optimizer가 직접 추종한다.
            # 원 경로 CTE를 다시 더하면 합류 곡선을 가로질러 오버슈팅하므로
            # MPC 이후의 직접 위치·heading 보정은 모두 끈다.
            forward_cross_track_scale = 0.0
            forward_heading_scale = 0.0
        elif direction > 0 and forward_endpoint_alignment:
            # 기어 전환을 오차로 막지는 않지만, 전환 직전에는 수렴 감쇠로
            # 위치 보정이 약해지지 않도록 경로선과 접선 보정을 유지한다.
            forward_cross_track_scale = self.limits.forward_cross_track_gain_scale
            forward_heading_scale = (
                self.limits.forward_straight_heading_gain_scale
                if forward_straight_tracking
                else 1.0
            )
        elif direction > 0:
            # 차량 heading만 유지해도 횡오차가 감소하는 상태에서는 위치
            # feedback을 계속 강하게 더하지 않는다. 기존에는 이 감쇠가
            # 직선에만 적용돼 곡선에서 경로 안쪽으로 수렴하는 중에도 반대
            # 조향이 생겼다. 경로 곡률과 heading feedback은 그대로 유지한다.
            convergence_ratio = 0.0
            if effective_cross_track_error * effective_heading_error > 0.0:
                convergence_ratio = float(
                    np.clip(
                        abs(effective_heading_error)
                        / max(2.0 * active_heading_deadband, 1e-6),
                        0.0,
                        1.0,
                    )
                )
            forward_cross_track_scale = (
                self.limits.forward_cross_track_gain_scale
                + convergence_ratio
                * (
                    self.limits.forward_converging_cross_track_gain_scale
                    - self.limits.forward_cross_track_gain_scale
                )
            )
            if forward_straight_tracking:
                forward_heading_scale = (
                    1.0
                    + convergence_ratio
                    * (self.limits.forward_straight_heading_gain_scale - 1.0)
                )
        feedback = (
            -self.limits.cross_track_feedback_gain_1pm2
            * (
                self.limits.reverse_cross_track_gain_scale
                if direction < 0
                else forward_cross_track_scale
            )
            * effective_cross_track_error
        )
        # 경로에 닿는 위치만 맞추면 차량이 접선과 비스듬한 상태로 선을
        # 통과한 뒤 반대 조향하며 S자가 된다. 진행 방향을 고려한 heading
        # feedback으로 합류 순간에 경로 접선과 나란해지도록 감쇠한다.
        feedback += (
            direction
            * self.limits.heading_feedback_gain_1pmprad
            * (
                self.limits.reverse_heading_gain_scale
                if direction < 0
                else forward_heading_scale
            )
            * effective_heading_error
        )
        curvature += feedback
        target_curvature_limit = path_curvature_limit
        recovery_scale = 0.0
        absolute_cross_track_error = abs(cross_track_error)
        if direction > 0 and self._forward_rejoin_active:
            # Horizon과 objective가 이미 같은 감속 속도를 사용한다. solve 뒤에
            # 다시 감속해 예측과 실제 이동을 어긋나게 하지 않고 상한만 지킨다.
            speed = math.copysign(
                min(abs(speed), rejoin_reference_speed_mps or abs(speed)),
                speed,
            )
        elif absolute_cross_track_error > self.limits.cross_track_slowdown_start_m:
            speed *= self._tracking_speed_scale(absolute_cross_track_error)
        if heading_recovery:
            # 큰 yaw 오차에서 정지하면 차동구동 차량은 스스로 복구할 수 없다.
            # 다만 기준속도로 계속 진행하면 좁은 벽 근처에서 최대 곡률로
            # 회복하기 전에 차체가 벽 쪽으로 더 이동한다. 횡오차 복귀용
            # 최소 속도로 감속하면서 최대 곡률을 유지한다. 장애물 정지는
            # follower가 이 명령을 발행하기 전에 기존대로 적용한다.
            acceleration_delta = (
                (
                    self.limits.forward_max_acceleration_mps2
                    if direction > 0
                    else self.limits.max_acceleration_mps2
                )
                * self.limits.dt_sec
            )
            deceleration_delta = (
                self.limits.max_acceleration_mps2 * self.limits.dt_sec
            )
            nominal_speed = (
                (
                    rejoin_reference_speed_mps
                    or self.limits.forward_speed_mps
                )
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
                    current_speed_magnitude - deceleration_delta,
                )
            else:
                recovery_speed_magnitude = min(
                    target_recovery_speed,
                    current_speed_magnitude + acceleration_delta,
                )
            speed = math.copysign(recovery_speed_magnitude, direction)
            recovery_scale = planned_recovery_scale
            recovery_limit = (
                path_curvature_limit
                + recovery_scale
                * (
                    self.limits.max_curvature_1pm
                    - path_curvature_limit
                )
            )
            target_curvature_limit = recovery_limit
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
                (
                    rejoin_reference_speed_mps
                    or self.limits.forward_speed_mps
                )
                if direction > 0
                else -self.limits.reverse_speed_mps
            )
            speed = (
                reference_speed
                * self.limits.minimum_tracking_speed_scale
            )
        # 모든 분기에서 만들어진 곡률을 마지막 한 곳에서 제한한다. 특히
        # heading recovery가 일반 rate limit을 우회해 한 주기에 20~30도
        # 상당의 조향을 만드는 현상을 막는다.
        desired_curvature = float(
            np.clip(
                curvature,
                -target_curvature_limit,
                target_curvature_limit,
            )
        )
        if direction > 0:
            # 빠른 상태에서 필요한 급곡선 곡률을 잘라 understeer를 만들지
            # 않고, 해당 곡률에서 각속도·횡가속도를 만족하도록 속도를 먼저
            # 낮춘다. 직선과 완만한 곡선에서는 기존 속도가 유지된다.
            speed_curvature_demand = max(
                abs(desired_curvature),
                abs(self.last_curvature_1pm),
            )
            adaptive_speed_limit = self._speed_limit_for_curvature(
                self.limits.max_forward_speed_mps,
                speed_curvature_demand,
            )
            speed = math.copysign(
                min(abs(speed), adaptive_speed_limit),
                speed,
            )
        curvature_rate = self._adaptive_curvature_rate(
            curve_ratio,
            speed,
            direction,
            recovery_scale,
        )
        curvature_delta = curvature_rate * self.limits.dt_sec
        curvature = float(
            np.clip(
                desired_curvature,
                self.last_curvature_1pm - curvature_delta,
                self.last_curvature_1pm + curvature_delta,
            )
        )
        physical_limit = self._speed_curvature_limit(
            self.limits.max_curvature_1pm,
            speed,
        )
        curvature = float(np.clip(curvature, -physical_limit, physical_limit))
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
            return self.stop("GEAR_CHANGE_REQUIRED", clear_rejoin=True)
        angular = speed * curvature
        if abs(angular) > self.limits.max_angular_speed_radps + 1e-6:
            return self.stop("SOLVER_ANGULAR_LIMIT")

        self.last_speed_mps = speed
        self.last_curvature_1pm = curvature
        debug_controls = controls.copy()
        # 첫 입력은 feed-forward, 변화율 제한, 물리 한계를 모두 적용한 실제
        # cmd_vel과 일치시킨다. 나머지는 optimizer가 계산한 horizon이다.
        debug_controls[0, 0] = speed
        debug_controls[0, 1] = curvature
        predicted_states = self._rollout_vehicle_center(state, debug_controls)
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
            predicted_states=predicted_states,
            predicted_controls=tuple(
                (float(control[0]), float(control[1]))
                for control in debug_controls
            ),
            reference_horizon=tuple(reference for reference, _ in references),
            reference_curvatures_1pm=tuple(
                float(reference_curvature)
                for _, reference_curvature in references
            ),
            cross_track_error_m=float(current_tracking_error),
            heading_error_rad=float(signed_tracking_yaw_error),
            speed_limit_mps=float(
                self.limits.max_forward_speed_mps
                if direction > 0
                else self.limits.max_reverse_speed_mps
            ),
            curvature_limit_1pm=float(physical_limit),
            angular_speed_limit_radps=float(self.limits.max_angular_speed_radps),
        )

    def _rollout_vehicle_center(
        self,
        state: VehicleState,
        controls: np.ndarray,
    ) -> tuple[VehicleState, ...]:
        """최적 입력 horizon을 DD rear-axle 모델로 적분해 중심 궤적으로 반환한다."""
        offset = self.limits.control_point_offset_m
        yaw = float(state.yaw_rad)
        rear_x = float(state.x_m) - offset * math.cos(yaw)
        rear_y = float(state.y_m) - offset * math.sin(yaw)
        predicted: list[VehicleState] = [state]
        for speed, curvature in np.asarray(controls, dtype=np.float64):
            angular = float(speed * curvature)
            rear_x += self.limits.dt_sec * float(speed) * math.cos(yaw)
            rear_y += self.limits.dt_sec * float(speed) * math.sin(yaw)
            yaw = normalize_angle(yaw + self.limits.dt_sec * angular)
            predicted.append(
                VehicleState(
                    x_m=rear_x + offset * math.cos(yaw),
                    y_m=rear_y + offset * math.sin(yaw),
                    yaw_rad=yaw,
                )
            )
        return tuple(predicted)

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

    def _forward_path_projection(self, state: VehicleState) -> _PathProjection:
        """전진 rear path 선분에 현재 자세를 투영해 연속 진행도를 만든다."""
        segment_start = self._segment_start(self.progress_index)
        segment_end = self._segment_end(self.progress_index)
        search_start = max(
            segment_start,
            self.progress_index - self.limits.nearest_backward_window - 1,
        )
        search_end = min(
            segment_end - 1,
            self.progress_index + self.limits.nearest_forward_window,
        )
        offset = self.limits.control_point_offset_m
        state_rear_x = state.x_m - offset * math.cos(state.yaw_rad)
        state_rear_y = state.y_m - offset * math.sin(state.yaw_rad)
        best: tuple[float, int, float, float, float] | None = None
        for lower in range(search_start, search_end + 1):
            first_x, first_y = self._rear_points[lower]
            second_x, second_y = self._rear_points[lower + 1]
            delta_x = second_x - first_x
            delta_y = second_y - first_y
            length_squared = delta_x * delta_x + delta_y * delta_y
            if length_squared <= 1e-16:
                continue
            ratio = float(
                np.clip(
                    (
                        (state_rear_x - first_x) * delta_x
                        + (state_rear_y - first_y) * delta_y
                    )
                    / length_squared,
                    0.0,
                    1.0,
                )
            )
            rear_x = float(first_x + ratio * delta_x)
            rear_y = float(first_y + ratio * delta_y)
            distance_squared = (
                (state_rear_x - rear_x) ** 2
                + (state_rear_y - rear_y) ** 2
            )
            candidate = (distance_squared, lower, ratio, rear_x, rear_y)
            if best is None or candidate[0] < best[0]:
                best = candidate

        if best is None:
            index = min(max(self.progress_index, segment_start), segment_end)
            rear_x, rear_y = self._rear_points[index]
            body_yaw = self.path[index].yaw_rad
            center_x = rear_x + offset * math.cos(body_yaw)
            center_y = rear_y + offset * math.sin(body_yaw)
            tangent_yaw = self._segment_tangent_yaw(index)
            return _PathProjection(
                index,
                0.0,
                float(self._rear_arc_length[index]),
                float(rear_x),
                float(rear_y),
                float(center_x),
                float(center_y),
                tangent_yaw,
                body_yaw,
                -math.sin(tangent_yaw) * (state.x_m - center_x)
                + math.cos(tangent_yaw) * (state.y_m - center_y),
                -math.sin(body_yaw) * (state_rear_x - rear_x)
                + math.cos(body_yaw) * (state_rear_y - rear_y),
            )

        _, lower, ratio, rear_x, rear_y = best
        first = self.path[lower]
        second = self.path[lower + 1]
        body_yaw = normalize_angle(
            first.yaw_rad
            + ratio * normalize_angle(second.yaw_rad - first.yaw_rad)
        )
        center_x = rear_x + offset * math.cos(body_yaw)
        center_y = rear_y + offset * math.sin(body_yaw)
        center_delta_x = second.x_m - first.x_m
        center_delta_y = second.y_m - first.y_m
        if math.hypot(center_delta_x, center_delta_y) <= 1e-12:
            center_tangent_yaw = body_yaw
        else:
            center_tangent_yaw = math.atan2(center_delta_y, center_delta_x)
        rear_s_m = float(
            self._rear_arc_length[lower]
            + ratio
            * (
                self._rear_arc_length[lower + 1]
                - self._rear_arc_length[lower]
            )
        )
        return _PathProjection(
            lower,
            ratio,
            rear_s_m,
            rear_x,
            rear_y,
            center_x,
            center_y,
            normalize_angle(center_tangent_yaw),
            body_yaw,
            -math.sin(center_tangent_yaw) * (state.x_m - center_x)
            + math.cos(center_tangent_yaw) * (state.y_m - center_y),
            -math.sin(body_yaw) * (state_rear_x - rear_x)
            + math.cos(body_yaw) * (state_rear_y - rear_y),
        )

    def _update_forward_rejoin(
        self,
        state: VehicleState,
        projection: _PathProjection,
        segment_start: int,
        segment_end: int,
        on_straight: bool,
    ) -> None:
        """현재 자세에서 원 경로까지 이어지는 고정 합류 곡선을 latch한다."""
        if self._forward_rejoin_active and projection.rear_s_m >= (
            self._forward_rejoin_start_s_m
            + self._forward_rejoin_length_m
        ):
            # 생성 당시 정한 합류 길이를 지나면 정상 경로 추종으로 반드시
            # 복귀한다. 실차의 mm/deg 잡음 때문에 합류 모드가 고착되면 큰
            # 곡선 보정과 직접 횡오차 보정이 계속 차단될 수 있다.
            self._reset_forward_rejoin()
        if self._forward_rejoin_active:
            return
        if abs(projection.center_cross_track_m) <= self.limits.cross_track_deadband_m:
            return

        minimum_length = (
            self.limits.forward_rejoin_lookahead_m
            if on_straight
            else self.limits.forward_curve_rejoin_lookahead_m
        )
        base_reference, base_curvature = self._interpolate_reference(
            projection.rear_s_m,
            segment_start,
            segment_end,
        )
        heading_error = normalize_angle(state.yaw_rad - base_reference.yaw_rad)
        limited_heading_error = float(
            np.clip(
                heading_error,
                -self.limits.forward_rejoin_max_heading_rad,
                self.limits.forward_rejoin_max_heading_rad,
            )
        )
        offset_m = projection.rear_cross_track_m
        target_offset_m = math.copysign(
            min(
                0.5 * self.limits.cross_track_deadband_m,
                0.5 * abs(offset_m),
            ),
            offset_m,
        )
        offset_change_m = offset_m - target_offset_m
        # 짧은 곡선으로 큰 횡오차를 닫으면 virtual curvature가 실차 한계를
        # 넘어 solver가 경로 바깥으로 밀린다. Quintic d''의 최대 계수(약 5.8)를
        # 이용해 필요한 최소 길이를 계산하고, 곡선 설정값은 하한으로 쓴다.
        curvature_reserve = max(
            0.35 * self.limits.max_curvature_1pm,
            0.8 * self.limits.max_curvature_1pm - abs(base_curvature),
        )
        curvature_safe_length = math.sqrt(
            6.0 * abs(offset_change_m) / curvature_reserve
        )
        requested_length = max(minimum_length, curvature_safe_length)
        remaining_length = float(
            self._rear_arc_length[segment_end] - projection.rear_s_m
        )
        if requested_length > remaining_length or requested_length < 0.03:
            # 기어 전환점 직전에는 짧은 잔여거리로 목표점 하나를 향하는
            # 불완전한 합류 곡선을 만들지 않는다. 현재 경로 접선을 유지해
            # 다음 기어를 안정된 자세로 시작한다.
            return
        rejoin_length = requested_length
        raw_slope = (
            1.0 - base_curvature * offset_m
        ) * math.tan(limited_heading_error)
        # 시작 heading을 가능한 만큼 이어받되, 합류 곡선 자체가 중심선을
        # 넘어가도록 가파른 slope는 제한한다. 경로 쪽 slope는 더 허용하고
        # 경로 반대쪽 slope는 작게 제한한다.
        if abs(offset_change_m) <= 1e-9:
            slope = 0.0
        else:
            slope_limit = (
                (1.5 if raw_slope * offset_change_m < 0.0 else 0.5)
                * abs(offset_change_m)
                / rejoin_length
            )
            slope = float(np.clip(raw_slope, -slope_limit, slope_limit))

        self._forward_rejoin_active = True
        self._forward_rejoin_start_s_m = projection.rear_s_m
        self._forward_rejoin_length_m = rejoin_length
        self._forward_rejoin_offset_m = offset_m
        self._forward_rejoin_target_offset_m = target_offset_m
        self._forward_rejoin_slope = slope

    def _forward_rejoin_reference(
        self,
        target_s_m: float,
        segment_start: int,
        segment_end: int,
    ) -> tuple[ReferencePoint, float]:
        """Latch된 quintic Frenet 합류 곡선 위 reference를 반환한다."""
        base_reference, base_curvature = self._interpolate_reference(
            target_s_m,
            segment_start,
            segment_end,
        )
        if not self._forward_rejoin_active:
            return base_reference, base_curvature
        length = self._forward_rejoin_length_m
        if length <= 1e-9:
            return base_reference, base_curvature
        u = float(
            np.clip(
                (target_s_m - self._forward_rejoin_start_s_m) / length,
                0.0,
                1.0,
            )
        )
        u2 = u * u
        u3 = u2 * u
        u4 = u3 * u
        u5 = u4 * u
        offset_shape = 1.0 - 10.0 * u3 + 15.0 * u4 - 6.0 * u5
        slope_shape = u - 6.0 * u3 + 8.0 * u4 - 3.0 * u5
        offset_shape_d1 = -30.0 * u2 + 60.0 * u3 - 30.0 * u4
        slope_shape_d1 = 1.0 - 18.0 * u2 + 32.0 * u3 - 15.0 * u4
        offset_shape_d2 = -60.0 * u + 180.0 * u2 - 120.0 * u3
        slope_shape_d2 = -36.0 * u + 96.0 * u2 - 60.0 * u3
        initial_offset = self._forward_rejoin_offset_m
        target_offset = self._forward_rejoin_target_offset_m
        offset_change = initial_offset - target_offset
        initial_slope = self._forward_rejoin_slope
        lateral_offset = (
            target_offset
            + offset_change * offset_shape
            + length * initial_slope * slope_shape
        )
        lateral_slope = (
            offset_change * offset_shape_d1 / length
            + initial_slope * slope_shape_d1
        )
        lateral_second = (
            offset_change * offset_shape_d2 / (length * length)
            + initial_slope * slope_shape_d2 / length
        )
        tangent_scale = 1.0 - base_curvature * lateral_offset
        tangent_norm = math.hypot(tangent_scale, lateral_slope)
        if tangent_norm <= 1e-9:
            return base_reference, base_curvature
        virtual_yaw = normalize_angle(
            base_reference.yaw_rad
            + math.atan2(lateral_slope, tangent_scale)
        )
        offset = self.limits.control_point_offset_m
        base_rear_x = base_reference.x_m - offset * math.cos(
            base_reference.yaw_rad
        )
        base_rear_y = base_reference.y_m - offset * math.sin(
            base_reference.yaw_rad
        )
        virtual_rear_x = (
            base_rear_x
            - math.sin(base_reference.yaw_rad) * lateral_offset
        )
        virtual_rear_y = (
            base_rear_y
            + math.cos(base_reference.yaw_rad) * lateral_offset
        )
        # k'(s)는 짧은 resampled 구간에서 작으므로 0으로 근사한다. d, d',
        # d''가 합류 끝에서 모두 0이어서 원 경로 곡률과는 연속이다.
        tangent_scale_derivative = -base_curvature * lateral_slope
        heading_offset_derivative = (
            tangent_scale * lateral_second
            - lateral_slope * tangent_scale_derivative
        ) / (tangent_norm * tangent_norm)
        virtual_curvature = float(
            np.clip(
                (base_curvature + heading_offset_derivative) / tangent_norm,
                -self.limits.max_curvature_1pm,
                self.limits.max_curvature_1pm,
            )
        )
        return (
            ReferencePoint(
                virtual_rear_x + offset * math.cos(virtual_yaw),
                virtual_rear_y + offset * math.sin(virtual_yaw),
                virtual_yaw,
                1,
            ),
            virtual_curvature,
        )

    def _current_cross_track_error(
        self,
        state: VehicleState,
        reference: ReferencePoint,
        reference_yaw: float,
        direction: int,
    ) -> float:
        """현재 제어점의 signed 횡오차를 계산한다."""
        if direction > 0:
            error_x = state.x_m - reference.x_m
            error_y = state.y_m - reference.y_m
        else:
            offset = self.limits.control_point_offset_m
            state_rear_x = state.x_m - offset * math.cos(state.yaw_rad)
            state_rear_y = state.y_m - offset * math.sin(state.yaw_rad)
            reference_rear_x = (
                reference.x_m - offset * math.cos(reference_yaw)
            )
            reference_rear_y = (
                reference.y_m - offset * math.sin(reference_yaw)
            )
            error_x = state_rear_x - reference_rear_x
            error_y = state_rear_y - reference_rear_y
        return (
            -math.sin(reference_yaw) * error_x
            + math.cos(reference_yaw) * error_y
        )

    def _initial_nearest_index(self, state: VehicleState) -> int:
        segment_end = self._segment_end(self.progress_index)
        # 새 경로를 처음 받은 순간에도 첫 direction 구간 전체를 검색하면,
        # 공간상 가까이 지나가는 후반 전진점으로 progress가 바로 건너뛸 수
        # 있다. 이후 end guard가 만족되면 실제 순서를 주행하지 않았는데도
        # 다음 후진 구간으로 전환된다. 고정 경로는 현재 차량 pose에서
        # 시작하므로 일반 최근접 검색과 같은 앞쪽 window만 허용한다.
        search_end = min(
            segment_end,
            self.progress_index + self.limits.nearest_forward_window,
        )
        return min(
            range(self.progress_index, search_end + 1),
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

    def _anticipated_curvature(
        self,
        start: int,
        segment_end: int,
        preview_points: int | None = None,
    ) -> float:
        """가까운 미래 곡률을 거리 가중 평균해 코너 선행 조향을 만든다."""
        if preview_points is None:
            preview_points = self.limits.curve_feedforward_preview_points
        stop = min(
            segment_end,
            start + preview_points,
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
        *,
        start_s_m: float | None = None,
        apply_forward_rejoin: bool = False,
        reference_speed_mps: float | None = None,
    ) -> tuple[tuple[ReferencePoint, float], ...]:
        direction = self.path[start].direction
        reference_speed = (
            (
                self.limits.forward_speed_mps
                if direction > 0
                else self.limits.reverse_speed_mps
            )
            if reference_speed_mps is None
            else abs(reference_speed_mps)
        )
        # cmd_vel의 선속도는 rear axle 속도다. 차량 중심 arc length를
        # 시간축으로 쓰면 곡선에서 horizon 위상이 앞서므로, 원래 rear axle
        # 궤적의 arc length로 예측 reference를 배치한다.
        start_s = (
            float(self._rear_arc_length[start])
            if start_s_m is None
            else float(
                np.clip(
                    start_s_m,
                    self._rear_arc_length[self._segment_start(start)],
                    self._rear_arc_length[segment_end],
                )
            )
        )
        segment_start = self._segment_start(start)
        references: list[tuple[ReferencePoint, float]] = []
        for step in range(1, self.limits.horizon_steps + 1):
            target_s = min(
                start_s + reference_speed * self.limits.dt_sec * step,
                self._rear_arc_length[segment_end],
            )
            if (
                apply_forward_rejoin
                and direction > 0
                and self._forward_rejoin_active
            ):
                references.append(
                    self._forward_rejoin_reference(
                        target_s,
                        segment_start,
                        segment_end,
                    )
                )
            else:
                references.append(
                    self._interpolate_reference(
                        target_s,
                        segment_start,
                        segment_end,
                    )
                )
        return tuple(references)

    def _interpolate_reference(
        self,
        target_s: float,
        start: int,
        segment_end: int,
    ) -> tuple[ReferencePoint, float]:
        upper = int(
            np.searchsorted(
                self._rear_arc_length[start : segment_end + 1],
                target_s,
                side="left",
            )
        ) + start
        upper = min(max(upper, start), segment_end)
        if upper == start:
            return self.path[start], float(self._reference_curvature[start])
        lower = upper - 1
        span = self._rear_arc_length[upper] - self._rear_arc_length[lower]
        if span <= 1e-12:
            return self.path[upper], float(self._reference_curvature[upper])
        ratio = (target_s - self._rear_arc_length[lower]) / span
        first, second = self.path[lower], self.path[upper]
        yaw = normalize_angle(
            first.yaw_rad
            + ratio * normalize_angle(second.yaw_rad - first.yaw_rad)
        )
        rear_x = self._rear_points[lower, 0] + ratio * (
            self._rear_points[upper, 0] - self._rear_points[lower, 0]
        )
        rear_y = self._rear_points[lower, 1] + ratio * (
            self._rear_points[upper, 1] - self._rear_points[lower, 1]
        )
        offset = self.limits.control_point_offset_m
        return (
            ReferencePoint(
                x_m=rear_x + offset * math.cos(yaw),
                y_m=rear_y + offset * math.sin(yaw),
                yaw_rad=yaw,
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
        curvature_rate_1pmps: float | None = None,
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
        acceleration_limit = (
            self.limits.forward_max_acceleration_mps2
            if direction > 0
            else self.limits.max_acceleration_mps2
        )
        acceleration_delta = acceleration_limit * self.limits.dt_sec
        deceleration_delta = (
            self.limits.max_acceleration_mps2 * self.limits.dt_sec
        )
        curvature_delta = (
            self.limits.max_curvature_rate_1pmps
            if curvature_rate_1pmps is None
            else curvature_rate_1pmps
        ) * self.limits.dt_sec
        previous_speed = self.last_speed_mps
        previous_curvature = self.last_curvature_1pm
        for index, (_, reference_curvature) in enumerate(references):
            controls[index, 0] = np.clip(
                controls[index, 0],
                previous_speed - deceleration_delta,
                previous_speed + acceleration_delta,
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
        # 후진 주차는 실차에서 검증된 기존 최대 곡률을 그대로 사용한다.
        if direction < 0:
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
        near_curvature = [
            abs(reference_curvature)
            for _, reference_curvature in near_references
        ]
        recent_absolute_curvature = np.abs(recent_curvature)
        peak_curvature = max(
            near_curvature + recent_absolute_curvature.tolist(),
            default=0.0,
        )
        lower = self.limits.straight_curvature_threshold_1pm
        upper = self.limits.full_curvature_path_threshold_1pm
        ratio = float(np.clip((peak_curvature - lower) / (upper - lower), 0.0, 1.0))
        # smoothstep은 양 끝의 기울기가 0이므로 임계값을 넘나들어도 제한값이
        # 갑자기 변하지 않는다.
        ratio = ratio * ratio * (3.0 - 2.0 * ratio)
        return float(
            self.limits.straight_max_curvature_1pm
            + ratio
            * (
                self.limits.max_curvature_1pm
                - self.limits.straight_max_curvature_1pm
            )
        )

    def _curve_ratio_from_limit(self, curvature_limit: float) -> float:
        span = (
            self.limits.max_curvature_1pm
            - self.limits.straight_max_curvature_1pm
        )
        if span <= 1e-12:
            return 1.0
        return float(
            np.clip(
                (curvature_limit - self.limits.straight_max_curvature_1pm)
                / span,
                0.0,
                1.0,
            )
        )

    def _adaptive_curvature_rate(
        self,
        curve_ratio: float,
        speed_mps: float,
        direction: int,
        recovery_ratio: float = 0.0,
    ) -> float:
        """경로 곡률과 속도에 맞춘 시간당 곡률 변화 한계를 반환한다."""
        if direction < 0:
            # 검증된 후진 주차 반응은 유지한다.
            return self.limits.max_curvature_rate_1pmps
        demand_ratio = float(np.clip(max(curve_ratio, recovery_ratio), 0.0, 1.0))
        curvature_rate = (
            self.limits.straight_curvature_rate_1pmps
            + demand_ratio
            * (
                self.limits.max_curvature_rate_1pmps
                - self.limits.straight_curvature_rate_1pmps
            )
        )
        speed_ratio = float(
            np.clip(
                abs(speed_mps) / self.limits.max_forward_speed_mps,
                0.0,
                1.0,
            )
        )
        return curvature_rate * (
            1.0
            - self.limits.curvature_rate_speed_reduction * speed_ratio
        )

    def _heading_recovery_ratio(self, yaw_error_rad: float) -> float:
        """복귀 시작각부터 최대 복귀각까지 0~1로 부드럽게 증가한다."""
        lower = self.limits.max_tracking_yaw_error_rad
        upper = self.limits.heading_recovery_full_curvature_error_rad
        ratio = float(
            np.clip((abs(yaw_error_rad) - lower) / (upper - lower), 0.0, 1.0)
        )
        return ratio * ratio * (3.0 - 2.0 * ratio)

    def _speed_curvature_limit(
        self,
        base_limit_1pm: float,
        speed_mps: float,
    ) -> float:
        """각속도와 등가 횡가속도를 만족하는 속도별 곡률 한계다."""
        speed = abs(speed_mps)
        if speed <= 1e-9:
            return min(base_limit_1pm, self.limits.max_curvature_1pm)
        return min(
            base_limit_1pm,
            self.limits.max_curvature_1pm,
            self.limits.max_angular_speed_radps / speed,
            self.limits.max_lateral_acceleration_mps2 / (speed * speed),
        )

    def _speed_limit_for_curvature(
        self,
        base_limit_mps: float,
        curvature_1pm: float,
    ) -> float:
        """필요 곡률을 유지할 수 있는 각속도·횡가속도 기반 속도 한계다."""
        curvature = abs(curvature_1pm)
        if curvature <= 1e-9:
            return base_limit_mps
        return min(
            base_limit_mps,
            self.limits.max_angular_speed_radps / curvature,
            math.sqrt(self.limits.max_lateral_acceleration_mps2 / curvature),
        )

    def _tracking_speed_scale(self, absolute_cross_track_error_m: float) -> float:
        """횡오차에 따른 연속 감속 비율을 반환한다."""
        if absolute_cross_track_error_m <= self.limits.cross_track_slowdown_start_m:
            return 1.0
        slowdown_ratio = float(
            np.clip(
                (
                    absolute_cross_track_error_m
                    - self.limits.cross_track_slowdown_start_m
                )
                / (
                    self.limits.cross_track_slowdown_full_m
                    - self.limits.cross_track_slowdown_start_m
                ),
                0.0,
                1.0,
            )
        )
        return 1.0 - slowdown_ratio * (
            1.0 - self.limits.minimum_tracking_speed_scale
        )

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

    def _rate_constraint(
        self,
        direction: int,
        curvature_rate_1pmps: float | None = None,
    ) -> LinearConstraint:
        horizon = self.limits.horizon_steps
        matrix = np.zeros((2 * horizon, 2 * horizon), dtype=np.float64)
        lower = np.empty(2 * horizon, dtype=np.float64)
        upper = np.empty(2 * horizon, dtype=np.float64)
        acceleration_limit = (
            self.limits.forward_max_acceleration_mps2
            if direction > 0
            else self.limits.max_acceleration_mps2
        )
        acceleration_delta = acceleration_limit * self.limits.dt_sec
        deceleration_delta = (
            self.limits.max_acceleration_mps2 * self.limits.dt_sec
        )
        curvature_delta = (
            self.limits.max_curvature_rate_1pmps
            if curvature_rate_1pmps is None
            else curvature_rate_1pmps
        ) * self.limits.dt_sec
        for step in range(horizon):
            speed_row = 2 * step
            curvature_row = speed_row + 1
            matrix[speed_row, 2 * step] = 1.0
            matrix[curvature_row, 2 * step + 1] = 1.0
            if step == 0:
                lower[speed_row] = (
                    self.last_speed_mps - deceleration_delta
                )
                upper[speed_row] = (
                    self.last_speed_mps + acceleration_delta
                )
                lower[curvature_row] = self.last_curvature_1pm - curvature_delta
                upper[curvature_row] = self.last_curvature_1pm + curvature_delta
            else:
                matrix[speed_row, 2 * (step - 1)] = -1.0
                matrix[curvature_row, 2 * (step - 1) + 1] = -1.0
                lower[speed_row] = -deceleration_delta
                upper[speed_row] = acceleration_delta
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
        reference_speed_mps: float | None = None,
    ) -> float:
        controls = np.asarray(flat_controls, dtype=np.float64).reshape(-1, 2)
        offset = self.limits.control_point_offset_m
        yaw = state.yaw_rad
        # 운동 방정식은 cmd_vel의 기준점인 rear axle에서 적분한다. 다만
        # 전진 reference는 중앙 카메라가 측정하고 경로도 정의하는 차량 중심과
        # 비교한다. 후진은 실차 주차 결과가 검증된 기존 rear 기준을 유지한다.
        x = state.x_m - offset * math.cos(yaw)
        y = state.y_m - offset * math.sin(yaw)
        reference_speed = (
            (
                self.limits.forward_speed_mps
                if direction > 0
                else -self.limits.reverse_speed_mps
            )
            if reference_speed_mps is None
            else math.copysign(abs(reference_speed_mps), direction)
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
            if direction > 0:
                control_x = x + offset * math.cos(yaw)
                control_y = y + offset * math.sin(yaw)
                reference_x = reference.x_m
                reference_y = reference.y_m
            else:
                control_x = x
                control_y = y
                reference_x = reference.x_m - offset * math.cos(reference.yaw_rad)
                reference_y = reference.y_m - offset * math.sin(reference.yaw_rad)
            error_x = control_x - reference_x
            error_y = control_y - reference_y
            target_yaw = reference.yaw_rad
            yaw_error = normalize_angle(yaw - target_yaw)
            if direction > 0:
                # 추가 feedback만 deadband 처리하면 optimizer가 mm 단위
                # camera 흔들림을 계속 쫓는다. 전진 위치 비용도 경로 접선
                # 방향으로 분해해 작은 횡오차만 제거하고, 종방향 진행 비용과
                # 실제 곡선 reference는 그대로 유지한다.
                longitudinal_error = (
                    math.cos(reference.yaw_rad) * error_x
                    + math.sin(reference.yaw_rad) * error_y
                )
                lateral_error = (
                    -math.sin(reference.yaw_rad) * error_x
                    + math.cos(reference.yaw_rad) * error_y
                )
                forward_deadband_scale = 1.0
                effective_lateral_error = math.copysign(
                    max(
                        0.0,
                        abs(lateral_error)
                        - self.limits.cross_track_deadband_m
                        * forward_deadband_scale,
                    ),
                    lateral_error,
                )
                if self._forward_rejoin_active:
                    # virtual reference는 이미 실제 합류 곡선 위의 차량 중심이다.
                    # body yaw를 중심 궤적 접선처럼 사용해 다시 횡분해하지 않는다.
                    position_error = error_x**2 + error_y**2
                else:
                    position_error = (
                        longitudinal_error**2 + effective_lateral_error**2
                    )
                yaw_error = math.copysign(
                    max(
                        0.0,
                        abs(yaw_error)
                        - self.limits.heading_feedback_deadband_rad
                        * forward_deadband_scale,
                    ),
                    yaw_error,
                )
            else:
                position_error = error_x**2 + error_y**2
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
