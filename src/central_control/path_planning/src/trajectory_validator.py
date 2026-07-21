"""제어기 전달 직전 trajectory를 검사하는 fail-closed 안전 게이트."""

from dataclasses import dataclass
import math
from typing import Callable, Sequence

try:
    from .trajectory_profile import TrajectoryPoint
except ImportError:
    from trajectory_profile import TrajectoryPoint


CollisionChecker = Callable[[float, float, float], bool]


@dataclass(frozen=True)
class TrajectoryValidationLimits:
    """실차와 경로 출력 설정에서 정해지는 trajectory 제한값."""

    wheelbase_cm: float = 8.0
    max_spacing_cm: float = 0.5
    max_steer_rad: float = math.radians(30.0)
    max_steer_change_rad_per_cm: float = math.radians(10.0) / 3.0
    steer_change_tolerance_rad: float = math.radians(0.1)
    max_forward_speed_mps: float = 0.05
    max_reverse_speed_mps: float = 0.03
    max_angular_speed_radps: float = 0.5
    max_acceleration_mps2: float = 0.05
    max_deceleration_mps2: float = 0.05
    max_yaw_change_rad: float = math.radians(5.0)
    max_motion_heading_error_rad: float = math.radians(5.0)
    tolerance: float = 1e-6

    def validate(self) -> None:
        positive_values = {
            "wheelbase_cm": self.wheelbase_cm,
            "max_spacing_cm": self.max_spacing_cm,
            "max_steer_rad": self.max_steer_rad,
            "max_steer_change_rad_per_cm": self.max_steer_change_rad_per_cm,
            "max_forward_speed_mps": self.max_forward_speed_mps,
            "max_reverse_speed_mps": self.max_reverse_speed_mps,
            "max_angular_speed_radps": self.max_angular_speed_radps,
            "max_acceleration_mps2": self.max_acceleration_mps2,
            "max_deceleration_mps2": self.max_deceleration_mps2,
            "max_yaw_change_rad": self.max_yaw_change_rad,
            "max_motion_heading_error_rad": self.max_motion_heading_error_rad,
        }
        for name, value in positive_values.items():
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
        if not math.isfinite(self.tolerance) or self.tolerance < 0.0:
            raise ValueError("tolerance must be finite and non-negative")
        if (
            not math.isfinite(self.steer_change_tolerance_rad)
            or self.steer_change_tolerance_rad < 0.0
        ):
            raise ValueError(
                "steer_change_tolerance_rad must be finite and non-negative"
            )


@dataclass(frozen=True)
class TrajectoryValidationIssue:
    """제어기 전송을 차단한 한 가지 오류."""

    code: str
    message: str
    index: int | None = None


@dataclass(frozen=True)
class TrajectoryValidationMetrics:
    """통과 결과를 터미널에서 확인하기 위한 핵심 최대값."""

    point_count: int
    path_length_cm: float
    max_spacing_cm: float
    max_abs_steer_deg: float
    max_abs_speed_mps: float
    max_abs_angular_speed_radps: float
    gear_switch_count: int


@dataclass(frozen=True)
class TrajectoryValidationResult:
    """valid가 False이면 trajectory를 저장·발행하지 않아야 한다."""

    valid: bool
    issues: tuple[TrajectoryValidationIssue, ...]
    metrics: TrajectoryValidationMetrics


def validate_trajectory(
    trajectory: Sequence[TrajectoryPoint],
    limits: TrajectoryValidationLimits,
    collision_checker: CollisionChecker | None = None,
) -> TrajectoryValidationResult:
    """수치·기구학·속도·정지·충돌 조건을 모두 검사한다."""
    limits.validate()
    issues: list[TrajectoryValidationIssue] = []
    if not trajectory:
        issues.append(
            TrajectoryValidationIssue("EMPTY_TRAJECTORY", "trajectory is empty")
        )
        return TrajectoryValidationResult(
            False,
            tuple(issues),
            TrajectoryValidationMetrics(0, 0.0, 0.0, 0.0, 0.0, 0.0, 0),
        )

    finite_fields = (
        "x_cm",
        "y_cm",
        "yaw_rad",
        "steer_rad",
        "curvature_1pm",
        "target_speed_mps",
        "target_angular_z_radps",
    )
    wheelbase_m = limits.wheelbase_cm / 100.0
    tolerance = limits.tolerance

    for index, point in enumerate(trajectory):
        for field in finite_fields:
            if not math.isfinite(getattr(point, field)):
                issues.append(
                    TrajectoryValidationIssue(
                        "NON_FINITE_VALUE",
                        f"{field} is not finite",
                        index,
                    )
                )
        if point.direction not in (-1, 1):
            issues.append(
                TrajectoryValidationIssue(
                    "INVALID_DIRECTION",
                    f"direction must be -1 or 1, got {point.direction}",
                    index,
                )
            )
            continue
        if not all(math.isfinite(getattr(point, field)) for field in finite_fields):
            continue

        if abs(point.steer_rad) > limits.max_steer_rad + tolerance:
            issues.append(
                TrajectoryValidationIssue(
                    "STEERING_LIMIT",
                    f"steering {math.degrees(point.steer_rad):.3f} deg exceeds limit",
                    index,
                )
            )
        expected_curvature = math.tan(point.steer_rad) / wheelbase_m
        if not math.isclose(
            point.curvature_1pm,
            expected_curvature,
            rel_tol=1e-6,
            abs_tol=tolerance,
        ):
            issues.append(
                TrajectoryValidationIssue(
                    "CURVATURE_MISMATCH",
                    "curvature does not match steering and wheelbase",
                    index,
                )
            )
        expected_angular_speed = point.target_speed_mps * point.curvature_1pm
        if not math.isclose(
            point.target_angular_z_radps,
            expected_angular_speed,
            rel_tol=1e-6,
            abs_tol=tolerance,
        ):
            issues.append(
                TrajectoryValidationIssue(
                    "ANGULAR_SPEED_MISMATCH",
                    "angular speed does not equal speed multiplied by curvature",
                    index,
                )
            )

        if point.direction > 0:
            if point.target_speed_mps < -tolerance:
                issues.append(
                    TrajectoryValidationIssue(
                        "SPEED_DIRECTION_MISMATCH",
                        "forward point has negative target speed",
                        index,
                    )
                )
            speed_limit = limits.max_forward_speed_mps
        else:
            if point.target_speed_mps > tolerance:
                issues.append(
                    TrajectoryValidationIssue(
                        "SPEED_DIRECTION_MISMATCH",
                        "reverse point has positive target speed",
                        index,
                    )
                )
            speed_limit = limits.max_reverse_speed_mps
        if abs(point.target_speed_mps) > speed_limit + tolerance:
            issues.append(
                TrajectoryValidationIssue(
                    "SPEED_LIMIT",
                    f"speed {point.target_speed_mps:.4f} m/s exceeds direction limit",
                    index,
                )
            )
        if (
            abs(point.target_angular_z_radps)
            > limits.max_angular_speed_radps + tolerance
        ):
            issues.append(
                TrajectoryValidationIssue(
                    "ANGULAR_SPEED_LIMIT",
                    f"angular speed {point.target_angular_z_radps:.4f} rad/s exceeds limit",
                    index,
                )
            )
        if point.stop_required and abs(point.target_speed_mps) > tolerance:
            issues.append(
                TrajectoryValidationIssue(
                    "STOP_WITH_NONZERO_SPEED",
                    "stop_required point must have zero target speed",
                    index,
                )
            )
        if collision_checker is not None and collision_checker(
            point.x_cm,
            point.y_cm,
            point.yaw_rad,
        ):
            issues.append(
                TrajectoryValidationIssue(
                    "FOOTPRINT_COLLISION",
                    "vehicle footprint collides with the map",
                    index,
                )
            )

    _validate_required_stops(trajectory, tolerance, issues)
    spacings = _validate_edges(trajectory, limits, issues)
    metrics = TrajectoryValidationMetrics(
        point_count=len(trajectory),
        path_length_cm=sum(spacings),
        max_spacing_cm=max(spacings, default=0.0),
        max_abs_steer_deg=max(
            (abs(math.degrees(point.steer_rad)) for point in trajectory),
            default=0.0,
        ),
        max_abs_speed_mps=max(
            (abs(point.target_speed_mps) for point in trajectory),
            default=0.0,
        ),
        max_abs_angular_speed_radps=max(
            (abs(point.target_angular_z_radps) for point in trajectory),
            default=0.0,
        ),
        gear_switch_count=sum(
            first.direction != second.direction
            for first, second in zip(trajectory, trajectory[1:])
        ),
    )
    return TrajectoryValidationResult(not issues, tuple(issues), metrics)


def _validate_required_stops(
    trajectory: Sequence[TrajectoryPoint],
    tolerance: float,
    issues: list[TrajectoryValidationIssue],
) -> None:
    required_indices = {0, len(trajectory) - 1}
    required_indices.update(
        index
        for index in range(len(trajectory) - 1)
        if trajectory[index].direction != trajectory[index + 1].direction
    )
    for index in sorted(required_indices):
        point = trajectory[index]
        if not point.stop_required or abs(point.target_speed_mps) > tolerance:
            issues.append(
                TrajectoryValidationIssue(
                    "MISSING_REQUIRED_STOP",
                    "start, goal, and pre-gear-switch points must stop",
                    index,
                )
            )


def _validate_edges(
    trajectory: Sequence[TrajectoryPoint],
    limits: TrajectoryValidationLimits,
    issues: list[TrajectoryValidationIssue],
) -> list[float]:
    spacings: list[float] = []
    for index, (first, second) in enumerate(zip(trajectory, trajectory[1:])):
        if not all(
            math.isfinite(value)
            for value in (
                first.x_cm,
                first.y_cm,
                first.yaw_rad,
                first.steer_rad,
                first.target_speed_mps,
                second.x_cm,
                second.y_cm,
                second.yaw_rad,
                second.steer_rad,
                second.target_speed_mps,
            )
        ):
            spacings.append(math.inf)
            continue
        dx = second.x_cm - first.x_cm
        dy = second.y_cm - first.y_cm
        spacing_cm = math.hypot(dx, dy)
        spacings.append(spacing_cm)
        if spacing_cm > limits.max_spacing_cm + limits.tolerance:
            issues.append(
                TrajectoryValidationIssue(
                    "SPACING_LIMIT",
                    f"spacing {spacing_cm:.4f} cm exceeds limit",
                    index + 1,
                )
            )

        yaw_change = abs(_angle_difference(second.yaw_rad, first.yaw_rad))
        if yaw_change > limits.max_yaw_change_rad + limits.tolerance:
            issues.append(
                TrajectoryValidationIssue(
                    "YAW_JUMP",
                    f"yaw change {math.degrees(yaw_change):.3f} deg exceeds limit",
                    index + 1,
                )
            )

        if spacing_cm > limits.tolerance:
            motion_heading = math.atan2(dy, dx)
            travel_direction = second.direction
            expected_heading = (
                motion_heading
                if travel_direction > 0
                else _normalize_yaw(motion_heading + math.pi)
            )
            midpoint_yaw = _normalize_yaw(
                first.yaw_rad
                + 0.5 * _angle_difference(second.yaw_rad, first.yaw_rad)
            )
            heading_error = abs(_angle_difference(expected_heading, midpoint_yaw))
            if heading_error > limits.max_motion_heading_error_rad + limits.tolerance:
                issues.append(
                    TrajectoryValidationIssue(
                        "MOTION_HEADING_MISMATCH",
                        f"motion/heading error {math.degrees(heading_error):.3f} deg",
                        index + 1,
                    )
                )

        if first.direction == second.direction:
            allowed_steer_change = (
                limits.max_steer_change_rad_per_cm * spacing_cm
                + limits.steer_change_tolerance_rad
                + limits.tolerance
            )
            if abs(second.steer_rad - first.steer_rad) > allowed_steer_change:
                issues.append(
                    TrajectoryValidationIssue(
                        "STEERING_RATE_LIMIT",
                        "steering change exceeds distance-based rate limit",
                        index + 1,
                    )
                )

        distance_m = spacing_cm / 100.0
        first_speed = abs(first.target_speed_mps)
        second_speed = abs(second.target_speed_mps)
        if second_speed**2 > (
            first_speed**2
            + 2.0 * limits.max_acceleration_mps2 * distance_m
            + limits.tolerance
        ):
            issues.append(
                TrajectoryValidationIssue(
                    "ACCELERATION_LIMIT",
                    "target speed increase exceeds acceleration limit",
                    index + 1,
                )
            )
        if first_speed**2 > (
            second_speed**2
            + 2.0 * limits.max_deceleration_mps2 * distance_m
            + limits.tolerance
        ):
            issues.append(
                TrajectoryValidationIssue(
                    "DECELERATION_LIMIT",
                    "target speed decrease exceeds deceleration limit",
                    index,
                )
            )
    return spacings


def _normalize_yaw(yaw_rad: float) -> float:
    return (yaw_rad + math.pi) % (2.0 * math.pi) - math.pi


def _angle_difference(first: float, second: float) -> float:
    return _normalize_yaw(first - second)
