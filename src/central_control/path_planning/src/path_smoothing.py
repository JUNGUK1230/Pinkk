"""기어 전환 pose를 보존하는 clamped cubic path smoothing."""

from dataclasses import dataclass
import math
from typing import Protocol, Sequence


class PathPose(Protocol):
    """Smoothing 입력에 필요한 pose 속성."""

    x_cm: float
    y_cm: float
    yaw_rad: float
    direction: int
    steer_rad: float


@dataclass(frozen=True)
class SmoothedPathPose:
    """Spline 미분값에서 yaw와 가상 조향각을 계산한 pose."""

    x_cm: float
    y_cm: float
    yaw_rad: float
    direction: int
    steer_rad: float


@dataclass(frozen=True)
class _Spline1D:
    parameters: tuple[float, ...]
    values: tuple[float, ...]
    second_derivatives: tuple[float, ...]

    def evaluate(self, segment: int, parameter: float) -> tuple[float, float, float]:
        """한 구간의 값, 1차 미분, 2차 미분을 반환한다."""
        left = self.parameters[segment]
        right = self.parameters[segment + 1]
        interval = right - left
        a = (right - parameter) / interval
        b = (parameter - left) / interval
        left_value = self.values[segment]
        right_value = self.values[segment + 1]
        left_second = self.second_derivatives[segment]
        right_second = self.second_derivatives[segment + 1]

        value = (
            a * left_value
            + b * right_value
            + ((a**3 - a) * left_second + (b**3 - b) * right_second)
            * interval**2
            / 6.0
        )
        first = (
            (right_value - left_value) / interval
            + interval
            * (-(3.0 * a * a - 1.0) * left_second + (3.0 * b * b - 1.0) * right_second)
            / 6.0
        )
        second = a * left_second + b * right_second
        return value, first, second


def smooth_hybrid_path(
    poses: Sequence[PathPose],
    wheelbase_cm: float,
    output_step_cm: float = 0.5,
    knot_spacing_cm: float = 3.0,
) -> list[SmoothedPathPose]:
    """기어별 C2 spline을 생성하고 최대 output 간격 이하로 샘플링한다.

    전진↔후진 cusp는 두 spline의 공통 endpoint로 고정한다. 각 구간의
    시작·끝 미분도 입력 yaw로 고정하므로 전체 start/goal pose와 cusp
    heading을 유지한다.
    """
    if wheelbase_cm <= 0.0 or output_step_cm <= 0.0 or knot_spacing_cm <= 0.0:
        raise ValueError("wheelbase, output step, and knot spacing must be positive")
    if not poses:
        return []
    if len(poses) == 1:
        pose = poses[0]
        return [
            SmoothedPathPose(
                pose.x_cm,
                pose.y_cm,
                _normalize_yaw(pose.yaw_rad),
                pose.direction,
                0.0,
            )
        ]

    output: list[SmoothedPathPose] = []
    for block, direction in _split_direction_blocks(poses):
        block_output = _smooth_block(
            block,
            direction,
            wheelbase_cm,
            output_step_cm,
            knot_spacing_cm,
        )
        if output and block_output:
            block_output = block_output[1:]
        output.extend(block_output)
    return output


def _smooth_block(
    poses: Sequence[PathPose],
    direction: int,
    wheelbase_cm: float,
    output_step_cm: float,
    knot_spacing_cm: float,
) -> list[SmoothedPathPose]:
    raw_points = [(pose.x_cm, pose.y_cm) for pose in poses]
    raw_parameters = _cumulative_distances(raw_points)
    if raw_parameters[-1] <= 1e-9:
        pose = poses[0]
        return [
            SmoothedPathPose(
                pose.x_cm,
                pose.y_cm,
                _normalize_yaw(pose.yaw_rad),
                direction,
                0.0,
            )
        ]

    transition_distances = [
        raw_parameters[index]
        for index in range(1, len(poses))
        if abs(poses[index].steer_rad - poses[index - 1].steer_rad)
        > math.radians(1.0)
    ]
    knot_indices = _select_knot_indices(
        raw_parameters,
        knot_spacing_cm,
        transition_distances,
    )
    points = [raw_points[index] for index in knot_indices]
    parameters = _cumulative_distances(points)
    start_dx, start_dy = _heading_tangent(poses[0].yaw_rad, direction)
    end_dx, end_dy = _heading_tangent(poses[-1].yaw_rad, direction)
    x_spline = _build_clamped_spline(
        parameters,
        [point[0] for point in points],
        start_dx,
        end_dx,
    )
    y_spline = _build_clamped_spline(
        parameters,
        [point[1] for point in points],
        start_dy,
        end_dy,
    )

    output: list[SmoothedPathPose] = []
    for segment in range(len(points) - 1):
        interval = parameters[segment + 1] - parameters[segment]
        estimated_length = _estimate_arc_length(x_spline, y_spline, segment)
        # 10% oversampling margin keeps Euclidean gaps below the requested step
        # even where spline speed is not uniform in the chord-length parameter.
        sample_count = max(
            1,
            math.ceil(estimated_length / (0.9 * output_step_cm)),
        )
        first_sample = 0 if segment == 0 else 1
        for sample_index in range(first_sample, sample_count + 1):
            parameter = parameters[segment] + interval * sample_index / sample_count
            output.append(
                _evaluate_pose(
                    x_spline,
                    y_spline,
                    segment,
                    parameter,
                    direction,
                    wheelbase_cm,
                )
            )

    last = output[-1]
    output[-1] = SmoothedPathPose(
        poses[-1].x_cm,
        poses[-1].y_cm,
        _normalize_yaw(poses[-1].yaw_rad),
        direction,
        last.steer_rad,
    )
    return output


def _build_clamped_spline(
    parameters: Sequence[float],
    values: Sequence[float],
    start_derivative: float,
    end_derivative: float,
) -> _Spline1D:
    """지정된 양 끝 미분을 만족하는 cubic spline을 계산한다."""
    count = len(values)
    if count < 2:
        raise ValueError("a spline requires at least two points")
    lower = [0.0] * count
    diagonal = [0.0] * count
    upper = [0.0] * count
    right_hand = [0.0] * count
    intervals = [
        parameters[index + 1] - parameters[index]
        for index in range(count - 1)
    ]
    if any(interval <= 1e-9 for interval in intervals):
        raise ValueError("spline knots must have distinct positions")

    diagonal[0] = 2.0 * intervals[0]
    upper[0] = intervals[0]
    right_hand[0] = 6.0 * (
        (values[1] - values[0]) / intervals[0] - start_derivative
    )
    for index in range(1, count - 1):
        previous_interval = intervals[index - 1]
        next_interval = intervals[index]
        lower[index] = previous_interval
        diagonal[index] = 2.0 * (previous_interval + next_interval)
        upper[index] = next_interval
        right_hand[index] = 6.0 * (
            (values[index + 1] - values[index]) / next_interval
            - (values[index] - values[index - 1]) / previous_interval
        )
    lower[-1] = intervals[-1]
    diagonal[-1] = 2.0 * intervals[-1]
    right_hand[-1] = 6.0 * (
        end_derivative
        - (values[-1] - values[-2]) / intervals[-1]
    )

    for index in range(1, count):
        factor = lower[index] / diagonal[index - 1]
        diagonal[index] -= factor * upper[index - 1]
        right_hand[index] -= factor * right_hand[index - 1]
    second = [0.0] * count
    second[-1] = right_hand[-1] / diagonal[-1]
    for index in range(count - 2, -1, -1):
        second[index] = (
            right_hand[index] - upper[index] * second[index + 1]
        ) / diagonal[index]
    return _Spline1D(tuple(parameters), tuple(values), tuple(second))


def _evaluate_pose(
    x_spline: _Spline1D,
    y_spline: _Spline1D,
    segment: int,
    parameter: float,
    direction: int,
    wheelbase_cm: float,
) -> SmoothedPathPose:
    x, dx, ddx = x_spline.evaluate(segment, parameter)
    y, dy, ddy = y_spline.evaluate(segment, parameter)
    speed_squared = dx * dx + dy * dy
    if speed_squared <= 1e-16:
        raise ValueError("smoothed path contains a zero tangent")
    tangent_yaw = math.atan2(dy, dx)
    yaw = tangent_yaw if direction > 0 else tangent_yaw + math.pi
    geometric_curvature = (dx * ddy - dy * ddx) / (speed_squared**1.5)
    vehicle_curvature = direction * geometric_curvature
    return SmoothedPathPose(
        x,
        y,
        _normalize_yaw(yaw),
        direction,
        math.atan(wheelbase_cm * vehicle_curvature),
    )


def _estimate_arc_length(
    x_spline: _Spline1D,
    y_spline: _Spline1D,
    segment: int,
) -> float:
    left = x_spline.parameters[segment]
    right = x_spline.parameters[segment + 1]
    previous_x, _, _ = x_spline.evaluate(segment, left)
    previous_y, _, _ = y_spline.evaluate(segment, left)
    length = 0.0
    for index in range(1, 17):
        parameter = left + (right - left) * index / 16.0
        x, _, _ = x_spline.evaluate(segment, parameter)
        y, _, _ = y_spline.evaluate(segment, parameter)
        length += math.hypot(x - previous_x, y - previous_y)
        previous_x, previous_y = x, y
    return length


def _split_direction_blocks(
    poses: Sequence[PathPose],
) -> list[tuple[Sequence[PathPose], int]]:
    blocks: list[tuple[Sequence[PathPose], int]] = []
    block_start = 0
    direction = poses[1].direction
    if direction not in (-1, 1):
        raise ValueError("path direction must be -1 or 1")
    for index in range(2, len(poses)):
        next_direction = poses[index].direction
        if next_direction not in (-1, 1):
            raise ValueError("path direction must be -1 or 1")
        if next_direction != direction:
            blocks.append((poses[block_start:index], direction))
            block_start = index - 1
            direction = next_direction
    blocks.append((poses[block_start:], direction))
    return blocks


def _select_knot_indices(
    cumulative: Sequence[float],
    minimum_spacing_cm: float,
    transition_distances: Sequence[float],
) -> list[int]:
    indices = [0]
    last_distance = cumulative[0]
    for index in range(1, len(cumulative) - 1):
        if any(
            abs(cumulative[index] - transition) < 1.5 * minimum_spacing_cm
            for transition in transition_distances
        ):
            continue
        if cumulative[index] - last_distance >= minimum_spacing_cm:
            indices.append(index)
            last_distance = cumulative[index]
    if indices[-1] != len(cumulative) - 1:
        indices.append(len(cumulative) - 1)
    return indices


def _heading_tangent(yaw_rad: float, direction: int) -> tuple[float, float]:
    return direction * math.cos(yaw_rad), direction * math.sin(yaw_rad)


def _cumulative_distances(
    points: Sequence[tuple[float, float]],
) -> list[float]:
    cumulative = [0.0]
    for first, second in zip(points, points[1:]):
        cumulative.append(
            cumulative[-1]
            + math.hypot(second[0] - first[0], second[1] - first[1])
        )
    return cumulative


def _normalize_yaw(yaw_rad: float) -> float:
    return (yaw_rad + math.pi) % (2.0 * math.pi) - math.pi
