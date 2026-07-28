"""MoveIt IK DRY RUN에 사용하는 순수 계산과 결과 검증."""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from ..geometry.transforms import validate_transform


@dataclass(frozen=True)
class FkPoseError:
    """요청 자세와 FK 자세 사이의 오차."""

    position_error_m: float
    z_error_m: float
    orientation_error_deg: float


@dataclass(frozen=True)
class MotionGuardMeasurement:
    """이동 시작 자세를 기준으로 측정한 실시간 안전 상태."""

    delta_x_m: float
    delta_y_m: float
    delta_z_m: float
    xy_distance_m: float
    direction_progress_m: float
    orientation_change_deg: float


def make_locked_xy_target(
    current_base_to_flange: np.ndarray,
    axis: str,
    distance_m: float,
    maximum_distance_m: float,
) -> np.ndarray:
    """현재 Z와 전체 자세를 유지하고 base X 또는 Y만 이동한 목표를 만든다."""
    current = validate_transform(current_base_to_flange)
    normalized_axis = axis.strip().lower()
    if normalized_axis not in {'x', 'y'}:
        raise ValueError("axis는 'x' 또는 'y'여야 합니다")
    if not math.isfinite(distance_m) or abs(distance_m) < 1e-9:
        raise ValueError('이동 거리는 0이 아닌 유한값이어야 합니다')
    if maximum_distance_m <= 0.0:
        raise ValueError('최대 이동 제한은 0보다 커야 합니다')
    if abs(distance_m) > maximum_distance_m + 1e-12:
        raise ValueError(
            f'요청 이동 {abs(distance_m) * 1000.0:.3f}mm가 '
            f'제한 {maximum_distance_m * 1000.0:.3f}mm를 초과합니다'
        )

    target = current.copy()
    target[0 if normalized_axis == 'x' else 1, 3] += float(distance_m)
    return validate_transform(target)


def pose_error(
    expected_base_to_flange: np.ndarray,
    actual_base_to_flange: np.ndarray,
) -> FkPoseError:
    """기대 자세와 실제/FK 자세의 위치·Z·회전 오차를 계산한다."""
    expected = validate_transform(expected_base_to_flange)
    actual = validate_transform(actual_base_to_flange)
    translation_error = actual[:3, 3] - expected[:3, 3]
    relative_rotation = expected[:3, :3].T @ actual[:3, :3]
    cosine = float(
        np.clip((np.trace(relative_rotation) - 1.0) * 0.5, -1.0, 1.0)
    )
    return FkPoseError(
        position_error_m=float(np.linalg.norm(translation_error)),
        z_error_m=float(translation_error[2]),
        orientation_error_deg=math.degrees(math.acos(cosine)),
    )


def validate_fk_pose(
    expected_base_to_flange: np.ndarray,
    actual_base_to_flange: np.ndarray,
    maximum_position_error_m: float,
    maximum_z_error_m: float,
    maximum_orientation_error_deg: float,
) -> FkPoseError:
    """IK 관절값을 FK로 되돌린 결과가 요청 자세와 일치하는지 검사한다."""
    error = pose_error(expected_base_to_flange, actual_base_to_flange)
    if error.position_error_m > maximum_position_error_m:
        raise ValueError(
            f'FK 위치 오차 {error.position_error_m * 1000.0:.3f}mm가 '
            f'허용값 {maximum_position_error_m * 1000.0:.3f}mm를 초과합니다'
        )
    if abs(error.z_error_m) > maximum_z_error_m:
        raise ValueError(
            f'FK Z 오차 {error.z_error_m * 1000.0:+.3f}mm가 '
            f'허용값 {maximum_z_error_m * 1000.0:.3f}mm를 초과합니다'
        )
    if error.orientation_error_deg > maximum_orientation_error_deg:
        raise ValueError(
            f'FK 자세 오차 {error.orientation_error_deg:.4f}deg가 '
            f'허용값 {maximum_orientation_error_deg:.4f}deg를 초과합니다'
        )
    return error


def validate_motion_guard(
    start_base_to_flange: np.ndarray,
    target_base_to_flange: np.ndarray,
    actual_base_to_flange: np.ndarray,
    maximum_z_change_m: float,
    maximum_orientation_change_deg: float,
    maximum_xy_overshoot_m: float,
    maximum_opposite_progress_m: float,
) -> MotionGuardMeasurement:
    """이동 중 Z·자세·XY 과주행·반대 방향 이동을 검사한다."""
    start = validate_transform(start_base_to_flange)
    target = validate_transform(target_base_to_flange)
    actual = validate_transform(actual_base_to_flange)
    planned_xy = target[:2, 3] - start[:2, 3]
    actual_delta = actual[:3, 3] - start[:3, 3]
    planned_distance = float(np.linalg.norm(planned_xy))
    direction_progress = (
        0.0
        if planned_distance < 1e-9
        else float(np.dot(actual_delta[:2], planned_xy) / planned_distance)
    )
    orientation_change = pose_error(start, actual).orientation_error_deg
    measurement = MotionGuardMeasurement(
        delta_x_m=float(actual_delta[0]),
        delta_y_m=float(actual_delta[1]),
        delta_z_m=float(actual_delta[2]),
        xy_distance_m=float(np.linalg.norm(actual_delta[:2])),
        direction_progress_m=direction_progress,
        orientation_change_deg=orientation_change,
    )
    if abs(measurement.delta_z_m) > maximum_z_change_m:
        raise ValueError(
            f'이동 중 Z 변화 {measurement.delta_z_m * 1000.0:+.3f}mm가 '
            f'제한 {maximum_z_change_m * 1000.0:.3f}mm를 초과합니다'
        )
    if measurement.orientation_change_deg > maximum_orientation_change_deg:
        raise ValueError(
            f'이동 중 자세 변화 {measurement.orientation_change_deg:.3f}deg가 '
            f'제한 {maximum_orientation_change_deg:.3f}deg를 초과합니다'
        )
    if measurement.xy_distance_m > planned_distance + maximum_xy_overshoot_m:
        maximum_xy_distance_mm = (
            planned_distance + maximum_xy_overshoot_m
        ) * 1000.0
        raise ValueError(
            f'이동 중 XY 거리 {measurement.xy_distance_m * 1000.0:.3f}mm가 '
            f'계획+여유 {maximum_xy_distance_mm:.3f}mm를 '
            '초과합니다'
        )
    if (
        planned_distance >= 1e-9
        and measurement.direction_progress_m < -maximum_opposite_progress_m
    ):
        raise ValueError(
            '이동 중 요청 반대 방향 진행량이 '
            f'{measurement.direction_progress_m * 1000.0:+.3f}mm입니다'
        )
    return measurement
