"""PBVS 단발 이동 목표의 기하학적 안전 조건을 검사한다."""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from ..geometry.transforms import validate_transform


@dataclass(frozen=True)
class PbvsStepValidation:
    """검증을 통과한 PBVS XY 이동량."""

    delta_x_m: float
    delta_y_m: float
    xy_distance_m: float


def make_fixed_z_xy_waypoints(
    current_base_to_flange: np.ndarray,
    target_base_to_flange: np.ndarray,
    maximum_waypoint_spacing_m: float,
) -> list[np.ndarray]:
    """현재 Z를 고정하고 XY와 작은 자세 변화를 보간한 waypoint를 생성한다."""
    if maximum_waypoint_spacing_m <= 0.0:
        raise ValueError('waypoint 간격은 0보다 커야 합니다')
    current = validate_transform(current_base_to_flange)
    target = validate_transform(target_base_to_flange)
    distance = float(np.linalg.norm(target[:2, 3] - current[:2, 3]))
    # 1 mm를 meter로 더한 결과처럼 부동소수점 오차로 비율이
    # 1.0000000000000009가 되어 불필요한 waypoint가 생기는 것을 막는다.
    count = max(
        1,
        math.ceil(distance / maximum_waypoint_spacing_m - 1e-9),
    )
    waypoints: list[np.ndarray] = []
    relative_rotation = current[:3, :3].T @ target[:3, :3]
    cosine = float(
        np.clip((np.trace(relative_rotation) - 1.0) * 0.5, -1.0, 1.0)
    )
    rotation_angle = math.acos(cosine)
    if rotation_angle > 1e-9:
        rotation_axis = np.array(
            (
                relative_rotation[2, 1] - relative_rotation[1, 2],
                relative_rotation[0, 2] - relative_rotation[2, 0],
                relative_rotation[1, 0] - relative_rotation[0, 1],
            ),
            dtype=np.float64,
        )
        rotation_axis /= 2.0 * math.sin(rotation_angle)
    else:
        rotation_axis = np.array((0.0, 0.0, 1.0), dtype=np.float64)
    for index in range(1, count + 1):
        ratio = index / count
        waypoint = current.copy()
        waypoint[:2, 3] = (
            current[:2, 3] + ratio * (target[:2, 3] - current[:2, 3])
        )
        interpolated_angle = rotation_angle * ratio
        axis_skew = np.array(
            (
                (0.0, -rotation_axis[2], rotation_axis[1]),
                (rotation_axis[2], 0.0, -rotation_axis[0]),
                (-rotation_axis[1], rotation_axis[0], 0.0),
            ),
            dtype=np.float64,
        )
        interpolated_rotation = (
            np.eye(3)
            + math.sin(interpolated_angle) * axis_skew
            + (1.0 - math.cos(interpolated_angle)) * (axis_skew @ axis_skew)
        )
        waypoint[:3, :3] = current[:3, :3] @ interpolated_rotation
        waypoints.append(validate_transform(waypoint))
    return waypoints


def validate_joint_step(
    current_positions: list[float],
    target_positions: list[float],
    maximum_joint_step_deg: float,
) -> float:
    """인접 IK 해 사이의 최대 관절 변화를 검사하고 degree로 반환한다."""
    if (
        len(current_positions) != len(target_positions)
        or not current_positions
    ):
        raise ValueError('현재/목표 관절 배열 크기가 다릅니다')
    differences = np.abs(
        np.asarray(target_positions, dtype=np.float64)
        - np.asarray(current_positions, dtype=np.float64)
    )
    if not np.all(np.isfinite(differences)):
        raise ValueError('관절값에 NaN 또는 inf가 있습니다')
    maximum_change_deg = math.degrees(float(np.max(differences)))
    if maximum_change_deg > maximum_joint_step_deg:
        raise ValueError(
            f'IK 관절 점프 {maximum_change_deg:.3f}deg가 '
            f'제한 {maximum_joint_step_deg:.3f}deg를 초과합니다'
        )
    return maximum_change_deg


def validate_fixed_z_pbvs_step(
    current_base_to_flange: np.ndarray,
    target_base_to_flange: np.ndarray,
    maximum_xy_step_m: float,
    maximum_z_change_m: float,
    maximum_orientation_change_deg: float,
) -> PbvsStepValidation:
    """XY 제한, Z 고정 및 자세 고정을 모두 확인한다."""
    current = validate_transform(current_base_to_flange)
    target = validate_transform(target_base_to_flange)
    delta = target[:3, 3] - current[:3, 3]
    xy_distance = float(np.linalg.norm(delta[:2]))
    if xy_distance > maximum_xy_step_m + 1e-9:
        raise ValueError(
            f'XY 이동 {xy_distance * 1000.0:.3f}mm가 '
            f'제한 {maximum_xy_step_m * 1000.0:.3f}mm를 초과합니다'
        )
    if abs(float(delta[2])) > maximum_z_change_m:
        raise ValueError(f'flange Z가 {delta[2] * 1000.0:+.3f}mm 변합니다')

    relative_rotation = current[:3, :3].T @ target[:3, :3]
    cosine = float(
        np.clip((np.trace(relative_rotation) - 1.0) * 0.5, -1.0, 1.0)
    )
    angle_deg = math.degrees(math.acos(cosine))
    if angle_deg > maximum_orientation_change_deg:
        raise ValueError(f'flange 자세가 {angle_deg:.4f}deg 변합니다')
    return PbvsStepValidation(
        delta_x_m=float(delta[0]),
        delta_y_m=float(delta[1]),
        xy_distance_m=xy_distance,
    )
