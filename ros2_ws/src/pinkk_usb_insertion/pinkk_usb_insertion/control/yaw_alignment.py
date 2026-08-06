"""포트와 플러그 긴 축의 평면 Yaw 오차를 계산한다."""

from __future__ import annotations

import math

import numpy as np

from ..geometry.transforms import (
    rotation_to_rpy_degrees,
    rpy_degrees_to_rotation,
    validate_transform,
)


def named_axis_vector(axis_name: str) -> np.ndarray:
    """x/y 축 이름을 3차원 단위벡터로 변환한다."""
    normalized = axis_name.strip().lower()
    if normalized == 'x':
        return np.array((1.0, 0.0, 0.0), dtype=np.float64)
    if normalized == 'y':
        return np.array((0.0, 1.0, 0.0), dtype=np.float64)
    raise ValueError('평면 장축 이름은 x 또는 y여야 합니다')


def undirected_planar_axis_error_rad(
    current_axis_base: np.ndarray,
    target_axis_base: np.ndarray,
) -> float:
    """방향 부호가 없는 두 XY 축의 최소 signed angle을 반환한다."""
    # 행렬의 column view가 들어와도 아래 in-place 정규화가 원본 회전행렬을
    # 훼손하지 않도록 독립 배열로 복사한다.
    current = (
        np.asarray(current_axis_base, dtype=np.float64)
        .reshape(3)[:2]
        .copy()
    )
    target = (
        np.asarray(target_axis_base, dtype=np.float64)
        .reshape(3)[:2]
        .copy()
    )
    current_norm = float(np.linalg.norm(current))
    target_norm = float(np.linalg.norm(target))
    if current_norm < 1e-9 or target_norm < 1e-9:
        raise ValueError('긴 축을 base XY 평면에 투영할 수 없습니다')
    current /= current_norm
    target /= target_norm
    angle = math.atan2(
        current[0] * target[1] - current[1] * target[0],
        float(np.dot(current, target)),
    )
    return (angle + math.pi / 2.0) % math.pi - math.pi / 2.0


def limited_yaw_step_rad(
    yaw_error_rad: float,
    maximum_step_deg: float,
    tolerance_deg: float,
) -> tuple[float, bool]:
    """Yaw 오차를 1회 제한값으로 자르고 수렴 여부를 반환한다."""
    if maximum_step_deg <= 0.0 or tolerance_deg < 0.0:
        raise ValueError('Yaw step은 양수이고 tolerance는 0 이상이어야 합니다')
    error = float(yaw_error_rad)
    if not math.isfinite(error):
        raise ValueError('Yaw 오차가 유한값이 아닙니다')
    tolerance = math.radians(tolerance_deg)
    maximum_step = math.radians(maximum_step_deg)
    converged = abs(error) <= tolerance
    return float(np.clip(error, -maximum_step, maximum_step)), converged


def keypoint_image_yaw_step_rad(
    measured_axis_deg: float,
    desired_axis_deg: float,
    maximum_step_deg: float,
    tolerance_deg: float,
    command_sign: float = -1.0,
) -> tuple[float, bool]:
    """영상 장축 오차를 base Yaw 단발 명령으로 변환한다."""
    measured = float(measured_axis_deg)
    desired = float(desired_axis_deg)
    sign = float(command_sign)
    if not all(math.isfinite(value) for value in (measured, desired, sign)):
        raise ValueError('keypoint Yaw 입력은 유한값이어야 합니다')
    if sign not in (-1.0, 1.0):
        raise ValueError('keypoint Yaw 명령 부호는 -1 또는 +1이어야 합니다')
    # 포트 장축은 방향성이 없으므로 영상 오차도 [-90, 90)로 정규화한다.
    image_error_deg = (measured - desired + 90.0) % 180.0 - 90.0
    return limited_yaw_step_rad(
        math.radians(sign * image_error_deg),
        maximum_step_deg,
        tolerance_deg,
    )


def calibrated_keypoint_joint_step_rad(
    measured_axis_deg: float,
    desired_axis_deg: float,
    joint_gain: float,
    maximum_joint_step_deg: float,
    tolerance_deg: float,
    command_sign: float = -1.0,
) -> tuple[float, bool]:
    """영상 Yaw 오차를 실측 gain이 적용된 Joint6 회전량으로 변환한다."""
    gain = float(joint_gain)
    if not math.isfinite(gain) or gain <= 0.0:
        raise ValueError('keypoint Joint6 gain은 양의 유한값이어야 합니다')
    image_step, converged = keypoint_image_yaw_step_rad(
        measured_axis_deg,
        desired_axis_deg,
        maximum_joint_step_deg / gain,
        tolerance_deg,
        command_sign,
    )
    return image_step * gain, converged


def joint6_yaw_target_rad(
    current_joint_rad: float,
    yaw_step_rad: float,
    direction: float = 1.0,
    limit_deg: float = 175.0,
) -> float:
    """영상 Yaw step을 안전 범위 안의 6번 관절 절대 목표로 변환한다."""
    current = float(current_joint_rad)
    step = float(yaw_step_rad)
    sign = float(direction)
    limit = float(limit_deg)
    if not all(
        math.isfinite(value) for value in (current, step, sign, limit)
    ):
        raise ValueError('joint6 Yaw 입력은 유한값이어야 합니다')
    if sign not in (-1.0, 1.0):
        raise ValueError('joint6 Yaw 방향은 -1 또는 +1이어야 합니다')
    if not 0.0 < limit < 180.0:
        raise ValueError('joint6 제한각은 0보다 크고 180보다 작아야 합니다')
    target = current + sign * step
    if abs(target) > math.radians(limit):
        raise ValueError(
            'joint6 Yaw 목표가 안전 제한을 초과합니다: '
            f'current={math.degrees(current):+.3f}deg, '
            f'target={math.degrees(target):+.3f}deg, '
            f'limit={limit:.3f}deg'
        )
    return target


def apply_base_yaw_step(
    current_base_to_flange: np.ndarray,
    yaw_step_rad: float,
) -> np.ndarray:
    """현재 위치와 tilt를 유지하며 base Z축 기준 Yaw step을 적용한다."""
    current = validate_transform(current_base_to_flange)
    angle = float(yaw_step_rad)
    if not math.isfinite(angle):
        raise ValueError('Yaw step이 유한값이 아닙니다')
    cosine = math.cos(angle)
    sine = math.sin(angle)
    base_yaw = np.array(
        (
            (cosine, -sine, 0.0),
            (sine, cosine, 0.0),
            (0.0, 0.0, 1.0),
        ),
        dtype=np.float64,
    )
    target = current.copy()
    target[:3, :3] = base_yaw @ current[:3, :3]
    return validate_transform(target)


def apply_rpy_locked_yaw_step(
    current_base_to_flange: np.ndarray,
    yaw_step_rad: float,
) -> np.ndarray:
    """현재 XYZ·Roll·Pitch를 유지하고 ZYX Yaw만 변경한다."""
    current = validate_transform(current_base_to_flange)
    step = float(yaw_step_rad)
    if not math.isfinite(step):
        raise ValueError('Cartesian Yaw step이 유한값이 아닙니다')
    roll, pitch, yaw = rotation_to_rpy_degrees(current[:3, :3])
    target = current.copy()
    target[:3, :3] = rpy_degrees_to_rotation(
        roll,
        pitch,
        yaw + math.degrees(step),
    )
    return validate_transform(target)


def apply_observation_roll_pitch_with_current_yaw(
    current_base_to_flange: np.ndarray,
    observation_base_to_flange: np.ndarray,
) -> np.ndarray:
    """현재 XYZ/Yaw와 초기 관측 Roll/Pitch를 결합한 목표를 만든다."""
    current = validate_transform(current_base_to_flange)
    observation = validate_transform(observation_base_to_flange)
    observation_roll, observation_pitch, _ = rotation_to_rpy_degrees(
        observation[:3, :3]
    )
    _, _, current_yaw = rotation_to_rpy_degrees(current[:3, :3])
    target = current.copy()
    target[:3, :3] = rpy_degrees_to_rotation(
        observation_roll,
        observation_pitch,
        current_yaw,
    )
    return validate_transform(target)


def apply_proportional_observation_roll_pitch_with_current_yaw(
    current_base_to_flange: np.ndarray,
    observation_base_to_flange: np.ndarray,
    gain: float,
) -> np.ndarray:
    """초기 관측 Roll/Pitch 오차의 일부만 적용하고 현재 Yaw를 유지한다."""
    current = validate_transform(current_base_to_flange)
    observation = validate_transform(observation_base_to_flange)
    kp = float(gain)
    if not math.isfinite(kp) or not 0.0 < kp <= 1.0:
        raise ValueError('Roll/Pitch P gain은 0보다 크고 1 이하여야 합니다')
    current_roll, current_pitch, current_yaw = rotation_to_rpy_degrees(
        current[:3, :3]
    )
    observation_roll, observation_pitch, _ = rotation_to_rpy_degrees(
        observation[:3, :3]
    )
    roll_error = (observation_roll - current_roll + 180.0) % 360.0 - 180.0
    pitch_error = (
        (observation_pitch - current_pitch + 180.0) % 360.0 - 180.0
    )
    target = current.copy()
    target[:3, :3] = rpy_degrees_to_rotation(
        current_roll + kp * roll_error,
        current_pitch + kp * pitch_error,
        current_yaw,
    )
    return validate_transform(target)


def invert_orientation_step(
    current_base_to_flange: np.ndarray,
    target_base_to_flange: np.ndarray,
) -> np.ndarray:
    """현재 자세 기준 목표 상대 회전을 반대 방향으로 뒤집는다."""
    current = validate_transform(current_base_to_flange)
    target = validate_transform(target_base_to_flange)
    relative_rotation = current[:3, :3].T @ target[:3, :3]
    inverted = target.copy()
    inverted[:3, :3] = current[:3, :3] @ relative_rotation.T
    return validate_transform(inverted)


def keypoint_long_axis_angle_deg(points_xy: np.ndarray) -> float:
    """네 모서리의 두 긴 변 평균 각도를 영상 좌표계 degree로 반환한다.

    keypoint 순서는 0→1과 3→2가 서로 평행한 긴 변이어야 한다. 영상의
    +Y가 아래쪽이므로 양의 각도는 화면에서 시계 방향이다. 축은 방향성이
    없으므로 결과를 [-90, 90) 범위로 정규화한다.
    """
    points = np.asarray(points_xy, dtype=np.float64)
    if points.shape != (4, 2) or not np.all(np.isfinite(points)):
        raise ValueError('Yaw 픽셀 각도에는 유한한 keypoint 4×2가 필요합니다')
    edges = (points[1] - points[0], points[2] - points[3])
    angles: list[float] = []
    for edge in edges:
        if float(np.linalg.norm(edge)) < 1e-6:
            raise ValueError('Yaw 픽셀 장축 keypoint 간격이 너무 작습니다')
        angles.append(math.atan2(float(edge[1]), float(edge[0])))
    doubled_sine = sum(math.sin(2.0 * angle) for angle in angles)
    doubled_cosine = sum(math.cos(2.0 * angle) for angle in angles)
    if math.hypot(doubled_sine, doubled_cosine) < 1e-6:
        raise ValueError('두 긴 변의 픽셀 각도가 서로 일치하지 않습니다')
    mean = 0.5 * math.atan2(doubled_sine, doubled_cosine)
    return math.degrees(mean)
