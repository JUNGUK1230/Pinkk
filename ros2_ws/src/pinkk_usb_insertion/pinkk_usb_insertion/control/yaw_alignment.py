"""포트와 플러그 긴 축의 평면 Yaw 오차를 계산한다."""

from __future__ import annotations

import math

import numpy as np

from ..geometry.transforms import validate_transform


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
    current = np.asarray(current_axis_base, dtype=np.float64).reshape(3)[:2]
    target = np.asarray(target_axis_base, dtype=np.float64).reshape(3)[:2]
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
