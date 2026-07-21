"""포트와 플러그 긴 축의 평면 Yaw 오차를 계산한다."""

from __future__ import annotations

import math

import numpy as np


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
