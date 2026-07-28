"""OBSERVE_POSE 관절 근접 여부를 검사한다."""

from __future__ import annotations

import math
from collections.abc import Sequence

import numpy as np


def observation_joint_errors_deg(
    actual_joint_positions_rad: Sequence[float],
    observation_joint_positions_deg: Sequence[float],
) -> np.ndarray:
    """관절별 최단 각도 오차를 degree로 반환한다."""
    actual = np.asarray(actual_joint_positions_rad, dtype=np.float64).reshape(-1)
    reference = np.radians(
        np.asarray(observation_joint_positions_deg, dtype=np.float64).reshape(-1)
    )
    if actual.size == 0 or actual.size != reference.size:
        raise ValueError('현재 관절과 OBSERVE_POSE 관절 배열 크기가 다릅니다')
    if not np.all(np.isfinite(actual)) or not np.all(np.isfinite(reference)):
        raise ValueError('관절값에 NaN 또는 inf가 있습니다')
    wrapped = np.arctan2(np.sin(actual - reference), np.cos(actual - reference))
    return np.degrees(wrapped)


def validate_observation_joint_pose(
    actual_joint_positions_rad: Sequence[float],
    observation_joint_positions_deg: Sequence[float],
    maximum_error_deg: float,
) -> float:
    """OBSERVE_POSE 근접 여부를 검사하고 최대 절대 오차를 반환한다."""
    if not math.isfinite(maximum_error_deg) or maximum_error_deg <= 0.0:
        raise ValueError('OBSERVE_POSE 관절 허용오차는 0보다 커야 합니다')
    errors = observation_joint_errors_deg(
        actual_joint_positions_rad,
        observation_joint_positions_deg,
    )
    maximum_error = float(np.max(np.abs(errors)))
    if maximum_error > maximum_error_deg:
        formatted = [round(float(value), 3) for value in errors]
        raise ValueError(
            f'OBSERVE_POSE가 아닙니다: error_deg={formatted}, '
            f'max_error={maximum_error:.3f}deg, '
            f'limit={maximum_error_deg:.3f}deg'
        )
    return maximum_error
