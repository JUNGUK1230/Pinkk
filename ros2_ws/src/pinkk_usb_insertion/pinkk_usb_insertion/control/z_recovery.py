"""포트 재검출을 위한 제한된 Z-only 복구 목표를 계산한다."""

from __future__ import annotations

import math

import numpy as np

from ..geometry.transforms import validate_transform


def raised_z_recovery_target(
    current_base_to_flange: np.ndarray,
    observation_base_to_flange: np.ndarray,
    step_m: float,
) -> np.ndarray:
    """현재 X/Y·자세를 유지하고 초기 관측 Z 방향으로 한 단계 올린다.

    초기 관측 Z를 상한으로 사용하므로 반복 복구가 관측 높이를 넘어가지 않는다.
    초기 관측 Z가 현재 Z보다 높지 않으면 안전한 상승 목표를 만들 수 없으므로
    명시적으로 거부한다.
    """
    current = np.asarray(current_base_to_flange, dtype=np.float64)
    observation = np.asarray(observation_base_to_flange, dtype=np.float64)
    validate_transform(current)
    validate_transform(observation)
    step = float(step_m)
    if not math.isfinite(step) or step <= 0.0:
        raise ValueError('Z-only 복구 step은 양의 유한값이어야 합니다')

    current_z = float(current[2, 3])
    observation_z = float(observation[2, 3])
    available_rise = observation_z - current_z
    if available_rise <= 1e-4:
        raise ValueError(
            '초기 관측 Z가 현재 Z보다 높지 않아 Z-only 상승 복구를 할 수 '
            f'없습니다: current_z={current_z:.6f}m, '
            f'observation_z={observation_z:.6f}m'
        )

    target = current.copy()
    target[2, 3] = current_z + min(step, available_rise)
    validate_transform(target)
    return target
