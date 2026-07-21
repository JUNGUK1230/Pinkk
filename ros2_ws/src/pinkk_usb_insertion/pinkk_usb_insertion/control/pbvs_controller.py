"""포트 자세로부터 안전한 사전 접근 자세를 계산한다."""

from __future__ import annotations

import numpy as np

from ..geometry.transforms import approach_transform, compose, inverse


def calculate_flange_approach_pose(
    base_to_port: np.ndarray,
    flange_to_plug: np.ndarray,
    standoff_m: float,
) -> np.ndarray:
    """
    T_base_flange_goal을 계산한다.

    T_base_flange_goal = T_base_port × T_port_approach × inv(T_flange_plug)
    """
    return compose(
        base_to_port,
        approach_transform(standoff_m),
        inverse(flange_to_plug),
    )
