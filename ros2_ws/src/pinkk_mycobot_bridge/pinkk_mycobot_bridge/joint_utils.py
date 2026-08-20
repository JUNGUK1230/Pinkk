"""MyCobot 관절 이름과 단위 변환처럼 시리얼 연결이 필요 없는 공통 도구."""

from __future__ import annotations

import math
from typing import Sequence


JOINT_NAMES = (
    'joint2_to_joint1',
    'joint3_to_joint2',
    'joint4_to_joint3',
    'joint5_to_joint4',
    'joint6_to_joint5',
    'joint6output_to_joint6',
)


def angles_deg_to_rad(values: Sequence[float]) -> list[float]:
    """검증한 6개 제조사 관절각을 degree에서 radian으로 바꾼다."""
    if not isinstance(values, (list, tuple)) or len(values) != len(JOINT_NAMES):
        raise ValueError(f'expected 6 joint angles, got {values!r}')
    numeric = [float(value) for value in values]
    if not all(math.isfinite(value) for value in numeric):
        raise ValueError(f'joint angles contain NaN or inf: {values!r}')
    if any(abs(value) > 360.0 for value in numeric):
        raise ValueError(f'joint angle outside sanity range: {values!r}')
    return [math.radians(value) for value in numeric]
