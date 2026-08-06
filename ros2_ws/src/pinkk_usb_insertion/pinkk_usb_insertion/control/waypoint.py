"""PBVS 절대 목표까지 한 번에 이동하지 않고 제한된 3D waypoint를 계산한다."""

from __future__ import annotations

import math

import numpy as np


def limited_waypoint_translation(
    current_xyz: np.ndarray,
    target_xyz: np.ndarray,
    maximum_xy_step_m: float,
    maximum_z_step_m: float,
    minimum_xy_step_m: float,
) -> tuple[np.ndarray, float, bool]:
    """현재 위치에서 목표 방향으로 XY/Z를 각각 제한한 다음 waypoint를 만든다.

    XY는 목표 방향 단위벡터를 따라 최대 `maximum_xy_step_m`까지만 이동하고,
    남은 거리가 `minimum_xy_step_m` 미만이면 기기 백래시/데드밴드 때문에
    움직이지 않는다. Z는 XY와 무관하게 최대 `maximum_z_step_m`까지만
    독립적으로 이동한다.

    반환값은 (waypoint_xyz, xy_distance_to_target, xy_step_skipped)이며
    xy_step_skipped는 XY를 옮기지 않고 현재 XY를 유지했음을 나타낸다.
    """
    current = np.asarray(current_xyz, dtype=np.float64).reshape(3)
    target = np.asarray(target_xyz, dtype=np.float64).reshape(3)
    if not np.all(np.isfinite(current)) or not np.all(np.isfinite(target)):
        raise ValueError('waypoint 위치는 유한값이어야 합니다')
    if maximum_xy_step_m <= 0.0 or maximum_z_step_m <= 0.0:
        raise ValueError('waypoint 최대 step은 0보다 커야 합니다')
    if minimum_xy_step_m < 0.0:
        raise ValueError('waypoint 최소 XY step은 0 이상이어야 합니다')

    delta = target - current
    xy_delta = delta[:2]
    xy_distance = float(np.linalg.norm(xy_delta))

    waypoint = current.copy()
    xy_step_skipped = xy_distance < minimum_xy_step_m
    if not xy_step_skipped:
        xy_step = min(xy_distance, maximum_xy_step_m)
        waypoint[:2] = current[:2] + xy_delta * (xy_step / xy_distance)

    z_delta = float(delta[2])
    z_step = math.copysign(min(abs(z_delta), maximum_z_step_m), z_delta)
    waypoint[2] = current[2] + z_step

    return waypoint, xy_distance, xy_step_skipped
