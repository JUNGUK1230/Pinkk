"""초기 관측에서 고정한 XY 목표를 향한 제한 이동을 계산한다."""

from __future__ import annotations

import math

import numpy as np


def xy_residual_m(current_xy, target_xy) -> float:
    """두 XY 좌표 사이의 유클리드 거리를 반환한다."""
    current = np.asarray(current_xy, dtype=np.float64).reshape(2)
    target = np.asarray(target_xy, dtype=np.float64).reshape(2)
    if not np.all(np.isfinite(current)) or not np.all(np.isfinite(target)):
        raise ValueError('XY 좌표는 유한값이어야 합니다')
    return float(np.linalg.norm(target - current))


def port_based_flange_target_z(
    port_z_m: float,
    flange_to_tip_z_m: float,
    insertion_depth_m: float,
) -> float:
    """수직 하향 TCP의 포트 삽입 깊이로 최종 flange base Z를 계산한다."""
    port_z = float(port_z_m)
    tcp_z = float(flange_to_tip_z_m)
    insertion = float(insertion_depth_m)
    if not all(math.isfinite(value) for value in (port_z, tcp_z, insertion)):
        raise ValueError('포트/TCP/삽입 깊이는 유한값이어야 합니다')
    if tcp_z < 0.0 or insertion < 0.0:
        raise ValueError('TCP 거리와 삽입 깊이는 0 이상이어야 합니다')
    if insertion > tcp_z:
        raise ValueError('삽입 깊이는 flange-to-tip TCP Z보다 클 수 없습니다')
    return port_z + tcp_z - insertion


def limited_xy_target(current_xy, target_xy, maximum_step_m: float):
    """목표 방향으로 최대 step만 이동한 다음 XY 목표를 반환한다."""
    current = np.asarray(current_xy, dtype=np.float64).reshape(2)
    target = np.asarray(target_xy, dtype=np.float64).reshape(2)
    maximum_step = float(maximum_step_m)
    if not np.all(np.isfinite(current)) or not np.all(np.isfinite(target)):
        raise ValueError('XY 좌표는 유한값이어야 합니다')
    if not math.isfinite(maximum_step) or maximum_step <= 0.0:
        raise ValueError('최대 XY step은 양의 유한값이어야 합니다')
    delta = target - current
    distance = float(np.linalg.norm(delta))
    if distance <= maximum_step:
        return target.copy(), distance
    return current + delta * (maximum_step / distance), distance


def proportional_xy_target(
    current_xy,
    target_xy,
    gain: float,
    maximum_step_m: float,
    minimum_step_m: float = 0.0,
):
    """XY 오차에 P gain을 적용하고 하드웨어 step 범위로 제한한다."""
    current = np.asarray(current_xy, dtype=np.float64).reshape(2)
    target = np.asarray(target_xy, dtype=np.float64).reshape(2)
    kp = float(gain)
    maximum_step = float(maximum_step_m)
    minimum_step = float(minimum_step_m)
    if not np.all(np.isfinite(current)) or not np.all(np.isfinite(target)):
        raise ValueError('XY 좌표는 유한값이어야 합니다')
    if not math.isfinite(kp) or not 0.0 < kp <= 1.0:
        raise ValueError('XY P gain은 0보다 크고 1 이하여야 합니다')
    if not math.isfinite(maximum_step) or maximum_step <= 0.0:
        raise ValueError('최대 XY step은 양의 유한값이어야 합니다')
    if not math.isfinite(minimum_step) or not 0.0 <= minimum_step <= maximum_step:
        raise ValueError('최소 XY step 범위가 잘못됐습니다')
    delta = target - current
    distance = float(np.linalg.norm(delta))
    if distance < 1e-12:
        return target.copy(), distance
    step_distance = min(distance * kp, maximum_step)
    if 0.0 < step_distance < minimum_step:
        step_distance = min(minimum_step, distance)
    return current + delta * (step_distance / distance), distance


def proportional_z_descent_m(
    remaining_m: float,
    gain: float,
    maximum_step_m: float,
    minimum_step_m: float,
) -> float:
    """남은 하강 오차에 P gain을 적용하고 실행 가능한 범위로 제한한다."""
    remaining = float(remaining_m)
    kp = float(gain)
    maximum_step = float(maximum_step_m)
    minimum_step = float(minimum_step_m)
    if not all(
        math.isfinite(value)
        for value in (remaining, kp, maximum_step, minimum_step)
    ):
        raise ValueError('Z P제어 값은 유한값이어야 합니다')
    if remaining < 0.0:
        raise ValueError('남은 Z 하강 오차는 0 이상이어야 합니다')
    if not 0.0 < kp <= 1.0:
        raise ValueError('Z P gain은 0보다 크고 1 이하여야 합니다')
    if maximum_step <= 0.0:
        raise ValueError('최대 Z step은 양수여야 합니다')
    if not 0.0 <= minimum_step <= maximum_step:
        raise ValueError('최소 Z step 범위가 잘못됐습니다')
    if remaining == 0.0:
        return 0.0
    command = min(remaining * kp, maximum_step, remaining)
    if command < minimum_step:
        command = min(minimum_step, remaining)
    return command
