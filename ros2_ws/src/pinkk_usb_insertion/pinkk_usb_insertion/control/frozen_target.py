"""초기 관측에서 고정한 XY 목표를 향한 제한 이동을 계산한다."""

from __future__ import annotations

import math

import numpy as np


def circular_mean_degrees(values) -> float:
    """Wrap 경계를 고려해 각도 표본의 원형 평균을 degree로 반환한다."""
    angles = np.asarray(values, dtype=np.float64).reshape(-1)
    if angles.size == 0 or not np.all(np.isfinite(angles)):
        raise ValueError('각도 표본은 하나 이상의 유한값이어야 합니다')
    radians = np.radians(angles)
    sine = float(np.mean(np.sin(radians)))
    cosine = float(np.mean(np.cos(radians)))
    if math.hypot(sine, cosine) < 1e-12:
        raise ValueError('각도 표본의 원형 평균 방향을 결정할 수 없습니다')
    return math.degrees(math.atan2(sine, cosine))


def circular_median_degrees(values) -> float:
    """Wrap 경계를 고려해 각도 표본의 중앙값을 degree로 반환한다."""
    angles = np.asarray(values, dtype=np.float64).reshape(-1)
    if angles.size == 0 or not np.all(np.isfinite(angles)):
        raise ValueError('각도 표본은 하나 이상의 유한값이어야 합니다')
    reference = circular_mean_degrees(angles)
    unwrapped = reference + (angles - reference + 180.0) % 360.0 - 180.0
    median = float(np.median(unwrapped))
    return (median + 180.0) % 360.0 - 180.0


def maximum_angular_deviation_degrees(values, center_deg: float) -> float:
    """중심각에서 각 표본까지의 최소 wrap 각도 중 최댓값을 반환한다."""
    angles = np.asarray(values, dtype=np.float64).reshape(-1)
    center = float(center_deg)
    if angles.size == 0 or not np.all(np.isfinite(angles)):
        raise ValueError('각도 표본은 하나 이상의 유한값이어야 합니다')
    if not math.isfinite(center):
        raise ValueError('중심각은 유한값이어야 합니다')
    deviations = (angles - center + 180.0) % 360.0 - 180.0
    return float(np.max(np.abs(deviations)))


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


def final_insertion_target_z_m(
    guard_start_z_m: float,
    relative_descent_m: float,
    configured_final_z_m: float,
) -> float:
    """guard에서 상대 하강하되 configured 포트 최종 Z 아래는 목표로 삼지 않는다."""
    start = float(guard_start_z_m)
    distance = float(relative_descent_m)
    configured = float(configured_final_z_m)
    if not all(math.isfinite(value) for value in (start, distance, configured)):
        raise ValueError('최종 삽입 Z 값은 유한값이어야 합니다')
    if distance <= 0.0:
        raise ValueError('최종 삽입 상대 거리는 양수여야 합니다')
    if configured > start:
        raise ValueError('configured 최종 Z가 guard 시작 Z보다 높습니다')
    return max(start - distance, configured)
