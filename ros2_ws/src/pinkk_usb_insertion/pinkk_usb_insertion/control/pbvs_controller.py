"""포트 자세로부터 PBVS 목표 자세를 계산한다."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ..geometry.transforms import (
    approach_transform,
    compose,
    inverse,
    validate_transform,
)
from .yaw_alignment import (
    apply_base_yaw_step,
    undirected_planar_axis_error_rad,
)


@dataclass(frozen=True)
class FixedZPbvsResult:
    """카메라 기준 고정-Z PBVS의 계산 결과."""

    base_to_port: np.ndarray
    target_base_to_flange: np.ndarray
    error_camera_xy_m: np.ndarray
    error_base_xy_m: np.ndarray
    applied_step_base_xy_m: np.ndarray
    converged: bool


@dataclass(frozen=True)
class AbsolutePortPoseResult:
    """포트 절대 X/Y와 초기 관측 Z/tilt를 결합한 목표."""

    target_base_to_flange: np.ndarray
    delta_base_xy_m: np.ndarray
    yaw_error_rad: float
    target_yaw_offset_rad: float
    converged: bool


def calculate_absolute_port_xy_yaw_target(
    current_base_to_flange: np.ndarray,
    locked_base_to_flange: np.ndarray,
    base_to_port: np.ndarray,
    plug_long_axis_in_flange: np.ndarray,
    port_long_axis: np.ndarray,
    xy_tolerance_m: float,
    yaw_tolerance_deg: float,
    pre_approach_height_m: float = 0.0,
) -> AbsolutePortPoseResult:
    """
    TCP offset을 0으로 보고 flange를 포트의 base X/Y로 직접 보낸다.

    pre_approach_height_m이 양수면 목표 Z는 port Z + 높이를 사용한다. 0이면
    기존 동작처럼 초기 OBSERVE_POSE Z를 사용한다. tilt는 초기 자세를
    사용하고 base Z축 둘레의 Yaw만 포트 장축의 180도 대칭 방향에 맞춘다.
    """
    current = validate_transform(current_base_to_flange)
    reference = validate_transform(locked_base_to_flange)
    port = validate_transform(base_to_port)
    plug_axis = np.asarray(plug_long_axis_in_flange, dtype=np.float64).reshape(3)
    port_axis = np.asarray(port_long_axis, dtype=np.float64).reshape(3)
    if not np.all(np.isfinite(plug_axis)) or not np.all(np.isfinite(port_axis)):
        raise ValueError('플러그/포트 장축은 유한값이어야 합니다')
    if xy_tolerance_m < 0.0 or yaw_tolerance_deg < 0.0:
        raise ValueError('XY/Yaw 허용오차는 0 이상이어야 합니다')
    height = float(pre_approach_height_m)
    if not np.isfinite(height) or height < 0.0:
        raise ValueError('pre-approach 높이는 0 이상의 유한값이어야 합니다')

    current_axis_base = current[:3, :3] @ plug_axis
    reference_axis_base = reference[:3, :3] @ plug_axis
    target_axis_base = port[:3, :3] @ port_axis
    target_yaw_offset = undirected_planar_axis_error_rad(
        reference_axis_base,
        target_axis_base,
    )
    yaw_error = undirected_planar_axis_error_rad(
        current_axis_base,
        target_axis_base,
    )
    target = apply_base_yaw_step(reference, target_yaw_offset)
    target[0, 3] = port[0, 3]
    target[1, 3] = port[1, 3]
    if height > 0.0:
        target[2, 3] = port[2, 3] + height
    delta_xy = target[:2, 3] - current[:2, 3]
    converged = (
        float(np.linalg.norm(delta_xy)) <= xy_tolerance_m
        and abs(np.degrees(yaw_error)) <= yaw_tolerance_deg
    )
    return AbsolutePortPoseResult(
        target_base_to_flange=validate_transform(target),
        delta_base_xy_m=delta_xy,
        yaw_error_rad=float(yaw_error),
        target_yaw_offset_rad=float(target_yaw_offset),
        converged=converged,
    )


def calculate_fixed_z_camera_pbvs(
    base_to_flange: np.ndarray,
    flange_to_camera: np.ndarray,
    camera_to_port: np.ndarray,
    maximum_xy_step_m: float,
    xy_tolerance_m: float,
    desired_port_x_m: float = 0.0,
    desired_port_y_m: float = 0.0,
    locked_base_to_flange: np.ndarray | None = None,
) -> FixedZPbvsResult:
    """
    현재 flange의 Z와 자세를 유지하며 포트를 카메라 중심에 맞춘다.

    포트의 카메라 X/Y 오차를 base 좌표계로 회전한 뒤 base X/Y 성분만
    적용한다. 반환되는 목표는 계산·검증용이며 이 함수는 로봇 명령을 만들지
    않는다.
    """
    if not np.isfinite(maximum_xy_step_m) or maximum_xy_step_m <= 0.0:
        raise ValueError('maximum_xy_step_m은 0보다 큰 유한값이어야 합니다')
    if not np.isfinite(xy_tolerance_m) or xy_tolerance_m < 0.0:
        raise ValueError('xy_tolerance_m은 0 이상의 유한값이어야 합니다')
    if not np.all(np.isfinite((desired_port_x_m, desired_port_y_m))):
        raise ValueError('목표 포트 좌표는 유한값이어야 합니다')

    current_flange = validate_transform(base_to_flange)
    flange_camera = validate_transform(flange_to_camera)
    camera_port = validate_transform(camera_to_port)
    base_to_camera = compose(current_flange, flange_camera)
    base_to_port = compose(base_to_camera, camera_port)

    error_camera = np.array(
        [
            camera_port[0, 3] - desired_port_x_m,
            camera_port[1, 3] - desired_port_y_m,
            0.0,
        ],
        dtype=np.float64,
    )
    error_base_xy = (base_to_camera[:3, :3] @ error_camera)[:2]
    error_norm = float(np.linalg.norm(error_base_xy))
    scale = min(1.0, maximum_xy_step_m / max(error_norm, 1e-12))
    applied_step = error_base_xy * scale

    reference = (
        current_flange
        if locked_base_to_flange is None
        else validate_transform(locked_base_to_flange)
    )
    target = current_flange.copy()
    target[0, 3] += applied_step[0]
    target[1, 3] += applied_step[1]
    target[2, 3] = reference[2, 3]
    target[:3, :3] = reference[:3, :3]

    return FixedZPbvsResult(
        base_to_port=base_to_port,
        target_base_to_flange=validate_transform(target),
        error_camera_xy_m=error_camera[:2],
        error_base_xy_m=error_base_xy,
        applied_step_base_xy_m=applied_step,
        converged=error_norm <= xy_tolerance_m,
    )


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
