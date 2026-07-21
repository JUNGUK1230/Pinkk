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


@dataclass(frozen=True)
class FixedZPbvsResult:
    """카메라 기준 고정-Z PBVS의 계산 결과."""

    base_to_port: np.ndarray
    target_base_to_flange: np.ndarray
    error_camera_xy_m: np.ndarray
    error_base_xy_m: np.ndarray
    applied_step_base_xy_m: np.ndarray
    converged: bool


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
