"""근거리 영상 오차를 작은 이동 명령으로 변환하는 초기 P 제어기."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class IbvsCommand:
    error_px: np.ndarray
    camera_delta_m: np.ndarray
    converged: bool


def proportional_xy_step(
    current_center_px: np.ndarray,
    desired_center_px: np.ndarray,
    gain_x: float,
    gain_y: float,
    maximum_step_m: float,
    tolerance_px: float,
) -> IbvsCommand:
    current = np.asarray(current_center_px, dtype=np.float64).reshape(2)
    desired = np.asarray(desired_center_px, dtype=np.float64).reshape(2)
    error = current - desired
    if not np.all(np.isfinite(error)):
        raise ValueError('IBVS 픽셀 오차가 유효하지 않습니다')
    converged = bool(np.linalg.norm(error) <= tolerance_px)
    delta = np.array([-gain_x * error[0], -gain_y * error[1], 0.0])
    norm = float(np.linalg.norm(delta))
    if maximum_step_m <= 0.0:
        raise ValueError('maximum_step_m은 0보다 커야 합니다')
    if norm > maximum_step_m:
        delta *= maximum_step_m / norm
    if converged:
        delta[:] = 0.0
    return IbvsCommand(error_px=error, camera_delta_m=delta, converged=converged)
