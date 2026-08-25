"""영상 중앙의 포트가 일정 시간 정지했는지 판정한다."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

import numpy as np


@dataclass(frozen=True)
class AutoStartSample:
    timestamp: float
    center_u: float
    center_v: float
    depth_m: float
    axis_deg: float


@dataclass(frozen=True)
class AutoStartResult:
    ready: bool
    reason: str
    sample_count: int
    duration_seconds: float
    center_error_px: float = math.inf
    center_spread_px: float = math.inf
    depth_spread_m: float = math.inf
    yaw_spread_deg: float = math.inf


def _axial_error_degrees(values: np.ndarray, center: float) -> np.ndarray:
    """방향 없는 장축의 180도 주기 오차를 반환한다."""
    return (values - center + 90.0) % 180.0 - 90.0


def evaluate_auto_start(
    samples: Sequence[AutoStartSample],
    *,
    now: float,
    stable_duration_seconds: float,
    minimum_samples: int,
    image_center_u_px: float,
    image_center_v_px: float,
    center_tolerance_px: float,
    maximum_center_spread_px: float,
    maximum_depth_spread_m: float,
    maximum_yaw_spread_deg: float,
    maximum_sample_gap_seconds: float,
) -> AutoStartResult:
    """최근 연속 표본이 중앙·정지 조건을 모두 만족하는지 반환한다."""
    recent = [
        sample
        for sample in samples
        if 0.0 <= now - sample.timestamp <= stable_duration_seconds
    ]
    if len(recent) < minimum_samples:
        return AutoStartResult(
            False,
            '표본 부족',
            len(recent),
            0.0,
        )
    timestamps = np.asarray(
        [sample.timestamp for sample in recent], dtype=np.float64
    )
    duration = float(timestamps[-1] - timestamps[0])
    if duration < stable_duration_seconds * 0.9:
        return AutoStartResult(
            False,
            '관찰 시간 부족',
            len(recent),
            duration,
        )
    if np.max(np.diff(timestamps)) > maximum_sample_gap_seconds:
        return AutoStartResult(
            False,
            '관측 연속성 부족',
            len(recent),
            duration,
        )

    centers = np.asarray(
        [(sample.center_u, sample.center_v) for sample in recent],
        dtype=np.float64,
    )
    depths = np.asarray(
        [sample.depth_m for sample in recent], dtype=np.float64
    )
    axes = np.asarray(
        [sample.axis_deg for sample in recent], dtype=np.float64
    )
    if not (
        np.all(np.isfinite(centers))
        and np.all(np.isfinite(depths))
        and np.all(np.isfinite(axes))
    ):
        return AutoStartResult(False, '유효하지 않은 표본', len(recent), duration)

    center = np.median(centers, axis=0)
    center_error = float(
        np.linalg.norm(
            center
            - np.asarray(
                [image_center_u_px, image_center_v_px], dtype=np.float64
            )
        )
    )
    center_spread = float(
        np.max(np.linalg.norm(centers - center, axis=1))
    )
    depth_center = float(np.median(depths))
    depth_spread = float(np.max(np.abs(depths - depth_center)))
    doubled_axis_radians = np.radians(2.0 * axes)
    axis_center = 0.5 * math.degrees(
        math.atan2(
            float(np.mean(np.sin(doubled_axis_radians))),
            float(np.mean(np.cos(doubled_axis_radians))),
        )
    )
    yaw_spread = float(
        np.max(np.abs(_axial_error_degrees(axes, axis_center)))
    )
    metrics = dict(
        sample_count=len(recent),
        duration_seconds=duration,
        center_error_px=center_error,
        center_spread_px=center_spread,
        depth_spread_m=depth_spread,
        yaw_spread_deg=yaw_spread,
    )
    if center_error > center_tolerance_px:
        return AutoStartResult(False, '포트가 영상 중앙 밖', **metrics)
    if center_spread > maximum_center_spread_px:
        return AutoStartResult(False, '영상 중심 흔들림', **metrics)
    if depth_spread > maximum_depth_spread_m:
        return AutoStartResult(False, '깊이 흔들림', **metrics)
    if yaw_spread > maximum_yaw_spread_deg:
        return AutoStartResult(False, 'Yaw 흔들림', **metrics)
    return AutoStartResult(True, '중앙 정지 관측 완료', **metrics)
