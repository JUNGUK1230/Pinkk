"""실행 전 공통 안전 조건을 한곳에서 검사한다."""

from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class SafetyDecision:
    allowed: bool
    reason: str


def validate_detection(
    depth_m: float,
    reprojection_error_px: float,
    age_seconds: float,
    minimum_depth_m: float,
    maximum_depth_m: float,
    maximum_reprojection_error_px: float,
    maximum_age_seconds: float,
) -> SafetyDecision:
    values = (depth_m, reprojection_error_px, age_seconds)
    if not all(math.isfinite(value) for value in values):
        return SafetyDecision(False, '검출 결과에 NaN 또는 inf가 있습니다')
    if not minimum_depth_m <= depth_m <= maximum_depth_m:
        return SafetyDecision(False, '포트 깊이가 허용 범위를 벗어났습니다')
    if reprojection_error_px > maximum_reprojection_error_px:
        return SafetyDecision(False, '재투영 오차가 너무 큽니다')
    if age_seconds < 0.0 or age_seconds > maximum_age_seconds:
        return SafetyDecision(False, '검출 결과가 오래됐습니다')
    return SafetyDecision(True, '검출 안전 조건 통과')
