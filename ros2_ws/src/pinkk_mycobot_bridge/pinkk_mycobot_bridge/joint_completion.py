"""관절 목표 도달과 실제 정지를 연속 표본으로 검증한다."""

from __future__ import annotations

import math
from typing import Sequence


def wrapped_joint_error(target: float, actual: float) -> float:
    """두 radian 관절각 사이의 최단 절대 오차를 반환한다."""
    return abs(math.remainder(float(target) - float(actual), 2.0 * math.pi))


def maximum_joint_error(
    target: Sequence[float],
    actual: Sequence[float],
) -> float:
    """같은 길이 관절 배열의 최대 절대 오차를 반환한다."""
    if len(target) != len(actual) or not target:
        raise ValueError('target과 actual 관절 배열 길이가 같고 비어 있지 않아야 합니다')
    return max(
        wrapped_joint_error(target_value, actual_value)
        for target_value, actual_value in zip(target, actual, strict=True)
    )


def signed_joint_errors_degrees(
    target: Sequence[float],
    actual: Sequence[float],
) -> list[float]:
    """각 관절의 target-actual 최단 오차를 degree로 반환한다."""
    if len(target) != len(actual) or not target:
        raise ValueError(
            'target과 actual 관절 배열 길이가 같고 비어 있지 않아야 합니다'
        )
    return [
        math.degrees(
            math.remainder(
                float(target_value) - float(actual_value),
                2.0 * math.pi,
            )
        )
        for target_value, actual_value in zip(target, actual, strict=True)
    ]


def compensated_joint_command_degrees(
    requested_target_deg: Sequence[float],
    previous_command_deg: Sequence[float],
    actual_deg: Sequence[float],
    *,
    gain: float,
    maximum_step_deg: float,
    maximum_total_offset_deg: float,
) -> tuple[list[float], list[float], list[float]]:
    """실측 관절 오차를 제한된 다음 명령으로 변환한다.

    최종 완료 판정의 기준은 ``requested_target_deg``이며, 보상된 명령은
    하드웨어의 정지 오차를 줄이기 위한 임시 입력으로만 사용한다.
    """
    lengths = {
        len(requested_target_deg),
        len(previous_command_deg),
        len(actual_deg),
    }
    if len(lengths) != 1 or not requested_target_deg:
        raise ValueError('보상 관절 배열 길이가 같고 비어 있지 않아야 합니다')
    if not 0.0 < gain <= 1.0:
        raise ValueError('관절 보상 gain은 0보다 크고 1 이하여야 합니다')
    if maximum_step_deg <= 0.0:
        raise ValueError('관절 1회 보상 제한은 0보다 커야 합니다')
    if maximum_total_offset_deg < maximum_step_deg:
        raise ValueError('관절 누적 보상 제한은 1회 보상 제한 이상이어야 합니다')

    target = [float(value) for value in requested_target_deg]
    previous = [float(value) for value in previous_command_deg]
    actual = [float(value) for value in actual_deg]
    if not all(math.isfinite(value) for values in (target, previous, actual)
               for value in values):
        raise ValueError('관절 보상 입력은 유한한 숫자여야 합니다')

    next_command: list[float] = []
    applied_correction: list[float] = []
    total_offset: list[float] = []
    for target_value, previous_value, actual_value in zip(
        target, previous, actual, strict=True
    ):
        error = math.remainder(target_value - actual_value, 360.0)
        correction = max(
            -maximum_step_deg,
            min(maximum_step_deg, gain * error),
        )
        proposed = previous_value + correction
        offset = max(
            -maximum_total_offset_deg,
            min(maximum_total_offset_deg, proposed - target_value),
        )
        command = target_value + offset
        next_command.append(round(command, 3))
        applied_correction.append(round(command - previous_value, 3))
        total_offset.append(round(offset, 3))
    return next_command, applied_correction, total_offset


class JointStabilityMonitor:
    """목표 오차·표본간 변화·로봇 정지를 모두 만족한 연속 횟수를 센다."""

    def __init__(
        self,
        tolerance_rad: float,
        stable_delta_rad: float,
        required_samples: int,
    ) -> None:
        """정지 판정에 사용할 오차·변화량·연속 표본 수를 설정한다."""
        if tolerance_rad <= 0.0 or stable_delta_rad <= 0.0:
            raise ValueError('관절 오차와 안정 변화량은 0보다 커야 합니다')
        if required_samples < 2:
            raise ValueError('정지 확인 표본은 최소 2개여야 합니다')
        self._tolerance = float(tolerance_rad)
        self._stable_delta = float(stable_delta_rad)
        self._required = int(required_samples)
        self._previous: list[float] | None = None
        self._consecutive = 0

    @property
    def consecutive_samples(self) -> int:
        """현재까지 연속으로 통과한 표본 수를 반환한다."""
        return self._consecutive

    def update(
        self,
        target: Sequence[float],
        actual: Sequence[float],
        robot_is_moving: bool,
    ) -> bool:
        """한 표본을 반영하고 목표에서 실제 정지했으면 true를 반환한다."""
        current = [float(value) for value in actual]
        if len(target) != len(current) or not current:
            raise ValueError('target과 actual 관절 배열 형식이 다릅니다')
        stable_between_samples = False
        if self._previous is not None:
            sample_delta = maximum_joint_error(
                self._previous, current
            )
            stable_between_samples = sample_delta <= self._stable_delta
        inside_goal = maximum_joint_error(target, current) <= self._tolerance
        if inside_goal and stable_between_samples and not robot_is_moving:
            self._consecutive += 1
        else:
            self._consecutive = 0
        self._previous = current
        return self._consecutive >= self._required
