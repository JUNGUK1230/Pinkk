"""관절 목표 완료가 실제 정지 연속 표본을 요구하는지 검증한다."""

import math

from pinkk_mycobot_bridge.joint_completion import (
    JointStabilityMonitor,
    maximum_joint_error,
    signed_joint_errors_degrees,
)


def test_maximum_joint_error_wraps_at_pi_boundary() -> None:
    """±180도 경계를 지나는 관절 오차는 최단 거리로 계산한다."""
    error = maximum_joint_error(
        [math.radians(179.0)],
        [math.radians(-179.0)],
    )
    assert math.isclose(error, math.radians(2.0))


def test_requires_consecutive_stopped_samples() -> None:
    """목표 안에서 실제 정지한 연속 표본만 완료 횟수로 센다."""
    monitor = JointStabilityMonitor(
        tolerance_rad=math.radians(1.0),
        stable_delta_rad=math.radians(0.2),
        required_samples=3,
    )
    target = [0.0, 0.0]

    assert monitor.update(target, [0.0, 0.0], robot_is_moving=True) is False
    assert monitor.update(target, [0.0, 0.0], robot_is_moving=False) is False
    assert monitor.update(target, [0.0, 0.0], robot_is_moving=False) is False
    assert monitor.update(target, [0.0, 0.0], robot_is_moving=False) is True


def test_motion_or_drift_resets_stability_count() -> None:
    """이동 중 응답이나 표본간 큰 변화가 정지 확인 횟수를 초기화한다."""
    monitor = JointStabilityMonitor(
        tolerance_rad=math.radians(1.0),
        stable_delta_rad=math.radians(0.2),
        required_samples=2,
    )
    target = [0.0]

    assert monitor.update(target, [0.0], robot_is_moving=False) is False
    assert monitor.update(target, [0.0], robot_is_moving=False) is False
    assert monitor.consecutive_samples == 1
    assert monitor.update(target, [0.0], robot_is_moving=True) is False
    assert monitor.consecutive_samples == 0
    assert monitor.update(
        target, [math.radians(0.8)], robot_is_moving=False
    ) is False
    assert monitor.consecutive_samples == 0


def test_signed_joint_errors_degrees_preserves_direction() -> None:
    """관절별 오차가 target-actual 방향을 유지한다."""
    target = [math.radians(1.0), math.radians(-2.0)]
    actual = [math.radians(0.5), math.radians(-1.0)]
    errors = signed_joint_errors_degrees(target, actual)
    assert math.isclose(errors[0], 0.5)
    assert math.isclose(errors[1], -1.0)
