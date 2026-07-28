"""관절 목표 완료가 실제 정지 연속 표본을 요구하는지 검증한다."""

import math

from pinkk_mycobot_bridge.joint_completion import (
    JointStabilityMonitor,
    compensated_joint_command_degrees,
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


def test_compensated_command_uses_bounded_error_direction() -> None:
    """실측 target-actual 방향으로만 제한된 다음 명령을 만든다."""
    command, correction, total = compensated_joint_command_degrees(
        [-2.304, -11.396, -35.214],
        [-2.304, -11.396, -35.214],
        [-1.140, -12.040, -36.560],
        gain=0.8,
        maximum_step_deg=1.0,
        maximum_total_offset_deg=2.0,
    )
    assert command == [-3.235, -10.881, -34.214]
    assert correction == [-0.931, 0.515, 1.0]
    assert total == [-0.931, 0.515, 1.0]


def test_compensated_command_caps_cumulative_offset() -> None:
    """반복 보상 명령은 원래 목표로부터 누적 제한을 넘지 않는다."""
    command, correction, total = compensated_joint_command_degrees(
        [0.0],
        [1.8],
        [-3.0],
        gain=1.0,
        maximum_step_deg=1.0,
        maximum_total_offset_deg=2.0,
    )
    assert command == [2.0]
    assert correction == [0.2]
    assert total == [2.0]
