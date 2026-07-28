"""MoveIt IK DRY RUN의 순수 안전 계산을 검증한다."""

import numpy as np

from pinkk_usb_insertion.control.moveit_ik_safety import (
    make_locked_xy_target,
    pose_error,
    validate_fk_pose,
    validate_motion_guard,
)
from pinkk_usb_insertion.control.pbvs_step_safety import (
    make_fixed_z_xy_waypoints,
)
from pinkk_usb_insertion.geometry.transforms import make_transform

import pytest


def test_locked_target_changes_only_selected_axis() -> None:
    """선택한 축 외의 Z와 자세가 그대로 유지된다."""
    current = make_transform(
        (0.159, -0.066, 0.265),
        (-0.371, 0.929, -0.001, 0.021),
    )
    target = make_locked_xy_target(current, 'x', 0.001, 0.010)
    assert np.isclose(target[0, 3] - current[0, 3], 0.001)
    assert np.isclose(target[1, 3], current[1, 3])
    assert np.isclose(target[2, 3], current[2, 3])
    assert np.allclose(target[:3, :3], current[:3, :3])


def test_locked_target_rejects_over_limit() -> None:
    """설정한 최대 거리를 넘는 목표를 거부한다."""
    with pytest.raises(ValueError, match='제한'):
        make_locked_xy_target(np.eye(4), 'y', -0.011, 0.010)


def test_pose_error_and_fk_validation() -> None:
    """허용 범위 안의 FK 오차를 계산하고 허용한다."""
    expected = np.eye(4)
    actual = make_transform(
        (0.0002, 0.0001, 0.00005),
        (0.0, 0.0, 0.0, 1.0),
    )
    error = validate_fk_pose(expected, actual, 0.001, 0.0005, 0.5)
    assert np.isclose(error.z_error_m, 0.00005)
    assert pose_error(expected, expected).position_error_m == 0.0


def test_fk_validation_rejects_z_error() -> None:
    """Z 오차가 허용값을 넘으면 거부한다."""
    actual = make_transform(
        (0.0, 0.0, 0.0006),
        (0.0, 0.0, 0.0, 1.0),
    )
    with pytest.raises(ValueError, match='Z 오차'):
        validate_fk_pose(np.eye(4), actual, 0.001, 0.0005, 0.5)


def test_pbvs_post_move_rejects_observed_z_drift() -> None:
    """실측된 6.244mm Z 하강은 고정-Z PBVS 완료로 인정하지 않는다."""
    actual = make_transform(
        (0.003152, 0.000489, -0.006244),
        (0.0, 0.0, 0.0, 1.0),
    )
    with pytest.raises(ValueError, match='Z 오차'):
        validate_fk_pose(np.eye(4), actual, 0.015, 0.002, 2.0)


def test_pbvs_post_move_rejects_observed_orientation_drift() -> None:
    """실측된 3.062도 자세 변화는 PBVS 다음 step을 차단한다."""
    angle = np.deg2rad(3.062)
    actual = make_transform(
        (0.0, 0.0, 0.0),
        (np.sin(angle / 2.0), 0.0, 0.0, np.cos(angle / 2.0)),
    )
    with pytest.raises(ValueError, match='자세 오차'):
        validate_fk_pose(np.eye(4), actual, 0.015, 0.002, 2.0)


def test_motion_guard_accepts_progress_with_locked_z() -> None:
    """계획 방향의 정상 진행과 고정 Z를 허용한다."""
    start = np.eye(4)
    target = make_transform((0.003, 0.0, 0.0), (0, 0, 0, 1))
    actual = make_transform((0.001, 0.0001, 0.0002), (0, 0, 0, 1))
    result = validate_motion_guard(
        start,
        target,
        actual,
        maximum_z_change_m=0.001,
        maximum_orientation_change_deg=1.0,
        maximum_xy_overshoot_m=0.0015,
        maximum_opposite_progress_m=0.0005,
    )
    assert np.isclose(result.direction_progress_m, 0.001)


def test_motion_guard_rejects_z_or_opposite_motion() -> None:
    """Z 이탈과 요청 반대 방향 이동을 거부한다."""
    start = np.eye(4)
    target = make_transform((0.003, 0.0, 0.0), (0, 0, 0, 1))
    changed_z = make_transform((0.001, 0.0, -0.0011), (0, 0, 0, 1))
    with pytest.raises(ValueError, match='Z 변화'):
        validate_motion_guard(
            start, target, changed_z, 0.001, 1.0, 0.0015, 0.0005
        )
    opposite = make_transform((-0.0006, 0.0, 0.0), (0, 0, 0, 1))
    with pytest.raises(ValueError, match='반대 방향'):
        validate_motion_guard(
            start, target, opposite, 0.001, 1.0, 0.0015, 0.0005
        )


def test_waypoints_interpolate_small_yaw_and_keep_z() -> None:
    """XY waypoint가 Yaw 목표를 버리지 않고 마지막 자세까지 보간한다."""
    start = np.eye(4)
    angle = np.deg2rad(2.0)
    target = np.eye(4)
    target[:3, :3] = (
        (np.cos(angle), -np.sin(angle), 0.0),
        (np.sin(angle), np.cos(angle), 0.0),
        (0.0, 0.0, 1.0),
    )
    target[:3, 3] = (0.002, 0.0, 0.0)
    waypoints = make_fixed_z_xy_waypoints(start, target, 0.001)
    assert len(waypoints) == 2
    assert np.allclose(waypoints[-1], target)
    assert np.isclose(waypoints[0][2, 3], 0.0)


def test_motion_guard_accepts_yaw_only_target_without_xy_division() -> None:
    """Yaw 전용 step에서도 Z와 XY drift를 정상 감시한다."""
    start = np.eye(4)
    angle = np.deg2rad(2.0)
    target = np.eye(4)
    target[:3, :3] = (
        (np.cos(angle), -np.sin(angle), 0.0),
        (np.sin(angle), np.cos(angle), 0.0),
        (0.0, 0.0, 1.0),
    )
    actual = target.copy()
    result = validate_motion_guard(
        start,
        target,
        actual,
        maximum_z_change_m=0.001,
        maximum_orientation_change_deg=3.0,
        maximum_xy_overshoot_m=0.001,
        maximum_opposite_progress_m=0.0005,
    )
    assert np.isclose(result.direction_progress_m, 0.0)
