"""MoveIt IK DRY RUN의 순수 안전 계산을 검증한다."""

import numpy as np

from pinkk_usb_insertion.control.moveit_ik_safety import (
    make_locked_xy_target,
    pose_error,
    validate_fk_pose,
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
