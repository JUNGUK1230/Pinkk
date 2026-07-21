import numpy as np
from pinkk_usb_insertion.control.pbvs_step_safety import (
    make_fixed_z_xy_waypoints,
    validate_fixed_z_pbvs_step,
    validate_joint_step,
)
from pinkk_usb_insertion.geometry.transforms import make_transform
import pytest


def test_accepts_five_millimeter_fixed_z_step() -> None:
    current = make_transform((0.1, 0.2, 0.4), (0.0, 0.0, 0.0, 1.0))
    target = make_transform((0.103, 0.204, 0.4), (0.0, 0.0, 0.0, 1.0))
    result = validate_fixed_z_pbvs_step(current, target, 0.005, 0.0005, 0.5)
    assert np.isclose(result.xy_distance_m, 0.005)


def test_rejects_step_over_limit() -> None:
    current = np.eye(4)
    target = make_transform((0.006, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    with pytest.raises(ValueError, match='제한'):
        validate_fixed_z_pbvs_step(current, target, 0.005, 0.0005, 0.5)


def test_rejects_z_or_orientation_change() -> None:
    current = np.eye(4)
    changed_z = make_transform((0.001, 0.0, 0.001), (0.0, 0.0, 0.0, 1.0))
    with pytest.raises(ValueError, match='flange Z'):
        validate_fixed_z_pbvs_step(current, changed_z, 0.005, 0.0005, 0.5)

    angle = np.deg2rad(1.0) * 0.5
    rotated = make_transform((0.001, 0.0, 0.0), (0.0, 0.0, np.sin(angle), np.cos(angle)))
    with pytest.raises(ValueError, match='자세'):
        validate_fixed_z_pbvs_step(current, rotated, 0.005, 0.0005, 0.5)


def test_fixed_z_waypoints_preserve_height_and_orientation() -> None:
    current = make_transform((0.1, 0.2, 0.4), (0.0, 0.0, 0.0, 1.0))
    target = make_transform((0.103, 0.204, 0.4), (0.0, 0.0, 0.0, 1.0))
    waypoints = make_fixed_z_xy_waypoints(current, target, 0.001)
    assert len(waypoints) == 5
    assert all(np.isclose(point[2, 3], 0.4) for point in waypoints)
    assert all(np.allclose(point[:3, :3], current[:3, :3]) for point in waypoints)
    assert np.allclose(waypoints[-1], target)


def test_joint_step_rejects_alternate_ik_branch() -> None:
    current = [0.0] * 6
    safe = [np.deg2rad(2.0)] * 6
    assert np.isclose(validate_joint_step(current, safe, 5.0), 2.0)
    unsafe = safe.copy()
    unsafe[2] = np.deg2rad(20.0)
    with pytest.raises(ValueError, match='관절 점프'):
        validate_joint_step(current, unsafe, 5.0)
