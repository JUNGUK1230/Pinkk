import numpy as np
from pinkk_usb_insertion.control.pbvs_step_safety import (
    make_fixed_z_xy_waypoints,
    validate_fixed_z_pbvs_step,
    validate_joint_step,
    validate_pbvs_reference_drift,
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


def test_exact_one_millimeter_uses_one_waypoint() -> None:
    """정확한 1mm 요청이 부동소수점 오차로 둘로 분할되지 않는다."""
    current = make_transform((0.158803, -0.068566, 0.262973), (0, 0, 0, 1))
    target = current.copy()
    target[0, 3] += 0.001
    waypoints = make_fixed_z_xy_waypoints(current, target, 0.001)
    assert len(waypoints) == 1


def test_waypoints_restore_locked_reference_z() -> None:
    """누적 Z 이탈을 시작 기준으로 되돌리는 waypoint를 보간한다."""
    current = make_transform((0.1, 0.2, 0.396), (0.0, 0.0, 0.0, 1.0))
    target = make_transform((0.103, 0.2, 0.4), (0.0, 0.0, 0.0, 1.0))
    waypoints = make_fixed_z_xy_waypoints(current, target, 0.001)
    assert len(waypoints) == 5
    assert current[2, 3] < waypoints[0][2, 3] < target[2, 3]
    assert np.allclose(waypoints[-1], target)


def test_reference_drift_accepts_limit_and_rejects_accumulation() -> None:
    """시작 기준 안의 이탈은 허용하고 누적 Z 하강은 거부한다."""
    reference = np.eye(4)
    inside = make_transform((0.0, 0.0, -0.0049), (0.0, 0.0, 0.0, 1.0))
    result = validate_pbvs_reference_drift(reference, inside, 0.005, 2.0)
    assert np.isclose(result.z_error_m, -0.0049)

    outside = make_transform((0.0, 0.0, -0.0051), (0.0, 0.0, 0.0, 1.0))
    with pytest.raises(ValueError, match='누적 이탈'):
        validate_pbvs_reference_drift(reference, outside, 0.005, 2.0)


def test_joint_step_rejects_alternate_ik_branch() -> None:
    current = [0.0] * 6
    safe = [np.deg2rad(2.0)] * 6
    assert np.isclose(validate_joint_step(current, safe, 5.0), 2.0)
    unsafe = safe.copy()
    unsafe[2] = np.deg2rad(20.0)
    with pytest.raises(ValueError, match='관절 점프'):
        validate_joint_step(current, unsafe, 5.0)
