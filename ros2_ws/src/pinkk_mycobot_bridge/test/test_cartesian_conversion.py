import math

from pinkk_mycobot_bridge.cartesian_conversion import (
    apply_cartesian_locks,
    is_z_dominant_recovery_motion,
    pose_error,
    pose_values_to_robot_coords,
    quaternion_to_rpy_degrees,
    robot_coords_to_pose_values,
    rpy_degrees_to_quaternion,
    wrapped_angle_difference_deg,
)
import pytest


@pytest.mark.parametrize(
    'rpy',
    [(0.0, 0.0, 0.0), (-180.0, 0.0, -79.27), (10.0, -20.0, 30.0)],
)
def test_rpy_quaternion_round_trip(rpy) -> None:
    quaternion = rpy_degrees_to_quaternion(*rpy)
    restored = quaternion_to_rpy_degrees(quaternion)
    assert restored == pytest.approx(rpy, abs=1e-9)


def test_pose_robot_coords_round_trip() -> None:
    quaternion = rpy_degrees_to_quaternion(-170.0, 5.0, 42.0)
    coords = pose_values_to_robot_coords((0.242, -0.008, 0.150), quaternion)
    position, restored_quaternion = robot_coords_to_pose_values(coords)
    assert position == pytest.approx((0.242, -0.008, 0.150))
    assert pose_error(position, quaternion, position, restored_quaternion) == pytest.approx(
        (0.0, 0.0), abs=1e-9
    )


def test_pose_error_handles_opposite_quaternion_sign() -> None:
    quaternion = rpy_degrees_to_quaternion(1.0, 2.0, 3.0)
    opposite = tuple(-value for value in quaternion)
    assert pose_error((0, 0, 0), quaternion, (0, 0, 0), opposite) == pytest.approx(
        (0.0, 0.0), abs=1e-9
    )


def test_pose_conversion_rejects_invalid_values() -> None:
    with pytest.raises(ValueError):
        pose_values_to_robot_coords((math.nan, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))


def test_wrapped_angle_difference() -> None:
    assert wrapped_angle_difference_deg(179.0, -179.0) == pytest.approx(-2.0)


def test_cartesian_locks_use_hardware_start_z_roll_pitch() -> None:
    """URDF FK가 달라도 고정 축은 MyCobot 시작 자세를 그대로 사용한다."""
    requested = [160.2, -66.0, 265.0, -178.9, 2.2, -136.4]
    start = [159.3, -66.1, 261.6, -178.96, 2.18, -136.45]

    target = apply_cartesian_locks(
        requested,
        start,
        lock_z=True,
        lock_roll_pitch=True,
    )

    assert target == pytest.approx(
        [160.2, -66.0, 261.6, -178.96, 2.18, -136.4]
    )


def test_cartesian_locks_leave_unlocked_axes_requested() -> None:
    requested = [160.2, -66.0, 265.0, -178.9, 2.2, -136.4]
    start = [159.3, -66.1, 261.6, -178.96, 2.18, -136.45]

    target = apply_cartesian_locks(
        requested,
        start,
        lock_z=False,
        lock_roll_pitch=False,
    )

    assert target == pytest.approx(requested)


def test_z_recovery_allows_small_tf_and_hardware_pose_difference() -> None:
    assert is_z_dominant_recovery_motion(
        planned_xy_distance_m=0.0012,
        planned_z_m=0.030,
        planned_yaw_deg=0.4,
        planned_orientation_change_deg=0.5,
        position_tolerance_m=0.003,
        orientation_tolerance_deg=3.0,
    )


def test_z_recovery_rejects_meaningful_xy_motion() -> None:
    assert not is_z_dominant_recovery_motion(
        planned_xy_distance_m=0.006,
        planned_z_m=0.030,
        planned_yaw_deg=0.4,
        planned_orientation_change_deg=0.5,
        position_tolerance_m=0.003,
        orientation_tolerance_deg=3.0,
    )


def test_z_recovery_requires_meaningful_z_motion() -> None:
    assert not is_z_dominant_recovery_motion(
        planned_xy_distance_m=0.001,
        planned_z_m=0.003,
        planned_yaw_deg=0.4,
        planned_orientation_change_deg=0.5,
        position_tolerance_m=0.003,
        orientation_tolerance_deg=3.0,
    )


def test_z_recovery_accepts_partial_final_step_with_lower_threshold() -> None:
    assert is_z_dominant_recovery_motion(
        planned_xy_distance_m=0.001,
        planned_z_m=0.002,
        planned_yaw_deg=0.4,
        planned_orientation_change_deg=0.5,
        position_tolerance_m=0.003,
        orientation_tolerance_deg=3.0,
        minimum_z_motion_m=0.001,
    )


def test_z_recovery_rejects_roll_pitch_correction() -> None:
    """작은 TF Z 차이가 있어도 큰 R/P 명령을 Z-only로 오인하지 않는다."""
    assert not is_z_dominant_recovery_motion(
        planned_xy_distance_m=0.001,
        planned_z_m=0.004,
        planned_yaw_deg=0.2,
        planned_orientation_change_deg=8.0,
        position_tolerance_m=0.005,
        orientation_tolerance_deg=6.0,
        minimum_z_motion_m=0.001,
    )
