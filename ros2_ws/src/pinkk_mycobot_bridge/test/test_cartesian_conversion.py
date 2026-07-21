import math

from pinkk_mycobot_bridge.cartesian_conversion import (
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
