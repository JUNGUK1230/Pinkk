import numpy as np
from pinkk_usb_insertion.control.yaw_alignment import (
    apply_base_yaw_step,
    invert_orientation_step,
    joint6_yaw_target_rad,
    keypoint_image_yaw_step_rad,
    keypoint_long_axis_angle_deg,
    limited_yaw_step_rad,
    named_axis_vector,
    undirected_planar_axis_error_rad,
)


def test_undirected_axis_uses_minimum_usb_angle() -> None:
    error = undirected_planar_axis_error_rad(
        np.array((1.0, 0.0, 0.0)),
        np.array((-1.0, 0.0, 0.0)),
    )
    assert np.isclose(error, 0.0)


def test_undirected_axis_does_not_mutate_rotation_column_views() -> None:
    current_rotation = np.array(
        (
            (0.8, -0.6, 0.0),
            (0.6, 0.8, 0.0),
            (0.0, 0.0, 1.0),
        ),
        dtype=np.float64,
    )
    target_rotation = np.array(
        (
            (0.0, -1.0, 0.0),
            (1.0, 0.0, 0.0),
            (0.0, 0.0, 1.0),
        ),
        dtype=np.float64,
    )
    current_before = current_rotation.copy()
    target_before = target_rotation.copy()

    undirected_planar_axis_error_rad(
        current_rotation[:, 0],
        target_rotation[:, 0],
    )

    assert np.array_equal(current_rotation, current_before)
    assert np.array_equal(target_rotation, target_before)


def test_yaw_step_is_limited_to_two_degrees() -> None:
    step, converged = limited_yaw_step_rad(np.deg2rad(10.0), 2.0, 1.0)
    assert np.isclose(np.rad2deg(step), 2.0)
    assert not converged


def test_keypoint_image_yaw_uses_negative_image_feedback_and_limit() -> None:
    step, converged = keypoint_image_yaw_step_rad(
        -13.8,
        0.0,
        10.0,
        1.0,
        -1.0,
    )
    assert np.isclose(np.rad2deg(step), 10.0)
    assert not converged


def test_keypoint_image_yaw_wraps_undirected_axis() -> None:
    step, converged = keypoint_image_yaw_step_rad(
        89.5,
        -89.5,
        10.0,
        2.0,
        -1.0,
    )
    assert np.isclose(np.rad2deg(step), 1.0)
    assert converged


def test_joint6_yaw_target_adds_limited_image_step() -> None:
    target = joint6_yaw_target_rad(
        np.deg2rad(45.0),
        np.deg2rad(6.6),
    )
    assert np.isclose(np.rad2deg(target), 51.6)


def test_joint6_yaw_target_rejects_joint_limit() -> None:
    try:
        joint6_yaw_target_rad(
            np.deg2rad(172.0),
            np.deg2rad(6.0),
        )
    except ValueError as error:
        assert '안전 제한' in str(error)
    else:
        raise AssertionError('joint6 제한 초과를 거부해야 합니다')


def test_base_yaw_step_preserves_position_and_tilt_axis() -> None:
    current = np.eye(4)
    current[:3, 3] = (0.1, -0.2, 0.3)
    target = apply_base_yaw_step(current, np.deg2rad(2.0))
    assert np.allclose(target[:3, 3], current[:3, 3])
    assert np.allclose(
        target[:3, :3] @ np.array((0.0, 0.0, 1.0)),
        current[:3, :3] @ np.array((0.0, 0.0, 1.0)),
    )
    assert np.allclose(named_axis_vector('x'), (1.0, 0.0, 0.0))


def test_inverts_relative_yaw_step_without_changing_position() -> None:
    current = np.eye(4)
    current[:3, 3] = (0.1, -0.2, 0.3)
    positive = apply_base_yaw_step(current, np.deg2rad(2.0))
    inverted = invert_orientation_step(current, positive)
    expected = apply_base_yaw_step(current, np.deg2rad(-2.0))
    assert np.allclose(inverted, expected)


def test_keypoint_long_axis_angle_averages_parallel_edges() -> None:
    points = np.array(
        (
            (10.0, 10.0),
            (30.0, 20.0),
            (28.0, 30.0),
            (8.0, 20.0),
        )
    )
    assert np.isclose(
        keypoint_long_axis_angle_deg(points),
        np.rad2deg(np.arctan2(10.0, 20.0)),
    )


def test_keypoint_axis_is_undirected() -> None:
    forward = np.array(((0.0, 0.0), (10.0, 0.0), (10.0, 5.0), (0.0, 5.0)))
    reversed_axis = np.array(
        ((10.0, 0.0), (0.0, 0.0), (0.0, 5.0), (10.0, 5.0))
    )
    assert np.isclose(keypoint_long_axis_angle_deg(forward), 0.0)
    assert np.isclose(abs(keypoint_long_axis_angle_deg(reversed_axis)), 0.0)
