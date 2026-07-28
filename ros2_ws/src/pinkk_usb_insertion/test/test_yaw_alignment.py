import numpy as np
from pinkk_usb_insertion.control.yaw_alignment import (
    apply_base_yaw_step,
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


def test_yaw_step_is_limited_to_two_degrees() -> None:
    step, converged = limited_yaw_step_rad(np.deg2rad(10.0), 2.0, 1.0)
    assert np.isclose(np.rad2deg(step), 2.0)
    assert not converged


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
