import numpy as np
from pinkk_usb_insertion.control.yaw_alignment import (
    limited_yaw_step_rad,
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
