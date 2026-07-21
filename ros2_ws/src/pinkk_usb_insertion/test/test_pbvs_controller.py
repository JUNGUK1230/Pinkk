import numpy as np

from pinkk_usb_insertion.control.pbvs_controller import (
    calculate_fixed_z_camera_pbvs,
)
from pinkk_usb_insertion.geometry.transforms import make_transform


def test_fixed_z_pbvs_limits_step_and_preserves_z_and_orientation() -> None:
    base_to_flange = make_transform((0.1, -0.2, 0.4), (0.0, 0.0, 0.0, 1.0))
    camera_to_port = make_transform((0.02, 0.0, 0.25), (0.0, 0.0, 0.0, 1.0))
    result = calculate_fixed_z_camera_pbvs(
        base_to_flange, np.eye(4), camera_to_port, 0.005, 0.003
    )

    assert np.allclose(result.applied_step_base_xy_m, (0.005, 0.0))
    assert np.isclose(result.target_base_to_flange[2, 3], 0.4)
    assert np.allclose(
        result.target_base_to_flange[:3, :3], base_to_flange[:3, :3]
    )
    assert not result.converged


def test_camera_axis_error_is_rotated_into_base_xy() -> None:
    half_angle = np.pi / 4.0
    base_to_flange = make_transform(
        (0.0, 0.0, 0.4), (0.0, 0.0, np.sin(half_angle), np.cos(half_angle))
    )
    camera_to_port = make_transform((0.004, 0.0, 0.2), (0.0, 0.0, 0.0, 1.0))
    result = calculate_fixed_z_camera_pbvs(
        base_to_flange, np.eye(4), camera_to_port, 0.01, 0.001
    )

    assert np.allclose(result.error_base_xy_m, (0.0, 0.004), atol=1e-12)
    assert np.allclose(result.applied_step_base_xy_m, (0.0, 0.004), atol=1e-12)


def test_pbvs_reports_convergence_inside_tolerance_and_composes_port_pose() -> None:
    base_to_flange = make_transform((0.1, 0.2, 0.3), (0.0, 0.0, 0.0, 1.0))
    flange_to_camera = make_transform((0.01, -0.02, 0.03), (0.0, 0.0, 0.0, 1.0))
    camera_to_port = make_transform((0.001, -0.001, 0.2), (0.0, 0.0, 0.0, 1.0))
    result = calculate_fixed_z_camera_pbvs(
        base_to_flange, flange_to_camera, camera_to_port, 0.005, 0.003
    )

    assert result.converged
    assert np.allclose(result.base_to_port[:3, 3], (0.111, 0.179, 0.53))


def test_pbvs_reuses_initial_z_and_orientation_lock() -> None:
    current = make_transform((0.1, 0.2, 0.395), (0.0, 0.0, 0.1, 0.995))
    locked = make_transform((0.0, 0.0, 0.4), (0.0, 0.0, 0.0, 1.0))
    camera_to_port = make_transform((0.01, 0.0, 0.2), (0.0, 0.0, 0.0, 1.0))
    result = calculate_fixed_z_camera_pbvs(
        current,
        np.eye(4),
        camera_to_port,
        0.005,
        0.003,
        locked_base_to_flange=locked,
    )
    assert np.isclose(result.target_base_to_flange[2, 3], 0.4)
    assert np.allclose(result.target_base_to_flange[:3, :3], np.eye(3))
