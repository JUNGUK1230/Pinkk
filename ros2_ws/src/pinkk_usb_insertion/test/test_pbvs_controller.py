import numpy as np

from pinkk_usb_insertion.control.pbvs_controller import (
    calculate_absolute_port_xy_yaw_target,
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


def test_absolute_port_target_uses_port_xy_and_observe_z() -> None:
    current = make_transform((0.10, -0.20, 0.39), (0.0, 0.0, 0.0, 1.0))
    locked = make_transform((0.09, -0.19, 0.40), (0.0, 0.0, 0.0, 1.0))
    half_yaw = np.deg2rad(15.0)
    port = make_transform(
        (0.14, -0.12, 0.20),
        (0.0, 0.0, np.sin(half_yaw), np.cos(half_yaw)),
    )

    result = calculate_absolute_port_xy_yaw_target(
        current,
        locked,
        port,
        np.array((1.0, 0.0, 0.0)),
        np.array((1.0, 0.0, 0.0)),
        0.003,
        1.0,
    )

    assert np.allclose(result.target_base_to_flange[:3, 3], (0.14, -0.12, 0.40))
    assert np.allclose(result.delta_base_xy_m, (0.04, 0.08))
    assert np.isclose(np.rad2deg(result.yaw_error_rad), 30.0)
    assert np.isclose(np.rad2deg(result.target_yaw_offset_rad), 30.0)
    assert not result.converged


def test_absolute_port_target_chooses_nearest_undirected_yaw() -> None:
    current = np.eye(4)
    locked = np.eye(4)
    port = make_transform(
        (0.0, 0.0, 0.2),
        (0.0, 0.0, np.sin(np.deg2rad(85.0)), np.cos(np.deg2rad(85.0))),
    )

    result = calculate_absolute_port_xy_yaw_target(
        current,
        locked,
        port,
        np.array((1.0, 0.0, 0.0)),
        np.array((1.0, 0.0, 0.0)),
        0.003,
        1.0,
    )

    assert np.isclose(np.rad2deg(result.yaw_error_rad), -10.0)
    assert np.isclose(np.rad2deg(result.target_yaw_offset_rad), -10.0)


def test_absolute_port_target_reports_current_yaw_residual() -> None:
    locked = np.eye(4)
    current = make_transform(
        (0.1, 0.2, 0.4),
        (0.0, 0.0, np.sin(np.deg2rad(15.0)), np.cos(np.deg2rad(15.0))),
    )
    port = current.copy()

    result = calculate_absolute_port_xy_yaw_target(
        current,
        locked,
        port,
        np.array((1.0, 0.0, 0.0)),
        np.array((1.0, 0.0, 0.0)),
        0.003,
        1.0,
    )

    assert np.isclose(np.rad2deg(result.yaw_error_rad), 0.0)
    assert np.isclose(np.rad2deg(result.target_yaw_offset_rad), 30.0)
    assert result.converged


def test_absolute_port_target_uses_port_pre_approach_height() -> None:
    current = make_transform((0.10, -0.20, 0.40), (0.0, 0.0, 0.0, 1.0))
    locked = current.copy()
    port = make_transform((0.14, -0.12, 0.065), (0.0, 0.0, 0.0, 1.0))

    result = calculate_absolute_port_xy_yaw_target(
        current,
        locked,
        port,
        np.array((1.0, 0.0, 0.0)),
        np.array((1.0, 0.0, 0.0)),
        0.003,
        1.0,
        0.050,
    )

    assert np.allclose(result.target_base_to_flange[:3, 3], (0.14, -0.12, 0.115))
