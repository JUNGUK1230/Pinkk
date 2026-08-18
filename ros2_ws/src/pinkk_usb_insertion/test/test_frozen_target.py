import numpy as np
import pytest

from pinkk_usb_insertion.control.frozen_target import (
    circular_mean_degrees,
    circular_median_degrees,
    final_insertion_target_z_m,
    flange_xy_for_tcp_lateral_offset,
    limited_xy_target,
    maximum_angular_deviation_degrees,
    port_based_flange_target_z,
    proportional_xy_target,
    proportional_z_descent_m,
    xy_residual_m,
)


def test_circular_mean_degrees_handles_wrap_boundary() -> None:
    mean = circular_mean_degrees((179.0, -179.0, 178.0, -178.0))
    assert abs(abs(mean) - 180.0) < 1e-6


def test_circular_median_degrees_rejects_angle_outlier() -> None:
    median = circular_median_degrees((6.8, 7.0, 7.1, 6.9, 40.0))
    assert median == pytest.approx(7.0)


def test_circular_median_degrees_handles_wrap_boundary() -> None:
    median = circular_median_degrees((179.0, -179.0, 178.0, -178.0, 177.0))
    assert median == pytest.approx(179.0)


def test_maximum_angular_deviation_uses_shortest_wrap() -> None:
    assert maximum_angular_deviation_degrees((179.0, -179.0), 180.0) == pytest.approx(
        1.0
    )


def test_final_insertion_target_uses_relative_10mm() -> None:
    assert final_insertion_target_z_m(0.170, 0.010, 0.154) == pytest.approx(
        0.160
    )


def test_final_insertion_target_clamps_at_configured_port_target() -> None:
    assert final_insertion_target_z_m(0.160, 0.010, 0.154) == pytest.approx(
        0.154
    )


def test_final_insertion_target_rejects_target_above_guard() -> None:
    with pytest.raises(ValueError):
        final_insertion_target_z_m(0.150, 0.010, 0.154)


def test_port_based_flange_target_z_uses_tcp_and_insertion_depth() -> None:
    assert port_based_flange_target_z(0.070, 0.100, 0.010) == pytest.approx(
        0.160
    )


def test_port_based_flange_target_z_rejects_depth_beyond_tcp() -> None:
    with pytest.raises(ValueError):
        port_based_flange_target_z(0.070, 0.100, 0.110)


def test_tcp_positive_x_moves_flange_negative_x_at_zero_yaw() -> None:
    target = flange_xy_for_tcp_lateral_offset(
        (0.200, -0.060),
        np.eye(3),
        0.005,
        0.0,
    )
    assert np.allclose(target, (0.195, -0.060))


def test_tcp_local_x_rotates_with_flange_yaw() -> None:
    yaw_90 = np.array(
        [
            [0.0, -1.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
        ]
    )
    target = flange_xy_for_tcp_lateral_offset(
        (0.200, -0.060),
        yaw_90,
        -0.005,
        0.0,
    )
    assert np.allclose(target, (0.200, -0.055))


def test_xy_residual() -> None:
    assert np.isclose(xy_residual_m((0.0, 0.0), (0.003, 0.004)), 0.005)


def test_limited_xy_target_clamps_in_target_direction() -> None:
    target, distance = limited_xy_target((0.0, 0.0), (0.03, 0.04), 0.01)

    assert np.isclose(distance, 0.05)
    assert np.allclose(target, (0.006, 0.008))


def test_limited_xy_target_uses_near_target_without_overshoot() -> None:
    target, distance = limited_xy_target((0.01, -0.02), (0.014, -0.02), 0.01)

    assert np.isclose(distance, 0.004)
    assert np.allclose(target, (0.014, -0.02))


def test_proportional_xy_target_applies_gain() -> None:
    target, distance = proportional_xy_target(
        (0.0, 0.0), (0.03, 0.04), 0.7, 0.1, 0.003
    )

    assert np.isclose(distance, 0.05)
    assert np.allclose(target, (0.021, 0.028))


def test_proportional_xy_target_respects_minimum_without_overshoot() -> None:
    target, distance = proportional_xy_target(
        (0.0, 0.0), (0.004, 0.0), 0.5, 0.02, 0.003
    )

    assert np.isclose(distance, 0.004)
    assert np.allclose(target, (0.003, 0.0))


def test_proportional_z_descent_applies_gain() -> None:
    assert proportional_z_descent_m(0.020, 0.4, 0.020, 0.002) == pytest.approx(
        0.008
    )


def test_proportional_z_descent_clamps_maximum() -> None:
    assert proportional_z_descent_m(0.050, 0.5, 0.005, 0.002) == pytest.approx(
        0.005
    )


def test_proportional_z_descent_does_not_overshoot_remaining() -> None:
    assert proportional_z_descent_m(0.001, 0.4, 0.005, 0.002) == pytest.approx(
        0.001
    )
