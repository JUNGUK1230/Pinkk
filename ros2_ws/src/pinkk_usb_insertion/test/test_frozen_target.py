import numpy as np
import pytest

from pinkk_usb_insertion.control.frozen_target import (
    limited_xy_target,
    port_based_flange_target_z,
    proportional_xy_target,
    proportional_z_descent_m,
    xy_residual_m,
)


def test_port_based_flange_target_z_uses_tcp_and_insertion_depth() -> None:
    assert port_based_flange_target_z(0.070, 0.100, 0.010) == pytest.approx(
        0.160
    )


def test_port_based_flange_target_z_rejects_depth_beyond_tcp() -> None:
    with pytest.raises(ValueError):
        port_based_flange_target_z(0.070, 0.100, 0.110)


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
