import numpy as np

from pinkk_usb_insertion.geometry.transforms import (
    approach_transform,
    compose,
    inverse,
    make_transform,
)


def test_transform_inverse_returns_identity() -> None:
    transform = make_transform((0.1, -0.2, 0.3), (0.0, 0.0, 0.0, 1.0))
    assert np.allclose(compose(transform, inverse(transform)), np.eye(4))


def test_approach_is_negative_port_z() -> None:
    transform = approach_transform(0.1)
    assert np.allclose(transform[:3, 3], (0.0, 0.0, -0.1))
