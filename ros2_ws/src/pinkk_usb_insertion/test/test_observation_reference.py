"""OBSERVE_POSE 자동 기준 저장 전 관절 검사를 검증한다."""

import numpy as np
import pytest

from pinkk_usb_insertion.control.observation_reference import (
    observation_joint_errors_deg,
    validate_observation_joint_pose,
)


def test_observation_pose_accepts_small_hardware_error() -> None:
    reference_deg = (-1.66, -8.08, -36.65, -39.9, 0.0, 45.0)
    actual_rad = np.radians((-1.5, -9.0, -38.0, -42.0, 0.5, 46.0))
    maximum = validate_observation_joint_pose(actual_rad, reference_deg, 3.0)
    assert np.isclose(maximum, 2.1)


def test_observation_pose_rejects_wrong_start_pose() -> None:
    with pytest.raises(ValueError, match='OBSERVE_POSE가 아닙니다'):
        validate_observation_joint_pose(
            np.radians((10.0, 0.0)),
            (0.0, 0.0),
            3.0,
        )


def test_observation_joint_error_wraps_at_180_degrees() -> None:
    errors = observation_joint_errors_deg(
        np.radians((-179.0,)),
        (179.0,),
    )
    assert np.allclose(errors, (2.0,))
