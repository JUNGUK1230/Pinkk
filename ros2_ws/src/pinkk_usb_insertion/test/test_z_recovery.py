import numpy as np

from pinkk_usb_insertion.control.z_recovery import raised_z_recovery_target
from pinkk_usb_insertion.geometry.transforms import rpy_degrees_to_rotation


def test_z_recovery_preserves_xy_and_orientation() -> None:
    current = np.eye(4)
    current[:3, 3] = (0.23, -0.06, 0.218)
    current[:3, :3] = rpy_degrees_to_rotation(-179.0, 2.0, -132.0)
    observation = np.eye(4)
    observation[2, 3] = 0.262

    target = raised_z_recovery_target(current, observation, 0.030)

    assert np.allclose(target[:2, 3], current[:2, 3])
    assert np.allclose(target[:3, :3], current[:3, :3])
    assert np.isclose(target[2, 3], 0.248)


def test_z_recovery_stops_at_observation_z() -> None:
    current = np.eye(4)
    current[2, 3] = 0.248
    observation = np.eye(4)
    observation[2, 3] = 0.262

    target = raised_z_recovery_target(current, observation, 0.030)

    assert np.isclose(target[2, 3], 0.262)


def test_z_recovery_rejects_when_observation_is_not_higher() -> None:
    current = np.eye(4)
    current[2, 3] = 0.262
    observation = np.eye(4)
    observation[2, 3] = 0.250

    try:
        raised_z_recovery_target(current, observation, 0.030)
    except ValueError as error:
        assert '높지 않아' in str(error)
    else:
        raise AssertionError('상승 가능한 Z가 없으면 복구를 거부해야 합니다')
