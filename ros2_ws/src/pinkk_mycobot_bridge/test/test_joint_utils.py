import math

import pytest

from pinkk_mycobot_bridge.joint_utils import JOINT_NAMES, angles_deg_to_rad


def test_angles_deg_to_rad_converts_six_joints() -> None:
    assert angles_deg_to_rad([0, 90, -90, 180, -180, 45]) == pytest.approx(
        [0, math.pi / 2, -math.pi / 2, math.pi, -math.pi, math.pi / 4]
    )
    assert len(JOINT_NAMES) == 6


@pytest.mark.parametrize(
    'values',
    ([0] * 5, [0, 0, 0, 0, 0, float('nan')], [0, 0, 0, 0, 0, 361]),
)
def test_angles_deg_to_rad_rejects_invalid_values(values) -> None:
    with pytest.raises(ValueError):
        angles_deg_to_rad(values)
