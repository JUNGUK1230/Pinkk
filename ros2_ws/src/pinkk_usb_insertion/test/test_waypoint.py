import numpy as np

from pinkk_usb_insertion.control.waypoint import limited_waypoint_translation


def test_waypoint_clips_xy_to_maximum_step() -> None:
    current = np.array((0.20, 0.00, 0.25))
    target = np.array((0.20 + 0.030, 0.00 + 0.040, 0.25))  # 50mm away

    waypoint, xy_distance, skipped = limited_waypoint_translation(
        current, target, 0.015, 0.010, 0.003
    )

    assert not skipped
    assert np.isclose(xy_distance, 0.050)
    moved = np.linalg.norm(waypoint[:2] - current[:2])
    assert np.isclose(moved, 0.015)
    # 방향은 유지되어야 한다.
    direction = (target[:2] - current[:2]) / xy_distance
    assert np.allclose(
        (waypoint[:2] - current[:2]) / moved, direction
    )


def test_waypoint_does_not_overshoot_close_target() -> None:
    current = np.array((0.20, 0.00, 0.25))
    target = np.array((0.204, 0.003, 0.25))  # 5mm away, under 15mm cap

    waypoint, xy_distance, skipped = limited_waypoint_translation(
        current, target, 0.015, 0.010, 0.003
    )

    assert not skipped
    assert np.allclose(waypoint[:2], target[:2])


def test_waypoint_skips_xy_below_minimum_step() -> None:
    current = np.array((0.20, 0.00, 0.25))
    target = np.array((0.2015, 0.00, 0.25))  # 1.5mm away, under 3mm minimum

    waypoint, xy_distance, skipped = limited_waypoint_translation(
        current, target, 0.015, 0.010, 0.003
    )

    assert skipped
    assert np.allclose(waypoint[:2], current[:2])


def test_waypoint_clips_z_independently_of_xy() -> None:
    current = np.array((0.20, 0.00, 0.25))
    target = np.array((0.20, 0.00, 0.25 + 0.030))  # 30mm Z, no XY motion

    waypoint, xy_distance, skipped = limited_waypoint_translation(
        current, target, 0.015, 0.010, 0.003
    )

    assert skipped
    assert np.isclose(waypoint[2], current[2] + 0.010)


def test_waypoint_z_direction_can_be_negative() -> None:
    current = np.array((0.20, 0.00, 0.25))
    target = np.array((0.20, 0.00, 0.25 - 0.030))

    waypoint, _, _ = limited_waypoint_translation(
        current, target, 0.015, 0.010, 0.003
    )

    assert np.isclose(waypoint[2], current[2] - 0.010)


def test_waypoint_rejects_non_positive_step_limits() -> None:
    current = np.array((0.0, 0.0, 0.0))
    target = np.array((0.1, 0.0, 0.0))

    try:
        limited_waypoint_translation(current, target, 0.0, 0.010, 0.003)
    except ValueError as error:
        assert 'step' in str(error)
    else:
        raise AssertionError('0 이하의 step 제한은 거부해야 합니다')
