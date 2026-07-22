from geometry_msgs.msg import PoseStamped
import pytest

from pinkk_usb_insertion.cartesian_smoke_test_node import make_xy_offset_target


def _pose() -> PoseStamped:
    message = PoseStamped()
    message.header.frame_id = 'g_base'
    message.pose.position.x = 0.1
    message.pose.position.y = -0.2
    message.pose.position.z = 0.3
    message.pose.orientation.w = 1.0
    return message


def test_x_offset_preserves_other_pose_values() -> None:
    current = _pose()
    target = make_xy_offset_target(current, 'x', 1.0)
    assert target.pose.position.x == pytest.approx(0.101)
    assert target.pose.position.y == pytest.approx(-0.2)
    assert target.pose.position.z == pytest.approx(0.3)
    assert target.pose.orientation.w == pytest.approx(1.0)
    assert current.pose.position.x == pytest.approx(0.1)


def test_negative_y_offset() -> None:
    target = make_xy_offset_target(_pose(), 'y', -5.0)
    assert target.pose.position.x == pytest.approx(0.1)
    assert target.pose.position.y == pytest.approx(-0.205)


@pytest.mark.parametrize('axis', ('z', 'yaw', ''))
def test_rejects_non_xy_axis(axis: str) -> None:
    with pytest.raises(ValueError, match='x 또는 y'):
        make_xy_offset_target(_pose(), axis, 1.0)


@pytest.mark.parametrize('distance', (0.0, 0.05, 10.1, -20.0))
def test_rejects_distance_outside_smoke_test_limit(distance: float) -> None:
    with pytest.raises(ValueError, match='0.1~10.0mm'):
        make_xy_offset_target(_pose(), 'x', distance)
