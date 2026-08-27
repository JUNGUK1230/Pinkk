"""Persistent vehicle identity and topic routing regression test."""

from pathlib import Path
from types import SimpleNamespace
import sys

import numpy as np


SRC_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(SRC_ROOT))

from central_control.vehicle_registry import get_vehicle  # noqa: E402
from central_control.overhead_vision.path_planning.direct_ros_publisher import (  # noqa: E402
    DirectRosPublisher,
    parse_path_target_request,
)
from central_control.overhead_vision.path_planning.lidar_vehicle_association import (  # noqa: E402
    LidarVehicleAssociator,
)
from vehicle_control.heading_fusion import PoseMatch  # noqa: E402


class _FakeLogger:
    def info(self, _message: str) -> None:
        pass

    def warning(self, _message: str) -> None:
        pass


class _FakeNode:
    def get_logger(self) -> _FakeLogger:
        return _FakeLogger()


class _FakePublisher:
    def __init__(self) -> None:
        self.messages: list[object] = []

    def publish(self, message: object) -> None:
        self.messages.append(message)


class _FakeBool:
    def __init__(self) -> None:
        self.data = True


class _FakePoseMatcher:
    def match_pose_near(
        self,
        initial_x_m: float,
        initial_y_m: float,
        scan_points: np.ndarray,
        **_kwargs: object,
    ) -> PoseMatch:
        vehicle_number = int(scan_points[0, 0])
        expected_x = 1.0 if vehicle_number == 1 else 3.0
        score = 0.01 + abs(initial_x_m - expected_x) * 0.1
        return PoseMatch(
            expected_x,
            initial_y_m,
            0.0,
            score,
            0.02,
            50,
        )


def main() -> int:
    first = get_vehicle("vehicle_1")
    second = get_vehicle("vehicle_2")
    assert first.topic("path") == "/pinkk/vehicle_1/path"
    assert second.topic("path") == "/pinkk/vehicle_2/path"
    assert second.topic("cmd_vel") == "/pinkk/vehicle_2/cmd_vel"
    assert second.topic("localization_pose") == "/pinkk/vehicle_2/localization_pose"
    assert second.topic("path_valid") == "/pinkk/vehicle_2/path_valid"
    assert second.topic("odom") == "/pinkk/vehicle_2/odom"
    assert second.topic("scan") == "/pinkk/vehicle_2/scan"
    assert first.topic("path") != second.topic("path")
    assert second.controller_id == "pinky_02"
    assert second.hardware_serial == "PINKY-002"
    try:
        get_vehicle("vehicle_2/../../vehicle_1")
    except ValueError:
        pass
    else:
        raise AssertionError("unknown vehicle IDs must fail closed")

    # ROS runtime 없이 선택 라우팅 자체를 검사한다. 선택 후 invalidation은
    # vehicle_1 publisher가 아니라 새 vehicle_2 publisher로만 가야 한다.
    router = object.__new__(DirectRosPublisher)
    router._active_vehicle_id = "vehicle_1"
    router._node = _FakeNode()
    first_validity = _FakePublisher()
    second_validity = _FakePublisher()
    router._vehicle_publishers = {
        "vehicle_1": {"path_valid": first_validity},
        "vehicle_2": {"path_valid": second_validity},
    }
    router._Bool = _FakeBool
    assert router.trajectory_topic == "/pinkk/vehicle_1/trajectory"
    assert router.select_vehicle("vehicle_2")
    assert not router.select_vehicle("vehicle_2")
    assert router.trajectory_topic == "/pinkk/vehicle_2/trajectory"
    router.invalidate_trajectory()
    assert not first_validity.messages
    assert len(second_validity.messages) == 1
    assert second_validity.messages[0].data is False

    # TRANSIT 경로 재생성은 차량별 마지막 trajectory를 그대로 다시 보내야
    # 하며, 다른 차량의 캐시를 섞으면 안 된다.
    cached_trajectory = (SimpleNamespace(x_cm=1.0), SimpleNamespace(x_cm=2.0))
    router._last_trajectories = {"vehicle_2": cached_trajectory}
    replayed: list[tuple[object, ...]] = []
    router.publish_trajectory = lambda points: replayed.append(tuple(points))
    assert router.republish_last_trajectory("vehicle_2")
    assert replayed == [cached_trajectory]
    assert not router.republish_last_trajectory("vehicle_1")
    try:
        router.select_vehicle("vehicle_3")
    except ValueError:
        pass
    else:
        raise AssertionError("publisher selection must reject unknown vehicle IDs")

    valid_request = (
        '{"vehicle_id":"vehicle_2","robot_id":"vehicle_2",'
        '"controller_id":"pinky_02","hardware_serial":"PINKY-002",'
        '"ros_namespace":"/pinkk/vehicle_2","command":"replan"}'
    )
    assert parse_path_target_request(valid_request) == ("vehicle_2", "replan")
    assert parse_path_target_request(
        '{"vehicle_id":"vehicle_1","command":"entry"}'
    ) == ("vehicle_1", "entry")
    assert parse_path_target_request(
        '{"vehicle_id":"vehicle_1","command":"park"}'
    ) == ("vehicle_1", "park")
    assert parse_path_target_request(
        '{"vehicle_id":"vehicle_1","ros_namespace":"/pinkk/vehicle_2",'
        '"command":"entry"}'
    ) is None
    assert parse_path_target_request(
        '{"vehicle_id":"vehicle_1","command":"emergency_stop"}'
    ) is None
    assert parse_path_target_request("not-json") is None

    associator = LidarVehicleAssociator(
        _FakePoseMatcher(),
        maximum_match_score_m=0.30,
        minimum_assignment_margin_m=0.05,
        required_confirmations=2,
    )
    scans = {
        "vehicle_1": np.asarray(((1.0, 0.0),)),
        "vehicle_2": np.asarray(((2.0, 0.0),)),
    }
    tracks = (
        SimpleNamespace(track_id=7, position_cm=(100.0, 50.0), visible=True),
        SimpleNamespace(track_id=12, position_cm=(300.0, 50.0), visible=True),
    )
    assert associator.associate(scans, tracks, now=1.0) is None
    association = associator.associate(scans, tracks, now=1.1)
    assert association is not None and association.confirmed
    assert association.vehicle_to_track == {"vehicle_1": 7, "vehicle_2": 12}
    print("Vehicle registry and topic routing regression passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
