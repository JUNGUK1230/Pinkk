"""YOLO 검출부터 Hybrid A* start/goal 입력까지의 mock 회귀 테스트."""

import json
import math
from pathlib import Path
import sys
import tempfile

import cv2
import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parent
CENTRAL_ROOT = PROJECT_ROOT.parent
sys.path.append(str(CENTRAL_ROOT))
sys.path.append(str(PROJECT_ROOT / "src"))

from overhead_vision.localization.scene_localizer import (  # noqa: E402
    AffineBevToLidar,
    Detection,
    EgoVehicleTracker,
    ParkingAssignmentPolicy,
    ParkingSlotMap,
    SceneLocalizer,
    save_scene_observation,
)
from overhead_vision.localization.live_localization import (  # noqa: E402
    ManualHeadingSelector,
    _draw_continuous_path,
)
from vision_scene_input import (  # noqa: E402
    VisionSceneUnavailable,
    load_vision_planning_request,
)


def rectangle(cx: float, cy: float, width: float, height: float) -> np.ndarray:
    return np.asarray(
        [
            [cx - width / 2.0, cy - height / 2.0],
            [cx + width / 2.0, cy - height / 2.0],
            [cx + width / 2.0, cy + height / 2.0],
            [cx - width / 2.0, cy + height / 2.0],
        ],
        dtype=np.float64,
    )


def detection(
    center: tuple[float, float],
    confidence: float,
    width: float = 80.0,
    height: float = 35.0,
    track_id: int | None = None,
) -> Detection:
    polygon = rectangle(center[0], center[1], width, height)
    return Detection(
        "car",
        confidence,
        polygon,
        (
            center[0] - width / 2.0,
            center[1] - height / 2.0,
            center[0] + width / 2.0,
            center[1] + height / 2.0,
        ),
        track_id,
    )


def main() -> int:
    registration = (
        CENTRAL_ROOT
        / "camera_tools"
        / "first_map"
        / "camera_to_lidar_rigid_registration.npz"
    )
    transform = AffineBevToLidar(registration)
    with tempfile.TemporaryDirectory() as temporary_dir:
        temporary = Path(temporary_dir)
        slots_path = temporary / "slots.json"
        slots_path.write_text(
            json.dumps(
                {
                    "occupied_slot": rectangle(500.0, 400.0, 150.0, 160.0).tolist(),
                    "free_slot": rectangle(900.0, 400.0, 150.0, 160.0).tolist(),
                    "C1": rectangle(1200.0, 300.0, 150.0, 160.0).tolist(),
                    "C2": rectangle(1200.0, 600.0, 150.0, 160.0).tolist(),
                }
            ),
            encoding="utf-8",
        )
        ego = detection((120.0, 120.0), 0.80)
        parked_car = detection((500.0, 400.0), 0.99, 100.0, 70.0)
        detections = [parked_car, ego]

        tracker = EgoVehicleTracker(
            transform,
            rear_axle_offset_cm=4.0,
            initial_center_bev_px=(120.0, 120.0),
            initial_yaw_rad=math.radians(40.0),
            position_alpha=1.0,
            yaw_alpha=1.0,
        )
        parking = ParkingSlotMap(
            slots_path,
            transform,
            rear_axle_offset_cm=4.0,
            occupancy_threshold=0.10,
        )
        localizer = SceneLocalizer(tracker, parking)
        scene = localizer.observe(
            detections,
            (800, 1600),
            frame_index=7,
            observed_at_unix_sec=100.0,
        )
        assert scene.vehicle is not None
        assert scene.vehicle.planning_ready
        assert math.hypot(
            scene.vehicle.center_bev_px[0] - 120.0,
            scene.vehicle.center_bev_px[1] - 120.0,
        ) < 1e-9
        assert scene.parking_slots[0].occupied
        assert not scene.parking_slots[1].occupied
        assert scene.planning_request is not None
        assert scene.planning_request.slot_name == "free_slot"
        first_goal, second_goal = scene.parking_slots[1].goal_pose_candidates_cm
        assert abs(
            abs((first_goal[2] - second_goal[2] + math.pi) % (2.0 * math.pi) - math.pi)
            - math.pi
        ) < 1e-9

        scene_path = temporary / "live_vision_scene.json"
        save_scene_observation(scene, scene_path)
        request = load_vision_planning_request(
            scene_path,
            max_age_sec=0.5,
            map_size_cells=(248, 218),
            resolution_cm=1.0,
            now_unix_sec=100.2,
        )
        assert request.frame_index == 7
        assert request.slot_name == "free_slot"
        assert request.start_pose_cm == scene.planning_request.start_pose_cm

        fixed_tracker = EgoVehicleTracker(
            transform,
            rear_axle_offset_cm=4.0,
            initial_center_bev_px=(120.0, 120.0),
            initial_yaw_rad=math.radians(40.0),
            position_alpha=1.0,
            yaw_alpha=1.0,
        )
        fixed_scene = SceneLocalizer(
            fixed_tracker,
            parking,
            target_slot_name="C1",
        ).observe(
            detections,
            (800, 1600),
            frame_index=8,
            observed_at_unix_sec=100.1,
        )
        assert fixed_scene.planning_request is not None
        assert fixed_scene.planning_request.slot_name == "C1"

        # 입구 기준 자동 배정은 허용 칸 중 빈 자리만 남긴 뒤, 가장 먼 칸부터
        # 후보 순서를 만든다. planner는 첫 후보만 받아 불필요한 주차칸 탐색을 줄인다.
        entry_policy = ParkingAssignmentPolicy(
            name="entry_to_parking",
            reference_bev_px=(120.0, 120.0),
            allowed_slots=("C1", "free_slot"),
            preference="farthest",
            candidate_limit=2,
        )
        assignment_tracker = EgoVehicleTracker(
            transform,
            rear_axle_offset_cm=4.0,
            initial_center_bev_px=(120.0, 120.0),
            initial_yaw_rad=math.radians(40.0),
            position_alpha=1.0,
            yaw_alpha=1.0,
        )
        assigned_scene = SceneLocalizer(
            assignment_tracker,
            parking,
            parking_assignment=entry_policy,
        ).observe(
            detections,
            (800, 1600),
            frame_index=8,
            observed_at_unix_sec=100.1,
        )
        assert assigned_scene.planning_request is not None
        assert assigned_scene.planning_request.slot_name == "C1"
        assert assigned_scene.planning_request.candidate_slot_names == (
            "C1",
            "free_slot",
        )
        assert assigned_scene.planning_request.assignment_policy == "entry_to_parking"

        # 가장 먼 C1이 점유되면 다음으로 먼 free_slot으로 즉시 넘어간다.
        occupied_c1 = detection((1200.0, 300.0), 0.99, 100.0, 70.0)
        fallback_tracker = EgoVehicleTracker(
            transform,
            rear_axle_offset_cm=4.0,
            initial_center_bev_px=(120.0, 120.0),
            initial_yaw_rad=math.radians(40.0),
            position_alpha=1.0,
            yaw_alpha=1.0,
        )
        fallback_scene = SceneLocalizer(
            fallback_tracker,
            parking,
            parking_assignment=entry_policy,
        ).observe(
            [parked_car, occupied_c1, ego],
            (800, 1600),
            frame_index=8,
            observed_at_unix_sec=100.1,
        )
        assert fallback_scene.planning_request is not None
        assert fallback_scene.planning_request.slot_name == "free_slot"
        assert fallback_scene.planning_request.candidate_slot_names == ("free_slot",)

        # 충전 단계는 거리와 관계없이 설정 순서대로 C2를 먼저 선택하고,
        # C2가 점유됐을 때만 C1로 대체한다.
        charge_policy = ParkingAssignmentPolicy(
            name="parking_to_charge",
            reference_bev_px=(120.0, 120.0),
            allowed_slots=("C2", "C1"),
            preference="ordered",
            candidate_limit=2,
        )
        charge_tracker = EgoVehicleTracker(
            transform,
            rear_axle_offset_cm=4.0,
            initial_center_bev_px=(120.0, 120.0),
            initial_yaw_rad=math.radians(40.0),
            position_alpha=1.0,
            yaw_alpha=1.0,
        )
        charge_scene = SceneLocalizer(
            charge_tracker,
            parking,
            parking_assignment=charge_policy,
        ).observe(
            detections,
            (800, 1600),
            frame_index=8,
            observed_at_unix_sec=100.1,
        )
        assert charge_scene.planning_request is not None
        assert charge_scene.planning_request.slot_name == "C2"
        assert charge_scene.planning_request.candidate_slot_names == (
            "C2",
            "C1",
        )

        occupied_c2 = detection((1200.0, 600.0), 0.99, 100.0, 70.0)
        charge_fallback_tracker = EgoVehicleTracker(
            transform,
            rear_axle_offset_cm=4.0,
            initial_center_bev_px=(120.0, 120.0),
            initial_yaw_rad=math.radians(40.0),
            position_alpha=1.0,
            yaw_alpha=1.0,
        )
        charge_fallback_scene = SceneLocalizer(
            charge_fallback_tracker,
            parking,
            parking_assignment=charge_policy,
        ).observe(
            [parked_car, occupied_c2, ego],
            (800, 1600),
            frame_index=8,
            observed_at_unix_sec=100.1,
        )
        assert charge_fallback_scene.planning_request is not None
        assert charge_fallback_scene.planning_request.slot_name == "C1"
        assert charge_fallback_scene.planning_request.candidate_slot_names == (
            "C1",
        )

        occupied_scene = SceneLocalizer(
            fixed_tracker,
            parking,
            target_slot_name="occupied_slot",
        ).observe(
            detections,
            (800, 1600),
            frame_index=9,
            observed_at_unix_sec=100.2,
        )
        assert occupied_scene.planning_request is None
        assert "occupied" in occupied_scene.status

        try:
            load_vision_planning_request(
                scene_path,
                max_age_sec=0.5,
                now_unix_sec=101.0,
            )
        except VisionSceneUnavailable as error:
            assert "stale" in str(error)
        else:
            raise AssertionError("stale scene must be rejected")

        ambiguous_tracker = EgoVehicleTracker(
            transform,
            rear_axle_offset_cm=4.0,
        )
        ambiguous_scene = SceneLocalizer(ambiguous_tracker, parking).observe(
            detections,
            (800, 1600),
            frame_index=8,
            observed_at_unix_sec=101.0,
        )
        assert ambiguous_scene.vehicle is not None
        assert ambiguous_scene.vehicle.ego_selection_ambiguous
        assert ambiguous_scene.vehicle.heading_ambiguous
        assert not ambiguous_scene.planning_ready

        heading_selector = ManualHeadingSelector(ambiguous_tracker, transform)
        heading_selector.update_scene(ambiguous_scene)
        heading_selector.arm()
        center_x, center_y = ambiguous_scene.vehicle.center_bev_px
        heading_selector.mouse_callback(
            cv2.EVENT_LBUTTONDOWN,
            round(center_x + 100.0),
            round(center_y),
            0,
            None,
        )
        assert ambiguous_tracker.manual_yaw_rad is not None

        ambiguous_tracker.set_manual_heading(math.radians(-25.0))
        manual_scene = SceneLocalizer(
            ambiguous_tracker,
            parking,
            target_slot_name="C1",
        ).observe(
            detections,
            (800, 1600),
            frame_index=9,
            observed_at_unix_sec=101.1,
        )
        assert manual_scene.vehicle is not None
        assert not manual_scene.vehicle.heading_ambiguous
        assert not manual_scene.vehicle.ego_selection_ambiguous
        assert math.isclose(
            manual_scene.vehicle.yaw_rad,
            math.radians(-25.0),
            abs_tol=1e-9,
        )
        assert manual_scene.planning_request is not None
        assert manual_scene.planning_request.slot_name == "C1"

        # ByteTrack ID가 생긴 뒤에는 이전 중심에 더 가까운 다른 차량이 있어도
        # 첫 ego ID를 계속 선택한다. ID가 사라지면 다른 차량으로 바꾸지 않는다.
        id_tracker = EgoVehicleTracker(
            transform,
            rear_axle_offset_cm=4.0,
            initial_center_bev_px=(120.0, 120.0),
            initial_yaw_rad=math.radians(40.0),
            position_alpha=1.0,
            yaw_alpha=1.0,
        )
        first_ego = detection((120.0, 120.0), 0.80, track_id=7)
        tracked_ego = id_tracker.update([first_ego])
        assert tracked_ego is not None and tracked_ego.track_id == 7
        moved_ego = detection((200.0, 120.0), 0.80, track_id=7)
        nearby_other = detection((121.0, 120.0), 0.99, track_id=11)
        tracked_ego = id_tracker.update([nearby_other, moved_ego])
        assert tracked_ego is not None
        assert tracked_ego.track_id == 7
        assert math.isclose(tracked_ego.center_bev_px[0], 200.0, abs_tol=1e-9)
        assert id_tracker.update([nearby_other]) is None

        line_image = np.zeros((100, 120, 3), dtype=np.uint8)
        skipped = _draw_continuous_path(
            line_image,
            ((10.0, 10.0), (40.0, 40.0), (80.0, 50.0)),
        )
        assert skipped == 0
        assert np.count_nonzero(line_image[:, :, 2]) > 0

        ambiguous_path = temporary / "ambiguous_scene.json"
        save_scene_observation(ambiguous_scene, ambiguous_path)
        try:
            load_vision_planning_request(
                ambiguous_path,
                now_unix_sec=101.1,
            )
        except VisionSceneUnavailable as error:
            assert "not planning-ready" in str(error)
        else:
            raise AssertionError("ambiguous ego pose must be rejected")

    print("Live vision scene regression passed")
    print(f"Ego rear axle: {scene.vehicle.rear_axle_cm}")
    print(f"Ego yaw: {math.degrees(scene.vehicle.yaw_rad):.3f} deg")
    print(f"Selected free slot: {scene.planning_request.slot_name}")
    print(f"Planning start: {scene.planning_request.start_pose_cm}")
    print(f"Planning goal: {scene.planning_request.goal_pose_cm}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
