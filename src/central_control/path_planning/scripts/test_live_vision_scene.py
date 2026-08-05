"""YOLO 검출부터 고정 경로 section/target 입력까지의 mock 회귀 테스트."""

import json
import math
from pathlib import Path
import sys
import tempfile

import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parent
CENTRAL_ROOT = PROJECT_ROOT.parent
sys.path.append(str(CENTRAL_ROOT))
sys.path.append(str(PROJECT_ROOT / "src"))

from overhead_vision.localization.scene_localizer import (  # noqa: E402
    AffineBevToLidar,
    ChargeEpisodeCoordinator,
    Detection,
    EgoVehicleTracker,
    ParkingAssignmentPolicy,
    ParkingSlotMap,
    SceneLocalizer,
    VehicleStateManager,
    save_scene_observation,
)
from overhead_vision.localization.live_localization import _draw_continuous_path  # noqa: E402
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
                    "P1": rectangle(1500.0, 600.0, 80.0, 120.0).tolist(),
                    "P2": rectangle(1400.0, 600.0, 80.0, 120.0).tolist(),
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

        # 사용자가 e 키를 누르는 동작은 화면에 보이는 ByteTrack ID를
        # 순환한다. 차량별 position filter 이력을 섞지 않고 즉시 새 중심을 쓴다.
        switch_tracker = EgoVehicleTracker(
            transform,
            rear_axle_offset_cm=4.0,
            initial_center_bev_px=(120.0, 120.0),
            initial_yaw_rad=math.radians(40.0),
            position_alpha=0.5,
            yaw_alpha=1.0,
        )
        first_car = detection((120.0, 120.0), 0.9, track_id=1)
        second_car = detection((320.0, 220.0), 0.8, track_id=2)
        selected_first = switch_tracker.update([first_car, second_car])
        assert selected_first is not None and selected_first.track_id == 1
        changed, message = switch_tracker.select_next_ego()
        assert changed and "1 -> 2" in message
        selected_second = switch_tracker.update([first_car, second_car])
        assert selected_second is not None and selected_second.track_id == 2
        assert selected_second.center_bev_px == (320.0, 220.0)
        changed, message = switch_tracker.select_next_ego()
        assert changed and "2 -> 1" in message
        selected_first_again = switch_tracker.update([first_car, second_car])
        assert selected_first_again is not None
        assert selected_first_again.track_id == 1

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

        # 운행 중인 ego mask가 목표 슬롯 가장자리를 10% 이상 스쳐도 그 차량
        # 자신 때문에 목표 칸이 occupied로 바뀌거나 경로가 사라지면 안 된다.
        grazing_ego = detection(
            (1115.0, 600.0),
            0.95,
            100.0,
            70.0,
            track_id=31,
        )
        raw_grazing_slots = parking.observe([grazing_ego], (800, 1600))
        raw_c2 = next(slot for slot in raw_grazing_slots if slot.name == "C2")
        assert raw_c2.occupied
        grazing_tracker = EgoVehicleTracker(
            transform,
            rear_axle_offset_cm=4.0,
            initial_center_bev_px=(1115.0, 600.0),
            initial_yaw_rad=math.radians(40.0),
            position_alpha=1.0,
            yaw_alpha=1.0,
        )
        grazing_scene = SceneLocalizer(
            grazing_tracker,
            parking,
            target_slot_name="C2",
        ).observe(
            [grazing_ego],
            (800, 1600),
            frame_index=8,
            observed_at_unix_sec=100.1,
        )
        filtered_c2 = next(
            slot for slot in grazing_scene.parking_slots if slot.name == "C2"
        )
        assert not filtered_c2.occupied
        assert grazing_scene.planning_request is not None
        assert grazing_scene.planning_request.slot_name == "C2"

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

        # 차량 운영 상태는 track_id와 차량 중심이 포함된 주차칸으로 분류한다.
        # C2 안의 차량은 charging, 주차칸 밖의 차량은 entry_or_transit 상태다.
        state_manager = VehicleStateManager(transform, track_ttl_sec=0.5)
        state_tracker = EgoVehicleTracker(
            transform,
            rear_axle_offset_cm=4.0,
            initial_center_bev_px=(120.0, 120.0),
            initial_yaw_rad=math.radians(40.0),
            position_alpha=1.0,
            yaw_alpha=1.0,
        )
        state_scene = SceneLocalizer(
            state_tracker,
            parking,
            vehicle_state_manager=state_manager,
        ).observe(
            [
                detection((120.0, 120.0), 0.80, track_id=7),
                detection((1200.0, 600.0), 0.95, 100.0, 70.0, track_id=22),
            ],
            (800, 1600),
            frame_index=10,
            observed_at_unix_sec=102.0,
        )
        states = {item.track_id: item for item in state_scene.tracked_vehicles}
        assert states[7].state == "entry_or_transit"
        assert states[7].visible
        assert states[22].state == "charging"
        assert states[22].assigned_slot_name == "C2"

        # 짧은 가림에서는 마지막 상태를 lost로 유지하고 TTL을 넘으면 제거한다.
        retained_scene = SceneLocalizer(
            state_tracker,
            parking,
            vehicle_state_manager=state_manager,
        ).observe(
            [detection((121.0, 120.0), 0.80, track_id=7)],
            (800, 1600),
            frame_index=11,
            observed_at_unix_sec=102.2,
        )
        retained = {
            item.track_id: item for item in retained_scene.tracked_vehicles
        }
        assert not retained[22].visible
        expired_scene = SceneLocalizer(
            state_tracker,
            parking,
            vehicle_state_manager=state_manager,
        ).observe(
            [detection((122.0, 120.0), 0.80, track_id=7)],
            (800, 1600),
            frame_index=12,
            observed_at_unix_sec=102.8,
        )
        assert {item.track_id for item in expired_scene.tracked_vehicles} == {7}

        # 충전 Episode는 먼저 보인 eligible 차량을 FIFO로 선택하고 C2를 먼저
        # 사용한다. C2가 점유되면 C1로 대체하고, 둘 다 점유되면 대기 상태다.
        coordinator = ChargeEpisodeCoordinator(("C2", "C1"))
        entry_vehicle = states[7]
        slots_free = parking.observe([], (800, 1600))
        assignment = coordinator.observe([entry_vehicle], slots_free)
        assert assignment is not None
        assert assignment.vehicle_track_id == 7
        assert assignment.target_slot_name == "C2"
        assert assignment.status == "assigned_to_charge"

        c2_occupied_slots = parking.observe(
            [detection((1200.0, 600.0), 0.99, 100.0, 70.0, track_id=22)],
            (800, 1600),
        )
        assignment = coordinator.observe([entry_vehicle], c2_occupied_slots)
        assert assignment is not None
        assert assignment.target_slot_name == "C1"
        assert assignment.status == "assigned_to_charge"

        both_charge_occupied_slots = parking.observe(
            [
                detection((1200.0, 300.0), 0.99, 100.0, 70.0, track_id=21),
                detection((1200.0, 600.0), 0.99, 100.0, 70.0, track_id=22),
            ],
            (800, 1600),
        )
        assignment = coordinator.observe([entry_vehicle], both_charge_occupied_slots)
        assert assignment is not None
        assert assignment.target_slot_name is None
        assert assignment.status == "waiting_for_charge_slot"

        # Episode 배정된 ego는 기존 active phase와 관계없이 C2 목표로 planner
        # 입력을 생성한다. 다른 ID가 배정되면 planner 입력을 만들지 않는다.
        episode_tracker = EgoVehicleTracker(
            transform,
            rear_axle_offset_cm=4.0,
            initial_center_bev_px=(120.0, 120.0),
            initial_yaw_rad=math.radians(40.0),
            position_alpha=1.0,
            yaw_alpha=1.0,
        )
        episode_scene = SceneLocalizer(
            episode_tracker,
            parking,
            vehicle_state_manager=VehicleStateManager(transform),
            charge_coordinator=ChargeEpisodeCoordinator(("C2", "C1")),
            parking_assignment=entry_policy,
        ).observe(
            [detection((120.0, 120.0), 0.80, track_id=7)],
            (800, 1600),
            frame_index=13,
            observed_at_unix_sec=103.0,
        )
        assert episode_scene.charge_assignment is not None
        assert episode_scene.charge_assignment.target_slot_name == "C2"
        assert episode_scene.planning_request is not None
        assert episode_scene.planning_request.slot_name == "C2"

        # C2에 실제로 도착한 ego가 space 충전 완료 이벤트를 받으면, 아직
        # 화면상 C2에 있어도 C2 재계획 대신 P1~P4 대기 주차 단계로 전환한다.
        post_charge_policy = ParkingAssignmentPolicy(
            name="charge_to_exit",
            reference_bev_px=(1600.0, 600.0),
            allowed_slots=("P1", "P2"),
            preference="nearest",
            candidate_limit=2,
        )
        post_charge_tracker = EgoVehicleTracker(
            transform,
            rear_axle_offset_cm=4.0,
            initial_center_bev_px=(1200.0, 600.0),
            initial_yaw_rad=math.radians(40.0),
            position_alpha=1.0,
            yaw_alpha=1.0,
        )
        post_charge_localizer = SceneLocalizer(
            post_charge_tracker,
            parking,
            vehicle_state_manager=VehicleStateManager(transform),
            charge_coordinator=ChargeEpisodeCoordinator(("C2", "C1")),
            parking_assignment=entry_policy,
            post_charge_parking_assignment=post_charge_policy,
        )
        c2_ego = detection((1200.0, 600.0), 0.95, 100.0, 70.0, track_id=42)
        charging_scene = post_charge_localizer.observe(
            [c2_ego], (800, 1600), frame_index=14, observed_at_unix_sec=104.0
        )
        assert charging_scene.planning_request is None
        completed, message = post_charge_localizer.complete_charging(
            42, charging_scene.tracked_vehicles
        )
        assert completed and "충전이 완료되었습니다" in message
        post_charge_scene = post_charge_localizer.observe(
            [c2_ego], (800, 1600), frame_index=15, observed_at_unix_sec=104.1
        )
        assert post_charge_scene.planning_request is not None
        assert post_charge_scene.planning_request.slot_name == "P1", (
            post_charge_scene.planning_request.slot_name,
            post_charge_scene.planning_request.candidate_slot_names,
        )
        assert post_charge_scene.planning_request.assignment_policy == "charge_to_exit"

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
    print(f"Ego vehicle center: {scene.vehicle.center_cm}")
    print(f"Ego yaw: {math.degrees(scene.vehicle.yaw_rad):.3f} deg")
    print(f"Selected free slot: {scene.planning_request.slot_name}")
    print(f"Planning start: {scene.planning_request.start_pose_cm}")
    print(f"Planning goal: {scene.planning_request.goal_pose_cm}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
