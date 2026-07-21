"""YOLO 검출부터 Hybrid A* start/goal 입력까지의 mock 회귀 테스트."""

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
    Detection,
    EgoVehicleTracker,
    ParkingSlotMap,
    SceneLocalizer,
    save_scene_observation,
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
