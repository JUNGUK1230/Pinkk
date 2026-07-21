"""상단 USB Camera BEV에서 차량 pose와 주차면 좌표를 실시간 생성한다.

실행:
    cd ~/PINKK
    .venv/bin/python -m src.central_control.overhead_vision.localization.live_localization

저장된 BEV 한 장 테스트:
    .venv/bin/python -m src.central_control.overhead_vision.localization.live_localization \
        --bev-image src/central_control/camera_tools/first_map/camera_bev.png
"""

import argparse
import math
from pathlib import Path
import time
from typing import Sequence

import cv2
import numpy as np
import yaml

from .scene_localizer import (
    AffineBevToLidar,
    Detection,
    EgoVehicleTracker,
    ParkingSlotMap,
    SceneLocalizer,
    SceneObservation,
    save_scene_observation,
)


REPO_ROOT = Path(__file__).resolve().parents[4]
DEFAULT_CONFIG = REPO_ROOT / "src/central_control/config/yolo/realtime_localization.yaml"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run YOLO BEV ego localization and parking-slot coordinates."
    )
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument(
        "--bev-image",
        type=Path,
        help="Use an existing 1600x800 BEV once instead of opening the camera.",
    )
    parser.add_argument("--no-display", action="store_true")
    parser.add_argument("--max-frames", type=int)
    parser.add_argument(
        "--initial-ego-center",
        type=float,
        nargs=2,
        metavar=("X_PX", "Y_PX"),
        help="Override the configured first-frame ego BEV center.",
    )
    parser.add_argument(
        "--initial-yaw-deg",
        type=float,
        help="Override the known initial ego heading in LiDAR-grid degrees.",
    )
    return parser.parse_args()


def resolve_path(value: str | Path) -> Path:
    path = Path(value).expanduser()
    return path if path.is_absolute() else REPO_ROOT / path


def load_config(path: Path) -> dict[str, object]:
    with path.open(encoding="utf-8") as file:
        data = yaml.safe_load(file) or {}
    config = data.get("realtime_localization")
    if not isinstance(config, dict):
        raise ValueError("config must contain realtime_localization mapping")
    return config


def load_camera_geometry(config: dict[str, object]) -> tuple[np.ndarray, np.ndarray, np.ndarray, int, int]:
    calibration_path = resolve_path(str(config["calibration_path"]))
    homography_path = resolve_path(str(config["homography_path"]))
    with np.load(calibration_path) as data:
        camera_matrix = np.asarray(data["camera_matrix"], dtype=np.float64)
        dist_coeffs = np.asarray(data["dist_coeffs"], dtype=np.float64)
    with np.load(homography_path) as data:
        homography = np.asarray(data["homography_matrix"], dtype=np.float64)
        bev_width = int(data["bev_width"])
        bev_height = int(data["bev_height"])
    if camera_matrix.shape != (3, 3) or homography.shape != (3, 3):
        raise ValueError("camera matrix and homography must both be 3x3")
    return camera_matrix, dist_coeffs, homography, bev_width, bev_height


def open_camera(config: dict[str, object]) -> cv2.VideoCapture:
    camera_id = int(config["camera_id"])
    camera = cv2.VideoCapture(camera_id, cv2.CAP_V4L2)
    camera.set(
        cv2.CAP_PROP_FOURCC,
        cv2.VideoWriter_fourcc(*str(config["camera_fourcc"])),
    )
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, int(config["camera_width"]))
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, int(config["camera_height"]))
    camera.set(cv2.CAP_PROP_FPS, int(config["camera_fps"]))
    if not camera.isOpened():
        camera.release()
        raise RuntimeError(f"could not open overhead camera_id={camera_id}")
    return camera


def detections_from_result(result: object) -> list[Detection]:
    detections: list[Detection] = []
    if result.boxes is None:
        return detections
    mask_polygons: Sequence[np.ndarray] = (
        result.masks.xy if result.masks is not None else ()
    )
    for index, box in enumerate(result.boxes):
        class_id = int(box.cls[0].item())
        class_name = str(result.names[class_id])
        confidence = float(box.conf[0].item())
        xyxy = tuple(float(value) for value in box.xyxy[0].cpu().numpy())
        if index < len(mask_polygons) and len(mask_polygons[index]) >= 3:
            polygon = np.asarray(mask_polygons[index], dtype=np.float64)
        else:
            x1, y1, x2, y2 = xyxy
            polygon = np.asarray(
                [[x1, y1], [x2, y1], [x2, y2], [x1, y2]],
                dtype=np.float64,
            )
        detections.append(
            Detection(class_name, confidence, polygon, xyxy)
        )
    return detections


def build_localizer(
    config: dict[str, object],
) -> tuple[SceneLocalizer, AffineBevToLidar]:
    transform = AffineBevToLidar(resolve_path(str(config["registration_path"])))
    initial_center_value = config.get("initial_ego_center_bev_px")
    initial_center = (
        (float(initial_center_value[0]), float(initial_center_value[1]))
        if isinstance(initial_center_value, list) and len(initial_center_value) == 2
        else None
    )
    initial_yaw_value = config.get("initial_ego_yaw_deg")
    initial_yaw = (
        math.radians(float(initial_yaw_value))
        if initial_yaw_value is not None
        else None
    )
    vehicle_config_path = resolve_path(str(config["vehicle_config_path"]))
    with vehicle_config_path.open(encoding="utf-8") as file:
        vehicle_config = (yaml.safe_load(file) or {}).get("vehicle", {})
    # Hybrid A* pose 기준은 rear axle이다. YOLO mask의 기하 중심을 차체
    # 중심으로 보고, 동일한 vehicle config에서 rear axle까지의 거리를 구한다.
    rear_axle_offset_cm = (
        float(vehicle_config["length_cm"]) / 2.0
        - float(vehicle_config["rear_overhang_cm"])
    )
    if rear_axle_offset_cm < 0.0:
        raise ValueError("rear axle offset derived from vehicle config is negative")
    tracker = EgoVehicleTracker(
        transform,
        rear_axle_offset_cm=rear_axle_offset_cm,
        initial_center_bev_px=initial_center,
        initial_yaw_rad=initial_yaw,
        position_alpha=float(config["position_filter_alpha"]),
        yaw_alpha=float(config["yaw_filter_alpha"]),
        max_center_jump_px=float(config["max_center_jump_px"]),
        minimum_elongation=float(config["minimum_mask_elongation"]),
    )
    slots = ParkingSlotMap(
        resolve_path(str(config["parking_slots_path"])),
        transform,
        rear_axle_offset_cm,
        float(config["occupancy_threshold"]),
    )
    return SceneLocalizer(tracker, slots), transform


def draw_scene(
    bev: np.ndarray,
    scene: SceneObservation,
    transform: AffineBevToLidar,
) -> np.ndarray:
    canvas = bev.copy()
    selected_name = (
        scene.planning_request.slot_name if scene.planning_request else None
    )
    for slot in scene.parking_slots:
        polygon = np.rint(slot.polygon_bev).astype(np.int32)
        color = (0, 0, 255) if slot.occupied else (0, 180, 0)
        if slot.name == selected_name:
            color = (0, 255, 255)
        cv2.polylines(canvas, [polygon], True, color, 2, cv2.LINE_AA)
        center = tuple(round(value) for value in slot.center_bev_px)
        cv2.putText(
            canvas,
            f"{slot.name} {'OCC' if slot.occupied else 'FREE'} {slot.occupancy_ratio:.0%}",
            (center[0] - 55, center[1]),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.42,
            color,
            1,
            cv2.LINE_AA,
        )
    if scene.vehicle is not None:
        vehicle = scene.vehicle
        polygon = np.rint(vehicle.polygon_bev).astype(np.int32)
        color = (0, 165, 255) if not vehicle.planning_ready else (255, 255, 0)
        cv2.polylines(canvas, [polygon], True, color, 3, cv2.LINE_AA)
        center = tuple(round(value) for value in vehicle.center_bev_px)
        rear = tuple(
            round(value)
            for value in transform.planner_cm_to_bev(vehicle.rear_axle_cm)
        )
        cv2.circle(canvas, center, 6, (0, 0, 255), -1)
        cv2.circle(canvas, rear, 7, (0, 255, 0), 2)
        if vehicle.planning_ready:
            front = (
                round(rear[0] + 2.0 * (center[0] - rear[0])),
                round(rear[1] + 2.0 * (center[1] - rear[1])),
            )
            cv2.arrowedLine(canvas, rear, front, (255, 255, 0), 3, tipLength=0.25)
        cv2.putText(
            canvas,
            f"ego ({vehicle.rear_axle_cm[0]:.1f}, {vehicle.rear_axle_cm[1]:.1f}) cm yaw={vehicle.yaw_deg:.1f}",
            (max(0, center[0] - 120), max(25, center[1] - 18)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            color,
            2,
            cv2.LINE_AA,
        )
    cv2.putText(
        canvas,
        f"planning_ready={scene.planning_ready} | {scene.status}",
        (20, 35),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (0, 255, 255),
        2,
        cv2.LINE_AA,
    )
    return canvas


def print_scene(scene: SceneObservation) -> None:
    print(f"Frame {scene.frame_index}: {scene.status}")
    if scene.vehicle:
        vehicle = scene.vehicle
        print(
            "  Ego rear axle: "
            f"({vehicle.rear_axle_cm[0]:.2f}, {vehicle.rear_axle_cm[1]:.2f}) cm, "
            f"yaw={vehicle.yaw_deg:.1f} deg, conf={vehicle.confidence:.2f}, "
            f"ambiguous={vehicle.heading_ambiguous or vehicle.ego_selection_ambiguous}"
        )
    free_count = sum(not slot.occupied for slot in scene.parking_slots)
    print(f"  Parking slots: {free_count}/{len(scene.parking_slots)} free")
    # 첫 프레임에는 고정 주차면의 LiDAR 좌표를 모두 출력해 정합 결과를
    # 현장에서 바로 확인한다. 이후 프레임에는 같은 좌표를 반복 출력하지 않는다.
    if scene.frame_index == 0:
        for slot in scene.parking_slots:
            print(
                f"    {slot.name}: center_lidar="
                f"({slot.center_lidar_px[0]:.2f}, {slot.center_lidar_px[1]:.2f}), "
                f"occupied={slot.occupied}, overlap={slot.occupancy_ratio:.1%}"
            )
    if scene.planning_request:
        request = scene.planning_request
        print(
            f"  Selected {request.slot_name}: start={request.start_pose_cm}, "
            f"goal={request.goal_pose_cm}"
        )


def main() -> int:
    args = parse_args()
    try:
        from ultralytics import YOLO

        config = load_config(args.config)
        if args.initial_ego_center is not None:
            config["initial_ego_center_bev_px"] = list(args.initial_ego_center)
        if args.initial_yaw_deg is not None:
            config["initial_ego_yaw_deg"] = args.initial_yaw_deg
        camera_matrix, dist_coeffs, homography, bev_width, bev_height = (
            load_camera_geometry(config)
        )
        localizer, transform = build_localizer(config)
        model_path = resolve_path(str(config["model_path"]))
        model = YOLO(str(model_path))
        if model.task != "segment":
            raise ValueError(
                f"vehicle heading and occupancy require segment weights, got {model.task}"
            )
        output_path = resolve_path(str(config["scene_output_path"]))
    except (ImportError, FileNotFoundError, KeyError, TypeError, ValueError, OSError) as error:
        print(f"ERROR: initialization failed: {error}")
        print("Run with the project virtual environment: .venv/bin/python")
        return 1

    camera: cv2.VideoCapture | None = None
    static_bev: np.ndarray | None = None
    if args.bev_image is not None:
        static_bev = cv2.imread(str(args.bev_image), cv2.IMREAD_COLOR)
        if static_bev is None:
            print(f"ERROR: could not read BEV image: {args.bev_image}")
            return 1
    else:
        try:
            camera = open_camera(config)
        except RuntimeError as error:
            print(f"ERROR: {error}")
            return 1

    print(f"Loaded YOLO segmentation: {model_path} names={model.names}")
    print(f"Scene output: {output_path}")
    print("q/ESC: quit")
    frame_index = 0
    last_print_time = 0.0
    output_every = max(1, int(config["output_every_n_frames"]))
    try:
        while True:
            if static_bev is not None:
                bev = static_bev.copy()
            else:
                assert camera is not None
                ok, frame = camera.read()
                if not ok or frame is None:
                    print("ERROR: failed to read overhead camera frame")
                    return 1
                undistorted = cv2.undistort(frame, camera_matrix, dist_coeffs)
                bev = cv2.warpPerspective(
                    undistorted,
                    homography,
                    (bev_width, bev_height),
                    flags=cv2.INTER_LINEAR,
                )
            if bev.shape[:2] != (bev_height, bev_width):
                print(
                    f"ERROR: BEV size must be {bev_width}x{bev_height}, "
                    f"got {bev.shape[1]}x{bev.shape[0]}"
                )
                return 1

            result = model.predict(
                source=bev,
                conf=float(config["confidence"]),
                iou=float(config["iou"]),
                verbose=False,
            )[0]
            detections = detections_from_result(result)
            scene = localizer.observe(
                detections,
                bev.shape[:2],
                frame_index,
            )
            if frame_index % output_every == 0:
                save_scene_observation(scene, output_path)
            now = time.monotonic()
            if now - last_print_time >= 1.0 or static_bev is not None:
                print_scene(scene)
                last_print_time = now

            if not args.no_display:
                cv2.imshow(
                    "PINKK Live Vehicle and Parking Localization",
                    draw_scene(bev, scene, transform),
                )
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), 27):
                    break
            frame_index += 1
            if static_bev is not None:
                break
            if args.max_frames is not None and frame_index >= args.max_frames:
                break
    finally:
        if camera is not None:
            camera.release()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
