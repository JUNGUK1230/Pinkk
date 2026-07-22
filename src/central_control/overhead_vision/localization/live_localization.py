"""상단 USB Camera BEV에서 차량 pose와 주차면 좌표를 실시간 생성한다.

실행:
    cd ~/PINKK
    .venv/bin/python -m src.central_control.overhead_vision.localization.live_localization

저장된 BEV 한 장 테스트:
    .venv/bin/python -m src.central_control.overhead_vision.localization.live_localization \
        --bev-image src/central_control/camera_tools/first_map/camera_bev.png
"""

import argparse
import json
import math
from pathlib import Path
import time
from typing import Sequence

import cv2
import numpy as np
import yaml

if __package__:
    from .scene_localizer import (
        AffineBevToLidar,
        Detection,
        EgoVehicleTracker,
        ParkingAssignmentPolicy,
        ParkingSlotMap,
        SceneLocalizer,
        SceneObservation,
        save_scene_observation,
    )
else:
    # `python3 path/to/live_localization.py` 직접 실행에서는 package 문맥이
    # 없으므로 같은 폴더를 import 경로에 넣어 module 실행과 동일하게 동작한다.
    import sys

    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from scene_localizer import (  # type: ignore[no-redef]
        AffineBevToLidar,
        Detection,
        EgoVehicleTracker,
        ParkingAssignmentPolicy,
        ParkingSlotMap,
        SceneLocalizer,
        SceneObservation,
        save_scene_observation,
    )


REPO_ROOT = Path(__file__).resolve().parents[4]
DEFAULT_CONFIG = REPO_ROOT / "src/central_control/config/yolo/realtime_localization.yaml"
WINDOW_NAME = "PINKK Live Vehicle and Parking Localization"


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
    target_slot_value = config.get("target_slot_name")
    target_slot_name = (
        str(target_slot_value) if target_slot_value is not None else None
    )
    parking_assignment = load_parking_assignment(config)
    return (
        SceneLocalizer(
            tracker,
            slots,
            target_slot_name=target_slot_name,
            parking_assignment=parking_assignment,
        ),
        transform,
    )


def load_parking_assignment(
    config: dict[str, object],
) -> ParkingAssignmentPolicy | None:
    """현재 에피소드의 입구·출구 기반 주차칸 배정 규칙을 읽는다."""
    raw_assignment = config.get("parking_assignment")
    if not isinstance(raw_assignment, dict):
        return None
    phase_name = raw_assignment.get("active_phase")
    phases = raw_assignment.get("phases")
    if not isinstance(phase_name, str) or not isinstance(phases, dict):
        raise ValueError("parking_assignment requires active_phase and phases")
    phase = phases.get(phase_name)
    if not isinstance(phase, dict):
        raise ValueError(f"parking assignment phase is missing: {phase_name}")
    reference_name = phase.get("reference_point")
    if not isinstance(reference_name, str):
        raise ValueError("parking assignment reference_point must be a string")
    points_path = resolve_path(str(config["parking_points_path"]))
    with points_path.open(encoding="utf-8") as file:
        points = json.load(file)
    reference = points.get(reference_name)
    if not isinstance(reference, dict):
        raise ValueError(f"parking reference point is missing: {reference_name}")
    center = reference.get("center_bev")
    if not isinstance(center, list) or len(center) != 2:
        raise ValueError(f"parking reference has invalid center_bev: {reference_name}")
    raw_slots = phase.get("allowed_slots")
    if not isinstance(raw_slots, list):
        raise ValueError("parking assignment allowed_slots must be a list")
    return ParkingAssignmentPolicy(
        name=phase_name,
        reference_bev_px=(float(center[0]), float(center[1])),
        allowed_slots=tuple(str(slot) for slot in raw_slots),
        preference=str(phase.get("preference", "nearest")),
        candidate_limit=int(phase.get("candidate_limit", len(raw_slots))),
    )


def draw_scene(
    bev: np.ndarray,
    scene: SceneObservation,
    transform: AffineBevToLidar,
    target_slot_name: str | None = None,
) -> np.ndarray:
    canvas = bev.copy()
    selected_name = (
        scene.planning_request.slot_name if scene.planning_request else None
    )
    for slot in scene.parking_slots:
        polygon = np.rint(slot.polygon_bev).astype(np.int32)
        color = (0, 0, 255) if slot.occupied else (0, 180, 0)
        if slot.name == target_slot_name and not slot.occupied:
            color = (255, 0, 255)
        if slot.name == selected_name and not slot.occupied:
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
        if not vehicle.heading_ambiguous:
            cv2.circle(canvas, rear, 7, (0, 255, 0), 2)
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


class ManualHeadingSelector:
    """h 키 이후 한 번의 BEV 클릭으로 검출 차량의 앞 방향을 지정한다."""

    def __init__(
        self,
        tracker: EgoVehicleTracker,
        transform: AffineBevToLidar,
    ) -> None:
        self.tracker = tracker
        self.transform = transform
        self.scene: SceneObservation | None = None
        self.armed = False

    def update_scene(self, scene: SceneObservation) -> None:
        self.scene = scene

    def arm(self) -> None:
        if self.scene is None or self.scene.vehicle is None:
            print("Manual heading unavailable: ego vehicle is not detected")
            return
        self.armed = True
        print("Manual heading armed: click a point in FRONT of the ego vehicle")

    def clear(self) -> None:
        self.tracker.clear_manual_heading()
        self.armed = False
        print("Manual heading cleared. Press h and click the vehicle front again")

    def mouse_callback(
        self,
        event: int,
        x: int,
        y: int,
        flags: int,
        parameter: object,
    ) -> None:
        del flags, parameter
        if event != cv2.EVENT_LBUTTONDOWN or not self.armed:
            return
        if self.scene is None or self.scene.vehicle is None:
            print("Manual heading failed: ego vehicle is not detected")
            self.armed = False
            return
        center = self.scene.vehicle.center_bev_px
        direction = (float(x) - center[0], float(y) - center[1])
        if math.hypot(*direction) < 10.0:
            print("Manual heading failed: click farther from the vehicle center")
            return
        yaw_rad = self.transform.axis_yaw_in_lidar(center, direction)
        self.tracker.set_manual_heading(yaw_rad)
        self.armed = False
        print(
            "Manual ego heading set: "
            f"click=({x}, {y}), yaw={math.degrees(yaw_rad):.1f} deg"
        )

    def draw_instruction(self, image: np.ndarray) -> None:
        if self.armed:
            message = "CLICK A POINT IN FRONT OF THE EGO VEHICLE"
            color = (0, 255, 255)
        elif self.tracker.manual_yaw_rad is None:
            if self.tracker.absolute_heading_resolved:
                message = "CONFIG HEADING SET | h: replace by click"
                color = (0, 255, 0)
            else:
                message = "PRESS h, THEN CLICK THE EGO FRONT | x: clear heading"
                color = (0, 165, 255)
        else:
            message = "MANUAL HEADING SET | h: set again | x: clear"
            color = (0, 255, 0)
        cv2.putText(
            image,
            message,
            (20, image.shape[0] - 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            color,
            2,
            cv2.LINE_AA,
        )


class PlannedPathOverlay:
    """검증된 자동 Hybrid JSON을 감시해 live BEV에 연속 경로선을 그린다."""

    def __init__(
        self,
        path: Path,
        transform: AffineBevToLidar,
        expected_slot_name: str | None,
    ) -> None:
        self.path = path
        self.transform = transform
        self.expected_slot_name = expected_slot_name
        self.modified_time_ns: int | None = None
        self.path_bev: tuple[tuple[float, float], ...] = ()
        self.start_cm: tuple[float, float] | None = None
        self.source_frame: int | None = None
        self.slot_name: str | None = None
        self.status = "no validated path"

    def refresh(self) -> None:
        if not self.path.exists():
            self._clear("no validated path")
            return
        modified_time_ns = self.path.stat().st_mtime_ns
        if modified_time_ns == self.modified_time_ns:
            return
        self.modified_time_ns = modified_time_ns
        try:
            with self.path.open(encoding="utf-8") as file:
                payload = json.load(file)
            source = payload["source"]
            slot_name = str(source["parking_slot"])
            if (
                self.expected_slot_name is not None
                and slot_name != self.expected_slot_name
            ):
                self._clear(
                    f"path target {slot_name} ignored; waiting for "
                    f"{self.expected_slot_name}"
                )
                return
            points_cm = [
                (float(point["x_cm"]), float(point["y_cm"]))
                for point in payload["path"]
            ]
            if len(points_cm) < 2:
                raise ValueError("planned path needs at least two points")
            self.path_bev = tuple(
                self.transform.planner_cm_to_bev(point)
                for point in points_cm
            )
            self.start_cm = points_cm[0]
            self.source_frame = int(source["frame_index"])
            self.slot_name = slot_name
            self.status = f"path frame={self.source_frame} target={slot_name}"
            print(f"Loaded validated live path: {len(points_cm)} points, {self.status}")
        except (KeyError, TypeError, ValueError, json.JSONDecodeError, OSError) as error:
            self._clear(f"invalid path overlay: {error}")

    def draw(self, image: np.ndarray, scene: SceneObservation) -> None:
        self.refresh()
        if not self.path_bev:
            return
        if scene.vehicle is not None and self.start_cm is not None:
            distance = math.hypot(
                scene.vehicle.rear_axle_cm[0] - self.start_cm[0],
                scene.vehicle.rear_axle_cm[1] - self.start_cm[1],
            )
            if distance > 8.0:
                cv2.putText(
                    image,
                    f"PATH HIDDEN: ego moved {distance:.1f} cm from path start",
                    (20, 65),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.65,
                    (0, 0, 255),
                    2,
                    cv2.LINE_AA,
                )
                return
        _draw_continuous_path(image, self.path_bev)
        cv2.putText(
            image,
            self.status,
            (20, 65),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 0, 255),
            2,
            cv2.LINE_AA,
        )

    def _clear(self, status: str) -> None:
        self.path_bev = ()
        self.start_cm = None
        self.source_frame = None
        self.slot_name = None
        self.status = status


def _draw_continuous_path(
    image: np.ndarray,
    path_bev: Sequence[tuple[float, float]],
) -> int:
    """화면 안의 연속 구간만 굵은 빨간 polyline으로 그린다."""
    height, width = image.shape[:2]
    segments: list[list[tuple[int, int]]] = []
    current: list[tuple[int, int]] = []
    skipped = 0
    for x_float, y_float in path_bev:
        point = (int(round(x_float)), int(round(y_float)))
        if 0 <= point[0] < width and 0 <= point[1] < height:
            current.append(point)
        else:
            skipped += 1
            if current:
                segments.append(current)
                current = []
    if current:
        segments.append(current)
    for segment in segments:
        if len(segment) >= 2:
            points = np.asarray(segment, dtype=np.int32).reshape(-1, 1, 2)
            cv2.polylines(image, [points], False, (0, 0, 255), 5, cv2.LINE_AA)
    return skipped


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
        if request.candidate_slot_names:
            print(
                "  Assignment candidates: "
                + " -> ".join(request.candidate_slot_names)
            )


def save_bev_image_atomic(image: np.ndarray, save_path: Path) -> None:
    """경로 시각화 프로세스가 반쯤 저장된 BEV를 읽지 않도록 원자 교체한다."""
    save_path.parent.mkdir(parents=True, exist_ok=True)
    temporary = save_path.with_name(
        f"{save_path.stem}.tmp{save_path.suffix}"
    )
    if not cv2.imwrite(str(temporary), image):
        raise RuntimeError(f"failed to save latest Camera BEV: {save_path}")
    temporary.replace(save_path)


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
        bev_output_path = resolve_path(str(config["bev_output_path"]))
        planned_path_path = resolve_path(str(config["planned_path_path"]))
        target_slot_value = config.get("target_slot_name")
        target_slot_name = (
            str(target_slot_value) if target_slot_value is not None else None
        )
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
    print(f"Latest Camera BEV: {bev_output_path}")
    if target_slot_name is not None:
        print(f"Fixed target parking slot: {target_slot_name}")
    elif localizer.parking_assignment is not None:
        policy = localizer.parking_assignment
        print(
            "Parking assignment: "
            f"{policy.name}, {policy.preference} from {policy.reference_bev_px}, "
            f"slots={list(policy.allowed_slots)}"
        )
    print("h: arm manual heading click | x: clear heading | q/ESC: quit")
    heading_selector = ManualHeadingSelector(localizer.tracker, transform)
    path_overlay = PlannedPathOverlay(
        planned_path_path,
        transform,
        expected_slot_name=target_slot_name,
    )
    if not args.no_display:
        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(WINDOW_NAME, heading_selector.mouse_callback)
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
            heading_selector.update_scene(scene)
            if frame_index % output_every == 0:
                try:
                    # BEV를 먼저 교체하고 같은 frame의 scene을 나중에 공개한다.
                    save_bev_image_atomic(bev, bev_output_path)
                except RuntimeError as error:
                    print(f"ERROR: {error}")
                    return 1
                save_scene_observation(scene, output_path)
            now = time.monotonic()
            if now - last_print_time >= 1.0 or static_bev is not None:
                print_scene(scene)
                last_print_time = now

            if not args.no_display:
                canvas = draw_scene(
                    bev,
                    scene,
                    transform,
                    target_slot_name=target_slot_name,
                )
                path_overlay.draw(canvas, scene)
                heading_selector.draw_instruction(canvas)
                cv2.imshow(WINDOW_NAME, canvas)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), 27):
                    break
                if key == ord("h"):
                    heading_selector.arm()
                elif key == ord("x"):
                    heading_selector.clear()
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
