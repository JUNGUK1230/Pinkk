"""상단 USB Camera BEV에서 차량 pose와 주차면 좌표를 실시간 생성한다.

실행:
    cd ~/PINKK
    .venv/bin/python -m src.central_control.overhead_vision.localization.live_localization

저장된 BEV 한 장 테스트:
    .venv/bin/python -m src.central_control.overhead_vision.localization.live_localization \
        --bev-image src/central_control/camera_tools/first_map/camera_bev.png
"""

import argparse
from concurrent.futures import Future, ThreadPoolExecutor
from dataclasses import dataclass
import json
import math
from pathlib import Path
import sys
import threading
import time
from typing import Sequence

import cv2
import numpy as np
import yaml

if __package__:
    from .scene_localizer import (
        AffineBevToLidar,
        ChargeEpisodeCoordinator,
        Detection,
        EgoVehicleTracker,
        ParkingAssignmentPolicy,
        ParkingSlotMap,
        SceneLocalizer,
        SceneObservation,
        VehicleStateManager,
        save_scene_observation,
    )
    from ..path_planning.direct_ros_publisher import DirectRosPublisher
else:
    # `python3 path/to/live_localization.py` 직접 실행에서는 package 문맥이
    # 없으므로 같은 폴더를 import 경로에 넣어 module 실행과 동일하게 동작한다.
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    sys.path.insert(
        0,
        str(Path(__file__).resolve().parents[1] / "path_planning"),
    )
    from scene_localizer import (  # type: ignore[no-redef]
        AffineBevToLidar,
        ChargeEpisodeCoordinator,
        Detection,
        EgoVehicleTracker,
        ParkingAssignmentPolicy,
        ParkingSlotMap,
        SceneLocalizer,
        SceneObservation,
        VehicleStateManager,
        save_scene_observation,
    )
    from direct_ros_publisher import DirectRosPublisher  # type: ignore[no-redef]


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
    parser.add_argument(
        "--no-ros",
        action="store_true",
        help="Disable direct ROS 2 topic publishing for image-only diagnostics.",
    )
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


class LatestFrameCamera:
    """카메라를 계속 비워 메인 처리보다 오래된 프레임이 쌓이지 않게 한다.

    YOLO 처리 속도가 카메라 FPS보다 낮아도 대기 프레임을 순서대로 소비하지
    않고, 호출 시점에 가장 최근에 취득된 한 장만 반환한다.
    """

    def __init__(self, camera: cv2.VideoCapture) -> None:
        self._camera = camera
        self._condition = threading.Condition()
        self._stop_event = threading.Event()
        self._thread = threading.Thread(
            target=self._capture_loop,
            name="overhead-camera-capture",
            daemon=True,
        )
        self._sequence = -1
        self._captured_at = 0.0
        self._frame: np.ndarray | None = None
        self._error: str | None = None

    def start(self, timeout_seconds: float = 3.0) -> "LatestFrameCamera":
        self._thread.start()
        with self._condition:
            ready = self._condition.wait_for(
                lambda: self._frame is not None or self._error is not None,
                timeout=timeout_seconds,
            )
            if not ready:
                self.close()
                raise RuntimeError("timed out waiting for the first camera frame")
            if self._error is not None:
                error = self._error
                self.close()
                raise RuntimeError(error)
        return self

    def read_latest(
        self,
        after_sequence: int,
        timeout_seconds: float = 2.0,
    ) -> tuple[int, float, np.ndarray]:
        """`after_sequence`보다 새로운 최신 프레임 하나를 반환한다."""
        with self._condition:
            ready = self._condition.wait_for(
                lambda: self._sequence > after_sequence or self._error is not None,
                timeout=timeout_seconds,
            )
            if not ready:
                raise RuntimeError("timed out waiting for a new camera frame")
            if self._error is not None:
                raise RuntimeError(self._error)
            assert self._frame is not None
            return self._sequence, self._captured_at, self._frame

    def close(self) -> None:
        self._stop_event.set()
        if self._thread.is_alive():
            self._thread.join(timeout=1.0)
        self._camera.release()
        if self._thread.is_alive():
            self._thread.join(timeout=1.0)

    def _capture_loop(self) -> None:
        while not self._stop_event.is_set():
            ok, frame = self._camera.read()
            captured_at = time.monotonic()
            if not ok or frame is None:
                with self._condition:
                    self._error = "failed to read overhead camera frame"
                    self._condition.notify_all()
                return
            with self._condition:
                self._sequence += 1
                self._captured_at = captured_at
                self._frame = frame
                self._condition.notify_all()


@dataclass(frozen=True)
class FixedRouteOutcome:
    """Fixed-route selection shaped for the existing overlay/publish bridge."""

    request: object
    adjusted_start: tuple[float, float, float]
    adjusted_goal: tuple[float, float, float]
    trajectory: tuple[object, ...]


class IntegratedPlanningController:
    """Select one fixed route for each new vehicle/target episode."""

    def __init__(self, route_selector: object) -> None:
        self.route_selector = route_selector
        self._executor = ThreadPoolExecutor(
            max_workers=1,
            thread_name_prefix="pinkk-fixed-route",
        )
        self._future: Future[object] | None = None
        self._active_key: tuple[int | None, str, int] | None = None
        self._completed_key: tuple[int | None, str, int] | None = None
        self.outcome: object | None = None
        self.last_error: str | None = None
        self._new_outcome = False
        self._discard_pending_result = False

    @property
    def status(self) -> str:
        if self._future is not None:
            return "Fixed route selecting..."
        if self.last_error is not None:
            return f"Fixed route blocked: {self.last_error}"
        if self.outcome is not None:
            return "Fixed route ready and published"
        return "waiting for vehicle section and target"

    def update(
        self,
        scene: SceneObservation,
        heading_revision: int,
    ) -> None:
        """새 차량/목표 조합일 때만 고정 경로 선택을 시작한다."""
        self._collect_finished()
        if scene.vehicle is None or scene.planning_request is None:
            return
        key = (
            scene.vehicle.track_id,
            scene.planning_request.slot_name,
            heading_revision,
        )
        if self._future is not None:
            return
        if key == self._completed_key:
            return
        if key != self._active_key:
            # 새 heading 또는 새 충전 배정은 이전 overlay를 숨기고 새 경로가
            # 검증될 때까지 기다린다.
            self.outcome = None
            self.last_error = None
            self._active_key = key
        request = scene.planning_request
        self._future = self._executor.submit(
            self._plan_request,
            scene.frame_index,
            scene.observed_at_unix_sec,
            request.slot_name,
            request.start_pose_cm,
            request.goal_pose_cm,
            request.alternative_goal_pose_cm,
        )

    def consume_new_outcome(self) -> object | None:
        self._collect_finished()
        if not self._new_outcome:
            return None
        self._new_outcome = False
        return self.outcome

    def invalidate(self) -> None:
        """단계 전환 시 이전 목적지의 실행 중/완료 경로를 폐기한다."""
        self.outcome = None
        self.last_error = None
        self._active_key = None
        self._completed_key = None
        self._new_outcome = False
        # ThreadPoolExecutor에서 실행 중인 Hybrid A*는 안전하게 강제 종료할 수
        # 없으므로 완료 결과만 버리고, 다음 프레임에서 새 목적지를 계획한다.
        self._discard_pending_result = self._future is not None

    def draw_overlay(
        self,
        image: np.ndarray,
        transform: AffineBevToLidar,
    ) -> np.ndarray:
        """파일을 읽지 않고 검증 완료 trajectory를 현재 BEV canvas에 그린다."""
        if self.outcome is None:
            return image
        outcome = self.outcome
        try:
            from plan_from_live_vision import draw_live_path_overlay

            trajectory = getattr(outcome, "trajectory")
            lidar_points = np.asarray(
                [
                    [
                        point.x_cm / transform.resolution_cm,
                        point.y_cm / transform.resolution_cm,
                        1.0,
                    ]
                    for point in trajectory
                ],
                dtype=np.float64,
            )
            camera_points = lidar_points @ transform.inverse_matrix.T
            canvas, _ = draw_live_path_overlay(
                image,
                camera_points,
                getattr(outcome, "adjusted_start"),
                getattr(outcome, "adjusted_goal"),
                transform.inverse_matrix,
                transform.resolution_cm,
                getattr(getattr(outcome, "request"), "frame_index"),
                getattr(getattr(outcome, "request"), "slot_name"),
            )
            return canvas
        except (AttributeError, ImportError, TypeError, ValueError) as error:
            self.last_error = f"overlay failed: {error}"
            return image

    def close(self) -> None:
        self._executor.shutdown(wait=False, cancel_futures=True)

    def _collect_finished(self) -> None:
        if self._future is None or not self._future.done():
            return
        future = self._future
        self._future = None
        if self._discard_pending_result:
            self._discard_pending_result = False
            return
        self._completed_key = self._active_key
        try:
            self.outcome = future.result()
            self.last_error = None
            self._new_outcome = True
        except (FileNotFoundError, KeyError, OSError, RuntimeError, TypeError, ValueError) as error:
            self.outcome = None
            self.last_error = str(error)

    def _plan_request(
        self,
        frame_index: int,
        observed_at_unix_sec: float,
        slot_name: str,
        start_pose_cm: tuple[float, float, float],
        goal_pose_cm: tuple[float, float, float],
        alternative_goal_pose_cm: tuple[float, float, float],
    ) -> object:
        """Select the configured full route from the localized source endpoint."""
        path_planning_root = REPO_ROOT / "src/central_control/path_planning"
        for module_path in (
            path_planning_root / "scripts",
            path_planning_root / "src",
        ):
            module_path_text = str(module_path)
            if module_path_text not in sys.path:
                sys.path.insert(0, module_path_text)
        from vision_scene_input import VisionPlanningRequest

        request = VisionPlanningRequest(
            frame_index=frame_index,
            observed_at_unix_sec=observed_at_unix_sec,
            slot_name=slot_name,
            start_pose_cm=start_pose_cm,
            goal_pose_cm=goal_pose_cm,
            alternative_goal_pose_cm=alternative_goal_pose_cm,
        )
        selector = getattr(self.route_selector, "select")
        selection = selector(start_pose_cm, slot_name)
        trajectory = tuple(selection.points)
        final = trajectory[-1]
        return FixedRouteOutcome(
            request=request,
            adjusted_start=start_pose_cm,
            adjusted_goal=(final.x_cm, final.y_cm, final.yaw_rad),
            trajectory=trajectory,
        )


def open_camera(config: dict[str, object]) -> LatestFrameCamera:
    camera_id = int(config["camera_id"])
    camera = cv2.VideoCapture(camera_id, cv2.CAP_V4L2)
    camera.set(
        cv2.CAP_PROP_FOURCC,
        cv2.VideoWriter_fourcc(*str(config["camera_fourcc"])),
    )
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, int(config["camera_width"]))
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, int(config["camera_height"]))
    camera.set(cv2.CAP_PROP_FPS, int(config["camera_fps"]))
    # 일부 V4L2/OpenCV 조합은 이 속성을 무시할 수 있다. 따라서 버퍼 1 설정과
    # 무관하게 LatestFrameCamera 스레드가 장치를 계속 읽어 최신성을 보장한다.
    buffer_requested = int(config.get("camera_buffer_size", 1))
    buffer_setting_accepted = camera.set(
        cv2.CAP_PROP_BUFFERSIZE,
        buffer_requested,
    )
    if not camera.isOpened():
        camera.release()
        raise RuntimeError(f"could not open overhead camera_id={camera_id}")
    actual_width = int(camera.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_height = int(camera.get(cv2.CAP_PROP_FRAME_HEIGHT))
    actual_fps = float(camera.get(cv2.CAP_PROP_FPS))
    print(
        "Opened overhead camera: "
        f"{actual_width}x{actual_height} @ {actual_fps:.1f} FPS, "
        f"latest-frame buffer request={buffer_requested} "
        f"accepted={buffer_setting_accepted}"
    )
    return LatestFrameCamera(camera).start()


def detections_from_result(result: object) -> list[Detection]:
    detections: list[Detection] = []
    if result.boxes is None:
        return detections
    mask_polygons: Sequence[np.ndarray] = (
        result.masks.xy if result.masks is not None else ()
    )
    tracked_ids = result.boxes.id if result.boxes.id is not None else None
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
        track_id = (
            int(round(float(tracked_ids[index].item())))
            if tracked_ids is not None
            else None
        )
        detections.append(
            Detection(class_name, confidence, polygon, xyxy, track_id)
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
    fixed_route_config_path = resolve_path(str(config["fixed_route_config_path"]))
    with fixed_route_config_path.open(encoding="utf-8") as file:
        fixed_route_config = yaml.safe_load(file)
    fixed_endpoints = fixed_route_config["endpoints"]
    fixed_yaws = {
        name: float(endpoint.get("goal", endpoint["staging"])[2])
        for name, endpoint in fixed_endpoints.items()
    }
    initial_yaw_value = config.get("initial_ego_yaw_deg")
    initial_yaw = (
        math.radians(float(initial_yaw_value))
        if initial_yaw_value is not None
        else fixed_yaws["START"]
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
    slots = ParkingSlotMap(
        resolve_path(str(config["parking_slots_path"])),
        transform,
        rear_axle_offset_cm,
        float(config["occupancy_threshold"]),
    )

    def fixed_heading_for_center(center_bev: tuple[float, float]) -> float | None:
        slot_name = slots.slot_name_for_point(center_bev)
        # Planning starts only at START or inside a configured slot. Outside a
        # slot, keep the one-way START/corridor heading instead of asking for a
        # mask-front click.
        return fixed_yaws.get(slot_name, fixed_yaws["START"])

    tracker = EgoVehicleTracker(
        transform,
        rear_axle_offset_cm=rear_axle_offset_cm,
        initial_center_bev_px=initial_center,
        initial_yaw_rad=initial_yaw,
        position_alpha=float(config["position_filter_alpha"]),
        yaw_alpha=float(config["yaw_filter_alpha"]),
        max_center_jump_px=float(config["max_center_jump_px"]),
        minimum_elongation=float(config["minimum_mask_elongation"]),
        fixed_heading_resolver=fixed_heading_for_center,
    )
    target_slot_value = config.get("target_slot_name")
    target_slot_name = (
        str(target_slot_value) if target_slot_value is not None else None
    )
    parking_assignment = load_parking_assignment(config)
    post_charge_parking_assignment = load_parking_assignment(
        config,
        phase_name="charge_to_exit",
    )
    vehicle_states = VehicleStateManager(
        transform,
        track_ttl_sec=float(config.get("vehicle_track_ttl_sec", 2.0)),
    )
    charge_priority_value = config.get("charge_slot_priority", ["C2", "C1"])
    if not isinstance(charge_priority_value, list):
        raise ValueError("charge_slot_priority must be a list")
    charge_coordinator = ChargeEpisodeCoordinator(
        tuple(str(slot) for slot in charge_priority_value)
    )
    return (
        SceneLocalizer(
            tracker,
            slots,
            vehicle_state_manager=vehicle_states,
            charge_coordinator=charge_coordinator,
            target_slot_name=target_slot_name,
            parking_assignment=parking_assignment,
            post_charge_parking_assignment=post_charge_parking_assignment,
        ),
        transform,
    )


def load_parking_assignment(
    config: dict[str, object],
    phase_name: str | None = None,
) -> ParkingAssignmentPolicy | None:
    """현재 에피소드의 거리 또는 명시 순서 기반 주차칸 배정 규칙을 읽는다."""
    raw_assignment = config.get("parking_assignment")
    if not isinstance(raw_assignment, dict):
        return None
    phase_name = phase_name or raw_assignment.get("active_phase")
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
    detections: Sequence[Detection] = (),
    route_selector: object | None = None,
) -> np.ndarray:
    canvas = bev.copy()
    tracked_by_id = {
        vehicle.track_id: vehicle for vehicle in scene.tracked_vehicles
    }
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
    # 모든 검출 차량에 ByteTrack ID를 표시한다. ego 차량은 아래에서 더 굵은
    # 외곽선과 rear-axle pose로 다시 강조한다.
    for detection in detections:
        if detection.class_name != "car":
            continue
        polygon = np.rint(detection.polygon_bev).astype(np.int32)
        cv2.polylines(canvas, [polygon], True, (255, 128, 0), 1, cv2.LINE_AA)
        x1, y1, _, _ = detection.bbox_xyxy
        label = (
            (
                f"car id={detection.track_id} "
                f"{tracked_by_id[detection.track_id].state} "
                f"{tracked_by_id[detection.track_id].assigned_slot_name or ''}"
            )
            if detection.track_id is not None
            and detection.track_id in tracked_by_id
            else f"car id=pending {detection.confidence:.2f}"
        )
        cv2.putText(
            canvas,
            label,
            (max(0, round(x1)), max(18, round(y1) - 6)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            (255, 128, 0),
            2,
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
            (
                f"ego id={vehicle.track_id} "
                f"({vehicle.rear_axle_cm[0]:.1f}, {vehicle.rear_axle_cm[1]:.1f}) cm "
                f"yaw={vehicle.yaw_deg:.1f}"
            ),
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
    current_section, route_target = route_context(scene, route_selector)
    cv2.putText(
        canvas,
        f"CURRENT SECTION: {current_section}  ->  TARGET: {route_target}",
        (20, 65),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.72,
        (0, 255, 255),
        2,
        cv2.LINE_AA,
    )
    if scene.charge_assignment is not None:
        assignment = scene.charge_assignment
        cv2.putText(
            canvas,
            (
                f"charge: id={assignment.vehicle_track_id} -> "
                f"{assignment.target_slot_name or 'WAIT'} "
                f"({assignment.status})"
            ),
            (20, 125),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.62,
            (255, 0, 255),
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
        self.revision = 0

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
        self.revision += 1
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
        self.revision += 1
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
            message = (
                "MANUAL HEADING SET | SPACE: charge complete | "
                "h: set again | x: clear"
            )
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


def route_context(
    scene: SceneObservation,
    route_selector: object | None,
) -> tuple[str, str]:
    """Return the live ego section and currently assigned route target."""
    current_section = "UNKNOWN"
    if scene.vehicle is not None:
        tracked = next(
            (
                item
                for item in scene.tracked_vehicles
                if item.track_id == scene.vehicle.track_id
            ),
            None,
        )
        if tracked is not None and tracked.assigned_slot_name is not None:
            current_section = tracked.assigned_slot_name
        elif route_selector is not None:
            detector = getattr(route_selector, "detect_location", None)
            if callable(detector):
                current_section = str(
                    detector(
                        (
                            scene.vehicle.rear_axle_cm[0],
                            scene.vehicle.rear_axle_cm[1],
                            scene.vehicle.yaw_rad,
                        )
                    )
                )
        else:
            current_section = "TRANSIT"

    if scene.planning_request is not None:
        target = scene.planning_request.slot_name
    elif (
        scene.charge_assignment is not None
        and scene.charge_assignment.target_slot_name is not None
    ):
        target = scene.charge_assignment.target_slot_name
    else:
        target = "WAIT"
    return current_section, target


def print_scene(
    scene: SceneObservation,
    route_selector: object | None = None,
) -> None:
    print(f"Frame {scene.frame_index}: {scene.status}")
    if scene.vehicle:
        vehicle = scene.vehicle
        print(
            "  Ego rear axle: "
            f"({vehicle.rear_axle_cm[0]:.2f}, {vehicle.rear_axle_cm[1]:.2f}) cm, "
            f"id={vehicle.track_id}, yaw={vehicle.yaw_deg:.1f} deg, "
            f"conf={vehicle.confidence:.2f}, "
            f"ambiguous={vehicle.heading_ambiguous or vehicle.ego_selection_ambiguous}"
        )
    current_section, route_target = route_context(scene, route_selector)
    print(f"  Route context: {current_section} -> {route_target}")
    free_count = sum(not slot.occupied for slot in scene.parking_slots)
    print(f"  Parking slots: {free_count}/{len(scene.parking_slots)} free")
    if scene.tracked_vehicles:
        print(
            "  Tracked vehicles: "
            + ", ".join(
                (
                    f"id={item.track_id}:{item.state}"
                    f"({item.assigned_slot_name or '-'},{'live' if item.visible else 'lost'})"
                )
                for item in scene.tracked_vehicles
            )
        )
    if scene.charge_assignment is not None:
        assignment = scene.charge_assignment
        print(
            "  Charge assignment: "
            f"id={assignment.vehicle_track_id} -> "
            f"{assignment.target_slot_name or 'waiting'} "
            f"({assignment.status})"
        )
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
    # 별도 localization 테스트와 live process가 동시에 실행되어도 서로의
    # 임시 파일을 지우지 않도록 실행 시각 기반의 고유 이름을 사용한다.
    temporary = save_path.with_name(
        f".{save_path.stem}.{time.time_ns()}.tmp{save_path.suffix}"
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
        write_runtime_files = bool(config.get("write_runtime_files", False))
        output_path = resolve_path(str(config["scene_output_path"]))
        bev_output_path = resolve_path(str(config["bev_output_path"]))
        ros_publish_enabled = (
            not args.no_ros and bool(config.get("ros_publish_enabled", True))
        )
        ros_publisher = DirectRosPublisher() if ros_publish_enabled else None
        target_slot_value = config.get("target_slot_name")
        target_slot_name = (
            str(target_slot_value) if target_slot_value is not None else None
        )
        path_planning_root = REPO_ROOT / "src/central_control/path_planning"
        route_module_path = str(path_planning_root / "src")
        if route_module_path not in sys.path:
            sys.path.insert(0, route_module_path)
        from fixed_route_selector import FixedRouteSelector

        route_selector = FixedRouteSelector(
            resolve_path(str(config["fixed_route_config_path"])),
            resolve_path(str(config["fixed_route_directory"])),
        )
    except (ImportError, FileNotFoundError, KeyError, TypeError, ValueError, OSError) as error:
        print(f"ERROR: initialization failed: {error}")
        print("Run with the project virtual environment: .venv/bin/python")
        return 1

    camera: LatestFrameCamera | None = None
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
    if write_runtime_files:
        print(f"Debug scene output: {output_path}")
        print(f"Debug Camera BEV: {bev_output_path}")
    else:
        print("Runtime path/scene files: disabled (direct ROS topics only)")
    if ros_publisher is not None:
        print(
            "Direct ROS topics: "
            f"{ros_publisher.pose_topic}, {ros_publisher.path_topic}, "
            f"{ros_publisher.trajectory_topic}"
        )
    if target_slot_name is not None:
        print(f"Fixed target parking slot: {target_slot_name}")
    elif localizer.parking_assignment is not None:
        policy = localizer.parking_assignment
        print(
            "Parking assignment: "
            f"{policy.name}, {policy.preference} from {policy.reference_bev_px}, "
            f"slots={list(policy.allowed_slots)}"
        )
    print("SPACE: charge complete | p: replan | q/ESC: quit")
    planning_controller = IntegratedPlanningController(route_selector)
    replan_revision = 0
    if not args.no_display:
        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    # 첫 실제 프레임 추론 전에 CUDA 커널과 모델을 준비한다. 준비 중에도 캡처
    # 스레드는 카메라를 계속 비우므로 완료 후 가장 최근 프레임부터 처리한다.
    inference_imgsz = int(config.get("inference_imgsz", 1600))
    inference_device = config.get("inference_device", 0)
    tracker_config = str(config.get("tracker_config", "bytetrack.yaml"))
    if static_bev is None:
        warmup_image = np.zeros((bev_height, bev_width, 3), dtype=np.uint8)
        model.predict(
            source=warmup_image,
            imgsz=inference_imgsz,
            device=inference_device,
            conf=float(config["confidence"]),
            iou=float(config["iou"]),
            verbose=False,
        )
        print(
            "YOLO warm-up complete: "
            f"imgsz={inference_imgsz}, device={inference_device}"
        )

    frame_index = 0
    processed_frames = 0
    last_camera_sequence = -1
    dropped_camera_frames = 0
    last_print_time = 0.0
    output_every = max(1, int(config["output_every_n_frames"]))
    latency_warning_ms = float(config.get("latency_warning_ms", 150.0))
    undistort_maps: tuple[np.ndarray, np.ndarray] | None = None
    try:
        while True:
            if static_bev is not None:
                bev = static_bev.copy()
                captured_at = time.monotonic()
            else:
                assert camera is not None
                previous_sequence = last_camera_sequence
                frame_index, captured_at, frame = camera.read_latest(
                    last_camera_sequence
                )
                last_camera_sequence = frame_index
                if previous_sequence >= 0:
                    dropped_camera_frames += max(
                        0,
                        frame_index - previous_sequence - 1,
                    )
                if undistort_maps is None:
                    frame_height, frame_width = frame.shape[:2]
                    undistort_maps = cv2.initUndistortRectifyMap(
                        camera_matrix,
                        dist_coeffs,
                        None,
                        camera_matrix,
                        (frame_width, frame_height),
                        cv2.CV_16SC2,
                    )
                undistorted = cv2.remap(
                    frame,
                    undistort_maps[0],
                    undistort_maps[1],
                    interpolation=cv2.INTER_LINEAR,
                )
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

            result = model.track(
                source=bev,
                imgsz=inference_imgsz,
                device=inference_device,
                tracker=tracker_config,
                persist=True,
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
            planning_controller.update(
                scene,
                replan_revision,
            )
            completed_outcome = planning_controller.consume_new_outcome()
            if completed_outcome is not None and ros_publisher is not None:
                ros_publisher.publish_trajectory(
                    getattr(completed_outcome, "trajectory")
                )
            if ros_publisher is not None and scene.vehicle is not None:
                if scene.vehicle.planning_ready:
                    ros_publisher.publish_pose(scene.vehicle)
                ros_publisher.spin_once()
            if write_runtime_files and frame_index % output_every == 0:
                try:
                    # 기존 파일 기반 진단 호환 모드에서만 저장한다.
                    save_bev_image_atomic(bev, bev_output_path)
                except RuntimeError as error:
                    print(f"ERROR: {error}")
                    return 1
                save_scene_observation(scene, output_path)
            now = time.monotonic()
            if now - last_print_time >= 1.0 or static_bev is not None:
                print_scene(scene, route_selector)
                capture_age_ms = (now - captured_at) * 1000.0
                print(
                    "  Live timing: "
                    f"capture_age={capture_age_ms:.1f} ms, "
                    f"dropped_for_freshness={dropped_camera_frames}"
                )
                last_print_time = now

            if not args.no_display:
                canvas = draw_scene(
                    bev,
                    scene,
                    transform,
                    target_slot_name=target_slot_name,
                    detections=detections,
                    route_selector=route_selector,
                )
                canvas = planning_controller.draw_overlay(canvas, transform)
                cv2.putText(
                    canvas,
                    planning_controller.status,
                    (20, 155),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.60,
                    (
                        (0, 0, 255)
                        if planning_controller.last_error is not None
                        else (255, 255, 0)
                    ),
                    2,
                    cv2.LINE_AA,
                )
                capture_age_ms = (time.monotonic() - captured_at) * 1000.0
                latency_color = (
                    (0, 0, 255)
                    if capture_age_ms > latency_warning_ms
                    else (0, 255, 0)
                )
                cv2.putText(
                    canvas,
                    (
                        f"capture age={capture_age_ms:.0f} ms | "
                        f"camera frame={frame_index} | "
                        f"skipped={dropped_camera_frames}"
                    ),
                    (20, 95),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.65,
                    latency_color,
                    2,
                    cv2.LINE_AA,
                )
                cv2.imshow(WINDOW_NAME, canvas)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), 27):
                    break
                if key == ord(" "):
                    completed, message = localizer.complete_charging(
                        scene.vehicle.track_id if scene.vehicle is not None else None,
                        scene.tracked_vehicles,
                    )
                    print(message)
                    if completed:
                        # 이전 C1/C2 경로를 토픽으로 발행하지 않고 P1~P5 경로만
                        # 새로 계획하도록 planner 결과를 무효화한다.
                        replan_revision += 1
                        planning_controller.invalidate()
                elif key == ord("p"):
                    replan_revision += 1
                    print("Manual replan requested")
            processed_frames += 1
            if static_bev is not None:
                break
            if (
                args.max_frames is not None
                and processed_frames >= args.max_frames
            ):
                break
    except RuntimeError as error:
        print(f"ERROR: {error}")
        return 1
    finally:
        if camera is not None:
            camera.close()
        planning_controller.close()
        if ros_publisher is not None:
            ros_publisher.close()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
