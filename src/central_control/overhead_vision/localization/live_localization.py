"""상단 USB Camera BEV에서 차량 pose와 주차면 좌표를 실시간 생성한다.

실행:
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
        "--vehicle-id",
        choices=("vehicle_1", "vehicle_2"),
        help="Initial vehicle receiving pose and trajectory topics.",
    )
    parser.add_argument(
        "--initial-ego-center",
        type=float,
        nargs=2,
        metavar=("X_PX", "Y_PX"),
        help="Override the configured first-frame ego BEV center.",
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


class RoutePublishScheduler:
    """새 경로는 즉시, 유지 중인 경로는 설정 주기로 발행한다."""

    def __init__(self, period_sec: float) -> None:
        if not math.isfinite(period_sec) or period_sec <= 0.0:
            raise ValueError("route republish period must be positive and finite")
        self.period_sec = period_sec
        self.last_publish_time: float | None = None

    def due(self, now: float, has_route: bool, is_new_route: bool = False) -> bool:
        if not math.isfinite(now):
            raise ValueError("route publish time must be finite")
        if not has_route:
            self.last_publish_time = None
            return False
        if (
            is_new_route
            or self.last_publish_time is None
            or now - self.last_publish_time >= self.period_sec
        ):
            self.last_publish_time = now
            return True
        return False


class IntegratedPlanningController:
    """Select one fixed route for each new vehicle/target episode."""

    def __init__(
        self,
        route_selector: object,
        completion_radius_cm: float = 3.0,
    ) -> None:
        if not math.isfinite(completion_radius_cm) or completion_radius_cm <= 0.0:
            raise ValueError("route completion radius must be positive and finite")
        self.route_selector = route_selector
        self.completion_radius_cm = float(completion_radius_cm)
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
        self._new_invalidation = False
        self._discard_pending_result = False
        self._idle_endpoint: str | None = None

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
        route_revision: int,
    ) -> None:
        """새 차량/목표 조합일 때만 고정 경로 선택을 시작한다."""
        self._collect_finished()
        if scene.vehicle is None:
            return
        if self.outcome is not None and self._active_key is not None:
            trajectory = tuple(getattr(self.outcome, "trajectory", ()))
            if not trajectory:
                self.invalidate()
                return
            goal = trajectory[-1]
            goal_distance_cm = math.hypot(
                float(getattr(goal, "x_cm")) - scene.vehicle.center_cm[0],
                float(getattr(goal, "y_cm")) - scene.vehicle.center_cm[1],
            )
            if goal_distance_cm <= self.completion_radius_cm:
                self.invalidate()
                return
            if route_revision == self._active_key[2]:
                # 출발 후에는 YOLO/점유 상태가 잠깐 흔들리거나 다른 목표가
                # 추천돼도 현재 목적지와 trajectory를 유지한다. 운영자의 p/e,
                # 충전 완료처럼 revision을 올린 명시적 전환만 경로를 바꾼다.
                return
        if (
            self._future is not None
            and self._active_key is not None
            and route_revision == self._active_key[2]
        ):
            return
        if scene.planning_request is None:
            if (
                self.outcome is not None
                or self._future is not None
                or self._active_key is not None
            ):
                # 주차칸 polygon에 차체가 일부 겹쳐 planning request가 먼저
                # 사라져도 실제 고정 경로 종점까지는 기존 경로 발행을 유지한다.
                if self.outcome is None:
                    return
                trajectory = tuple(getattr(self.outcome, "trajectory", ()))
                if not trajectory:
                    self.invalidate()
                    return
                goal = trajectory[-1]
                goal_distance_cm = math.hypot(
                    float(getattr(goal, "x_cm")) - scene.vehicle.center_cm[0],
                    float(getattr(goal, "y_cm")) - scene.vehicle.center_cm[1],
                )
                if goal_distance_cm <= self.completion_radius_cm:
                    self.invalidate()
                return
            detector = getattr(self.route_selector, "detect_location", None)
            detected_section = (
                str(
                    detector(
                        (
                            scene.vehicle.center_cm[0],
                            scene.vehicle.center_cm[1],
                            scene.vehicle.yaw_rad,
                        )
                    )
                )
                if callable(detector)
                else "TRANSIT"
            )
            if (
                detected_section != "TRANSIT"
                and detected_section != self._idle_endpoint
            ):
                # endpoint에 세워둔 상태로 프로세스를 새로 시작해도 이전
                # subscriber 경로를 지울 수 있도록 false를 한 번 발행한다.
                self._new_invalidation = True
            self._idle_endpoint = (
                detected_section
                if detected_section != "TRANSIT"
                else None
            )
            return
        self._idle_endpoint = None
        key = (
            scene.vehicle.track_id,
            scene.planning_request.slot_name,
            route_revision,
        )
        if self._future is not None:
            return
        if key == self._completed_key:
            return
        if key != self._active_key:
            # 새 운행 단계 또는 새 충전 배정은 이전 overlay를 숨기고 새 고정
            # 경로가 선택될 때까지 기다린다.
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

    def consume_invalidation(self) -> bool:
        """경로 폐기 이벤트를 한 번만 반환한다."""
        if not self._new_invalidation:
            return False
        self._new_invalidation = False
        return True

    def invalidate(self) -> None:
        """단계 전환 시 이전 목적지의 실행 중/완료 경로를 폐기한다."""
        had_route_state = (
            self.outcome is not None
            or self._future is not None
            or self._active_key is not None
        )
        self.outcome = None
        self.last_error = None
        self._active_key = None
        self._completed_key = None
        self._new_outcome = False
        # 작업 스레드에서 이미 CSV를 읽는 중이면 완료 결과만 버리고, 다음
        # 프레임에서 새 운행 단계의 경로를 선택한다.
        self._discard_pending_result = self._future is not None
        self._new_invalidation = self._new_invalidation or had_route_state

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


def detections_from_result(
    result: object,
    minimum_confidence_by_class: dict[str, float] | None = None,
) -> list[Detection]:
    """YOLO 결과를 변환하며 클래스별 추가 confidence 기준을 적용한다."""
    detections: list[Detection] = []
    thresholds = minimum_confidence_by_class or {}
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
        if confidence < thresholds.get(class_name, 0.0):
            continue
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
    # 제어 pose와 고정 경로는 차량 중심 기준이다. 아래 offset은 화면에 rear
    # axle 진단점을 함께 표시하고 차동구동 물리 모델을 검증할 때만 사용한다.
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
        # slot, keep the configured one-way START/corridor heading.
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
    vehicle_min_confidence = float(config.get("car_confidence", 0.5))
    return (
        SceneLocalizer(
            tracker,
            slots,
            vehicle_state_manager=vehicle_states,
            charge_coordinator=charge_coordinator,
            target_slot_name=target_slot_name,
            parking_assignment=parking_assignment,
            post_charge_parking_assignment=post_charge_parking_assignment,
            vehicle_min_confidence=vehicle_min_confidence,
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


def load_operational_space_polygons(
    config: dict[str, object],
) -> dict[str, tuple[tuple[float, float], ...]]:
    """입구·출구의 BEV polygon을 관제 상태 판정용으로 읽는다."""
    points_path = resolve_path(str(config["parking_points_path"]))
    with points_path.open(encoding="utf-8") as file:
        points = json.load(file)
    polygons: dict[str, tuple[tuple[float, float], ...]] = {}
    corner_names = (
        "top_left_bev",
        "top_right_bev",
        "bottom_right_bev",
        "bottom_left_bev",
    )
    for name in ("entrance", "exit"):
        raw_space = points.get(name)
        if not isinstance(raw_space, dict):
            raise ValueError(f"parking operational space is missing: {name}")
        polygon = []
        for corner_name in corner_names:
            point = raw_space.get(corner_name)
            if not isinstance(point, list) or len(point) != 2:
                raise ValueError(
                    f"{name} has invalid operational-space corner: {corner_name}"
                )
            polygon.append((float(point[0]), float(point[1])))
        polygons[name] = tuple(polygon)
    return polygons


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
    # 차량과 주차칸 점유용 dummy 검출을 함께 표시한다. ego 차량은 아래에서
    # 더 굵은 외곽선과 차량 중심 pose로 다시 강조한다.
    for detection in detections:
        if detection.class_name not in {"car", "dummy"}:
            continue
        polygon = np.rint(detection.polygon_bev).astype(np.int32)
        detection_color = (
            (255, 128, 0) if detection.class_name == "car" else (0, 165, 255)
        )
        cv2.polylines(
            canvas,
            [polygon],
            True,
            detection_color,
            1,
            cv2.LINE_AA,
        )
        x1, y1, _, _ = detection.bbox_xyxy
        if detection.class_name == "dummy":
            label = f"dummy OCC {detection.confidence:.2f}"
        else:
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
            detection_color,
            2,
            cv2.LINE_AA,
        )
    if scene.vehicle is not None:
        vehicle = scene.vehicle
        polygon = np.rint(vehicle.polygon_bev).astype(np.int32)
        color = (0, 165, 255) if not vehicle.planning_ready else (255, 255, 0)
        cv2.polylines(canvas, [polygon], True, color, 3, cv2.LINE_AA)
        center = tuple(round(value) for value in vehicle.center_bev_px)
        cv2.circle(canvas, center, 6, (0, 0, 255), -1)
        center_cm = (
            vehicle.center_lidar_px[0] * transform.resolution_cm,
            vehicle.center_lidar_px[1] * transform.resolution_cm,
        )
        cv2.putText(
            canvas,
            (
                f"ego id={vehicle.track_id} "
                f"camera-center=({center_cm[0]:.1f}, {center_cm[1]:.1f}) cm"
            ),
            (max(0, center[0] - 120), max(25, center[1] - 18)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            color,
            2,
            cv2.LINE_AA,
        )
    return canvas


def draw_runtime_hud(
    image: np.ndarray,
    lines: Sequence[tuple[str, tuple[int, int, int]]],
) -> np.ndarray:
    """배경과 외곽선 없이 상태 문구를 고정 행에 한 번만 그린다."""
    if not lines:
        return image
    canvas = image.copy()
    _, width = canvas.shape[:2]
    panel_x, panel_y = 12, 12
    line_height = 28
    panel_width = min(1040, max(300, width - panel_x * 2))
    max_text_width = panel_width - 24
    for index, (label, color) in enumerate(lines):
        fitted = str(label)
        while fitted and cv2.getTextSize(
            fitted,
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            1,
        )[0][0] > max_text_width:
            fitted = fitted[:-1]
        if fitted != label:
            fitted = fitted.rstrip() + "..."
        origin = (panel_x + 12, panel_y + 22 + index * line_height)
        cv2.putText(
            canvas,
            fitted,
            origin,
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            color,
            1,
            cv2.LINE_AA,
        )
    return canvas


def draw_lidar_map_scene(
    lidar_map: np.ndarray,
    scene: SceneObservation,
    display_scale: int = 4,
) -> np.ndarray:
    """실제 LiDAR 맵에 같은 YOLO 프레임의 차량 좌표를 빨간 점으로 그린다."""
    canvas = cv2.cvtColor(lidar_map, cv2.COLOR_GRAY2BGR)
    canvas = cv2.resize(
        canvas,
        (lidar_map.shape[1] * display_scale, lidar_map.shape[0] * display_scale),
        interpolation=cv2.INTER_NEAREST,
    )
    for vehicle in scene.tracked_vehicles:
        if not vehicle.visible:
            continue
        center = (
            round(vehicle.center_lidar_px[0] * display_scale),
            round(vehicle.center_lidar_px[1] * display_scale),
        )
        if not (0 <= center[0] < canvas.shape[1] and 0 <= center[1] < canvas.shape[0]):
            continue
        cv2.circle(canvas, center, 10, (0, 0, 255), -1, cv2.LINE_AA)
        cv2.circle(canvas, center, 14, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(
            canvas,
            (
                f"id={vehicle.track_id} "
                f"({vehicle.position_cm[0]:.1f}, {vehicle.position_cm[1]:.1f}) cm"
            ),
            (center[0] + 16, max(24, center[1] - 12)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 0, 255),
            2,
            cv2.LINE_AA,
        )
    cv2.putText(
        canvas,
        f"frame={scene.frame_index} | red=live YOLO LiDAR coordinate",
        (14, 26),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (0, 255, 255),
        2,
        cv2.LINE_AA,
    )
    return canvas


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
                            scene.vehicle.center_cm[0],
                            scene.vehicle.center_cm[1],
                            scene.vehicle.yaw_rad,
                        )
                    )
                )
        else:
            current_section = "TRANSIT"

    target = "WAIT"
    if scene.planning_request is not None:
        requested_target = scene.planning_request.slot_name
        allowed_transitions = getattr(
            route_selector,
            "allowed_transitions",
            {},
        )
        allowed_targets = allowed_transitions.get(current_section, ())
        if requested_target in allowed_targets:
            target = requested_target
    return current_section, target


def print_scene(
    scene: SceneObservation,
    route_selector: object | None = None,
) -> None:
    print(f"Frame {scene.frame_index}: {scene.status}")
    if scene.vehicle:
        vehicle = scene.vehicle
        print(
            "  Ego vehicle center: "
            f"({vehicle.center_cm[0]:.2f}, {vehicle.center_cm[1]:.2f}) cm, "
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
    if (
        scene.charge_assignment is not None
        and scene.vehicle is not None
        and scene.charge_assignment.vehicle_track_id == scene.vehicle.track_id
    ):
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
        if args.vehicle_id is not None:
            config["vehicle_id"] = args.vehicle_id
        if args.initial_ego_center is not None:
            config["initial_ego_center_bev_px"] = list(args.initial_ego_center)
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
        lidar_map_path = resolve_path(str(config["lidar_map_path"]))
        ros_publisher = (
            DirectRosPublisher(
                vehicle_id=str(config.get("vehicle_id", "vehicle_1")),
                image_topic=str(
                    config.get("ros_image_topic", "/pinkk/localization/image")
                ),
                lidar_image_topic=str(
                    config.get("ros_lidar_image_topic", "/pinkk/lidar_map/image")
                ),
                management_status_topic=str(
                    config.get(
                        "ros_management_status_topic",
                        "/pinkk/management/status",
                    )
                ),
                lidar_map_path=lidar_map_path,
                lidar_resolution_cm=transform.resolution_cm,
                lidar_association_period_sec=float(
                    config.get("lidar_association_period_sec", 0.75)
                ),
                lidar_scan_timeout_sec=float(
                    config.get("lidar_scan_timeout_sec", 1.5)
                ),
                lidar_position_search_half_width_m=float(
                    config.get("lidar_position_search_half_width_m", 0.25)
                ),
                lidar_maximum_match_score_m=float(
                    config.get("lidar_maximum_match_score_m", 0.08)
                ),
                lidar_minimum_assignment_margin_m=float(
                    config.get("lidar_minimum_assignment_margin_m", 0.01)
                ),
                lidar_required_confirmations=int(
                    config.get("lidar_required_confirmations", 2)
                ),
                operational_space_polygons=load_operational_space_polygons(config),
            )
            if ros_publish_enabled
            else None
        )
        lidar_map = cv2.imread(str(lidar_map_path), cv2.IMREAD_GRAYSCALE)
        if lidar_map is None:
            raise FileNotFoundError(f"LiDAR map image not found: {lidar_map_path}")
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
            f"{ros_publisher.trajectory_topic}, {ros_publisher.path_valid_topic}"
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
    print("Automatic vehicle selection enabled | SPACE: charge complete | q/ESC: quit")
    planning_controller = IntegratedPlanningController(
        route_selector,
        completion_radius_cm=float(
            config.get("route_completion_radius_cm", 3.0)
        ),
    )
    route_publish_scheduler = RoutePublishScheduler(
        float(config.get("route_republish_period_sec", 1.0))
    )
    print(
        "Fixed route publish: immediate + every "
        f"{route_publish_scheduler.period_sec:.2f} sec"
    )
    replan_revision = 0
    automatic_target_initialized = False
    if not args.no_display:
        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    # 첫 실제 프레임 추론 전에 CUDA 커널과 모델을 준비한다. 준비 중에도 캡처
    # 스레드는 카메라를 계속 비우므로 완료 후 가장 최근 프레임부터 처리한다.
    inference_imgsz = int(config.get("inference_imgsz", 1600))
    inference_device = config.get("inference_device", 0)
    tracker_config = str(config.get("tracker_config", "bytetrack.yaml"))
    car_confidence = float(config.get("car_confidence", 0.5))
    if not math.isfinite(car_confidence) or not 0.0 <= car_confidence <= 1.0:
        print(
            "ERROR: car_confidence must be finite and between 0 and 1, "
            f"got {car_confidence}"
        )
        return 1
    occupancy_car_confidence = float(
        config.get("occupancy_car_confidence", config["confidence"])
    )
    if (
        not math.isfinite(occupancy_car_confidence)
        or not 0.0 <= occupancy_car_confidence <= car_confidence
    ):
        print(
            "ERROR: occupancy_car_confidence must be finite and between 0 "
            f"and car_confidence ({car_confidence}), "
            f"got {occupancy_car_confidence}"
        )
        return 1
    print(
        "Detection confidence: "
        f"base={float(config['confidence']):.2f}, "
        f"occupancy_car={occupancy_car_confidence:.2f}, "
        f"ego_car={car_confidence:.2f}"
    )
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
            detections = detections_from_result(
                result,
                minimum_confidence_by_class={
                    "car": occupancy_car_confidence,
                },
            )
            scene = localizer.observe(
                detections,
                bev.shape[:2],
                frame_index,
            )
            identity_ready = ros_publisher is None
            if ros_publisher is not None:
                # scan/control callback을 먼저 처리해야 이번 프레임 경로가 최신
                # vehicle namespace와 LiDAR-camera identity를 사용한다.
                ros_publisher.spin_once()
                path_target_request = ros_publisher.consume_path_target_request()
                if path_target_request is not None:
                    selected_vehicle_id, requested_command = path_target_request
                    automatic_target_initialized = True
                    replan_revision += 1
                    planning_controller.invalidate()
                    route_publish_scheduler.due(
                        time.monotonic(),
                        has_route=False,
                    )
                    print(
                        "Automatic ROS path target: "
                        f"{selected_vehicle_id} command={requested_command} -> "
                        f"{ros_publisher.trajectory_topic}"
                    )
                association = ros_publisher.update_vehicle_association(
                    scene.tracked_vehicles
                )
                if (
                    not automatic_target_initialized
                    and association is not None
                    and scene.vehicle is not None
                ):
                    automatic_vehicle_id = next(
                        (
                            vehicle_id
                            for vehicle_id, track_id in (
                                association.vehicle_to_track.items()
                            )
                            if track_id == scene.vehicle.track_id
                        ),
                        None,
                    )
                    if automatic_vehicle_id is not None:
                        ros_publisher.select_vehicle(automatic_vehicle_id)
                        automatic_target_initialized = True
                        print(
                            "Initial ROS path target selected automatically: "
                            f"{automatic_vehicle_id}=track_{scene.vehicle.track_id}"
                        )
                target_track_id = (
                    association.vehicle_to_track.get(
                        ros_publisher.active_vehicle_id
                    )
                    if association is not None
                    else None
                )
                if target_track_id is not None and localizer.select_ego(
                    target_track_id
                ):
                    replan_revision += 1
                    planning_controller.invalidate()
                    route_publish_scheduler.due(
                        time.monotonic(),
                        has_route=False,
                    )
                    print(
                        "Automatic camera ego selected: "
                        f"{ros_publisher.active_vehicle_id}=track_{target_track_id}"
                    )
                identity_ready = (
                    target_track_id is not None
                    and scene.vehicle is not None
                    and scene.vehicle.track_id == target_track_id
                )
            if identity_ready:
                planning_controller.update(
                    scene,
                    replan_revision,
                )
            else:
                planning_controller.invalidate()
            if (
                planning_controller.consume_invalidation()
                and ros_publisher is not None
            ):
                ros_publisher.invalidate_trajectory()
            completed_outcome = planning_controller.consume_new_outcome()
            active_outcome = planning_controller.outcome
            publish_now = route_publish_scheduler.due(
                time.monotonic(),
                has_route=active_outcome is not None,
                is_new_route=completed_outcome is not None,
            )
            if publish_now and ros_publisher is not None and identity_ready:
                ros_publisher.publish_trajectory(
                    getattr(active_outcome, "trajectory")
                )
            if ros_publisher is not None:
                if (
                    identity_ready
                    and scene.vehicle is not None
                    and scene.vehicle.planning_ready
                ):
                    ros_publisher.publish_pose(
                        scene.vehicle,
                        measurement_age_sec=max(
                            0.0,
                            time.monotonic() - captured_at,
                        ),
                    )
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

            # --no-display는 GUI만 끄며 웹용 ROS 영상 발행은 계속한다.
            if ros_publisher is not None or not args.no_display:
                canvas = draw_scene(
                    bev,
                    scene,
                    transform,
                    target_slot_name=target_slot_name,
                    detections=detections,
                    route_selector=route_selector,
                )
                canvas = planning_controller.draw_overlay(canvas, transform)
                current_section, route_target = route_context(
                    scene,
                    route_selector,
                )
                hud_lines: list[tuple[str, tuple[int, int, int]]] = []
                if ros_publisher is not None:
                    active_lidar_pose = ros_publisher.active_lidar_pose
                    identity_text = (
                        f"track={ros_publisher.active_track_id} "
                        f"lidar=({active_lidar_pose.position_x_m:.2f},"
                        f"{active_lidar_pose.position_y_m:.2f})"
                        if active_lidar_pose is not None
                        else ros_publisher.association_status
                    )
                    hud_lines.append(
                        (
                            (
                                f"VEHICLE  {ros_publisher.active_vehicle_id}  |  "
                                "NAMESPACE  "
                                f"{ros_publisher.vehicle.ros_namespace}"
                            ),
                            (0, 255, 255),
                        )
                    )
                    hud_lines.append(
                        (
                            f"IDENTITY  {identity_text}  |  "
                            f"TOPIC  {ros_publisher.trajectory_topic}",
                            (0, 220, 255),
                        )
                    )
                else:
                    hud_lines.append(("ROS  disabled", (0, 165, 255)))
                hud_lines.append(
                    (
                        f"ROUTE  {current_section} -> {route_target}  |  "
                        f"{planning_controller.status}",
                        (
                            (0, 0, 255)
                            if planning_controller.last_error is not None
                            else (255, 255, 0)
                        ),
                    )
                )
                hud_lines.append(
                    (
                        f"SCENE  ready={scene.planning_ready}  |  {scene.status}",
                        (220, 220, 220),
                    )
                )
                if scene.charge_assignment is not None:
                    assignment = scene.charge_assignment
                    hud_lines.append(
                        (
                            f"CHARGE  track={assignment.vehicle_track_id} -> "
                            f"{assignment.target_slot_name or 'WAIT'}  |  "
                            f"{assignment.status}",
                            (255, 0, 255),
                        )
                    )
                capture_age_ms = (time.monotonic() - captured_at) * 1000.0
                latency_color = (
                    (0, 0, 255)
                    if capture_age_ms > latency_warning_ms
                    else (0, 255, 0)
                )
                hud_lines.append(
                    (
                        (
                            f"CAMERA  age={capture_age_ms:.0f}ms  |  "
                            f"frame={frame_index}  |  "
                            f"skipped={dropped_camera_frames}"
                        ),
                        latency_color,
                    ),
                )
                canvas = draw_runtime_hud(canvas, hud_lines)
                if ros_publisher is not None:
                    ros_publisher.publish_image(canvas)
                    ros_publisher.publish_lidar_image(
                        draw_lidar_map_scene(lidar_map, scene)
                    )
            if not args.no_display:
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
                        # 이전 C1/C2 경로를 토픽으로 발행하지 않고 P1~P4 경로만
                        # 새로 계획하도록 planner 결과를 무효화한다.
                        replan_revision += 1
                        planning_controller.invalidate()
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
