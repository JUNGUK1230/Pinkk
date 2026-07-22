"""YOLO BEV mask를 차량 pose와 주차면 goal 좌표로 변환한다."""

from dataclasses import dataclass
import json
import math
from pathlib import Path
import time
from typing import Sequence

import cv2
import numpy as np


Point = tuple[float, float]
Pose = tuple[float, float, float]


@dataclass(frozen=True)
class Detection:
    """Ultralytics와 분리된 한 개의 segmentation 검출 결과."""

    class_name: str
    confidence: float
    polygon_bev: np.ndarray
    bbox_xyxy: tuple[float, float, float, float]


@dataclass(frozen=True)
class VehicleObservation:
    confidence: float
    center_bev_px: Point
    center_lidar_px: Point
    rear_axle_cm: Point
    yaw_rad: float
    yaw_deg: float
    elongation: float
    heading_ambiguous: bool
    ego_selection_ambiguous: bool
    polygon_bev: np.ndarray

    @property
    def planning_ready(self) -> bool:
        return not self.heading_ambiguous and not self.ego_selection_ambiguous

    def to_dict(self) -> dict[str, object]:
        return {
            "confidence": self.confidence,
            "center_bev_px": list(self.center_bev_px),
            "center_lidar_px": list(self.center_lidar_px),
            "rear_axle_cm": list(self.rear_axle_cm),
            "yaw_rad": self.yaw_rad,
            "yaw_deg": self.yaw_deg,
            "elongation": self.elongation,
            "heading_ambiguous": self.heading_ambiguous,
            "ego_selection_ambiguous": self.ego_selection_ambiguous,
            "planning_ready": self.planning_ready,
        }


@dataclass(frozen=True)
class ParkingSlotObservation:
    name: str
    center_bev_px: Point
    center_lidar_px: Point
    occupancy_ratio: float
    occupied: bool
    goal_pose_candidates_cm: tuple[Pose, Pose]
    polygon_bev: np.ndarray

    def to_dict(self) -> dict[str, object]:
        return {
            "name": self.name,
            "center_bev_px": list(self.center_bev_px),
            "center_lidar_px": list(self.center_lidar_px),
            "occupancy_ratio": self.occupancy_ratio,
            "occupied": self.occupied,
            "goal_pose_candidates_cm": [
                {"x_cm": pose[0], "y_cm": pose[1], "yaw_rad": pose[2]}
                for pose in self.goal_pose_candidates_cm
            ],
        }


@dataclass(frozen=True)
class ParkingAssignmentPolicy:
    """입구·출구 기준으로 빈 주차칸 후보를 정렬하는 공통 운영 규칙."""

    name: str
    reference_bev_px: Point
    allowed_slots: tuple[str, ...]
    preference: str
    candidate_limit: int

    def __post_init__(self) -> None:
        if self.preference not in ("nearest", "farthest"):
            raise ValueError("parking preference must be nearest or farthest")
        if self.candidate_limit <= 0:
            raise ValueError("parking candidate limit must be positive")
        if not self.allowed_slots:
            raise ValueError("parking assignment must contain allowed slots")

    def rank_free_slots(
        self,
        slots: Sequence[ParkingSlotObservation],
    ) -> tuple[ParkingSlotObservation, ...]:
        """점유된 칸을 제외하고 기준점 실제 BEV 거리 순으로 정렬한다."""
        allowed = set(self.allowed_slots)
        free_slots = [
            slot for slot in slots if slot.name in allowed and not slot.occupied
        ]
        ranked = sorted(
            free_slots,
            key=lambda slot: math.hypot(
                slot.center_bev_px[0] - self.reference_bev_px[0],
                slot.center_bev_px[1] - self.reference_bev_px[1],
            ),
            reverse=self.preference == "farthest",
        )
        return tuple(ranked[: self.candidate_limit])


@dataclass(frozen=True)
class PlanningRequest:
    slot_name: str
    start_pose_cm: Pose
    goal_pose_cm: Pose
    alternative_goal_pose_cm: Pose
    candidate_slot_names: tuple[str, ...] = ()
    assignment_policy: str | None = None

    def to_dict(self) -> dict[str, object]:
        return {
            "slot_name": self.slot_name,
            "start_pose_cm": _pose_dict(self.start_pose_cm),
            "goal_pose_cm": _pose_dict(self.goal_pose_cm),
            "alternative_goal_pose_cm": _pose_dict(
                self.alternative_goal_pose_cm
            ),
            "candidate_slot_names": list(self.candidate_slot_names),
            "assignment_policy": self.assignment_policy,
        }


@dataclass(frozen=True)
class SceneObservation:
    frame_index: int
    observed_at_unix_sec: float
    vehicle: VehicleObservation | None
    parking_slots: tuple[ParkingSlotObservation, ...]
    planning_request: PlanningRequest | None
    status: str

    @property
    def planning_ready(self) -> bool:
        return self.planning_request is not None

    def to_dict(self) -> dict[str, object]:
        return {
            "frame": "lidar_map_cm",
            "frame_index": self.frame_index,
            "observed_at_unix_sec": self.observed_at_unix_sec,
            "status": self.status,
            "planning_ready": self.planning_ready,
            "vehicle": self.vehicle.to_dict() if self.vehicle else None,
            "parking_slots": [slot.to_dict() for slot in self.parking_slots],
            "planning_request": (
                self.planning_request.to_dict()
                if self.planning_request is not None
                else None
            ),
        }


class AffineBevToLidar:
    """Camera BEV pixel을 LiDAR pixel·planner cm로 변환한다."""

    def __init__(self, registration_path: str | Path) -> None:
        path = Path(registration_path)
        if not path.exists():
            raise FileNotFoundError(f"registration file not found: {path}")
        with np.load(path) as data:
            self.matrix = np.asarray(data["affine_matrix"], dtype=np.float64)
            self.lidar_width = int(data["lidar_width"])
            self.lidar_height = int(data["lidar_height"])
            self.resolution_cm = float(data["resolution"]) * 100.0
        if self.matrix.shape != (2, 3):
            raise ValueError("affine_matrix must have shape (2, 3)")
        self.inverse_matrix = cv2.invertAffineTransform(self.matrix)
        if self.resolution_cm <= 0.0:
            raise ValueError("registration resolution must be positive")

    def point_to_lidar(self, point: Point) -> Point:
        result = self.matrix @ np.asarray([point[0], point[1], 1.0])
        return float(result[0]), float(result[1])

    def point_to_planner_cm(self, point: Point) -> Point:
        x_px, y_px = self.point_to_lidar(point)
        return x_px * self.resolution_cm, y_px * self.resolution_cm

    def planner_cm_to_bev(self, point: Point) -> Point:
        lidar_point = np.asarray(
            [
                point[0] / self.resolution_cm,
                point[1] / self.resolution_cm,
                1.0,
            ]
        )
        result = self.inverse_matrix @ lidar_point
        return float(result[0]), float(result[1])

    def axis_yaw_in_lidar(self, center: Point, axis: Point) -> float:
        start = self.point_to_lidar(center)
        end = self.point_to_lidar(
            (center[0] + axis[0] * 20.0, center[1] + axis[1] * 20.0)
        )
        return _normalize_yaw(math.atan2(end[1] - start[1], end[0] - start[0]))


class EgoVehicleTracker:
    """여러 car mask 중 ego를 선택하고 중심·장축 yaw를 시간적으로 추적한다."""

    def __init__(
        self,
        transform: AffineBevToLidar,
        rear_axle_offset_cm: float,
        initial_center_bev_px: Point | None = None,
        initial_yaw_rad: float | None = None,
        position_alpha: float = 0.5,
        yaw_alpha: float = 0.35,
        max_center_jump_px: float = 250.0,
        minimum_elongation: float = 1.05,
    ) -> None:
        if rear_axle_offset_cm < 0.0:
            raise ValueError("rear_axle_offset_cm must not be negative")
        if not 0.0 < position_alpha <= 1.0 or not 0.0 < yaw_alpha <= 1.0:
            raise ValueError("tracking alpha values must be in (0, 1]")
        self.transform = transform
        self.rear_axle_offset_cm = rear_axle_offset_cm
        self.initial_center_bev_px = initial_center_bev_px
        self.initial_yaw_rad = (
            _normalize_yaw(initial_yaw_rad)
            if initial_yaw_rad is not None
            else None
        )
        self.position_alpha = position_alpha
        self.yaw_alpha = yaw_alpha
        self.max_center_jump_px = max_center_jump_px
        self.minimum_elongation = minimum_elongation
        self.previous_center_bev: Point | None = None
        self.previous_yaw_rad: float | None = self.initial_yaw_rad
        self.manual_yaw_rad: float | None = None
        # Mask의 장축은 방향축만 제공한다. 초기 yaw가 없으면 앞뒤 절대 방향은
        # 이후 프레임에서도 결정할 수 없으므로 계속 ambiguous로 유지한다.
        self.absolute_heading_resolved = self.initial_yaw_rad is not None

    def set_manual_heading(self, yaw_rad: float) -> None:
        """사용자가 Camera BEV에서 지정한 차량 앞 방향을 고정 heading으로 사용한다."""
        if not math.isfinite(yaw_rad):
            raise ValueError("manual yaw must be finite")
        self.manual_yaw_rad = _normalize_yaw(yaw_rad)
        self.previous_yaw_rad = self.manual_yaw_rad
        self.absolute_heading_resolved = True

    def clear_manual_heading(self) -> None:
        """수동 heading을 지우고 다시 사용자 입력이 필요한 상태로 되돌린다."""
        self.manual_yaw_rad = None
        self.previous_yaw_rad = self.initial_yaw_rad
        self.absolute_heading_resolved = self.initial_yaw_rad is not None

    def update(self, detections: Sequence[Detection]) -> VehicleObservation | None:
        car_detections = [item for item in detections if item.class_name == "car"]
        if not car_detections:
            return None
        centers = [_polygon_center(item.polygon_bev) for item in car_detections]
        selection_reference = self.previous_center_bev or self.initial_center_bev_px
        selection_ambiguous = selection_reference is None and len(car_detections) > 1
        if selection_reference is None:
            selected_index = max(
                range(len(car_detections)),
                key=lambda index: car_detections[index].confidence,
            )
        else:
            distances = [
                math.hypot(
                    center[0] - selection_reference[0],
                    center[1] - selection_reference[1],
                )
                for center in centers
            ]
            selected_index = min(range(len(distances)), key=distances.__getitem__)
            if (
                self.previous_center_bev is not None
                and distances[selected_index] > self.max_center_jump_px
            ):
                return None

        detection = car_detections[selected_index]
        raw_center = centers[selected_index]
        if self.previous_center_bev is None:
            center = raw_center
        else:
            center = (
                self.previous_center_bev[0]
                + self.position_alpha * (raw_center[0] - self.previous_center_bev[0]),
                self.previous_center_bev[1]
                + self.position_alpha * (raw_center[1] - self.previous_center_bev[1]),
            )
        axis, elongation = _principal_axis(detection.polygon_bev)
        if self.manual_yaw_rad is not None:
            # 테스트 단계에서는 mask 장축으로 heading을 갱신하지 않는다.
            # 사용자가 다시 지정할 때까지 클릭한 절대 방향을 그대로 유지한다.
            yaw = self.manual_yaw_rad
            heading_ambiguous = False
        else:
            axis_yaw = self.transform.axis_yaw_in_lidar(center, axis)
            yaw_candidates = (axis_yaw, _normalize_yaw(axis_yaw + math.pi))
            reference_yaw = self.previous_yaw_rad
            if reference_yaw is None:
                yaw = yaw_candidates[0]
            else:
                measured_yaw = min(
                    yaw_candidates,
                    key=lambda value: abs(
                        _angle_difference(value, reference_yaw)
                    ),
                )
                yaw = _normalize_yaw(
                    reference_yaw
                    + self.yaw_alpha
                    * _angle_difference(measured_yaw, reference_yaw)
                )
            heading_ambiguous = (
                not self.absolute_heading_resolved
                or elongation < self.minimum_elongation
            )
        center_lidar = self.transform.point_to_lidar(center)
        center_cm = (
            center_lidar[0] * self.transform.resolution_cm,
            center_lidar[1] * self.transform.resolution_cm,
        )
        rear_axle_cm = (
            center_cm[0] - self.rear_axle_offset_cm * math.cos(yaw),
            center_cm[1] - self.rear_axle_offset_cm * math.sin(yaw),
        )
        self.previous_center_bev = center
        self.previous_yaw_rad = yaw
        return VehicleObservation(
            confidence=detection.confidence,
            center_bev_px=center,
            center_lidar_px=center_lidar,
            rear_axle_cm=rear_axle_cm,
            yaw_rad=yaw,
            yaw_deg=math.degrees(yaw),
            elongation=elongation,
            heading_ambiguous=heading_ambiguous,
            ego_selection_ambiguous=selection_ambiguous,
            polygon_bev=detection.polygon_bev,
        )


class ParkingSlotMap:
    """고정 BEV 주차면을 LiDAR 좌표와 점유 상태로 제공한다."""

    def __init__(
        self,
        slots_path: str | Path,
        transform: AffineBevToLidar,
        rear_axle_offset_cm: float,
        occupancy_threshold: float = 0.10,
    ) -> None:
        path = Path(slots_path)
        if not path.exists():
            raise FileNotFoundError(f"parking slots file not found: {path}")
        with path.open(encoding="utf-8") as file:
            raw = json.load(file)
        self.polygons = {
            str(name): np.asarray(points, dtype=np.float64)
            for name, points in raw.items()
        }
        if not self.polygons:
            raise ValueError("parking slots file is empty")
        if not 0.0 <= occupancy_threshold <= 1.0:
            raise ValueError("occupancy_threshold must be in [0, 1]")
        self.transform = transform
        self.rear_axle_offset_cm = rear_axle_offset_cm
        self.occupancy_threshold = occupancy_threshold

    def observe(
        self,
        detections: Sequence[Detection],
        image_shape: tuple[int, int],
    ) -> tuple[ParkingSlotObservation, ...]:
        height, width = image_shape
        vehicle_mask = np.zeros((height, width), dtype=np.uint8)
        for detection in detections:
            if detection.class_name != "car":
                continue
            polygon = np.rint(detection.polygon_bev).astype(np.int32)
            if len(polygon) >= 3:
                cv2.fillPoly(vehicle_mask, [polygon], 255)

        observations: list[ParkingSlotObservation] = []
        for name, polygon in self.polygons.items():
            slot_mask = np.zeros((height, width), dtype=np.uint8)
            polygon_int = np.rint(polygon).astype(np.int32)
            cv2.fillPoly(slot_mask, [polygon_int], 255)
            slot_area = cv2.countNonZero(slot_mask)
            overlap = cv2.bitwise_and(slot_mask, vehicle_mask)
            ratio = cv2.countNonZero(overlap) / max(slot_area, 1)
            center = _polygon_center(polygon)
            axis, _ = _principal_axis(polygon)
            axis_yaw = self.transform.axis_yaw_in_lidar(center, axis)
            center_lidar = self.transform.point_to_lidar(center)
            center_cm = (
                center_lidar[0] * self.transform.resolution_cm,
                center_lidar[1] * self.transform.resolution_cm,
            )
            yaw_candidates = (axis_yaw, _normalize_yaw(axis_yaw + math.pi))
            goals = tuple(
                (
                    center_cm[0] - self.rear_axle_offset_cm * math.cos(yaw),
                    center_cm[1] - self.rear_axle_offset_cm * math.sin(yaw),
                    yaw,
                )
                for yaw in yaw_candidates
            )
            observations.append(
                ParkingSlotObservation(
                    name=name,
                    center_bev_px=center,
                    center_lidar_px=center_lidar,
                    occupancy_ratio=ratio,
                    occupied=ratio >= self.occupancy_threshold,
                    goal_pose_candidates_cm=(goals[0], goals[1]),
                    polygon_bev=polygon,
                )
            )
        return tuple(observations)


class SceneLocalizer:
    """Ego pose, 주차면 점유, 지정 또는 가까운 free goal을 생성한다."""

    def __init__(
        self,
        tracker: EgoVehicleTracker,
        parking_slots: ParkingSlotMap,
        goal_heading_weight_cm: float = 14.0,
        target_slot_name: str | None = None,
        parking_assignment: ParkingAssignmentPolicy | None = None,
    ) -> None:
        self.tracker = tracker
        self.parking_slots = parking_slots
        self.goal_heading_weight_cm = goal_heading_weight_cm
        self.target_slot_name = target_slot_name
        self.parking_assignment = parking_assignment

    def observe(
        self,
        detections: Sequence[Detection],
        image_shape: tuple[int, int],
        frame_index: int,
        observed_at_unix_sec: float | None = None,
    ) -> SceneObservation:
        observed_at = (
            time.time()
            if observed_at_unix_sec is None
            else observed_at_unix_sec
        )
        vehicle = self.tracker.update(detections)
        slots = self.parking_slots.observe(detections, image_shape)
        if vehicle is None:
            return SceneObservation(
                frame_index,
                observed_at,
                None,
                slots,
                None,
                "ego vehicle not detected",
            )
        if not vehicle.planning_ready:
            return SceneObservation(
                frame_index,
                observed_at,
                vehicle,
                slots,
                None,
                "ego heading is required; press h and click the vehicle front",
            )
        if self.target_slot_name is not None:
            target_slot = next(
                (slot for slot in slots if slot.name == self.target_slot_name),
                None,
            )
            if target_slot is None:
                return SceneObservation(
                    frame_index,
                    observed_at,
                    vehicle,
                    slots,
                    None,
                    f"target parking slot not found: {self.target_slot_name}",
                )
            if target_slot.occupied:
                return SceneObservation(
                    frame_index,
                    observed_at,
                    vehicle,
                    slots,
                    None,
                    f"target parking slot is occupied: {self.target_slot_name}",
                )
            empty_slots = [target_slot]
            candidate_slot_names = (target_slot.name,)
            assignment_policy = "fixed_target"
        elif self.parking_assignment is not None:
            empty_slots = list(self.parking_assignment.rank_free_slots(slots))
            if not empty_slots:
                return SceneObservation(
                    frame_index,
                    observed_at,
                    vehicle,
                    slots,
                    None,
                    "no free parking slot in active assignment policy",
                )
            candidate_slot_names = tuple(slot.name for slot in empty_slots)
            assignment_policy = self.parking_assignment.name
        else:
            empty_slots = [slot for slot in slots if not slot.occupied]
            candidate_slot_names = tuple(slot.name for slot in empty_slots)
            assignment_policy = None
        if not empty_slots:
            return SceneObservation(
                frame_index,
                observed_at,
                vehicle,
                slots,
                None,
                "no empty parking slot",
            )

        start = (
            vehicle.rear_axle_cm[0],
            vehicle.rear_axle_cm[1],
            vehicle.yaw_rad,
        )
        ranked: list[tuple[float, ParkingSlotObservation, Pose, Pose]] = []
        for slot in empty_slots:
            first, second = slot.goal_pose_candidates_cm
            candidates = sorted(
                (first, second),
                key=lambda pose: (
                    math.hypot(pose[0] - start[0], pose[1] - start[1])
                    + self.goal_heading_weight_cm
                    * abs(_angle_difference(pose[2], start[2]))
                ),
            )
            score = (
                math.hypot(candidates[0][0] - start[0], candidates[0][1] - start[1])
                + self.goal_heading_weight_cm
                * abs(_angle_difference(candidates[0][2], start[2]))
            )
            ranked.append((score, slot, candidates[0], candidates[1]))
        if self.parking_assignment is not None and self.target_slot_name is None:
            # 정책에서 이미 입구·출구 거리 기준 우선순위를 정했으므로, 첫 빈 칸만
            # 선택한다. yaw 정렬 점수는 같은 칸의 두 rear-axle pose 순서에만 쓴다.
            _, selected_slot, goal, alternative = ranked[0]
        else:
            _, selected_slot, goal, alternative = min(
                ranked, key=lambda item: item[0]
            )
        request = PlanningRequest(
            selected_slot.name,
            start,
            goal,
            alternative,
            candidate_slot_names,
            assignment_policy,
        )
        return SceneObservation(
            frame_index,
            observed_at,
            vehicle,
            slots,
            request,
            (
                "planning input ready"
                if assignment_policy is None
                else f"planning input ready ({assignment_policy})"
            ),
        )


def save_scene_observation(scene: SceneObservation, save_path: str | Path) -> None:
    """경로계획 프로세스가 읽을 최신 scene JSON을 원자적으로 교체한다."""
    path = Path(save_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("w", encoding="utf-8") as file:
        json.dump(scene.to_dict(), file, indent=2, ensure_ascii=False)
        file.write("\n")
    temporary.replace(path)


def _polygon_center(polygon: np.ndarray) -> Point:
    points = np.asarray(polygon, dtype=np.float64).reshape(-1, 2)
    if len(points) < 3:
        raise ValueError("a detection polygon requires at least three points")
    moments = cv2.moments(points.astype(np.float32))
    if abs(moments["m00"]) > 1e-9:
        return (
            float(moments["m10"] / moments["m00"]),
            float(moments["m01"] / moments["m00"]),
        )
    return float(np.mean(points[:, 0])), float(np.mean(points[:, 1]))


def _principal_axis(polygon: np.ndarray) -> tuple[Point, float]:
    points = np.asarray(polygon, dtype=np.float64).reshape(-1, 2)
    centered = points - np.mean(points, axis=0)
    covariance = centered.T @ centered / max(len(points), 1)
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    major_index = int(np.argmax(eigenvalues))
    minor_index = 1 - major_index
    axis = eigenvectors[:, major_index]
    elongation = math.sqrt(
        max(float(eigenvalues[major_index]), 1e-12)
        / max(float(eigenvalues[minor_index]), 1e-12)
    )
    return (float(axis[0]), float(axis[1])), elongation


def _pose_dict(pose: Pose) -> dict[str, float]:
    return {"x_cm": pose[0], "y_cm": pose[1], "yaw_rad": pose[2]}


def _normalize_yaw(yaw_rad: float) -> float:
    return (yaw_rad + math.pi) % (2.0 * math.pi) - math.pi


def _angle_difference(first: float, second: float) -> float:
    return _normalize_yaw(first - second)
