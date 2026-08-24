"""파일 중계 없이 live localization에서 ROS 2 경로·pose 토픽을 발행한다."""

from concurrent.futures import Future, ThreadPoolExecutor
import json
import math
from pathlib import Path
import sys
import time
from typing import Mapping, Sequence

import numpy as np

try:
    from ...vehicle_registry import VEHICLES, get_vehicle
except ImportError:  # direct script compatibility
    sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
    from central_control.vehicle_registry import VEHICLES, get_vehicle

try:
    from .lidar_vehicle_association import (
        LidarVehicleAssociator,
        VehicleTrackAssociation,
    )
except ImportError:  # direct script compatibility
    from lidar_vehicle_association import LidarVehicleAssociator, VehicleTrackAssociation

try:
    from ....vehicle_control.heading_fusion import LidarMapHeadingMatcher
except ImportError:  # direct script compatibility
    from vehicle_control.heading_fusion import LidarMapHeadingMatcher

IMAGE_TOPIC = "/pinkk/localization/image"
LIDAR_IMAGE_TOPIC = "/pinkk/lidar_map/image"
MANAGEMENT_STATUS_TOPIC = "/pinkk/management/status"
CONTROL_REQUEST_TOPIC = "/pinkk/web/control"
PATH_COMMANDS = frozenset(("entry", "exit", "charge", "replan"))
TRAJECTORY_FIELDS = (
    "x_m",
    "y_m",
    "yaw_rad",
    "direction",
)


def parse_path_target_request(raw_data: str) -> tuple[str, str] | None:
    """검증된 관제 경로 요청에서 영속 차량 ID와 명령을 반환한다."""
    try:
        payload = json.loads(raw_data)
    except (TypeError, json.JSONDecodeError):
        return None
    if not isinstance(payload, dict):
        return None
    command = str(payload.get("command", ""))
    if command not in PATH_COMMANDS:
        return None
    try:
        vehicle = get_vehicle(str(payload.get("vehicle_id", "")))
    except ValueError:
        return None
    expected_identity = {
        "robot_id": vehicle.vehicle_id,
        "controller_id": vehicle.controller_id,
        "hardware_serial": vehicle.hardware_serial,
        "ros_namespace": vehicle.ros_namespace,
    }
    if any(
        key in payload and str(payload[key]) != expected
        for key, expected in expected_identity.items()
    ):
        return None
    return vehicle.vehicle_id, command


class DirectRosPublisher:
    """선택 차량의 namespace로 trajectory와 ego pose를 즉시 발행한다."""

    def __init__(
        self,
        vehicle_id: str,
        image_topic: str = IMAGE_TOPIC,
        lidar_image_topic: str = LIDAR_IMAGE_TOPIC,
        management_status_topic: str = MANAGEMENT_STATUS_TOPIC,
        control_request_topic: str = CONTROL_REQUEST_TOPIC,
        lidar_map_path: str | Path | None = None,
        lidar_resolution_cm: float = 1.0,
        lidar_association_period_sec: float = 0.75,
        lidar_scan_timeout_sec: float = 1.5,
        lidar_position_search_half_width_m: float = 0.25,
        lidar_maximum_match_score_m: float = 0.08,
        lidar_minimum_assignment_margin_m: float = 0.01,
        lidar_required_confirmations: int = 2,
        operational_space_polygons: Mapping[
            str, Sequence[Sequence[float]]
        ] | None = None,
    ) -> None:
        initial_vehicle = get_vehicle(vehicle_id)
        try:
            import rclpy
            from geometry_msgs.msg import PoseStamped
            from nav_msgs.msg import Path as RosPath
            from rclpy.node import Node
            from rclpy.qos import (
                DurabilityPolicy,
                QoSProfile,
                ReliabilityPolicy,
                qos_profile_sensor_data,
            )
            from sensor_msgs.msg import Image, LaserScan
            from std_msgs.msg import (
                Bool,
                Float64MultiArray,
                MultiArrayDimension,
                String,
            )
        except ImportError as error:
            raise RuntimeError(
                "ROS 2 rclpy is required for direct topic publishing. "
                "Source /opt/ros/jazzy/setup.bash and install/setup.bash first."
            ) from error

        self._rclpy = rclpy
        self._PoseStamped = PoseStamped
        self._RosPath = RosPath
        self._Float64MultiArray = Float64MultiArray
        self._MultiArrayDimension = MultiArrayDimension
        self._Bool = Bool
        self._Image = Image
        self._String = String
        self._pending_path_request: tuple[str, str] | None = None
        self._owns_context = not rclpy.ok()
        if self._owns_context:
            rclpy.init(args=None)
        self._node = Node("pinkk_live_planning_bridge")
        self._active_vehicle_id = initial_vehicle.vehicle_id
        trajectory_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        pose_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        # ROS 2 publisher의 topic은 생성 후 변경할 수 없다. 차량 선택 시 노드를
        # 재시작하지 않도록 등록된 차량별 publisher를 시작할 때 모두 생성한다.
        self._vehicle_publishers: dict[str, dict[str, object]] = {}
        for registered_vehicle_id in VEHICLES:
            registered_vehicle = get_vehicle(registered_vehicle_id)
            self._vehicle_publishers[registered_vehicle_id] = {
                "path": self._node.create_publisher(
                    RosPath,
                    registered_vehicle.topic("path"),
                    trajectory_qos,
                ),
                "trajectory": self._node.create_publisher(
                    Float64MultiArray,
                    registered_vehicle.topic("trajectory"),
                    trajectory_qos,
                ),
                "path_valid": self._node.create_publisher(
                    Bool,
                    registered_vehicle.topic("path_valid"),
                    trajectory_qos,
                ),
                "pose": self._node.create_publisher(
                    PoseStamped,
                    registered_vehicle.topic("localization_pose"),
                    pose_qos,
                ),
            }
        # web_video_server의 기본 image_transport 구독은 Reliable QoS를
        # 사용하므로 같은 정책으로 발행해야 브라우저 스트림이 연결된다.
        image_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self._image_publisher = self._node.create_publisher(
            Image,
            image_topic,
            image_qos,
        )
        self._lidar_image_publisher = self._node.create_publisher(
            Image,
            lidar_image_topic,
            image_qos,
        )
        self._management_status_publisher = self._node.create_publisher(
            String,
            management_status_topic,
            pose_qos,
        )
        self._control_request_subscription = self._node.create_subscription(
            String,
            control_request_topic,
            self._control_request_callback,
            pose_qos,
        )
        self._lidar_matcher: LidarMapHeadingMatcher | None = None
        self._lidar_associator: LidarVehicleAssociator | None = None
        self._latest_scan_points: dict[str, tuple[float, np.ndarray]] = {}
        self._scan_subscriptions: list[object] = []
        self._last_association_time = -math.inf
        self._last_association: VehicleTrackAssociation | None = None
        self._association_executor: ThreadPoolExecutor | None = None
        self._association_future: Future[VehicleTrackAssociation | None] | None = None
        self._lidar_association_period_sec = float(lidar_association_period_sec)
        self._lidar_scan_timeout_sec = float(lidar_scan_timeout_sec)
        if self._lidar_association_period_sec <= 0.0:
            raise ValueError("lidar_association_period_sec must be positive")
        if self._lidar_scan_timeout_sec <= 0.0:
            raise ValueError("lidar_scan_timeout_sec must be positive")
        if lidar_map_path is not None:
            self._lidar_matcher = LidarMapHeadingMatcher(
                lidar_map_path,
                resolution_m_per_px=float(lidar_resolution_cm) / 100.0,
                lidar_x_m=-0.057,
                lidar_y_m=0.0,
                scan_frame_yaw_deg=180.0,
                scan_subsample=6,
                minimum_points=20,
            )
            self._lidar_associator = LidarVehicleAssociator(
                self._lidar_matcher,
                maximum_match_score_m=float(lidar_maximum_match_score_m),
                minimum_assignment_margin_m=float(
                    lidar_minimum_assignment_margin_m
                ),
                required_confirmations=int(lidar_required_confirmations),
                confirmed_mapping_ttl_sec=max(2.0, self._lidar_scan_timeout_sec),
                position_search_half_width_m=float(
                    lidar_position_search_half_width_m
                ),
            )
            self._association_executor = ThreadPoolExecutor(
                max_workers=1,
                thread_name_prefix="pinkk-lidar-association",
            )
            for registered_vehicle_id in VEHICLES:
                scan_topic = get_vehicle(registered_vehicle_id).topic("scan")
                self._scan_subscriptions.append(
                    self._node.create_subscription(
                        LaserScan,
                        scan_topic,
                        lambda message, vehicle_id=registered_vehicle_id: (
                            self._scan_callback(vehicle_id, message)
                        ),
                        qos_profile_sensor_data,
                    )
                )
        self.image_topic = image_topic
        self.lidar_image_topic = lidar_image_topic
        self.management_status_topic = management_status_topic
        self.control_request_topic = control_request_topic
        self.lidar_resolution_cm = float(lidar_resolution_cm)
        if self.lidar_resolution_cm <= 0.0:
            raise ValueError("lidar_resolution_cm must be positive")
        self.operational_space_polygons = {
            str(name): tuple(
                (float(point[0]), float(point[1])) for point in polygon
            )
            for name, polygon in (operational_space_polygons or {}).items()
        }

    @property
    def vehicle(self) -> object:
        return get_vehicle(self._active_vehicle_id)

    @property
    def active_vehicle_id(self) -> str:
        return self._active_vehicle_id

    @property
    def available_vehicle_ids(self) -> tuple[str, ...]:
        return tuple(self._vehicle_publishers)

    @property
    def path_topic(self) -> str:
        return self.vehicle.topic("path")

    @property
    def trajectory_topic(self) -> str:
        return self.vehicle.topic("trajectory")

    @property
    def pose_topic(self) -> str:
        return self.vehicle.topic("localization_pose")

    @property
    def path_valid_topic(self) -> str:
        return self.vehicle.topic("path_valid")

    def select_vehicle(self, vehicle_id: str) -> bool:
        """이후 경로와 pose를 보낼 차량을 allow-list 안에서 선택한다."""
        selected = get_vehicle(vehicle_id)
        if selected.vehicle_id == self._active_vehicle_id:
            return False
        previous = self._active_vehicle_id
        self._active_vehicle_id = selected.vehicle_id
        self._node.get_logger().info(
            f"Selected path target: {previous} -> {selected.vehicle_id} "
            f"({selected.ros_namespace})"
        )
        return True

    def _active_publishers(self) -> dict[str, object]:
        return self._vehicle_publishers[self._active_vehicle_id]

    def _control_request_callback(self, message: object) -> None:
        request = parse_path_target_request(str(getattr(message, "data", "")))
        if request is None:
            return
        vehicle_id, command = request
        self.select_vehicle(vehicle_id)
        self._pending_path_request = request
        self._node.get_logger().info(
            f"Accepted automatic path target: {vehicle_id} command={command}"
        )

    def consume_path_target_request(self) -> tuple[str, str] | None:
        """ROS callback에서 받은 최신 차량 선택 요청을 한 번만 반환한다."""
        request = self._pending_path_request
        self._pending_path_request = None
        return request

    def _scan_callback(self, vehicle_id: str, message: object) -> None:
        if self._lidar_matcher is None:
            return
        points = self._lidar_matcher.scan_points(
            getattr(message, "ranges"),
            float(getattr(message, "angle_min")),
            float(getattr(message, "angle_increment")),
            float(getattr(message, "range_min")),
            float(getattr(message, "range_max")),
        )
        if len(points) > 0:
            self._latest_scan_points[vehicle_id] = (time.monotonic(), points)

    def update_vehicle_association(
        self,
        tracked_vehicles: Sequence[object],
        now: float | None = None,
    ) -> VehicleTrackAssociation | None:
        """최신 namespaced scan을 camera track과 주기적으로 자동 연결한다."""
        timestamp = time.monotonic() if now is None else float(now)
        if self._lidar_associator is None:
            return None
        if self._association_future is not None and self._association_future.done():
            try:
                self._last_association = self._association_future.result()
            except (RuntimeError, TypeError, ValueError) as error:
                self._node.get_logger().warning(
                    f"LiDAR-camera association failed: {error}"
                )
                self._last_association = None
            self._association_future = None
            if self._last_association is not None:
                summary = ", ".join(
                    f"{vehicle_id}=track_{track_id}"
                    for vehicle_id, track_id in sorted(
                        self._last_association.vehicle_to_track.items()
                    )
                )
                self._node.get_logger().info(
                    "LiDAR-camera vehicle association: "
                    f"{summary}, "
                    f"score={self._last_association.total_score_m:.3f}m, "
                    "margin="
                    f"{self._last_association.assignment_margin_m:.3f}m"
                )
        if self._association_future is not None:
            return self._last_association
        if timestamp - self._last_association_time < self._lidar_association_period_sec:
            return self._last_association
        self._last_association_time = timestamp
        fresh_scans = {
            vehicle_id: points
            for vehicle_id, (received_at, points) in self._latest_scan_points.items()
            if timestamp - received_at <= self._lidar_scan_timeout_sec
        }
        assert self._association_executor is not None
        self._association_future = self._association_executor.submit(
            self._lidar_associator.associate,
            dict(fresh_scans),
            tuple(tracked_vehicles),
            timestamp,
        )
        return self._last_association

    @property
    def active_track_id(self) -> int | None:
        if self._last_association is None:
            return None
        return self._last_association.vehicle_to_track.get(self._active_vehicle_id)

    @property
    def active_lidar_pose(self) -> object | None:
        if self._last_association is None:
            return None
        return self._last_association.lidar_poses.get(self._active_vehicle_id)

    @property
    def association_status(self) -> str:
        if self._lidar_associator is None:
            return "identity=disabled"
        now = time.monotonic()
        missing = [
            vehicle_id
            for vehicle_id in VEHICLES
            if vehicle_id not in self._latest_scan_points
            or now - self._latest_scan_points[vehicle_id][0]
            > self._lidar_scan_timeout_sec
        ]
        if missing:
            return "waiting_scan=" + ",".join(missing)
        if self._association_future is not None:
            return "identity=matching"
        return "identity=unconfirmed"

    def publish_image(self, image: object) -> None:
        """OpenCV BGR 화면을 cv_bridge 없이 sensor_msgs/Image로 발행한다."""
        self._publish_image(self._image_publisher, image, "overhead_camera_bev")

    def publish_lidar_image(self, image: object) -> None:
        """차량 좌표가 표시된 실제 LiDAR 맵을 웹 영상 토픽으로 발행한다."""
        self._publish_image(self._lidar_image_publisher, image, "lidar_map")

    def _publish_image(self, publisher: object, image: object, frame_id: str) -> None:
        if not hasattr(image, "shape") or len(image.shape) != 3:
            raise ValueError("image must be an HxWx3 BGR array")
        height, width, channels = image.shape
        if channels != 3:
            raise ValueError("image must have exactly 3 BGR channels")
        contiguous = image if image.flags.c_contiguous else image.copy(order="C")
        message = self._Image()
        message.header.stamp = self._node.get_clock().now().to_msg()
        message.header.frame_id = frame_id
        message.height = int(height)
        message.width = int(width)
        message.encoding = "bgr8"
        message.is_bigendian = False
        message.step = int(width * channels)
        message.data = contiguous.tobytes()
        publisher.publish(message)

    def publish_pose(
        self,
        vehicle: object,
        measurement_age_sec: float = 0.0,
    ) -> None:
        """카메라 차체 중심 x/y와 측정 장축 yaw를 `lidar_map`으로 발행한다."""
        center_lidar_px = getattr(vehicle, "center_lidar_px")
        age_sec = float(measurement_age_sec)
        if not math.isfinite(age_sec) or age_sec < 0.0:
            age_sec = 0.0
        now_stamp = self._node.get_clock().now().to_msg()
        stamp_ns = max(
            0,
            int(now_stamp.sec) * 1_000_000_000
            + int(now_stamp.nanosec)
            - round(age_sec * 1_000_000_000),
        )
        message = self._PoseStamped()
        message.header.stamp.sec = stamp_ns // 1_000_000_000
        message.header.stamp.nanosec = stamp_ns % 1_000_000_000
        # 차량은 topic namespace로 구분하지만 모든 pose 좌표는 하나의 상단
        # LiDAR map을 공유한다. frame_id까지 차량별로 바꾸면 fused pose가
        # 서로 다른 좌표계로 오해하므로 공통 frame을 유지한다.
        message.header.frame_id = "lidar_map"
        message.pose.position.x = (
            float(center_lidar_px[0]) * self.lidar_resolution_cm / 100.0
        )
        message.pose.position.y = (
            float(center_lidar_px[1]) * self.lidar_resolution_cm / 100.0
        )
        message.pose.position.z = 0.0
        yaw_rad = float(getattr(vehicle, "yaw_rad"))
        message.pose.orientation.z = math.sin(yaw_rad / 2.0)
        message.pose.orientation.w = math.cos(yaw_rad / 2.0)
        self._active_publishers()["pose"].publish(message)

    @staticmethod
    def _estimate_trajectory_seconds(trajectory: Sequence[object]) -> int | None:
        """trajectory 거리와 목표 속도로 남은 실행시간의 근삿값을 계산한다."""
        if len(trajectory) < 2:
            return None
        seconds = 0.0
        for first, second in zip(trajectory, trajectory[1:]):
            distance_m = math.hypot(
                float(getattr(second, "x_cm")) - float(getattr(first, "x_cm")),
                float(getattr(second, "y_cm")) - float(getattr(first, "y_cm")),
            ) / 100.0
            speeds = [
                abs(float(getattr(point, "target_speed_mps", 0.0)))
                for point in (first, second)
                if abs(float(getattr(point, "target_speed_mps", 0.0))) > 1e-3
            ]
            if distance_m > 0.0 and speeds:
                seconds += distance_m / (sum(speeds) / len(speeds))
        return max(1, int(math.ceil(seconds))) if seconds > 0.0 else None

    def publish_management_status(
        self,
        scene: object,
        planning_status: str,
        planning_outcome: object | None,
    ) -> None:
        """관제 웹에서 사용할 주차장·경로 상태를 JSON 토픽으로 발행한다."""
        slots = tuple(getattr(scene, "parking_slots", ()))
        parking_slots = [slot for slot in slots if str(getattr(slot, "name", "")).startswith("P")]
        charging_slots = [slot for slot in slots if str(getattr(slot, "name", "")).startswith("C")]
        occupied_count = sum(bool(getattr(slot, "occupied")) for slot in parking_slots)
        charging_available = sum(
            not bool(getattr(slot, "occupied")) for slot in charging_slots
        )
        planning_request = getattr(scene, "planning_request", None)
        target_slot = (
            str(getattr(planning_request, "slot_name"))
            if planning_request is not None
            else None
        )
        trajectory = (
            tuple(getattr(planning_outcome, "trajectory", ()))
            if planning_outcome is not None
            else ()
        )
        total_count = len(parking_slots)
        tracked_vehicles = tuple(getattr(scene, "tracked_vehicles", ()))
        operational_spaces = {}
        for name, polygon in self.operational_space_polygons.items():
            track_ids = [
                int(getattr(vehicle, "track_id"))
                for vehicle in tracked_vehicles
                if bool(getattr(vehicle, "visible"))
                and self._point_in_polygon(
                    getattr(vehicle, "center_bev_px"),
                    polygon,
                )
            ]
            operational_spaces[name] = {
                "occupied": bool(track_ids),
                "vehicle_track_ids": track_ids,
            }
        payload = {
            "vehicle_id": self.vehicle.vehicle_id,
            "controller_id": self.vehicle.controller_id,
            "hardware_serial": self.vehicle.hardware_serial,
            "ros_namespace": self.vehicle.ros_namespace,
            "online": True,
            "identity_confirmed": True,
            "localization_valid": bool(getattr(scene, "planning_ready", False)),
            "frame_index": int(getattr(scene, "frame_index")),
            "total_parking_slots": total_count,
            "occupied_parking_slots": occupied_count,
            "occupancy_percent": (
                round(occupied_count * 100.0 / total_count, 1)
                if total_count
                else 0.0
            ),
            "charging_available": charging_available,
            "charging_total": len(charging_slots),
            "target_slot": target_slot,
            "route_status": planning_status,
            "estimated_completion_sec": self._estimate_trajectory_seconds(trajectory),
            "scene_status": str(getattr(scene, "status")),
            "spaces": {
                **operational_spaces,
                "parking": {
                    str(getattr(slot, "name")): bool(getattr(slot, "occupied"))
                    for slot in parking_slots
                },
                "charging": {
                    str(getattr(slot, "name")): bool(getattr(slot, "occupied"))
                    for slot in charging_slots
                },
            },
        }
        message = self._String()
        message.data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        self._management_status_publisher.publish(message)

    @staticmethod
    def _point_in_polygon(
        point: Sequence[float],
        polygon: Sequence[Sequence[float]],
    ) -> bool:
        """경계 포함 ray-casting으로 BEV 점의 polygon 포함 여부를 판정한다."""
        if len(polygon) < 3:
            return False
        x, y = float(point[0]), float(point[1])
        inside = False
        previous_x, previous_y = polygon[-1]
        for current_x, current_y in polygon:
            cross = (
                (x - previous_x) * (current_y - previous_y)
                - (y - previous_y) * (current_x - previous_x)
            )
            if (
                abs(cross) <= 1e-9
                and min(previous_x, current_x) <= x <= max(previous_x, current_x)
                and min(previous_y, current_y) <= y <= max(previous_y, current_y)
            ):
                return True
            if (current_y > y) != (previous_y > y):
                intersection_x = (
                    (previous_x - current_x)
                    * (y - current_y)
                    / (previous_y - current_y)
                    + current_x
                )
                if x < intersection_x:
                    inside = not inside
            previous_x, previous_y = current_x, current_y
        return inside

    def publish_trajectory(self, trajectory: Sequence[object]) -> None:
        """고정 경로를 표준 path와 x/y/yaw/direction 행렬로 발행한다."""
        if not trajectory:
            raise ValueError("trajectory must not be empty")
        stamp = self._node.get_clock().now().to_msg()
        path_message = self._RosPath()
        path_message.header.stamp = stamp
        path_message.header.frame_id = "lidar_map"
        matrix: list[float] = []
        for point in trajectory:
            pose = self._PoseStamped()
            pose.header = path_message.header
            yaw_rad = float(getattr(point, "yaw_rad"))
            pose.pose.position.x = float(getattr(point, "x_cm")) / 100.0
            pose.pose.position.y = float(getattr(point, "y_cm")) / 100.0
            pose.pose.position.z = 0.0
            pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
            pose.pose.orientation.w = math.cos(yaw_rad / 2.0)
            path_message.poses.append(pose)
            matrix.extend(
                (
                    pose.pose.position.x,
                    pose.pose.position.y,
                    yaw_rad,
                    float(getattr(point, "direction")),
                )
            )
        trajectory_message = self._Float64MultiArray()
        point_dim = self._MultiArrayDimension()
        point_dim.label = "point"
        point_dim.size = len(trajectory)
        point_dim.stride = len(trajectory) * len(TRAJECTORY_FIELDS)
        field_dim = self._MultiArrayDimension()
        field_dim.label = "fields=" + ",".join(TRAJECTORY_FIELDS)
        field_dim.size = len(TRAJECTORY_FIELDS)
        field_dim.stride = len(TRAJECTORY_FIELDS)
        trajectory_message.layout.dim = [point_dim, field_dim]
        trajectory_message.data = matrix
        validity = self._Bool()
        validity.data = True
        publishers = self._active_publishers()
        publishers["path_valid"].publish(validity)
        publishers["path"].publish(path_message)
        publishers["trajectory"].publish(trajectory_message)
        self._node.get_logger().info(
            f"Published {len(trajectory)} points: "
            f"{self.path_topic}, {self.trajectory_topic}"
        )

    def invalidate_trajectory(self) -> None:
        """Ego 전환 중 이전 차량 경로를 제어기가 즉시 폐기하게 알린다."""
        validity = self._Bool()
        validity.data = False
        self._active_publishers()["path_valid"].publish(validity)
        self._node.get_logger().warning(
            f"Invalidated active trajectory: {self._active_vehicle_id}"
        )

    def spin_once(self) -> None:
        self._rclpy.spin_once(self._node, timeout_sec=0.0)

    def close(self) -> None:
        if self._association_executor is not None:
            self._association_executor.shutdown(wait=False, cancel_futures=True)
        self._node.destroy_node()
        if self._owns_context and self._rclpy.ok():
            self._rclpy.shutdown()
