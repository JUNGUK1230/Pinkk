"""파일 중계 없이 live localization에서 ROS 2 경로·pose 토픽을 발행한다."""

import json
import math
from typing import Mapping, Sequence


PATH_TOPIC = "/pinkk/planned_path"
TRAJECTORY_TOPIC = "/pinkk/planned_trajectory"
POSE_TOPIC = "/pinkk/vehicle_pose"
PATH_VALID_TOPIC = "/pinkk/path_valid"
IMAGE_TOPIC = "/pinkk/localization/image"
LIDAR_IMAGE_TOPIC = "/pinkk/lidar_map/image"
MANAGEMENT_STATUS_TOPIC = "/pinkk/management/status"
TRAJECTORY_FIELDS = (
    "x_m",
    "y_m",
    "yaw_rad",
    "direction",
)


class DirectRosPublisher:
    """검증된 메모리 trajectory와 최신 ego pose를 즉시 ROS 2로 발행한다."""

    def __init__(
        self,
        path_topic: str = PATH_TOPIC,
        trajectory_topic: str = TRAJECTORY_TOPIC,
        pose_topic: str = POSE_TOPIC,
        path_valid_topic: str = PATH_VALID_TOPIC,
        image_topic: str = IMAGE_TOPIC,
        lidar_image_topic: str = LIDAR_IMAGE_TOPIC,
        management_status_topic: str = MANAGEMENT_STATUS_TOPIC,
        lidar_resolution_cm: float = 1.0,
        operational_space_polygons: Mapping[
            str, Sequence[Sequence[float]]
        ] | None = None,
    ) -> None:
        try:
            import rclpy
            from geometry_msgs.msg import PoseStamped
            from nav_msgs.msg import Path as RosPath
            from rclpy.node import Node
            from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
            from sensor_msgs.msg import Image
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
        self._owns_context = not rclpy.ok()
        if self._owns_context:
            rclpy.init(args=None)
        self._node = Node("pinkk_live_planning_bridge")
        trajectory_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        pose_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._path_publisher = self._node.create_publisher(
            RosPath,
            path_topic,
            trajectory_qos,
        )
        self._trajectory_publisher = self._node.create_publisher(
            Float64MultiArray,
            trajectory_topic,
            trajectory_qos,
        )
        self._path_valid_publisher = self._node.create_publisher(
            Bool,
            path_valid_topic,
            trajectory_qos,
        )
        self._pose_publisher = self._node.create_publisher(
            PoseStamped,
            pose_topic,
            pose_qos,
        )
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
        self.path_topic = path_topic
        self.trajectory_topic = trajectory_topic
        self.pose_topic = pose_topic
        self.path_valid_topic = path_valid_topic
        self.image_topic = image_topic
        self.lidar_image_topic = lidar_image_topic
        self.management_status_topic = management_status_topic
        self.lidar_resolution_cm = float(lidar_resolution_cm)
        if self.lidar_resolution_cm <= 0.0:
            raise ValueError("lidar_resolution_cm must be positive")
        self.operational_space_polygons = {
            str(name): tuple(
                (float(point[0]), float(point[1])) for point in polygon
            )
            for name, polygon in (operational_space_polygons or {}).items()
        }

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

    def publish_pose(self, vehicle: object) -> None:
        """카메라 차체 중심 x/y와 측정 장축 yaw를 `lidar_map`으로 발행한다."""
        center_lidar_px = getattr(vehicle, "center_lidar_px")
        stamp = self._node.get_clock().now().to_msg()
        message = self._PoseStamped()
        message.header.stamp = stamp
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
        self._pose_publisher.publish(message)

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
        self._path_valid_publisher.publish(validity)
        self._path_publisher.publish(path_message)
        self._trajectory_publisher.publish(trajectory_message)
        self._node.get_logger().info(
            f"Published {len(trajectory)} points: "
            f"{self.path_topic}, {self.trajectory_topic}"
        )

    def invalidate_trajectory(self) -> None:
        """Ego 전환 중 이전 차량 경로를 제어기가 즉시 폐기하게 알린다."""
        validity = self._Bool()
        validity.data = False
        self._path_valid_publisher.publish(validity)
        self._node.get_logger().warning("Invalidated active trajectory")

    def spin_once(self) -> None:
        self._rclpy.spin_once(self._node, timeout_sec=0.0)

    def close(self) -> None:
        self._node.destroy_node()
        if self._owns_context and self._rclpy.ok():
            self._rclpy.shutdown()
