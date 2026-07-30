"""파일 중계 없이 live localization에서 ROS 2 경로·pose 토픽을 발행한다."""

import math
from typing import Sequence


PATH_TOPIC = "/pinkk/planned_path"
TRAJECTORY_TOPIC = "/pinkk/planned_trajectory"
POSE_TOPIC = "/pinkk/vehicle_pose"
PATH_VALID_TOPIC = "/pinkk/path_valid"
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
        lidar_resolution_cm: float = 1.0,
    ) -> None:
        try:
            import rclpy
            from geometry_msgs.msg import PoseStamped
            from nav_msgs.msg import Path as RosPath
            from rclpy.node import Node
            from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
            from std_msgs.msg import Bool, Float64MultiArray, MultiArrayDimension
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
        self.path_topic = path_topic
        self.trajectory_topic = trajectory_topic
        self.pose_topic = pose_topic
        self.path_valid_topic = path_valid_topic
        self.lidar_resolution_cm = float(lidar_resolution_cm)
        if self.lidar_resolution_cm <= 0.0:
            raise ValueError("lidar_resolution_cm must be positive")

    def publish_pose(self, vehicle: object) -> None:
        """Heading과 무관한 카메라 차체 중심 x/y를 `lidar_map`으로 발행한다."""
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
        # 이 토픽의 orientation은 사용하지 않는다. 실제 heading은 IMU·LiDAR
        # 융합 노드가 계산한다.
        message.pose.orientation.w = 1.0
        self._pose_publisher.publish(message)

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
