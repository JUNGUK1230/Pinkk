"""상단 카메라 위치와 LiDAR map heading으로 MPC pose를 발행한다."""

from __future__ import annotations

import math
from pathlib import Path
import time

import numpy as np

try:
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import (
        DurabilityPolicy,
        HistoryPolicy,
        QoSProfile,
        ReliabilityPolicy,
    )
    from sensor_msgs.msg import Imu, LaserScan
    from std_msgs.msg import Float64MultiArray
except ImportError as error:  # pragma: no cover - ROS 환경 안내용
    raise RuntimeError("ROS 2 환경을 source한 뒤 실행해야 합니다") from error

from .heading_fusion import (
    HeadingMatch,
    ImuLidarHeadingFusion,
    LidarMapHeadingMatcher,
    quaternion_yaw_xyzw,
)


class FusedPoseEstimator(Node):
    def __init__(self) -> None:
        super().__init__("pinkk_fused_pose_estimator")
        self._declare_parameters()
        map_path = Path(str(self.get_parameter("map_image_path").value))
        self._matcher = LidarMapHeadingMatcher(
            map_image_path=map_path,
            resolution_m_per_px=self._positive("map_resolution_m_per_px"),
            occupied_pixel_threshold=int(
                self.get_parameter("occupied_pixel_threshold").value
            ),
            lidar_x_m=float(self.get_parameter("lidar_x_m").value),
            lidar_y_m=float(self.get_parameter("lidar_y_m").value),
            scan_frame_yaw_deg=float(
                self.get_parameter("scan_frame_yaw_deg").value
            ),
            minimum_range_m=self._positive("minimum_scan_range_m"),
            maximum_range_m=self._positive("maximum_scan_range_m"),
            scan_subsample=int(self.get_parameter("scan_subsample").value),
            trimmed_fraction=self._positive("trimmed_fraction"),
            outside_penalty_m=self._positive("outside_penalty_m"),
            minimum_points=int(self.get_parameter("minimum_scan_points").value),
        )
        self._fusion = ImuLidarHeadingFusion(
            lidar_correction_alpha=self._positive("lidar_correction_alpha"),
            imu_yaw_sign=float(self.get_parameter("imu_yaw_sign").value),
        )
        self._require_imu = bool(self.get_parameter("require_imu").value)
        self._lidar_only_yaw_alpha = self._positive("lidar_only_yaw_alpha")
        if self._lidar_only_yaw_alpha > 1.0:
            raise ValueError("lidar_only_yaw_alpha must not exceed 1")
        self._rear_axle_offset_m = self._positive("rear_axle_offset_m")
        self._camera_timeout_sec = self._positive("camera_pose_timeout_sec")
        self._imu_timeout_sec = self._positive("imu_timeout_sec")
        self._lidar_timeout_sec = self._positive("lidar_heading_timeout_sec")
        self._maximum_match_score_m = self._positive("maximum_match_score_m")
        self._minimum_match_margin_m = self._nonnegative(
            "minimum_match_margin_m"
        )
        self._local_search_half_width_deg = self._positive(
            "local_search_half_width_deg"
        )
        self._local_search_step_deg = self._positive("local_search_step_deg")
        self._heading_reset_position_jump_m = self._positive(
            "heading_reset_position_jump_m"
        )

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        pose_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._camera_subscription = self.create_subscription(
            PoseStamped,
            str(self.get_parameter("camera_pose_topic").value),
            self._camera_callback,
            pose_qos,
        )
        self._imu_subscription = self.create_subscription(
            Imu,
            str(self.get_parameter("imu_topic").value),
            self._imu_callback,
            sensor_qos,
        )
        self._scan_subscription = self.create_subscription(
            LaserScan,
            str(self.get_parameter("scan_topic").value),
            self._scan_callback,
            sensor_qos,
        )
        self._pose_publisher = self.create_publisher(
            PoseStamped,
            str(self.get_parameter("fused_pose_topic").value),
            pose_qos,
        )
        self._diagnostic_publisher = self.create_publisher(
            Float64MultiArray,
            str(self.get_parameter("diagnostic_topic").value),
            10,
        )
        publish_rate = self._positive("publish_rate_hz")
        self._timer = self.create_timer(1.0 / publish_rate, self._publish_tick)

        self._position_m: tuple[float, float] | None = None
        self._imu_yaw_rad: float | None = None
        self._lidar_only_yaw_rad: float | None = None
        self._last_camera_monotonic: float | None = None
        self._last_imu_monotonic: float | None = None
        self._last_lidar_heading_monotonic: float | None = None
        self._last_match: HeadingMatch | None = None
        self._last_status: str | None = None
        self.get_logger().info(
            (
                "Fused pose: camera x/y + IMU relative yaw + LiDAR map absolute yaw"
                if self._require_imu
                else "Fused pose: camera x/y + LiDAR map heading (IMU disabled)"
            )
        )

    def _declare_parameters(self) -> None:
        defaults = {
            "camera_pose_topic": "/pinkk/vehicle_pose",
            "imu_topic": "/imu_raw",
            "require_imu": False,
            "scan_topic": "/scan",
            "fused_pose_topic": "/pinkk/fused_vehicle_pose",
            "diagnostic_topic": "/pinkk/heading_diagnostics",
            "map_image_path": (
                "src/central_control/camera_tools/first_map/"
                "my_test_map0710.png"
            ),
            "map_resolution_m_per_px": 0.01,
            "occupied_pixel_threshold": 100,
            # 카메라 차체 중심 기준: rear axle=-4cm, LiDAR=-5.7cm.
            "lidar_x_m": -0.057,
            "lidar_y_m": 0.0,
            "rear_axle_offset_m": 0.04,
            "scan_frame_yaw_deg": 180.0,
            # map y축은 이미지 아래 방향이므로 ROS IMU yaw 부호를 반전한다.
            "imu_yaw_sign": -1.0,
            "minimum_scan_range_m": 0.08,
            "maximum_scan_range_m": 2.5,
            "scan_subsample": 3,
            "minimum_scan_points": 25,
            "trimmed_fraction": 0.70,
            "outside_penalty_m": 0.30,
            "maximum_match_score_m": 0.08,
            "minimum_match_margin_m": 0.0005,
            "local_search_half_width_deg": 15.0,
            "local_search_step_deg": 0.5,
            "lidar_correction_alpha": 0.15,
            "lidar_only_yaw_alpha": 0.35,
            "heading_reset_position_jump_m": 0.10,
            "camera_pose_timeout_sec": 0.6,
            "imu_timeout_sec": 0.3,
            "lidar_heading_timeout_sec": 1.5,
            "publish_rate_hz": 15.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _positive(self, name: str) -> float:
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"{name} must be positive and finite")
        return value

    def _nonnegative(self, name: str) -> float:
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value < 0.0:
            raise ValueError(f"{name} must be finite and non-negative")
        return value

    def _camera_callback(self, message: PoseStamped) -> None:
        if message.header.frame_id and message.header.frame_id != "lidar_map":
            self._log_status(f"CAMERA_FRAME_REJECTED_{message.header.frame_id}")
            return
        x_m = float(message.pose.position.x)
        y_m = float(message.pose.position.y)
        if not math.isfinite(x_m) or not math.isfinite(y_m):
            self._log_status("CAMERA_POSITION_INVALID")
            return
        if (
            not self._require_imu
            and self._position_m is not None
            and math.hypot(
                x_m - self._position_m[0],
                y_m - self._position_m[1],
            )
            > self._heading_reset_position_jump_m
        ):
            # e 키로 다른 ego 차량을 선택하면 이전 차량 주변의 local yaw
            # search를 버리고 새 위치에서 다시 360도 전역 정합한다.
            self._lidar_only_yaw_rad = None
            self._last_lidar_heading_monotonic = None
            self._last_match = None
            self._log_status("LIDAR_HEADING_RESET_FOR_NEW_POSITION")
        self._position_m = (x_m, y_m)
        self._last_camera_monotonic = time.monotonic()

    def _imu_callback(self, message: Imu) -> None:
        try:
            self._imu_yaw_rad = quaternion_yaw_xyzw(
                float(message.orientation.x),
                float(message.orientation.y),
                float(message.orientation.z),
                float(message.orientation.w),
            )
        except ValueError:
            self._log_status("IMU_ORIENTATION_INVALID")
            return
        self._last_imu_monotonic = time.monotonic()

    def _scan_callback(self, message: LaserScan) -> None:
        if self._position_m is None:
            return
        if self._require_imu and self._imu_yaw_rad is None:
            return
        points = self._matcher.scan_points(
            message.ranges,
            float(message.angle_min),
            float(message.angle_increment),
            float(message.range_min),
            float(message.range_max),
        )
        if len(points) == 0:
            self._log_status("LIDAR_POINTS_INVALID")
            return
        predicted = (
            self._fusion.heading(self._imu_yaw_rad)
            if self._require_imu and self._imu_yaw_rad is not None
            else self._lidar_only_yaw_rad
        )
        if predicted is None:
            match = self._matcher.match_global(
                self._position_m[0],
                self._position_m[1],
                points,
            )
        else:
            match = self._matcher.match_local(
                self._position_m[0],
                self._position_m[1],
                points,
                predicted,
                self._local_search_half_width_deg,
                self._local_search_step_deg,
            )
        if match is None:
            self._log_status("LIDAR_MATCH_UNAVAILABLE")
            return
        if (
            match.score_m > self._maximum_match_score_m
            or match.distinct_margin_m < self._minimum_match_margin_m
        ):
            self._last_match = match
            self._log_status(
                "LIDAR_MATCH_REJECTED_"
                f"score={match.score_m:.3f}_margin={match.distinct_margin_m:.4f}"
            )
            return
        if self._require_imu:
            assert self._imu_yaw_rad is not None
            self._fusion.correct(self._imu_yaw_rad, match.yaw_rad)
        elif self._lidar_only_yaw_rad is None:
            self._lidar_only_yaw_rad = match.yaw_rad
        else:
            error = (
                match.yaw_rad - self._lidar_only_yaw_rad + math.pi
            ) % (2.0 * math.pi) - math.pi
            self._lidar_only_yaw_rad = (
                self._lidar_only_yaw_rad
                + self._lidar_only_yaw_alpha * error
                + math.pi
            ) % (2.0 * math.pi) - math.pi
        self._last_match = match
        self._last_lidar_heading_monotonic = time.monotonic()

    def _publish_tick(self) -> None:
        now = time.monotonic()
        if self._position_m is None or self._last_camera_monotonic is None:
            self._log_status("WAITING_FOR_CAMERA_POSITION")
            return
        if now - self._last_camera_monotonic > self._camera_timeout_sec:
            self._log_status("CAMERA_POSITION_TIMEOUT")
            return
        if self._require_imu:
            if self._imu_yaw_rad is None or self._last_imu_monotonic is None:
                self._log_status("WAITING_FOR_IMU")
                return
            if now - self._last_imu_monotonic > self._imu_timeout_sec:
                self._log_status("IMU_TIMEOUT")
                return
        if (
            (
                not self._fusion.initialized
                if self._require_imu
                else self._lidar_only_yaw_rad is None
            )
            or self._last_lidar_heading_monotonic is None
        ):
            self._log_status("WAITING_FOR_LIDAR_HEADING")
            return
        if now - self._last_lidar_heading_monotonic > self._lidar_timeout_sec:
            self._log_status("LIDAR_HEADING_TIMEOUT")
            return
        heading = (
            self._fusion.heading(self._imu_yaw_rad)
            if self._require_imu and self._imu_yaw_rad is not None
            else self._lidar_only_yaw_rad
        )
        if heading is None:
            self._log_status("HEADING_UNINITIALIZED")
            return

        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "lidar_map"
        # 카메라 중심에서 fused heading의 뒤쪽으로 rear axle을 계산한다.
        rear_x = self._position_m[0] - self._rear_axle_offset_m * math.cos(heading)
        rear_y = self._position_m[1] - self._rear_axle_offset_m * math.sin(heading)
        message.pose.position.x = rear_x
        message.pose.position.y = rear_y
        message.pose.orientation.z = math.sin(heading / 2.0)
        message.pose.orientation.w = math.cos(heading / 2.0)
        self._pose_publisher.publish(message)

        diagnostic = Float64MultiArray()
        match = self._last_match
        diagnostic.data = [
            rear_x,
            rear_y,
            self._imu_yaw_rad if self._imu_yaw_rad is not None else math.nan,
            heading,
            match.yaw_rad if match is not None else math.nan,
            match.score_m if match is not None else math.nan,
            match.distinct_margin_m if match is not None else math.nan,
        ]
        self._diagnostic_publisher.publish(diagnostic)
        self._log_status(
            "TRACKING_"
            f"yaw={math.degrees(heading):.1f}deg_"
            f"score={match.score_m if match is not None else math.nan:.3f}m"
        )

    def _log_status(self, status: str) -> None:
        # TRACKING에는 값이 들어가므로 매 프레임 로그가 쌓이지 않게 상태명만
        # 비교한다.
        category = status.split("_yaw=", 1)[0]
        if category == self._last_status:
            return
        self._last_status = category
        self.get_logger().info(f"Fused pose status: {status}")


def main(args: list[str] | None = None) -> int:
    rclpy.init(args=args)
    node: FusedPoseEstimator | None = None
    exit_code = 0
    try:
        node = FusedPoseEstimator()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except (FileNotFoundError, ValueError) as error:
        exit_code = 1
        if node is not None:
            node.get_logger().error(f"Fused pose configuration error: {error}")
        else:
            print(f"Fused pose configuration error: {error}")
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
