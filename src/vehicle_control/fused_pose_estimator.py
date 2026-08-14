"""상단 카메라 위치와 LiDAR map heading으로 MPC pose를 발행한다."""

from __future__ import annotations

import math
from pathlib import Path
import time

import numpy as np

try:
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry
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
        self._require_odom = bool(self.get_parameter("require_odom").value)
        self._odom_yaw_sign = float(self.get_parameter("odom_yaw_sign").value)
        if self._odom_yaw_sign not in (-1.0, 1.0):
            raise ValueError("odom_yaw_sign must be -1 or 1")
        self._lidar_only_yaw_alpha = self._positive("lidar_only_yaw_alpha")
        if self._lidar_only_yaw_alpha > 1.0:
            raise ValueError("lidar_only_yaw_alpha must not exceed 1")
        self._rear_axle_offset_m = self._positive("rear_axle_offset_m")
        self._camera_position_alpha = self._positive(
            "camera_position_filter_alpha"
        )
        if self._camera_position_alpha > 1.0:
            raise ValueError("camera_position_filter_alpha must not exceed 1")
        self._camera_yaw_alpha = self._positive("camera_yaw_correction_alpha")
        if self._camera_yaw_alpha > 1.0:
            raise ValueError("camera_yaw_correction_alpha must not exceed 1")
        self._maximum_camera_yaw_error_deg = self._positive(
            "maximum_camera_yaw_error_deg"
        )
        self._camera_timeout_sec = self._positive("camera_pose_timeout_sec")
        self._imu_timeout_sec = self._positive("imu_timeout_sec")
        self._odom_timeout_sec = self._positive("odom_timeout_sec")
        self._lidar_timeout_sec = self._positive("lidar_heading_timeout_sec")
        self._maximum_match_score_m = self._positive("maximum_match_score_m")
        self._minimum_match_margin_m = self._nonnegative(
            "minimum_match_margin_m"
        )
        self._maximum_lidar_correction_deg = self._positive(
            "maximum_lidar_correction_deg"
        )
        self._local_search_half_width_deg = self._positive(
            "local_search_half_width_deg"
        )
        self._local_search_step_deg = self._positive("local_search_step_deg")
        self._initial_search_half_width_deg = self._positive(
            "initial_search_half_width_deg"
        )
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
        trajectory_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
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
        self._odom_subscription = self.create_subscription(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            self._odom_callback,
            sensor_qos,
        )
        self._scan_subscription = self.create_subscription(
            LaserScan,
            str(self.get_parameter("scan_topic").value),
            self._scan_callback,
            sensor_qos,
        )
        self._trajectory_subscription = self.create_subscription(
            Float64MultiArray,
            str(self.get_parameter("trajectory_topic").value),
            self._trajectory_callback,
            trajectory_qos,
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
        self._camera_yaw_rad: float | None = None
        self._imu_yaw_rad: float | None = None
        self._odom_yaw_rad: float | None = None
        self._odom_origin_yaw_rad: float | None = None
        self._map_origin_yaw_rad: float | None = None
        self._lidar_only_yaw_rad: float | None = None
        self._heading_prior_rad: float | None = None
        self._trajectory_signature: tuple[float, ...] | None = None
        self._last_camera_monotonic: float | None = None
        self._last_imu_monotonic: float | None = None
        self._last_odom_monotonic: float | None = None
        self._last_lidar_heading_monotonic: float | None = None
        self._last_match: HeadingMatch | None = None
        self._last_status: str | None = None
        self.get_logger().info(
            (
                "Fused pose: camera x/y + IMU relative yaw + LiDAR map absolute yaw"
                if self._require_imu
                else (
                    "Fused pose: camera x/y + fixed-route yaw + wheel odom; "
                    "camera/LiDAR gated correction (IMU disabled)"
                    if self._require_odom
                    else "Fused pose: camera x/y + LiDAR map heading (IMU disabled)"
                )
            )
        )

    def _declare_parameters(self) -> None:
        defaults = {
            "camera_pose_topic": "camera_pose",
            "imu_topic": "imu_raw",
            "require_imu": False,
            "odom_topic": "odom",
            "require_odom": True,
            # ROS odom +yaw는 반시계, 이미지 map +yaw는 시계 방향이다.
            "odom_yaw_sign": -1.0,
            "scan_topic": "scan",
            "trajectory_topic": "trajectory",
            "fused_pose_topic": "localization_pose",
            "diagnostic_topic": "heading_diagnostics",
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
            # 상단 카메라 mask 중심의 프레임별 흔들림을 줄인다.
            "camera_position_filter_alpha": 0.35,
            "camera_yaw_correction_alpha": 0.10,
            "maximum_camera_yaw_error_deg": 10.0,
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
            "maximum_lidar_correction_deg": 10.0,
            "local_search_half_width_deg": 15.0,
            "local_search_step_deg": 0.5,
            # 차량은 고정 endpoint에서 출발하므로 경로 첫 yaw 주변에서 초기
            # LiDAR 정합을 수행해 직선 벽의 180도 모호성을 제거한다.
            "initial_search_half_width_deg": 15.0,
            "lidar_correction_alpha": 0.15,
            "lidar_only_yaw_alpha": 0.35,
            "heading_reset_position_jump_m": 0.20,
            "camera_pose_timeout_sec": 1.0,
            "imu_timeout_sec": 0.3,
            "odom_timeout_sec": 0.5,
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
        try:
            camera_yaw = quaternion_yaw_xyzw(
                float(message.pose.orientation.x),
                float(message.pose.orientation.y),
                float(message.pose.orientation.z),
                float(message.pose.orientation.w),
            )
        except ValueError:
            self._log_status("CAMERA_ORIENTATION_INVALID")
            return
        position_jump = (
            not self._require_imu
            and self._position_m is not None
            and math.hypot(
                x_m - self._position_m[0],
                y_m - self._position_m[1],
            )
            > self._heading_reset_position_jump_m
        )
        if position_jump:
            # e 키로 다른 ego 차량을 선택하면 이전 차량 주변의 local yaw
            # search를 버리고 새 위치에서 다시 360도 전역 정합한다.
            self._lidar_only_yaw_rad = None
            self._last_lidar_heading_monotonic = None
            self._last_match = None
            self._odom_origin_yaw_rad = None
            self._map_origin_yaw_rad = None
            self._log_status("LIDAR_HEADING_RESET_FOR_NEW_POSITION")
        if self._position_m is None or position_jump:
            self._position_m = (x_m, y_m)
        else:
            alpha = self._camera_position_alpha
            self._position_m = (
                self._position_m[0] + alpha * (x_m - self._position_m[0]),
                self._position_m[1] + alpha * (y_m - self._position_m[1]),
            )
        self._camera_yaw_rad = camera_yaw
        if self._require_odom and self._odom_yaw_rad is not None:
            predicted = self._odom_map_heading()
            if predicted is not None:
                camera_error = (
                    camera_yaw - predicted + math.pi
                ) % (2.0 * math.pi) - math.pi
                if (
                    abs(math.degrees(camera_error))
                    <= self._maximum_camera_yaw_error_deg
                    and self._map_origin_yaw_rad is not None
                ):
                    self._map_origin_yaw_rad = (
                        self._map_origin_yaw_rad
                        + self._camera_yaw_alpha * camera_error
                        + math.pi
                    ) % (2.0 * math.pi) - math.pi
                else:
                    self._log_status(
                        "CAMERA_YAW_REJECTED_"
                        f"error={math.degrees(camera_error):.1f}deg"
                    )
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

    def _odom_callback(self, message: Odometry) -> None:
        try:
            self._odom_yaw_rad = quaternion_yaw_xyzw(
                float(message.pose.pose.orientation.x),
                float(message.pose.pose.orientation.y),
                float(message.pose.pose.orientation.z),
                float(message.pose.pose.orientation.w),
            )
        except ValueError:
            self._log_status("ODOM_ORIENTATION_INVALID")
            return
        self._last_odom_monotonic = time.monotonic()
        if (
            self._heading_prior_rad is not None
            and self._odom_origin_yaw_rad is None
        ):
            self._odom_origin_yaw_rad = self._odom_yaw_rad
            self._map_origin_yaw_rad = self._heading_prior_rad

    def _trajectory_callback(self, message: Float64MultiArray) -> None:
        """고정 경로의 첫 yaw를 LiDAR 전역 정합의 초기 방향 힌트로 사용한다."""
        if len(message.data) < 4 or len(message.data) % 4 != 0:
            self._log_status("TRAJECTORY_HEADING_HINT_INVALID")
            return
        heading = float(message.data[2])
        if not math.isfinite(heading):
            self._log_status("TRAJECTORY_HEADING_HINT_INVALID")
            return
        signature = (
            float(len(message.data)),
            float(message.data[0]),
            float(message.data[1]),
            float(message.data[2]),
            float(message.data[-4]),
            float(message.data[-3]),
            float(message.data[-2]),
        )
        if signature == self._trajectory_signature:
            return
        self._trajectory_signature = signature
        heading = (heading + math.pi) % (2.0 * math.pi) - math.pi
        previous = self._heading_prior_rad
        self._heading_prior_rad = heading
        if self._require_odom:
            self._odom_origin_yaw_rad = self._odom_yaw_rad
            self._map_origin_yaw_rad = heading
        if (
            not self._require_imu
            and previous is not None
            and abs((heading - previous + math.pi) % (2.0 * math.pi) - math.pi)
            > math.radians(45.0)
        ):
            # 다른 차량/mission 경로로 전환했으면 이전 local optimum을 버린다.
            self._lidar_only_yaw_rad = None
            self._last_lidar_heading_monotonic = None
            self._last_match = None
            self._log_status("LIDAR_HEADING_RESET_FOR_NEW_ROUTE")

    def _odom_map_heading(self) -> float | None:
        if (
            self._odom_yaw_rad is None
            or self._odom_origin_yaw_rad is None
            or self._map_origin_yaw_rad is None
        ):
            return None
        odom_delta = (
            self._odom_yaw_rad - self._odom_origin_yaw_rad + math.pi
        ) % (2.0 * math.pi) - math.pi
        return (
            self._map_origin_yaw_rad
            + self._odom_yaw_sign * odom_delta
            + math.pi
        ) % (2.0 * math.pi) - math.pi

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
            else (
                self._odom_map_heading()
                if self._require_odom
                else self._lidar_only_yaw_rad
            )
        )
        if predicted is None:
            if self._heading_prior_rad is None:
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
                    self._heading_prior_rad,
                    self._initial_search_half_width_deg,
                    self._local_search_step_deg,
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
        elif self._require_odom:
            odom_heading = self._odom_map_heading()
            if odom_heading is None or self._map_origin_yaw_rad is None:
                return
            correction = (
                match.yaw_rad - odom_heading + math.pi
            ) % (2.0 * math.pi) - math.pi
            if abs(math.degrees(correction)) > self._maximum_lidar_correction_deg:
                self._last_match = match
                self._log_status(
                    "LIDAR_CORRECTION_REJECTED_"
                    f"error={math.degrees(correction):.1f}deg"
                )
                return
            self._map_origin_yaw_rad = (
                self._map_origin_yaw_rad
                + self._lidar_only_yaw_alpha * correction
                + math.pi
            ) % (2.0 * math.pi) - math.pi
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
        if self._require_odom:
            if self._odom_yaw_rad is None or self._last_odom_monotonic is None:
                self._log_status("WAITING_FOR_ODOM")
                return
            if now - self._last_odom_monotonic > self._odom_timeout_sec:
                self._log_status("ODOM_TIMEOUT")
                return
            if self._odom_map_heading() is None:
                self._log_status("WAITING_FOR_ROUTE_HEADING")
                return
        else:
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
            else (
                self._odom_map_heading()
                if self._require_odom
                else self._lidar_only_yaw_rad
            )
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
            self._camera_yaw_rad
            if self._camera_yaw_rad is not None
            else math.nan,
            self._odom_yaw_rad if self._odom_yaw_rad is not None else math.nan,
        ]
        self._diagnostic_publisher.publish(diagnostic)
        self._log_status(
            "TRACKING_"
            f"yaw={math.degrees(heading):.1f}deg_"
            f"camera={math.degrees(self._camera_yaw_rad) if self._camera_yaw_rad is not None else math.nan:.1f}deg_"
            f"lidar={math.degrees(match.yaw_rad) if match is not None else math.nan:.1f}deg_"
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
