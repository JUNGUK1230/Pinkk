"""고정 trajectory를 추종해 실차 cmd_vel을 내보내는 안전 정지형 MPC 노드."""

from __future__ import annotations

import hashlib
import math
from pathlib import Path
import struct
import sys
import time
from typing import Sequence

try:
    import rclpy
    from geometry_msgs.msg import PoseStamped, Twist
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import Bool, Float64MultiArray
except ImportError as error:  # pragma: no cover - ROS 환경 오류 메시지용
    raise RuntimeError("ROS 2 Jazzy 환경을 source한 뒤 실행해야 합니다") from error

try:
    from .mpc_controller import (
        DifferentialDriveMpc,
        MpcLimits,
        MpcWeights,
        ReferencePoint,
        VehicleState,
    )
except ImportError:
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from mpc_controller import (  # type: ignore
        DifferentialDriveMpc,
        MpcLimits,
        MpcWeights,
        ReferencePoint,
        VehicleState,
    )


EXPECTED_FIELDS = ("x_m", "y_m", "yaw_rad", "direction")


def trajectory_signature(data: Sequence[float]) -> bytes:
    """동일 경로 주기 발행이 MPC progress를 초기화하지 않게 식별한다."""
    digest = hashlib.blake2b(digest_size=16)
    for value in data:
        digest.update(struct.pack("<d", float(value)))
    return digest.digest()


def quaternion_yaw(message: PoseStamped) -> float:
    quaternion = message.pose.orientation
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y**2 + quaternion.z**2),
    )


def scan_sector_minima(
    ranges: Sequence[float],
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    front_half_angle_rad: float,
    rear_half_angle_rad: float,
    front_center_angle_rad: float = 0.0,
    rear_center_angle_rad: float = math.pi,
) -> tuple[float | None, float | None]:
    """LaserScan에서 차량 전방·후방 sector의 최소 유효 거리를 계산한다."""
    front: list[float] = []
    rear: list[float] = []
    for index, raw_distance in enumerate(ranges):
        distance = float(raw_distance)
        if (
            not math.isfinite(distance)
            or distance < range_min
            or distance > range_max
        ):
            continue
        angle = angle_min + index * angle_increment
        front_error = abs(
            (angle - front_center_angle_rad + math.pi)
            % (2.0 * math.pi)
            - math.pi
        )
        rear_error = abs(
            (angle - rear_center_angle_rad + math.pi)
            % (2.0 * math.pi)
            - math.pi
        )
        if front_error <= front_half_angle_rad:
            front.append(distance)
        if rear_error <= rear_half_angle_rad:
            rear.append(distance)
    return (
        min(front) if front else None,
        min(rear) if rear else None,
    )


class MpcPathFollower(Node):
    def __init__(self) -> None:
        super().__init__("pinkk_mpc_path_follower")
        self._declare_parameters()
        self._controller = DifferentialDriveMpc(
            limits=self._load_limits(),
            weights=self._load_weights(),
        )
        self._path_timeout_sec = self._positive_parameter("path_timeout_sec")
        self._gear_pause_sec = self._positive_parameter("gear_pause_sec")
        self._scan_timeout_sec = self._positive_parameter("scan_timeout_sec")
        self._front_stop_distance_m = self._positive_parameter(
            "front_stop_distance_m"
        )
        self._rear_stop_distance_m = self._positive_parameter(
            "rear_stop_distance_m"
        )
        self._front_half_angle_rad = math.radians(
            self._positive_parameter("front_scan_half_angle_deg")
        )
        self._rear_half_angle_rad = math.radians(
            self._positive_parameter("rear_scan_half_angle_deg")
        )
        self._front_center_angle_rad = math.radians(
            float(self.get_parameter("front_scan_center_deg").value)
        )
        self._rear_center_angle_rad = math.radians(
            float(self.get_parameter("rear_scan_center_deg").value)
        )
        self._require_scan = bool(self.get_parameter("require_scan").value)
        self._reject_cmd_vel_conflicts = bool(
            self.get_parameter("reject_cmd_vel_conflicts").value
        )
        self._cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self._angular_command_sign = float(
            self.get_parameter("angular_command_sign").value
        )
        if self._angular_command_sign not in (-1.0, 1.0):
            raise ValueError("angular_command_sign must be -1 or 1")
        control_frequency = self._positive_parameter("control_frequency_hz")

        trajectory_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        pose_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._trajectory_subscription = self.create_subscription(
            Float64MultiArray,
            str(self.get_parameter("trajectory_topic").value),
            self._trajectory_callback,
            trajectory_qos,
        )
        self._path_valid_subscription = self.create_subscription(
            Bool,
            str(self.get_parameter("path_valid_topic").value),
            self._path_valid_callback,
            trajectory_qos,
        )
        self._pose_subscription = self.create_subscription(
            PoseStamped,
            str(self.get_parameter("pose_topic").value),
            self._pose_callback,
            pose_qos,
        )
        self._scan_subscription = self.create_subscription(
            LaserScan,
            str(self.get_parameter("scan_topic").value),
            self._scan_callback,
            10,
        )
        self._command_publisher = self.create_publisher(
            Twist,
            self._cmd_vel_topic,
            10,
        )
        self._timer = self.create_timer(1.0 / control_frequency, self._control_tick)

        self._state: VehicleState | None = None
        self._last_pose_monotonic: float | None = None
        self._last_path_monotonic: float | None = None
        self._last_scan_monotonic: float | None = None
        self._front_obstacle_m: float | None = None
        self._rear_obstacle_m: float | None = None
        self._path_signature: bytes | None = None
        self._path_valid = True
        self._gear_resume_monotonic: float | None = None
        self._last_status: str | None = None
        self._last_solve_log_monotonic = 0.0
        self.get_logger().warning(
            "MPC motion output enabled on %s; do not run another velocity controller"
            % self._cmd_vel_topic
        )

    def _declare_parameters(self) -> None:
        defaults = {
            "trajectory_topic": "trajectory",
            "path_valid_topic": "path_valid",
            "pose_topic": "localization_pose",
            "cmd_vel_topic": "cmd_vel",
            # Pinky 실차 구동계에서 map curvature와 동일한 부호를 사용한다.
            "angular_command_sign": 1.0,
            "scan_topic": "scan",
            "require_scan": True,
            "scan_timeout_sec": 0.8,
            "front_scan_half_angle_deg": 18.0,
            "rear_scan_half_angle_deg": 30.0,
            "front_scan_center_deg": 180.0,
            "rear_scan_center_deg": 0.0,
            "front_stop_distance_m": 0.15,
            "rear_stop_distance_m": 0.12,
            "reject_cmd_vel_conflicts": True,
            "control_frequency_hz": 4.0,
            "path_timeout_sec": 2.5,
            "gear_pause_sec": 0.7,
            "dt_sec": 0.25,
            "horizon_steps": 10,
            "forward_speed_mps": 0.06,
            "reverse_speed_mps": 0.02,
            "max_forward_speed_mps": 0.08,
            "max_reverse_speed_mps": 0.03,
            "max_acceleration_mps2": 0.12,
            "max_curvature_1pm": 7.0,
            "max_curvature_rate_1pmps": 10.0,
            "max_angular_speed_radps": 0.35,
            "straight_curvature_threshold_1pm": 0.35,
            "straight_max_curvature_1pm": 3.0,
            "max_tracking_yaw_error_deg": 25.0,
            "pose_timeout_sec": 0.6,
            "goal_position_tolerance_m": 0.025,
            "goal_yaw_tolerance_deg": 8.0,
            "gear_position_tolerance_m": 0.01,
            "nearest_forward_window": 140,
            "nearest_backward_window": 4,
            "solver_max_iterations": 45,
            "solver_ftol": 1e-5,
            "weight_position": 200.0,
            "weight_yaw": 2.0,
            "weight_terminal_position": 500.0,
            "weight_terminal_yaw": 5.0,
            "weight_speed": 50.0,
            "weight_curvature": 0.08,
            "weight_speed_rate": 1.0,
            "weight_curvature_rate": 0.04,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _load_limits(self) -> MpcLimits:
        return MpcLimits(
            dt_sec=self._positive_parameter("dt_sec"),
            horizon_steps=int(self.get_parameter("horizon_steps").value),
            forward_speed_mps=self._positive_parameter("forward_speed_mps"),
            reverse_speed_mps=self._positive_parameter("reverse_speed_mps"),
            max_forward_speed_mps=self._positive_parameter(
                "max_forward_speed_mps"
            ),
            max_reverse_speed_mps=self._positive_parameter(
                "max_reverse_speed_mps"
            ),
            max_acceleration_mps2=self._positive_parameter(
                "max_acceleration_mps2"
            ),
            max_curvature_1pm=self._positive_parameter("max_curvature_1pm"),
            max_curvature_rate_1pmps=self._positive_parameter(
                "max_curvature_rate_1pmps"
            ),
            max_angular_speed_radps=self._positive_parameter(
                "max_angular_speed_radps"
            ),
            straight_curvature_threshold_1pm=self._positive_parameter(
                "straight_curvature_threshold_1pm"
            ),
            straight_max_curvature_1pm=self._positive_parameter(
                "straight_max_curvature_1pm"
            ),
            max_tracking_yaw_error_rad=math.radians(
                self._positive_parameter("max_tracking_yaw_error_deg")
            ),
            pose_timeout_sec=self._positive_parameter("pose_timeout_sec"),
            goal_position_tolerance_m=self._positive_parameter(
                "goal_position_tolerance_m"
            ),
            goal_yaw_tolerance_rad=math.radians(
                self._positive_parameter("goal_yaw_tolerance_deg")
            ),
            gear_position_tolerance_m=self._positive_parameter(
                "gear_position_tolerance_m"
            ),
            nearest_forward_window=int(
                self.get_parameter("nearest_forward_window").value
            ),
            nearest_backward_window=int(
                self.get_parameter("nearest_backward_window").value
            ),
            solver_max_iterations=int(
                self.get_parameter("solver_max_iterations").value
            ),
            solver_ftol=self._positive_parameter("solver_ftol"),
        )

    def _load_weights(self) -> MpcWeights:
        return MpcWeights(
            position=self._nonnegative_parameter("weight_position"),
            yaw=self._nonnegative_parameter("weight_yaw"),
            terminal_position=self._nonnegative_parameter(
                "weight_terminal_position"
            ),
            terminal_yaw=self._nonnegative_parameter("weight_terminal_yaw"),
            speed=self._nonnegative_parameter("weight_speed"),
            curvature=self._nonnegative_parameter("weight_curvature"),
            speed_rate=self._nonnegative_parameter("weight_speed_rate"),
            curvature_rate=self._nonnegative_parameter("weight_curvature_rate"),
        )

    def _positive_parameter(self, name: str) -> float:
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"{name} must be positive and finite")
        return value

    def _nonnegative_parameter(self, name: str) -> float:
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value < 0.0:
            raise ValueError(f"{name} must be finite and non-negative")
        return value

    def _trajectory_callback(self, message: Float64MultiArray) -> None:
        now = time.monotonic()
        try:
            points = self._decode_trajectory(message)
            signature = trajectory_signature(message.data)
        except (TypeError, ValueError) as error:
            self.get_logger().error(f"Rejected trajectory: {error}")
            self._controller.clear_path()
            self._path_signature = None
            self._last_path_monotonic = None
            return

        self._last_path_monotonic = now
        if signature == self._path_signature:
            return
        self._controller.set_path(points)
        self._path_signature = signature
        self._gear_resume_monotonic = None
        self.get_logger().info(
            f"Loaded new MPC path: {len(points)} points, "
            f"direction={points[0].direction}->{points[-1].direction}"
        )

    def _path_valid_callback(self, message: Bool) -> None:
        self._path_valid = bool(message.data)
        if self._path_valid:
            return
        self._controller.clear_path()
        self._path_signature = None
        self._last_path_monotonic = None
        self._gear_resume_monotonic = None
        self._publish_zero("PATH_INVALIDATED")

    def _decode_trajectory(
        self,
        message: Float64MultiArray,
    ) -> tuple[ReferencePoint, ...]:
        if len(message.layout.dim) < 2:
            raise ValueError("trajectory layout needs point and field dimensions")
        field_dimension = message.layout.dim[1]
        expected_label = "fields=" + ",".join(EXPECTED_FIELDS)
        if field_dimension.size != len(EXPECTED_FIELDS):
            raise ValueError("trajectory must contain exactly four fields")
        if field_dimension.label != expected_label:
            raise ValueError(
                f"trajectory field label must be '{expected_label}'"
            )
        if len(message.data) % len(EXPECTED_FIELDS) != 0:
            raise ValueError("trajectory data length is not divisible by four")
        points: list[ReferencePoint] = []
        for index in range(0, len(message.data), len(EXPECTED_FIELDS)):
            direction_value = float(message.data[index + 3])
            if direction_value not in (-1.0, 1.0):
                raise ValueError(
                    f"trajectory point {index // 4} direction must be -1 or 1"
                )
            points.append(
                ReferencePoint(
                    x_m=float(message.data[index]),
                    y_m=float(message.data[index + 1]),
                    yaw_rad=float(message.data[index + 2]),
                    direction=int(direction_value),
                )
            )
        if len(points) < 2:
            raise ValueError("trajectory needs at least two points")
        return tuple(points)

    def _pose_callback(self, message: PoseStamped) -> None:
        if message.header.frame_id and message.header.frame_id != "lidar_map":
            self.get_logger().error(
                f"Rejected pose frame '{message.header.frame_id}', expected lidar_map"
            )
            return
        self._state = VehicleState(
            x_m=float(message.pose.position.x),
            y_m=float(message.pose.position.y),
            yaw_rad=quaternion_yaw(message),
        )
        self._last_pose_monotonic = time.monotonic()

    def _scan_callback(self, message: LaserScan) -> None:
        self._front_obstacle_m, self._rear_obstacle_m = scan_sector_minima(
            message.ranges,
            float(message.angle_min),
            float(message.angle_increment),
            float(message.range_min),
            float(message.range_max),
            self._front_half_angle_rad,
            self._rear_half_angle_rad,
            self._front_center_angle_rad,
            self._rear_center_angle_rad,
        )
        self._last_scan_monotonic = time.monotonic()

    def _control_tick(self) -> None:
        now = time.monotonic()
        if self._has_cmd_vel_conflict():
            self._publish_zero("CMD_VEL_PUBLISHER_CONFLICT")
            return
        if not self._path_valid:
            self._publish_zero("PATH_INVALIDATED")
            return
        if self._state is None or self._last_pose_monotonic is None:
            self._publish_zero("WAITING_FOR_POSE")
            return
        if now - self._last_pose_monotonic > self._controller.limits.pose_timeout_sec:
            self._publish_zero("POSE_TIMEOUT")
            return
        if self._last_path_monotonic is None:
            self._publish_zero("WAITING_FOR_PATH")
            return
        if now - self._last_path_monotonic > self._path_timeout_sec:
            self._publish_zero("PATH_TIMEOUT")
            return
        if self._require_scan:
            if self._last_scan_monotonic is None:
                self._publish_zero("WAITING_FOR_SCAN")
                return
            if now - self._last_scan_monotonic > self._scan_timeout_sec:
                self._publish_zero("SCAN_TIMEOUT")
                return
            if self._front_obstacle_m is None or self._rear_obstacle_m is None:
                self._publish_zero("SCAN_SECTOR_INVALID")
                return

        if self._gear_resume_monotonic is not None:
            if now < self._gear_resume_monotonic:
                self._publish_zero("GEAR_PAUSE")
                return
            if not self._controller.advance_gear_segment():
                self._publish_zero("GEAR_ADVANCE_FAILED")
                return
            self._gear_resume_monotonic = None

        command = self._controller.command(self._state)
        if command.status == "GEAR_CHANGE_REQUIRED":
            self._gear_resume_monotonic = now + self._gear_pause_sec
            self._publish_zero("GEAR_PAUSE")
            return
        if command.status != "TRACKING":
            self._publish_zero(command.status)
            return
        if (
            command.linear_mps > 0.0
            and self._front_obstacle_m is not None
            and self._front_obstacle_m <= self._front_stop_distance_m
        ):
            self._publish_zero(
                f"FRONT_OBSTACLE_{self._front_obstacle_m:.3f}m"
            )
            return
        if (
            command.linear_mps < 0.0
            and self._rear_obstacle_m is not None
            and self._rear_obstacle_m <= self._rear_stop_distance_m
        ):
            self._publish_zero(
                f"REAR_OBSTACLE_{self._rear_obstacle_m:.3f}m"
            )
            return

        message = Twist()
        message.linear.x = command.linear_mps
        message.angular.z = self._angular_command_sign * command.angular_radps
        self._command_publisher.publish(message)
        self._log_status(command.status)
        if (
            command.solve_time_sec > self._controller.limits.dt_sec
            and now - self._last_solve_log_monotonic > 1.0
        ):
            self.get_logger().warning(
                "MPC solve overrun: "
                f"{command.solve_time_sec * 1000.0:.1f} ms > "
                f"{self._controller.limits.dt_sec * 1000.0:.1f} ms"
            )
            self._last_solve_log_monotonic = now

    def _has_cmd_vel_conflict(self) -> bool:
        if not self._reject_cmd_vel_conflicts:
            return False
        publishers = self.get_publishers_info_by_topic(self._cmd_vel_topic)
        return any(
            publisher.node_name != self.get_name()
            or publisher.node_namespace.rstrip("/") != self.get_namespace().rstrip("/")
            for publisher in publishers
        )

    def _publish_zero(self, status: str) -> None:
        self._controller.stop(status)
        self._command_publisher.publish(Twist())
        self._log_status(status)

    def _log_status(self, status: str) -> None:
        if status == self._last_status:
            return
        self._last_status = status
        self.get_logger().info(f"MPC status: {status}")


def main(args: list[str] | None = None) -> int:
    rclpy.init(args=args)
    node: MpcPathFollower | None = None
    exit_code = 0
    try:
        node = MpcPathFollower()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except ValueError as error:
        exit_code = 1
        if node is not None:
            node.get_logger().error(f"MPC configuration error: {error}")
        else:
            print(f"MPC configuration error: {error}")
    finally:
        if node is not None:
            if rclpy.ok():
                node._command_publisher.publish(Twist())
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
