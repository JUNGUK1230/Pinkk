"""고정 trajectory를 추종해 실차 cmd_vel을 내보내는 안전 정지형 MPC 노드."""

from __future__ import annotations

import hashlib
import math
from pathlib import Path
import struct
import sys
import time
from typing import Sequence

import yaml

try:
    import rclpy
    from geometry_msgs.msg import PoseStamped, Twist
    from rcl_interfaces.msg import SetParametersResult
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.parameter import Parameter
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
STATIC_PARAMETERS = frozenset(
    {
        "trajectory_topic",
        "path_valid_topic",
        "pose_topic",
        "cmd_vel_topic",
        "scan_topic",
        "tuning_file",
        "tuning_reload_period_sec",
    }
)


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
        self._pose_received_after_path = False
        self._path_valid = True
        self._gear_resume_monotonic: float | None = None
        self._last_status: str | None = None
        self._last_solve_log_monotonic = 0.0
        self.add_on_set_parameters_callback(self._on_set_parameters)
        tuning_file = str(self.get_parameter("tuning_file").value)
        self._tuning_path = Path(tuning_file).expanduser()
        if not self._tuning_path.is_absolute():
            self._tuning_path = Path.cwd() / self._tuning_path
        self._auto_reload_tuning = bool(
            self.get_parameter("auto_reload_tuning").value
        )
        self._tuning_modified_ns = (
            self._tuning_path.stat().st_mtime_ns
            if self._tuning_path.is_file()
            else None
        )
        tuning_reload_period_sec = self._positive_parameter(
            "tuning_reload_period_sec"
        )
        self._tuning_timer = self.create_timer(
            tuning_reload_period_sec,
            self._reload_tuning_if_changed,
        )
        self.get_logger().warning(
            "MPC motion output enabled on %s; do not run another velocity controller"
            % self._cmd_vel_topic
        )

    def _declare_parameters(self) -> None:
        defaults = {
            "trajectory_topic": "/pinkk/planned_trajectory",
            "path_valid_topic": "/pinkk/path_valid",
            "pose_topic": "/pinkk/fused_vehicle_pose",
            "tuning_file": "src/vehicle_control/config/mpc/mpc.yaml",
            "auto_reload_tuning": True,
            "tuning_reload_period_sec": 1.0,
            "cmd_vel_topic": "/cmd_vel",
            # Pinky 실차 구동계에서 map curvature와 동일한 부호를 사용한다.
            "angular_command_sign": 1.0,
            "scan_topic": "/scan",
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
            "control_point_offset_m": 0.04,
            "wheel_radius_m": 0.027,
            "wheel_separation_m": 0.0961,
            "max_wheel_angular_speed_radps": 0.105 / 0.027,
            "dt_sec": 0.25,
            "horizon_steps": 10,
            "forward_speed_mps": 0.06,
            "reverse_speed_mps": 0.02,
            "max_forward_speed_mps": 0.08,
            "max_reverse_speed_mps": 0.03,
            "max_acceleration_mps2": 0.12,
            "max_curvature_1pm": 1.0 / 0.12,
            "max_curvature_rate_1pmps": 10.0,
            "max_angular_speed_radps": 0.40,
            "straight_curvature_threshold_1pm": 0.35,
            "straight_max_curvature_1pm": 3.0,
            "cross_track_feedback_gain_1pm2": 15.0,
            "heading_feedback_gain_1pmprad": 4.0,
            "heading_feedback_deadband_deg": 2.0,
            "reverse_cross_track_deadband_m": 0.006,
            "reverse_heading_feedback_deadband_deg": 2.0,
            "reverse_cross_track_gain_scale": 1.20,
            "reverse_heading_gain_scale": 1.15,
            "cross_track_deadband_m": 0.003,
            "cross_track_slowdown_start_m": 0.01,
            "cross_track_slowdown_full_m": 0.04,
            "minimum_tracking_speed_scale": 0.35,
            "max_tracking_yaw_error_deg": 25.0,
            "heading_recovery_full_curvature_error_deg": 45.0,
            "heading_recovery_speed_scale": 0.60,
            "pose_timeout_sec": 0.6,
            "goal_position_tolerance_m": 0.03,
            "goal_yaw_tolerance_deg": 8.0,
            "gear_position_tolerance_m": 0.01,
            "gear_fallback_position_tolerance_m": 0.04,
            "gear_stall_speed_threshold_mps": 0.003,
            "gear_fallback_max_segment_length_m": 0.15,
            "gear_passed_endpoint_lateral_tolerance_m": 0.06,
            "gear_transition_end_guard_points": 20,
            "nearest_forward_window": 140,
            "nearest_backward_window": 4,
            "steering_preview_points": 6,
            "steering_preview_weight": 0.30,
            "steering_rejoin_preview_points": 12,
            "steering_rejoin_full_error_m": 0.03,
            "steering_rejoin_preview_weight": 0.65,
            "curve_feedforward_preview_points": 14,
            "curve_feedforward_gain": 0.55,
            "curve_feedforward_deadband_1pm": 0.5,
            "curvature_smoothing_points": 5,
            "straight_lookahead_points": 4,
            "straight_history_points": 12,
            "straight_end_guard_points": 30,
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

    def _parameter_value(
        self,
        name: str,
        overrides: dict[str, object] | None = None,
    ) -> object:
        if overrides is not None and name in overrides:
            return overrides[name]
        return self.get_parameter(name).value

    def _load_limits(
        self,
        overrides: dict[str, object] | None = None,
    ) -> MpcLimits:
        return MpcLimits(
            control_point_offset_m=self._nonnegative_parameter(
                "control_point_offset_m", overrides
            ),
            wheel_radius_m=self._positive_parameter(
                "wheel_radius_m", overrides
            ),
            wheel_separation_m=self._positive_parameter(
                "wheel_separation_m", overrides
            ),
            max_wheel_angular_speed_radps=self._positive_parameter(
                "max_wheel_angular_speed_radps", overrides
            ),
            dt_sec=self._positive_parameter("dt_sec", overrides),
            horizon_steps=int(self._parameter_value("horizon_steps", overrides)),
            forward_speed_mps=self._positive_parameter(
                "forward_speed_mps", overrides
            ),
            reverse_speed_mps=self._positive_parameter(
                "reverse_speed_mps", overrides
            ),
            max_forward_speed_mps=self._positive_parameter(
                "max_forward_speed_mps", overrides
            ),
            max_reverse_speed_mps=self._positive_parameter(
                "max_reverse_speed_mps", overrides
            ),
            max_acceleration_mps2=self._positive_parameter(
                "max_acceleration_mps2", overrides
            ),
            max_curvature_1pm=self._positive_parameter(
                "max_curvature_1pm", overrides
            ),
            max_curvature_rate_1pmps=self._positive_parameter(
                "max_curvature_rate_1pmps", overrides
            ),
            max_angular_speed_radps=self._positive_parameter(
                "max_angular_speed_radps", overrides
            ),
            straight_curvature_threshold_1pm=self._positive_parameter(
                "straight_curvature_threshold_1pm", overrides
            ),
            straight_max_curvature_1pm=self._positive_parameter(
                "straight_max_curvature_1pm", overrides
            ),
            cross_track_feedback_gain_1pm2=self._nonnegative_parameter(
                "cross_track_feedback_gain_1pm2", overrides
            ),
            heading_feedback_gain_1pmprad=self._nonnegative_parameter(
                "heading_feedback_gain_1pmprad", overrides
            ),
            heading_feedback_deadband_rad=math.radians(
                self._nonnegative_parameter(
                    "heading_feedback_deadband_deg",
                    overrides,
                )
            ),
            reverse_cross_track_deadband_m=self._positive_parameter(
                "reverse_cross_track_deadband_m",
                overrides,
            ),
            reverse_heading_feedback_deadband_rad=math.radians(
                self._nonnegative_parameter(
                    "reverse_heading_feedback_deadband_deg",
                    overrides,
                )
            ),
            reverse_cross_track_gain_scale=self._positive_parameter(
                "reverse_cross_track_gain_scale",
                overrides,
            ),
            reverse_heading_gain_scale=self._positive_parameter(
                "reverse_heading_gain_scale",
                overrides,
            ),
            cross_track_deadband_m=self._positive_parameter(
                "cross_track_deadband_m", overrides
            ),
            cross_track_slowdown_start_m=self._positive_parameter(
                "cross_track_slowdown_start_m", overrides
            ),
            cross_track_slowdown_full_m=self._positive_parameter(
                "cross_track_slowdown_full_m", overrides
            ),
            minimum_tracking_speed_scale=self._positive_parameter(
                "minimum_tracking_speed_scale", overrides
            ),
            max_tracking_yaw_error_rad=math.radians(
                self._positive_parameter("max_tracking_yaw_error_deg", overrides)
            ),
            heading_recovery_full_curvature_error_rad=math.radians(
                self._positive_parameter(
                    "heading_recovery_full_curvature_error_deg", overrides
                )
            ),
            heading_recovery_speed_scale=self._positive_parameter(
                "heading_recovery_speed_scale", overrides
            ),
            pose_timeout_sec=self._positive_parameter(
                "pose_timeout_sec", overrides
            ),
            goal_position_tolerance_m=self._positive_parameter(
                "goal_position_tolerance_m", overrides
            ),
            goal_yaw_tolerance_rad=math.radians(
                self._positive_parameter("goal_yaw_tolerance_deg", overrides)
            ),
            gear_position_tolerance_m=self._positive_parameter(
                "gear_position_tolerance_m", overrides
            ),
            gear_fallback_position_tolerance_m=self._positive_parameter(
                "gear_fallback_position_tolerance_m", overrides
            ),
            gear_stall_speed_threshold_mps=self._positive_parameter(
                "gear_stall_speed_threshold_mps", overrides
            ),
            gear_fallback_max_segment_length_m=self._positive_parameter(
                "gear_fallback_max_segment_length_m", overrides
            ),
            gear_passed_endpoint_lateral_tolerance_m=self._positive_parameter(
                "gear_passed_endpoint_lateral_tolerance_m",
                overrides,
            ),
            gear_transition_end_guard_points=int(
                self._parameter_value(
                    "gear_transition_end_guard_points", overrides
                )
            ),
            nearest_forward_window=int(
                self._parameter_value("nearest_forward_window", overrides)
            ),
            nearest_backward_window=int(
                self._parameter_value("nearest_backward_window", overrides)
            ),
            steering_preview_points=int(
                self._parameter_value("steering_preview_points", overrides)
            ),
            steering_preview_weight=self._nonnegative_parameter(
                "steering_preview_weight", overrides
            ),
            steering_rejoin_preview_points=int(
                self._parameter_value(
                    "steering_rejoin_preview_points", overrides
                )
            ),
            steering_rejoin_full_error_m=self._positive_parameter(
                "steering_rejoin_full_error_m", overrides
            ),
            steering_rejoin_preview_weight=self._nonnegative_parameter(
                "steering_rejoin_preview_weight", overrides
            ),
            curve_feedforward_preview_points=int(
                self._parameter_value(
                    "curve_feedforward_preview_points",
                    overrides,
                )
            ),
            curve_feedforward_gain=self._nonnegative_parameter(
                "curve_feedforward_gain",
                overrides,
            ),
            curve_feedforward_deadband_1pm=self._nonnegative_parameter(
                "curve_feedforward_deadband_1pm",
                overrides,
            ),
            curvature_smoothing_points=int(
                self._parameter_value("curvature_smoothing_points", overrides)
            ),
            straight_lookahead_points=int(
                self._parameter_value("straight_lookahead_points", overrides)
            ),
            straight_history_points=int(
                self._parameter_value("straight_history_points", overrides)
            ),
            straight_end_guard_points=int(
                self._parameter_value("straight_end_guard_points", overrides)
            ),
            solver_max_iterations=int(
                self._parameter_value("solver_max_iterations", overrides)
            ),
            solver_ftol=self._positive_parameter("solver_ftol", overrides),
        )

    def _load_weights(
        self,
        overrides: dict[str, object] | None = None,
    ) -> MpcWeights:
        return MpcWeights(
            position=self._nonnegative_parameter("weight_position", overrides),
            yaw=self._nonnegative_parameter("weight_yaw", overrides),
            terminal_position=self._nonnegative_parameter(
                "weight_terminal_position", overrides
            ),
            terminal_yaw=self._nonnegative_parameter(
                "weight_terminal_yaw", overrides
            ),
            speed=self._nonnegative_parameter("weight_speed", overrides),
            curvature=self._nonnegative_parameter(
                "weight_curvature", overrides
            ),
            speed_rate=self._nonnegative_parameter(
                "weight_speed_rate", overrides
            ),
            curvature_rate=self._nonnegative_parameter(
                "weight_curvature_rate", overrides
            ),
        )

    def _positive_parameter(
        self,
        name: str,
        overrides: dict[str, object] | None = None,
    ) -> float:
        value = float(self._parameter_value(name, overrides))
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"{name} must be positive and finite")
        return value

    def _nonnegative_parameter(
        self,
        name: str,
        overrides: dict[str, object] | None = None,
    ) -> float:
        value = float(self._parameter_value(name, overrides))
        if not math.isfinite(value) or value < 0.0:
            raise ValueError(f"{name} must be finite and non-negative")
        return value

    def _on_set_parameters(self, parameters: Sequence[object]) -> SetParametersResult:
        """YAML/ros2 param 변경을 검증한 뒤 실행 중 제어기에 반영한다."""
        updates = {
            str(getattr(parameter, "name")): getattr(parameter, "value")
            for parameter in parameters
        }
        for name in STATIC_PARAMETERS.intersection(updates):
            if updates[name] != self.get_parameter(name).value:
                return SetParametersResult(
                    successful=False,
                    reason=f"{name} requires node restart",
                )
        try:
            new_limits = self._load_limits(updates)
            new_weights = self._load_weights(updates)
            new_limits.validate()
            new_weights.validate()
            path_timeout_sec = self._positive_parameter(
                "path_timeout_sec", updates
            )
            gear_pause_sec = self._positive_parameter("gear_pause_sec", updates)
            scan_timeout_sec = self._positive_parameter(
                "scan_timeout_sec", updates
            )
            front_stop_distance_m = self._positive_parameter(
                "front_stop_distance_m", updates
            )
            rear_stop_distance_m = self._positive_parameter(
                "rear_stop_distance_m", updates
            )
            front_half_angle_rad = math.radians(
                self._positive_parameter("front_scan_half_angle_deg", updates)
            )
            rear_half_angle_rad = math.radians(
                self._positive_parameter("rear_scan_half_angle_deg", updates)
            )
            front_center_angle_rad = math.radians(
                float(self._parameter_value("front_scan_center_deg", updates))
            )
            rear_center_angle_rad = math.radians(
                float(self._parameter_value("rear_scan_center_deg", updates))
            )
            angular_command_sign = float(
                self._parameter_value("angular_command_sign", updates)
            )
            if angular_command_sign not in (-1.0, 1.0):
                raise ValueError("angular_command_sign must be -1 or 1")
            control_frequency_hz = self._positive_parameter(
                "control_frequency_hz", updates
            )
            require_scan = bool(
                self._parameter_value("require_scan", updates)
            )
            reject_cmd_vel_conflicts = bool(
                self._parameter_value("reject_cmd_vel_conflicts", updates)
            )
            auto_reload_tuning = bool(
                self._parameter_value("auto_reload_tuning", updates)
            )
        except (TypeError, ValueError) as error:
            return SetParametersResult(successful=False, reason=str(error))

        controller_changed = (
            new_limits != self._controller.limits
            or new_weights != self._controller.weights
        )
        if controller_changed:
            path = self._controller.path
            progress_index = self._controller.progress_index
            self._controller.limits = new_limits
            self._controller.weights = new_weights
            if path:
                self._controller.set_path(path)
                self._controller.restore_progress(
                    min(progress_index, len(path) - 1)
                )

        self._path_timeout_sec = path_timeout_sec
        self._gear_pause_sec = gear_pause_sec
        self._scan_timeout_sec = scan_timeout_sec
        self._front_stop_distance_m = front_stop_distance_m
        self._rear_stop_distance_m = rear_stop_distance_m
        self._front_half_angle_rad = front_half_angle_rad
        self._rear_half_angle_rad = rear_half_angle_rad
        self._front_center_angle_rad = front_center_angle_rad
        self._rear_center_angle_rad = rear_center_angle_rad
        self._angular_command_sign = angular_command_sign
        self._require_scan = require_scan
        self._reject_cmd_vel_conflicts = reject_cmd_vel_conflicts
        self._auto_reload_tuning = auto_reload_tuning

        current_frequency = 1.0 / self._timer.timer_period_ns * 1e9
        if not math.isclose(
            control_frequency_hz,
            current_frequency,
            rel_tol=1e-9,
        ):
            self.destroy_timer(self._timer)
            self._timer = self.create_timer(
                1.0 / control_frequency_hz,
                self._control_tick,
            )

        self._publish_zero("PARAMETERS_UPDATED")
        self.get_logger().info(
            "Applied MPC tuning parameters; path progress preserved"
        )
        return SetParametersResult(successful=True)

    def _reload_tuning_if_changed(self) -> None:
        """저장된 YAML 전체를 한 번에 검증하고 안전하게 적용한다."""
        if not self._auto_reload_tuning:
            return
        try:
            modified_ns = self._tuning_path.stat().st_mtime_ns
        except OSError as error:
            self.get_logger().warning(
                f"MPC tuning file unavailable: {self._tuning_path}: {error}"
            )
            return
        if modified_ns == self._tuning_modified_ns:
            return
        self._tuning_modified_ns = modified_ns
        try:
            with self._tuning_path.open(encoding="utf-8") as file:
                document = yaml.safe_load(file)
            node_config = document.get(self.get_name())
            if not isinstance(node_config, dict):
                raise ValueError(
                    f"missing YAML node section: {self.get_name()}"
                )
            raw_parameters = node_config.get("ros__parameters")
            if not isinstance(raw_parameters, dict):
                raise ValueError("missing ros__parameters mapping")
            parameters = [
                Parameter(str(name), value=value)
                for name, value in raw_parameters.items()
            ]
            result = self.set_parameters_atomically(parameters)
            if not result.successful:
                raise ValueError(result.reason)
        except (OSError, TypeError, ValueError, yaml.YAMLError) as error:
            self.get_logger().error(
                f"Rejected MPC tuning file; keeping previous values: {error}"
            )
            return
        self.get_logger().info(f"Reloaded MPC tuning file: {self._tuning_path}")

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
            self._pose_received_after_path = False
            return

        self._last_path_monotonic = now
        if signature == self._path_signature:
            return
        self._controller.set_path(points)
        self._path_signature = signature
        self._pose_received_after_path = False
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
        self._pose_received_after_path = False
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
        if self._path_signature is not None:
            self._pose_received_after_path = True

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
        if self._last_path_monotonic is None:
            self._publish_zero("WAITING_FOR_PATH")
            return
        if not self._pose_received_after_path:
            self._publish_zero("WAITING_FOR_POSE_AFTER_PATH")
            return
        if self._state is None or self._last_pose_monotonic is None:
            self._publish_zero("WAITING_FOR_POSE")
            return
        if now - self._last_pose_monotonic > self._controller.limits.pose_timeout_sec:
            self._publish_zero("POSE_TIMEOUT")
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
            self.get_logger().info(
                "Advanced gear segment: "
                f"path_index={self._controller.progress_index}, "
                f"direction={self._controller.path[self._controller.progress_index].direction}"
            )
            self._gear_resume_monotonic = None

        command = self._controller.command(self._state)
        if command.status == "GEAR_CHANGE_REQUIRED":
            self.get_logger().info(
                f"Gear change requested at path_index={command.progress_index}"
            )
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
