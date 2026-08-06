"""MyCobot280용 단순 FollowJointTrajectory 실행 및 관절 상태 브리지."""

from __future__ import annotations

import math
import threading
import time
from pathlib import Path
from typing import Sequence

import rclpy
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseStamped
from pinkk_usb_insertion_interfaces.action import CartesianMove
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

from .cartesian_conversion import (
    apply_cartesian_locks,
    is_z_dominant_recovery_motion,
    pose_error,
    pose_values_to_robot_coords,
    robot_coords_to_pose_values,
    wrapped_angle_difference_deg,
)
from .command_queue import (
    prepare_command_queue,
    require_no_explicit_command_failure,
    stop_and_clear_command_queue,
)
from .joint_state_publisher import JOINT_NAMES, angles_deg_to_rad
from .joint_completion import (
    JointStabilityMonitor,
    compensated_joint_command_degrees,
    maximum_joint_error,
    signed_joint_errors_degrees,
)


def duration_seconds(duration: object) -> float:
    """ROS Duration 메시지를 초 단위 실수로 변환한다."""
    seconds = float(getattr(duration, "sec")) + float(getattr(duration, "nanosec")) * 1e-9
    if not math.isfinite(seconds) or seconds < 0.0:
        raise ValueError("time_from_start가 유효하지 않습니다")
    return seconds


def validate_trajectory(joint_names: Sequence[str], points: Sequence[object]) -> list[float]:
    """MoveIt trajectory를 검사하고 마지막 목표 관절각(rad)을 반환한다."""
    if tuple(joint_names) != JOINT_NAMES:
        raise ValueError(f"관절 이름 또는 순서가 다릅니다: {list(joint_names)}")
    if not points:
        raise ValueError("trajectory point가 없습니다")

    previous_time = -1.0
    for index, point in enumerate(points):
        positions = list(getattr(point, "positions"))
        if len(positions) != len(JOINT_NAMES):
            raise ValueError(f"point {index}의 관절각 개수가 6개가 아닙니다")
        numeric = [float(value) for value in positions]
        if not all(math.isfinite(value) for value in numeric):
            raise ValueError(f"point {index}에 NaN 또는 inf가 있습니다")
        if any(abs(value) > math.pi for value in numeric):
            raise ValueError(f"point {index}가 ±180도 범위를 벗어났습니다")
        current_time = duration_seconds(getattr(point, "time_from_start"))
        if current_time < previous_time:
            raise ValueError("time_from_start가 증가 순서가 아닙니다")
        previous_time = current_time
    return [float(value) for value in points[-1].positions]


def radians_to_degrees(values: Sequence[float]) -> list[float]:
    return [round(math.degrees(float(value)), 3) for value in values]


class MyCobotTrajectoryBridge(Node):
    """하나의 serial 연결로 실제 상태 발행과 최종 목표 자세 실행을 담당한다."""

    ACTION_NAME = "/arm_group_controller/follow_joint_trajectory"
    CARTESIAN_ACTION_NAME = "/robot_arm/cartesian_move"

    def __init__(self) -> None:
        super().__init__("pinkk_mycobot_trajectory_bridge")
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baud", 1_000_000)
        self.declare_parameter("speed", 10)
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("cartesian_pose_publish_rate_hz", 2.0)
        self.declare_parameter("goal_tolerance_deg", 2.0)
        self.declare_parameter("joint_execution_enabled", False)
        self.declare_parameter("joint_stable_sample_count", 5)
        self.declare_parameter("joint_stable_delta_deg", 0.2)
        self.declare_parameter("joint_hold_check_seconds", 2.0)
        self.declare_parameter("joint_max_command_attempts", 1)
        self.declare_parameter("joint_retry_stable_sample_count", 3)
        self.declare_parameter("joint_retry_minimum_progress_deg", 0.1)
        self.declare_parameter("joint_retry_compensation_enabled", False)
        self.declare_parameter("joint_retry_compensation_gain", 0.8)
        self.declare_parameter("joint_retry_max_step_deg", 1.0)
        self.declare_parameter("joint_retry_max_total_offset_deg", 2.0)
        self.declare_parameter("max_execution_seconds", 60.0)
        self.declare_parameter("cartesian_execution_enabled", False)
        self.declare_parameter("cartesian_base_frame", "g_base")
        self.declare_parameter("cartesian_position_tolerance_m", 0.0005)
        self.declare_parameter("cartesian_orientation_tolerance_deg", 1.0)
        self.declare_parameter("cartesian_max_translation_m", 0.0105)
        self.declare_parameter("cartesian_max_rotation_deg", 2.1)
        self.declare_parameter("cartesian_path_z_tolerance_m", 0.002)
        self.declare_parameter("cartesian_path_tilt_tolerance_deg", 3.0)
        self.declare_parameter("cartesian_timeout_seconds", 15.0)
        self.declare_parameter("cartesian_ignore_z_tracking_error", False)
        self.declare_parameter("cartesian_free_z_minimum_motion_m", 0.001)
        self.declare_parameter("cartesian_no_motion_timeout_seconds", 5.0)
        self.declare_parameter("cartesian_progress_log_seconds", 2.0)

        port = str(self.get_parameter("port").value)
        baud = int(self.get_parameter("baud").value)
        self._speed = int(self.get_parameter("speed").value)
        rate = float(self.get_parameter("publish_rate_hz").value)
        cartesian_pose_rate = float(
            self.get_parameter("cartesian_pose_publish_rate_hz").value
        )
        self._tolerance_rad = math.radians(
            float(self.get_parameter("goal_tolerance_deg").value)
        )
        self._joint_execution_enabled = bool(
            self.get_parameter("joint_execution_enabled").value
        )
        self._joint_stable_sample_count = int(
            self.get_parameter("joint_stable_sample_count").value
        )
        self._joint_stable_delta_rad = math.radians(
            float(self.get_parameter("joint_stable_delta_deg").value)
        )
        self._joint_hold_check_seconds = float(
            self.get_parameter("joint_hold_check_seconds").value
        )
        self._joint_max_command_attempts = int(
            self.get_parameter("joint_max_command_attempts").value
        )
        self._joint_retry_stable_sample_count = int(
            self.get_parameter("joint_retry_stable_sample_count").value
        )
        self._joint_retry_minimum_progress_rad = math.radians(
            float(
                self.get_parameter(
                    "joint_retry_minimum_progress_deg"
                ).value
            )
        )
        self._joint_retry_compensation_enabled = bool(
            self.get_parameter("joint_retry_compensation_enabled").value
        )
        self._joint_retry_compensation_gain = float(
            self.get_parameter("joint_retry_compensation_gain").value
        )
        self._joint_retry_max_step_deg = float(
            self.get_parameter("joint_retry_max_step_deg").value
        )
        self._joint_retry_max_total_offset_deg = float(
            self.get_parameter(
                "joint_retry_max_total_offset_deg"
            ).value
        )
        self._max_execution_seconds = float(
            self.get_parameter("max_execution_seconds").value
        )
        self._cartesian_execution_enabled = bool(
            self.get_parameter("cartesian_execution_enabled").value
        )
        self._cartesian_base_frame = str(
            self.get_parameter("cartesian_base_frame").value
        )
        self._cartesian_position_tolerance = float(
            self.get_parameter("cartesian_position_tolerance_m").value
        )
        self._cartesian_orientation_tolerance = float(
            self.get_parameter("cartesian_orientation_tolerance_deg").value
        )
        self._cartesian_max_translation = float(
            self.get_parameter("cartesian_max_translation_m").value
        )
        self._cartesian_max_rotation = float(
            self.get_parameter("cartesian_max_rotation_deg").value
        )
        self._cartesian_path_z_tolerance = float(
            self.get_parameter("cartesian_path_z_tolerance_m").value
        )
        self._cartesian_path_tilt_tolerance = float(
            self.get_parameter("cartesian_path_tilt_tolerance_deg").value
        )
        self._cartesian_timeout = float(
            self.get_parameter("cartesian_timeout_seconds").value
        )
        self._cartesian_ignore_z_tracking_error = bool(
            self.get_parameter("cartesian_ignore_z_tracking_error").value
        )
        self._cartesian_free_z_minimum_motion = float(
            self.get_parameter("cartesian_free_z_minimum_motion_m").value
        )
        self._cartesian_no_motion_timeout = float(
            self.get_parameter("cartesian_no_motion_timeout_seconds").value
        )
        self._cartesian_progress_log_seconds = float(
            self.get_parameter("cartesian_progress_log_seconds").value
        )
        if not Path(port).exists():
            raise FileNotFoundError(f"로봇 serial port가 없습니다: {port}")
        if not 1 <= self._speed <= 100:
            raise ValueError("speed는 1~100이어야 합니다")
        if rate <= 0.0 or rate > 50.0:
            raise ValueError("publish_rate_hz는 0보다 크고 50 이하여야 합니다")
        if cartesian_pose_rate <= 0.0 or cartesian_pose_rate > 10.0:
            raise ValueError(
                "cartesian_pose_publish_rate_hz는 0보다 크고 10 이하여야 합니다"
            )
        if self._tolerance_rad <= 0.0:
            raise ValueError("goal_tolerance_deg는 0보다 커야 합니다")
        if self._joint_stable_sample_count < 2:
            raise ValueError("joint_stable_sample_count는 최소 2여야 합니다")
        if self._joint_stable_delta_rad <= 0.0:
            raise ValueError("joint_stable_delta_deg는 0보다 커야 합니다")
        if self._joint_hold_check_seconds <= 0.0:
            raise ValueError("joint_hold_check_seconds는 0보다 커야 합니다")
        if not 1 <= self._joint_max_command_attempts <= 3:
            raise ValueError("joint_max_command_attempts는 1~3이어야 합니다")
        if self._joint_retry_stable_sample_count < 2:
            raise ValueError(
                "joint_retry_stable_sample_count는 최소 2여야 합니다"
            )
        if self._joint_retry_minimum_progress_rad <= 0.0:
            raise ValueError(
                "joint_retry_minimum_progress_deg는 0보다 커야 합니다"
            )
        if not 0.0 < self._joint_retry_compensation_gain <= 1.0:
            raise ValueError(
                "joint_retry_compensation_gain은 0보다 크고 1 이하여야 합니다"
            )
        if self._joint_retry_max_step_deg <= 0.0:
            raise ValueError("joint_retry_max_step_deg는 0보다 커야 합니다")
        if (
            self._joint_retry_max_total_offset_deg
            < self._joint_retry_max_step_deg
        ):
            raise ValueError(
                "joint_retry_max_total_offset_deg는 "
                "joint_retry_max_step_deg 이상이어야 합니다"
            )
        if self._max_execution_seconds <= 0.0:
            raise ValueError("max_execution_seconds는 0보다 커야 합니다")
        positive_cartesian_parameters = (
            self._cartesian_position_tolerance,
            self._cartesian_orientation_tolerance,
            self._cartesian_max_translation,
            self._cartesian_max_rotation,
            self._cartesian_path_z_tolerance,
            self._cartesian_path_tilt_tolerance,
            self._cartesian_timeout,
            self._cartesian_free_z_minimum_motion,
            self._cartesian_no_motion_timeout,
            self._cartesian_progress_log_seconds,
        )
        if any(value <= 0.0 for value in positive_cartesian_parameters):
            raise ValueError("Cartesian 제한값과 timeout은 0보다 커야 합니다")

        try:
            from pymycobot import MyCobot280
        except ImportError as error:
            raise RuntimeError("pymycobot을 찾을 수 없습니다") from error

        self._serial_lock = threading.Lock()
        self._motion_lock = threading.Lock()
        self._robot = MyCobot280(port, baud)
        try:
            with self._serial_lock:
                prepare_command_queue(self._robot)
        except Exception as error:
            raise RuntimeError(
                f'MyCobot 이동 큐 안전 초기화 실패: {error}'
            ) from error
        self.get_logger().warning(
            'MyCobot 이동 안전 상태 확인 완료: fresh_mode=1, '
            'queue clear requested, stopped verified'
        )
        self._cartesian_ready = self._check_cartesian_api()
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._joint_publisher = self.create_publisher(JointState, "/joint_states", qos)
        self._cartesian_pose_publisher = self.create_publisher(
            PoseStamped,
            "/robot_arm/cartesian_pose_actual",
            qos,
        )
        self._latest_positions: list[float] | None = None
        self._state_timer = self.create_timer(1.0 / rate, self._read_and_publish_state)
        self._cartesian_pose_timer = self.create_timer(
            1.0 / cartesian_pose_rate,
            self._read_and_publish_cartesian_pose,
        )
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            self.ACTION_NAME,
            execute_callback=self._execute,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=ReentrantCallbackGroup(),
        )
        self._cartesian_action_server = ActionServer(
            self,
            CartesianMove,
            self.CARTESIAN_ACTION_NAME,
            execute_callback=self._execute_cartesian,
            goal_callback=self._cartesian_goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=ReentrantCallbackGroup(),
        )
        self.get_logger().info(
            f"실제 실행 브리지 준비: port={port}, baud={baud}, speed={self._speed}, "
            f"action={self.ACTION_NAME}"
        )
        joint_mode = "실행 허용" if self._joint_execution_enabled else "실행 차단"
        self.get_logger().warning(
            f"관절 send_angles action {joint_mode}: action={self.ACTION_NAME}, "
            f"tolerance={math.degrees(self._tolerance_rad):.2f}deg, "
            f"stable_samples={self._joint_stable_sample_count}, "
            f"max_attempts={self._joint_max_command_attempts}, "
            "retry_compensation="
            f"{self._joint_retry_compensation_enabled}"
        )
        api_mode = "API 준비" if self._cartesian_ready else "API 사용 불가"
        execution_mode = (
            "실행 허용" if self._cartesian_execution_enabled else "실행 차단"
        )
        self.get_logger().warning(
            f"Cartesian send_coords action {api_mode}, {execution_mode}: "
            f"action={self.CARTESIAN_ACTION_NAME}, max_step="
            f"{self._cartesian_max_translation * 1000.0:.1f}mm, "
            "ignore_z_tracking_error="
            f"{self._cartesian_ignore_z_tracking_error}, "
            "free_z_minimum_motion="
            f"{self._cartesian_free_z_minimum_motion * 1000.0:.1f}mm, "
            f"no_motion_timeout={self._cartesian_no_motion_timeout:.1f}s, "
            f"progress_log={self._cartesian_progress_log_seconds:.1f}s"
        )

    def _robot_error_diagnostic(self) -> str:
        reader = getattr(self._robot, 'get_error_information', None)
        if not callable(reader):
            return 'get_error_information API 없음'
        try:
            with self._serial_lock:
                code = reader()
        except Exception as error:
            return f'get_error_information 호출 실패: {error}'
        descriptions = {
            0: '펌웨어 오류 없음',
            32: '역기구학 해 없음',
            33: '선형 이동 인접 해 없음',
            34: '선형 이동 인접 해 없음',
        }
        if isinstance(code, int) and 1 <= code <= 6:
            return f'error={code}: joint{code} 한계 초과'
        return f'error={code!r}: {descriptions.get(code, "제조사 미정의 오류")}'

    def _check_cartesian_api(self) -> bool:
        required = (
            "get_coords",
            "send_coords",
            "get_reference_frame",
            "get_end_type",
        )
        missing = [
            name for name in required
            if not callable(getattr(self._robot, name, None))
        ]
        if missing:
            self.get_logger().error(f"Cartesian API가 없습니다: {missing}")
            return False
        try:
            with self._serial_lock:
                reference_frame = self._robot.get_reference_frame()
                end_type = self._robot.get_end_type()
        except Exception as error:
            self.get_logger().error(f"Cartesian 좌표계 확인 실패: {error}")
            return False
        if reference_frame != 0 or end_type != 0:
            self.get_logger().error(
                "Cartesian action 차단: reference_frame과 end_type은 "
                f"base(0)/flange(0)여야 합니다: {reference_frame}/{end_type}"
            )
            return False
        return True

    def _goal_callback(self, goal_request: FollowJointTrajectory.Goal) -> GoalResponse:
        try:
            if not self._joint_execution_enabled:
                raise ValueError("joint_execution_enabled=false")
            validate_trajectory(
                goal_request.trajectory.joint_names,
                goal_request.trajectory.points,
            )
            planned_seconds = duration_seconds(
                goal_request.trajectory.points[-1].time_from_start
            )
            if planned_seconds > self._max_execution_seconds:
                raise ValueError(
                    f"trajectory 시간이 {self._max_execution_seconds:.1f}초를 초과합니다"
                )
        except ValueError as error:
            self.get_logger().error(f"trajectory 거부: {error}")
            return GoalResponse.REJECT
        self.get_logger().info(
            f"trajectory 수락: points={len(goal_request.trajectory.points)}, "
            f"planned={planned_seconds:.2f}s"
        )
        return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle: object) -> CancelResponse:
        self.get_logger().warning("trajectory 취소 요청 수신")
        return CancelResponse.ACCEPT

    def _execute(self, goal_handle: object) -> FollowJointTrajectory.Result:
        result = FollowJointTrajectory.Result()
        if not self._motion_lock.acquire(blocking=False):
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = "다른 로봇 이동이 실행 중입니다"
            goal_handle.abort()
            return result
        try:
            return self._execute_joint_trajectory(goal_handle)
        finally:
            self._motion_lock.release()

    def _execute_joint_trajectory(
        self, goal_handle: object
    ) -> FollowJointTrajectory.Result:
        trajectory = goal_handle.request.trajectory
        target_rad = validate_trajectory(trajectory.joint_names, trajectory.points)
        target_deg = radians_to_degrees(target_rad)
        planned_seconds = duration_seconds(trajectory.points[-1].time_from_start)
        timeout = min(
            self._max_execution_seconds,
            max(10.0, planned_seconds + 8.0)
            * self._joint_max_command_attempts,
        )
        result = FollowJointTrajectory.Result()
        started = time.monotonic()
        attempts_sent = 0
        previous_attempt_error: float | None = None
        retry_stable_samples = 0
        previous_actual: list[float] | None = None
        last_command_deg = list(target_deg)

        try:
            self._send_joint_target(target_deg, attempts_sent + 1)
            attempts_sent += 1
        except Exception as error:
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = f"send_angles 실패: {error}"
            goal_handle.abort()
            return result

        stability = JointStabilityMonitor(
            self._tolerance_rad,
            self._joint_stable_delta_rad,
            self._joint_stable_sample_count,
        )
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self._stop_robot()
                goal_handle.canceled()
                result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                result.error_string = "사용자가 trajectory를 취소했습니다"
                return result

            actual = self._read_and_publish_state()
            if actual is not None:
                errors = [target - value for target, value in zip(target_rad, actual, strict=True)]
                maximum_error = maximum_joint_error(target_rad, actual)
                feedback = FollowJointTrajectory.Feedback()
                feedback.header.stamp = self.get_clock().now().to_msg()
                feedback.joint_names = list(JOINT_NAMES)
                feedback.desired = JointTrajectoryPoint(positions=list(target_rad))
                feedback.actual = JointTrajectoryPoint(positions=list(actual))
                feedback.error = JointTrajectoryPoint(positions=errors)
                goal_handle.publish_feedback(feedback)
                try:
                    robot_is_moving = self._robot_is_moving()
                    stable = stability.update(
                        target_rad,
                        actual,
                        robot_is_moving,
                    )
                except Exception as error:
                    self._stop_robot()
                    goal_handle.abort()
                    result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                    result.error_string = f"관절 정지 상태 확인 실패: {error}"
                    return result
                if stable:
                    if not self._stop_robot():
                        goal_handle.abort()
                        result.error_code = (
                            FollowJointTrajectory.Result.INVALID_GOAL
                        )
                        result.error_string = (
                            '목표 도달 후 정지·큐 삭제에 실패했습니다'
                        )
                        return result
                    if not self._verify_joint_hold(target_rad):
                        goal_handle.abort()
                        result.error_code = (
                            FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
                        )
                        result.error_string = (
                            "정지 후 관절 자세 유지 검증에 실패했습니다"
                        )
                        self.get_logger().error(result.error_string)
                        return result
                    goal_handle.succeed()
                    result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                    result.error_string = "목표 관절 자세 도달 및 정지 유지 확인"
                    self.get_logger().info(
                        "trajectory 실행 완료: 로봇 정지와 자세 유지 확인"
                    )
                    return result

                sample_stable = (
                    previous_actual is not None
                    and maximum_joint_error(previous_actual, actual)
                    <= self._joint_stable_delta_rad
                )
                if sample_stable and not robot_is_moving:
                    retry_stable_samples += 1
                else:
                    retry_stable_samples = 0
                previous_actual = list(actual)

                if (
                    maximum_error > self._tolerance_rad
                    and retry_stable_samples
                    >= self._joint_retry_stable_sample_count
                ):
                    diagnostic = self._joint_diagnostic(
                        target_rad,
                        actual,
                    )
                    if attempts_sent >= self._joint_max_command_attempts:
                        self._stop_robot()
                        goal_handle.abort()
                        result.error_code = (
                            FollowJointTrajectory.Result
                            .GOAL_TOLERANCE_VIOLATED
                        )
                        result.error_string = (
                            '관절 재전송 제한 후에도 목표 오차가 남았습니다: '
                            f'{diagnostic}'
                        )
                        self.get_logger().error(result.error_string)
                        return result
                    if (
                        previous_attempt_error is not None
                        and maximum_error
                        > previous_attempt_error
                        - self._joint_retry_minimum_progress_rad
                    ):
                        self._stop_robot()
                        goal_handle.abort()
                        result.error_code = (
                            FollowJointTrajectory.Result
                            .GOAL_TOLERANCE_VIOLATED
                        )
                        result.error_string = (
                            '관절 오차가 충분히 감소하지 않아 재전송을 '
                            f'중단합니다: {diagnostic}'
                        )
                        self.get_logger().error(result.error_string)
                        return result

                    previous_attempt_error = maximum_error
                    next_command_deg = list(target_deg)
                    if self._joint_retry_compensation_enabled:
                        actual_deg = [
                            math.degrees(value) for value in actual
                        ]
                        (
                            next_command_deg,
                            correction_deg,
                            total_offset_deg,
                        ) = compensated_joint_command_degrees(
                            target_deg,
                            last_command_deg,
                            actual_deg,
                            gain=self._joint_retry_compensation_gain,
                            maximum_step_deg=self._joint_retry_max_step_deg,
                            maximum_total_offset_deg=(
                                self._joint_retry_max_total_offset_deg
                            ),
                        )
                        self.get_logger().warning(
                            '관절 오차 보상 명령 계산: '
                            f'correction_deg={correction_deg}, '
                            f'total_offset_deg={total_offset_deg}, '
                            f'command_deg={next_command_deg}'
                        )
                    self.get_logger().warning(
                        f'관절 목표 재전송 전 상태: {diagnostic}'
                    )
                    try:
                        self._send_joint_target(
                            next_command_deg,
                            attempts_sent + 1,
                        )
                    except Exception as error:
                        self._stop_robot()
                        goal_handle.abort()
                        result.error_code = (
                            FollowJointTrajectory.Result.INVALID_GOAL
                        )
                        result.error_string = (
                            f'관절 목표 재전송 실패: {error}'
                        )
                        return result
                    last_command_deg = list(next_command_deg)
                    attempts_sent += 1
                    retry_stable_samples = 0
                    previous_actual = None

            if time.monotonic() - started > timeout:
                self._stop_robot()
                goal_handle.abort()
                result.error_code = FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
                detail = (
                    '실제 관절을 읽지 못했습니다'
                    if actual is None
                    else self._joint_diagnostic(target_rad, actual)
                )
                result.error_string = (
                    f"{timeout:.1f}초 안에 목표 자세에 도달하지 "
                    f"못했습니다: {detail}"
                )
                self.get_logger().error(result.error_string)
                return result
            time.sleep(0.1)

        self._stop_robot()
        goal_handle.abort()
        result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
        result.error_string = "ROS가 종료되었습니다"
        return result

    def _send_joint_target(
        self,
        target_deg: Sequence[float],
        attempt: int,
    ) -> None:
        """큐를 안전 초기화하고 관절 목표를 한 번 전송하며 시간을 기록한다."""
        started = time.monotonic()
        self.get_logger().info(
            f'MyCobot 목표 전송 {attempt}/'
            f'{self._joint_max_command_attempts} [deg]: {list(target_deg)}'
        )
        with self._serial_lock:
            prepare_command_queue(self._robot)
            queue_ready_at = time.monotonic()
            response = self._robot.send_angles(
                list(target_deg),
                self._speed,
            )
        finished = time.monotonic()
        require_no_explicit_command_failure('send_angles', response)
        self.get_logger().info(
            'MyCobot 명령 호출 완료: '
            f'queue_prepare={queue_ready_at - started:.3f}s, '
            f'send_angles={finished - queue_ready_at:.3f}s, '
            f'response={response!r}'
        )

    @staticmethod
    def _joint_diagnostic(
        target_rad: Sequence[float],
        actual_rad: Sequence[float],
    ) -> str:
        """목표·실제·관절별 오차를 degree 문자열로 만든다."""
        target_deg = radians_to_degrees(target_rad)
        actual_deg = radians_to_degrees(actual_rad)
        errors_deg = [
            round(value, 3)
            for value in signed_joint_errors_degrees(
                target_rad,
                actual_rad,
            )
        ]
        maximum_error_deg = max(abs(value) for value in errors_deg)
        return (
            f'target_deg={target_deg}, actual_deg={actual_deg}, '
            f'error_deg={errors_deg}, max_error={maximum_error_deg:.3f}deg'
        )

    def _robot_is_moving(self) -> bool:
        is_moving = getattr(self._robot, "is_moving", None)
        if not callable(is_moving):
            raise RuntimeError("pymycobot is_moving() API가 없습니다")
        with self._serial_lock:
            state = is_moving()
        if state in (False, 0):
            return False
        if state in (True, 1):
            return True
        raise RuntimeError(f"is_moving() 응답이 유효하지 않습니다: {state!r}")

    def _verify_joint_hold(self, target_rad: Sequence[float]) -> bool:
        deadline = time.monotonic() + self._joint_hold_check_seconds
        while rclpy.ok() and time.monotonic() < deadline:
            actual = self._read_and_publish_state()
            if actual is None:
                return False
            if maximum_joint_error(target_rad, actual) > self._tolerance_rad:
                self._stop_robot()
                return False
            if self._robot_is_moving():
                self._stop_robot()
                return False
            time.sleep(0.1)
        return rclpy.ok()

    def _cartesian_goal_callback(self, goal_request: CartesianMove.Goal) -> GoalResponse:
        try:
            if not self._cartesian_execution_enabled:
                raise ValueError("cartesian_execution_enabled=false")
            if not self._cartesian_ready:
                raise ValueError(
                    "로봇 Cartesian API와 base/flange 좌표계가 확인되지 않았습니다"
                )
            if goal_request.target.header.frame_id != self._cartesian_base_frame:
                raise ValueError(
                    f"목표 frame은 {self._cartesian_base_frame}여야 합니다"
                )
            if not 1 <= int(goal_request.speed) <= 100:
                raise ValueError("Cartesian speed는 1~100이어야 합니다")
            if int(goal_request.mode) not in (0, 1):
                raise ValueError("Cartesian mode는 0 또는 1이어야 합니다")
            self._pose_message_to_coords(goal_request.target)
        except ValueError as error:
            self.get_logger().error(f"Cartesian 목표 거부: {error}")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _execute_cartesian(self, goal_handle: object) -> CartesianMove.Result:
        result = CartesianMove.Result()
        if not self._motion_lock.acquire(blocking=False):
            result.success = False
            result.message = "다른 로봇 이동이 실행 중입니다"
            goal_handle.abort()
            return result
        try:
            return self._execute_cartesian_locked(goal_handle)
        finally:
            self._motion_lock.release()

    def _execute_cartesian_locked(self, goal_handle: object) -> CartesianMove.Result:
        request = goal_handle.request
        result = CartesianMove.Result()
        requested_coords = self._pose_message_to_coords(request.target)
        try:
            start_coords = self._read_robot_coords()
            start_pose = self._pose_from_coords(start_coords)
            target_coords = apply_cartesian_locks(
                requested_coords,
                start_coords,
                lock_z=bool(request.lock_z),
                lock_roll_pitch=bool(request.lock_roll_pitch),
            )
            target_pose = self._pose_from_coords(target_coords)
            target_position, target_quaternion = self._pose_values(target_pose)
            start_position, start_quaternion = self._pose_values(start_pose)
            distance, rotation = pose_error(
                target_position,
                target_quaternion,
                start_position,
                start_quaternion,
            )
            if distance > self._cartesian_max_translation:
                raise ValueError(
                    f"Cartesian 이동량 {distance * 1000.0:.3f}mm가 "
                    "제한 "
                    f"{self._cartesian_max_translation * 1000.0:.3f}mm를 초과합니다"
                )
            if rotation > self._cartesian_max_rotation:
                raise ValueError(
                    f"Cartesian 회전량 {rotation:.3f}deg가 "
                    f"제한 {self._cartesian_max_rotation:.3f}deg를 초과합니다"
                )
            self.get_logger().warning(
                "Cartesian 목표 전송 [mm, deg]: "
                f"{[round(value, 3) for value in target_coords]}, "
                f"speed={request.speed}, mode={request.mode}"
            )
            with self._serial_lock:
                prepare_command_queue(self._robot)
                response = self._robot.send_coords(
                    target_coords, int(request.speed), int(request.mode)
                )
            require_no_explicit_command_failure('send_coords', response)
            self.get_logger().info(
                'send_coords 호출 종료: 실제 Cartesian 상태 감시를 시작합니다'
            )
        except Exception as error:
            result.success = False
            result.message = f"Cartesian 명령 실패: {error}"
            goal_handle.abort()
            return result

        planned_xy = (
            (target_coords[0] - start_coords[0]) / 1000.0,
            (target_coords[1] - start_coords[1]) / 1000.0,
        )
        planned_distance = math.hypot(*planned_xy)
        planned_z_m = (target_coords[2] - start_coords[2]) / 1000.0
        planned_yaw_deg = wrapped_angle_difference_deg(
            target_coords[5],
            start_coords[5],
        )
        free_z_only_command = (
            self._cartesian_ignore_z_tracking_error
            and is_z_dominant_recovery_motion(
                planned_distance,
                planned_z_m,
                planned_yaw_deg,
                self._cartesian_position_tolerance,
                self._cartesian_orientation_tolerance,
                self._cartesian_free_z_minimum_motion,
            )
        )
        started = time.monotonic()
        next_progress_log = started
        stationary_samples = 0
        motion_observed = False
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self._stop_robot()
                result.success = False
                result.message = "사용자가 Cartesian 이동을 취소했습니다"
                goal_handle.canceled()
                return result
            try:
                actual_coords = self._read_robot_coords()
                actual = self._pose_from_coords(actual_coords)
                actual_position, actual_quaternion = self._pose_values(actual)
                position_error_m, orientation_error_deg = pose_error(
                    target_position,
                    target_quaternion,
                    actual_position,
                    actual_quaternion,
                )
                if self._cartesian_ignore_z_tracking_error:
                    position_error_m = math.hypot(
                        target_position[0] - actual_position[0],
                        target_position[1] - actual_position[1],
                    )
                self._validate_locked_path(request, start_coords, actual_coords)
                robot_is_moving = self._robot_is_moving()
                moved_position_m, moved_orientation_deg = pose_error(
                    actual_position,
                    actual_quaternion,
                    start_position,
                    start_quaternion,
                )
                motion_observed = motion_observed or (
                    robot_is_moving
                    or moved_position_m >= 0.00025
                    or moved_orientation_deg >= 0.25
                )
            except Exception as error:
                self._stop_robot()
                result.success = False
                result.message = f"Cartesian 실행 감시 실패: {error}"
                goal_handle.abort()
                return result

            feedback = CartesianMove.Feedback()
            feedback.actual = actual
            feedback.position_error_m = position_error_m
            feedback.orientation_error_deg = orientation_error_deg
            goal_handle.publish_feedback(feedback)
            now = time.monotonic()
            if now >= next_progress_log:
                self.get_logger().info(
                    'Cartesian 진행 상태: '
                    f'moving={robot_is_moving}, '
                    f'xy_residual={position_error_m * 1000.0:.3f}mm, '
                    f'orientation_residual={orientation_error_deg:.3f}deg, '
                    'actual=[mm, deg] '
                    f'{[round(value, 3) for value in actual_coords]}'
                )
                next_progress_log = now + self._cartesian_progress_log_seconds
            if (
                not motion_observed
                and now - started
                >= self._cartesian_no_motion_timeout
            ):
                diagnostic = self._robot_error_diagnostic()
                self._stop_robot()
                result.success = False
                result.message = (
                    'Cartesian 명령 후 로봇 무동작: '
                    f'{self._cartesian_no_motion_timeout:.1f}초 동안 '
                    f'0.25mm/0.25deg 이상의 변화가 없습니다; {diagnostic}'
                )
                result.actual = actual
                goal_handle.abort()
                self.get_logger().error(f'[CARTESIAN-DIAG] {result.message}')
                return result
            if (
                position_error_m <= self._cartesian_position_tolerance
                and orientation_error_deg <= self._cartesian_orientation_tolerance
                and not free_z_only_command
            ):
                if not self._stop_robot():
                    result.success = False
                    result.message = (
                        'Cartesian 목표 도달 후 정지·큐 삭제에 실패했습니다'
                    )
                    result.actual = actual
                    goal_handle.abort()
                    return result
                result.success = True
                result.message = (
                    'Cartesian 목표 자세 도달: '
                    f'xy_residual={position_error_m * 1000.0:.3f}mm, '
                    f'orientation_residual={orientation_error_deg:.3f}deg, '
                    'actual=[mm, deg] '
                    f'{[round(value, 3) for value in actual_coords]}'
                )
                result.actual = actual
                goal_handle.succeed()
                self.get_logger().info(result.message)
                return result
            if self._cartesian_ignore_z_tracking_error:
                actual_xy = (
                    (actual_coords[0] - start_coords[0]) / 1000.0,
                    (actual_coords[1] - start_coords[1]) / 1000.0,
                )
                directional_progress = (
                    0.0
                    if planned_distance < 1e-9
                    else (
                        actual_xy[0] * planned_xy[0]
                        + actual_xy[1] * planned_xy[1]
                    )
                    / planned_distance
                )
                actual_yaw_deg = wrapped_angle_difference_deg(
                    actual_coords[5],
                    start_coords[5],
                )
                directional_yaw_progress_deg = (
                    0.0
                    if abs(planned_yaw_deg) < 1e-9
                    else actual_yaw_deg
                    * math.copysign(1.0, planned_yaw_deg)
                )
                actual_z_m = (actual_coords[2] - start_coords[2]) / 1000.0
                directional_z_progress_m = (
                    0.0
                    if abs(planned_z_m) < 1e-9
                    else actual_z_m * math.copysign(1.0, planned_z_m)
                )
                if free_z_only_command:
                    # TF 목표와 get_coords 시작값 사이에 작은 XY 차이가 있어도
                    # Z 복구 완료는 실제 Z 방향 진행량으로 판정한다.
                    sufficient_directional_progress = (
                        directional_z_progress_m >= 0.00025
                    )
                else:
                    sufficient_directional_progress = (
                        directional_progress >= 0.00025
                        if planned_distance >= 1e-9
                        else (
                            directional_yaw_progress_deg >= 0.25
                            if abs(planned_yaw_deg) >= 1e-9
                            else directional_z_progress_m >= 0.00025
                        )
                    )
                stationary_samples = (
                    stationary_samples + 1
                    if (
                        free_z_only_command
                        and not robot_is_moving
                        and time.monotonic() - started >= 0.5
                        and sufficient_directional_progress
                    )
                    else 0
                )
                if free_z_only_command and stationary_samples >= 3:
                    if not self._stop_robot():
                        result.success = False
                        result.message = (
                            'Cartesian 정지 위치 수용 전 큐 정리에 실패했습니다'
                        )
                        result.actual = actual
                        goal_handle.abort()
                        return result
                    result.success = True
                    result.message = (
                        'free-Z stop-and-go 정지 위치 수용: '
                        f'xy_residual={position_error_m * 1000.0:.3f}mm, '
                        f'directional_progress='
                        f'{directional_progress * 1000.0:.3f}mm, '
                        f'directional_yaw_progress='
                        f'{directional_yaw_progress_deg:.3f}deg, '
                        f'directional_z_progress='
                        f'{directional_z_progress_m * 1000.0:.3f}mm'
                    )
                    result.actual = actual
                    goal_handle.succeed()
                    self.get_logger().warning(result.message)
                    return result
            if now - started > self._cartesian_timeout:
                self._stop_robot()
                result.success = False
                result.message = (
                    f"{self._cartesian_timeout:.1f}초 안에 Cartesian 목표에 "
                    '도달하지 못했습니다: '
                    f'xy_residual={position_error_m * 1000.0:.3f}mm, '
                    f'orientation_residual={orientation_error_deg:.3f}deg, '
                    'actual=[mm, deg] '
                    f'{[round(value, 3) for value in actual_coords]}'
                )
                result.actual = actual
                goal_handle.abort()
                return result
            time.sleep(0.1)

        self._stop_robot()
        result.success = False
        result.message = "ROS가 종료되었습니다"
        goal_handle.abort()
        return result

    def _validate_locked_path(
        self, request, start: list[float], actual: list[float]
    ) -> None:
        if request.lock_z and not self._cartesian_ignore_z_tracking_error:
            z_error_m = abs(actual[2] - start[2]) / 1000.0
            if z_error_m > self._cartesian_path_z_tolerance:
                raise ValueError(
                    f"고정 Z 이탈 {z_error_m * 1000.0:.3f}mm가 "
                    "허용값을 초과했습니다"
                )
        if request.lock_roll_pitch:
            roll_error = abs(wrapped_angle_difference_deg(actual[3], start[3]))
            pitch_error = abs(wrapped_angle_difference_deg(actual[4], start[4]))
            if max(roll_error, pitch_error) > self._cartesian_path_tilt_tolerance:
                raise ValueError(
                    "고정 Roll/Pitch 이탈이 허용값을 초과했습니다: "
                    f"roll={roll_error:.3f}deg pitch={pitch_error:.3f}deg"
                )

    def _read_robot_coords(self) -> list[float]:
        last_error: Exception | None = None
        for _ in range(5):
            try:
                with self._serial_lock:
                    coords = self._robot.get_coords()
                position, quaternion = robot_coords_to_pose_values(coords)
                return pose_values_to_robot_coords(position, quaternion)
            except Exception as error:
                last_error = error
                time.sleep(0.1)
        raise RuntimeError(f'get_coords 5회 실패: {last_error}')

    def _pose_message_to_coords(self, message: PoseStamped) -> list[float]:
        position, quaternion = self._pose_values(message)
        return pose_values_to_robot_coords(position, quaternion)

    @staticmethod
    def _pose_values(
        message: PoseStamped,
    ) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
        pose = message.pose
        return (
            (pose.position.x, pose.position.y, pose.position.z),
            (
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ),
        )

    def _pose_from_coords(self, coords: Sequence[float]) -> PoseStamped:
        position, quaternion = robot_coords_to_pose_values(coords)
        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self._cartesian_base_frame
        (
            message.pose.position.x,
            message.pose.position.y,
            message.pose.position.z,
        ) = position
        (
            message.pose.orientation.x,
            message.pose.orientation.y,
            message.pose.orientation.z,
            message.pose.orientation.w,
        ) = quaternion
        return message

    def _read_and_publish_state(self) -> list[float] | None:
        try:
            with self._serial_lock:
                positions = angles_deg_to_rad(self._robot.get_angles())
        except Exception as error:
            self.get_logger().warning(
                f"get_angles 실패: {error}", throttle_duration_sec=2.0
            )
            return None

        self._latest_positions = positions
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "base_link"
        message.name = list(JOINT_NAMES)
        message.position = positions
        self._joint_publisher.publish(message)
        return positions

    def _read_and_publish_cartesian_pose(self) -> None:
        """정지 중 제조사 get_coords pose를 저주파 ROS topic으로 발행한다."""
        if self._motion_lock.locked() or not self._cartesian_ready:
            return
        try:
            coords = self._read_robot_coords()
        except Exception as error:
            self.get_logger().warning(
                f"get_coords pose 발행 실패: {error}",
                throttle_duration_sec=2.0,
            )
            return
        self._cartesian_pose_publisher.publish(self._pose_from_coords(coords))

    def _stop_robot(self) -> bool:
        try:
            with self._serial_lock:
                stop_and_clear_command_queue(self._robot)
        except Exception as error:
            self.get_logger().error(f"로봇 정지·큐 삭제 실패: {error}")
            return False
        return True

    def close(self) -> None:
        self._stop_robot()
        self._action_server.destroy()
        self._cartesian_action_server.destroy()
        close = getattr(self._robot, "close", None)
        if callable(close):
            close()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node: MyCobotTrajectoryBridge | None = None
    executor = MultiThreadedExecutor(num_threads=3)
    try:
        node = MyCobotTrajectoryBridge()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
