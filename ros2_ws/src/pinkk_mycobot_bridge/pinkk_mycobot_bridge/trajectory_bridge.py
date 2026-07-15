"""MyCobot280용 단순 FollowJointTrajectory 실행 및 관절 상태 브리지."""

from __future__ import annotations

import math
import threading
import time
from pathlib import Path
from typing import Sequence

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

from .joint_state_publisher import JOINT_NAMES, angles_deg_to_rad


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

    def __init__(self) -> None:
        super().__init__("pinkk_mycobot_trajectory_bridge")
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baud", 1_000_000)
        self.declare_parameter("speed", 10)
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("goal_tolerance_deg", 2.0)
        self.declare_parameter("max_execution_seconds", 60.0)

        port = str(self.get_parameter("port").value)
        baud = int(self.get_parameter("baud").value)
        self._speed = int(self.get_parameter("speed").value)
        rate = float(self.get_parameter("publish_rate_hz").value)
        self._tolerance_rad = math.radians(
            float(self.get_parameter("goal_tolerance_deg").value)
        )
        self._max_execution_seconds = float(
            self.get_parameter("max_execution_seconds").value
        )
        if not Path(port).exists():
            raise FileNotFoundError(f"로봇 serial port가 없습니다: {port}")
        if not 1 <= self._speed <= 100:
            raise ValueError("speed는 1~100이어야 합니다")
        if rate <= 0.0 or rate > 50.0:
            raise ValueError("publish_rate_hz는 0보다 크고 50 이하여야 합니다")
        if self._tolerance_rad <= 0.0:
            raise ValueError("goal_tolerance_deg는 0보다 커야 합니다")
        if self._max_execution_seconds <= 0.0:
            raise ValueError("max_execution_seconds는 0보다 커야 합니다")

        try:
            from pymycobot import MyCobot280
        except ImportError as error:
            raise RuntimeError("pymycobot을 찾을 수 없습니다") from error

        self._serial_lock = threading.Lock()
        self._robot = MyCobot280(port, baud)
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._joint_publisher = self.create_publisher(JointState, "/joint_states", qos)
        self._latest_positions: list[float] | None = None
        self._state_timer = self.create_timer(1.0 / rate, self._read_and_publish_state)
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            self.ACTION_NAME,
            execute_callback=self._execute,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=ReentrantCallbackGroup(),
        )
        self.get_logger().info(
            f"실제 실행 브리지 준비: port={port}, baud={baud}, speed={self._speed}, "
            f"action={self.ACTION_NAME}"
        )

    def _goal_callback(self, goal_request: FollowJointTrajectory.Goal) -> GoalResponse:
        try:
            validate_trajectory(goal_request.trajectory.joint_names, goal_request.trajectory.points)
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
        trajectory = goal_handle.request.trajectory
        target_rad = validate_trajectory(trajectory.joint_names, trajectory.points)
        target_deg = radians_to_degrees(target_rad)
        planned_seconds = duration_seconds(trajectory.points[-1].time_from_start)
        timeout = min(
            self._max_execution_seconds,
            max(10.0, planned_seconds + 8.0),
        )
        result = FollowJointTrajectory.Result()

        try:
            self.get_logger().info(f"MyCobot 목표 전송 [deg]: {target_deg}")
            with self._serial_lock:
                self._robot.send_angles(target_deg, self._speed)
        except Exception as error:
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = f"send_angles 실패: {error}"
            goal_handle.abort()
            return result

        started = time.monotonic()
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
                feedback = FollowJointTrajectory.Feedback()
                feedback.header.stamp = self.get_clock().now().to_msg()
                feedback.joint_names = list(JOINT_NAMES)
                feedback.desired = JointTrajectoryPoint(positions=list(target_rad))
                feedback.actual = JointTrajectoryPoint(positions=list(actual))
                feedback.error = JointTrajectoryPoint(positions=errors)
                goal_handle.publish_feedback(feedback)
                if max(abs(value) for value in errors) <= self._tolerance_rad:
                    goal_handle.succeed()
                    result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                    result.error_string = "목표 관절 자세 도달"
                    self.get_logger().info("trajectory 실행 완료")
                    return result

            if time.monotonic() - started > timeout:
                self._stop_robot()
                goal_handle.abort()
                result.error_code = FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
                result.error_string = f"{timeout:.1f}초 안에 목표 자세에 도달하지 못했습니다"
                self.get_logger().error(result.error_string)
                return result
            time.sleep(0.1)

        self._stop_robot()
        goal_handle.abort()
        result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
        result.error_string = "ROS가 종료되었습니다"
        return result

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

    def _stop_robot(self) -> None:
        stop = getattr(self._robot, "stop", None)
        if not callable(stop):
            return
        try:
            with self._serial_lock:
                stop()
        except Exception as error:
            self.get_logger().error(f"로봇 정지 명령 실패: {error}")

    def close(self) -> None:
        self._action_server.destroy()
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
