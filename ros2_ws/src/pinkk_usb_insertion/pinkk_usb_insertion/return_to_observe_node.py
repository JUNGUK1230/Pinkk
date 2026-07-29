"""통합 trajectory bridge를 통해 초기 관측 관절 자세로 한 번 복귀한다."""

from __future__ import annotations

import math
import time

from control_msgs.action import FollowJointTrajectory
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint


JOINT_NAMES = (
    'joint2_to_joint1',
    'joint3_to_joint2',
    'joint4_to_joint3',
    'joint5_to_joint4',
    'joint6_to_joint5',
    'joint6output_to_joint6',
)
ACTION_NAME = '/arm_group_controller/follow_joint_trajectory'


class ReturnToObserveNode(Node):
    """YAML의 관측 관절각을 기존 제조사 bridge에 전송한다."""

    def __init__(self) -> None:
        super().__init__('pinkk_return_to_observe')
        self.declare_parameter(
            'observation_joint_degrees',
            [-1.66, -8.08, -36.65, -39.9, 0.0, 45.0],
        )
        self.declare_parameter('motion_seconds', 20.0)
        self.declare_parameter('warning_delay_seconds', 3.0)
        self.declare_parameter('accepted_error_deg', 3.5)
        target_deg = [
            float(value)
            for value in self.get_parameter(
                'observation_joint_degrees'
            ).value
        ]
        self._motion_seconds = float(
            self.get_parameter('motion_seconds').value
        )
        self._warning_delay = float(
            self.get_parameter('warning_delay_seconds').value
        )
        self._accepted_error = float(
            self.get_parameter('accepted_error_deg').value
        )
        if len(target_deg) != 6 or not all(
            math.isfinite(value) for value in target_deg
        ):
            raise ValueError('observation_joint_degrees는 유한한 6개 값이어야 합니다')
        if not 1.0 <= self._motion_seconds <= 60.0:
            raise ValueError('motion_seconds는 1~60초여야 합니다')
        if not 0.0 <= self._warning_delay <= 10.0:
            raise ValueError('warning_delay_seconds는 0~10초여야 합니다')
        if not 0.1 <= self._accepted_error <= 5.0:
            raise ValueError('accepted_error_deg는 0.1~5도여야 합니다')
        self._target_rad = [math.radians(value) for value in target_deg]
        self._latest_joints: list[float] | None = None
        self._joint_received_at: float | None = None
        self._action = ActionClient(
            self,
            FollowJointTrajectory,
            ACTION_NAME,
        )
        self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_callback,
            10,
        )
        self.get_logger().info(f'초기 관측 목표 [deg]: {target_deg}')

    def _joint_callback(self, message: JointState) -> None:
        values = dict(zip(message.name, message.position))
        if all(name in values for name in JOINT_NAMES):
            self._latest_joints = [
                float(values[name]) for name in JOINT_NAMES
            ]
            self._joint_received_at = time.monotonic()

    def _spin_until(self, future, timeout_seconds: float):
        rclpy.spin_until_future_complete(
            self,
            future,
            timeout_sec=timeout_seconds,
        )
        if not future.done():
            raise TimeoutError('ROS action 응답 시간이 초과됐습니다')
        if future.exception() is not None:
            raise RuntimeError(str(future.exception()))
        return future.result()

    def _actual_error_deg(self) -> float | None:
        deadline = time.monotonic() + 2.0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if (
                self._latest_joints is not None
                and self._joint_received_at is not None
                and time.monotonic() - self._joint_received_at <= 0.5
            ):
                return max(
                    abs(
                        math.degrees(
                            math.remainder(target - actual, 2.0 * math.pi)
                        )
                    )
                    for target, actual in zip(
                        self._target_rad,
                        self._latest_joints,
                        strict=True,
                    )
                )
        return None

    def execute(self) -> None:
        if not self._action.wait_for_server(timeout_sec=20.0):
            raise RuntimeError(
                '통합 trajectory bridge action이 없습니다: '
                f'{ACTION_NAME}'
            )
        self.get_logger().warning(
            f'{self._warning_delay:.1f}초 후 초기 관측 자세로 복귀합니다'
        )
        time.sleep(self._warning_delay)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINT_NAMES)
        point = JointTrajectoryPoint()
        point.positions = list(self._target_rad)
        point.time_from_start = Duration(
            seconds=self._motion_seconds
        ).to_msg()
        goal.trajectory.points = [point]
        handle = self._spin_until(
            self._action.send_goal_async(goal),
            5.0,
        )
        if not handle.accepted:
            raise RuntimeError(
                '관측 자세 목표가 거절됐습니다. bridge의 '
                'joint_execution_enabled를 확인하세요'
            )
        result_future = handle.get_result_async()
        try:
            wrapped = self._spin_until(
                result_future,
                self._motion_seconds + 50.0,
            )
        except TimeoutError:
            self._spin_until(handle.cancel_goal_async(), 3.0)
            raise
        actual_error = self._actual_error_deg()
        if actual_error is None:
            raise RuntimeError('이동 후 fresh /joint_states가 없습니다')
        if actual_error > self._accepted_error:
            detail = wrapped.result.error_string or wrapped.result.error_code
            raise RuntimeError(
                f'관측 자세 오차 {actual_error:.3f}deg가 허용값 '
                f'{self._accepted_error:.3f}deg를 초과했습니다: {detail}'
            )
        if wrapped.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().warning(
                'bridge의 엄격한 목표 판정은 실패했지만 관측 자세 허용범위 '
                f'안이므로 수용합니다: max_error={actual_error:.3f}deg'
            )
        self.get_logger().info(
            '초기 관측 자세 복귀 완료: '
            f'max_error={actual_error:.3f}deg'
        )


def main(args: list[str] | None = None) -> int:
    rclpy.init(args=args)
    node = ReturnToObserveNode()
    result = 0
    try:
        node.execute()
    except KeyboardInterrupt:
        result = 130
    except Exception as error:
        node.get_logger().error(f'초기 관측 자세 복귀 실패: {error}')
        result = 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return result
