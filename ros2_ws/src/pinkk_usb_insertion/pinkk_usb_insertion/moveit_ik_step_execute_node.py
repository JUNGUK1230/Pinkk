"""검증된 MoveIt IK 관절 목표를 명시적 승인 후 한 번만 실행한다."""

from __future__ import annotations

import argparse
import math
import sys
import time

from control_msgs.action import FollowJointTrajectory

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration

from trajectory_msgs.msg import JointTrajectoryPoint

from .control.moveit_ik_safety import validate_fk_pose
from .control.pbvs_step_safety import validate_joint_step
from .moveit_ik_step_executor_node import JOINT_NAMES, MoveItIkDryRunNode


ACTION_NAME = '/arm_group_controller/follow_joint_trajectory'


class MoveItIkStepExecuteNode(MoveItIkDryRunNode):
    """1mm 이하의 검증된 IK 목표 한 개만 관절 action으로 실행한다."""

    def __init__(self) -> None:
        """DRY RUN 계산 기능에 관절 trajectory client만 추가한다."""
        super().__init__('pinkk_moveit_ik_step_execute')
        self._action = ActionClient(self, FollowJointTrajectory, ACTION_NAME)

    def execute(
        self,
        axis: str,
        distance_m: float,
        move_seconds: float,
    ) -> None:
        """계획을 다시 계산한 뒤 3초 경고 후 관절 목표를 한 번 전송한다."""
        if abs(distance_m) > 0.001 + 1e-12:
            raise ValueError(
                '첫 실기 시험은 절대 이동량 1.000mm 이하만 허용합니다'
            )
        plan = self.calculate_plan(
            axis=axis,
            distance_m=distance_m,
            maximum_distance_m=0.001,
            waypoint_spacing_m=0.001,
            maximum_joint_step_deg=1.0,
        )
        total_joint_change = validate_joint_step(
            plan.start_joints,
            plan.target_joints,
            maximum_joint_step_deg=1.0,
        )
        if not self._action.wait_for_server(timeout_sec=5.0):
            raise RuntimeError(f'관절 action server가 없습니다: {ACTION_NAME}')

        start_deg = [
            round(math.degrees(value), 3)
            for value in plan.start_joints
        ]
        target_deg = [
            round(math.degrees(value), 3)
            for value in plan.target_joints
        ]
        self.get_logger().warning(
            '실제 로봇 1회 이동 승인됨\n'
            f'  요청: base {axis.upper()} {distance_m * 1000.0:+.3f}mm\n'
            f'  시작 관절 [deg]: {start_deg}\n'
            f'  목표 관절 [deg]: {target_deg}\n'
            f'  최대 총 관절 변화: {total_joint_change:.3f}deg\n'
            '  3초 후 한 개의 FollowJointTrajectory 목표를 전송합니다'
        )
        time.sleep(3.0)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINT_NAMES)
        point = JointTrajectoryPoint()
        point.positions = list(plan.target_joints)
        point.time_from_start = Duration(seconds=move_seconds).to_msg()
        goal.trajectory.points = [point]
        handle = self._wait_future(self._action.send_goal_async(goal), 5.0)
        if not handle.accepted:
            raise RuntimeError(
                'trajectory 목표가 거절됐습니다. 로봇 PC의 '
                'joint_execution_enabled 설정을 확인하세요'
            )
        result_future = handle.get_result_async()
        try:
            wrapped = self._wait_future(
                result_future,
                move_seconds + 50.0,
            )
        except TimeoutError as error:
            self.get_logger().error(
                'trajectory 결과 timeout: 안전 취소를 요청합니다'
            )
            try:
                cancel_response = self._wait_future(
                    handle.cancel_goal_async(),
                    3.0,
                )
                if not cancel_response.goals_canceling:
                    raise RuntimeError('bridge가 action 취소를 수락하지 않았습니다')
            except Exception as cancel_error:
                raise RuntimeError(
                    'trajectory 결과 timeout 후 취소 확인에도 실패했습니다. '
                    '로봇 PC bridge를 즉시 종료하세요: '
                    f'{cancel_error}'
                ) from error
            raise RuntimeError(
                'trajectory 결과 timeout으로 action 취소를 요청했습니다'
            ) from error
        if (
            wrapped.result.error_code
            != FollowJointTrajectory.Result.SUCCESSFUL
        ):
            raise RuntimeError(
                f'trajectory 실행 실패: code={wrapped.result.error_code}, '
                f'message={wrapped.result.error_string}'
            )

        actual = self.read_current_transform(timeout_seconds=3.0)
        error = validate_fk_pose(
            plan.target_transform,
            actual,
            maximum_position_error_m=0.00075,
            maximum_z_error_m=0.001,
            maximum_orientation_error_deg=1.0,
        )
        actual_dx = (
            actual[0, 3] - plan.current_transform[0, 3]
        ) * 1000.0
        actual_dy = (
            actual[1, 3] - plan.current_transform[1, 3]
        ) * 1000.0
        actual_dz = (
            actual[2, 3] - plan.current_transform[2, 3]
        ) * 1000.0
        actual_axis_move = actual_dx if axis == 'x' else actual_dy
        if actual_axis_move * distance_m <= 0.0:
            raise RuntimeError('실제 이동 방향이 요청 방향과 반대입니다')
        if abs(actual_axis_move) < 0.25:
            raise RuntimeError(
                f'요청 축 실제 이동이 너무 작습니다: {actual_axis_move:+.3f}mm'
            )
        self.get_logger().info(
            '실제 1회 이동 및 이동 후 TF 검사 통과\n'
            f'  측정 이동: dx={actual_dx:+.3f}mm, '
            f'dy={actual_dy:+.3f}mm, dz={actual_dz:+.3f}mm\n'
            f'  목표 대비 위치 오차: '
            f'{error.position_error_m * 1000.0:.3f}mm\n'
            f'  목표 대비 Z 오차: {error.z_error_m * 1000.0:+.3f}mm\n'
            f'  목표 대비 자세 오차: {error.orientation_error_deg:.3f}deg\n'
            '  자동 복귀하지 않습니다'
        )


def _parse_arguments(arguments: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='MoveIt IK 고정-Z 1mm 목표를 실제 로봇에 한 번 전송합니다'
    )
    parser.add_argument('--axis', choices=('x', 'y'), required=True)
    parser.add_argument('--distance-mm', type=float, required=True)
    parser.add_argument('--move-seconds', type=float, default=5.0)
    parser.add_argument(
        '--execute',
        action='store_true',
        help='이 옵션이 있어야 실제 관절 목표를 전송합니다',
    )
    parsed = parser.parse_args(arguments)
    if not parsed.execute:
        parser.error('실제 실행에는 --execute를 명시해야 합니다')
    if parsed.move_seconds < 2.0 or parsed.move_seconds > 10.0:
        parser.error('--move-seconds는 2~10초여야 합니다')
    return parsed


def main(args: list[str] | None = None) -> None:
    """명시적 실행 승인을 확인하고 실제 1mm 단발 시험을 수행한다."""
    raw_arguments = sys.argv if args is None else [sys.argv[0], *args]
    cli = _parse_arguments(rclpy.utilities.remove_ros_args(raw_arguments)[1:])
    rclpy.init(args=[])
    node = MoveItIkStepExecuteNode()
    exit_code = 0
    try:
        node.execute(
            axis=cli.axis,
            distance_m=cli.distance_mm / 1000.0,
            move_seconds=cli.move_seconds,
        )
    except Exception as error:
        node.get_logger().error(f'MoveIt IK 실제 단발 실행 실패: {error}')
        exit_code = 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if exit_code:
        raise SystemExit(exit_code)
