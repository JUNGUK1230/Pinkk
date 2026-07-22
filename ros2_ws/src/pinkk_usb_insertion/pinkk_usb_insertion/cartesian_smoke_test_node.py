"""현재 flange pose에서 X 또는 Y만 이동하는 승인형 Cartesian 시험 도구."""

from __future__ import annotations

import argparse
from copy import deepcopy
import sys
import time

from geometry_msgs.msg import PoseStamped
from pinkk_usb_insertion_interfaces.action import CartesianMove
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


ACTION_NAME = '/robot_arm/cartesian_move'


def make_xy_offset_target(
    current: PoseStamped, axis: str, distance_mm: float
) -> PoseStamped:
    """현재 pose의 Z와 자세를 복사하고 X 또는 Y만 제한 범위에서 변경한다."""
    normalized_axis = axis.strip().lower()
    distance = float(distance_mm)
    if normalized_axis not in ('x', 'y'):
        raise ValueError('시험 축은 x 또는 y만 허용합니다')
    if not 0.1 <= abs(distance) <= 10.0:
        raise ValueError('시험 거리는 절댓값 0.1~10.0mm만 허용합니다')
    target = deepcopy(current)
    delta_m = distance / 1000.0
    if normalized_axis == 'x':
        target.pose.position.x += delta_m
    else:
        target.pose.position.y += delta_m
    return target


def _pose_text(label: str, message: PoseStamped) -> str:
    pose = message.pose
    return (
        f'{label}: frame={message.header.frame_id} '
        f'xyz_m=[{pose.position.x:+.6f}, {pose.position.y:+.6f}, '
        f'{pose.position.z:+.6f}] q_xyzw=['
        f'{pose.orientation.x:+.6f}, {pose.orientation.y:+.6f}, '
        f'{pose.orientation.z:+.6f}, {pose.orientation.w:+.6f}]'
    )


class CartesianSmokeTestNode(Node):
    """DRY RUN을 기본으로 하는 X/Y Cartesian action 시험 클라이언트."""

    def __init__(self, base_frame: str, flange_frame: str) -> None:
        super().__init__('pinkk_cartesian_smoke_test')
        self._base_frame = base_frame
        self._flange_frame = flange_frame
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._action = ActionClient(self, CartesianMove, ACTION_NAME)

    def current_pose(self, timeout_seconds: float = 5.0) -> PoseStamped:
        """최신 base-to-flange TF를 기다려 PoseStamped로 반환한다."""
        deadline = time.monotonic() + timeout_seconds
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._tf_buffer.can_transform(
                self._base_frame, self._flange_frame, Time()
            ):
                break
        try:
            transform = self._tf_buffer.lookup_transform(
                self._base_frame,
                self._flange_frame,
                Time(),
                timeout=Duration(seconds=0.5),
            )
        except TransformException as error:
            raise RuntimeError(
                f'{self._base_frame} → {self._flange_frame} TF를 읽지 못했습니다: '
                f'{error}'
            ) from error
        result = PoseStamped()
        result.header = transform.header
        result.pose.position.x = transform.transform.translation.x
        result.pose.position.y = transform.transform.translation.y
        result.pose.position.z = transform.transform.translation.z
        result.pose.orientation = transform.transform.rotation
        return result

    def execute(
        self,
        target: PoseStamped,
        speed: int,
        timeout_seconds: float,
    ) -> PoseStamped:
        """고정 Z/Roll/Pitch Cartesian 목표를 보내고 실제 최종 pose를 반환한다."""
        if not self._action.wait_for_server(timeout_sec=5.0):
            raise RuntimeError(f'Cartesian action server가 없습니다: {ACTION_NAME}')
        goal = CartesianMove.Goal()
        goal.target = target
        goal.speed = int(speed)
        goal.mode = 1
        goal.lock_z = True
        goal.lock_roll_pitch = True

        future = self._action.send_goal_async(goal, feedback_callback=self._feedback)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if not future.done() or future.result() is None:
            raise TimeoutError('Cartesian goal 전송 응답 시간이 초과됐습니다')
        handle = future.result()
        if not handle.accepted:
            raise RuntimeError(
                'Cartesian 목표가 거부됐습니다. 로봇 PC의 '
                'cartesian_execution_enabled와 bridge 로그를 확인하세요'
            )
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(
            self, result_future, timeout_sec=timeout_seconds
        )
        if not result_future.done() or result_future.result() is None:
            handle.cancel_goal_async()
            raise TimeoutError('Cartesian 실행 결과 시간이 초과돼 취소를 요청했습니다')
        wrapped = result_future.result()
        if not wrapped.result.success:
            raise RuntimeError(f'Cartesian 실행 실패: {wrapped.result.message}')
        self.get_logger().info(wrapped.result.message)
        return wrapped.result.actual

    def _feedback(self, message) -> None:
        feedback = message.feedback
        self.get_logger().info(
            '이동 피드백: '
            f'position_error={feedback.position_error_m * 1000.0:.3f}mm '
            f'orientation_error={feedback.orientation_error_deg:.3f}deg'
        )


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            '현재 flange pose에서 X 또는 Y만 이동한다. 기본은 DRY RUN이며 실제 '
            '실행에는 --execute --confirm MOVE가 모두 필요하다.'
        )
    )
    parser.add_argument('--axis', required=True, choices=('x', 'y'))
    parser.add_argument('--distance-mm', required=True, type=float)
    parser.add_argument('--speed', type=int, default=5)
    parser.add_argument('--base-frame', default='g_base')
    parser.add_argument('--flange-frame', default='joint6_flange')
    parser.add_argument('--result-timeout-seconds', type=float, default=25.0)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--confirm', default='')
    return parser


def main(args: list[str] | None = None) -> None:
    """CLI 인자를 검사하고 DRY RUN 또는 명시적으로 승인된 이동을 실행한다."""
    arguments, ros_arguments = _parser().parse_known_args(args)
    if not 1 <= arguments.speed <= 100:
        raise ValueError('--speed는 1~100이어야 합니다')
    if arguments.result_timeout_seconds <= 0.0:
        raise ValueError('--result-timeout-seconds는 0보다 커야 합니다')
    rclpy.init(args=ros_arguments)
    node = CartesianSmokeTestNode(arguments.base_frame, arguments.flange_frame)
    try:
        current = node.current_pose()
        target = make_xy_offset_target(
            current, arguments.axis, arguments.distance_mm
        )
        target.header.stamp = node.get_clock().now().to_msg()
        node.get_logger().warning(_pose_text('현재 pose', current))
        node.get_logger().warning(_pose_text('목표 pose', target))
        node.get_logger().warning(
            f'요청 delta: {arguments.axis.upper()} '
            f'{arguments.distance_mm:+.3f}mm, speed={arguments.speed}, mode=1'
        )
        if not arguments.execute:
            node.get_logger().warning(
                'DRY RUN 완료: action을 보내지 않았습니다. 출력 확인 후 실제 실행은 '
                '동일 명령에 --execute --confirm MOVE를 추가하세요'
            )
            return
        if arguments.confirm != 'MOVE':
            raise ValueError('실제 실행에는 --confirm MOVE가 필요합니다')
        for seconds in (3, 2, 1):
            node.get_logger().warning(f'{seconds}초 후 실제 Cartesian 이동')
            time.sleep(1.0)
        actual = node.execute(
            target, arguments.speed, arguments.result_timeout_seconds
        )
        node.get_logger().warning(_pose_text('실제 최종 pose', actual))
        node.get_logger().warning(
            '한 번의 시험이 끝났습니다. 다음 명령 전에 로봇 정지와 실제 자세를 '
            '확인하세요'
        )
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv[1:])
