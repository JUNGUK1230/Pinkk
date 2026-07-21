"""계산 계층과 실제 로봇 실행 백엔드 사이의 단일 안전 경계."""

from __future__ import annotations

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

from .configuration import execution_gate, load_yaml


def _default_config(filename: str) -> str:
    return str(Path(get_package_share_directory('pinkk_usb_insertion')) / 'config' / filename)


class ArmMotionNode(Node):
    """현재 버전은 명령 안전 조건과 인터페이스만 검증한다."""

    def __init__(self) -> None:
        super().__init__('pinkk_arm_motion_node')
        self.declare_parameter('control_config', _default_config('insertion_control.yaml'))
        self.declare_parameter('tool_config', _default_config('tool_transform.yaml'))
        self._control = load_yaml(str(self.get_parameter('control_config').value))
        self._tool = load_yaml(str(self.get_parameter('tool_config').value))
        self._done_publisher = self.create_publisher(Bool, '/robot_arm/motion_done', 10)
        self._status_publisher = self.create_publisher(String, '/robot_arm/motion_status', 10)
        self.create_subscription(
            PoseStamped, '/robot_arm/target_pose', self._target_callback, 10
        )
        allowed, reason = execution_gate(self._control, self._tool)
        mode = 'EXECUTION REQUESTED' if allowed else f'DRY RUN ({reason})'
        self.get_logger().info(f'로봇 이동 인터페이스 준비: {mode}')

    def _target_callback(self, message: PoseStamped) -> None:
        allowed, reason = execution_gate(self._control, self._tool)
        if not allowed:
            self._done_publisher.publish(Bool(data=False))
            self._status_publisher.publish(
                String(data=f'DRY_RUN: 목표 수신, 로봇에는 전송하지 않음 ({reason})')
            )
            return

        # 실제 MoveIt action 연동은 좌표 검증과 TCP 확정 후 이 경계 안에 추가한다.
        self._done_publisher.publish(Bool(data=False))
        self._status_publisher.publish(
            String(data='REJECTED: 실행 백엔드가 아직 구현되지 않았습니다')
        )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ArmMotionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
