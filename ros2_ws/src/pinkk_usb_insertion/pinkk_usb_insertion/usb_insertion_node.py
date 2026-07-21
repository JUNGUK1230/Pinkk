"""전체 USB 삽입 절차를 조정하는 상위 상태 머신 노드."""

from __future__ import annotations

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from pinkk_usb_insertion_interfaces.msg import UsbPortObservation
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .configuration import load_yaml
from .state_machine.insertion_state_machine import InsertionStateMachine
from .state_machine.states import InsertionState


def _default_config(filename: str) -> str:
    return str(Path(get_package_share_directory('pinkk_usb_insertion')) / 'config' / filename)


class UsbInsertionNode(Node):
    def __init__(self) -> None:
        super().__init__('pinkk_usb_insertion_node')
        self.declare_parameter('control_config', _default_config('insertion_control.yaml'))
        self._control = load_yaml(str(self.get_parameter('control_config').value))
        self._machine = InsertionStateMachine()
        self._detection_valid = False
        self._state_publisher = self.create_publisher(
            String, '/robot_arm/usb_insertion/state', 10
        )
        self._status_publisher = self.create_publisher(
            String, '/robot_arm/usb_insertion/status', 10
        )
        self.create_subscription(
            String, '/robot_arm/usb_insertion/command', self._command_callback, 10
        )
        self.create_subscription(
            UsbPortObservation,
            '/robot_arm/perception/usb_port/observation',
            self._observation_callback,
            10,
        )
        self._publish_state('초기화 완료')

    def _publish_state(self, status: str) -> None:
        self._state_publisher.publish(String(data=self._machine.state.value))
        self._status_publisher.publish(String(data=status))

    def _observation_callback(self, message: UsbPortObservation) -> None:
        self._detection_valid = bool(message.valid)

    def _command_callback(self, message: String) -> None:
        command = message.data.strip().lower()
        if command == 'reset':
            try:
                self._machine.reset()
                self._publish_state('상태 머신 초기화')
            except ValueError as error:
                self._publish_state(str(error))
            return
        if command != 'start':
            self._publish_state(f'알 수 없는 명령: {message.data}')
            return
        if self._machine.state != InsertionState.IDLE:
            self._publish_state('IDLE 상태에서만 시작할 수 있습니다')
            return

        self._machine.transition(InsertionState.ACQUIRE_PORT)
        self._publish_state('USB 포트 검출 확인')
        if not self._detection_valid:
            self._machine.transition(InsertionState.ERROR)
            self._publish_state('유효한 USB 포트 검출이 없습니다')
            return
        self._machine.transition(InsertionState.ESTIMATE_POSE)
        self._machine.transition(InsertionState.CALCULATE_APPROACH)

        if not bool(self._control['execution']['execution_enabled']):
            self._machine.transition(InsertionState.DRY_RUN_COMPLETE)
            self._publish_state('DRY RUN 완료: 실제 이동 명령은 보내지 않았습니다')
            return
        self._machine.transition(InsertionState.MOVE_PRE_APPROACH)
        self._publish_state('접근 실행 단계 준비; 실제 실행 연결은 아직 구현되지 않음')


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = UsbInsertionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
