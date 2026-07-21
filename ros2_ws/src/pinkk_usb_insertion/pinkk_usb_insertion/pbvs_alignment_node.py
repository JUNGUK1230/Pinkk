"""수동/YOLO 관측으로 고정-Z PBVS 목표를 계산해 DRY RUN 토픽에 발행한다."""

from __future__ import annotations

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
import numpy as np
from pinkk_usb_insertion_interfaces.msg import UsbPortObservation
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener

from .configuration import load_yaml
from .control.pbvs_controller import calculate_fixed_z_camera_pbvs
from .geometry.transforms import make_transform, validate_transform
from .ros_utils import pose_to_transform, transform_to_pose


def _default_config(filename: str) -> str:
    share = Path(get_package_share_directory('pinkk_usb_insertion'))
    return str(share / 'config' / filename)


def _tf_to_matrix(message) -> np.ndarray:
    transform = message.transform
    return make_transform(
        (transform.translation.x, transform.translation.y, transform.translation.z),
        (
            transform.rotation.x,
            transform.rotation.y,
            transform.rotation.z,
            transform.rotation.w,
        ),
    )


class PbvsAlignmentNode(Node):
    """현재 flange TF와 포트 관측을 결합해 다음 XY 목표를 계산한다."""

    def __init__(self) -> None:
        super().__init__('pinkk_pbvs_alignment_node')
        self.declare_parameter('control_config', _default_config('insertion_control.yaml'))
        self.declare_parameter('handeye_config', _default_config('handeye.yaml'))
        self.declare_parameter('use_latest_flange_tf', False)

        control = load_yaml(str(self.get_parameter('control_config').value))
        handeye = load_yaml(str(self.get_parameter('handeye_config').value))['handeye']
        frames = control['frames']
        pbvs = control['pbvs']
        reference = control['alignment_reference']
        self._base_frame = str(frames['base'])
        self._flange_frame = str(frames['flange'])
        self._camera_frame = str(frames['camera'])
        self._flange_to_camera = validate_transform(
            np.asarray(handeye['matrix_4x4'], dtype=np.float64)
        )
        if handeye['parent_frame'] != self._flange_frame:
            raise ValueError('Hand-eye parent_frame과 제어 flange frame이 다릅니다')
        if handeye['child_frame'] != self._camera_frame:
            raise ValueError('Hand-eye child_frame과 제어 camera frame이 다릅니다')
        if not bool(handeye['calibrated']):
            raise ValueError('검증된 Hand-eye 보정값이 필요합니다')
        if not bool(pbvs['enabled']):
            raise ValueError('pbvs.enabled=false입니다')
        if not bool(pbvs['keep_flange_z']):
            raise ValueError('이 노드는 keep_flange_z=true만 지원합니다')
        if bool(pbvs['publish_motion_commands']):
            raise ValueError('DRY RUN 노드에서는 publish_motion_commands=true를 거부합니다')

        self._maximum_step = float(pbvs['maximum_xy_step_m'])
        self._tolerance = float(pbvs['xy_tolerance_m'])
        self._desired_x = float(pbvs['desired_port_x_m'])
        self._desired_y = float(pbvs['desired_port_y_m'])
        self._use_latest_flange_tf = bool(
            self.get_parameter('use_latest_flange_tf').value
        )
        self._capture_on_first_observation = bool(
            reference['capture_on_first_valid_observation']
        )
        self._locked_base_to_flange: np.ndarray | None = None
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._port_pose_publisher = self.create_publisher(
            PoseStamped, '/robot_arm/pbvs/port_pose_base', 10
        )
        self._target_publisher = self.create_publisher(
            PoseStamped, '/robot_arm/pbvs/target_flange_pose', 10
        )
        self._converged_publisher = self.create_publisher(
            Bool, '/robot_arm/pbvs/converged', 10
        )
        self._status_publisher = self.create_publisher(
            String, '/robot_arm/pbvs/status', 10
        )
        self.create_subscription(
            UsbPortObservation,
            '/robot_arm/perception/usb_port/observation',
            self._observation_callback,
            10,
        )
        self.create_subscription(
            String,
            '/robot_arm/pbvs/reference_command',
            self._reference_command_callback,
            10,
        )
        self.get_logger().info(
            '고정-Z PBVS 목표 계산 시작: 이 노드 자체는 로봇 명령을 보내지 않습니다'
        )
        if self._use_latest_flange_tf:
            self.get_logger().warning(
                '수동 stop-and-go 모드: 로봇 정지 상태의 최신 flange TF를 사용합니다'
            )

    def _publish_status(self, text: str) -> None:
        message = String()
        message.data = text
        self._status_publisher.publish(message)

    def _latest_flange_transform(self) -> np.ndarray:
        message = self._tf_buffer.lookup_transform(
            self._base_frame,
            self._flange_frame,
            Time(),
            timeout=Duration(seconds=1.0),
        )
        return _tf_to_matrix(message)

    def _reference_command_callback(self, message: String) -> None:
        command = message.data.strip().lower()
        if command == 'reset':
            self._locked_base_to_flange = None
            self._publish_status('정렬 기준을 초기화했습니다')
            return
        if command != 'capture':
            self._publish_status('정렬 기준 명령은 capture 또는 reset만 허용합니다')
            return
        try:
            self._locked_base_to_flange = self._latest_flange_transform()
        except (TransformException, ValueError) as error:
            self._publish_status(f'정렬 기준 캡처 거부: {error}')
            return
        z_m = self._locked_base_to_flange[2, 3]
        self._publish_status(
            f'정렬 기준 캡처 완료: z_lock={z_m:.6f}m, '
            '초기 quaternion lock; Yaw PBVS는 TCP 전까지 비활성'
        )

    def _observation_callback(self, observation: UsbPortObservation) -> None:
        if not observation.valid:
            self._publish_status(f'관측 거부: {observation.rejection_reason}')
            return
        if observation.header.frame_id != self._camera_frame:
            self._publish_status(
                f'관측 거부: frame={observation.header.frame_id}, expected={self._camera_frame}'
            )
            return
        if self._locked_base_to_flange is None:
            if self._capture_on_first_observation:
                try:
                    self._locked_base_to_flange = self._latest_flange_transform()
                except (TransformException, ValueError) as error:
                    self._publish_status(f'정렬 기준 자동 캡처 거부: {error}')
                    return
            else:
                self._publish_status(
                    'PBVS 계산 거부: 초기 관측 자세에서 reference_command=capture가 필요합니다'
                )
                return
        try:
            lookup_time = (
                Time()
                if self._use_latest_flange_tf
                else Time.from_msg(observation.header.stamp)
            )
            transform = self._tf_buffer.lookup_transform(
                self._base_frame,
                self._flange_frame,
                lookup_time,
                timeout=Duration(seconds=0.2),
            )
            result = calculate_fixed_z_camera_pbvs(
                _tf_to_matrix(transform),
                self._flange_to_camera,
                pose_to_transform(observation.pose),
                self._maximum_step,
                self._tolerance,
                self._desired_x,
                self._desired_y,
                self._locked_base_to_flange,
            )
        except (TransformException, ValueError) as error:
            self._publish_status(f'PBVS 계산 거부: {error}')
            return

        port_pose = PoseStamped()
        port_pose.header.stamp = observation.header.stamp
        port_pose.header.frame_id = self._base_frame
        port_pose.pose = transform_to_pose(result.base_to_port)
        self._port_pose_publisher.publish(port_pose)

        target_pose = PoseStamped()
        target_pose.header = port_pose.header
        target_pose.pose = transform_to_pose(result.target_base_to_flange)
        self._target_publisher.publish(target_pose)

        converged = Bool()
        converged.data = result.converged
        self._converged_publisher.publish(converged)
        self._publish_status(
            'TARGET_ONLY | camera_error_xy_m='
            f'[{result.error_camera_xy_m[0]:+.6f}, {result.error_camera_xy_m[1]:+.6f}] '
            'base_step_xy_m='
            f'[{result.applied_step_base_xy_m[0]:+.6f}, '
            f'{result.applied_step_base_xy_m[1]:+.6f}] '
            f'converged={result.converged} '
            f'tf_mode={"latest_stationary" if self._use_latest_flange_tf else "timestamp"} '
            f'z_lock={self._locked_base_to_flange[2, 3]:.6f}m yaw_pbvs=pending_tcp'
        )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = PbvsAlignmentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
