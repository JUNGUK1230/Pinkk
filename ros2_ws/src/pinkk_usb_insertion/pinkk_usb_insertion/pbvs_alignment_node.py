"""수동/YOLO 관측으로 고정-Z PBVS 목표를 계산해 DRY RUN 토픽에 발행한다."""

from __future__ import annotations

from pathlib import Path
import time

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, TransformStamped, Vector3Stamped
import numpy as np
from pinkk_usb_insertion_interfaces.msg import UsbPortObservation
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener

from .configuration import load_yaml
from .control.pbvs_controller import calculate_fixed_z_camera_pbvs
from .control.observation_reference import validate_observation_joint_pose
from .control.yaw_alignment import (
    apply_base_yaw_step,
    limited_yaw_step_rad,
    named_axis_vector,
    undirected_planar_axis_error_rad,
)
from .geometry.transforms import compose, make_transform, validate_transform
from .ros_utils import pose_to_transform, transform_to_pose


JOINT_NAMES = (
    'joint2_to_joint1',
    'joint3_to_joint2',
    'joint4_to_joint3',
    'joint5_to_joint4',
    'joint6_to_joint5',
    'joint6output_to_joint6',
)


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
        self.declare_parameter('enable_yaw_pbvs', False)

        control = load_yaml(str(self.get_parameter('control_config').value))
        handeye = load_yaml(str(self.get_parameter('handeye_config').value))['handeye']
        frames = control['frames']
        pbvs = control['pbvs']
        yaw_pbvs = control['yaw_pbvs']
        reference = control['alignment_reference']
        self._base_frame = str(frames['base'])
        self._flange_frame = str(frames['flange'])
        self._camera_frame = str(frames['camera'])
        self._port_frame = str(frames['port'])
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
        self._yaw_enabled = bool(yaw_pbvs['enabled']) or bool(
            self.get_parameter('enable_yaw_pbvs').value
        )
        flange_long_axis = named_axis_vector(
            str(yaw_pbvs['flange_long_axis'])
        )
        flange_to_plug_yaw_rad = np.radians(
            float(yaw_pbvs['flange_to_plug_yaw_offset_deg'])
        )
        if not np.isfinite(flange_to_plug_yaw_rad):
            raise ValueError('flange_to_plug_yaw_offset_deg가 유한값이 아닙니다')
        self._plug_long_axis_in_flange = np.array(
            (
                np.cos(flange_to_plug_yaw_rad) * flange_long_axis[0]
                - np.sin(flange_to_plug_yaw_rad) * flange_long_axis[1],
                np.sin(flange_to_plug_yaw_rad) * flange_long_axis[0]
                + np.cos(flange_to_plug_yaw_rad) * flange_long_axis[1],
                flange_long_axis[2],
            ),
            dtype=np.float64,
        )
        self._port_long_axis = named_axis_vector(
            str(yaw_pbvs['port_long_axis'])
        )
        self._maximum_yaw_step_deg = float(yaw_pbvs['maximum_step_deg'])
        self._yaw_tolerance_deg = float(yaw_pbvs['tolerance_deg'])
        self._use_latest_flange_tf = bool(
            self.get_parameter('use_latest_flange_tf').value
        )
        if not bool(reference['automatic_from_observe_pose']):
            raise ValueError('automatic_from_observe_pose=true가 필요합니다')
        self._observation_joints_deg = tuple(
            float(value) for value in reference['observation_joint_degrees']
        )
        if len(self._observation_joints_deg) != len(JOINT_NAMES):
            raise ValueError('observation_joint_degrees는 관절 6개가 필요합니다')
        self._maximum_observation_joint_error_deg = float(
            reference['maximum_observation_joint_error_deg']
        )
        self._maximum_joint_state_age_seconds = float(
            reference['maximum_joint_state_age_seconds']
        )
        self._locked_base_to_flange: np.ndarray | None = None
        self._latest_joints: tuple[float, ...] | None = None
        self._joint_received_at: float | None = None
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._tf_broadcaster = TransformBroadcaster(self)

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
        self._error_publisher = self.create_publisher(
            Vector3Stamped, '/robot_arm/pbvs/error', 10
        )
        self._yaw_enabled_publisher = self.create_publisher(
            Bool, '/robot_arm/pbvs/yaw_enabled', 10
        )
        self.create_subscription(
            UsbPortObservation,
            '/robot_arm/perception/usb_port/observation',
            self._observation_callback,
            10,
        )
        self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_callback,
            10,
        )
        self.get_logger().info(
            '고정-Z PBVS 목표 계산 시작: 이 노드 자체는 로봇 명령을 보내지 않습니다'
        )
        if self._use_latest_flange_tf:
            self.get_logger().warning(
                '수동 stop-and-go 모드: 로봇 정지 상태의 최신 flange TF를 사용합니다'
            )
        if self._yaw_enabled:
            self.get_logger().warning(
                'Yaw PBVS 계산 활성화: 실제 실행 전 flange_long_axis와 '
                'port_long_axis 방향을 RViz에서 확인해야 합니다'
            )
        self.get_logger().info(
            '수동 capture 없이 OBSERVE_POSE 관절 확인 후 기준 TF를 자동 저장합니다'
        )

    def _publish_status(self, text: str) -> None:
        message = String()
        message.data = text
        self._status_publisher.publish(message)

    def _joint_state_callback(self, message: JointState) -> None:
        values = dict(zip(message.name, message.position))
        if all(name in values for name in JOINT_NAMES):
            self._latest_joints = tuple(
                float(values[name]) for name in JOINT_NAMES
            )
            self._joint_received_at = time.monotonic()

    def _validate_and_lock_observation_reference(
        self,
        base_to_flange: np.ndarray,
    ) -> None:
        if self._latest_joints is None or self._joint_received_at is None:
            raise ValueError('/joint_states를 아직 받지 못했습니다')
        age = time.monotonic() - self._joint_received_at
        if age > self._maximum_joint_state_age_seconds:
            raise ValueError(f'/joint_states가 오래됐습니다: age={age:.3f}s')
        maximum_error = validate_observation_joint_pose(
            self._latest_joints,
            self._observation_joints_deg,
            self._maximum_observation_joint_error_deg,
        )
        self._locked_base_to_flange = base_to_flange.copy()
        self.get_logger().warning(
            'OBSERVE_POSE 확인 후 정렬 기준 자동 저장: '
            f'max_joint_error={maximum_error:.3f}deg, '
            f'z_lock={base_to_flange[2, 3]:.6f}m'
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
            base_to_flange = _tf_to_matrix(transform)
            camera_to_port = pose_to_transform(observation.pose)
            base_to_port = compose(
                base_to_flange,
                self._flange_to_camera,
                camera_to_port,
            )
        except (TransformException, ValueError) as error:
            self._publish_status(f'포트 TF 계산 거부: {error}')
            return

        port_pose = PoseStamped()
        port_pose.header.stamp = observation.header.stamp
        port_pose.header.frame_id = self._base_frame
        port_pose.pose = transform_to_pose(base_to_port)
        self._port_pose_publisher.publish(port_pose)

        port_transform = TransformStamped()
        # use_latest_flange_tf 모드는 서로 다른 PC의 시계 차이를 피하기 위해
        # 정지 상태의 최신 TF를 사용한다. RViz용 TF도 현재 시각으로 발행해
        # 과거/미래 extrapolation 없이 바로 표시되도록 한다.
        port_transform.header.stamp = self.get_clock().now().to_msg()
        port_transform.header.frame_id = self._base_frame
        port_transform.child_frame_id = self._port_frame
        port_transform.transform.translation.x = float(base_to_port[0, 3])
        port_transform.transform.translation.y = float(base_to_port[1, 3])
        port_transform.transform.translation.z = float(base_to_port[2, 3])
        port_transform.transform.rotation = port_pose.pose.orientation
        self._tf_broadcaster.sendTransform(port_transform)

        if self._locked_base_to_flange is None:
            try:
                self._validate_and_lock_observation_reference(base_to_flange)
            except ValueError as error:
                self._publish_status(
                    f'포트 TF 발행 중; PBVS 자동 기준 저장 거부: {error}'
                )
                return

        try:
            result = calculate_fixed_z_camera_pbvs(
                base_to_flange,
                self._flange_to_camera,
                camera_to_port,
                self._maximum_step,
                self._tolerance,
                self._desired_x,
                self._desired_y,
                self._locked_base_to_flange,
            )
        except (TransformException, ValueError) as error:
            self._publish_status(f'PBVS 계산 거부: {error}')
            return

        yaw_error_rad = 0.0
        yaw_step_rad = 0.0
        yaw_converged = True
        target_base_to_flange = result.target_base_to_flange.copy()
        if self._yaw_enabled:
            try:
                current_axis_base = (
                    base_to_flange[:3, :3]
                    @ self._plug_long_axis_in_flange
                )
                target_axis_base = base_to_port[:3, :3] @ self._port_long_axis
                yaw_error_rad = undirected_planar_axis_error_rad(
                    current_axis_base,
                    target_axis_base,
                )
                yaw_step_rad, yaw_converged = limited_yaw_step_rad(
                    yaw_error_rad,
                    self._maximum_yaw_step_deg,
                    self._yaw_tolerance_deg,
                )
                yaw_target = apply_base_yaw_step(
                    base_to_flange,
                    yaw_step_rad,
                )
                target_base_to_flange[:3, :3] = yaw_target[:3, :3]
            except ValueError as error:
                self._publish_status(f'Yaw PBVS 계산 거부: {error}')
                return

        target_pose = PoseStamped()
        target_pose.header = port_pose.header
        target_pose.pose = transform_to_pose(target_base_to_flange)
        self._target_publisher.publish(target_pose)

        converged = Bool()
        converged.data = result.converged and yaw_converged
        self._converged_publisher.publish(converged)

        error_message = Vector3Stamped()
        error_message.header = target_pose.header
        error_message.vector.x = float(result.error_base_xy_m[0])
        error_message.vector.y = float(result.error_base_xy_m[1])
        error_message.vector.z = float(yaw_error_rad)
        self._error_publisher.publish(error_message)

        yaw_enabled_message = Bool()
        yaw_enabled_message.data = self._yaw_enabled
        self._yaw_enabled_publisher.publish(yaw_enabled_message)

        self._publish_status(
            'TARGET_ONLY | camera_error_xy_m='
            f'[{result.error_camera_xy_m[0]:+.6f}, {result.error_camera_xy_m[1]:+.6f}] '
            'base_step_xy_m='
            f'[{result.applied_step_base_xy_m[0]:+.6f}, '
            f'{result.applied_step_base_xy_m[1]:+.6f}] '
            f'yaw_error_deg={np.degrees(yaw_error_rad):+.3f} '
            f'yaw_step_deg={np.degrees(yaw_step_rad):+.3f} '
            f'converged={converged.data} '
            f'tf_mode={"latest_stationary" if self._use_latest_flange_tf else "timestamp"} '
            f'z_lock={self._locked_base_to_flange[2, 3]:.6f}m '
            f'yaw_pbvs={"enabled" if self._yaw_enabled else "disabled"}'
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
