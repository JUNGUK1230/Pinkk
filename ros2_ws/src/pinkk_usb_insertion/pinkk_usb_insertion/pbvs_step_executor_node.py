"""PBVS 거친 이동과 영상 기반 미세 보정을 제조사 API bridge로 실행한다."""

from __future__ import annotations

import math
import threading
import time

from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseStamped
import numpy as np
from pinkk_usb_insertion_interfaces.action import CartesianMove
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String
from tf2_ros import Buffer, TransformException, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint

from .control.yaw_alignment import (
    apply_observation_roll_pitch_with_current_yaw,
    calibrated_keypoint_joint_step_rad,
    joint6_yaw_target_rad,
)
from .control.waypoint import limited_waypoint_translation
from .control.z_recovery import raised_z_recovery_target
from .geometry.transforms import make_transform, rotation_to_rpy_degrees
from .ros_utils import pose_to_transform, transform_to_pose


JOINT_NAMES = (
    'joint2_to_joint1',
    'joint3_to_joint2',
    'joint4_to_joint3',
    'joint5_to_joint4',
    'joint6_to_joint5',
    'joint6output_to_joint6',
)
CARTESIAN_ACTION = '/robot_arm/cartesian_move'
JOINT_ACTION = '/arm_group_controller/follow_joint_trajectory'
COMMAND_TOPIC = '/robot_arm/hybrid/command'
STATUS_TOPIC = '/robot_arm/hybrid/status'


class PbvsStepExecutorNode(Node):
    """명시적인 단발 명령만 제조사 bridge에 전달한다."""

    def __init__(self) -> None:
        super().__init__('pinkk_pbvs_step_executor_node')
        self.declare_parameter('enable_execution', False)
        self.declare_parameter('base_frame', 'g_base')
        self.declare_parameter('flange_frame', 'joint6_flange')
        self.declare_parameter('maximum_input_age_seconds', 1.0)
        self.declare_parameter('minimum_xy_step_m', 0.003)
        self.declare_parameter('maximum_xy_step_m', 0.100)
        self.declare_parameter('coarse_use_observation_z', True)
        self.declare_parameter('cartesian_speed', 10)
        self.declare_parameter('cartesian_mode', 0)
        self.declare_parameter('cartesian_timeout_seconds', 100.0)
        self.declare_parameter('warning_delay_seconds', 3.0)
        self.declare_parameter('post_coarse_yaw_delay_seconds', 0.5)
        self.declare_parameter('reobservation_timeout_seconds', 5.0)
        self.declare_parameter('enable_automatic_z_recovery', True)
        self.declare_parameter('z_recovery_step_m', 0.030)
        self.declare_parameter('z_recovery_cartesian_mode', 1)
        self.declare_parameter('z_recovery_max_attempts', 2)
        self.declare_parameter('z_recovery_settle_seconds', 0.8)
        self.declare_parameter(
            'z_recovery_use_observation_roll_pitch_target', False
        )
        self.declare_parameter('z_recovery_lock_roll_pitch', True)
        self.declare_parameter('yaw_start_xy_tolerance_m', 0.005)
        self.declare_parameter('automatic_refine_xy_max_cycles', 2)
        self.declare_parameter('waypoint_maximum_xy_step_m', 0.015)
        self.declare_parameter('waypoint_maximum_z_step_m', 0.010)
        self.declare_parameter('waypoint_max_cycles', 6)
        self.declare_parameter('waypoint_settle_seconds', 0.8)
        self.declare_parameter('enable_last_visible_pose_recovery', True)
        self.declare_parameter('use_observation_roll_pitch_target', True)
        self.declare_parameter('lock_roll_pitch_during_xy', True)
        self.declare_parameter('observation_roll_pitch_tolerance_deg', 5.0)
        self.declare_parameter('keypoint_desired_axis_deg', 0.0)
        self.declare_parameter('minimum_yaw_step_deg', 3.0)
        self.declare_parameter('keypoint_maximum_step_deg', 5.0)
        self.declare_parameter('keypoint_tolerance_deg', 1.0)
        self.declare_parameter('keypoint_command_sign', -1.0)
        self.declare_parameter('keypoint_joint_gain', 1.0)
        self.declare_parameter('joint6_direction', -1.0)
        self.declare_parameter('joint6_limit_deg', 175.0)
        self.declare_parameter('joint6_move_seconds', 6.0)

        self._enabled = bool(self.get_parameter('enable_execution').value)
        self._base_frame = str(self.get_parameter('base_frame').value)
        self._flange_frame = str(self.get_parameter('flange_frame').value)
        self._maximum_age = float(
            self.get_parameter('maximum_input_age_seconds').value
        )
        self._minimum_xy_step = float(
            self.get_parameter('minimum_xy_step_m').value
        )
        self._maximum_xy_step = float(
            self.get_parameter('maximum_xy_step_m').value
        )
        self._coarse_use_observation_z = bool(
            self.get_parameter('coarse_use_observation_z').value
        )
        self._cartesian_speed = int(
            self.get_parameter('cartesian_speed').value
        )
        self._cartesian_mode = int(
            self.get_parameter('cartesian_mode').value
        )
        self._cartesian_timeout = float(
            self.get_parameter('cartesian_timeout_seconds').value
        )
        self._warning_delay = float(
            self.get_parameter('warning_delay_seconds').value
        )
        self._post_coarse_yaw_delay = float(
            self.get_parameter('post_coarse_yaw_delay_seconds').value
        )
        self._reobservation_timeout = float(
            self.get_parameter('reobservation_timeout_seconds').value
        )
        self._enable_automatic_z_recovery = bool(
            self.get_parameter('enable_automatic_z_recovery').value
        )
        self._z_recovery_step = float(
            self.get_parameter('z_recovery_step_m').value
        )
        self._z_recovery_cartesian_mode = int(
            self.get_parameter('z_recovery_cartesian_mode').value
        )
        self._z_recovery_max_attempts = int(
            self.get_parameter('z_recovery_max_attempts').value
        )
        self._z_recovery_settle = float(
            self.get_parameter('z_recovery_settle_seconds').value
        )
        self._z_recovery_use_observation_roll_pitch = bool(
            self.get_parameter(
                'z_recovery_use_observation_roll_pitch_target'
            ).value
        )
        self._z_recovery_lock_roll_pitch = bool(
            self.get_parameter('z_recovery_lock_roll_pitch').value
        )
        self._yaw_start_xy_tolerance = float(
            self.get_parameter('yaw_start_xy_tolerance_m').value
        )
        self._automatic_refine_xy_max_cycles = int(
            self.get_parameter('automatic_refine_xy_max_cycles').value
        )
        self._waypoint_maximum_xy_step = float(
            self.get_parameter('waypoint_maximum_xy_step_m').value
        )
        self._waypoint_maximum_z_step = float(
            self.get_parameter('waypoint_maximum_z_step_m').value
        )
        self._waypoint_max_cycles = int(
            self.get_parameter('waypoint_max_cycles').value
        )
        self._waypoint_settle = float(
            self.get_parameter('waypoint_settle_seconds').value
        )
        self._enable_last_visible_pose_recovery = bool(
            self.get_parameter('enable_last_visible_pose_recovery').value
        )
        self._use_observation_roll_pitch = bool(
            self.get_parameter('use_observation_roll_pitch_target').value
        )
        self._lock_roll_pitch_during_xy = bool(
            self.get_parameter('lock_roll_pitch_during_xy').value
        )
        self._observation_roll_pitch_tolerance = float(
            self.get_parameter('observation_roll_pitch_tolerance_deg').value
        )
        self._desired_axis = float(
            self.get_parameter('keypoint_desired_axis_deg').value
        )
        self._minimum_yaw_step = float(
            self.get_parameter('minimum_yaw_step_deg').value
        )
        self._maximum_yaw_step = float(
            self.get_parameter('keypoint_maximum_step_deg').value
        )
        self._yaw_tolerance = float(
            self.get_parameter('keypoint_tolerance_deg').value
        )
        self._keypoint_command_sign = float(
            self.get_parameter('keypoint_command_sign').value
        )
        self._keypoint_joint_gain = float(
            self.get_parameter('keypoint_joint_gain').value
        )
        self._joint6_direction = float(
            self.get_parameter('joint6_direction').value
        )
        self._joint6_limit = float(
            self.get_parameter('joint6_limit_deg').value
        )
        self._joint6_move_seconds = float(
            self.get_parameter('joint6_move_seconds').value
        )
        self._validate_parameters()

        self._latest_target: PoseStamped | None = None
        self._target_received_at: float | None = None
        self._observation_reference: PoseStamped | None = None
        self._latest_joints: list[float] | None = None
        self._joints_received_at: float | None = None
        self._latest_keypoint_axis: float | None = None
        self._keypoint_received_at: float | None = None
        self._last_visible_flange: np.ndarray | None = None
        self._executing = False
        self._lock = threading.Lock()

        callbacks = ReentrantCallbackGroup()
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._cartesian_action = ActionClient(
            self,
            CartesianMove,
            CARTESIAN_ACTION,
            callback_group=callbacks,
        )
        self._joint_action = ActionClient(
            self,
            FollowJointTrajectory,
            JOINT_ACTION,
            callback_group=callbacks,
        )
        self._status = self.create_publisher(String, STATUS_TOPIC, 10)
        self.create_subscription(
            PoseStamped,
            '/robot_arm/pbvs/target_flange_pose',
            self._target_callback,
            10,
            callback_group=callbacks,
        )
        self.create_subscription(
            PoseStamped,
            '/robot_arm/pbvs/observation_reference_pose',
            self._observation_reference_callback,
            10,
            callback_group=callbacks,
        )
        self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_callback,
            10,
            callback_group=callbacks,
        )
        self.create_subscription(
            Float64,
            '/robot_arm/perception/usb_port/keypoint_axis_angle_deg',
            self._keypoint_callback,
            10,
            callback_group=callbacks,
        )
        self.create_subscription(
            String,
            COMMAND_TOPIC,
            self._command_callback,
            10,
            callback_group=callbacks,
        )
        mode = '허용' if self._enabled else '차단'
        self.get_logger().warning(
            f'하이브리드 단발 실행기: 실행={mode}, '
            'commands=coarse_xy/coarse_xy_then_yaw/refine_xy/refine_yaw/'
            'recover_z_once/waypoint_pbvs_align, '
            f'min_xy={self._minimum_xy_step * 1000.0:.1f}mm, '
            f'max_xy={self._maximum_xy_step * 1000.0:.1f}mm, '
            f'coarse_observation_z={self._coarse_use_observation_z}, '
            f'yaw_start_xy={self._yaw_start_xy_tolerance * 1000.0:.1f}mm, '
            f'min_yaw_step={self._minimum_yaw_step:.1f}deg, '
            f'yaw_step={self._maximum_yaw_step:.1f}deg, '
            f'yaw_joint_gain={self._keypoint_joint_gain:.3f}, '
            f'joint6_direction={self._joint6_direction:+.0f}, '
            f'observation_roll_pitch={self._use_observation_roll_pitch}, '
            f'auto_z_recovery={self._enable_automatic_z_recovery}, '
            f'z_recovery_step={self._z_recovery_step * 1000.0:.1f}mm, '
            f'z_recovery_mode={self._z_recovery_cartesian_mode}, '
            f'z_recovery_attempts={self._z_recovery_max_attempts}, '
            'z_recovery_observation_roll_pitch='
            f'{self._z_recovery_use_observation_roll_pitch}, '
            f'z_recovery_roll_pitch_lock={self._z_recovery_lock_roll_pitch}, '
            'automatic_refine_xy_max_cycles='
            f'{self._automatic_refine_xy_max_cycles}, '
            f'xy_roll_pitch_lock={self._lock_roll_pitch_during_xy}, '
            f'waypoint_max_xy={self._waypoint_maximum_xy_step * 1000.0:.1f}mm, '
            f'waypoint_max_z={self._waypoint_maximum_z_step * 1000.0:.1f}mm, '
            f'waypoint_max_cycles={self._waypoint_max_cycles}, '
            f'waypoint_settle={self._waypoint_settle:.1f}s, '
            'last_visible_pose_recovery='
            f'{self._enable_last_visible_pose_recovery}'
        )

    def _validate_parameters(self) -> None:
        if not 0.0 < self._maximum_age <= 5.0:
            raise ValueError('maximum_input_age_seconds는 0~5초 범위여야 합니다')
        if not 0.0 < self._minimum_xy_step <= 0.020:
            raise ValueError('minimum_xy_step_m은 0~0.02m 범위여야 합니다')
        if not self._minimum_xy_step <= self._maximum_xy_step <= 0.200:
            raise ValueError('maximum_xy_step_m은 0~0.2m 범위여야 합니다')
        if not 1 <= self._cartesian_speed <= 100:
            raise ValueError('cartesian_speed는 1~100이어야 합니다')
        if self._cartesian_mode not in (0, 1):
            raise ValueError('cartesian_mode는 0 또는 1이어야 합니다')
        if not 1.0 <= self._cartesian_timeout <= 120.0:
            raise ValueError('cartesian_timeout_seconds는 1~120초여야 합니다')
        if not 0.0 <= self._warning_delay <= 10.0:
            raise ValueError('warning_delay_seconds는 0~10초여야 합니다')
        if not 0.0 <= self._post_coarse_yaw_delay <= 5.0:
            raise ValueError(
                'post_coarse_yaw_delay_seconds는 0~5초여야 합니다'
            )
        if not 0.5 <= self._reobservation_timeout <= 30.0:
            raise ValueError(
                'reobservation_timeout_seconds는 0.5~30초 범위여야 합니다'
            )
        if not 0.005 <= self._z_recovery_step <= 0.100:
            raise ValueError('z_recovery_step_m은 0.005~0.100m 범위여야 합니다')
        if self._z_recovery_cartesian_mode not in (0, 1):
            raise ValueError('z_recovery_cartesian_mode는 0 또는 1이어야 합니다')
        if not 1 <= self._z_recovery_max_attempts <= 5:
            raise ValueError('z_recovery_max_attempts는 1~5 범위여야 합니다')
        if not 0.0 <= self._z_recovery_settle <= 5.0:
            raise ValueError('z_recovery_settle_seconds는 0~5초 범위여야 합니다')
        if not self._minimum_xy_step <= self._yaw_start_xy_tolerance <= 0.050:
            raise ValueError(
                'yaw_start_xy_tolerance_m은 minimum_xy_step_m 이상이고 '
                '0.05m 이하여야 합니다'
            )
        if not 0 <= self._automatic_refine_xy_max_cycles <= 5:
            raise ValueError(
                'automatic_refine_xy_max_cycles는 0~5 범위여야 합니다'
            )
        if not (
            self._minimum_xy_step
            <= self._waypoint_maximum_xy_step
            <= self._maximum_xy_step
        ):
            raise ValueError(
                'waypoint_maximum_xy_step_m은 minimum_xy_step_m 이상이고 '
                'maximum_xy_step_m 이하여야 합니다'
            )
        if not 0.005 <= self._waypoint_maximum_z_step <= 0.100:
            raise ValueError(
                'waypoint_maximum_z_step_m은 0.005~0.100m 범위여야 합니다'
            )
        if not 1 <= self._waypoint_max_cycles <= 20:
            raise ValueError('waypoint_max_cycles는 1~20 범위여야 합니다')
        if not 0.0 <= self._waypoint_settle <= 5.0:
            raise ValueError('waypoint_settle_seconds는 0~5초 범위여야 합니다')
        if not 0.5 <= self._observation_roll_pitch_tolerance <= 15.0:
            raise ValueError(
                'observation_roll_pitch_tolerance_deg는 '
                '0.5~15도 범위여야 합니다'
            )
        finite = (
            self._desired_axis,
            self._minimum_yaw_step,
            self._maximum_yaw_step,
            self._yaw_tolerance,
            self._keypoint_command_sign,
            self._keypoint_joint_gain,
            self._joint6_direction,
            self._joint6_limit,
            self._joint6_move_seconds,
        )
        if not all(math.isfinite(value) for value in finite):
            raise ValueError('Yaw 파라미터는 유한값이어야 합니다')
        if not 0.0 < self._maximum_yaw_step <= 30.0:
            raise ValueError('keypoint_maximum_step_deg는 0~30도여야 합니다')
        if not 0.0 < self._minimum_yaw_step <= self._maximum_yaw_step:
            raise ValueError(
                'minimum_yaw_step_deg는 0보다 크고 '
                'keypoint_maximum_step_deg 이하여야 합니다'
            )
        if not 0.0 <= self._yaw_tolerance <= 5.0:
            raise ValueError('keypoint_tolerance_deg는 0~5도여야 합니다')
        if self._keypoint_command_sign not in (-1.0, 1.0):
            raise ValueError('keypoint_command_sign은 -1 또는 +1이어야 합니다')
        if not 0.1 <= self._keypoint_joint_gain <= 5.0:
            raise ValueError('keypoint_joint_gain은 0.1~5.0이어야 합니다')
        if self._joint6_direction not in (-1.0, 1.0):
            raise ValueError('joint6_direction은 -1 또는 +1이어야 합니다')
        if not 0.0 < self._joint6_limit < 180.0:
            raise ValueError('joint6_limit_deg는 0~180도 범위여야 합니다')
        if not 1.0 <= self._joint6_move_seconds <= 30.0:
            raise ValueError('joint6_move_seconds는 1~30초여야 합니다')

    def _publish(self, text: str) -> None:
        self._status.publish(String(data=text))
        self.get_logger().info(text)

    def _target_callback(self, message: PoseStamped) -> None:
        with self._lock:
            self._latest_target = message
            self._target_received_at = time.monotonic()

    def _observation_reference_callback(self, message: PoseStamped) -> None:
        if message.header.frame_id == self._base_frame:
            with self._lock:
                self._observation_reference = message

    def _joint_callback(self, message: JointState) -> None:
        values = dict(zip(message.name, message.position))
        if all(name in values for name in JOINT_NAMES):
            with self._lock:
                self._latest_joints = [
                    float(values[name]) for name in JOINT_NAMES
                ]
                self._joints_received_at = time.monotonic()

    def _keypoint_callback(self, message: Float64) -> None:
        value = float(message.data)
        if math.isfinite(value):
            with self._lock:
                self._latest_keypoint_axis = value
                self._keypoint_received_at = time.monotonic()

    def _command_callback(self, message: String) -> None:
        command = message.data.strip().lower()
        valid_commands = (
            'coarse_xy',
            'coarse_xy_then_yaw',
            'refine_xy',
            'refine_yaw',
            'recover_z_once',
            'waypoint_pbvs_align',
        )
        if command not in valid_commands:
            self._publish(
                'REJECTED: 명령은 coarse_xy, coarse_xy_then_yaw, '
                'refine_xy, refine_yaw, recover_z_once, '
                'waypoint_pbvs_align 중 하나입니다'
            )
            return
        with self._lock:
            if self._executing:
                self._publish('REJECTED: 이전 이동이 실행 중입니다')
                return
            self._executing = True
        try:
            if not self._enabled:
                raise ValueError('enable_execution=false')
            if command == 'coarse_xy_then_yaw':
                self._execute_coarse_xy_then_yaw()
            elif command == 'refine_yaw':
                self._execute_yaw()
            elif command == 'recover_z_once':
                self._execute_z_recovery_once(attempt=1)
                time.sleep(self._z_recovery_settle)
                self._wait_for_reobservation()
                self._publish(
                    'EXECUTED: recover_z_once 완료, 포트를 다시 검출했습니다'
                )
            elif command == 'waypoint_pbvs_align':
                self._execute_waypoint_pbvs_align()
            else:
                self._execute_xy(command)
        except Exception as error:
            self._publish(f'REJECTED: {error}')
        finally:
            with self._lock:
                self._executing = False

    def _fresh(self, received_at: float | None, label: str) -> None:
        if received_at is None:
            raise ValueError(f'{label} 입력이 없습니다')
        age = time.monotonic() - received_at
        if not 0.0 <= age <= self._maximum_age:
            raise ValueError(f'{label} 입력이 오래됐습니다: age={age:.3f}s')

    def _current_flange(self):
        try:
            message = self._tf_buffer.lookup_transform(
                self._base_frame,
                self._flange_frame,
                Time(),
                timeout=Duration(seconds=1.0),
            )
        except TransformException as error:
            raise ValueError(f'현재 flange TF를 읽지 못했습니다: {error}') from error
        transform = message.transform
        return make_transform(
            (
                transform.translation.x,
                transform.translation.y,
                transform.translation.z,
            ),
            (
                transform.rotation.x,
                transform.rotation.y,
                transform.rotation.z,
                transform.rotation.w,
            ),
        )

    def _execute_xy(self, command: str) -> None:
        with self._lock:
            target = self._latest_target
            received_at = self._target_received_at
            observation_reference = self._observation_reference
        self._fresh(received_at, 'PBVS target')
        if target is None:
            raise ValueError('PBVS target 입력이 없습니다')
        if target.header.frame_id != self._base_frame:
            raise ValueError(f'목표 frame이 {self._base_frame}가 아닙니다')
        current = self._current_flange()
        goal_transform = pose_to_transform(target.pose)
        if self._use_observation_roll_pitch:
            if observation_reference is None:
                raise ValueError('초기 관측 Roll/Pitch 기준 자세가 없습니다')
            reference_transform = pose_to_transform(
                observation_reference.pose
            )
            orientation_target = apply_observation_roll_pitch_with_current_yaw(
                current,
                reference_transform,
            )
            goal_transform[:3, :3] = orientation_target[:3, :3]
            orientation_source = 'initial_observation'
        else:
            goal_transform[:3, :3] = current[:3, :3]
            orientation_source = 'current_flange'
        z_source = 'pbvs_target'
        if command == 'coarse_xy' and self._coarse_use_observation_z:
            if observation_reference is None:
                raise ValueError('coarse용 초기 관측 Z 기준 자세가 없습니다')
            reference_transform = pose_to_transform(
                observation_reference.pose
            )
            goal_transform[2, 3] = reference_transform[2, 3]
            z_source = 'initial_observation'
        elif command == 'refine_xy':
            goal_transform[2, 3] = current[2, 3]
            z_source = 'current_flange'
        lock_z_path = (
            command == 'refine_xy'
            or (command == 'coarse_xy' and self._coarse_use_observation_z)
        )
        delta_xy = goal_transform[:2, 3] - current[:2, 3]
        distance = math.hypot(float(delta_xy[0]), float(delta_xy[1]))
        if distance > self._maximum_xy_step:
            raise ValueError(
                f'XY 이동 {distance * 1000.0:.1f}mm가 제한 '
                f'{self._maximum_xy_step * 1000.0:.1f}mm를 초과합니다'
            )
        if distance < self._minimum_xy_step:
            raise ValueError(
                f'XY 이동 {distance * 1000.0:.1f}mm가 기기 최소 유효값 '
                f'{self._minimum_xy_step * 1000.0:.1f}mm 미만이므로 '
                '이동하지 않습니다'
            )
        execution_target = PoseStamped()
        execution_target.header = target.header
        execution_target.header.stamp = self.get_clock().now().to_msg()
        execution_target.pose = transform_to_pose(goal_transform)
        target_roll, target_pitch, target_yaw = rotation_to_rpy_degrees(
            goal_transform[:3, :3]
        )
        self._publish(
            f'{self._warning_delay:.1f}초 후 {command}: '
            f'dx={delta_xy[0] * 1000.0:+.1f}mm, '
            f'dy={delta_xy[1] * 1000.0:+.1f}mm, '
            f'z={goal_transform[2, 3] * 1000.0:.1f}mm, '
            f'z_source={z_source}, '
            f'target_rpy=[{target_roll:+.2f}, {target_pitch:+.2f}, '
            f'{target_yaw:+.2f}]deg, '
            f'roll_pitch_source={orientation_source}, '
            f'path_z_locked={lock_z_path}, '
            'path_roll_pitch_locked='
            f'{self._lock_roll_pitch_during_xy}'
        )
        time.sleep(self._warning_delay)
        actual = self._send_cartesian(
            execution_target,
            lock_z=lock_z_path,
            lock_roll_pitch=self._lock_roll_pitch_during_xy,
        )
        if self._use_observation_roll_pitch:
            self._validate_observation_roll_pitch(actual)
        with self._lock:
            self._latest_target = None
            self._target_received_at = None
        self._publish(f'EXECUTED: {command} 완료, 새 관측을 기다립니다')

    def _execute_coarse_xy_then_yaw(self) -> None:
        with self._lock:
            initial_axis = self._latest_keypoint_axis
            initial_axis_received_at = self._keypoint_received_at
        self._fresh(initial_axis_received_at, '초기 keypoint angle')
        if initial_axis is None:
            raise ValueError('초기 keypoint angle 입력이 없습니다')
        self._publish(
            '초기 관측 Yaw 저장: '
            f'keypoint_axis={initial_axis:+.3f}deg; '
            '이 값으로 즉시 회전하지 않고 이동 후 재관측합니다'
        )
        self._execute_xy('coarse_xy')
        time.sleep(self._post_coarse_yaw_delay)
        self._wait_for_reobservation_with_z_recovery()

        refine_cycle = 0
        while True:
            residual = self._current_xy_residual()
            if residual <= self._yaw_start_xy_tolerance:
                break
            if refine_cycle >= self._automatic_refine_xy_max_cycles:
                self._publish(
                    'EXECUTED: coarse 및 가시성 복구 완료, 자동 refine_xy '
                    f'{refine_cycle}회 후에도 XY 잔여오차 '
                    f'{residual * 1000.0:.1f}mm가 허용값 '
                    f'{self._yaw_start_xy_tolerance * 1000.0:.1f}mm를 '
                    '초과하여 Yaw 없이 중단합니다'
                )
                return
            refine_cycle += 1
            self._publish(
                f'자동 refine_xy {refine_cycle}/'
                f'{self._automatic_refine_xy_max_cycles}: '
                f'새 관측 XY 잔여오차={residual * 1000.0:.1f}mm'
            )
            self._execute_xy('refine_xy')
            time.sleep(self._post_coarse_yaw_delay)
            self._wait_for_reobservation_with_z_recovery()

        self._publish(
            '이동 후 재관측 XY가 Yaw 허용조건 안입니다: '
            f'{residual * 1000.0:.1f}mm, '
            f'automatic_refine_xy_cycles={refine_cycle}'
        )
        self._execute_yaw(
            axis_label='이동 후 재관측 keypoint angle',
            allow_already_converged=True,
        )
        self._publish(
            'EXECUTED: coarse_xy_then_yaw 자동 순서 완료. '
            'Yaw 후 포트 재관측은 요구하지 않고 현재 위치에서 정지합니다'
        )

    def _execute_yaw(
        self,
        *,
        axis_override: float | None = None,
        axis_label: str = 'keypoint angle',
        allow_already_converged: bool = False,
    ) -> None:
        with self._lock:
            joints = (
                None
                if self._latest_joints is None
                else list(self._latest_joints)
            )
            joints_received_at = self._joints_received_at
            latest_axis = self._latest_keypoint_axis
            axis_received_at = self._keypoint_received_at
        self._fresh(joints_received_at, '/joint_states')
        if axis_override is None:
            self._fresh(axis_received_at, axis_label)
            axis = latest_axis
        else:
            axis = float(axis_override)
        if joints is None or axis is None:
            raise ValueError('Joint6 Yaw 입력이 없습니다')
        self._require_yaw_xy_ready()
        yaw_step, converged = calibrated_keypoint_joint_step_rad(
            axis,
            self._desired_axis,
            self._keypoint_joint_gain,
            self._maximum_yaw_step,
            self._yaw_tolerance,
            self._keypoint_command_sign,
        )
        if converged:
            if allow_already_converged:
                self._publish(
                    '초기 관측 Yaw가 허용오차 안이므로 회전을 생략합니다: '
                    f'{axis:+.3f}deg'
                )
                return
            raise ValueError(
                f'영상 Yaw가 이미 허용오차 안입니다: {axis:+.3f}deg'
            )
        if abs(math.degrees(yaw_step)) < self._minimum_yaw_step:
            raise ValueError(
                f'Joint6 Yaw 이동 {math.degrees(yaw_step):+.2f}deg가 '
                f'기기 최소 유효값 {self._minimum_yaw_step:.2f}deg 미만이므로 '
                '이동하지 않습니다'
            )
        target_joints = list(joints)
        target_joints[-1] = joint6_yaw_target_rad(
            joints[-1],
            yaw_step,
            direction=self._joint6_direction,
            limit_deg=self._joint6_limit,
        )
        delta_deg = math.degrees(target_joints[-1] - joints[-1])
        self._publish(
            f'{self._warning_delay:.1f}초 후 refine_yaw: '
            f'image={axis:+.2f}deg, joint6_delta={delta_deg:+.2f}deg, '
            'roll_pitch_locked=False, '
            f'gain={self._keypoint_joint_gain:.3f}, source={axis_label}'
        )
        time.sleep(self._warning_delay)
        self._send_joint_target(target_joints)
        self._publish('EXECUTED: refine_yaw 완료, 새 관측을 기다립니다')

    def _wait_for_reobservation(self) -> None:
        with self._lock:
            self._latest_target = None
            self._target_received_at = None
            self._latest_keypoint_axis = None
            self._keypoint_received_at = None
        deadline = time.monotonic() + self._reobservation_timeout
        while rclpy.ok() and time.monotonic() < deadline:
            with self._lock:
                ready = (
                    self._latest_target is not None
                    and self._target_received_at is not None
                    and self._latest_keypoint_axis is not None
                    and self._keypoint_received_at is not None
                )
            if ready:
                return
            time.sleep(0.05)
        raise TimeoutError(
            '이동 후 새 포트 PBVS/Yaw 관측을 받지 못했습니다: '
            f'timeout={self._reobservation_timeout:.1f}s'
        )

    def _wait_for_reobservation_with_z_recovery(self) -> None:
        """재관측 실패 시 현재 XY를 유지하며 Z만 올려 다시 관측한다."""
        try:
            self._wait_for_reobservation()
            return
        except TimeoutError as initial_error:
            if not self._enable_automatic_z_recovery:
                raise initial_error
            self._publish(
                '포트 재관측 실패: 초기 관절자세 대신 Z-only 자동 복구를 '
                '시작합니다'
            )

        last_error: Exception | None = None
        for attempt in range(1, self._z_recovery_max_attempts + 1):
            try:
                self._execute_z_recovery_once(attempt=attempt)
                time.sleep(self._z_recovery_settle)
                self._wait_for_reobservation()
                self._publish(
                    f'Z-only 복구 {attempt}회 후 포트를 다시 검출했습니다'
                )
                return
            except Exception as error:
                last_error = error
                self._publish(
                    f'Z-only 복구 {attempt}/{self._z_recovery_max_attempts} '
                    f'후에도 재관측 실패: {error}'
                )
        raise TimeoutError(
            'Z-only 복구 최대 횟수 후에도 포트를 재검출하지 못했습니다: '
            f'{last_error}'
        )

    def _execute_z_recovery_once(self, *, attempt: int) -> None:
        with self._lock:
            observation_reference = self._observation_reference
        if observation_reference is None:
            raise ValueError('Z-only 복구용 초기 관측 기준 자세가 없습니다')
        current = self._current_flange()
        reference = pose_to_transform(observation_reference.pose)
        recovery_target = raised_z_recovery_target(
            current,
            reference,
            self._z_recovery_step,
        )
        if self._z_recovery_use_observation_roll_pitch:
            orientation_target = apply_observation_roll_pitch_with_current_yaw(
                current,
                reference,
            )
            recovery_target[:3, :3] = orientation_target[:3, :3]
            orientation_source = 'initial_observation'
        else:
            orientation_source = 'current_flange'

        target = PoseStamped()
        target.header.frame_id = self._base_frame
        target.header.stamp = self.get_clock().now().to_msg()
        target.pose = transform_to_pose(recovery_target)
        dz = float(recovery_target[2, 3] - current[2, 3])
        self._publish(
            f'{self._warning_delay:.1f}초 후 Z-only 복구 {attempt}: '
            f'dz={dz * 1000.0:+.1f}mm, '
            f'target_z={recovery_target[2, 3] * 1000.0:.1f}mm, '
            f'upper_limit=observation_z({reference[2, 3] * 1000.0:.1f}mm), '
            f'mode={self._z_recovery_cartesian_mode}, '
            f'roll_pitch_source={orientation_source}, '
            'path_roll_pitch_locked='
            f'{self._z_recovery_lock_roll_pitch}'
        )
        time.sleep(self._warning_delay)
        actual = self._send_cartesian(
            target,
            lock_z=False,
            lock_roll_pitch=self._z_recovery_lock_roll_pitch,
            mode_override=self._z_recovery_cartesian_mode,
        )
        if self._z_recovery_use_observation_roll_pitch:
            self._validate_observation_roll_pitch(actual)
        self._publish(
            f'EXECUTED: Z-only 복구 {attempt} 이동 완료, 새 관측을 기다립니다'
        )

    def _execute_waypoint_pbvs_align(self) -> None:
        """PBVS 절대 목표까지 제한된 waypoint로 나눠 stop-and-go로 접근한다.

        한 번의 명령 안에서 최대 waypoint_max_cycles회까지 XY<=15mm/Z<=10mm
        이동과 재관측을 반복한다. 포트가 사라지면 임의로 Z를 올리지 않고
        last_visible_flange_pose(방금 지나온 도달 가능한 자세)로 복귀한다.
        """
        try:
            self._last_visible_flange = self._current_flange_if_port_visible()
        except ValueError as error:
            if self._last_visible_flange is None:
                raise ValueError(
                    f'포트가 보이지 않고 이전 last_visible_flange_pose도 '
                    f'없습니다: {error}'
                ) from error
            self._publish(
                f'포트가 현재 보이지 않아 이전 last_visible_flange_pose를 '
                f'계속 사용합니다: {error}'
            )

        total_xy_moved = 0.0
        total_xy_budget = (
            self._waypoint_max_cycles * self._waypoint_maximum_xy_step
        )
        converged = False
        cycle = 0
        while cycle < self._waypoint_max_cycles:
            cycle += 1
            try:
                residual = self._require_yaw_xy_ready()
            except ValueError as error:
                self._recover_to_last_visible_pose(cycle, error)
                self._publish(
                    'EXECUTED: last_visible_flange_pose 복귀 후 포트를 '
                    '다시 검출했습니다. 자동 연속 이동을 중단합니다. 더 '
                    '작은 waypoint나 refine_xy로 이어가세요'
                )
                return
            self._publish(
                f'waypoint {cycle}/{self._waypoint_max_cycles} 재관측 PBVS '
                f'xy_residual={residual * 1000.0:.1f}mm '
                f'(yaw_start_xy_tolerance={self._yaw_start_xy_tolerance * 1000.0:.1f}mm)'
            )
            if residual <= self._yaw_start_xy_tolerance:
                converged = True
                break
            if total_xy_moved >= total_xy_budget:
                self._publish(
                    'EXECUTED: waypoint_pbvs_align 최대 총 이동량 '
                    f'{total_xy_budget * 1000.0:.1f}mm 도달, XY 잔여오차 '
                    f'{residual * 1000.0:.1f}mm에서 자동 이동을 중단합니다. '
                    'refine_xy로 이어가세요'
                )
                return
            try:
                total_xy_moved += self._execute_single_waypoint_step(cycle)
            except TimeoutError as error:
                self._recover_to_last_visible_pose(cycle, error)
                self._publish(
                    'EXECUTED: last_visible_flange_pose 복귀 후 포트를 '
                    '다시 검출했습니다. 자동 연속 이동을 중단합니다. 더 '
                    '작은 waypoint나 refine_xy로 이어가세요'
                )
                return

        if not converged:
            self._publish(
                'EXECUTED: waypoint_pbvs_align 최대 반복 횟수 '
                f'{self._waypoint_max_cycles}회 도달, XY가 아직 허용값 밖이라 '
                '자동 이동을 중단합니다. 필요하면 다시 실행하세요'
            )
            return

        self._execute_waypoint_yaw_phase()

    def _current_flange_if_port_visible(self):
        """PBVS target이 지금 fresh하면 현재 flange를 포트 가시 자세로 본다."""
        with self._lock:
            target = self._latest_target
            received_at = self._target_received_at
        self._fresh(received_at, 'PBVS target')
        if target is None:
            raise ValueError('PBVS target 입력이 없습니다')
        if target.header.frame_id != self._base_frame:
            raise ValueError(f'목표 frame이 {self._base_frame}가 아닙니다')
        return self._current_flange()

    def _execute_single_waypoint_step(self, cycle: int) -> float:
        with self._lock:
            target = self._latest_target
            observation_reference = self._observation_reference
        if target is None or target.header.frame_id != self._base_frame:
            raise ValueError('waypoint 목표 PBVS target이 유효하지 않습니다')
        current = self._current_flange()
        target_transform = pose_to_transform(target.pose)
        waypoint_xyz, xy_distance, xy_step_skipped = (
            limited_waypoint_translation(
                current[:3, 3],
                target_transform[:3, 3],
                self._waypoint_maximum_xy_step,
                self._waypoint_maximum_z_step,
                self._minimum_xy_step,
            )
        )
        if xy_step_skipped:
            raise ValueError(
                f'waypoint XY 잔여거리 {xy_distance * 1000.0:.1f}mm가 기기 '
                f'최소 유효값 {self._minimum_xy_step * 1000.0:.1f}mm '
                '미만입니다'
            )
        waypoint_transform = current.copy()
        waypoint_transform[:3, 3] = waypoint_xyz
        if self._use_observation_roll_pitch:
            if observation_reference is None:
                raise ValueError('초기 관측 Roll/Pitch 기준 자세가 없습니다')
            reference_transform = pose_to_transform(
                observation_reference.pose
            )
            orientation_target = apply_observation_roll_pitch_with_current_yaw(
                current,
                reference_transform,
            )
            waypoint_transform[:3, :3] = orientation_target[:3, :3]
            orientation_source = 'initial_observation'
        else:
            orientation_source = 'current_flange'

        execution_target = PoseStamped()
        execution_target.header = target.header
        execution_target.header.stamp = self.get_clock().now().to_msg()
        execution_target.pose = transform_to_pose(waypoint_transform)
        delta_xy = waypoint_transform[:2, 3] - current[:2, 3]
        dz = float(waypoint_transform[2, 3] - current[2, 3])
        self._publish(
            f'{self._warning_delay:.1f}초 후 waypoint '
            f'{cycle}/{self._waypoint_max_cycles}: '
            f'dx={delta_xy[0] * 1000.0:+.1f}mm, '
            f'dy={delta_xy[1] * 1000.0:+.1f}mm, '
            f'dz={dz * 1000.0:+.1f}mm, '
            f'xy_to_target={xy_distance * 1000.0:.1f}mm, '
            f'roll_pitch_source={orientation_source}, '
            'path_roll_pitch_locked=False'
        )
        time.sleep(self._warning_delay)
        actual = self._send_cartesian(
            execution_target,
            lock_z=False,
            lock_roll_pitch=False,
        )
        if self._use_observation_roll_pitch:
            self._validate_observation_roll_pitch(actual)

        time.sleep(self._waypoint_settle)
        self._wait_for_reobservation()
        self._last_visible_flange = self._current_flange()
        moved_xy = float(math.hypot(float(delta_xy[0]), float(delta_xy[1])))
        self._publish(
            f'waypoint {cycle}/{self._waypoint_max_cycles} 완료, 새 관측으로 '
            f'last_visible_flange_pose를 갱신했습니다: '
            f'moved_xy={moved_xy * 1000.0:.1f}mm'
        )
        return moved_xy

    def _recover_to_last_visible_pose(
        self, cycle: int, error: Exception
    ) -> None:
        """재관측 실패 시 last_visible_flange_pose로 Cartesian 복귀한다.

        이 자세는 방금 이 명령이 실제로 지나온 도달 가능한 자세이므로
        임의의 Z-only 상승 복구와 달리 IK 실패 위험이 없다.
        """
        self._publish(
            f'waypoint {cycle}/{self._waypoint_max_cycles} 재관측 실패: '
            f'{error}'
        )
        if (
            not self._enable_last_visible_pose_recovery
            or self._last_visible_flange is None
        ):
            raise RuntimeError(
                'waypoint_pbvs_align 중단: last_visible_flange_pose 복귀를 '
                f'사용할 수 없습니다: {error}'
            )
        self._publish(
            'last_visible_flange_pose로 복귀합니다: 방금 지나온 도달 가능한 '
            '자세입니다'
        )
        recovery_target = PoseStamped()
        recovery_target.header.frame_id = self._base_frame
        recovery_target.header.stamp = self.get_clock().now().to_msg()
        recovery_target.pose = transform_to_pose(self._last_visible_flange)
        self._send_cartesian(
            recovery_target,
            lock_z=False,
            lock_roll_pitch=False,
        )
        time.sleep(self._waypoint_settle)
        self._wait_for_reobservation()

    def _execute_waypoint_yaw_phase(self) -> None:
        try:
            self._execute_yaw(
                axis_label='waypoint 수렴 후 keypoint angle',
                allow_already_converged=True,
            )
        except ValueError as error:
            self._publish(
                f'EXECUTED: waypoint_pbvs_align XY 수렴, Yaw 생략: {error}'
            )
            return
        time.sleep(self._post_coarse_yaw_delay)
        self._wait_for_reobservation()
        self._last_visible_flange = self._current_flange()
        residual = self._require_yaw_xy_ready()
        if residual > self._yaw_start_xy_tolerance:
            self._publish(
                f'Yaw 후 XY 잔여오차 {residual * 1000.0:.1f}mm가 허용값을 '
                '넘어 refine_xy를 한 번 실행합니다'
            )
            self._execute_xy('refine_xy')
            self._publish(
                'EXECUTED: waypoint_pbvs_align 완료 (Yaw 후 refine_xy 포함)'
            )
            return
        self._publish(
            'EXECUTED: waypoint_pbvs_align 완료, XY/Yaw 모두 재관측으로 '
            f'확인했습니다: xy_residual={residual * 1000.0:.1f}mm'
        )

    def _require_yaw_xy_ready(self) -> float:
        residual = self._current_xy_residual()
        if residual > self._yaw_start_xy_tolerance:
            raise ValueError(
                f'Yaw 시작 전 XY 오차 {residual * 1000.0:.1f}mm가 '
                f'허용값 {self._yaw_start_xy_tolerance * 1000.0:.1f}mm를 '
                '초과합니다'
            )
        return residual

    def _current_xy_residual(self) -> float:
        """최신 PBVS 목표와 현재 flange 사이의 XY 거리만 반환한다."""
        with self._lock:
            target = self._latest_target
            received_at = self._target_received_at
        self._fresh(received_at, 'PBVS target')
        if target is None:
            raise ValueError('PBVS target 입력이 없습니다')
        if target.header.frame_id != self._base_frame:
            raise ValueError(f'목표 frame이 {self._base_frame}가 아닙니다')
        current = self._current_flange()
        target_transform = pose_to_transform(target.pose)
        return float(
            math.hypot(
                target_transform[0, 3] - current[0, 3],
                target_transform[1, 3] - current[1, 3],
            )
        )

    def _validate_observation_roll_pitch(self, actual: PoseStamped) -> None:
        with self._lock:
            observation_reference = self._observation_reference
        if observation_reference is None:
            raise ValueError('초기 관측 Roll/Pitch 기준 자세가 없습니다')
        actual_transform = pose_to_transform(actual.pose)
        reference_transform = pose_to_transform(observation_reference.pose)
        actual_rpy = rotation_to_rpy_degrees(actual_transform[:3, :3])
        reference_rpy = rotation_to_rpy_degrees(reference_transform[:3, :3])
        roll_error = abs(
            (actual_rpy[0] - reference_rpy[0] + 180.0) % 360.0 - 180.0
        )
        pitch_error = abs(
            (actual_rpy[1] - reference_rpy[1] + 180.0) % 360.0 - 180.0
        )
        if max(roll_error, pitch_error) > self._observation_roll_pitch_tolerance:
            raise RuntimeError(
                '초기 관측 Roll/Pitch 목표의 정지 후 허용오차를 초과했습니다: '
                f'roll={roll_error:.2f}deg, pitch={pitch_error:.2f}deg, '
                f'limit={self._observation_roll_pitch_tolerance:.2f}deg'
            )
        self._publish(
            '초기 관측 Roll/Pitch 정지 후 확인 완료: '
            f'roll_error={roll_error:.2f}deg, '
            f'pitch_error={pitch_error:.2f}deg'
        )

    def _wait_future(self, future, timeout_seconds: float):
        deadline = time.monotonic() + timeout_seconds
        while rclpy.ok() and not future.done():
            if time.monotonic() >= deadline:
                raise TimeoutError('ROS action 응답 시간이 초과됐습니다')
            time.sleep(0.05)
        if future.exception() is not None:
            raise RuntimeError(str(future.exception()))
        return future.result()

    def _send_cartesian(
        self,
        target: PoseStamped,
        *,
        lock_z: bool,
        lock_roll_pitch: bool,
        mode_override: int | None = None,
    ) -> PoseStamped:
        if not self._cartesian_action.wait_for_server(timeout_sec=3.0):
            raise RuntimeError(f'action server가 없습니다: {CARTESIAN_ACTION}')
        goal = CartesianMove.Goal()
        goal.target = target
        goal.speed = self._cartesian_speed
        goal.mode = (
            self._cartesian_mode
            if mode_override is None
            else int(mode_override)
        )
        goal.lock_z = lock_z
        goal.lock_roll_pitch = lock_roll_pitch
        handle = self._wait_future(
            self._cartesian_action.send_goal_async(goal),
            5.0,
        )
        if not handle.accepted:
            raise RuntimeError('Cartesian 목표가 거절됐습니다')
        wrapped = self._wait_future(
            handle.get_result_async(),
            self._cartesian_timeout,
        )
        if not wrapped.result.success:
            raise RuntimeError(wrapped.result.message)
        return wrapped.result.actual

    def _send_joint_target(self, target_joints: list[float]) -> None:
        if not self._joint_action.wait_for_server(timeout_sec=3.0):
            raise RuntimeError(f'action server가 없습니다: {JOINT_ACTION}')
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINT_NAMES)
        point = JointTrajectoryPoint()
        point.positions = target_joints
        point.time_from_start = Duration(
            seconds=self._joint6_move_seconds
        ).to_msg()
        goal.trajectory.points = [point]
        handle = self._wait_future(
            self._joint_action.send_goal_async(goal),
            5.0,
        )
        if not handle.accepted:
            raise RuntimeError('Joint6 목표가 거절됐습니다')
        wrapped = self._wait_future(
            handle.get_result_async(),
            self._joint6_move_seconds + 20.0,
        )
        if wrapped.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            raise RuntimeError(
                wrapped.result.error_string
                or f'Joint6 error={wrapped.result.error_code}'
            )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = PbvsStepExecutorNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
