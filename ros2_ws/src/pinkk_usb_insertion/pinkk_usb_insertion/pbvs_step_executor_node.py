"""PBVS 거친 이동과 영상 기반 미세 보정을 제조사 API bridge로 실행한다."""

from __future__ import annotations

import math
import threading
import time

from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseStamped
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
    calibrated_keypoint_joint_step_rad,
    joint6_yaw_target_rad,
)
from .geometry.transforms import make_transform
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
        self.declare_parameter('maximum_xy_step_m', 0.100)
        self.declare_parameter('cartesian_speed', 10)
        self.declare_parameter('cartesian_mode', 0)
        self.declare_parameter('cartesian_timeout_seconds', 100.0)
        self.declare_parameter('warning_delay_seconds', 3.0)
        self.declare_parameter('post_coarse_yaw_delay_seconds', 0.5)
        self.declare_parameter('keypoint_desired_axis_deg', 0.0)
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
        self._maximum_xy_step = float(
            self.get_parameter('maximum_xy_step_m').value
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
        self._desired_axis = float(
            self.get_parameter('keypoint_desired_axis_deg').value
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
        self._latest_joints: list[float] | None = None
        self._joints_received_at: float | None = None
        self._latest_keypoint_axis: float | None = None
        self._keypoint_received_at: float | None = None
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
            'commands=coarse_xy/coarse_xy_then_yaw/refine_xy/refine_yaw, '
            f'max_xy={self._maximum_xy_step * 1000.0:.1f}mm, '
            f'yaw_step={self._maximum_yaw_step:.1f}deg, '
            f'yaw_joint_gain={self._keypoint_joint_gain:.3f}, '
            f'joint6_direction={self._joint6_direction:+.0f}'
        )

    def _validate_parameters(self) -> None:
        if not 0.0 < self._maximum_age <= 5.0:
            raise ValueError('maximum_input_age_seconds는 0~5초 범위여야 합니다')
        if not 0.0 < self._maximum_xy_step <= 0.200:
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
        finite = (
            self._desired_axis,
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
        )
        if command not in valid_commands:
            self._publish(
                'REJECTED: 명령은 coarse_xy, coarse_xy_then_yaw, '
                'refine_xy, refine_yaw 중 하나입니다'
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
        self._fresh(received_at, 'PBVS target')
        if target is None:
            raise ValueError('PBVS target 입력이 없습니다')
        if target.header.frame_id != self._base_frame:
            raise ValueError(f'목표 frame이 {self._base_frame}가 아닙니다')
        current = self._current_flange()
        goal_transform = pose_to_transform(target.pose)
        goal_transform[:3, :3] = current[:3, :3]
        if command == 'refine_xy':
            goal_transform[2, 3] = current[2, 3]
        delta_xy = goal_transform[:2, 3] - current[:2, 3]
        distance = math.hypot(float(delta_xy[0]), float(delta_xy[1]))
        if distance > self._maximum_xy_step:
            raise ValueError(
                f'XY 이동 {distance * 1000.0:.1f}mm가 제한 '
                f'{self._maximum_xy_step * 1000.0:.1f}mm를 초과합니다'
            )
        if distance < 0.0002:
            raise ValueError('XY 오차가 0.2mm 미만이므로 이동하지 않습니다')
        execution_target = PoseStamped()
        execution_target.header = target.header
        execution_target.header.stamp = self.get_clock().now().to_msg()
        execution_target.pose = transform_to_pose(goal_transform)
        self._publish(
            f'{self._warning_delay:.1f}초 후 {command}: '
            f'dx={delta_xy[0] * 1000.0:+.1f}mm, '
            f'dy={delta_xy[1] * 1000.0:+.1f}mm, '
            f'z={goal_transform[2, 3] * 1000.0:.1f}mm'
        )
        time.sleep(self._warning_delay)
        self._send_cartesian(
            execution_target,
            lock_z=False,
            lock_roll_pitch=True,
        )
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
            f'keypoint_axis={initial_axis:+.3f}deg'
        )
        self._execute_xy('coarse_xy')
        time.sleep(self._post_coarse_yaw_delay)
        self._execute_yaw(
            axis_override=initial_axis,
            axis_label='초기 관측 keypoint angle',
            allow_already_converged=True,
        )
        self._publish(
            'EXECUTED: coarse_xy_then_yaw 완료, 새 관측을 기다립니다'
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
    ) -> None:
        if not self._cartesian_action.wait_for_server(timeout_sec=3.0):
            raise RuntimeError(f'action server가 없습니다: {CARTESIAN_ACTION}')
        goal = CartesianMove.Goal()
        goal.target = target
        goal.speed = self._cartesian_speed
        goal.mode = self._cartesian_mode
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
