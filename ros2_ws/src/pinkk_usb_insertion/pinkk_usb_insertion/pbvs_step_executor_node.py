"""사용자 승인 한 번에 최신 PBVS 목표를 한 번만 실행한다."""

from __future__ import annotations

import threading
import time

from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetPositionIK
from pinkk_usb_insertion_interfaces.action import CartesianMove
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener

from .configuration import load_yaml
from .control.pbvs_step_safety import (
    make_fixed_z_xy_waypoints,
    validate_fixed_z_pbvs_step,
    validate_joint_step,
)
from .geometry.transforms import make_transform
from .port_pose_node import _default_config
from .ros_utils import pose_to_transform, transform_to_pose


JOINT_NAMES = (
    'joint2_to_joint1',
    'joint3_to_joint2',
    'joint4_to_joint3',
    'joint5_to_joint4',
    'joint6_to_joint5',
    'joint6output_to_joint6',
)
IK_SERVICE = '/compute_ik'
CARTESIAN_ACTION_NAME = '/robot_arm/cartesian_move'


class PbvsStepExecutorNode(Node):
    """명시적인 execute_once 명령에만 최신 고정-Z 목표를 실행한다."""

    def __init__(self) -> None:
        super().__init__('pinkk_pbvs_step_executor_node')
        self.declare_parameter('control_config', _default_config('insertion_control.yaml'))
        self.declare_parameter('enable_test_execution', False)
        control = load_yaml(str(self.get_parameter('control_config').value))
        frames = control['frames']
        pbvs = control['pbvs']
        test = control['pbvs_test_execution']
        self._enabled = bool(self.get_parameter('enable_test_execution').value)
        self._base_frame = str(frames['base'])
        self._flange_frame = str(frames['flange'])
        self._maximum_step = float(pbvs['maximum_xy_step_m'])
        self._maximum_age = float(test['maximum_target_age_seconds'])
        self._maximum_z_change = float(test['maximum_z_change_m'])
        self._maximum_orientation_change = float(
            test['maximum_orientation_change_deg']
        )
        self._waypoint_spacing = float(test['cartesian_waypoint_spacing_m'])
        self._maximum_joint_step_deg = float(test['maximum_joint_step_deg'])
        self._post_move_z_tolerance = float(test['post_move_z_tolerance_m'])
        self._post_move_orientation_tolerance = float(
            test['post_move_orientation_tolerance_deg']
        )
        self._cartesian_speed = int(test['cartesian_speed'])
        self._cartesian_mode = int(test['cartesian_mode'])
        self._cartesian_timeout = float(test['cartesian_action_timeout_seconds'])
        self._latest_target: PoseStamped | None = None
        self._latest_converged: bool | None = None
        self._latest_joints: list[float] | None = None
        self._executing = False
        self._lock = threading.Lock()

        callback_group = ReentrantCallbackGroup()
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._ik = self.create_client(
            GetPositionIK, IK_SERVICE, callback_group=callback_group
        )
        self._action = ActionClient(
            self,
            CartesianMove,
            CARTESIAN_ACTION_NAME,
            callback_group=callback_group,
        )
        self._status_publisher = self.create_publisher(
            String, '/robot_arm/pbvs/execution_status', 10
        )
        self.create_subscription(
            PoseStamped,
            '/robot_arm/pbvs/target_flange_pose',
            self._target_callback,
            10,
            callback_group=callback_group,
        )
        self.create_subscription(
            Bool,
            '/robot_arm/pbvs/converged',
            self._converged_callback,
            10,
            callback_group=callback_group,
        )
        self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_callback,
            10,
            callback_group=callback_group,
        )
        self.create_subscription(
            String,
            '/robot_arm/pbvs/step_command',
            self._command_callback,
            10,
            callback_group=callback_group,
        )
        mode = '실행 허용' if self._enabled else '실행 차단'
        self.get_logger().warning(f'PBVS Cartesian 단발 실행기 시작: {mode}')

    def _publish_status(self, text: str) -> None:
        self._status_publisher.publish(String(data=text))
        self.get_logger().info(text)

    def _target_callback(self, message: PoseStamped) -> None:
        with self._lock:
            self._latest_target = message

    def _converged_callback(self, message: Bool) -> None:
        with self._lock:
            self._latest_converged = bool(message.data)

    def _joint_callback(self, message: JointState) -> None:
        values = dict(zip(message.name, message.position))
        if all(name in values for name in JOINT_NAMES):
            with self._lock:
                self._latest_joints = [float(values[name]) for name in JOINT_NAMES]

    def _wait_future(self, future, timeout_seconds: float):
        deadline = time.monotonic() + timeout_seconds
        while rclpy.ok() and not future.done():
            if time.monotonic() >= deadline:
                raise TimeoutError('ROS 응답 대기 시간이 초과됐습니다')
            time.sleep(0.05)
        if future.exception() is not None:
            raise RuntimeError(str(future.exception()))
        return future.result()

    def _command_callback(self, message: String) -> None:
        if message.data.strip().lower() != 'execute_once':
            self._publish_status('REJECTED: 명령은 execute_once만 허용합니다')
            return
        with self._lock:
            if self._executing:
                self._publish_status('REJECTED: 이전 이동이 실행 중입니다')
                return
            self._executing = True
        try:
            self._execute_latest_target()
        except Exception as error:
            self._publish_status(f'REJECTED: {error}')
        finally:
            with self._lock:
                self._latest_target = None
                self._latest_converged = None
                self._executing = False

    def _execute_latest_target(self) -> None:
        if not self._enabled:
            raise ValueError('enable_test_execution=false')
        with self._lock:
            target = self._latest_target
            converged = self._latest_converged
            joints = None if self._latest_joints is None else list(self._latest_joints)
        if target is None or converged is None:
            raise ValueError('새 PBVS 목표와 converged 결과가 모두 필요합니다')
        if converged:
            raise ValueError('이미 PBVS 허용오차 안이므로 이동하지 않습니다')
        if joints is None:
            raise ValueError('/joint_states를 받지 못했습니다')
        if target.header.frame_id != self._base_frame:
            raise ValueError(f'목표 frame이 {self._base_frame}가 아닙니다')
        stamp_seconds = target.header.stamp.sec + target.header.stamp.nanosec * 1e-9
        age = self.get_clock().now().nanoseconds * 1e-9 - stamp_seconds
        if not 0.0 <= age <= self._maximum_age:
            raise ValueError(f'PBVS 목표가 오래됐습니다: age={age:.3f}s')

        try:
            current_message = self._tf_buffer.lookup_transform(
                self._base_frame,
                self._flange_frame,
                Time(),
                timeout=Duration(seconds=1.0),
            )
        except TransformException as error:
            raise ValueError(f'현재 flange TF를 읽지 못했습니다: {error}') from error
        transform = current_message.transform
        current = make_transform(
            (transform.translation.x, transform.translation.y, transform.translation.z),
            (
                transform.rotation.x,
                transform.rotation.y,
                transform.rotation.z,
                transform.rotation.w,
            ),
        )
        validation = validate_fixed_z_pbvs_step(
            current,
            pose_to_transform(target.pose),
            self._maximum_step,
            self._maximum_z_change,
            self._maximum_orientation_change,
        )
        if validation.xy_distance_m < 1e-5:
            raise ValueError('XY 이동량이 너무 작습니다')
        waypoints = make_fixed_z_xy_waypoints(
            current,
            pose_to_transform(target.pose),
            self._waypoint_spacing,
        )
        if not self._ik.wait_for_service(timeout_sec=3.0):
            raise RuntimeError(f'IK service가 없습니다: {IK_SERVICE}')
        if not self._action.wait_for_server(timeout_sec=3.0):
            raise RuntimeError(f'action server가 없습니다: {CARTESIAN_ACTION_NAME}')

        solutions: list[list[float]] = []
        seed = joints
        maximum_joint_change = 0.0
        for index, waypoint in enumerate(waypoints, start=1):
            waypoint_pose = PoseStamped()
            waypoint_pose.header.frame_id = self._base_frame
            waypoint_pose.header.stamp = self.get_clock().now().to_msg()
            waypoint_pose.pose = transform_to_pose(waypoint)
            solution = self._solve_ik(waypoint_pose, seed)
            joint_change = validate_joint_step(
                seed, solution, self._maximum_joint_step_deg
            )
            maximum_joint_change = max(maximum_joint_change, joint_change)
            solutions.append(solution)
            seed = solution

        self._publish_status(
            f'Cartesian 사전검사 통과: waypoints={len(waypoints)}, '
            f'spacing≤{self._waypoint_spacing * 1000.0:.1f}mm, '
            f'max_joint_step={maximum_joint_change:.3f}deg'
        )

        self._publish_status(
            '3초 후 PBVS 단발 이동: '
            f'dx={validation.delta_x_m * 1000.0:+.3f}mm, '
            f'dy={validation.delta_y_m * 1000.0:+.3f}mm, '
            f'distance={validation.xy_distance_m * 1000.0:.3f}mm'
        )
        time.sleep(3.0)
        self._execute_cartesian_goal(target)
        self._verify_actual_fixed_z(current)
        self._publish_status(
            'EXECUTED: 연속 Cartesian 이동 완료; Z·자세 검사 통과. '
            '이전 목표를 폐기하고 새 관측을 기다립니다'
        )

    def _solve_ik(
        self, target: PoseStamped, seed: list[float]
    ) -> list[float]:
        request = GetPositionIK.Request()
        request.ik_request.group_name = 'arm_group'
        request.ik_request.ik_link_name = self._flange_frame
        request.ik_request.pose_stamped = target
        request.ik_request.robot_state.joint_state.name = list(JOINT_NAMES)
        request.ik_request.robot_state.joint_state.position = seed
        request.ik_request.robot_state.is_diff = True
        request.ik_request.avoid_collisions = True
        request.ik_request.timeout = Duration(seconds=2.0).to_msg()
        response = self._wait_future(self._ik.call_async(request), 5.0)
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(f'MoveIt IK 실패: code={response.error_code.val}')
        solution_map = dict(
            zip(response.solution.joint_state.name, response.solution.joint_state.position)
        )
        if not all(name in solution_map for name in JOINT_NAMES):
            raise RuntimeError('IK 결과에 필요한 관절이 없습니다')
        return [float(solution_map[name]) for name in JOINT_NAMES]

    def _execute_cartesian_goal(self, target: PoseStamped) -> None:
        goal = CartesianMove.Goal()
        goal.target = target
        goal.speed = self._cartesian_speed
        goal.mode = self._cartesian_mode
        goal.lock_z = True
        goal.lock_roll_pitch = True
        handle = self._wait_future(self._action.send_goal_async(goal), 5.0)
        if not handle.accepted:
            raise RuntimeError('Cartesian 목표가 거절됐습니다')
        wrapped = self._wait_future(
            handle.get_result_async(), self._cartesian_timeout
        )
        if not wrapped.result.success:
            raise RuntimeError(
                f'Cartesian 이동 실패: {wrapped.result.message}'
            )

    def _verify_actual_fixed_z(self, start_transform) -> None:
        try:
            message = self._tf_buffer.lookup_transform(
                self._base_frame,
                self._flange_frame,
                Time(),
                timeout=Duration(seconds=1.0),
            )
        except TransformException as error:
            raise RuntimeError(f'이동 후 flange TF를 읽지 못했습니다: {error}') from error
        transform = message.transform
        actual = make_transform(
            (transform.translation.x, transform.translation.y, transform.translation.z),
            (
                transform.rotation.x,
                transform.rotation.y,
                transform.rotation.z,
                transform.rotation.w,
            ),
        )
        validate_fixed_z_pbvs_step(
            start_transform,
            actual,
            self._maximum_step + 0.002,
            self._post_move_z_tolerance,
            self._post_move_orientation_tolerance,
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
