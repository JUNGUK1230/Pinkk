"""로봇을 움직이지 않고 고정-Z XY 목표의 MoveIt IK 안전성을 검사한다."""

from __future__ import annotations

import argparse
import math
import sys
import time
from dataclasses import dataclass

from geometry_msgs.msg import PoseStamped

from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetPositionFK, GetPositionIK, GetStateValidity

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time

from sensor_msgs.msg import JointState

from tf2_ros import Buffer, TransformException, TransformListener

from .control.moveit_ik_safety import make_locked_xy_target, validate_fk_pose
from .control.pbvs_step_safety import (
    make_fixed_z_xy_waypoints,
    validate_joint_step,
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


@dataclass(frozen=True)
class IkStepPlan:
    """검증을 모두 통과한 한 번의 고정-Z IK 이동 계획."""

    current_transform: object
    target_transform: object
    start_joints: list[float]
    target_joints: list[float]
    waypoint_count: int
    maximum_joint_change_deg: float
    fk_position_error_m: float
    fk_z_error_m: float
    fk_orientation_error_deg: float


class MoveItIkDryRunNode(Node):
    """IK·충돌·FK 검사만 수행하며 제어 토픽과 액션은 생성하지 않는다."""

    def __init__(
        self,
        node_name: str = 'pinkk_moveit_ik_dry_run',
    ) -> None:
        """서비스 client와 현재 상태 입력만 준비한다."""
        super().__init__(node_name)
        self._base_frame = 'g_base'
        self._flange_frame = 'joint6_flange'
        self._group_name = 'arm_group'
        self._latest_joints: list[float] | None = None
        self._joint_received_at: float | None = None

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._ik_client = self.create_client(GetPositionIK, '/compute_ik')
        self._validity_client = self.create_client(
            GetStateValidity,
            '/check_state_validity',
        )
        self._fk_client = self.create_client(GetPositionFK, '/compute_fk')
        self.create_subscription(
            JointState,
            '/joint_states',
            self._on_joints,
            10,
        )

    def _on_joints(self, message: JointState) -> None:
        values = dict(zip(message.name, message.position))
        if all(name in values for name in JOINT_NAMES):
            self._latest_joints = [float(values[name]) for name in JOINT_NAMES]
            self._joint_received_at = time.monotonic()

    def _wait_future(self, future, timeout_seconds: float):
        rclpy.spin_until_future_complete(
            self,
            future,
            timeout_sec=timeout_seconds,
        )
        if not future.done():
            raise TimeoutError(
                f'ROS service 응답이 {timeout_seconds:.1f}초 안에 없습니다'
            )
        if future.exception() is not None:
            raise RuntimeError(str(future.exception()))
        return future.result()

    def _wait_for_inputs(
        self,
        timeout_seconds: float,
    ) -> tuple[list[float], object]:
        deadline = time.monotonic() + timeout_seconds
        transform = None
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                transform = self._tf_buffer.lookup_transform(
                    self._base_frame, self._flange_frame, Time()
                )
            except TransformException:
                continue
            if self._latest_joints is not None:
                break
        if self._latest_joints is None:
            raise RuntimeError('/joint_states를 받지 못했습니다')
        if transform is None:
            raise RuntimeError(
                f'{self._base_frame} → {self._flange_frame} TF를 읽지 못했습니다'
            )
        if (
            self._joint_received_at is None
            or time.monotonic() - self._joint_received_at > 1.0
        ):
            raise RuntimeError('/joint_states가 1초 이상 갱신되지 않았습니다')
        return list(self._latest_joints), transform

    def _wait_for_services(self) -> None:
        services = (
            (self._ik_client, '/compute_ik'),
            (self._validity_client, '/check_state_validity'),
            (self._fk_client, '/compute_fk'),
        )
        missing = [
            name for client, name in services
            if not client.wait_for_service(timeout_sec=3.0)
        ]
        if missing:
            raise RuntimeError(f"MoveIt service가 없습니다: {', '.join(missing)}")

    def read_current_transform(self, timeout_seconds: float = 2.0):
        """최신 base→flange TF를 내부 4×4 transform으로 반환한다."""
        deadline = time.monotonic() + timeout_seconds
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                message = self._tf_buffer.lookup_transform(
                    self._base_frame,
                    self._flange_frame,
                    Time(),
                )
            except TransformException:
                continue
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
        raise RuntimeError(
            f'{self._base_frame} → {self._flange_frame} TF를 읽지 못했습니다'
        )

    def _solve_ik(self, target: PoseStamped, seed: list[float]) -> list[float]:
        request = GetPositionIK.Request()
        request.ik_request.group_name = self._group_name
        request.ik_request.ik_link_name = self._flange_frame
        request.ik_request.pose_stamped = target
        request.ik_request.robot_state.joint_state.name = list(JOINT_NAMES)
        request.ik_request.robot_state.joint_state.position = seed
        request.ik_request.robot_state.is_diff = True
        request.ik_request.avoid_collisions = True
        request.ik_request.timeout = Duration(seconds=2.0).to_msg()
        response = self._wait_future(self._ik_client.call_async(request), 5.0)
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(f'MoveIt IK 실패: code={response.error_code.val}')
        solution = dict(
            zip(
                response.solution.joint_state.name,
                response.solution.joint_state.position,
            )
        )
        if not all(name in solution for name in JOINT_NAMES):
            raise RuntimeError('IK 결과에 필요한 6개 관절값이 없습니다')
        return [float(solution[name]) for name in JOINT_NAMES]

    def _check_state_validity(self, positions: list[float]) -> None:
        request = GetStateValidity.Request()
        request.group_name = self._group_name
        request.robot_state.joint_state.name = list(JOINT_NAMES)
        request.robot_state.joint_state.position = positions
        request.robot_state.is_diff = True
        response = self._wait_future(
            self._validity_client.call_async(request), 5.0
        )
        if not response.valid:
            raise RuntimeError(
                f'MoveIt 상태 유효성 검사 실패: contacts={len(response.contacts)}'
            )

    def _compute_fk(self, positions: list[float]):
        request = GetPositionFK.Request()
        request.header.frame_id = self._base_frame
        request.fk_link_names = [self._flange_frame]
        request.robot_state.joint_state.name = list(JOINT_NAMES)
        request.robot_state.joint_state.position = positions
        request.robot_state.is_diff = True
        response = self._wait_future(self._fk_client.call_async(request), 5.0)
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(f'MoveIt FK 실패: code={response.error_code.val}')
        if not response.pose_stamped:
            raise RuntimeError('MoveIt FK 결과 자세가 비어 있습니다')
        return pose_to_transform(response.pose_stamped[0].pose)

    def calculate_plan(
        self,
        axis: str,
        distance_m: float,
        maximum_distance_m: float,
        waypoint_spacing_m: float,
        maximum_joint_step_deg: float,
    ) -> IkStepPlan:
        """한 개의 X/Y 상대 목표를 계산하고 모든 사전검사를 수행한다."""
        joints, tf_message = self._wait_for_inputs(10.0)
        transform = tf_message.transform
        current = make_transform(
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
        target = make_locked_xy_target(
            current, axis, distance_m, maximum_distance_m
        )
        waypoints = make_fixed_z_xy_waypoints(
            current, target, waypoint_spacing_m
        )
        self._wait_for_services()

        seed = joints
        maximum_joint_change = 0.0
        final_fk = None
        for index, waypoint in enumerate(waypoints, start=1):
            stamped = PoseStamped()
            stamped.header.frame_id = self._base_frame
            stamped.header.stamp = self.get_clock().now().to_msg()
            stamped.pose = transform_to_pose(waypoint)
            solution = self._solve_ik(stamped, seed)
            joint_change = validate_joint_step(
                seed, solution, maximum_joint_step_deg
            )
            self._check_state_validity(solution)
            final_fk = self._compute_fk(solution)
            validate_fk_pose(
                waypoint,
                final_fk,
                maximum_position_error_m=0.001,
                maximum_z_error_m=0.0005,
                maximum_orientation_error_deg=0.5,
            )
            maximum_joint_change = max(maximum_joint_change, joint_change)
            seed = solution
            self.get_logger().info(
                f'waypoint {index}/{len(waypoints)} 통과: '
                f'max_joint_step={joint_change:.3f}deg'
            )

        assert final_fk is not None
        fk_error = validate_fk_pose(
            target,
            final_fk,
            maximum_position_error_m=0.001,
            maximum_z_error_m=0.0005,
            maximum_orientation_error_deg=0.5,
        )
        return IkStepPlan(
            current_transform=current,
            target_transform=target,
            start_joints=joints,
            target_joints=seed,
            waypoint_count=len(waypoints),
            maximum_joint_change_deg=maximum_joint_change,
            fk_position_error_m=fk_error.position_error_m,
            fk_z_error_m=fk_error.z_error_m,
            fk_orientation_error_deg=fk_error.orientation_error_deg,
        )

    def run(
        self,
        axis: str,
        distance_m: float,
        maximum_distance_m: float,
        waypoint_spacing_m: float,
        maximum_joint_step_deg: float,
    ) -> None:
        """한 개의 X/Y 상대 목표에 대한 MoveIt 사전검사를 실행한다."""
        self.get_logger().warning(
            'DRY RUN 전용: 이 노드는 로봇 제어 토픽이나 액션을 발행하지 않습니다'
        )
        plan = self.calculate_plan(
            axis,
            distance_m,
            maximum_distance_m,
            waypoint_spacing_m,
            maximum_joint_step_deg,
        )
        current = plan.current_transform
        target = plan.target_transform
        joints = plan.start_joints
        seed = plan.target_joints
        current_deg = [math.degrees(value) for value in joints]
        target_deg = [math.degrees(value) for value in seed]
        delta_deg = [
            target_value - current_value
            for current_value, target_value in zip(current_deg, target_deg)
        ]
        self.get_logger().info(
            'DRY RUN 통과\n'
            f'  요청: base {axis.upper()} {distance_m * 1000.0:+.3f}mm\n'
            f'  현재 XYZ [m]: {current[0, 3]:+.6f}, '
            f'{current[1, 3]:+.6f}, {current[2, 3]:+.6f}\n'
            f'  목표 XYZ [m]: {target[0, 3]:+.6f}, '
            f'{target[1, 3]:+.6f}, {target[2, 3]:+.6f}\n'
            f'  waypoint: {plan.waypoint_count}, '
            f'최대 관절 변화: {plan.maximum_joint_change_deg:.3f}deg\n'
            f'  현재 관절 [deg]: {[round(v, 3) for v in current_deg]}\n'
            f'  IK 관절 [deg]: {[round(v, 3) for v in target_deg]}\n'
            f'  관절 차이 [deg]: {[round(v, 3) for v in delta_deg]}\n'
            f'  FK 위치 오차: {plan.fk_position_error_m * 1000.0:.4f}mm, '
            f'Z 오차: {plan.fk_z_error_m * 1000.0:+.4f}mm, '
            f'자세 오차: {plan.fk_orientation_error_deg:.4f}deg\n'
            '  로봇 명령 발행: 없음'
        )


def _parse_arguments(arguments: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='고정-Z X/Y 이동의 MoveIt IK를 로봇 이동 없이 검사합니다'
    )
    parser.add_argument('--axis', choices=('x', 'y'), required=True)
    parser.add_argument('--distance-mm', type=float, required=True)
    parser.add_argument('--maximum-distance-mm', type=float, default=10.0)
    parser.add_argument('--waypoint-spacing-mm', type=float, default=1.0)
    parser.add_argument('--maximum-joint-step-deg', type=float, default=5.0)
    return parser.parse_args(arguments)


def main(args: list[str] | None = None) -> None:
    """CLI 인수를 읽고 한 번의 DRY RUN 후 종료한다."""
    raw_arguments = sys.argv if args is None else [sys.argv[0], *args]
    cli = _parse_arguments(rclpy.utilities.remove_ros_args(raw_arguments)[1:])
    rclpy.init(args=[])
    node = MoveItIkDryRunNode()
    exit_code = 0
    try:
        node.run(
            axis=cli.axis,
            distance_m=cli.distance_mm / 1000.0,
            maximum_distance_m=cli.maximum_distance_mm / 1000.0,
            waypoint_spacing_m=cli.waypoint_spacing_mm / 1000.0,
            maximum_joint_step_deg=cli.maximum_joint_step_deg,
        )
    except Exception as error:
        node.get_logger().error(f'MoveIt IK DRY RUN 실패: {error}')
        exit_code = 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if exit_code:
        raise SystemExit(exit_code)
