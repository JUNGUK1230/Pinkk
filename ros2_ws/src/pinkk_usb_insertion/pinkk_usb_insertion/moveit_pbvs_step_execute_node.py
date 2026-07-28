"""최신 YOLO/PnP PBVS 목표를 MoveIt 관절 명령으로 한 번 실행한다."""

from __future__ import annotations

import argparse
import math
import sys
import time

from geometry_msgs.msg import PoseStamped

import rclpy

from std_msgs.msg import Bool

from .moveit_ik_step_execute_node import MoveItIkStepExecuteNode
from .ros_utils import pose_to_transform


PBVS_MAXIMUM_POST_Z_ERROR_M = 0.002
PBVS_MAXIMUM_POST_ORIENTATION_ERROR_DEG = 2.0


class MoveItPbvsStepExecuteNode(MoveItIkStepExecuteNode):
    """신선한 PBVS 목표 하나만 받아 최대 3mm의 단발 이동을 실행한다."""

    def __init__(self) -> None:
        """PBVS 목표·수렴 상태 입력을 추가한다."""
        super().__init__('pinkk_moveit_pbvs_step_execute')
        self._latest_target: PoseStamped | None = None
        self._latest_converged: bool | None = None
        self._target_received_at: float | None = None
        self.create_subscription(
            PoseStamped,
            '/robot_arm/pbvs/target_flange_pose',
            self._on_target,
            10,
        )
        self.create_subscription(
            Bool,
            '/robot_arm/pbvs/converged',
            self._on_converged,
            10,
        )

    def _on_target(self, message: PoseStamped) -> None:
        self._latest_target = message
        self._target_received_at = time.monotonic()

    def _on_converged(self, message: Bool) -> None:
        self._latest_converged = bool(message.data)

    def _wait_for_fresh_target(
        self,
        timeout_seconds: float,
        maximum_age_seconds: float,
    ) -> PoseStamped:
        deadline = time.monotonic() + timeout_seconds
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if (
                self._latest_target is not None
                and self._latest_converged is not None
                and self._target_received_at is not None
                and time.monotonic() - self._target_received_at
                <= maximum_age_seconds
            ):
                break
        if self._latest_target is None or self._latest_converged is None:
            raise RuntimeError('PBVS 목표와 converged 결과를 받지 못했습니다')
        if self._target_received_at is None:
            raise RuntimeError('PBVS 목표 수신 시간이 없습니다')
        age = time.monotonic() - self._target_received_at
        if age > maximum_age_seconds:
            raise RuntimeError(f'PBVS 목표가 오래됐습니다: age={age:.3f}s')
        if self._latest_converged:
            raise RuntimeError('이미 PBVS XY 허용오차 안이므로 이동하지 않습니다')
        if self._latest_target.header.frame_id != self._base_frame:
            raise RuntimeError(
                f'PBVS 목표 frame이 {self._base_frame}가 아닙니다'
            )
        return self._latest_target

    def execute_latest_pbvs(self, move_seconds: float) -> None:
        """최신 PBVS 목표를 재검증하고 최대 3mm 한 번만 이동한다."""
        target_message = self._wait_for_fresh_target(
            timeout_seconds=10.0,
            maximum_age_seconds=1.0,
        )
        self.execute_pbvs_target(
            target_message,
            move_seconds=move_seconds,
            apply_target_yaw=False,
            warning_delay_seconds=3.0,
        )

    def execute_pbvs_target(
        self,
        target_message: PoseStamped,
        move_seconds: float,
        apply_target_yaw: bool,
        warning_delay_seconds: float,
        locked_reference_transform=None,
        maximum_reference_z_correction_m: float = 0.0005,
        maximum_reference_orientation_correction_deg: float = 0.5,
    ):
        """주어진 PBVS 목표를 현재 Z 기준으로 검증하고 한 번 실행한다."""
        target = pose_to_transform(target_message.pose)
        current = self.read_current_transform(timeout_seconds=2.0)
        # 거친 정렬에서는 PBVS가 계산한 base XY만 사용한다. 기기 오차로
        # 초기 Z·자세와 차이가 생겨도 매 단발 시작 자세를 유지하고,
        # 초기 기준 Z·Roll/Pitch 복귀는 최종 삽입 직전에 별도로 수행한다.
        reference = (
            current
            if locked_reference_transform is None
            else locked_reference_transform
        )
        target[2, 3] = reference[2, 3]
        if not apply_target_yaw:
            target[:3, :3] = reference[:3, :3]
        plan = self.calculate_target_plan(
            target=target,
            maximum_distance_m=0.003,
            waypoint_spacing_m=0.001,
            maximum_joint_step_deg=5.0,
            maximum_z_change_m=maximum_reference_z_correction_m,
            maximum_orientation_change_deg=(
                2.1
                if apply_target_yaw
                else maximum_reference_orientation_correction_deg
            ),
        )
        dx_mm = (
            plan.target_transform[0, 3]
            - plan.current_transform[0, 3]
        ) * 1000.0
        dy_mm = (
            plan.target_transform[1, 3]
            - plan.current_transform[1, 3]
        ) * 1000.0
        relative_rotation = (
            plan.current_transform[:3, :3].T
            @ plan.target_transform[:3, :3]
        )
        orientation_step_deg = float(
            math.degrees(
                math.acos(
                    max(
                        -1.0,
                        min(1.0, (float(relative_rotation.trace()) - 1.0) * 0.5),
                    )
                )
            )
        )
        self.execute_plan(
            plan,
            label=(
                f'PBVS dx={dx_mm:+.3f}mm, dy={dy_mm:+.3f}mm, '
                f'orientation_step={orientation_step_deg:.3f}deg'
            ),
            move_seconds=move_seconds,
            maximum_total_joint_change_deg=5.0,
            maximum_target_error_m=0.015,
            minimum_progress_m=0.0005,
            maximum_post_z_error_m=PBVS_MAXIMUM_POST_Z_ERROR_M,
            maximum_post_orientation_error_deg=(
                PBVS_MAXIMUM_POST_ORIENTATION_ERROR_DEG
            ),
            # 관절 공간 보간 중의 순간 경로 이탈은 별도 비상 기준으로
            # 감시한다. 정지 후에는 위의 더 엄격한 고정-Z 기준을 적용해
            # 다음 PBVS step으로 넘어가지 않는다.
            motion_guard_z_change_m=0.010,
            motion_guard_orientation_change_deg=5.0,
            motion_guard_xy_overshoot_m=0.010,
            motion_guard_opposite_progress_m=0.003,
            warning_delay_seconds=warning_delay_seconds,
        )
        return plan


def _parse_arguments(arguments: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='최신 PBVS 목표를 실제 로봇에 최대 3mm 한 번 전송합니다'
    )
    parser.add_argument('--move-seconds', type=float, default=8.0)
    parser.add_argument(
        '--execute',
        action='store_true',
        help='이 옵션이 있어야 실제 관절 목표를 전송합니다',
    )
    parsed = parser.parse_args(arguments)
    if not parsed.execute:
        parser.error('실제 PBVS 실행에는 --execute를 명시해야 합니다')
    if parsed.move_seconds < 5.0 or parsed.move_seconds > 15.0:
        parser.error('--move-seconds는 5~15초여야 합니다')
    return parsed


def main(args: list[str] | None = None) -> None:
    """최신 PBVS 목표를 한 번 실행하고 종료한다."""
    raw_arguments = sys.argv if args is None else [sys.argv[0], *args]
    cli = _parse_arguments(rclpy.utilities.remove_ros_args(raw_arguments)[1:])
    rclpy.init(args=[])
    node = MoveItPbvsStepExecuteNode()
    exit_code = 0
    try:
        node.execute_latest_pbvs(move_seconds=cli.move_seconds)
    except Exception as error:
        node.get_logger().error(f'MoveIt PBVS 실제 단발 실행 실패: {error}')
        exit_code = 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if exit_code:
        raise SystemExit(exit_code)
