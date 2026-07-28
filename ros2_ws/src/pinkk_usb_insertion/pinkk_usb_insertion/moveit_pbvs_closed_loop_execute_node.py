"""YOLO 재관측 사이에 정지하며 X/Y/Yaw PBVS를 제한 횟수 반복한다."""

from __future__ import annotations

import argparse
import math
import sys
import time
from dataclasses import dataclass

from geometry_msgs.msg import Vector3Stamped
import numpy as np
import rclpy
from std_msgs.msg import Bool

from .control.pbvs_step_safety import validate_pbvs_reference_drift
from .moveit_pbvs_step_execute_node import MoveItPbvsStepExecuteNode


@dataclass(frozen=True)
class VisualMeasurement:
    """같은 관측 stamp의 PBVS 목표와 영상 오차."""

    sequence: int
    target: object
    error_x_m: float
    error_y_m: float
    yaw_error_rad: float
    converged: bool
    received_at: float

    @property
    def xy_norm_m(self) -> float:
        """Base XY 오차 norm을 반환한다."""
        return float(math.hypot(self.error_x_m, self.error_y_m))


def _stamp_key(message) -> tuple[int, int]:
    return int(message.header.stamp.sec), int(message.header.stamp.nanosec)


class MoveItPbvsClosedLoopExecuteNode(MoveItPbvsStepExecuteNode):
    """명시적 승인 후 정지-관측-단발 이동 PBVS를 반복한다."""

    def __init__(self) -> None:
        """PBVS 목표·오차·Yaw 모드 입력을 준비한다."""
        super().__init__()
        self._yaw_mode_received: bool | None = None
        self._latest_measurement: VisualMeasurement | None = None
        self._measurement_sequence = 0
        self.create_subscription(
            Vector3Stamped,
            '/robot_arm/pbvs/error',
            self._on_error,
            10,
        )
        self.create_subscription(
            Bool,
            '/robot_arm/pbvs/yaw_enabled',
            self._on_yaw_enabled,
            10,
        )

    def _on_yaw_enabled(self, message: Bool) -> None:
        self._yaw_mode_received = bool(message.data)

    def _on_error(self, message: Vector3Stamped) -> None:
        if self._latest_target is None or self._latest_converged is None:
            return
        if _stamp_key(self._latest_target) != _stamp_key(message):
            return
        self._measurement_sequence += 1
        self._latest_measurement = VisualMeasurement(
            sequence=self._measurement_sequence,
            target=self._latest_target,
            error_x_m=float(message.vector.x),
            error_y_m=float(message.vector.y),
            yaw_error_rad=float(message.vector.z),
            converged=bool(self._latest_converged),
            received_at=time.monotonic(),
        )

    def _wait_for_stable_measurement(
        self,
        after_sequence: int,
        received_after: float,
        timeout_seconds: float,
        stable_samples: int,
    ) -> VisualMeasurement:
        deadline = time.monotonic() + timeout_seconds
        consumed_sequence = after_sequence
        stable: list[VisualMeasurement] = []
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            measurement = self._latest_measurement
            if (
                measurement is None
                or measurement.sequence <= consumed_sequence
                or measurement.received_at < received_after
            ):
                continue
            consumed_sequence = measurement.sequence
            if stable:
                previous = stable[-1]
                xy_jump = math.hypot(
                    measurement.error_x_m - previous.error_x_m,
                    measurement.error_y_m - previous.error_y_m,
                )
                yaw_jump = abs(
                    measurement.yaw_error_rad - previous.yaw_error_rad
                )
                if xy_jump > 0.0015 or yaw_jump > math.radians(0.75):
                    stable.clear()
            stable.append(measurement)
            if len(stable) >= stable_samples:
                return stable[-1]
        raise RuntimeError(
            f'{timeout_seconds:.1f}초 안에 안정된 YOLO/PBVS 관측 '
            f'{stable_samples}개를 받지 못했습니다'
        )

    def execute_closed_loop(
        self,
        *,
        enable_yaw: bool,
        maximum_steps: int,
        move_seconds: float,
        settle_seconds: float,
        stable_samples: int,
        maximum_runtime_seconds: float,
        maximum_total_xy_m: float,
        maximum_cumulative_z_m: float,
    ) -> None:
        """수렴 또는 안전 종료 조건까지 제한된 PBVS step을 반복한다."""
        started_at = time.monotonic()
        loop_reference = self.read_current_transform(timeout_seconds=3.0)
        required_after = started_at
        sequence = 0
        total_planned_xy_m = 0.0
        total_actual_xy_m = 0.0
        previous: VisualMeasurement | None = None

        self.get_logger().warning(
            '실제 PBVS 폐루프 승인됨\n'
            f'  Yaw 실행: {enable_yaw}\n'
            f'  최대 step: {maximum_steps}\n'
            f'  최대 누적 계획 XY: {maximum_total_xy_m * 1000.0:.1f}mm\n'
            f'  시작 Z 대비 최대 누적 이탈: '
            f'{maximum_cumulative_z_m * 1000.0:.1f}mm\n'
            f'  step당 XY: 3.0mm 이하, Yaw: 2.0deg 이하\n'
            '  Z 하강과 삽입 명령은 발행하지 않습니다'
        )

        for step_index in range(maximum_steps + 1):
            if time.monotonic() - started_at > maximum_runtime_seconds:
                raise RuntimeError('PBVS 최대 실행 시간을 초과했습니다')
            actual_before = self.read_current_transform(timeout_seconds=3.0)
            validate_pbvs_reference_drift(
                loop_reference,
                actual_before,
                maximum_z_error_m=maximum_cumulative_z_m,
                maximum_orientation_error_deg=2.0,
            )
            measurement = self._wait_for_stable_measurement(
                after_sequence=sequence,
                received_after=required_after,
                timeout_seconds=8.0,
                stable_samples=stable_samples,
            )
            sequence = measurement.sequence
            if self._yaw_mode_received is None:
                raise RuntimeError('PBVS Yaw 활성 상태 토픽을 받지 못했습니다')
            if self._yaw_mode_received != enable_yaw:
                raise RuntimeError(
                    '실행기와 pbvs_alignment_node의 Yaw 설정이 다릅니다: '
                    f'executor={enable_yaw}, alignment={self._yaw_mode_received}'
                )

            self.get_logger().info(
                f'관측 {step_index}: '
                f'XY={measurement.xy_norm_m * 1000.0:.3f}mm, '
                f'Yaw={math.degrees(measurement.yaw_error_rad):+.3f}deg, '
                f'converged={measurement.converged}'
            )
            if previous is not None:
                if measurement.xy_norm_m > previous.xy_norm_m + 0.003:
                    raise RuntimeError(
                        '새 영상의 XY 오차가 직전보다 3mm 이상 증가했습니다'
                    )
                if (
                    enable_yaw
                    and abs(measurement.yaw_error_rad)
                    > abs(previous.yaw_error_rad) + math.radians(1.5)
                ):
                    raise RuntimeError(
                        '새 영상의 Yaw 오차가 직전보다 1.5도 이상 증가했습니다'
                    )
            if measurement.converged:
                self.get_logger().warning(
                    'PBVS 폐루프 수렴\n'
                    f'  최종 XY 오차: {measurement.xy_norm_m * 1000.0:.3f}mm\n'
                    f'  최종 Yaw 오차: '
                    f'{math.degrees(measurement.yaw_error_rad):+.3f}deg\n'
                    '  로봇은 정지 상태이며 Z 삽입은 실행하지 않았습니다'
                )
                return
            if step_index >= maximum_steps:
                break
            if total_planned_xy_m + 0.003 > maximum_total_xy_m + 1e-12:
                raise RuntimeError('다음 step이 최대 누적 XY 제한을 초과합니다')

            plan = self.execute_pbvs_target(
                measurement.target,
                move_seconds=move_seconds,
                apply_target_yaw=enable_yaw,
                warning_delay_seconds=3.0 if step_index == 0 else 0.5,
                locked_reference_transform=loop_reference,
                maximum_reference_z_correction_m=maximum_cumulative_z_m,
                maximum_reference_orientation_correction_deg=2.0,
            )
            planned_xy = float(
                np.linalg.norm(
                    plan.target_transform[:2, 3]
                    - plan.current_transform[:2, 3]
                )
            )
            total_planned_xy_m += planned_xy
            actual_after = self.read_current_transform(timeout_seconds=3.0)
            reference_drift = validate_pbvs_reference_drift(
                loop_reference,
                actual_after,
                maximum_z_error_m=maximum_cumulative_z_m,
                maximum_orientation_error_deg=2.0,
            )
            self.get_logger().info(
                '폐루프 기준 유지: '
                f'z_drift={reference_drift.z_error_m * 1000.0:+.3f}mm, '
                'orientation_drift='
                f'{reference_drift.orientation_error_deg:.3f}deg'
            )
            actual_xy = float(
                np.linalg.norm(
                    actual_after[:2, 3] - plan.current_transform[:2, 3]
                )
            )
            total_actual_xy_m += actual_xy
            if total_actual_xy_m > maximum_total_xy_m:
                raise RuntimeError(
                    '실제 누적 XY 이동이 제한을 초과했습니다: '
                    f'{total_actual_xy_m * 1000.0:.3f}mm'
                )
            previous = measurement
            required_after = time.monotonic() + settle_seconds

        raise RuntimeError(
            f'최대 {maximum_steps} step 안에 PBVS가 수렴하지 않았습니다'
        )


def _parse_arguments(arguments: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='YOLO 기반 X/Y/Yaw PBVS를 실제 로봇에서 제한 반복합니다'
    )
    parser.add_argument('--max-steps', type=int, default=12)
    parser.add_argument('--move-seconds', type=float, default=8.0)
    parser.add_argument('--settle-seconds', type=float, default=1.0)
    parser.add_argument('--stable-samples', type=int, default=5)
    parser.add_argument('--max-runtime-seconds', type=float, default=300.0)
    parser.add_argument('--max-total-xy-mm', type=float, default=40.0)
    parser.add_argument('--max-cumulative-z-mm', type=float, default=5.0)
    parser.add_argument(
        '--enable-yaw',
        action='store_true',
        help='flange/port 장축 확인 후에만 Yaw 실제 실행을 허용합니다',
    )
    parser.add_argument(
        '--execute',
        action='store_true',
        help='이 옵션이 있어야 실제 폐루프 관절 목표를 전송합니다',
    )
    parsed = parser.parse_args(arguments)
    if not parsed.execute:
        parser.error('실제 폐루프 실행에는 --execute를 명시해야 합니다')
    if parsed.max_steps < 1 or parsed.max_steps > 20:
        parser.error('--max-steps는 1~20이어야 합니다')
    if parsed.move_seconds < 5.0 or parsed.move_seconds > 15.0:
        parser.error('--move-seconds는 5~15초여야 합니다')
    if parsed.settle_seconds < 0.5 or parsed.settle_seconds > 5.0:
        parser.error('--settle-seconds는 0.5~5초여야 합니다')
    if parsed.stable_samples < 3 or parsed.stable_samples > 20:
        parser.error('--stable-samples는 3~20이어야 합니다')
    if parsed.max_runtime_seconds < 30.0 or parsed.max_runtime_seconds > 600.0:
        parser.error('--max-runtime-seconds는 30~600초여야 합니다')
    if parsed.max_total_xy_mm < 3.0 or parsed.max_total_xy_mm > 60.0:
        parser.error('--max-total-xy-mm는 3~60mm여야 합니다')
    if (
        parsed.max_cumulative_z_mm < 2.0
        or parsed.max_cumulative_z_mm > 10.0
    ):
        parser.error('--max-cumulative-z-mm는 2~10mm여야 합니다')
    return parsed


def main(args: list[str] | None = None) -> None:
    """CLI 승인을 확인하고 실제 PBVS 폐루프를 실행한다."""
    raw_arguments = sys.argv if args is None else [sys.argv[0], *args]
    cli = _parse_arguments(rclpy.utilities.remove_ros_args(raw_arguments)[1:])
    rclpy.init(args=[])
    node = MoveItPbvsClosedLoopExecuteNode()
    exit_code = 0
    try:
        node.execute_closed_loop(
            enable_yaw=cli.enable_yaw,
            maximum_steps=cli.max_steps,
            move_seconds=cli.move_seconds,
            settle_seconds=cli.settle_seconds,
            stable_samples=cli.stable_samples,
            maximum_runtime_seconds=cli.max_runtime_seconds,
            maximum_total_xy_m=cli.max_total_xy_mm / 1000.0,
            maximum_cumulative_z_m=cli.max_cumulative_z_mm / 1000.0,
        )
    except Exception as error:
        node.get_logger().error(f'MoveIt PBVS 폐루프 종료: {error}')
        exit_code = 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if exit_code:
        raise SystemExit(exit_code)
