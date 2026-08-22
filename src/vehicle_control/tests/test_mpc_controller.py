"""차동구동 MPC가 실제 START→C2 전후진 경로를 추종하는지 검사한다."""

import csv
from dataclasses import fields, replace
import math
from pathlib import Path
import statistics
import sys

import numpy as np
import yaml


SCRIPT_DIR = Path(__file__).resolve().parent
SRC_ROOT = SCRIPT_DIR.parents[1]
REPO_ROOT = SRC_ROOT.parent
sys.path.insert(0, str(SRC_ROOT))

from vehicle_control.mpc_controller import (  # noqa: E402
    DifferentialDriveMpc,
    MpcLimits,
    MpcWeights,
    ReferencePoint,
    VehicleState,
    normalize_angle,
)
from vehicle_control.mpc_path_follower import (  # noqa: E402
    scan_sector_minima,
    trajectory_signature,
)


def load_c2_path() -> list[ReferencePoint]:
    path = (
        SRC_ROOT
        / "central_control/path_planning/output/fixed_route_start_to_c2.csv"
    )
    with path.open(encoding="utf-8") as file:
        rows = csv.DictReader(file)
        return [
            ReferencePoint(
                x_m=float(row["x_cm"]) / 100.0,
                y_m=float(row["y_cm"]) / 100.0,
                yaw_rad=float(row["yaw_rad"]),
                direction=int(row["direction"]),
            )
            for row in rows
        ]


def hysteresis_crossings(values: list[float], threshold: float) -> int:
    previous_sign = 0
    crossings = 0
    for value in values:
        sign = 1 if value > threshold else -1 if value < -threshold else 0
        if sign == 0:
            continue
        if previous_sign != 0 and sign != previous_sign:
            crossings += 1
        previous_sign = sign
    return crossings


def production_controller() -> DifferentialDriveMpc:
    config_path = SRC_ROOT / "vehicle_control/config/mpc/mpc.yaml"
    with config_path.open(encoding="utf-8") as file:
        document = yaml.safe_load(file)
        parameters = (
            document.get("pinkk_mpc_path_follower") or document["/**"]
        )["ros__parameters"]
    limit_names = {field.name for field in fields(MpcLimits)}
    limit_values = {
        key: value
        for key, value in parameters.items()
        if key in limit_names
    }
    limit_values.update(
        max_tracking_yaw_error_rad=math.radians(
            parameters["max_tracking_yaw_error_deg"]
        ),
        heading_recovery_full_curvature_error_rad=math.radians(
            parameters["heading_recovery_full_curvature_error_deg"]
        ),
        heading_recovery_speed_scale=parameters[
            "heading_recovery_speed_scale"
        ],
        heading_feedback_deadband_rad=math.radians(
            parameters["heading_feedback_deadband_deg"]
        ),
        reverse_heading_feedback_deadband_rad=math.radians(
            parameters["reverse_heading_feedback_deadband_deg"]
        ),
        forward_rejoin_max_heading_rad=math.radians(
            parameters["forward_rejoin_max_heading_deg"]
        ),
        goal_yaw_tolerance_rad=math.radians(
            parameters["goal_yaw_tolerance_deg"]
        ),
    )
    weights = MpcWeights(
        **{
            name: parameters[f"weight_{name}"]
            for name in (
                "position",
                "yaw",
                "terminal_position",
                "terminal_yaw",
                "speed",
                "curvature",
                "speed_rate",
                "curvature_rate",
            )
        }
    )
    return DifferentialDriveMpc(MpcLimits(**limit_values), weights)


def simulate(
    controller: DifferentialDriveMpc,
    state: VehicleState,
    maximum_steps: int,
) -> tuple[VehicleState, int, int, list[float], float, float]:
    gear_changes = 0
    solve_times: list[float] = []
    maximum_angular_speed = 0.0
    maximum_cross_track_error = 0.0
    for step in range(maximum_steps):
        previous_curvature = controller.last_curvature_1pm
        command = controller.command(state)
        solve_times.append(command.solve_time_sec)
        maximum_angular_speed = max(
            maximum_angular_speed,
            abs(command.angular_radps),
        )
        reference = controller.path[command.progress_index]
        if reference.direction > 0:
            cross_track_error = (
                controller._forward_path_projection(state).center_cross_track_m
            )
        else:
            reference_yaw = controller._segment_tangent_yaw(
                command.progress_index
            )
            cross_track_error = controller._current_cross_track_error(
                state,
                reference,
                reference_yaw,
                reference.direction,
            )
        maximum_cross_track_error = max(
            maximum_cross_track_error,
            abs(cross_track_error),
        )
        assert (
            abs(command.angular_radps) <= controller.limits.max_angular_speed_radps + 1e-6
        )
        if command.status == "TRACKING":
            assert abs(command.curvature_1pm - previous_curvature) <= (
                controller.limits.max_curvature_rate_1pmps
                * controller.limits.dt_sec
                + 1e-6
            ), (
                step,
                previous_curvature,
                command.curvature_1pm,
                command.linear_mps,
                command.status,
            )
        assert (
            command.linear_mps**2 * abs(command.curvature_1pm)
            <= controller.limits.max_lateral_acceleration_mps2 + 1e-6
        )
        half_track = controller.limits.wheel_separation_m * 0.5
        left_wheel_radps = (
            command.linear_mps - command.angular_radps * half_track
        ) / controller.limits.wheel_radius_m
        right_wheel_radps = (
            command.linear_mps + command.angular_radps * half_track
        ) / controller.limits.wheel_radius_m
        assert (
            abs(left_wheel_radps)
            <= controller.limits.max_wheel_angular_speed_radps + 1e-6
        )
        assert (
            abs(right_wheel_radps)
            <= controller.limits.max_wheel_angular_speed_radps + 1e-6
        )
        if abs(command.linear_mps) <= 1e-10:
            assert abs(command.angular_radps) <= 1e-10
        if command.status == "GEAR_CHANGE_REQUIRED":
            assert controller.advance_gear_segment()
            gear_changes += 1
            continue
        if command.status == "GOAL_REACHED":
            return (
                state,
                step,
                gear_changes,
                solve_times,
                maximum_angular_speed,
                maximum_cross_track_error,
            )
        assert command.status == "TRACKING", command.status
        state = controller.propagate_state(
            state,
            command.linear_mps,
            command.angular_radps,
        )
    raise AssertionError("MPC did not reach the goal within the simulation limit")


def main() -> int:
    path = load_c2_path()

    # 전진 곡률 한계는 직선에서 급곡선까지 계단 없이 연속 증가한다.
    # 후진은 검증된 기존 전체 한계를 유지한다.
    curve_limit_controller = production_controller()
    middle_path_curvature = 0.5 * (
        curve_limit_controller.limits.straight_curvature_threshold_1pm
        + curve_limit_controller.limits.full_curvature_path_threshold_1pm
    )
    middle_references = [
        (
            ReferencePoint(index * 0.005, 0.0, 0.0, 1),
            middle_path_curvature,
        )
        for index in range(8)
    ]
    curve_limit_controller._reference_curvature = np.full(
        80, middle_path_curvature
    )
    expected_middle_limit = 0.5 * (
        curve_limit_controller.limits.straight_max_curvature_1pm
        + curve_limit_controller.limits.max_curvature_1pm
    )
    assert math.isclose(
        curve_limit_controller._curvature_limit(
            middle_references, 20, 79, 1
        ),
        expected_middle_limit,
    )
    sharp_references = [
        (ReferencePoint(index * 0.005, 0.0, 0.0, 1), 6.0)
        for index in range(8)
    ]
    curve_limit_controller._reference_curvature[:] = 6.0
    assert math.isclose(
        curve_limit_controller._curvature_limit(sharp_references, 20, 79, 1),
        curve_limit_controller.limits.max_curvature_1pm,
    )
    assert math.isclose(
        curve_limit_controller._curvature_limit(
            middle_references, 20, 79, -1
        ),
        curve_limit_controller.limits.max_curvature_1pm,
    )

    # 같은 급곡선도 빠를 때는 등가 횡가속도로 제한되고 감속하면 실차의
    # 0.12m 최소회전반경을 다시 모두 사용할 수 있어야 한다.
    fast_curvature_limit = curve_limit_controller._speed_curvature_limit(
        curve_limit_controller.limits.max_curvature_1pm,
        curve_limit_controller.limits.max_forward_speed_mps,
    )
    slow_curvature_limit = curve_limit_controller._speed_curvature_limit(
        curve_limit_controller.limits.max_curvature_1pm,
        0.03,
    )
    assert fast_curvature_limit < slow_curvature_limit
    assert math.isclose(
        slow_curvature_limit,
        curve_limit_controller.limits.max_curvature_1pm,
    )
    sharp_curve_speed_limit = curve_limit_controller._speed_limit_for_curvature(
        curve_limit_controller.limits.max_forward_speed_mps,
        curve_limit_controller.limits.max_curvature_1pm,
    )
    assert sharp_curve_speed_limit < (
        curve_limit_controller.limits.max_forward_speed_mps
    )
    assert (
        sharp_curve_speed_limit**2
        * curve_limit_controller.limits.max_curvature_1pm
        <= curve_limit_controller.limits.max_lateral_acceleration_mps2 + 1e-9
    )
    assert math.isclose(
        curve_limit_controller._speed_curvature_limit(
            curve_limit_controller.limits.max_curvature_1pm,
            sharp_curve_speed_limit,
        ),
        curve_limit_controller.limits.max_curvature_1pm,
        rel_tol=1e-9,
    )
    straight_fast_rate = curve_limit_controller._adaptive_curvature_rate(
        0.0,
        curve_limit_controller.limits.max_forward_speed_mps,
        1,
    )
    curve_fast_rate = curve_limit_controller._adaptive_curvature_rate(
        1.0,
        curve_limit_controller.limits.max_forward_speed_mps,
        1,
    )
    curve_slow_rate = curve_limit_controller._adaptive_curvature_rate(
        1.0,
        0.02,
        1,
    )
    assert straight_fast_rate < curve_fast_rate < curve_slow_rate

    # MPC를 경로 중간에서 시작해도 첫 nearest 검색은 제한된 forward window가
    # 아니라 첫 direction segment 전체에서 차량 위치에 정렬되어야 한다.
    restart_index = 184
    assert path[restart_index].direction == path[0].direction
    restart_guard = DifferentialDriveMpc()
    restart_guard.set_path(path)
    restart_state = VehicleState(
        path[restart_index].x_m,
        path[restart_index].y_m,
        path[restart_index].yaw_rad,
    )
    assert restart_guard._nearest_index(restart_state) < restart_index
    restart_command = restart_guard.command(restart_state)
    assert restart_command.status == "TRACKING"
    assert restart_command.progress_index == restart_index

    # 큰 heading 오차 복귀는 최대 곡률을 사용하더라도 좁은 벽 쪽으로
    # 기준속도로 계속 진행하지 않고 정밀 추종용 저속으로 제한되어야 한다.
    recovery_controller = production_controller()
    recovery_controller.set_path(path)
    recovery_state = VehicleState(
        path[0].x_m,
        path[0].y_m,
        normalize_angle(path[0].yaw_rad - math.radians(35.0)),
    )
    recovery_command = recovery_controller.command(recovery_state)
    assert recovery_command.status == "TRACKING"
    expected_recovery_speed = min(
        recovery_controller.limits.forward_speed_mps
        * recovery_controller.limits.heading_recovery_speed_scale,
        recovery_controller.limits.forward_max_acceleration_mps2
        * recovery_controller.limits.dt_sec,
    )
    assert math.isclose(
        recovery_command.linear_mps,
        expected_recovery_speed,
        rel_tol=0.0,
        abs_tol=1e-9,
    )
    recovery_ratio = recovery_controller._heading_recovery_ratio(
        math.radians(35.0)
    )
    recovery_rate = recovery_controller._adaptive_curvature_rate(
        0.0,
        recovery_command.linear_mps,
        1,
        recovery_ratio,
    )
    assert 0.0 < abs(recovery_command.curvature_1pm) <= (
        recovery_rate * recovery_controller.limits.dt_sec + 1e-9
    )
    recovery_start = recovery_controller.limits.max_tracking_yaw_error_rad
    recovery_full = (
        recovery_controller.limits.heading_recovery_full_curvature_error_rad
    )
    assert recovery_controller._heading_recovery_ratio(
        recovery_start - math.radians(0.1)
    ) == 0.0
    assert 0.0 < recovery_controller._heading_recovery_ratio(
        recovery_start + math.radians(0.1)
    ) < 0.01
    assert recovery_controller._heading_recovery_ratio(recovery_full) == 1.0

    # 카메라 검출의 mm 단위 횡방향 흔들림은 직선 조향 명령을 좌우로
    # 뒤집지 않아야 한다.
    deadband_controller = production_controller()
    straight_path = [
        ReferencePoint(index * 0.005, 0.0, 0.0, 1)
        for index in range(40)
    ]
    deadband_controller.set_path(straight_path)
    deadband_command = deadband_controller.command(
        VehicleState(0.0, 0.002, 0.0)
    )
    assert deadband_command.status == "TRACKING"
    assert abs(deadband_command.angular_radps) <= 0.01

    # 전진 optimizer 자체도 deadband를 적용해야 5mm 위치 흔들림과 1.5도
    # heading 흔들림을 별도 조향 명령으로 만들지 않는다.
    micro_error_controller = production_controller()
    micro_error_controller.set_path(straight_path)
    micro_error_command = micro_error_controller.command(
        VehicleState(0.0, 0.005, math.radians(1.5))
    )
    assert micro_error_command.status == "TRACKING"
    assert abs(micro_error_command.angular_radps) <= 0.002

    # 전진 pose와 path는 차량 중심 좌표이므로 yaw가 서로 달라도 중심이
    # 선 위에 있으면 4cm control-point offset을 횡오차로 만들면 안 된다.
    centered_heading_error = VehicleState(
        0.0,
        0.0,
        math.radians(12.0),
    )
    assert abs(
        deadband_controller._current_cross_track_error(
            centered_heading_error,
            straight_path[0],
            0.0,
            1,
        )
    ) <= 1e-12

    # MPC 내부는 rear axle에서 운동을 적분하더라도 전진 위치 비용은 다시
    # 차량 중심으로 환산해야 한다. 정지 예측의 위치 비용은 offset 설정과
    # 무관해야 해 heading 오차가 가짜 횡오차로 중복 반영되지 않는다.
    centered_weights = MpcWeights(
        position=1.0,
        yaw=0.0,
        terminal_position=1.0,
        terminal_yaw=0.0,
        speed=0.0,
        curvature=0.0,
        speed_rate=0.0,
        curvature_rate=0.0,
    )
    zero_offset_controller = DifferentialDriveMpc(
        replace(deadband_controller.limits, control_point_offset_m=0.0),
        centered_weights,
    )
    offset_controller = DifferentialDriveMpc(
        deadband_controller.limits,
        centered_weights,
    )
    zero_offset_controller.set_path(straight_path)
    offset_controller.set_path(straight_path)
    zero_controls = np.zeros(
        2 * deadband_controller.limits.horizon_steps,
        dtype=np.float64,
    )
    zero_references = zero_offset_controller._reference_horizon(
        0,
        len(straight_path) - 1,
    )
    offset_references = offset_controller._reference_horizon(
        0,
        len(straight_path) - 1,
    )
    assert math.isclose(
        zero_offset_controller._objective(
            zero_controls,
            centered_heading_error,
            zero_references,
            1,
        ),
        offset_controller._objective(
            zero_controls,
            centered_heading_error,
            offset_references,
            1,
        ),
        rel_tol=0.0,
        abs_tol=1e-12,
    )

    # 1cm 전진 이탈은 매 주기 목표 heading을 다시 만들지 않고, 최초 자세에서
    # 원 경로 허용 띠까지 이어지는 하나의 합류 곡선을 latch해야 한다.
    assert deadband_controller.limits.forward_cross_track_gain_scale > 1.0
    latched_path = [
        ReferencePoint(index * 0.005, 0.0, 0.0, 1)
        for index in range(100)
    ]
    latched_controller = production_controller()
    latched_controller.set_path(latched_path)
    latched_state = VehicleState(0.0, 0.01, 0.0)
    latched_command = latched_controller.command(latched_state)
    assert latched_command.status == "TRACKING"
    assert latched_controller._forward_rejoin_active
    latched_start_s = latched_controller._forward_rejoin_start_s_m
    latched_length = latched_controller._forward_rejoin_length_m
    latched_projection = latched_controller._forward_path_projection(latched_state)
    latched_reference, _ = latched_controller._forward_rejoin_reference(
        latched_projection.rear_s_m,
        0,
        len(latched_path) - 1,
    )
    assert math.hypot(
        latched_reference.x_m - latched_state.x_m,
        latched_reference.y_m - latched_state.y_m,
    ) <= 1e-9
    for _ in range(5):
        latched_state = latched_controller.propagate_state(
            latched_state,
            latched_command.linear_mps,
            latched_command.angular_radps,
        )
        latched_command = latched_controller.command(latched_state)
        assert latched_controller._forward_rejoin_active
        assert math.isclose(
            latched_controller._forward_rejoin_start_s_m,
            latched_start_s,
            abs_tol=1e-12,
        )
        assert math.isclose(
            latched_controller._forward_rejoin_length_m,
            latched_length,
            abs_tol=1e-12,
        )

    # 차체가 이미 경로 쪽을 향하면 합류 곡선의 시작 yaw가 현재 차체 yaw와
    # 이어져야 하며, 현재점으로 순간적으로 꺾는 reference를 만들면 안 된다.
    converging_state = VehicleState(0.0, 0.02, math.radians(-6.0))
    damped_controller = production_controller()
    damped_controller.set_path(latched_path)
    damped_command = damped_controller.command(converging_state)
    damped_projection = damped_controller._forward_path_projection(
        converging_state
    )
    damped_reference, _ = damped_controller._forward_rejoin_reference(
        damped_projection.rear_s_m,
        0,
        len(latched_path) - 1,
    )
    assert abs(
        normalize_angle(damped_reference.yaw_rad - converging_state.yaw_rad)
    ) <= 1e-9
    assert abs(damped_command.curvature_1pm) < 1.0

    # 직선 경로에서 2cm 떨어져 시작해도 가장 가까운 점을 지나쳐 S자로
    # 복귀하지 않고, 앞쪽 접선에 자연스럽게 합류해야 한다.
    rejoin_controller = production_controller()
    rejoin_path = [
        ReferencePoint(index * 0.005, 0.0, 0.0, 1)
        for index in range(200)
    ]
    rejoin_controller.set_path(rejoin_path)
    rejoin_state = VehicleState(0.0, 0.02, 0.0)
    rejoin_offsets: list[float] = []
    rejoin_angular_commands: list[float] = []
    rejoin_target_offset = 0.0
    for _ in range(120):
        rejoin_command = rejoin_controller.command(rejoin_state)
        assert rejoin_command.status == "TRACKING"
        if rejoin_controller._forward_rejoin_active:
            rejoin_target_offset = (
                rejoin_controller._forward_rejoin_target_offset_m
            )
        rejoin_offsets.append(rejoin_state.y_m)
        rejoin_angular_commands.append(rejoin_command.angular_radps)
        rejoin_state = rejoin_controller.propagate_state(
            rejoin_state,
            rejoin_command.linear_mps,
            rejoin_command.angular_radps,
        )
    assert abs(rejoin_offsets[-1]) <= rejoin_controller.limits.cross_track_deadband_m
    # 합류는 최초 쪽에서 정상 deadband 안으로 들어와야 하며, 중심선을
    # 1mm 이상 가로질러 반대 조향하는 오버슈팅이 없어야 한다.
    assert min(rejoin_offsets) >= -0.001
    assert hysteresis_crossings(rejoin_offsets, 0.001) == 0
    assert (
        sum(
            first * second < 0.0
            for first, second in zip(
                rejoin_angular_commands,
                rejoin_angular_commands[1:],
            )
        )
        <= 1
    )

    # 전진 preview는 경로 합류를 부드럽게 하기 위해 후진보다 조금 길지만,
    # 후진 전용 기존 preview 값은 바뀌지 않아야 한다.
    assert (
        rejoin_controller.limits.forward_steering_preview_points
        > rejoin_controller.limits.steering_preview_points
    )
    assert (
        rejoin_controller.limits.forward_steering_preview_weight
        > rejoin_controller.limits.steering_preview_weight
    )
    assert rejoin_controller.limits.steering_rejoin_preview_points >= 20
    assert rejoin_controller.limits.steering_rejoin_preview_weight >= 0.50
    assert math.isclose(
        rejoin_controller.limits.forward_rejoin_lookahead_m,
        0.16,
    )
    assert math.isclose(
        rejoin_controller.limits.forward_curve_rejoin_lookahead_m,
        0.15,
    )
    assert (
        0.0
        < abs(rejoin_target_offset)
        <= 0.5 * rejoin_controller.limits.cross_track_deadband_m
    )

    # 실제 고정 경로처럼 rear axle 원호를 body yaw 방향으로 4cm 옮긴 차량
    # 중심 경로를 만든다. 중심 원호 자체에 같은 yaw를 넣으면 물리적으로
    # 존재하지 않는 경로가 되어 controller를 잘못 평가한다.
    curve_radius_m = 0.25
    curve_fixture_controller = production_controller()
    control_offset_m = curve_fixture_controller.limits.control_point_offset_m
    curve_path = [
        ReferencePoint(
            curve_radius_m * math.sin(theta)
            + control_offset_m * (math.cos(theta) - 1.0),
            curve_radius_m * (1.0 - math.cos(theta))
            + control_offset_m * math.sin(theta),
            theta,
            1,
        )
        for theta in (
            math.pi * 0.5 * index / 99.0
            for index in range(100)
        )
    ]
    curve_rejoin_controller = production_controller()
    curve_rejoin_controller.set_path(curve_path)
    curve_state = VehicleState(
        curve_path[0].x_m,
        curve_path[0].y_m,
        curve_path[0].yaw_rad,
    )
    curve_errors: list[float] = []
    curve_commands: list[float] = []
    for _ in range(100):
        curve_command = curve_rejoin_controller.command(curve_state)
        assert curve_command.status == "TRACKING"
        curve_errors.append(
            curve_rejoin_controller._forward_path_projection(
                curve_state
            ).center_cross_track_m
        )
        curve_commands.append(curve_command.angular_radps)
        if curve_command.progress_index >= 90:
            break
        curve_state = curve_rejoin_controller.propagate_state(
            curve_state,
            curve_command.linear_mps,
            curve_command.angular_radps,
        )
    assert max(abs(error) for error in curve_errors) <= 0.008
    assert abs(curve_errors[-1]) <= curve_rejoin_controller.limits.cross_track_deadband_m
    assert all(command >= -1e-6 for command in curve_commands)

    # 같은 곡선 바깥쪽에서 1cm 이탈한 경우에도 짧고 과도한 곡률로 더
    # 밀려나거나 중심선 반대편까지 가로지르지 않고 같은 쪽에서 합류한다.
    outer_curve_controller = production_controller()
    outer_curve_controller.set_path(curve_path)
    outer_curve_state = VehicleState(
        curve_path[0].x_m,
        curve_path[0].y_m - 0.01,
        curve_path[0].yaw_rad,
    )
    outer_curve_errors: list[float] = []
    for _ in range(120):
        outer_curve_command = outer_curve_controller.command(outer_curve_state)
        assert outer_curve_command.status == "TRACKING"
        outer_curve_errors.append(
            outer_curve_controller._forward_path_projection(
                outer_curve_state
            ).center_cross_track_m
        )
        if outer_curve_command.progress_index >= 90:
            break
        outer_curve_state = outer_curve_controller.propagate_state(
            outer_curve_state,
            outer_curve_command.linear_mps,
            outer_curve_command.angular_radps,
        )
    assert outer_curve_command.progress_index >= 90
    assert math.isclose(outer_curve_errors[0], -0.01, abs_tol=2e-4)
    assert min(outer_curve_errors) >= outer_curve_errors[0] - 0.001
    assert max(outer_curve_errors) <= 0.001
    assert hysteresis_crossings(outer_curve_errors, 0.001) == 0
    assert abs(outer_curve_errors[-1]) <= (
        outer_curve_controller.limits.cross_track_deadband_m
    )

    # 차량 중심 횡오차에는 중심 좌표의 기하 접선을 쓰고, 차체 heading에는
    # CSV의 body/rear yaw를 사용한다. 후진 중심 접선은 좌표 진행 방향에
    # 180도를 더한다.
    tangent_guard = DifferentialDriveMpc()
    tangent_guard.set_path(
        [
            ReferencePoint(index * 0.01, index * 0.01, 0.0, 1)
            for index in range(7)
        ]
    )
    assert abs(
        normalize_angle(tangent_guard._segment_tangent_yaw(0) - math.pi / 4.0)
    ) <= 1e-9
    reverse_tangent_guard = DifferentialDriveMpc()
    reverse_tangent_guard.set_path(
        [
            ReferencePoint(index * 0.01, 0.0, 0.0, -1)
            for index in range(7)
        ]
    )
    assert abs(
        normalize_angle(reverse_tangent_guard._segment_tangent_yaw(0) - math.pi)
    ) <= 1e-9

    # 긴 첫 후진 구간은 fallback 길이 제한보다 길다. 전환점을 조금
    # 지나친 뒤 원형 거리 오차가 다시 커져도 무한 후진하지 않고 다음
    # 기어로 전환해야 한다.
    passed_cusp_guard = production_controller()
    passed_cusp_path = [
        ReferencePoint(-index * 0.01, 0.0, 0.0, -1)
        for index in range(11)
    ] + [
        ReferencePoint(-0.10 + index * 0.01, 0.0, 0.0, 1)
        for index in range(1, 8)
    ]
    passed_cusp_guard.set_path(passed_cusp_path)
    passed_cusp_guard.restore_progress(9)
    passed_cusp_command = passed_cusp_guard.command(
        VehicleState(-0.13, 0.01, 0.0)
    )
    assert passed_cusp_command.status == "GEAR_CHANGE_REQUIRED"

    # 직선 뒤에 곡선이 가까워지면 현재점 곡률이 아직 0이어도 미래 곡률을
    # 약하게 반영해 코너 진입 전에 조향을 시작해야 한다.
    anticipation_guard = DifferentialDriveMpc()
    anticipation_path = [
        ReferencePoint(
            index * 0.005,
            0.0,
            0.0 if index < 12 else (index - 11) * 0.025,
            1,
        )
        for index in range(30)
    ]
    anticipation_guard.set_path(anticipation_path)
    assert abs(anticipation_guard._reference_curvature[0]) <= 1e-9
    assert anticipation_guard._anticipated_curvature(0, 29) > 0.0
    tiny_curve_guard = DifferentialDriveMpc()
    tiny_curve_guard.set_path(
        [
            ReferencePoint(
                index * 0.01,
                0.0,
                index * 0.002,
                1,
            )
            for index in range(20)
        ]
    )
    assert all(
        abs(value) <= 1e-9
        for value in tiny_curve_guard._reference_curvature
    )

    # 14 cm minimum-radius parking and all gear changes must pass with the exact
    # parameters used by the ROS node, not with dataclass fallback values.
    controller = production_controller()
    controller.set_path(path)
    state = VehicleState(path[0].x_m, path[0].y_m, path[0].yaw_rad)
    (
        final_state,
        steps,
        gear_changes,
        solve_times,
        maximum_angular_speed,
        maximum_cross_track_error,
    ) = simulate(
        controller,
        state,
        # Precision tuning deliberately slows down once lateral error exceeds
        # 0.5 cm, so the complete four-segment parking route needs more steps.
        maximum_steps=900,
    )
    final = path[-1]
    final_error = math.hypot(
        final.x_m - final_state.x_m,
        final.y_m - final_state.y_m,
    )
    final_yaw_error = abs(
        normalize_angle(final.yaw_rad - final_state.yaw_rad)
    )
    assert final_error <= controller.limits.goal_position_tolerance_m
    assert final_yaw_error <= controller.limits.goal_yaw_tolerance_rad
    expected_gear_changes = sum(
        first.direction != second.direction
        for first, second in zip(path, path[1:])
    )
    assert gear_changes == expected_gear_changes
    remaining_path_m = sum(
        math.hypot(second.x_m - first.x_m, second.y_m - first.y_m)
        for first, second in zip(
            path[controller.progress_index :],
            path[controller.progress_index + 1 :],
        )
    )
    assert (
        remaining_path_m
        <= controller.limits.goal_position_tolerance_m + 0.005
    )

    # 기어 전환 직후 cusp의 전진 마지막점이 더 가까워도 현재 후진 segment
    # 밖으로 nearest 검색이 돌아가면 안 된다.
    cusp_index = next(
        index
        for index in range(1, len(path))
        if path[index].direction != path[index - 1].direction
    )
    # 기본 0.5cm 전환 반경 밖이더라도 전환점 4cm 안에서 최적화 속도가
    # 사실상 0이면 보조 조건이 후진 segment로 넘길 수 있어야 한다.
    fallback_guard = DifferentialDriveMpc(
        replace(
            DifferentialDriveMpc().limits,
            gear_position_tolerance_m=0.005,
            gear_fallback_position_tolerance_m=0.04,
            gear_stall_speed_threshold_mps=0.10,
            gear_fallback_max_segment_length_m=10.0,
        )
    )
    fallback_guard.set_path(path)
    forward_cusp = path[cusp_index - 1]
    fallback_state = VehicleState(
        forward_cusp.x_m - 0.03 * math.cos(forward_cusp.yaw_rad),
        forward_cusp.y_m - 0.03 * math.sin(forward_cusp.yaw_rad),
        forward_cusp.yaw_rad,
    )
    fallback_command = fallback_guard.command(fallback_state)
    assert fallback_command.status == "GEAR_CHANGE_REQUIRED"

    reverse_start_guard = DifferentialDriveMpc()
    reverse_start_guard.set_path(path)
    reverse_start_guard.progress_index = cusp_index
    cusp_state = VehicleState(
        forward_cusp.x_m,
        forward_cusp.y_m,
        forward_cusp.yaw_rad,
    )
    guarded_index = reverse_start_guard._nearest_index(cusp_state)
    assert guarded_index >= cusp_index
    assert path[guarded_index].direction == -1
    reverse_command = reverse_start_guard.command(cusp_state)
    assert reverse_command.status == "TRACKING"
    assert reverse_command.progress_index >= cusp_index
    assert reverse_command.linear_mps < 0.0

    straight_guard = DifferentialDriveMpc()
    straight_guard.set_path(path)
    start = path[0]
    lateral_offset = 0.05
    offset_state = VehicleState(
        start.x_m - lateral_offset * math.sin(start.yaw_rad),
        start.y_m + lateral_offset * math.cos(start.yaw_rad),
        start.yaw_rad,
    )
    gentle_command = straight_guard.command(offset_state)
    assert gentle_command.status == "TRACKING"
    # 경로 오른쪽(+ lateral)에 있으면 map yaw가 감소하는 방향으로 복귀한다.
    assert gentle_command.curvature_1pm < 0.0
    assert gentle_command.linear_mps < straight_guard.limits.forward_speed_mps
    assert (
        abs(gentle_command.curvature_1pm)
        <= straight_guard.limits.straight_max_curvature_1pm + 1e-6
    )
    wrong_heading = VehicleState(
        start.x_m,
        start.y_m,
        normalize_angle(start.yaw_rad + math.radians(90.0)),
    )
    previous_curvature = straight_guard.last_curvature_1pm
    guarded_command = straight_guard.command(wrong_heading)
    assert guarded_command.status == "TRACKING"
    assert guarded_command.linear_mps > 0.0
    assert guarded_command.curvature_1pm < 0.0
    guarded_rate = straight_guard._adaptive_curvature_rate(
        0.0,
        guarded_command.linear_mps,
        1,
        1.0,
    )
    assert abs(guarded_command.curvature_1pm - previous_curvature) <= (
        guarded_rate * straight_guard.limits.dt_sec + 1e-9
    )

    flat_path = [
        value
        for point in path
        for value in (point.x_m, point.y_m, point.yaw_rad, point.direction)
    ]
    signature = trajectory_signature(flat_path)
    assert signature == trajectory_signature(list(flat_path))
    changed = list(flat_path)
    changed[-4] += 0.001
    assert signature != trajectory_signature(changed)

    scan_ranges = [0.10, 1.0, 1.0, 1.0, 0.14, 1.0, 1.0, 1.0, 0.11]
    front_minimum, rear_minimum = scan_sector_minima(
        scan_ranges,
        angle_min=-math.pi,
        angle_increment=math.pi / 4.0,
        range_min=0.05,
        range_max=10.0,
        front_half_angle_rad=math.radians(18.0),
        rear_half_angle_rad=math.radians(30.0),
    )
    assert math.isclose(front_minimum or 0.0, 0.14)
    assert math.isclose(rear_minimum or 0.0, 0.10)
    mounted_front, mounted_rear = scan_sector_minima(
        scan_ranges,
        angle_min=-math.pi,
        angle_increment=math.pi / 4.0,
        range_min=0.05,
        range_max=10.0,
        front_half_angle_rad=math.radians(18.0),
        rear_half_angle_rad=math.radians(30.0),
        front_center_angle_rad=math.pi,
        rear_center_angle_rad=0.0,
    )
    assert math.isclose(mounted_front or 0.0, 0.10)
    assert math.isclose(mounted_rear or 0.0, 0.14)
    invalid_front, invalid_rear = scan_sector_minima(
        [math.nan] * 9,
        angle_min=-math.pi,
        angle_increment=math.pi / 4.0,
        range_min=0.05,
        range_max=10.0,
        front_half_angle_rad=math.radians(18.0),
        rear_half_angle_rad=math.radians(30.0),
    )
    assert invalid_front is None and invalid_rear is None

    nonzero_solves = [value for value in solve_times if value > 0.0]
    median_solve = statistics.median(nonzero_solves)
    assert median_solve < controller.limits.dt_sec
    print("Differential-drive MPC START -> C2 simulation passed")
    print(f"Steps: {steps}, gear changes: {gear_changes}")
    print(f"Final position error: {final_error * 100.0:.2f} cm")
    print(f"Final yaw error: {math.degrees(final_yaw_error):.2f} deg")
    print(f"Maximum angular speed: {maximum_angular_speed:.3f} rad/s")
    print(
        f"Maximum cross-track error: {maximum_cross_track_error * 100.0:.2f} cm"
    )
    print(f"Median solve time: {median_solve * 1000.0:.1f} ms")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
