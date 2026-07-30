"""차동구동 MPC가 실제 START→C2 전후진 경로를 추종하는지 검사한다."""

import csv
import math
from pathlib import Path
import statistics
import sys


SCRIPT_DIR = Path(__file__).resolve().parent
SRC_ROOT = SCRIPT_DIR.parents[1]
REPO_ROOT = SRC_ROOT.parent
sys.path.insert(0, str(SRC_ROOT))

from vehicle_control.mpc_controller import (  # noqa: E402
    DifferentialDriveMpc,
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


def simulate(
    controller: DifferentialDriveMpc,
    state: VehicleState,
    maximum_steps: int,
) -> tuple[VehicleState, int, int, list[float], float]:
    gear_changes = 0
    solve_times: list[float] = []
    maximum_angular_speed = 0.0
    for step in range(maximum_steps):
        command = controller.command(state)
        solve_times.append(command.solve_time_sec)
        maximum_angular_speed = max(
            maximum_angular_speed,
            abs(command.angular_radps),
        )
        assert (
            abs(command.angular_radps) <= controller.limits.max_angular_speed_radps + 1e-6
        )
        if abs(command.linear_mps) <= 1e-10:
            assert abs(command.angular_radps) <= 1e-10
        if command.status == "GEAR_CHANGE_REQUIRED":
            assert controller.advance_gear_segment()
            gear_changes += 1
            continue
        if command.status == "GOAL_REACHED":
            return state, step, gear_changes, solve_times, maximum_angular_speed
        assert command.status == "TRACKING", command.status
        dt = controller.limits.dt_sec
        state = VehicleState(
            x_m=state.x_m
            + dt * command.linear_mps * math.cos(state.yaw_rad),
            y_m=state.y_m
            + dt * command.linear_mps * math.sin(state.yaw_rad),
            yaw_rad=normalize_angle(
                state.yaw_rad + dt * command.angular_radps
            ),
        )
    raise AssertionError("MPC did not reach the goal within the simulation limit")


def main() -> int:
    path = load_c2_path()
    controller = DifferentialDriveMpc()
    controller.set_path(path)
    state = VehicleState(path[0].x_m, path[0].y_m, path[0].yaw_rad)
    final_state, steps, gear_changes, solve_times, maximum_angular_speed = simulate(
        controller,
        state,
        maximum_steps=250,
    )
    final = path[-1]
    final_error = math.hypot(
        final.x_m - final_state.x_m,
        final.y_m - final_state.y_m,
    )
    assert final_error <= controller.limits.goal_position_tolerance_m
    assert gear_changes == 1
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
    assert (
        abs(gentle_command.curvature_1pm)
        <= straight_guard.limits.straight_max_curvature_1pm + 1e-6
    )
    wrong_heading = VehicleState(
        start.x_m,
        start.y_m,
        normalize_angle(start.yaw_rad + math.radians(90.0)),
    )
    guarded_command = straight_guard.command(wrong_heading)
    assert guarded_command.status == "HEADING_ERROR_TOO_LARGE"
    assert guarded_command.linear_mps == 0.0
    assert guarded_command.angular_radps == 0.0

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
    print(f"Maximum angular speed: {maximum_angular_speed:.3f} rad/s")
    print(f"Median solve time: {median_solve * 1000.0:.1f} ms")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
