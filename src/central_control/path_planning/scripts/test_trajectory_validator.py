"""제어기 전달 전 trajectory 안전 검사의 회귀 테스트."""

from dataclasses import replace
import math
from pathlib import Path
import sys

import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parent
sys.path.append(str(PROJECT_ROOT / "src"))

from hybrid_astar_planner import HybridAStarPlanner, HybridState  # noqa: E402
from trajectory_profile import build_trajectory_profile  # noqa: E402
from trajectory_validator import (  # noqa: E402
    TrajectoryValidationLimits,
    TrajectoryValidationResult,
    validate_trajectory,
)


def build_forward_reverse_path() -> list[HybridState]:
    """직진 후 정지하고 같은 heading으로 후진하는 일관된 경로를 만든다."""
    path = [
        HybridState(30.0 + 0.5 * index, 40.0, 0.0, 1, 0.0)
        for index in range(21)
    ]
    path.extend(
        HybridState(40.0 - 0.5 * index, 40.0, 0.0, -1, 0.0)
        for index in range(1, 11)
    )
    return path


def issue_codes(result: TrajectoryValidationResult) -> set[str]:
    return {issue.code for issue in result.issues}


def main() -> int:
    """정상 trajectory 통과와 주요 오류의 fail-closed 차단을 확인한다."""
    limits = TrajectoryValidationLimits()
    trajectory = build_trajectory_profile(
        build_forward_reverse_path(),
        wheelbase_cm=8.0,
        max_steer_rad=math.radians(30.0),
    )
    valid_result = validate_trajectory(
        trajectory,
        limits,
        collision_checker=lambda _x, _y, _yaw: False,
    )
    assert valid_result.valid, valid_result.issues
    assert not valid_result.issues
    assert valid_result.metrics.point_count == len(trajectory)
    assert valid_result.metrics.gear_switch_count == 1
    assert valid_result.metrics.max_spacing_cm <= 0.5 + 1e-9

    non_finite = list(trajectory)
    non_finite[5] = replace(non_finite[5], x_cm=math.nan)
    assert "NON_FINITE_VALUE" in issue_codes(
        validate_trajectory(non_finite, limits)
    )

    overspeed = list(trajectory)
    overspeed[8] = replace(overspeed[8], target_speed_mps=0.2)
    assert "SPEED_LIMIT" in issue_codes(validate_trajectory(overspeed, limits))

    wrong_direction = list(trajectory)
    wrong_direction[8] = replace(wrong_direction[8], target_speed_mps=-0.01)
    assert "SPEED_DIRECTION_MISMATCH" in issue_codes(
        validate_trajectory(wrong_direction, limits)
    )

    missing_stop = list(trajectory)
    missing_stop[20] = replace(missing_stop[20], stop_required=False)
    assert "MISSING_REQUIRED_STOP" in issue_codes(
        validate_trajectory(missing_stop, limits)
    )

    angular_mismatch = list(trajectory)
    angular_mismatch[8] = replace(
        angular_mismatch[8],
        target_angular_z_radps=0.1,
    )
    assert "ANGULAR_SPEED_MISMATCH" in issue_codes(
        validate_trajectory(angular_mismatch, limits)
    )

    excessive_steer = list(trajectory)
    steer_rad = math.radians(35.0)
    curvature = math.tan(steer_rad) / 0.08
    excessive_steer[8] = replace(
        excessive_steer[8],
        steer_rad=steer_rad,
        curvature_1pm=curvature,
        target_angular_z_radps=(
            excessive_steer[8].target_speed_mps * curvature
        ),
    )
    assert "STEERING_LIMIT" in issue_codes(
        validate_trajectory(excessive_steer, limits)
    )

    large_gap = list(trajectory)
    large_gap[8] = replace(large_gap[8], x_cm=large_gap[8].x_cm + 2.0)
    assert "SPACING_LIMIT" in issue_codes(validate_trajectory(large_gap, limits))

    yaw_jump = list(trajectory)
    yaw_jump[8] = replace(yaw_jump[8], yaw_rad=math.radians(20.0))
    assert "YAW_JUMP" in issue_codes(validate_trajectory(yaw_jump, limits))

    excessive_acceleration = list(trajectory)
    excessive_acceleration[1] = replace(
        excessive_acceleration[1],
        target_speed_mps=0.05,
    )
    assert "ACCELERATION_LIMIT" in issue_codes(
        validate_trajectory(excessive_acceleration, limits)
    )

    collision_result = validate_trajectory(
        trajectory,
        limits,
        collision_checker=lambda x, _y, _yaw: 34.9 <= x <= 35.1,
    )
    assert "FOOTPRINT_COLLISION" in issue_codes(collision_result)

    # 실제 planner의 곡선·smoothing·속도 프로파일 조합도 같은 gate를 통과한다.
    planner = HybridAStarPlanner(
        np.zeros((140, 140), dtype=np.uint8),
        minimum_turning_radius_cm=14.0,
        analytic_expansion_distance_cm=100.0,
    )
    planned = planner.plan(
        (40.0, 40.0, 0.0),
        (70.0, 75.0, math.pi / 2.0),
    )
    assert planned.success, planned.message
    planned_trajectory = build_trajectory_profile(
        planned.path,
        wheelbase_cm=planner.wheelbase_cm,
        max_steer_rad=max(abs(value) for value in planner.steer_set_rad),
    )
    planned_limits = replace(
        limits,
        max_spacing_cm=planner.path_output_step_cm,
        max_steer_change_rad_per_cm=(
            planner.max_steer_change_rad / planner.motion_step_cm
        ),
    )
    planned_validation = validate_trajectory(
        planned_trajectory,
        planned_limits,
        collision_checker=planner.is_pose_collision,
    )
    assert planned_validation.valid, planned_validation.issues

    empty_result = validate_trajectory([], limits)
    assert not empty_result.valid
    assert "EMPTY_TRAJECTORY" in issue_codes(empty_result)

    print("Trajectory validator regression passed")
    print(f"Valid points: {valid_result.metrics.point_count}")
    print(f"Gear switches: {valid_result.metrics.gear_switch_count}")
    print(f"Planned curve points: {planned_validation.metrics.point_count}")
    print(
        "Planned maxima: "
        f"spacing={planned_validation.metrics.max_spacing_cm:.3f} cm, "
        f"steer={planned_validation.metrics.max_abs_steer_deg:.3f} deg, "
        f"speed={planned_validation.metrics.max_abs_speed_mps:.3f} m/s"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
