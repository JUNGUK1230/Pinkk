"""T자 후면주차의 staging stop과 실제 후진 maneuver 회귀 테스트."""

from pathlib import Path
import sys


SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parent
sys.path.append(str(SCRIPT_DIR))
sys.path.append(str(PROJECT_ROOT / "src"))

from plan_from_live_vision import (  # noqa: E402
    derive_reverse_parking_goal,
    load_planner_stack,
    plan_t_parking_and_validate,
)


def main() -> int:
    grid_map, planner, profile_config, limits, parking_config = load_planner_stack()
    goal, _, _ = derive_reverse_parking_goal(planner, "P6", grid_map.resolution_cm)
    _, result, trajectory, validation, staging = plan_t_parking_and_validate(
        planner,
        profile_config,
        limits,
        (84.0, 26.0, 1.914),
        goal,
        parking_config.required_final_direction,
    )
    assert validation.valid
    assert trajectory[-1].direction == -1
    assert sum(point.direction < 0 for point in trajectory) > 20
    # 동일 위치의 연속 stop 두 점은 T maneuver를 시작하기 전 정지·조향 reset이다.
    stage_stops = [index for index, point in enumerate(trajectory) if point.stop_required]
    assert any(
        abs(trajectory[index].x_cm - staging[0]) < 1e-6
        and abs(trajectory[index].y_cm - staging[1]) < 1e-6
        for index in stage_stops
    )

    print("T reverse parking regression passed")
    print(f"Staging pose: ({staging[0]:.1f}, {staging[1]:.1f})")
    print(f"Expanded nodes: {result.expanded_nodes}")
    print(f"Trajectory points: {len(trajectory)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
