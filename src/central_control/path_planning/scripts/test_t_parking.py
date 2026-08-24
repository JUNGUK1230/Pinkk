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
    goal, _, _ = derive_reverse_parking_goal(planner, "P5", grid_map.resolution_cm)
    for start in ((84.0, 26.0, 1.914), (68.0, 26.0, 0.7207016179371806)):
        _, result, trajectory, validation, staging = plan_t_parking_and_validate(
            planner,
            profile_config,
            limits,
            start,
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

    # 첫 주차 단계가 끝난 P5 자세에서 충전 구역으로 이동하는 실제 다음
    # 에피소드를 검사한다. C2 우선 경로와 C2 점유 시 C1 대체 경로 모두
    # 2D guide + 짧은 Hybrid 구간 + 마지막 후진 maneuver를 통과해야 한다.
    charge_results = {}
    for parked_slot in ("P5", "P6", "P7", "P8"):
        parked_start, _, _ = derive_reverse_parking_goal(
            planner,
            parked_slot,
            grid_map.resolution_cm,
        )
        for charge_slot in ("C2", "C1"):
            charge_goal, _, _ = derive_reverse_parking_goal(
                planner,
                charge_slot,
                grid_map.resolution_cm,
            )
            _, charge_result, charge_trajectory, charge_validation, _ = (
                plan_t_parking_and_validate(
                    planner,
                    profile_config,
                    limits,
                    parked_start,
                    charge_goal,
                    parking_config.required_final_direction,
                )
            )
            assert charge_validation.valid
            assert charge_trajectory[-1].direction == -1
            assert sum(point.direction < 0 for point in charge_trajectory) > 20
            charge_results[(parked_slot, charge_slot)] = (
                charge_result.expanded_nodes,
                len(charge_trajectory),
            )

    print("T reverse parking regression passed")
    print(f"Staging pose: ({staging[0]:.1f}, {staging[1]:.1f})")
    print(f"Expanded nodes: {result.expanded_nodes}")
    print(f"Trajectory points: {len(trajectory)}")
    for (
        parked_slot,
        charge_slot,
    ), (expanded_nodes, point_count) in charge_results.items():
        print(
            f"{parked_slot} -> {charge_slot}: expanded={expanded_nodes}, "
            f"trajectory points={point_count}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
