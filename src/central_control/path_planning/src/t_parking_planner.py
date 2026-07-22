"""T자 후면주차를 통로 이동과 주차 maneuver로 분리하는 planner."""

from dataclasses import dataclass
import math
from typing import Iterable

try:
    from .hybrid_astar_planner import HybridAStarPlanner, HybridAStarResult, HybridState
except ImportError:
    from hybrid_astar_planner import HybridAStarPlanner, HybridAStarResult, HybridState


Pose = tuple[float, float, float]


@dataclass(frozen=True)
class TParkingPlanResult:
    """통로 접근과 T자 후진 maneuver를 연결한 결과."""

    path: list[HybridState]
    staging_pose: Pose
    global_result: HybridAStarResult
    maneuver_result: HybridAStarResult
    stage_stop_indices: tuple[int, int]

    @property
    def expanded_nodes(self) -> int:
        return self.global_result.expanded_nodes + self.maneuver_result.expanded_nodes

    @property
    def total_cost(self) -> float:
        return self.global_result.total_cost + self.maneuver_result.total_cost


def plan_t_reverse_parking(
    planner: HybridAStarPlanner,
    start: Pose,
    goal: Pose,
    required_final_direction: int = -1,
    minimum_reverse_distance_cm: float = 10.0,
    staging_goal_tolerance_cm: float = 1.0,
    approach_timeout_sec: float = 3.0,
    max_staging_candidates: int = 4,
) -> TParkingPlanResult:
    """통로 → 정지/조향 → T자 후진주차 순서로 두 번의 작은 탐색을 수행한다.

    주차칸 final pose의 앞(입구 방향) 15~30cm 통로에 staging pose를 둔다.
    staging yaw는 주차칸 입구 방향의 좌/우 90도, 즉 통로 진행 방향이다.
    첫 Hybrid A*는 현재 위치에서 staging까지, 두 번째 Hybrid A*는 staging에서
    final pose까지 전진·후진 maneuver만 탐색한다. 따라서 지도 전체에서 주차
    maneuver까지 동시에 탐색하는 기존 방식보다 상태공간이 작다.
    """
    if required_final_direction != -1:
        raise ValueError("T reverse parking requires final direction -1")
    if minimum_reverse_distance_cm <= 0.0:
        raise ValueError("minimum reverse distance must be positive")
    if approach_timeout_sec <= 0.0 or max_staging_candidates <= 0:
        raise ValueError("T parking approach limits must be positive")

    failures: list[str] = []
    for candidate_index, staging_pose in enumerate(
        _staging_candidates(planner, goal)
    ):
        if candidate_index >= max_staging_candidates:
            break
        if planner.is_pose_collision(*staging_pose):
            continue

        # 주차 maneuver가 가능한 staging만 전역 통로 탐색 대상으로 삼는다.
        maneuver = planner.plan(
            staging_pose,
            goal,
            holonomic_cost_to_goal=planner.build_holonomic_cost_to_goal(
                goal[0], goal[1]
            ),
            require_smoothed_path=True,
            required_goal_direction=required_final_direction,
            min_final_direction_distance_cm=minimum_reverse_distance_cm,
        )
        if not maneuver.success:
            failures.append(f"maneuver: {maneuver.message}")
            continue

        # staging pose에서 오차가 남으면 두 결과를 연결할 때 yaw/spacing jump가
        # 생긴다. 1cm 이내 exact staging 연결만 허용한다.
        original_tolerance = planner.goal_tolerance_cm
        original_timeout = planner.timeout_sec
        planner.goal_tolerance_cm = min(
            original_tolerance,
            staging_goal_tolerance_cm,
        )
        planner.timeout_sec = min(original_timeout, approach_timeout_sec)
        try:
            global_path = planner.plan(
                start,
                staging_pose,
                holonomic_cost_to_goal=planner.build_holonomic_cost_to_goal(
                    staging_pose[0], staging_pose[1]
                ),
                require_smoothed_path=True,
                require_goal_heading=True,
            )
        finally:
            planner.goal_tolerance_cm = original_tolerance
            planner.timeout_sec = original_timeout
        if not global_path.success:
            failures.append(f"approach: {global_path.message}")
            continue

        # 두 planner의 동일 staging pose를 의도적으로 중복 저장한다. 제어기는
        # 이 두 점에서 정지한 뒤 조향을 바꾸고 주차 maneuver를 시작할 수 있다.
        stage_index = len(global_path.path) - 1
        combined = list(global_path.path) + list(maneuver.path)
        return TParkingPlanResult(
            path=combined,
            staging_pose=staging_pose,
            global_result=global_path,
            maneuver_result=maneuver,
            stage_stop_indices=(stage_index, stage_index + 1),
        )

    detail = failures[-1] if failures else "no footprint-valid staging pose"
    raise RuntimeError(f"T reverse parking failed: {detail}")


def _staging_candidates(
    planner: HybridAStarPlanner,
    goal: Pose,
) -> Iterable[Pose]:
    """주차칸 입구 앞 통로에서 T자 maneuver 시작 pose 후보를 만든다."""
    goal_x, goal_y, goal_yaw = goal
    forward = (math.cos(goal_yaw), math.sin(goal_yaw))
    left = (-forward[1], forward[0])

    # 먼저 바로 앞 통로 중앙의 간결한 T maneuver를 시도하고, 막혀 있으면
    # 전후/좌우 여유 후보를 넓힌다. 주차칸 이름에는 의존하지 않는다.
    for approach_cm in (20.0, 25.0, 15.0, 30.0):
        for lateral_cm in (0.0, -5.0, 5.0, -10.0, 10.0):
            for turn_side in (1.0, -1.0):
                yield (
                    goal_x + approach_cm * forward[0] + lateral_cm * left[0],
                    goal_y + approach_cm * forward[1] + lateral_cm * left[1],
                    _normalize_yaw(goal_yaw + turn_side * math.pi / 2.0),
                )


def _normalize_yaw(yaw_rad: float) -> float:
    return (yaw_rad + math.pi) % (2.0 * math.pi) - math.pi
