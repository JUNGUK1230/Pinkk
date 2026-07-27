"""T자 후면주차를 통로 이동과 주차 maneuver로 분리하는 planner."""

from dataclasses import dataclass
import heapq
import itertools
import math
from typing import Iterable

import cv2
import numpy as np

try:
    from .hybrid_astar_planner import HybridAStarPlanner, HybridAStarResult, HybridState
    from .reeds_shepp import ReedsSheppPlanner
except ImportError:
    from hybrid_astar_planner import HybridAStarPlanner, HybridAStarResult, HybridState
    from reeds_shepp import ReedsSheppPlanner


Pose = tuple[float, float, float]


@dataclass(frozen=True)
class TParkingPlanResult:
    """통로 접근과 T자 후진 maneuver를 연결한 결과."""

    path: list[HybridState]
    staging_pose: Pose
    global_result: HybridAStarResult
    maneuver_result: HybridAStarResult
    stage_stop_indices: tuple[int, ...]

    @property
    def expanded_nodes(self) -> int:
        return self.global_result.expanded_nodes + self.maneuver_result.expanded_nodes

    @property
    def total_cost(self) -> float:
        return self.global_result.total_cost + self.maneuver_result.total_cost


@dataclass(frozen=True)
class ReverseOnlyManeuverResult:
    """A collision-free final parking maneuver with no gear changes."""

    path: list[HybridState]
    staging_pose: Pose
    stop_indices: tuple[int, ...]
    total_length_cm: float


def plan_t_reverse_parking(
    planner: HybridAStarPlanner,
    start: Pose,
    goal: Pose,
    required_final_direction: int = -1,
    minimum_reverse_distance_cm: float = 10.0,
    approach_timeout_sec: float = 3.0,
    max_staging_candidates: int = 4,
    guide_clearance_cm: float = 5.0,
    guide_segment_length_cm: float = 12.0,
) -> TParkingPlanResult:
    """2D guide → 분할 Hybrid 접근 → T자 후진주차 순서로 탐색한다.

    주차칸 final pose의 앞(입구 방향) 15~30cm 통로에 staging pose를 둔다.
    staging yaw는 주차칸 입구 방향의 좌/우 90도, 즉 통로 진행 방향이다.
    2D A* guide를 따라 짧은 Hybrid A* 접근 구간을 만든 뒤 staging에서
    final pose까지 전진·후진 maneuver를 탐색한다. 따라서 지도 전체에서
    주차 maneuver까지 동시에 탐색하는 방식보다 상태공간이 작다.
    """
    if required_final_direction != -1:
        raise ValueError("T reverse parking requires final direction -1")
    if minimum_reverse_distance_cm <= 0.0:
        raise ValueError("minimum reverse distance must be positive")
    if approach_timeout_sec <= 0.0 or max_staging_candidates <= 0:
        raise ValueError("T parking approach limits must be positive")
    if guide_clearance_cm < 0.0 or guide_segment_length_cm <= 0.0:
        raise ValueError("guide clearance and segment length must be valid")

    failures: list[str] = []
    for candidate_index, staging_pose in enumerate(
        _staging_candidates(planner, goal)
    ):
        if candidate_index >= max_staging_candidates:
            break
        if planner.is_pose_collision(*staging_pose):
            continue

        # 긴 전역 Hybrid A* 한 번으로 통로와 정확한 staging yaw를 동시에
        # 찾으면 analytic smoothing 후보를 수천 번 거절할 수 있다. 먼저
        # footprint 반폭만큼 장애물을 확장한 2D A* guide를 만들고, 약 12cm
        # 간격의 짧은 Hybrid A* 구간으로 나눠 상태공간을 제한한다.
        try:
            global_path, guide_stop_indices = _plan_guided_approach(
                planner,
                start,
                staging_pose,
                approach_timeout_sec=approach_timeout_sec,
                guide_clearance_cm=guide_clearance_cm,
                guide_segment_length_cm=guide_segment_length_cm,
            )
        except (RuntimeError, ValueError) as error:
            failures.append(f"approach: {error}")
            continue

        # 각 guide 구간은 허용오차 안에서 끝날 수 있으므로 실제 마지막 pose를
        # maneuver 시작점으로 사용한다. 두 경로 사이 위치/yaw 간극이 없다.
        actual_stage = global_path.path[-1]
        maneuver_start = (
            actual_stage.x_cm,
            actual_stage.y_cm,
            actual_stage.yaw_rad,
        )
        maneuver = planner.plan(
            maneuver_start,
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

        # 두 planner의 동일 staging pose를 의도적으로 중복 저장한다. 제어기는
        # 이 두 점에서 정지한 뒤 조향을 바꾸고 주차 maneuver를 시작할 수 있다.
        stage_index = len(global_path.path) - 1
        combined = list(global_path.path) + list(maneuver.path)
        stop_indices = tuple(guide_stop_indices) + (
            stage_index,
            stage_index + 1,
        )
        return TParkingPlanResult(
            path=combined,
            staging_pose=maneuver_start,
            global_result=global_path,
            maneuver_result=maneuver,
            stage_stop_indices=stop_indices,
        )

    detail = failures[-1] if failures else "no footprint-valid staging pose"
    raise RuntimeError(f"T reverse parking failed: {detail}")


def plan_reverse_only_maneuver(
    planner: HybridAStarPlanner,
    goal: Pose,
    minimum_reverse_distance_cm: float = 10.0,
    max_staging_candidates: int = 40,
) -> ReverseOnlyManeuverResult:
    """Find a final maneuver made exclusively of reverse bounded-curvature motion.

    Unlike the general Hybrid A* maneuver, this function rejects every candidate
    containing a forward segment.  A duplicated stopped pose is inserted at each
    steering-mode transition so the existing trajectory validator can permit the
    steering reset without interpreting it as an in-place vehicle rotation.
    """
    if minimum_reverse_distance_cm <= 0.0 or max_staging_candidates <= 0:
        raise ValueError("reverse-only maneuver limits must be positive")

    reeds_shepp = ReedsSheppPlanner(
        planner.minimum_turning_radius_cm,
        step_size_cm=planner.path_output_step_cm,
    )
    for candidate_index, staging_pose in enumerate(_staging_candidates(planner, goal)):
        if candidate_index >= max_staging_candidates:
            break
        if planner.is_pose_collision(*staging_pose):
            continue
        for candidate in reeds_shepp.iter_candidates(staging_pose, goal):
            if not candidate.poses or any(pose.direction != -1 for pose in candidate.poses):
                continue
            if candidate.total_length_cm + 1e-6 < minimum_reverse_distance_cm:
                continue
            if not planner.is_path_collision_free(candidate.poses):
                continue
            path, stop_indices = _states_with_steering_reset_stops(planner, candidate.poses)
            return ReverseOnlyManeuverResult(
                path=path,
                staging_pose=staging_pose,
                stop_indices=tuple(stop_indices),
                total_length_cm=candidate.total_length_cm,
            )
    raise RuntimeError("no collision-free reverse-only maneuver was found")


def _states_with_steering_reset_stops(
    planner: HybridAStarPlanner,
    poses: Iterable[object],
) -> tuple[list[HybridState], set[int]]:
    """Convert analytic poses and stop at mode transitions for steering reset."""
    states: list[HybridState] = []
    stop_indices: set[int] = set()
    for pose in poses:
        mode = getattr(pose, "segment_mode")
        steer_rad = (
            planner.analytic_steer_rad
            if mode == "L"
            else -planner.analytic_steer_rad
            if mode == "R"
            else 0.0
        )
        state = HybridState(
            float(getattr(pose, "x_cm")),
            float(getattr(pose, "y_cm")),
            float(getattr(pose, "yaw_rad")),
            int(getattr(pose, "direction")),
            steer_rad,
        )
        if states and abs(state.steer_rad - states[-1].steer_rad) > 1e-12:
            previous = states[-1]
            states.append(
                HybridState(
                    previous.x_cm,
                    previous.y_cm,
                    previous.yaw_rad,
                    previous.direction,
                    state.steer_rad,
                )
            )
            stop_indices.update((len(states) - 2, len(states) - 1))
        states.append(state)
    return states, stop_indices


def _plan_guided_approach(
    planner: HybridAStarPlanner,
    start: Pose,
    staging_pose: Pose,
    approach_timeout_sec: float,
    guide_clearance_cm: float,
    guide_segment_length_cm: float,
) -> tuple[HybridAStarResult, tuple[int, ...]]:
    """빠른 2D 통로 guide를 짧은 kinematic Hybrid 구간으로 변환한다."""
    guide_grid = _inflate_guide_grid(planner, guide_clearance_cm)
    start_cell = (
        int(round(start[0] / planner.resolution_cm)),
        int(round(start[1] / planner.resolution_cm)),
    )
    goal_cell = (
        int(round(staging_pose[0] / planner.resolution_cm)),
        int(round(staging_pose[1] / planner.resolution_cm)),
    )
    # 주차면 안의 시작점과 통로 staging은 원형 inflation에서 경계 셀로
    # 잡힐 수 있다. 실제 안전성은 각 Hybrid pose의 회전 footprint로 다시
    # 검사하므로 guide의 두 endpoint만 열어 연결성을 구한다.
    for cell in (start_cell, goal_cell):
        if not _in_bounds(guide_grid, cell):
            raise RuntimeError(f"guide endpoint is outside the map: {cell}")
        guide_grid[cell[1], cell[0]] = 0
    guide_cells = _astar_grid_path(
        guide_grid,
        start_cell,
        goal_cell,
        planner.obstacle_threshold,
    )
    if not guide_cells:
        raise RuntimeError("2D corridor guide was not found")

    first_guide_index = min(3, len(guide_cells) - 1)
    first_guide_yaw = math.atan2(
        guide_cells[first_guide_index][1] - guide_cells[0][1],
        guide_cells[first_guide_index][0] - guide_cells[0][0],
    )
    # 차량이 통로와 45도 넘게 어긋나 있으면 짧은 첫 목표가 급회전을
    # 요구한다. 이 경우만 30cm 구간을 사용해 회전할 공간을 확보한다.
    effective_segment_length_cm = (
        max(guide_segment_length_cm, 30.0)
        if abs(_normalize_yaw(first_guide_yaw - start[2]))
        > math.radians(45.0)
        else guide_segment_length_cm
    )
    targets = _sample_guide_targets(
        guide_cells,
        start,
        staging_pose,
        planner.resolution_cm,
        effective_segment_length_cm,
    )
    original_timeout = planner.timeout_sec
    planner.timeout_sec = min(original_timeout, approach_timeout_sec)
    combined: list[HybridState] = []
    stop_indices: list[int] = []
    total_cost = 0.0
    expanded_nodes = 0
    messages: list[str] = []
    current = start
    try:
        for target_index, target in enumerate(targets, start=1):
            segment = planner.plan(
                current,
                target,
                holonomic_cost_to_goal=planner.build_holonomic_cost_to_goal(
                    target[0], target[1]
                ),
                require_smoothed_path=True,
                require_goal_heading=True,
            )
            if not segment.success:
                raise RuntimeError(
                    f"guide segment {target_index}/{len(targets)}: "
                    f"{segment.message}"
                )
            if combined:
                # 동일 위치의 두 점을 stop으로 만들면 정지 상태에서 다음
                # segment의 조향각으로 안전하게 재설정할 수 있다.
                stop_indices.extend((len(combined) - 1, len(combined)))
            combined.extend(segment.path)
            total_cost += segment.total_cost
            expanded_nodes += segment.expanded_nodes
            messages.append(segment.message)
            last = segment.path[-1]
            current = (last.x_cm, last.y_cm, last.yaw_rad)
    finally:
        planner.timeout_sec = original_timeout

    return (
        HybridAStarResult(
            combined,
            total_cost,
            True,
            expanded_nodes,
            f"2D-guided approach ({len(targets)} segments): "
            + "; ".join(messages),
        ),
        tuple(stop_indices),
    )


def _inflate_guide_grid(
    planner: HybridAStarPlanner,
    clearance_cm: float,
) -> np.ndarray:
    """2D guide에만 쓰는 중심점 clearance grid를 만든다."""
    obstacle = (planner.grid >= planner.obstacle_threshold).astype(np.uint8)
    radius_cells = int(math.ceil(clearance_cm / planner.resolution_cm))
    if radius_cells > 0:
        size = radius_cells * 2 + 1
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (size, size))
        obstacle = cv2.dilate(obstacle, kernel, iterations=1)
    guide_grid = np.zeros_like(planner.grid, dtype=np.uint8)
    guide_grid[obstacle > 0] = 100
    return guide_grid


def _astar_grid_path(
    grid: np.ndarray,
    start: tuple[int, int],
    goal: tuple[int, int],
    obstacle_threshold: int,
) -> list[tuple[int, int]]:
    """8-connected 2D A*로 통로 중심 guide를 빠르게 계산한다."""
    if start == goal:
        return [start]
    motions = (
        (1, 0, 1.0),
        (-1, 0, 1.0),
        (0, 1, 1.0),
        (0, -1, 1.0),
        (1, 1, math.sqrt(2.0)),
        (1, -1, math.sqrt(2.0)),
        (-1, 1, math.sqrt(2.0)),
        (-1, -1, math.sqrt(2.0)),
    )
    counter = itertools.count()
    open_heap: list[tuple[float, int, tuple[int, int]]] = [
        (
            math.hypot(goal[0] - start[0], goal[1] - start[1]),
            next(counter),
            start,
        )
    ]
    parents: dict[tuple[int, int], tuple[int, int]] = {}
    costs = {start: 0.0}
    closed: set[tuple[int, int]] = set()
    while open_heap:
        _, _, current = heapq.heappop(open_heap)
        if current in closed:
            continue
        if current == goal:
            path = [current]
            while current in parents:
                current = parents[current]
                path.append(current)
            path.reverse()
            return path
        closed.add(current)
        for dx, dy, move_cost in motions:
            neighbor = (current[0] + dx, current[1] + dy)
            if (
                not _in_bounds(grid, neighbor)
                or grid[neighbor[1], neighbor[0]] >= obstacle_threshold
            ):
                continue
            if dx and dy and (
                grid[current[1], current[0] + dx] >= obstacle_threshold
                or grid[current[1] + dy, current[0]] >= obstacle_threshold
            ):
                continue
            tentative = costs[current] + move_cost
            if tentative >= costs.get(neighbor, math.inf):
                continue
            costs[neighbor] = tentative
            parents[neighbor] = current
            priority = tentative + math.hypot(
                goal[0] - neighbor[0],
                goal[1] - neighbor[1],
            )
            heapq.heappush(open_heap, (priority, next(counter), neighbor))
    return []


def _sample_guide_targets(
    guide_cells: list[tuple[int, int]],
    start: Pose,
    staging_pose: Pose,
    resolution_cm: float,
    segment_length_cm: float,
) -> list[Pose]:
    """2D guide를 일정 길이마다 잘라 각 짧은 Hybrid 목표 pose를 만든다."""
    sampled: list[tuple[float, float]] = []
    accumulated_cm = 0.0
    previous = guide_cells[0]
    for cell in guide_cells[1:-1]:
        accumulated_cm += (
            math.hypot(cell[0] - previous[0], cell[1] - previous[1])
            * resolution_cm
        )
        previous = cell
        if accumulated_cm >= segment_length_cm:
            point = (cell[0] * resolution_cm, cell[1] * resolution_cm)
            if math.hypot(
                point[0] - staging_pose[0],
                point[1] - staging_pose[1],
            ) >= max(_guide_endpoint_skip_cm(segment_length_cm), resolution_cm):
                sampled.append(point)
            accumulated_cm = 0.0

    targets: list[Pose] = []
    previous_point = (start[0], start[1])
    previous_yaw = start[2]
    maximum_guide_yaw_change = math.radians(15.0)
    for point in sampled:
        desired_yaw = math.atan2(
            point[1] - previous_point[1],
            point[0] - previous_point[0],
        )
        yaw_change = _normalize_yaw(desired_yaw - previous_yaw)
        yaw = _normalize_yaw(
            previous_yaw
            + max(
                -maximum_guide_yaw_change,
                min(maximum_guide_yaw_change, yaw_change),
            )
        )
        targets.append((point[0], point[1], yaw))
        previous_point = point
        previous_yaw = yaw
    targets.append(staging_pose)
    return targets


def _guide_endpoint_skip_cm(segment_length_cm: float) -> float:
    """마지막 staging과 너무 가까운 중복 guide 목표를 제거한다."""
    return min(8.0, segment_length_cm * 0.4)


def _in_bounds(grid: np.ndarray, point: tuple[int, int]) -> bool:
    return 0 <= point[0] < grid.shape[1] and 0 <= point[1] < grid.shape[0]


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
