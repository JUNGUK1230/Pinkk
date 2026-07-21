"""Hybrid A*와 충돌 안전 Reeds-Shepp 목표 연결의 회귀 테스트."""

import json
import math
from pathlib import Path
import sys
import tempfile

import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parent
sys.path.append(str(PROJECT_ROOT / "src"))

from hybrid_astar_planner import HybridAStarPlanner, HybridState  # noqa: E402
from path_smoothing import save_path_smoothing_stats  # noqa: E402


def make_planner(
    grid: np.ndarray,
    expansion_distance_cm: float = 80.0,
    smoothing_enabled: bool = True,
) -> HybridAStarPlanner:
    """실차 제원과 analytic expansion을 사용하는 테스트 planner를 만든다."""
    return HybridAStarPlanner(
        grid,
        resolution_cm=1.0,
        wheelbase_cm=8.0,
        vehicle_length_cm=12.0,
        vehicle_width_cm=8.0,
        rear_overhang_cm=2.0,
        minimum_turning_radius_cm=14.0,
        motion_step_cm=3.0,
        path_output_step_cm=0.5,
        timeout_sec=5.0,
        analytic_expansion_enabled=True,
        analytic_expansion_distance_cm=expansion_distance_cm,
        path_smoothing_enabled=smoothing_enabled,
    )


def assert_continuous_safe_path(
    planner: HybridAStarPlanner,
    path: list[HybridState],
    goal: tuple[float, float, float],
) -> None:
    """정확한 끝 pose, 0.5cm 간격, yaw 연속성과 footprint를 확인한다."""
    assert path
    endpoint = path[-1]
    assert math.hypot(endpoint.x_cm - goal[0], endpoint.y_cm - goal[1]) < 1e-8
    assert abs(planner._angle_difference(endpoint.yaw_rad, goal[2])) < 1e-8
    assert planner.is_path_collision_free(path)

    position_gaps = [
        math.hypot(second.x_cm - first.x_cm, second.y_cm - first.y_cm)
        for first, second in zip(path, path[1:])
    ]
    yaw_gaps = [
        abs(planner._angle_difference(second.yaw_rad, first.yaw_rad))
        for first, second in zip(path, path[1:])
    ]
    assert position_gaps
    assert max(position_gaps) <= planner.path_output_step_cm + 1e-6
    assert max(yaw_gaps) <= planner.path_output_step_cm / 14.0 + 1e-6


def main() -> int:
    """직접 연결과 충돌 후보 거부 후 탐색 fallback을 검증한다."""
    free_grid = np.zeros((120, 120), dtype=np.uint8)
    direct_planner = make_planner(free_grid)
    direct_goal = (90.0, 60.0, 0.0)
    direct_result = direct_planner.plan((30.0, 60.0, 0.0), direct_goal)
    assert direct_result.success, direct_result.message
    assert "Reeds-Shepp" in direct_result.message
    assert direct_result.expanded_nodes == 0
    assert "curvature-smoothed path accepted" in direct_result.message
    assert_continuous_safe_path(direct_planner, direct_result.path, direct_goal)

    curved_goal = (70.0, 75.0, math.pi / 2.0)
    curved_result = direct_planner.plan((40.0, 40.0, 0.0), curved_goal)
    assert curved_result.success, curved_result.message
    assert "curvature-smoothed path accepted" in curved_result.message
    assert curved_result.smoothing_stats is not None
    assert curved_result.smoothing_stats.attempted
    assert curved_result.smoothing_stats.accepted
    assert curved_result.smoothing_stats.candidate is not None
    assert curved_result.smoothing_stats.raw.pose_count != (
        curved_result.smoothing_stats.final.pose_count
    )
    with tempfile.TemporaryDirectory() as temporary_dir:
        stats_path = Path(temporary_dir) / "hybrid_smoothing_stats.json"
        save_path_smoothing_stats(
            curved_result.smoothing_stats,
            stats_path,
            curved_result.message,
        )
        saved_stats = json.loads(stats_path.read_text(encoding="utf-8"))
        assert saved_stats["smoothing"]["accepted"] is True
        assert saved_stats["smoothing"]["raw"]["pose_count"] == (
            curved_result.smoothing_stats.raw.pose_count
        )
        assert saved_stats["smoothing"]["final"]["max_spacing_cm"] <= 0.5
    assert_continuous_safe_path(direct_planner, curved_result.path, curved_goal)
    assert max(abs(state.steer_rad) for state in curved_result.path) <= math.radians(
        30.0
    ) + 1e-6

    # Spline이 raw swept area 밖으로 약간 이동하는 위치에 장애물을 둔다.
    # raw 경로는 안전하지만 smoothing 결과만 충돌하므로 raw fallback이어야 한다.
    raw_planner = make_planner(free_grid, smoothing_enabled=False)
    raw_curved_result = raw_planner.plan((40.0, 40.0, 0.0), curved_goal)
    assert raw_curved_result.success
    smoothing_blocked_grid = free_grid.copy()
    smoothing_blocked_grid[39, 56] = 100
    smoothing_blocked_planner = make_planner(smoothing_blocked_grid)
    assert smoothing_blocked_planner.is_path_collision_free(raw_curved_result.path)
    fallback_path, smoothing_stats = (
        smoothing_blocked_planner.smooth_path_with_fallback(
            raw_curved_result.path
        )
    )
    assert fallback_path == raw_curved_result.path
    assert smoothing_stats.attempted
    assert not smoothing_stats.accepted
    assert "smoothed collision" in smoothing_stats.status
    assert smoothing_stats.candidate is not None
    assert smoothing_stats.raw == smoothing_stats.final

    # 현재 pose와 goal 사이의 수직 벽은 모든 직접 analytic 후보를 막는다.
    # Hybrid A*가 먼저 벽을 우회한 뒤 새 pose에서 안전한 연결을 찾아야 한다.
    blocked_grid = free_grid.copy()
    blocked_grid[25:76, 60] = 100
    fallback_planner = make_planner(blocked_grid)
    start_state = HybridState(30.0, 60.0, 0.0, 1, 0.0)
    assert fallback_planner.try_analytic_expansion(start_state, direct_goal) is None

    fallback_result = fallback_planner.plan(
        (start_state.x_cm, start_state.y_cm, start_state.yaw_rad),
        direct_goal,
    )
    assert fallback_result.success, fallback_result.message
    assert fallback_result.expanded_nodes > 0
    assert "Reeds-Shepp" in fallback_result.message
    assert_continuous_safe_path(fallback_planner, fallback_result.path, direct_goal)

    print("Hybrid A* analytic expansion regression passed")
    print(f"Direct connection poses: {len(direct_result.path)}")
    print(f"Direct expanded nodes: {direct_result.expanded_nodes}")
    print(f"Smoothed curve poses: {len(curved_result.path)}")
    print(
        "Smoothed curve maximum steer: "
        f"{max(abs(math.degrees(state.steer_rad)) for state in curved_result.path):.3f} deg"
    )
    print(f"Fallback connection poses: {len(fallback_result.path)}")
    print(f"Fallback expanded nodes: {fallback_result.expanded_nodes}")
    print(f"Fallback total cost: {fallback_result.total_cost:.3f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
