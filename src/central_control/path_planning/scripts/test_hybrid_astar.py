"""Regression test for Hybrid A* kinematics and vehicle-footprint collision."""

import math
from pathlib import Path
import sys

import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parent
sys.path.append(str(PROJECT_ROOT / "src"))

from hybrid_astar_planner import HybridAStarPlanner, HybridState  # noqa: E402


def make_planner(grid: np.ndarray) -> HybridAStarPlanner:
    """Build the current 12 x 10 cm vehicle on a small synthetic map."""
    return HybridAStarPlanner(
        grid,
        resolution_cm=1.0,
        wheelbase_cm=8.0,
        vehicle_length_cm=12.0,
        vehicle_width_cm=10.0,
        rear_overhang_cm=2.0,
        motion_step_cm=3.0,
        path_output_step_cm=0.5,
        steer_set_deg=(-30.0, -20.0, -10.0, 0.0, 10.0, 20.0, 30.0),
        max_steer_change_deg=10.0,
        timeout_sec=2.0,
    )


def main() -> int:
    """Verify heading-dependent collision and a collision-free planned path."""
    free_grid = np.zeros((100, 100), dtype=np.uint8)
    free_planner = make_planner(free_grid)

    # From zero steering, only -10/0/+10-degree primitives may be expanded.
    initial_state = HybridState(50.0, 50.0, 0.0, 1, 0.0)
    initial_neighbor_steers = {
        round(math.degrees(neighbor.steer_rad))
        for neighbor, _ in free_planner._neighbors(initial_state)
    }
    assert initial_neighbor_steers == {-10, 0, 10}

    # At yaw=0, the rectangle nose covers the obstacle 10 cm ahead.
    front_obstacle_grid = free_grid.copy()
    front_obstacle_grid[50, 60] = 100
    front_obstacle_planner = make_planner(front_obstacle_grid)
    assert front_obstacle_planner.is_pose_collision(50.0, 50.0, 0.0)
    assert not front_obstacle_planner.is_pose_collision(
        50.0, 50.0, math.pi / 2.0
    )

    # A 90-degree heading must rotate the rectangle and hit y=60 instead.
    rotated_obstacle_grid = free_grid.copy()
    rotated_obstacle_grid[60, 50] = 100
    rotated_obstacle_planner = make_planner(rotated_obstacle_grid)
    assert rotated_obstacle_planner.is_pose_collision(
        50.0, 50.0, math.pi / 2.0
    )

    # The pose is invalid when any part of the vehicle leaves the map.
    assert free_planner.is_pose_collision(1.0, 50.0, 0.0)

    # Analytic expansion 없이 일반 goal 도달 분기도 smoothing을 적용해야 한다.
    free_planner.analytic_expansion_enabled = False
    result = free_planner.plan((20.0, 20.0, 0.0), (60.0, 20.0, 0.0))
    assert result.success, result.message
    assert result.smoothing_stats is not None
    assert result.smoothing_stats.attempted
    assert all(
        not free_planner.is_pose_collision(state.x_cm, state.y_cm, state.yaw_rad)
        for state in result.path
    )
    segment_lengths = [
        math.hypot(second.x_cm - first.x_cm, second.y_cm - first.y_cm)
        for first, second in zip(result.path, result.path[1:])
    ]
    assert segment_lengths
    assert max(segment_lengths) <= free_planner.path_output_step_cm + 1e-6

    # 시간 제한과 별도로 상태공간 폭증을 확장 노드 상한에서 차단해야 한다.
    free_planner.max_expanded_nodes = 1
    capped_result = free_planner.plan(
        (20.0, 20.0, 0.0),
        (80.0, 80.0, math.pi / 2.0),
    )
    assert not capped_result.success
    assert capped_result.expanded_nodes == 1
    assert "maximum expanded nodes reached" in capped_result.message

    print("Hybrid A* footprint regression passed")
    print(f"Path poses: {len(result.path)}")
    print(f"Expanded nodes: {result.expanded_nodes}")
    print(f"Total cost: {result.total_cost:.3f}")
    print(f"Maximum output spacing: {max(segment_lengths):.3f} cm")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
