"""Reeds-Shepp 경로와 Hybrid A* footprint 충돌 검사의 회귀 테스트."""

import math
from pathlib import Path
import sys

import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parent
sys.path.append(str(PROJECT_ROOT / "src"))

from hybrid_astar_planner import HybridAStarPlanner  # noqa: E402
from reeds_shepp import ReedsSheppPlanner  # noqa: E402


def make_footprint_checker(grid: np.ndarray) -> HybridAStarPlanner:
    """실차 제원과 같은 footprint 검사기를 생성한다."""
    return HybridAStarPlanner(
        grid,
        resolution_cm=1.0,
        wheelbase_cm=8.0,
        vehicle_length_cm=12.0,
        vehicle_width_cm=8.0,
        rear_overhang_cm=2.0,
        path_output_step_cm=0.5,
        timeout_sec=1.0,
    )


def main() -> int:
    """중간 장애물과 지도 외부 footprint를 빠짐없이 검출한다."""
    turning_radius_cm = 8.0 / math.tan(math.radians(30.0))
    reeds_shepp = ReedsSheppPlanner(turning_radius_cm, step_size_cm=0.5)
    path = reeds_shepp.plan((30.0, 50.0, 0.0), (70.0, 50.0, 0.0))
    assert path is not None
    assert len(path.poses) == 81

    free_grid = np.zeros((100, 100), dtype=np.uint8)
    free_checker = make_footprint_checker(free_grid)
    assert free_checker.first_path_collision_index(path.poses) is None
    assert free_checker.is_path_collision_free(path.poses)

    # 시작/도착 footprint에는 닿지 않고 경로 중간에서만 닿는 장애물이다.
    blocked_grid = free_grid.copy()
    blocked_grid[50, 50] = 100
    blocked_checker = make_footprint_checker(blocked_grid)
    assert not blocked_checker.is_pose_collision(
        path.poses[0].x_cm,
        path.poses[0].y_cm,
        path.poses[0].yaw_rad,
    )
    assert not blocked_checker.is_pose_collision(
        path.poses[-1].x_cm,
        path.poses[-1].y_cm,
        path.poses[-1].yaw_rad,
    )

    collision_index = blocked_checker.first_path_collision_index(path.poses)
    assert collision_index is not None
    assert 0 < collision_index < len(path.poses) - 1
    assert not blocked_checker.is_path_collision_free(path.poses)

    # 기준점은 지도 안이지만 차량 뒤쪽이 지도 밖으로 나간 첫 pose도 검출한다.
    boundary_path = reeds_shepp.plan((1.0, 20.0, 0.0), (20.0, 20.0, 0.0))
    assert boundary_path is not None
    assert free_checker.first_path_collision_index(boundary_path.poses) == 0

    collision_pose = path.poses[collision_index]
    print("Reeds-Shepp footprint collision regression passed")
    print(f"Path poses checked: {len(path.poses)}")
    print(f"First obstacle collision index: {collision_index}")
    print(
        "First obstacle collision pose: "
        f"({collision_pose.x_cm:.1f}, {collision_pose.y_cm:.1f}, "
        f"{math.degrees(collision_pose.yaw_rad):.1f} deg)"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
