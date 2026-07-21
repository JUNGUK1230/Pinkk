"""Reeds-Shepp 경로 생성기의 독립 회귀 테스트."""

import math
from pathlib import Path
import sys

SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parent
sys.path.append(str(PROJECT_ROOT / "src"))

from reeds_shepp import ReedsSheppPath, ReedsSheppPlanner  # noqa: E402


TURNING_RADIUS_CM = 8.0 / math.tan(math.radians(30.0))
OUTPUT_STEP_CM = 0.5


def angle_difference(first: float, second: float) -> float:
    """두 각도의 최소 signed 차이를 반환한다."""
    return (first - second + math.pi) % (2.0 * math.pi) - math.pi


def assert_valid_path(
    path: ReedsSheppPath | None,
    goal: tuple[float, float, float],
) -> ReedsSheppPath:
    """끝 pose, 샘플 간격 및 segment 정보를 공통 검증한다."""
    assert path is not None
    assert path.poses

    endpoint = path.poses[-1]
    assert math.hypot(endpoint.x_cm - goal[0], endpoint.y_cm - goal[1]) < 1e-8
    assert abs(angle_difference(endpoint.yaw_rad, goal[2])) < 1e-8
    assert math.isclose(
        path.total_length_cm,
        sum(abs(segment.length_cm) for segment in path.segments),
        abs_tol=1e-9,
    )
    assert all(segment.mode in {"L", "R", "S"} for segment in path.segments)
    assert all(pose.direction in {-1, 1} for pose in path.poses)

    sample_gaps = [
        math.hypot(second.x_cm - first.x_cm, second.y_cm - first.y_cm)
        for first, second in zip(path.poses, path.poses[1:])
    ]
    assert not sample_gaps or max(sample_gaps) <= OUTPUT_STEP_CM + 1e-6

    directions = [segment.direction for segment in path.segments]
    direction_changes = sum(
        first != second for first, second in zip(directions, directions[1:])
    )
    assert direction_changes <= 2
    return path


def main() -> int:
    """직진, 후진, 회전, 좌표 이동 대칭성을 검증한다."""
    planner = ReedsSheppPlanner(TURNING_RADIUS_CM, OUTPUT_STEP_CM)
    cases = (
        ((0.0, 0.0, 0.0), (40.0, 0.0, 0.0)),
        ((0.0, 0.0, 0.0), (-20.0, 0.0, 0.0)),
        ((0.0, 0.0, 0.0), (20.0, 20.0, math.pi / 2.0)),
        ((0.0, 0.0, 0.0), (0.0, 0.0, math.pi)),
        (
            (10.0, -5.0, math.radians(30.0)),
            (35.0, 12.0, math.radians(-45.0)),
        ),
    )

    paths = [
        assert_valid_path(planner.plan(start, goal), goal) for start, goal in cases
    ]

    # 뒤쪽의 동일 헤딩 목표는 직선 후진 하나로 도달해야 한다.
    reverse_path = paths[1]
    assert len(reverse_path.segments) == 1
    assert reverse_path.segments[0].mode == "S"
    assert reverse_path.segments[0].direction == -1

    # 제자리 반대 헤딩은 전진과 후진을 함께 사용하는 cusp를 포함한다.
    turn_in_place_path = paths[3]
    assert {segment.direction for segment in turn_in_place_path.segments} == {-1, 1}

    same_pose = planner.plan((3.0, 4.0, 0.2), (3.0, 4.0, 0.2))
    assert same_pose is not None
    assert same_pose.total_length_cm == 0.0
    assert len(same_pose.poses) == 1

    print("Reeds-Shepp regression passed")
    print(f"Turning radius: {TURNING_RADIUS_CM:.3f} cm")
    print(f"Output spacing: {OUTPUT_STEP_CM:.1f} cm")
    for index, path in enumerate(paths, start=1):
        modes = "-".join(segment.mode for segment in path.segments)
        print(
            f"Case {index}: {modes}, length={path.total_length_cm:.3f} cm, "
            f"poses={len(path.poses)}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
