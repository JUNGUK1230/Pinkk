"""Regression checks for localization-pose to fixed-route selection."""

from pathlib import Path
import sys

import yaml


SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parent
sys.path.append(str(PROJECT_ROOT / "src"))

from fixed_route_selector import FixedRouteSelector  # noqa: E402


def _pose(values: list[float]) -> tuple[float, float, float]:
    return float(values[0]), float(values[1]), float(values[2])


def main() -> int:
    config_path = PROJECT_ROOT / "config/fixed_mission_routes.yaml"
    with config_path.open(encoding="utf-8") as file:
        config = yaml.safe_load(file)
    selector = FixedRouteSelector(config_path, PROJECT_ROOT / "output")

    cases = (
        ("START", "C2"),
        ("P8", "C1"),
        ("C2", "P3"),
        ("P4", "EXIT"),
    )
    for expected_source, target in cases:
        endpoint = config["endpoints"][expected_source]
        current_pose = _pose(endpoint.get("goal", endpoint["staging"]))
        selection = selector.select(current_pose, target)
        configured_points = selector._load_route(expected_source, target)
        assert selection.source == expected_source
        assert selection.detected_location == expected_source
        assert selection.join_distance_cm < 1e-6
        assert selection.points == tuple(configured_points)
        assert selection.points[-1]
        print(
            f"{expected_source} -> {target}: "
            f"join={selection.join_index}, remaining={len(selection.points)}"
        )

    # START 검출 오차가 있어도 live pose를 CSV 앞에 삽입하지 않는다. 원본
    # 첫 구간의 방향을 유지해 시작 직후 가짜 우회전을 만들지 않아야 한다.
    start = _pose(config["endpoints"]["START"]["staging"])
    noisy_start = (start[0] - 2.8, start[1] + 0.4, start[2])
    noisy_selection = selector.select(noisy_start, "C2")
    configured_start = selector._load_route("START", "C2")
    assert noisy_selection.join_distance_cm > 2.0
    assert noisy_selection.points == tuple(configured_start)
    assert noisy_selection.points[0].x_cm != noisy_start[0]

    # Planning is intentionally rejected while the vehicle is between endpoints.
    base = selector._load_route("START", "C2")
    current = base[180]
    try:
        selector.select(
            (current.x_cm + 0.5, current.y_cm - 0.3, current.yaw_rad), "C2"
        )
    except ValueError as error:
        assert "not at START" in str(error)
    else:
        raise AssertionError("transit pose must not start a new fixed route")
    print("TRANSIT planning rejection passed")
    print("Fixed route selector regression passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
