"""Build and validate the configured one-way routes between mission endpoints."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
import sys

import numpy as np
import yaml


SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parent
sys.path.extend((str(SCRIPT_DIR), str(PROJECT_ROOT / "src")))

from plan_from_live_vision import load_planner_stack  # noqa: E402
from reeds_shepp import ReedsSheppPlanner  # noqa: E402


Pose = tuple[float, float, float]
ROAD_TURNING_RADIUS_MARGIN_CM = 2.0
ENDPOINT_ATTACHMENT_SEARCH_CM = 25.0


def _load_config() -> dict:
    with (PROJECT_ROOT / "config/fixed_mission_routes.yaml").open(encoding="utf-8") as file:
        return yaml.safe_load(file)


def _pose(values: list[float]) -> Pose:
    return float(values[0]), float(values[1]), float(values[2])


def _direction_only_path(
    planner,
    start: Pose,
    goal: Pose,
    direction: int,
    turning_radius_cm: float | None = None,
):
    reeds_shepp = ReedsSheppPlanner(
        (
            float(turning_radius_cm)
            if turning_radius_cm is not None
            else planner.minimum_turning_radius_cm
        ),
        planner.path_output_step_cm,
    )
    for candidate in reeds_shepp.iter_candidates(start, goal):
        if candidate.poses and all(pose.direction == direction for pose in candidate.poses):
            if planner.is_path_collision_free(candidate.poses):
                return candidate
    raise RuntimeError(
        f"no collision-free {'forward' if direction > 0 else 'reverse'} path"
    )


def _resample_segment(first: Pose, second: Pose, step_cm: float = 0.5) -> list[dict]:
    dx, dy = second[0] - first[0], second[1] - first[1]
    length = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    distances = list(np.arange(0.0, length, step_cm)) + [length]
    return [
        {
            "x_cm": first[0] + dx * distance / length,
            "y_cm": first[1] + dy * distance / length,
            "yaw_rad": yaw,
            "direction": 1,
        }
        for distance in distances
    ]


def _append_line(
    rows: list[dict],
    first: np.ndarray,
    second: np.ndarray,
    step_cm: float,
) -> None:
    delta = second - first
    length = float(np.linalg.norm(delta))
    if length <= 1e-9:
        return
    yaw = math.atan2(float(delta[1]), float(delta[0]))
    sample_count = max(1, math.ceil(length / step_cm))
    first_index = 0 if not rows else 1
    for index in range(first_index, sample_count + 1):
        point = first + delta * index / sample_count
        rows.append(
            {
                "x_cm": float(point[0]),
                "y_cm": float(point[1]),
                "yaw_rad": yaw,
                "direction": 1,
            }
        )


def _rounded_centerline(
    waypoints: list[Pose],
    turning_radius_cm: float,
    step_cm: float,
) -> list[dict]:
    """직선 waypoint chain의 모서리를 접선 원호로 교체한다."""
    points = [np.asarray(pose[:2], dtype=np.float64) for pose in waypoints]
    if len(points) < 2:
        raise ValueError("road centerline needs at least two waypoints")
    corners: list[dict[str, object]] = []
    for index in range(1, len(points) - 1):
        incoming_delta = points[index] - points[index - 1]
        outgoing_delta = points[index + 1] - points[index]
        incoming_length = float(np.linalg.norm(incoming_delta))
        outgoing_length = float(np.linalg.norm(outgoing_delta))
        if incoming_length <= 1e-9 or outgoing_length <= 1e-9:
            raise ValueError(f"duplicate road waypoint at index {index}")
        incoming = incoming_delta / incoming_length
        outgoing = outgoing_delta / outgoing_length
        turn = math.atan2(
            float(np.cross(incoming, outgoing)),
            float(np.dot(incoming, outgoing)),
        )
        tangent_length = turning_radius_cm * math.tan(abs(turn) / 2.0)
        corners.append(
            {
                "entry": points[index] - incoming * tangent_length,
                "exit": points[index] + outgoing * tangent_length,
                "incoming": incoming,
                "turn": turn,
                "tangent_length": tangent_length,
            }
        )

    for segment_index in range(len(points) - 1):
        previous_tangent = (
            float(corners[segment_index - 1]["tangent_length"])
            if segment_index > 0
            else 0.0
        )
        next_tangent = (
            float(corners[segment_index]["tangent_length"])
            if segment_index < len(corners)
            else 0.0
        )
        segment_length = float(
            np.linalg.norm(points[segment_index + 1] - points[segment_index])
        )
        if previous_tangent + next_tangent > segment_length + 1e-6:
            raise RuntimeError(
                "road waypoints are too close for the configured turning radius: "
                f"segment={segment_index}, required="
                f"{previous_tangent + next_tangent:.2f} cm, "
                f"available={segment_length:.2f} cm"
            )

    rows: list[dict] = []
    current = points[0]
    for corner in corners:
        entry = np.asarray(corner["entry"], dtype=np.float64)
        exit_point = np.asarray(corner["exit"], dtype=np.float64)
        incoming = np.asarray(corner["incoming"], dtype=np.float64)
        turn = float(corner["turn"])
        _append_line(rows, current, entry, step_cm)
        turn_sign = 1.0 if turn > 0.0 else -1.0
        left_normal = np.asarray([-incoming[1], incoming[0]])
        center = entry + left_normal * turn_sign * turning_radius_cm
        start_angle = math.atan2(
            float(entry[1] - center[1]),
            float(entry[0] - center[0]),
        )
        sample_count = max(
            1,
            math.ceil(turning_radius_cm * abs(turn) / step_cm),
        )
        for sample_index in range(1, sample_count + 1):
            angle = start_angle + turn * sample_index / sample_count
            point = center + turning_radius_cm * np.asarray(
                [math.cos(angle), math.sin(angle)]
            )
            tangent_yaw = angle + turn_sign * math.pi / 2.0
            rows.append(
                {
                    "x_cm": float(point[0]),
                    "y_cm": float(point[1]),
                    "yaw_rad": (tangent_yaw + math.pi) % (2.0 * math.pi) - math.pi,
                    "direction": 1,
                }
            )
        current = exit_point
    _append_line(rows, current, points[-1], step_cm)
    return rows


def _build_road_network(config: dict, planner) -> dict[str, object]:
    """하나의 곡률 제한 도로와 각 주차면의 접선 연결 pose를 만든다."""
    route_order = list(config["route_order"])
    turning_radius_cm = (
        float(planner.minimum_turning_radius_cm)
        + ROAD_TURNING_RADIUS_MARGIN_CM
    )
    road_waypoints = [
        _pose(config["road_backbone"][name])
        for name in route_order
    ]
    # START/EXIT는 검출 endpoint와 CSV 끝점이 수치상 완전히 같아야 한다.
    road_waypoints[0] = _pose(config["endpoints"][route_order[0]]["staging"])
    road_waypoints[-1] = _pose(config["endpoints"][route_order[-1]]["staging"])
    centerline = _rounded_centerline(
        road_waypoints,
        turning_radius_cm,
        float(planner.path_output_step_cm),
    )
    for row in centerline:
        if planner.is_pose_collision(row["x_cm"], row["y_cm"], row["yaw_rad"]):
            raise RuntimeError("rounded road centerline collides with the map")

    attachment_indices: dict[str, int] = {}
    exit_connectors: dict[str, object] = {}
    entry_connectors: dict[str, object] = {}
    minimum_index = 0
    for name in route_order:
        endpoint = config["endpoints"][name]
        nominal = _pose(endpoint["staging"])
        candidates = sorted(
            range(minimum_index, len(centerline)),
            key=lambda index: (
                (float(centerline[index]["x_cm"]) - nominal[0]) ** 2
                + (float(centerline[index]["y_cm"]) - nominal[1]) ** 2
            ),
        )
        selected_index: int | None = None
        selected_exit = None
        selected_entry = None
        for index in candidates:
            row = centerline[index]
            distance = math.hypot(
                float(row["x_cm"]) - nominal[0],
                float(row["y_cm"]) - nominal[1],
            )
            if distance > ENDPOINT_ATTACHMENT_SEARCH_CM:
                break
            if "goal" not in endpoint:
                selected_index = index
                break
            attachment = (
                float(row["x_cm"]),
                float(row["y_cm"]),
                float(row["yaw_rad"]),
            )
            goal = _pose(endpoint["goal"])
            try:
                selected_exit = _direction_only_path(
                    planner,
                    goal,
                    attachment,
                    1,
                    turning_radius_cm,
                )
                selected_entry = _direction_only_path(
                    planner,
                    attachment,
                    goal,
                    -1,
                    turning_radius_cm,
                )
            except RuntimeError:
                continue
            selected_index = index
            break
        if selected_index is None:
            raise RuntimeError(
                f"no curvature-safe road attachment found for endpoint {name}"
            )
        attachment_indices[name] = selected_index
        if selected_exit is not None and selected_entry is not None:
            exit_connectors[name] = selected_exit
            entry_connectors[name] = selected_entry
        minimum_index = selected_index + 1

    return {
        "centerline": centerline,
        "attachment_indices": attachment_indices,
        "exit_connectors": exit_connectors,
        "entry_connectors": entry_connectors,
        "turning_radius_cm": turning_radius_cm,
    }


def _validate_route(
    rows: list[dict],
    planner,
    turning_radius_cm: float,
    source: str,
    target: str,
) -> None:
    maximum_curvature_1pcm = 1.0 / turning_radius_cm
    for index, row in enumerate(rows):
        if planner.is_pose_collision(row["x_cm"], row["y_cm"], row["yaw_rad"]):
            raise RuntimeError(f"route collision: {source} -> {target}, index={index}")
    for index, (first, second) in enumerate(zip(rows, rows[1:]), start=1):
        if int(first["direction"]) != int(second["direction"]):
            continue
        spacing_cm = math.hypot(
            float(second["x_cm"]) - float(first["x_cm"]),
            float(second["y_cm"]) - float(first["y_cm"]),
        )
        if spacing_cm <= 1e-9:
            continue
        yaw_change = abs(
            (
                float(second["yaw_rad"])
                - float(first["yaw_rad"])
                + math.pi
            )
            % (2.0 * math.pi)
            - math.pi
        )
        curvature_1pcm = yaw_change / spacing_cm
        if curvature_1pcm > maximum_curvature_1pcm * 1.02:
            raise RuntimeError(
                f"route curvature exceeds 1/{turning_radius_cm:.1f} cm: "
                f"{source} -> {target}, index={index}, "
                f"curvature={curvature_1pcm:.4f} 1/cm"
            )


def build_route(
    config: dict,
    planner,
    source: str,
    target: str,
    road_network: dict[str, object] | None = None,
) -> list[dict]:
    endpoints = config["endpoints"]
    allowed = config["allowed_transitions"]
    if source not in allowed or target not in allowed[source]:
        raise ValueError(f"unsupported fixed mission transition: {source} -> {target}")
    network = road_network or _build_road_network(config, planner)
    centerline = network["centerline"]
    attachment_indices = network["attachment_indices"]
    exit_connectors = network["exit_connectors"]
    entry_connectors = network["entry_connectors"]
    source_index = int(attachment_indices[source])
    target_index = int(attachment_indices[target])
    rows: list[dict] = []

    # START에서 출발하는 경로는 목표 staging에서 공통 도로를 끝낸다.
    # 이렇게 해야 C1/C2를 지나 다음 구간으로 향하는 원호가 목표 주차 진입
    # pose를 잘라내지 않으며, 기존의 검증된 후진 주차 연결을 그대로 쓸 수 있다.
    if source == config["route_order"][0]:
        order = config["route_order"]
        source_order_index = order.index(source)
        target_order_index = order.index(target)
        corridor_names = order[source_order_index : target_order_index + 1]
        road_waypoints = [_pose(endpoints[source]["staging"])]
        road_waypoints.extend(
            _pose(config["road_backbone"][name])
            for name in corridor_names[1:-1]
        )
        road_waypoints.append(_pose(endpoints[target]["staging"]))
        rows = _rounded_centerline(
            road_waypoints,
            float(network["turning_radius_cm"]),
            float(planner.path_output_step_cm),
        )
        if "goal" in endpoints[target]:
            reverse_entry = _direction_only_path(
                planner,
                _pose(endpoints[target]["staging"]),
                _pose(endpoints[target]["goal"]),
                -1,
            )
            rows.extend(
                {
                    "x_cm": pose.x_cm,
                    "y_cm": pose.y_cm,
                    "yaw_rad": pose.yaw_rad,
                    "direction": -1,
                }
                for pose in reverse_entry.poses[1:]
            )
        _validate_route(
            rows,
            planner,
            float(planner.minimum_turning_radius_cm),
            source,
            target,
        )
        return rows

    if "goal" in endpoints[source]:
        forward_exit = exit_connectors[source]
        rows.extend(
            {
                "x_cm": pose.x_cm,
                "y_cm": pose.y_cm,
                "yaw_rad": pose.yaw_rad,
                "direction": 1,
            }
            for pose in forward_exit.poses
        )
    else:
        rows.append(dict(centerline[source_index]))

    rows.extend(
        dict(row)
        for row in centerline[source_index + 1 : target_index + 1]
    )

    if "goal" in endpoints[target]:
        reverse_entry = entry_connectors[target]
        rows.extend(
            {
                "x_cm": pose.x_cm,
                "y_cm": pose.y_cm,
                "yaw_rad": pose.yaw_rad,
                "direction": -1,
            }
            for pose in reverse_entry.poses[1:]
        )

    _validate_route(
        rows,
        planner,
        float(network["turning_radius_cm"]),
        source,
        target,
    )
    return rows


def save_route(rows: list[dict], source: str, target: str) -> Path:
    output_dir = PROJECT_ROOT / "output"
    output_dir.mkdir(exist_ok=True)
    stem = f"fixed_route_{source.lower()}_to_{target.lower()}"
    csv_path = output_dir / f"{stem}.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=["index", "x_cm", "y_cm", "yaw_rad", "direction"])
        writer.writeheader()
        writer.writerows({"index": index, **row} for index, row in enumerate(rows))
    return csv_path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", default="START")
    parser.add_argument("--target", default="C2")
    parser.add_argument("--check-all", action="store_true")
    parser.add_argument(
        "--generate-all",
        action="store_true",
        help="save CSV files for every allowed transition",
    )
    args = parser.parse_args()
    config = _load_config()
    _, planner, _, _, _ = load_planner_stack()
    road_network = _build_road_network(config, planner)
    source, target = args.source.upper(), args.target.upper()

    if args.check_all or args.generate_all:
        count = 0
        manifest_rows: list[dict[str, object]] = []
        for candidate_source, targets in config["allowed_transitions"].items():
            for candidate_target in targets:
                candidate_rows = build_route(
                    config,
                    planner,
                    candidate_source,
                    candidate_target,
                    road_network,
                )
                if args.generate_all:
                    candidate_csv = save_route(
                        candidate_rows, candidate_source, candidate_target
                    )
                    manifest_rows.append(
                        {
                            "source": candidate_source,
                            "target": candidate_target,
                            "path_points": len(candidate_rows),
                            "csv_file": candidate_csv.name,
                        }
                    )
                count += 1
        if args.generate_all:
            manifest_path = PROJECT_ROOT / "output/fixed_route_manifest.csv"
            with manifest_path.open("w", newline="", encoding="utf-8") as file:
                writer = csv.DictWriter(
                    file,
                    fieldnames=["source", "target", "path_points", "csv_file"],
                )
                writer.writeheader()
                writer.writerows(manifest_rows)
            print(f"saved manifest: {manifest_path}")
        action = "generated" if args.generate_all else "validated"
        print(f"{action} fixed mission transitions: {count}")
        return 0

    rows = build_route(config, planner, source, target, road_network)
    csv_path = save_route(rows, source, target)
    print(f"fixed route: {source} -> {target}")
    print(f"path points: {len(rows)}")
    print(f"saved csv: {csv_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
