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


def _load_config() -> dict:
    with (PROJECT_ROOT / "config/fixed_mission_routes.yaml").open(encoding="utf-8") as file:
        return yaml.safe_load(file)


def _pose(values: list[float]) -> Pose:
    return float(values[0]), float(values[1]), float(values[2])


def _direction_only_path(planner, start: Pose, goal: Pose, direction: int):
    reeds_shepp = ReedsSheppPlanner(
        planner.minimum_turning_radius_cm, planner.path_output_step_cm
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


def build_route(config: dict, planner, source: str, target: str) -> list[dict]:
    endpoints = config["endpoints"]
    allowed = config["allowed_transitions"]
    if source not in allowed or target not in allowed[source]:
        raise ValueError(f"unsupported fixed mission transition: {source} -> {target}")
    order = config["route_order"]
    backbone = config["road_backbone"]
    source_index, target_index = order.index(source), order.index(target)
    rows: list[dict] = []

    if "goal" in endpoints[source]:
        forward_exit = _direction_only_path(
            planner,
            _pose(endpoints[source]["goal"]),
            _pose(endpoints[source]["staging"]),
            1,
        )
        rows.extend(
            {
                "x_cm": pose.x_cm,
                "y_cm": pose.y_cm,
                "yaw_rad": pose.yaw_rad,
                "direction": 1,
            }
            for pose in forward_exit.poses
        )

    corridor_names = order[source_index : target_index + 1]
    corridor = [_pose(endpoints[source]["staging"])]
    corridor.extend(_pose(backbone[name]) for name in corridor_names[1:-1])
    corridor.append(_pose(endpoints[target]["staging"]))
    for first, second in zip(corridor, corridor[1:]):
        segment = _resample_segment(first, second)
        if rows and segment:
            segment = segment[1:]
        rows.extend(segment)

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

    for row in rows:
        if planner.is_pose_collision(row["x_cm"], row["y_cm"], row["yaw_rad"]):
            raise RuntimeError(f"route collision: {source} -> {target}")
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
    source, target = args.source.upper(), args.target.upper()

    if args.check_all or args.generate_all:
        count = 0
        manifest_rows: list[dict[str, object]] = []
        for candidate_source, targets in config["allowed_transitions"].items():
            for candidate_target in targets:
                candidate_rows = build_route(
                    config, planner, candidate_source, candidate_target
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

    rows = build_route(config, planner, source, target)
    csv_path = save_route(rows, source, target)
    print(f"fixed route: {source} -> {target}")
    print(f"path points: {len(rows)}")
    print(f"saved csv: {csv_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
