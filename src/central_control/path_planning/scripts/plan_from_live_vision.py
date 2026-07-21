"""최신 상단 카메라 pose와 빈 주차면 좌표로 Hybrid A* 경로를 생성한다.

상단 카메라 localization을 먼저 실행한다. 이 스크립트는 planning-ready
scene이 나타날 때까지 잠시 기다린 뒤 한 번만 계획하고 종료한다. 제어기나
ROS 2로 전송하지 않으며, validator를 통과한 trajectory만 파일로 저장한다.
"""

import argparse
import csv
import json
import math
from pathlib import Path
import sys
import time
from typing import Sequence

import cv2
import numpy as np
import yaml


SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parent
CENTRAL_ROOT = PROJECT_ROOT.parent
sys.path.append(str(PROJECT_ROOT / "src"))
sys.path.append(str(SCRIPT_DIR))

from hybrid_astar_planner import (  # noqa: E402
    HybridAStarPlanner,
    HybridAStarResult,
)
from occupancy_grid import OccupancyGridMap  # noqa: E402
from test_astar import resolve_map_image  # noqa: E402
from trajectory_profile import (  # noqa: E402
    TrajectoryPoint,
    build_trajectory_profile,
)
from trajectory_validator import (  # noqa: E402
    TrajectoryValidationLimits,
    TrajectoryValidationResult,
    validate_trajectory,
)
from vision_scene_input import (  # noqa: E402
    VisionPlanningRequest,
    VisionSceneUnavailable,
    load_vision_planning_request,
)


Pose = tuple[float, float, float]
POSE_ADJUSTMENT_RADIUS_CM = 30.0
DEFAULT_SCENE_PATH = PROJECT_ROOT / "output/live_vision_scene.json"
DEFAULT_REGISTRATION_PATH = (
    CENTRAL_ROOT
    / "camera_tools/first_map/camera_to_lidar_rigid_registration.npz"
)
OUTPUT_FILES = (
    "live_hybrid_path_world_cm.csv",
    "live_hybrid_path_camera_bev.csv",
    "live_hybrid_path_world_cm.json",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Plan once from a fresh overhead-camera scene."
    )
    parser.add_argument("--scene", type=Path, default=DEFAULT_SCENE_PATH)
    parser.add_argument("--max-age-sec", type=float, default=0.5)
    parser.add_argument(
        "--wait-sec",
        type=float,
        default=10.0,
        help="Maximum time to wait for a fresh planning-ready detection.",
    )
    return parser.parse_args()


def load_planner_stack() -> tuple[
    OccupancyGridMap,
    HybridAStarPlanner,
    dict[str, float],
    TrajectoryValidationLimits,
]:
    """클릭 테스트와 동일한 차량·지도·planner 안전 설정을 구성한다."""
    first_map_dir = CENTRAL_ROOT / "camera_tools/first_map"
    yaml_path = first_map_dir / "my_test_map0710.yaml"
    pgm_path = first_map_dir / "my_test_map0710.pgm"
    with (PROJECT_ROOT / "config/vehicle_config.yaml").open(
        encoding="utf-8"
    ) as file:
        vehicle = (yaml.safe_load(file) or {})["vehicle"]
    with (PROJECT_ROOT / "config/planner_config.yaml").open(
        encoding="utf-8"
    ) as file:
        config = yaml.safe_load(file) or {}

    grid_map = OccupancyGridMap(
        str(resolve_map_image(pgm_path, yaml_path)),
        str(yaml_path),
        block_outside_area=True,
    )
    hybrid = config["hybrid_astar"]
    cost = config["cost"]
    profile_config = {
        key: float(value)
        for key, value in config["trajectory_profile"].items()
    }
    validation_config = config["trajectory_validation"]
    planning_grid = grid_map.inflate_obstacles(
        float(hybrid["footprint_safety_margin_cm"]),
        grid_map.resolution_cm,
    )
    planner = HybridAStarPlanner(
        planning_grid,
        resolution_cm=grid_map.resolution_cm,
        wheelbase_cm=float(vehicle["wheelbase_cm"]),
        vehicle_length_cm=float(vehicle["length_cm"]),
        vehicle_width_cm=float(vehicle["width_cm"]),
        rear_overhang_cm=float(vehicle["rear_overhang_cm"]),
        minimum_turning_radius_cm=float(vehicle["min_turning_radius_cm"]),
        motion_step_cm=float(hybrid["motion_step_cm"]),
        path_output_step_cm=float(hybrid["path_output_step_cm"]),
        yaw_resolution_deg=float(hybrid["yaw_resolution_deg"]),
        steer_set_deg=tuple(float(value) for value in hybrid["steer_set_deg"]),
        max_steer_change_deg=float(hybrid["max_steer_change_deg"]),
        allow_reverse=bool(hybrid["allow_reverse"]),
        timeout_sec=float(hybrid["timeout_sec"]),
        goal_tolerance_cm=float(hybrid["goal_tolerance_cm"]),
        goal_yaw_tolerance_deg=float(hybrid["goal_yaw_tolerance_deg"]),
        reverse_penalty=float(cost["reverse_penalty"]),
        gear_switch_penalty=float(cost["gear_switch_penalty"]),
        steer_penalty=float(cost["steer_penalty"]),
        steer_change_penalty=float(cost["steer_change_penalty"]),
        analytic_expansion_enabled=bool(hybrid["analytic_expansion_enabled"]),
        analytic_expansion_distance_cm=float(
            hybrid["analytic_expansion_distance_cm"]
        ),
        analytic_turning_radius_margin_cm=float(
            hybrid["analytic_turning_radius_margin_cm"]
        ),
        path_smoothing_enabled=bool(hybrid["path_smoothing_enabled"]),
        smoothing_knot_spacing_cm=float(hybrid["smoothing_knot_spacing_cm"]),
    )
    limits = TrajectoryValidationLimits(
        wheelbase_cm=planner.wheelbase_cm,
        max_spacing_cm=planner.path_output_step_cm,
        max_steer_rad=max(abs(value) for value in planner.steer_set_rad),
        max_steer_change_rad_per_cm=(
            planner.max_steer_change_rad / planner.motion_step_cm
        ),
        steer_change_tolerance_rad=math.radians(
            float(validation_config["steering_rate_tolerance_deg"])
        ),
        max_forward_speed_mps=profile_config["max_forward_speed_mps"],
        max_reverse_speed_mps=profile_config["max_reverse_speed_mps"],
        max_angular_speed_radps=profile_config["max_angular_speed_radps"],
        max_acceleration_mps2=profile_config["max_acceleration_mps2"],
        max_deceleration_mps2=profile_config["max_deceleration_mps2"],
        max_yaw_change_rad=math.radians(
            float(validation_config["max_yaw_change_deg"])
        ),
        max_motion_heading_error_rad=math.radians(
            float(validation_config["max_motion_heading_error_deg"])
        ),
        tolerance=float(validation_config["numeric_tolerance"]),
    )
    return grid_map, planner, profile_config, limits


def wait_for_request(
    scene_path: Path,
    max_age_sec: float,
    wait_sec: float,
    map_size: tuple[int, int],
    resolution_cm: float,
) -> VisionPlanningRequest:
    """차량이 검출돼 fresh planning request가 생길 때까지 제한 시간 대기한다."""
    if wait_sec < 0.0:
        raise ValueError("wait-sec must not be negative")
    deadline = time.monotonic() + wait_sec
    last_error: VisionSceneUnavailable | None = None
    while True:
        try:
            return load_vision_planning_request(
                scene_path,
                max_age_sec=max_age_sec,
                map_size_cells=map_size,
                resolution_cm=resolution_cm,
            )
        except VisionSceneUnavailable as error:
            last_error = error
            if time.monotonic() >= deadline:
                raise VisionSceneUnavailable(
                    f"no fresh planning-ready vehicle within {wait_sec:.1f} sec: "
                    f"{last_error}"
                ) from error
            time.sleep(0.05)


def adjust_pose(
    planner: HybridAStarPlanner,
    pose: Pose,
    label: str,
) -> Pose:
    """차체 footprint가 충돌하면 같은 yaw를 유지하며 가까운 위치로 보정한다."""
    adjusted_xy = planner.find_nearest_valid_pose(
        pose[0],
        pose[1],
        pose[2],
        POSE_ADJUSTMENT_RADIUS_CM,
    )
    adjusted = (adjusted_xy[0], adjusted_xy[1], pose[2])
    if math.hypot(adjusted[0] - pose[0], adjusted[1] - pose[1]) > 1e-6:
        print(f"{label} footprint adjustment: {pose} -> {adjusted}")
    return adjusted


def plan_and_validate(
    planner: HybridAStarPlanner,
    profile_config: dict[str, float],
    limits: TrajectoryValidationLimits,
    start: Pose,
    goal_candidates: Sequence[tuple[str, Pose]],
) -> tuple[
    str,
    Pose,
    Pose,
    HybridAStarResult,
    list[TrajectoryPoint],
    TrajectoryValidationResult,
]:
    """주 goal 실패 시 180도 반대 goal을 시도하고 검증된 첫 경로를 반환한다."""
    adjusted_start = adjust_pose(planner, start, "Start")
    failures: list[str] = []
    for candidate_name, raw_goal in goal_candidates:
        try:
            adjusted_goal = adjust_pose(planner, raw_goal, candidate_name)
            result = planner.plan(adjusted_start, adjusted_goal)
            if not result.success:
                failures.append(f"{candidate_name}: {result.message}")
                continue
            trajectory = build_trajectory_profile(
                result.path,
                wheelbase_cm=planner.wheelbase_cm,
                max_steer_rad=max(
                    abs(value) for value in planner.steer_set_rad
                ),
                **profile_config,
            )
            validation = validate_trajectory(
                trajectory,
                limits,
                collision_checker=planner.is_pose_collision,
            )
            if not validation.valid:
                codes = ", ".join(issue.code for issue in validation.issues)
                failures.append(
                    f"{candidate_name}: {result.message}; "
                    f"validation failed ({codes})"
                )
                continue
            return (
                candidate_name,
                adjusted_start,
                adjusted_goal,
                result,
                trajectory,
                validation,
            )
        except (TypeError, ValueError) as error:
            failures.append(f"{candidate_name}: {error}")
    raise RuntimeError("; ".join(failures))


def trajectory_rows(
    trajectory: Sequence[TrajectoryPoint],
) -> list[dict[str, int | float]]:
    return [
        {
            "index": index,
            "x_cm": point.x_cm,
            "y_cm": point.y_cm,
            "yaw_rad": point.yaw_rad,
            "yaw_deg": math.degrees(point.yaw_rad),
            "direction": point.direction,
            "steer_deg": math.degrees(point.steer_rad),
            "curvature_1pm": point.curvature_1pm,
            "target_speed_mps": point.target_speed_mps,
            "target_angular_z_radps": point.target_angular_z_radps,
            "stop_required": int(point.stop_required),
        }
        for index, point in enumerate(trajectory)
    ]


def save_outputs(
    request: VisionPlanningRequest,
    selected_candidate: str,
    adjusted_start: Pose,
    adjusted_goal: Pose,
    result: HybridAStarResult,
    trajectory: Sequence[TrajectoryPoint],
    validation: TrajectoryValidationResult,
    resolution_cm: float,
    profile_config: dict[str, float],
    output_dir: Path | None = None,
) -> None:
    """검증된 경로만 world·Camera CSV와 metadata JSON으로 원자 저장한다."""
    output_dir = PROJECT_ROOT / "output" if output_dir is None else output_dir
    output_dir.mkdir(parents=True, exist_ok=True)
    world_path = output_dir / OUTPUT_FILES[0]
    camera_path = output_dir / OUTPUT_FILES[1]
    json_path = output_dir / OUTPUT_FILES[2]
    rows = trajectory_rows(trajectory)
    columns = list(rows[0])

    with np.load(DEFAULT_REGISTRATION_PATH) as registration:
        lidar_from_camera = np.asarray(
            registration["affine_matrix"], dtype=np.float64
        )
    camera_from_lidar = cv2.invertAffineTransform(lidar_from_camera)
    lidar_points = np.asarray(
        [
            [point.x_cm / resolution_cm, point.y_cm / resolution_cm, 1.0]
            for point in trajectory
        ],
        dtype=np.float64,
    )
    camera_points = lidar_points @ camera_from_lidar.T

    _atomic_dict_csv(world_path, columns, rows)
    _atomic_camera_csv(camera_path, camera_points)
    payload = {
        "frame": "lidar_map_cm",
        "planner": "hybrid_astar",
        "source": {
            "type": "overhead_camera_yolo",
            "frame_index": request.frame_index,
            "observed_at_unix_sec": request.observed_at_unix_sec,
            "parking_slot": request.slot_name,
            "goal_candidate": selected_candidate,
        },
        "resolution_cm": resolution_cm,
        "requested_start_pose_cm": _pose_dict(request.start_pose_cm),
        "adjusted_start_pose_cm": _pose_dict(adjusted_start),
        "adjusted_goal_pose_cm": _pose_dict(adjusted_goal),
        "goal_connection": result.message,
        "total_cost": result.total_cost,
        "expanded_nodes": result.expanded_nodes,
        "trajectory_profile": profile_config,
        "validation_metrics": validation.metrics.__dict__,
        "smoothing": (
            result.smoothing_stats.to_dict()
            if result.smoothing_stats is not None
            else None
        ),
        "path": rows,
    }
    _atomic_json(json_path, payload)


def invalidate_outputs(reason: str) -> None:
    """실패 후 이전 성공 경로가 최신 경로로 오인되지 않도록 제거한다."""
    output_dir = PROJECT_ROOT / "output"
    for name in OUTPUT_FILES:
        (output_dir / name).unlink(missing_ok=True)
    _atomic_json(
        output_dir / "live_hybrid_planning_status.json",
        {
            "success": False,
            "updated_at_unix_sec": time.time(),
            "message": reason,
        },
    )


def _atomic_dict_csv(
    path: Path,
    columns: Sequence[str],
    rows: Sequence[dict[str, int | float]],
) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=columns)
        writer.writeheader()
        writer.writerows(rows)
    temporary.replace(path)


def _atomic_camera_csv(path: Path, points: np.ndarray) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("w", newline="", encoding="utf-8") as file:
        writer = csv.writer(file)
        writer.writerow(["index", "x_px", "y_px"])
        for index, point in enumerate(points):
            writer.writerow([index, float(point[0]), float(point[1])])
    temporary.replace(path)


def _atomic_json(path: Path, payload: dict[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("w", encoding="utf-8") as file:
        json.dump(payload, file, indent=2, ensure_ascii=False)
        file.write("\n")
    temporary.replace(path)


def _pose_dict(pose: Pose) -> dict[str, float]:
    return {"x_cm": pose[0], "y_cm": pose[1], "yaw_rad": pose[2]}


def main() -> int:
    args = parse_args()
    try:
        grid_map, planner, profile_config, limits = load_planner_stack()
        request = wait_for_request(
            args.scene,
            args.max_age_sec,
            args.wait_sec,
            (grid_map.width, grid_map.height),
            grid_map.resolution_cm,
        )
        (
            selected_candidate,
            adjusted_start,
            adjusted_goal,
            result,
            trajectory,
            validation,
        ) = plan_and_validate(
            planner,
            profile_config,
            limits,
            request.start_pose_cm,
            (
                ("primary goal", request.goal_pose_cm),
                ("alternative goal", request.alternative_goal_pose_cm),
            ),
        )
        save_outputs(
            request,
            selected_candidate,
            adjusted_start,
            adjusted_goal,
            result,
            trajectory,
            validation,
            grid_map.resolution_cm,
            profile_config,
        )
        _atomic_json(
            PROJECT_ROOT / "output/live_hybrid_planning_status.json",
            {
                "success": True,
                "updated_at_unix_sec": time.time(),
                "source_frame_index": request.frame_index,
                "parking_slot": request.slot_name,
                "trajectory_points": len(trajectory),
            },
        )
    except (
        FileNotFoundError,
        KeyError,
        OSError,
        RuntimeError,
        TypeError,
        ValueError,
        VisionSceneUnavailable,
    ) as error:
        message = f"Automatic Hybrid A* blocked: {error}"
        invalidate_outputs(message)
        print(message)
        return 1

    print(f"Source frame: {request.frame_index}")
    print(f"Detected vehicle start: {request.start_pose_cm}")
    print(f"Adjusted start: {adjusted_start}")
    print(f"Selected parking slot: {request.slot_name}")
    print(f"Selected goal: {adjusted_goal} ({selected_candidate})")
    print("Hybrid A* path found and validation passed")
    print(f"Trajectory points: {len(trajectory)}")
    print(f"Total cost: {result.total_cost:.3f}")
    print(f"Expanded nodes: {result.expanded_nodes}")
    print("Saved live Hybrid world path: output/live_hybrid_path_world_cm.csv")
    print("Saved live Hybrid Camera path: output/live_hybrid_path_camera_bev.csv")
    print("Saved live Hybrid JSON: output/live_hybrid_path_world_cm.json")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
