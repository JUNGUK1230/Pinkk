"""최신 상단 카메라 pose와 빈 주차면 좌표로 Hybrid A* 경로를 생성한다.

상단 카메라 localization을 먼저 실행한다. 이 스크립트는 planning-ready
scene이 나타날 때까지 잠시 기다린 뒤 한 번만 계획하고 종료한다. 제어기나
ROS 2로 전송하지 않으며, validator를 통과한 trajectory만 파일로 저장한다.
"""

import argparse
import csv
from dataclasses import dataclass
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
from t_parking_planner import plan_t_reverse_parking  # noqa: E402
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
from visualization import draw_visible_polyline  # noqa: E402


Pose = tuple[float, float, float]
POSE_ADJUSTMENT_RADIUS_CM = 30.0
# 주차면 명목 중심이 안전마진·정합 오차에 걸리거나 smoothing 곡률이 큰 경우,
# 주차면 안쪽의 작은 범위에서 동일 heading의 rear-axle goal을 다시 탐색한다.
GOAL_SEARCH_RADIUS_CM = 3.0
GOAL_SEARCH_STEP_CM = 1.0
# 실시간 경로 생성은 주차면 입구를 향하는 하나의 자동 final yaw를 사용한다.
# 시작 yaw는 차체 footprint와 전진/후진 kinematics에 반드시 필요하다.
LIVE_REQUIRE_GOAL_HEADING = True
ENTRY_CLEARANCE_PROBE_CM = 20.0
DEFAULT_PARKING_SLOTS_PATH = CENTRAL_ROOT / "config/map/parking_slots_bev.json"
# 0은 시간 제한 없음이다. 실제 자동 주차 기본 실행은 후보별/전체 시간으로
# 중단하지 않고, planner의 확장 노드 상한만 비정상 폭증 보호용으로 유지한다.
DEFAULT_TOTAL_PLANNING_TIMEOUT_SEC = 0.0
DEFAULT_CANDIDATE_TIMEOUT_SEC = 0.0
DEFAULT_SCENE_PATH = PROJECT_ROOT / "output/live_vision_scene.json"
DEFAULT_REGISTRATION_PATH = (
    CENTRAL_ROOT
    / "camera_tools/first_map/camera_to_lidar_rigid_registration.npz"
)
DEFAULT_LIVE_BEV_PATH = PROJECT_ROOT / "output/live_camera_bev.png"
FALLBACK_BEV_PATH = CENTRAL_ROOT / "camera_tools/first_map/camera_bev.png"
OUTPUT_FILES = (
    "live_hybrid_path_world_cm.csv",
    "live_hybrid_path_camera_bev.csv",
    "live_hybrid_path_world_cm.json",
    "live_hybrid_path_on_camera_bev.png",
)


@dataclass(frozen=True)
class AutomaticParkingConfig:
    """주차칸 종류와 무관한 자동 주차 종단 방향 조건."""

    required_final_direction: int


@dataclass(frozen=True)
class LivePlanningOutcome:
    """파일 저장과 분리된 한 번의 검증 완료 Hybrid A* 결과."""

    request: VisionPlanningRequest
    selected_candidate: str
    adjusted_start: Pose
    adjusted_goal: Pose
    result: HybridAStarResult
    trajectory: tuple[TrajectoryPoint, ...]
    validation: TrajectoryValidationResult
    staging_pose: Pose


def resolve_map_image(pgm_path: Path, yaml_path: Path) -> Path:
    """PGM이 없으면 지도 YAML의 image 항목을 기준으로 실제 이미지를 찾는다.

    실시간 경로 생성기가 예전 A* 테스트 스크립트에 의존하지 않도록 지도
    이미지 선택 로직을 이 실행 파일 안에 둔다.
    """
    if pgm_path.exists():
        return pgm_path

    with yaml_path.open("r", encoding="utf-8") as file:
        image_name = (yaml.safe_load(file) or {}).get("image")

    yaml_image = yaml_path.parent / image_name if image_name else None
    if yaml_image is not None and yaml_image.exists():
        print(f"PGM not found; using YAML map image instead: {yaml_image}")
        return yaml_image

    # OccupancyGridMap이 파일 누락 원인을 명확하게 보고하도록 원래 경로를 반환한다.
    return pgm_path


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
    parser.add_argument(
        "--planning-timeout-sec",
        type=float,
        default=DEFAULT_TOTAL_PLANNING_TIMEOUT_SEC,
        help=(
            "Maximum total Hybrid A* planning time. "
            "The default 0 disables the time limit."
        ),
    )
    parser.add_argument(
        "--candidate-timeout-sec",
        type=float,
        default=DEFAULT_CANDIDATE_TIMEOUT_SEC,
        help=(
            "Maximum time for one nearby goal candidate. "
            "The default 0 disables the time limit."
        ),
    )
    return parser.parse_args()


def load_planner_stack() -> tuple[
    OccupancyGridMap,
    HybridAStarPlanner,
    dict[str, float],
    TrajectoryValidationLimits,
    AutomaticParkingConfig,
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
    automatic_config = config["automatic_parking"]
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
        max_expanded_nodes=int(hybrid["max_expanded_nodes"]),
        analytic_expansion_interval_nodes=int(
            hybrid["analytic_expansion_interval_nodes"]
        ),
        analytic_max_candidates=int(hybrid["analytic_max_candidates"]),
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
    parking_config = AutomaticParkingConfig(
        required_final_direction=int(
            automatic_config["required_final_direction"]
        )
    )
    if parking_config.required_final_direction not in (-1, 1):
        raise ValueError("automatic parking direction must be -1 or 1")
    return grid_map, planner, profile_config, limits, parking_config


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
    planning_budget_sec: float = DEFAULT_TOTAL_PLANNING_TIMEOUT_SEC,
    candidate_timeout_sec: float = DEFAULT_CANDIDATE_TIMEOUT_SEC,
    required_goal_direction: int | None = None,
    require_goal_heading: bool = True,
    search_nearby_goal: bool = True,
) -> tuple[
    str,
    Pose,
    Pose,
    HybridAStarResult,
    list[TrajectoryPoint],
    TrajectoryValidationResult,
]:
    """목표 위치 후보 중 검증된 첫 경로를 반환한다.

    두 timeout은 독립적이다. 0은 명시적인 진단 실행에서만 무제한을 뜻한다.
    """
    if planning_budget_sec < 0.0:
        raise ValueError("planning budget must not be negative")
    if candidate_timeout_sec < 0.0:
        raise ValueError("candidate timeout must not be negative")
    adjusted_start = adjust_pose(planner, start, "Start")
    planning_deadline = (
        None
        if planning_budget_sec == 0.0
        else time.monotonic() + planning_budget_sec
    )
    failures: list[str] = []
    prepared_candidates: list[
        tuple[str, Sequence[tuple[str, Pose]], np.ndarray]
    ] = []
    candidate_failures: dict[str, list[str]] = {}
    attempted_counts: dict[str, int] = {}
    for candidate_name, raw_goal in goal_candidates:
        try:
            # Heading-free 목표는 특정 차체 각도로 주차면 중심을 보정하지 않는다.
            # planner가 내부 yaw 상태 중 충돌 없는 도착 자세를 직접 선택한다.
            nominal_goal = (
                adjust_pose(planner, raw_goal, candidate_name)
                if require_goal_heading
                else raw_goal
            )
            poses = (
                _nearby_goal_poses(planner, nominal_goal, candidate_name)
                if require_goal_heading and search_nearby_goal
                else ((candidate_name, nominal_goal),)
            )
            # 같은 목표 위치의 nearby pose는 불과 몇 cm 차이다. 명목 goal에서
            # 만든 expensive 2D cost-to-go를 재사용해 후보당 Dijkstra를 없앤다.
            cost_map = planner.build_holonomic_cost_to_goal(
                nominal_goal[0],
                nominal_goal[1],
            )
            prepared_candidates.append((candidate_name, poses, cost_map))
            candidate_failures[candidate_name] = []
            attempted_counts[candidate_name] = 0
        except (TypeError, ValueError) as error:
            failures.append(f"{candidate_name}: {error}")

    # 두 heading의 명목 goal을 먼저 확인한다. 이어서 각 heading의 가까운
    # nearby pose에 같은 수의 기회를 주고 나머지도 번갈아 탐색한다.
    ordered_attempts: list[tuple[str, str, Pose, np.ndarray, int]] = []
    for candidate_name, poses, cost_map in prepared_candidates:
        if poses:
            goal_label, adjusted_goal = poses[0]
            ordered_attempts.append(
                (
                    candidate_name,
                    goal_label,
                    adjusted_goal,
                    cost_map,
                    len(poses),
                )
            )
    priority_nearby_count = 2
    for candidate_name, poses, cost_map in prepared_candidates:
        for goal_label, adjusted_goal in poses[1 : 1 + priority_nearby_count]:
            ordered_attempts.append(
                (
                    candidate_name,
                    goal_label,
                    adjusted_goal,
                    cost_map,
                    len(poses),
                )
            )
    maximum_candidate_count = max(
        (len(poses) for _, poses, _ in prepared_candidates),
        default=0,
    )
    for pose_index in range(1 + priority_nearby_count, maximum_candidate_count):
        for candidate_name, poses, cost_map in prepared_candidates:
            if pose_index >= len(poses):
                continue
            goal_label, adjusted_goal = poses[pose_index]
            ordered_attempts.append(
                (
                    candidate_name,
                    goal_label,
                    adjusted_goal,
                    cost_map,
                    len(poses),
                )
            )

    for (
        candidate_name,
        goal_label,
        adjusted_goal,
        cost_map,
        candidate_pose_count,
    ) in ordered_attempts:
        remaining_sec = (
            None
            if planning_deadline is None
            else planning_deadline - time.monotonic()
        )
        if remaining_sec is not None and remaining_sec <= 0.0:
            candidate_failures[candidate_name].append(
                f"automatic planning budget exceeded "
                f"({planning_budget_sec:.1f} sec)"
            )
            break
        attempted_counts[candidate_name] += 1
        configured_timeout_sec = planner.timeout_sec
        effective_candidate_timeout = (
            math.inf
            if candidate_timeout_sec == 0.0
            else candidate_timeout_sec
        )
        if remaining_sec is not None:
            effective_candidate_timeout = min(
                effective_candidate_timeout,
                max(0.1, remaining_sec),
            )
        planner.timeout_sec = effective_candidate_timeout
        print(
            "Planning candidate "
            f"{goal_label} ({attempted_counts[candidate_name]}/"
            f"{candidate_pose_count})..."
        )
        candidate_started_at = time.monotonic()

        def report_progress(
            expanded_nodes: int,
            elapsed_sec: float,
            open_nodes: int,
        ) -> None:
            print(
                "  progress: "
                f"expanded={expanded_nodes}, open={open_nodes}, "
                f"elapsed={elapsed_sec:.1f}s"
            )

        try:
            result = planner.plan(
                adjusted_start,
                adjusted_goal,
                holonomic_cost_to_goal=cost_map,
                progress_callback=report_progress,
                progress_interval_nodes=1000,
                require_smoothed_path=True,
                # 주차칸 이름이나 고정 후진거리를 사용하지 않고 목표 pose까지
                # 직접 탐색한다. 실시간 모드에서는 final yaw를 강제하지 않지만
                # 마지막 이동 방향은 자동 주차 조건(후진)으로 제한한다.
                required_goal_direction=required_goal_direction,
                require_goal_heading=require_goal_heading,
            )
        except (TypeError, ValueError) as error:
            candidate_failures[candidate_name].append(str(error))
            continue
        finally:
            planner.timeout_sec = configured_timeout_sec
        candidate_elapsed_sec = time.monotonic() - candidate_started_at
        print(
            "  candidate result: "
            f"success={result.success}, expanded={result.expanded_nodes}, "
            f"elapsed={candidate_elapsed_sec:.2f}s, message={result.message}"
        )
        if not result.success:
            candidate_failures[candidate_name].append(result.message)
            continue
        try:
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
                required_final_direction=required_goal_direction,
            )
        except (TypeError, ValueError) as error:
            candidate_failures[candidate_name].append(str(error))
            continue
        if not validation.valid:
            codes = ", ".join(
                sorted({issue.code for issue in validation.issues})
            )
            candidate_failures[candidate_name].append(
                f"{result.message}; validation failed ({codes})"
            )
            continue
        if goal_label != candidate_name:
            print(
                "Selected nearby parking goal: "
                f"{goal_label} -> {adjusted_goal}"
            )
        return (
            goal_label,
            adjusted_start,
            adjusted_goal,
            result,
            trajectory,
            validation,
        )

    for candidate_name, _, _ in prepared_candidates:
        messages = candidate_failures[candidate_name]
        last_failure = (
            messages[-1] if messages else "no footprint-valid nearby goal"
        )
        failures.append(
            f"{candidate_name}: {attempted_counts[candidate_name]} nearby poses "
            f"rejected; last={last_failure}"
        )
    raise RuntimeError("; ".join(failures))


def plan_t_parking_and_validate(
    planner: HybridAStarPlanner,
    profile_config: dict[str, float],
    limits: TrajectoryValidationLimits,
    start: Pose,
    goal: Pose,
    required_goal_direction: int,
) -> tuple[
    Pose,
    HybridAStarResult,
    list[TrajectoryPoint],
    TrajectoryValidationResult,
    Pose,
]:
    """통로 접근과 T자 후면주차 maneuver를 정지 지점에서 연결·검증한다."""
    adjusted_start = adjust_pose(planner, start, "Start")
    t_plan = plan_t_reverse_parking(
        planner,
        adjusted_start,
        goal,
        required_final_direction=required_goal_direction,
        minimum_reverse_distance_cm=10.0,
    )
    trajectory = build_trajectory_profile(
        t_plan.path,
        wheelbase_cm=planner.wheelbase_cm,
        max_steer_rad=max(abs(value) for value in planner.steer_set_rad),
        additional_stop_indices=set(t_plan.stage_stop_indices),
        **profile_config,
    )
    validation = validate_trajectory(
        trajectory,
        limits,
        collision_checker=planner.is_pose_collision,
        required_final_direction=required_goal_direction,
        min_final_direction_distance_cm=10.0,
    )
    if not validation.valid:
        codes = ", ".join(sorted({issue.code for issue in validation.issues}))
        raise RuntimeError(f"T reverse parking validation failed ({codes})")
    result = HybridAStarResult(
        t_plan.path,
        t_plan.total_cost,
        True,
        t_plan.expanded_nodes,
        (
            "T reverse parking: "
            f"approach={t_plan.global_result.message}; "
            f"maneuver={t_plan.maneuver_result.message}"
        ),
    )
    return adjusted_start, result, trajectory, validation, t_plan.staging_pose


def derive_reverse_parking_goal(
    planner: HybridAStarPlanner,
    slot_name: str,
    resolution_cm: float,
) -> tuple[Pose, int, float]:
    """주차칸 입구를 향하는 후진주차 goal을 polygon에서 공통 규칙으로 만든다.

    주차칸의 짧은 두 변은 차량이 출입하는 입구 후보로 본다. 각 변 바깥쪽의
    free-cell 길이를 검사해 더 열린 쪽을 입구로 선택한다. 차량의 앞은 입구를
    향하고 rear axle은 차체 중심보다 입구 반대쪽에 있어야 하므로, 선택한
    outward yaw를 유지한 채 rear axle을 중심에서 반대쪽으로 보정한다.
    """
    with DEFAULT_PARKING_SLOTS_PATH.open(encoding="utf-8") as file:
        slot_polygons = json.load(file)
    if slot_name not in slot_polygons:
        raise ValueError(f"parking slot is missing from config: {slot_name}")
    with np.load(DEFAULT_REGISTRATION_PATH) as registration:
        lidar_from_camera = np.asarray(
            registration["affine_matrix"], dtype=np.float64
        )

    bev_points = np.asarray(slot_polygons[slot_name], dtype=np.float64)
    if bev_points.ndim != 2 or bev_points.shape[0] < 3 or bev_points.shape[1] != 2:
        raise ValueError(f"invalid parking polygon for {slot_name}")
    lidar_points_cm = (
        np.c_[bev_points, np.ones(len(bev_points))] @ lidar_from_camera.T
    ) * resolution_cm
    center_cm = lidar_points_cm.mean(axis=0)
    edge_lengths = np.linalg.norm(
        np.roll(lidar_points_cm, -1, axis=0) - lidar_points_cm, axis=1
    )
    shortest_edge_cm = float(edge_lengths.min())
    # 정합 반올림 오차로 서로 마주 보는 두 입구 변 길이가 약간 다를 수 있다.
    entrance_edge_indices = np.flatnonzero(
        edge_lengths <= shortest_edge_cm * 1.05
    )
    if len(entrance_edge_indices) < 2:
        raise ValueError(f"could not identify entrance edges for {slot_name}")

    rear_axle_offset_cm = (
        planner.vehicle_length_cm / 2.0 - planner.rear_overhang_cm
    )
    candidates: list[tuple[float, int, Pose]] = []
    for edge_index in entrance_edge_indices:
        first = lidar_points_cm[edge_index]
        second = lidar_points_cm[(edge_index + 1) % len(lidar_points_cm)]
        outward = (first + second) / 2.0 - center_cm
        outward_norm = float(np.linalg.norm(outward))
        if outward_norm <= 1e-9:
            continue
        outward /= outward_norm
        yaw_rad = math.atan2(float(outward[1]), float(outward[0]))
        requested_goal = (
            float(center_cm[0] - rear_axle_offset_cm * outward[0]),
            float(center_cm[1] - rear_axle_offset_cm * outward[1]),
            yaw_rad,
        )
        try:
            adjusted_xy = planner.find_nearest_valid_pose(
                requested_goal[0],
                requested_goal[1],
                requested_goal[2],
                POSE_ADJUSTMENT_RADIUS_CM,
            )
            goal = (adjusted_xy[0], adjusted_xy[1], requested_goal[2])
        except ValueError:
            continue
        free_length_cm = 0.0
        for distance_cm in np.arange(
            resolution_cm,
            ENTRY_CLEARANCE_PROBE_CM + resolution_cm * 0.5,
            resolution_cm,
        ):
            probe_x = int(round((center_cm[0] + outward[0] * distance_cm) / resolution_cm))
            probe_y = int(round((center_cm[1] + outward[1] * distance_cm) / resolution_cm))
            if not (
                0 <= probe_x < planner.width
                and 0 <= probe_y < planner.height
                and planner.grid[probe_y, probe_x] < planner.obstacle_threshold
            ):
                break
            free_length_cm = float(distance_cm)
        candidates.append((free_length_cm, int(edge_index), goal))
    if not candidates:
        raise ValueError(f"no footprint-valid entrance goal for {slot_name}")

    free_length_cm, edge_index, goal = max(
        candidates,
        key=lambda item: (item[0], -item[1]),
    )
    return goal, edge_index, free_length_cm


def plan_live_request(
    request: VisionPlanningRequest,
    planner: HybridAStarPlanner,
    profile_config: dict[str, float],
    limits: TrajectoryValidationLimits,
    parking_config: AutomaticParkingConfig,
) -> LivePlanningOutcome:
    """Camera scene request를 받아 파일 저장 없이 T자 후진 trajectory를 만든다."""
    entrance_goal, entrance_edge, entrance_clearance_cm = (
        derive_reverse_parking_goal(
            planner,
            request.slot_name,
            planner.resolution_cm,
        )
    )
    print(
        "Reverse parking entrance: "
        f"slot={request.slot_name}, edge={entrance_edge}, "
        f"outward clearance={entrance_clearance_cm:.1f} cm, "
        f"goal yaw={math.degrees(entrance_goal[2]):.1f} deg"
    )
    adjusted_start, result, trajectory, validation, staging_pose = (
        plan_t_parking_and_validate(
            planner,
            profile_config,
            limits,
            request.start_pose_cm,
            entrance_goal,
            parking_config.required_final_direction,
        )
    )
    print(
        "T parking staging pose: "
        f"({staging_pose[0]:.1f}, {staging_pose[1]:.1f}, "
        f"{math.degrees(staging_pose[2]):.1f} deg)"
    )
    return LivePlanningOutcome(
        request=request,
        selected_candidate="T reverse parking entrance",
        adjusted_start=adjusted_start,
        adjusted_goal=entrance_goal,
        result=result,
        trajectory=tuple(trajectory),
        validation=validation,
        staging_pose=staging_pose,
    )


def _nearby_goal_poses(
    planner: HybridAStarPlanner,
    nominal_goal: Pose,
    candidate_name: str,
) -> Sequence[tuple[str, Pose]]:
    """명목 goal부터 heading 축·수직 축의 작은 square ring 순서로 생성한다."""
    yaw = nominal_goal[2]
    forward = (math.cos(yaw), math.sin(yaw))
    lateral = (-math.sin(yaw), math.cos(yaw))
    ring_count = int(math.floor(GOAL_SEARCH_RADIUS_CM / GOAL_SEARCH_STEP_CM))
    candidates: list[tuple[str, Pose]] = []
    seen: set[tuple[int, int]] = set()
    for ring in range(ring_count + 1):
        for along_index in range(-ring, ring + 1):
            for lateral_index in range(-ring, ring + 1):
                if max(abs(along_index), abs(lateral_index)) != ring:
                    continue
                along_cm = along_index * GOAL_SEARCH_STEP_CM
                lateral_cm = lateral_index * GOAL_SEARCH_STEP_CM
                pose = (
                    nominal_goal[0]
                    + along_cm * forward[0]
                    + lateral_cm * lateral[0],
                    nominal_goal[1]
                    + along_cm * forward[1]
                    + lateral_cm * lateral[1],
                    yaw,
                )
                key = (round(pose[0] * 1000), round(pose[1] * 1000))
                if key in seen or planner.is_pose_collision(*pose):
                    continue
                seen.add(key)
                label = candidate_name
                if ring > 0:
                    label += (
                        f" offset(along={along_cm:+.1f}, "
                        f"lateral={lateral_cm:+.1f})cm"
                    )
                candidates.append((label, pose))
    return candidates


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
    parking_config: AutomaticParkingConfig | None = None,
    output_dir: Path | None = None,
    bev_image_path: Path | None = None,
) -> None:
    """검증된 경로만 좌표 파일과 Camera BEV overlay로 원자 저장한다."""
    output_dir = PROJECT_ROOT / "output" if output_dir is None else output_dir
    output_dir.mkdir(parents=True, exist_ok=True)
    world_path = output_dir / OUTPUT_FILES[0]
    camera_path = output_dir / OUTPUT_FILES[1]
    json_path = output_dir / OUTPUT_FILES[2]
    overlay_path = output_dir / OUTPUT_FILES[3]
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

    selected_bev_path = bev_image_path
    if selected_bev_path is None:
        selected_bev_path = (
            DEFAULT_LIVE_BEV_PATH
            if DEFAULT_LIVE_BEV_PATH.exists()
            else FALLBACK_BEV_PATH
        )
        if selected_bev_path == FALLBACK_BEV_PATH:
            print(
                "WARNING: latest live Camera BEV is missing; "
                f"using fallback image: {FALLBACK_BEV_PATH}"
            )
    bev_image = cv2.imread(str(selected_bev_path), cv2.IMREAD_COLOR)
    if bev_image is None:
        raise FileNotFoundError(
            f"Camera BEV for path overlay is unreadable: {selected_bev_path}"
        )
    overlay, skipped_points = draw_live_path_overlay(
        bev_image,
        camera_points,
        adjusted_start,
        adjusted_goal,
        camera_from_lidar,
        resolution_cm,
        request.frame_index,
        request.slot_name,
    )

    _atomic_dict_csv(world_path, columns, rows)
    _atomic_camera_csv(camera_path, camera_points)
    _atomic_image(overlay_path, overlay)
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
        "parking_maneuver": (
            {
                "required_final_direction": (
                    parking_config.required_final_direction
                ),
            }
            if parking_config is not None
            else None
        ),
        "validation_metrics": validation.metrics.__dict__,
        "smoothing": (
            result.smoothing_stats.to_dict()
            if result.smoothing_stats is not None
            else None
        ),
        "visualization": {
            "bev_image_path": str(selected_bev_path),
            "overlay_path": str(overlay_path),
            "out_of_bounds_path_points": skipped_points,
        },
        "path": rows,
    }
    _atomic_json(json_path, payload)


def draw_live_path_overlay(
    bev_image: np.ndarray,
    camera_points: np.ndarray,
    start_pose: Pose,
    goal_pose: Pose,
    camera_from_lidar: np.ndarray,
    resolution_cm: float,
    frame_index: int,
    slot_name: str,
) -> tuple[np.ndarray, int]:
    """Camera BEV에 경로와 rear-axle start/goal heading을 BGR 색상으로 표시한다."""
    canvas = bev_image.copy()
    path_px = [(float(point[0]), float(point[1])) for point in camera_points]
    skipped = draw_visible_polyline(
        canvas,
        path_px,
        color=(0, 0, 255),
        thickness=4,
    )
    _draw_pose_arrow(
        canvas,
        start_pose,
        camera_from_lidar,
        resolution_cm,
        color=(0, 255, 0),
    )
    _draw_pose_arrow(
        canvas,
        goal_pose,
        camera_from_lidar,
        resolution_cm,
        color=(255, 0, 0),
    )
    cv2.putText(
        canvas,
        f"frame={frame_index} slot={slot_name} | path=red start=green goal=blue",
        (20, 35),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.75,
        (0, 255, 255),
        2,
        cv2.LINE_AA,
    )
    return canvas, skipped


def _draw_pose_arrow(
    image: np.ndarray,
    pose: Pose,
    camera_from_lidar: np.ndarray,
    resolution_cm: float,
    color: tuple[int, int, int],
    arrow_length_cm: float = 12.0,
) -> None:
    points_cm = (
        (pose[0], pose[1]),
        (
            pose[0] + arrow_length_cm * math.cos(pose[2]),
            pose[1] + arrow_length_cm * math.sin(pose[2]),
        ),
    )
    lidar_points = np.asarray(
        [
            [x_cm / resolution_cm, y_cm / resolution_cm, 1.0]
            for x_cm, y_cm in points_cm
        ],
        dtype=np.float64,
    )
    camera_points = lidar_points @ camera_from_lidar.T
    start = tuple(int(round(value)) for value in camera_points[0])
    end = tuple(int(round(value)) for value in camera_points[1])
    height, width = image.shape[:2]
    if 0 <= start[0] < width and 0 <= start[1] < height:
        cv2.circle(image, start, 10, color, 3, cv2.LINE_AA)
        cv2.arrowedLine(
            image,
            start,
            end,
            color,
            3,
            cv2.LINE_AA,
            tipLength=0.2,
        )


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


def _atomic_image(path: Path, image: np.ndarray) -> None:
    temporary = path.with_name(f"{path.stem}.tmp{path.suffix}")
    if not cv2.imwrite(str(temporary), image):
        raise OSError(f"failed to save Camera BEV path overlay: {path}")
    temporary.replace(path)


def _pose_dict(pose: Pose) -> dict[str, float]:
    return {"x_cm": pose[0], "y_cm": pose[1], "yaw_rad": pose[2]}


def main() -> int:
    args = parse_args()
    try:
        (
            grid_map,
            planner,
            profile_config,
            limits,
            parking_config,
        ) = load_planner_stack()
        request = wait_for_request(
            args.scene,
            args.max_age_sec,
            args.wait_sec,
            (grid_map.width, grid_map.height),
            grid_map.resolution_cm,
        )
        outcome = plan_live_request(
            request,
            planner,
            profile_config,
            limits,
            parking_config,
        )
        save_outputs(
            outcome.request,
            outcome.selected_candidate,
            outcome.adjusted_start,
            outcome.adjusted_goal,
            outcome.result,
            outcome.trajectory,
            outcome.validation,
            grid_map.resolution_cm,
            profile_config,
            parking_config,
        )
        _atomic_json(
            PROJECT_ROOT / "output/live_hybrid_planning_status.json",
            {
                "success": True,
                "updated_at_unix_sec": time.time(),
                "source_frame_index": request.frame_index,
                "parking_slot": request.slot_name,
                "trajectory_points": len(outcome.trajectory),
                "camera_bev_overlay": (
                    "output/live_hybrid_path_on_camera_bev.png"
                ),
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
    print(f"Adjusted start: {outcome.adjusted_start}")
    print(f"Selected parking slot: {request.slot_name}")
    print(f"Selected goal: {outcome.adjusted_goal} ({outcome.selected_candidate})")
    print("Hybrid A* path found and validation passed")
    print(f"Trajectory points: {len(outcome.trajectory)}")
    print(f"Total cost: {outcome.result.total_cost:.3f}")
    print(f"Expanded nodes: {outcome.result.expanded_nodes}")
    print("Saved live Hybrid world path: output/live_hybrid_path_world_cm.csv")
    print("Saved live Hybrid Camera path: output/live_hybrid_path_camera_bev.csv")
    print("Saved live Hybrid JSON: output/live_hybrid_path_world_cm.json")
    print("Saved Camera BEV overlay: output/live_hybrid_path_on_camera_bev.png")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
