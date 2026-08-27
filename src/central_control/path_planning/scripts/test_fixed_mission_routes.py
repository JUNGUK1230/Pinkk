"""Build and validate the configured one-way routes between mission endpoints."""

from __future__ import annotations

import argparse
import copy
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
from reeds_shepp import ReedsSheppPlanner, ReedsSheppPose  # noqa: E402


Pose = tuple[float, float, float]
ROAD_TURNING_RADIUS_MARGIN_CM = 2.0
ENDPOINT_ATTACHMENT_SEARCH_CM = 25.0


def _center_pose_to_rear_axle(values: list[float], offset_cm: float) -> list[float]:
    """Convert an authored vehicle-center pose to planner rear-axle geometry."""
    x_cm, y_cm, yaw_rad = map(float, values)
    return [
        x_cm - offset_cm * math.cos(yaw_rad),
        y_cm - offset_cm * math.sin(yaw_rad),
        yaw_rad,
    ]


def _rear_axle_planning_config(authored_config: dict) -> dict:
    """Return a copy whose vehicle poses are converted for internal planning."""
    if authored_config.get("waypoint_reference") != "vehicle_center":
        raise ValueError("fixed route waypoint reference must be vehicle_center")
    offset_cm = float(authored_config["rear_axle_to_center_cm"])
    if offset_cm < 0.0:
        raise ValueError("rear_axle_to_center_cm must not be negative")
    config = copy.deepcopy(authored_config)

    for name, pose in config["road_backbone"].items():
        config["road_backbone"][name] = _center_pose_to_rear_axle(
            pose, offset_cm
        )
    for targets in config.get("route_waypoint_sequences", {}).values():
        for target, poses in targets.items():
            targets[target] = [
                _center_pose_to_rear_axle(pose, offset_cm) for pose in poses
            ]
    for targets in config.get("route_waypoint_overrides", {}).values():
        for overrides in targets.values():
            for name, pose in overrides.items():
                overrides[name] = _center_pose_to_rear_axle(pose, offset_cm)
    for endpoint in config["endpoints"].values():
        for key in ("staging", "entry_staging", "goal"):
            if key in endpoint:
                endpoint[key] = _center_pose_to_rear_axle(
                    endpoint[key], offset_cm
                )
    return config


def _load_config() -> dict:
    with (PROJECT_ROOT / "config/fixed_mission_routes.yaml").open(encoding="utf-8") as file:
        return _rear_axle_planning_config(yaml.safe_load(file))


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


def _smooth_bezier_path(
    start: Pose,
    goal: Pose,
    direction: int,
    step_cm: float,
    start_handle_cm: float,
    end_handle_cm: float,
) -> tuple[ReedsSheppPose, ...]:
    """Sample a zero-end-curvature quintic path between two vehicle poses."""
    if direction not in (-1, 1):
        raise ValueError("direction must be -1 or 1")
    if min(step_cm, start_handle_cm, end_handle_cm) <= 0.0:
        raise ValueError("Bezier step and handles must be positive")

    travel_sign = float(direction)
    start_tangent = travel_sign * np.asarray(
        [math.cos(start[2]), math.sin(start[2])], dtype=np.float64
    )
    end_tangent = travel_sign * np.asarray(
        [math.cos(goal[2]), math.sin(goal[2])], dtype=np.float64
    )
    first = np.asarray(start[:2], dtype=np.float64)
    last = np.asarray(goal[:2], dtype=np.float64)
    controls = np.asarray(
        [
            first,
            first + start_handle_cm * start_tangent,
            first + 2.0 * start_handle_cm * start_tangent,
            last - 2.0 * end_handle_cm * end_tangent,
            last - end_handle_cm * end_tangent,
            last,
        ],
        dtype=np.float64,
    )
    chord_cm = float(np.linalg.norm(last - first))
    dense_count = max(240, int(math.ceil(chord_cm / step_cm)) * 12)
    parameter = np.linspace(0.0, 1.0, dense_count + 1)
    one_minus = 1.0 - parameter
    basis = np.column_stack(
        [
            one_minus**5,
            5.0 * one_minus**4 * parameter,
            10.0 * one_minus**3 * parameter**2,
            10.0 * one_minus**2 * parameter**3,
            5.0 * one_minus * parameter**4,
            parameter**5,
        ]
    )
    dense_points = basis @ controls
    dense_distance = np.concatenate(
        (
            np.asarray([0.0]),
            np.cumsum(np.linalg.norm(np.diff(dense_points, axis=0), axis=1)),
        )
    )
    total_length_cm = float(dense_distance[-1])
    sample_count = max(1, int(math.ceil(total_length_cm / step_cm)))
    sample_distance = np.linspace(0.0, total_length_cm, sample_count + 1)
    sample_parameter = np.interp(sample_distance, dense_distance, parameter)

    poses: list[ReedsSheppPose] = []
    for index, value in enumerate(sample_parameter):
        omt = 1.0 - value
        point_basis = np.asarray(
            [
                omt**5,
                5.0 * omt**4 * value,
                10.0 * omt**3 * value**2,
                10.0 * omt**2 * value**3,
                5.0 * omt * value**4,
                value**5,
            ]
        )
        derivative_basis = np.asarray(
            [
                omt**4,
                4.0 * omt**3 * value,
                6.0 * omt**2 * value**2,
                4.0 * omt * value**3,
                value**4,
            ]
        )
        point = point_basis @ controls
        derivative = 5.0 * derivative_basis @ np.diff(controls, axis=0)
        travel_yaw = math.atan2(float(derivative[1]), float(derivative[0]))
        vehicle_yaw = travel_yaw if direction > 0 else travel_yaw + math.pi
        vehicle_yaw = (vehicle_yaw + math.pi) % (2.0 * math.pi) - math.pi
        if index == 0:
            point = first
            vehicle_yaw = start[2]
        elif index == len(sample_parameter) - 1:
            point = last
            vehicle_yaw = goal[2]
        poses.append(
            ReedsSheppPose(
                x_cm=float(point[0]),
                y_cm=float(point[1]),
                yaw_rad=float(vehicle_yaw),
                direction=direction,
                segment_mode="B",
            )
        )
    return tuple(poses)


def _aligned_parking_entry(
    planner,
    start: Pose,
    goal: Pose,
    turning_radius_cm: float,
    final_straight_cm: float,
    slot_polygon_cm: list[list[float]] | None = None,
    maximum_cusp_yaw_error_deg: float | None = None,
    minimum_initial_reverse_cm: float | None = None,
    cusp_relative_to_alignment: dict[str, float] | None = None,
    prefer_single_steer_per_gear: bool = False,
):
    """칸 밖에서 goal yaw를 완성한 뒤 직선 후진으로만 진입한다."""
    if final_straight_cm <= 0.0:
        return list(
            _direction_only_path(
                planner,
                start,
                goal,
                -1,
                turning_radius_cm,
            ).poses
        )

    alignment = (
        goal[0] + final_straight_cm * math.cos(goal[2]),
        goal[1] + final_straight_cm * math.sin(goal[2]),
        goal[2],
    )
    alignment_poses = None
    if cusp_relative_to_alignment is not None:
        longitudinal_cm = float(
            cusp_relative_to_alignment["longitudinal_cm"]
        )
        lateral_cm = float(cusp_relative_to_alignment["lateral_cm"])
        yaw_offset_rad = math.radians(
            float(cusp_relative_to_alignment["yaw_offset_deg"])
        )
        tangent_x = math.cos(goal[2])
        tangent_y = math.sin(goal[2])
        normal_x = -tangent_y
        normal_y = tangent_x
        cusp = (
            alignment[0]
            + longitudinal_cm * tangent_x
            + lateral_cm * normal_x,
            alignment[1]
            + longitudinal_cm * tangent_y
            + lateral_cm * normal_y,
            goal[2] + yaw_offset_rad,
        )
        initial_reverse = _direction_only_path(
            planner, start, cusp, -1, turning_radius_cm
        )
        forward_alignment = _direction_only_path(
            planner, cusp, alignment, 1, turning_radius_cm
        )
        if (
            minimum_initial_reverse_cm is not None
            and initial_reverse.total_length_cm + 1e-6
            < minimum_initial_reverse_cm
        ):
            raise RuntimeError("configured initial reverse is too short")
        if (
            maximum_cusp_yaw_error_deg is not None
            and abs(math.degrees(yaw_offset_rad))
            > maximum_cusp_yaw_error_deg
        ):
            raise RuntimeError("configured cusp yaw error is too large")
        alignment_poses = [
            *initial_reverse.poses,
            *forward_alignment.poses,
        ]
        if slot_polygon_cm and any(
            _point_in_convex_polygon(
                (
                    pose.x_cm
                    - planner.rear_overhang_cm * math.cos(pose.yaw_rad),
                    pose.y_cm
                    - planner.rear_overhang_cm * math.sin(pose.yaw_rad),
                ),
                slot_polygon_cm,
            )
            for pose in alignment_poses
        ):
            raise RuntimeError("configured parking alignment enters the slot early")
    else:
        reeds_shepp = ReedsSheppPlanner(
            turning_radius_cm,
            planner.path_output_step_cm,
        )
        candidates = []
        for candidate in reeds_shepp.iter_candidates(start, alignment):
            if not candidate.poses or not planner.is_path_collision_free(candidate.poses):
                continue
            if slot_polygon_cm and any(
                _point_in_convex_polygon(
                    (
                        pose.x_cm
                        - planner.rear_overhang_cm * math.cos(pose.yaw_rad),
                        pose.y_cm
                        - planner.rear_overhang_cm * math.sin(pose.yaw_rad),
                    ),
                    slot_polygon_cm,
                )
                for pose in candidate.poses
            ):
                # 정렬 maneuver 중 후미가 먼저 칸에 들어가는 후보는 버린다.
                # slot 진입은 goal yaw가 고정된 마지막 직선에서만 허용한다.
                continue
            directions = [pose.direction for pose in candidate.poses]
            # 통로에서 먼저 후진해 차체를 정렬하고, 짧게 전진해 alignment
            # 자세를 완성한다. 이후 마지막 직선 후진과 합쳐 3-point 진입이 된다.
            if directions[0] != -1 or directions[-1] != 1:
                continue
            switch_indices = [
                index
                for index in range(1, len(directions))
                if directions[index] != directions[index - 1]
            ]
            gear_switches = len(switch_indices)
            if prefer_single_steer_per_gear:
                gear_blocks: list[tuple[int, set[str]]] = []
                for segment in candidate.segments:
                    # 0.5 cm보다 짧은 수치상 잔여 원호는 실제 조향 구간으로
                    # 보지 않는다. 그 이상인 한 기어 구간에서 L/R이 함께
                    # 나오면 차체가 꼬불꼬불 움직이므로 후보에서 제외한다.
                    if abs(segment.length_cm) < planner.path_output_step_cm:
                        continue
                    if not gear_blocks or gear_blocks[-1][0] != segment.direction:
                        gear_blocks.append((segment.direction, set()))
                    gear_blocks[-1][1].add(segment.mode)
                if any(
                    "L" in modes and "R" in modes
                    for _, modes in gear_blocks
                ):
                    continue
            if minimum_initial_reverse_cm is not None:
                first_switch_index = switch_indices[0]
                initial_reverse_cm = sum(
                    math.hypot(
                        candidate.poses[index].x_cm
                        - candidate.poses[index - 1].x_cm,
                        candidate.poses[index].y_cm
                        - candidate.poses[index - 1].y_cm,
                    )
                    for index in range(1, first_switch_index)
                )
                if initial_reverse_cm + 1e-6 < minimum_initial_reverse_cm:
                    continue
            if maximum_cusp_yaw_error_deg is not None:
                cusp = candidate.poses[switch_indices[0]]
                cusp_yaw_error = abs(
                    (cusp.yaw_rad - goal[2] + math.pi)
                    % (2.0 * math.pi)
                    - math.pi
                )
                if cusp_yaw_error > math.radians(maximum_cusp_yaw_error_deg):
                    continue
            candidates.append((gear_switches, candidate.total_length_cm, candidate))
        if not candidates:
            raise RuntimeError(
                "no collision-free parking alignment path before slot entry"
            )
        _, _, alignment_path = min(candidates, key=lambda item: (item[0], item[1]))
        alignment_poses = list(alignment_path.poses)
    sample_count = max(
        1,
        int(math.ceil(final_straight_cm / planner.path_output_step_cm)),
    )
    final_reverse = tuple(
        ReedsSheppPose(
            x_cm=alignment[0] + (goal[0] - alignment[0]) * index / sample_count,
            y_cm=alignment[1] + (goal[1] - alignment[1]) * index / sample_count,
            yaw_rad=goal[2],
            direction=-1,
            segment_mode="S",
        )
        for index in range(sample_count + 1)
    )
    if not planner.is_path_collision_free(final_reverse):
        raise RuntimeError("final aligned reverse collides with the map")
    # 기어 전환 pose는 같은 좌표에서 direction만 바뀐 행으로 보존한다.
    # 이를 제거하면 첫 0.5 cm가 이전 전진 구간에 포함되어 마지막 직선
    # 후진 길이가 설정값보다 짧아지고 제어기의 기어 전환점도 흐려진다.
    return [*alignment_poses, *final_reverse]


def _point_in_convex_polygon(
    point: tuple[float, float],
    polygon: list[list[float]],
) -> bool:
    signs = []
    for first, second in zip(polygon, [*polygon[1:], polygon[0]]):
        cross = (
            (float(second[0]) - float(first[0]))
            * (point[1] - float(first[1]))
            - (float(second[1]) - float(first[1]))
            * (point[0] - float(first[0]))
        )
        if abs(cross) > 1e-9:
            signs.append(cross > 0.0)
    return not signs or all(value == signs[0] for value in signs)


def _validate_final_straight(
    rows: list[dict],
    goal: Pose,
    required_length_cm: float,
    source: str,
    target: str,
) -> None:
    if required_length_cm <= 0.0:
        return
    accumulated_cm = 0.0
    for index in range(len(rows) - 1, 0, -1):
        current = rows[index]
        previous = rows[index - 1]
        if int(current["direction"]) != -1:
            raise RuntimeError(
                f"final parking segment is not reverse: {source} -> {target}"
            )
        if int(previous["direction"]) != int(current["direction"]):
            break
        yaw_error = abs(
            (float(current["yaw_rad"]) - goal[2] + math.pi)
            % (2.0 * math.pi)
            - math.pi
        )
        if yaw_error > 1e-6:
            raise RuntimeError(
                f"parking yaw changes inside final straight: {source} -> {target}"
            )
        accumulated_cm += math.hypot(
            float(current["x_cm"]) - float(previous["x_cm"]),
            float(current["y_cm"]) - float(previous["y_cm"]),
        )
        if accumulated_cm + 1e-6 >= required_length_cm:
            return
    raise RuntimeError(
        f"final aligned reverse is shorter than {required_length_cm:.1f} cm: "
        f"{source} -> {target}"
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
    waypoint_turning_radii_cm: list[float] | None = None,
) -> list[dict]:
    """직선 waypoint chain의 모서리를 접선 원호로 교체한다."""
    points = [np.asarray(pose[:2], dtype=np.float64) for pose in waypoints]
    if len(points) < 2:
        raise ValueError("road centerline needs at least two waypoints")
    corners: list[dict[str, object]] = []
    if (
        waypoint_turning_radii_cm is not None
        and len(waypoint_turning_radii_cm) != len(points)
    ):
        raise ValueError("waypoint turning radii must match waypoint count")

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
            # NumPy 2.x에서 제거된 2D np.cross 대신 z 성분을 직접 계산한다.
            float(incoming[0] * outgoing[1] - incoming[1] * outgoing[0]),
            float(np.dot(incoming, outgoing)),
        )
        corner_radius_cm = (
            float(waypoint_turning_radii_cm[index])
            if waypoint_turning_radii_cm is not None
            else turning_radius_cm
        )
        if corner_radius_cm < turning_radius_cm:
            raise ValueError(
                "local road corner radius must not be below the common radius"
            )
        tangent_length = corner_radius_cm * math.tan(abs(turn) / 2.0)
        corners.append(
            {
                "entry": points[index] - incoming * tangent_length,
                "exit": points[index] + outgoing * tangent_length,
                "incoming": incoming,
                "turn": turn,
                "tangent_length": tangent_length,
                "radius_cm": corner_radius_cm,
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
        corner_radius_cm = float(corner["radius_cm"])
        center = entry + left_normal * turn_sign * corner_radius_cm
        start_angle = math.atan2(
            float(entry[1] - center[1]),
            float(entry[0] - center[0]),
        )
        sample_count = max(
            1,
            math.ceil(corner_radius_cm * abs(turn) / step_cm),
        )
        for sample_index in range(1, sample_count + 1):
            angle = start_angle + turn * sample_index / sample_count
            point = center + corner_radius_cm * np.asarray(
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
    turning_radius_cm = float(
        config.get(
            "road_turning_radius_cm",
            float(planner.minimum_turning_radius_cm)
            + ROAD_TURNING_RADIUS_MARGIN_CM,
        )
    )
    if turning_radius_cm < float(planner.minimum_turning_radius_cm):
        raise ValueError(
            "road_turning_radius_cm must not be below the vehicle minimum"
        )
    road_waypoints = [
        _pose(config["road_backbone"][name])
        for name in route_order
    ]
    local_corner_radii = config.get("road_corner_radii_cm", {})
    road_waypoint_radii = [
        float(local_corner_radii.get(name, turning_radius_cm))
        for name in route_order
    ]
    # START/EXIT는 검출 endpoint와 CSV 끝점이 수치상 완전히 같아야 한다.
    road_waypoints[0] = _pose(config["endpoints"][route_order[0]]["staging"])
    road_waypoints[-1] = _pose(config["endpoints"][route_order[-1]]["staging"])
    centerline = _rounded_centerline(
        road_waypoints,
        turning_radius_cm,
        float(planner.path_output_step_cm),
        road_waypoint_radii,
    )
    for row in centerline:
        if planner.is_pose_collision(row["x_cm"], row["y_cm"], row["yaw_rad"]):
            raise RuntimeError("rounded road centerline collides with the map")

    attachment_indices: dict[str, int] = {}
    entry_attachment_indices: dict[str, int] = {}
    exit_connectors: dict[str, object] = {}
    entry_connectors: dict[str, object] = {}
    minimum_index = 0
    for name in route_order:
        # route_order에는 주차 endpoint가 아닌 도로 형상 전용 transit
        # waypoint도 들어갈 수 있다. transit은 centerline에만 사용하고
        # 위치 검출이나 경로 source/target 후보로 등록하지 않는다.
        if name not in config["endpoints"]:
            continue
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
                exit_turning_radius_cm = float(
                    endpoint.get(
                        "exit_turning_radius_cm",
                        turning_radius_cm,
                    )
                )
                selected_exit = _direction_only_path(
                    planner,
                    goal,
                    attachment,
                    1,
                    exit_turning_radius_cm,
                )
                if (
                    "entry_staging" not in endpoint
                    and "entry_attachment_hint_cm" not in endpoint
                ):
                    entry_turning_radius_cm = float(
                        endpoint.get(
                            "road_attachment_entry_turning_radius_cm",
                            endpoint.get(
                                "entry_turning_radius_cm",
                                turning_radius_cm,
                            ),
                        )
                    )
                    selected_entry = _aligned_parking_entry(
                        planner,
                        attachment,
                        goal,
                        entry_turning_radius_cm,
                        (
                            0.0
                            if bool(endpoint.get("entry_from_staging", False))
                            else float(endpoint.get("entry_final_straight_cm", 0.0))
                        ),
                        endpoint.get("entry_slot_polygon_cm"),
                        endpoint.get("entry_maximum_cusp_yaw_error_deg"),
                        endpoint.get("entry_minimum_initial_reverse_cm"),
                        endpoint.get("entry_cusp_relative_to_alignment"),
                        bool(
                            endpoint.get(
                                "entry_prefer_single_steer_per_gear",
                                False,
                            )
                        ),
                    )
                else:
                    # 충전칸 진입은 삭제된 인접 주차면을 활용하는 별도
                    # staging에서 수행하므로 출차 attachment만 먼저 정한다.
                    selected_entry = []
            except RuntimeError:
                continue
            selected_index = index
            break
        if selected_index is None:
            raise RuntimeError(
                f"no curvature-safe road attachment found for endpoint {name}"
            )
        attachment_indices[name] = selected_index
        entry_attachment_indices[name] = selected_index
        if selected_exit is not None:
            exit_connectors[name] = selected_exit
        if selected_entry:
            entry_connectors[name] = selected_entry
        minimum_index = selected_index + 1

    # 출차 attachment와 주차 진입 attachment가 반드시 같을 필요는 없다.
    # DD 차량이 더 큰 반경으로 주차할 수 있도록 도로를 몇 cm 더 진행한
    # 지점에서 maneuver를 시작하되, 출차 경로와 endpoint 검출 기준은
    # 기존 attachment에 그대로 유지한다.
    for name in route_order:
        endpoint = config["endpoints"].get(name)
        if (
            endpoint is None
            or "entry_staging" in endpoint
            or "entry_attachment_hint_cm" not in endpoint
            or "goal" not in endpoint
        ):
            continue
        hint = endpoint["entry_attachment_hint_cm"]
        hint_xy = (float(hint[0]), float(hint[1]))
        sources = [
            source
            for source, targets in config["allowed_transitions"].items()
            if name in targets
        ]
        minimum_entry_index = max(
            attachment_indices[source]
            for source in sources
        )
        candidates = sorted(
            range(minimum_entry_index, len(centerline)),
            key=lambda index: (
                (float(centerline[index]["x_cm"]) - hint_xy[0]) ** 2
                + (float(centerline[index]["y_cm"]) - hint_xy[1]) ** 2
            ),
        )
        search_cm = float(
            endpoint.get(
                "entry_attachment_search_cm",
                ENDPOINT_ATTACHMENT_SEARCH_CM,
            )
        )
        goal = _pose(endpoint["goal"])
        for index in candidates:
            row = centerline[index]
            if math.hypot(
                float(row["x_cm"]) - hint_xy[0],
                float(row["y_cm"]) - hint_xy[1],
            ) > search_cm:
                break
            attachment = (
                float(row["x_cm"]),
                float(row["y_cm"]),
                float(row["yaw_rad"]),
            )
            try:
                parking_entry = _aligned_parking_entry(
                    planner,
                    attachment,
                    goal,
                    float(endpoint["entry_turning_radius_cm"]),
                    float(endpoint.get("entry_final_straight_cm", 0.0)),
                    endpoint.get("entry_slot_polygon_cm"),
                    endpoint.get("entry_maximum_cusp_yaw_error_deg"),
                    endpoint.get("entry_minimum_initial_reverse_cm"),
                    endpoint.get("entry_cusp_relative_to_alignment"),
                    bool(
                        endpoint.get(
                            "entry_prefer_single_steer_per_gear",
                            False,
                        )
                    ),
                )
            except RuntimeError:
                continue
            entry_attachment_indices[name] = index
            entry_connectors[name] = parking_entry
            break
        else:
            raise RuntimeError(
                f"no curvature-safe entry attachment found for endpoint {name}"
            )

    # 넓어진 충전칸 전면 공간은 공통 도로를 억지로 옮기지 않고,
    # 진입 전용 forward approach + 3-point parking 구간에서만 사용한다.
    for name, endpoint in config["endpoints"].items():
        if "entry_staging" not in endpoint:
            continue
        sources = [
            source
            for source, targets in config["allowed_transitions"].items()
            if name in targets
        ]
        minimum_entry_index = max(
            attachment_indices[source]
            for source in sources
        )
        maximum_entry_index = attachment_indices[name]
        entry_staging = _pose(endpoint["entry_staging"])
        goal = _pose(endpoint["goal"])
        parking_entry = _aligned_parking_entry(
            planner,
            entry_staging,
            goal,
            float(endpoint["entry_turning_radius_cm"]),
            float(endpoint.get("entry_final_straight_cm", 0.0)),
            endpoint.get("entry_slot_polygon_cm"),
            endpoint.get("entry_maximum_cusp_yaw_error_deg"),
            endpoint.get("entry_minimum_initial_reverse_cm"),
            endpoint.get("entry_cusp_relative_to_alignment"),
            bool(endpoint.get("entry_prefer_single_steer_per_gear", False)),
        )
        search_cm = float(
            endpoint.get(
                "entry_attachment_search_cm",
                ENDPOINT_ATTACHMENT_SEARCH_CM,
            )
        )
        attachment_hint = endpoint.get("entry_attachment_hint_cm")
        attachment_target = (
            (float(attachment_hint[0]), float(attachment_hint[1]))
            if attachment_hint is not None
            else entry_staging[:2]
        )
        candidates = sorted(
            range(minimum_entry_index, maximum_entry_index + 1),
            key=lambda index: (
                (float(centerline[index]["x_cm"]) - attachment_target[0]) ** 2
                + (float(centerline[index]["y_cm"]) - attachment_target[1]) ** 2
            ),
        )
        approach_final_straight_cm = float(
            endpoint.get("entry_approach_final_straight_cm", 0.0)
        )
        approach_goal = (
            entry_staging[0]
            - approach_final_straight_cm * math.cos(entry_staging[2]),
            entry_staging[1]
            - approach_final_straight_cm * math.sin(entry_staging[2]),
            entry_staging[2],
        )
        for index in candidates:
            row = centerline[index]
            distance = math.hypot(
                float(row["x_cm"]) - entry_staging[0],
                float(row["y_cm"]) - entry_staging[1],
            )
            if distance > search_cm:
                continue
            attachment = (
                float(row["x_cm"]),
                float(row["y_cm"]),
                float(row["yaw_rad"]),
            )
            try:
                approach = _direction_only_path(
                    planner,
                    attachment,
                    approach_goal,
                    1,
                    float(
                        endpoint.get(
                            "entry_approach_turning_radius_cm",
                            turning_radius_cm,
                        )
                    ),
                )
                straight_sample_count = max(
                    1,
                    int(
                        math.ceil(
                            approach_final_straight_cm
                            / planner.path_output_step_cm
                        )
                    ),
                )
                final_approach_straight = tuple(
                    ReedsSheppPose(
                        x_cm=approach_goal[0]
                        + (entry_staging[0] - approach_goal[0])
                        * sample_index
                        / straight_sample_count,
                        y_cm=approach_goal[1]
                        + (entry_staging[1] - approach_goal[1])
                        * sample_index
                        / straight_sample_count,
                        yaw_rad=entry_staging[2],
                        direction=1,
                        segment_mode="S",
                    )
                    for sample_index in range(straight_sample_count + 1)
                )
                if not planner.is_path_collision_free(final_approach_straight):
                    continue
            except RuntimeError:
                continue
            entry_attachment_indices[name] = index
            entry_connectors[name] = [
                *approach.poses,
                *final_approach_straight[1:],
                *parking_entry[1:],
            ]
            break
        else:
            raise RuntimeError(
                f"no curvature-safe entry approach found for endpoint {name}"
            )

    return {
        "centerline": centerline,
        "attachment_indices": attachment_indices,
        "entry_attachment_indices": entry_attachment_indices,
        "exit_attachment_indices": attachment_indices,
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
    entry_attachment_indices = network.get(
        "entry_attachment_indices",
        network["attachment_indices"],
    )
    exit_attachment_indices = network.get(
        "exit_attachment_indices",
        network["attachment_indices"],
    )
    exit_connectors = network["exit_connectors"]
    entry_connectors = network["entry_connectors"]
    source_index = int(exit_attachment_indices[source])
    target_index = int(entry_attachment_indices[target])
    rows: list[dict] = []

    # 3-point 주차 endpoint는 도로 attachment와 주차 maneuver를 분리한다.
    # 도로는 목표 staging에서 정확히 끝내고, 그 뒤에 endpoint별
    # 후진-전진 정렬-직선 후진 구간을 붙여 공통 도로 선택을 흔들지 않는다.
    if bool(endpoints[target].get("entry_from_staging", False)):
        custom_waypoints = (
            config.get("route_waypoint_sequences", {})
            .get(source, {})
            .get(target)
        )
        source_attachment = centerline[source_index]
        road_waypoints = [
            (
                float(source_attachment["x_cm"]),
                float(source_attachment["y_cm"]),
                float(source_attachment["yaw_rad"]),
            )
        ]
        if custom_waypoints is not None:
            road_waypoints.extend(_pose(values) for values in custom_waypoints)
        else:
            order = config["route_order"]
            source_order_index = order.index(source)
            target_order_index = order.index(target)
            corridor_names = order[source_order_index : target_order_index + 1]
            route_overrides = (
                config.get("route_waypoint_overrides", {})
                .get(source, {})
                .get(target, {})
            )
            road_waypoints.extend(
                _pose(route_overrides.get(name, config["road_backbone"][name]))
                for name in corridor_names[1:-1]
            )
        road_waypoints.append(_pose(endpoints[target]["staging"]))
        waypoint_radii = None
        if custom_waypoints is None:
            local_corner_radii = config.get("road_corner_radii_cm", {})
            waypoint_names = [source, *corridor_names[1:-1], target]
            waypoint_radii = [
                float(local_corner_radii.get(name, network["turning_radius_cm"]))
                for name in waypoint_names
            ]
        road_rows = _rounded_centerline(
            road_waypoints,
            float(network["turning_radius_cm"]),
            float(planner.path_output_step_cm),
            waypoint_radii,
        )
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
            rows.extend(dict(row) for row in road_rows[1:])
        else:
            rows = road_rows
        if "goal" in endpoints[target]:
            entry_turning_radius_cm = float(
                endpoints[target].get(
                    "entry_turning_radius_cm",
                    planner.minimum_turning_radius_cm,
                )
            )
            entry_poses = _aligned_parking_entry(
                planner,
                _pose(endpoints[target]["staging"]),
                _pose(endpoints[target]["goal"]),
                entry_turning_radius_cm,
                float(endpoints[target].get("entry_final_straight_cm", 0.0)),
                endpoints[target].get("entry_slot_polygon_cm"),
                endpoints[target].get("entry_maximum_cusp_yaw_error_deg"),
                endpoints[target].get("entry_minimum_initial_reverse_cm"),
                endpoints[target].get("entry_cusp_relative_to_alignment"),
                bool(
                    endpoints[target].get(
                        "entry_prefer_single_steer_per_gear",
                        False,
                    )
                ),
            )
            rows.extend(
                {
                    "x_cm": pose.x_cm,
                    "y_cm": pose.y_cm,
                    "yaw_rad": pose.yaw_rad,
                    "direction": pose.direction,
                }
                for pose in entry_poses[1:]
            )
            _validate_final_straight(
                rows,
                _pose(endpoints[target]["goal"]),
                float(endpoints[target].get("entry_final_straight_cm", 0.0)),
                source,
                target,
            )
        _validate_route(
            rows,
            planner,
            min(
                float(network["turning_radius_cm"]),
                float(
                    endpoints[target].get(
                        "entry_turning_radius_cm",
                        planner.minimum_turning_radius_cm,
                    )
                ),
            ),
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

    custom_waypoints = (
        config.get("route_waypoint_sequences", {})
        .get(source, {})
        .get(target)
    )
    smooth_connection = config.get("smooth_road_connections", {}).get(source)
    smooth_merge_index: int | None = None
    if smooth_connection is not None:
        smooth_targets = set(smooth_connection.get("targets", []))
        merge_name = str(smooth_connection["merge_target"])
        smooth_merge_index = int(entry_attachment_indices[merge_name])
        if target not in smooth_targets or smooth_merge_index > target_index:
            smooth_connection = None

    if custom_waypoints is None and smooth_connection is not None:
        assert smooth_merge_index is not None
        source_attachment = centerline[source_index]
        merge_attachment = centerline[smooth_merge_index]
        smooth_start = (
            float(source_attachment["x_cm"]),
            float(source_attachment["y_cm"]),
            float(source_attachment["yaw_rad"]),
        )
        smooth_goal = (
            float(merge_attachment["x_cm"]),
            float(merge_attachment["y_cm"]),
            float(merge_attachment["yaw_rad"]),
        )
        if smooth_connection.get("method") == "direction_only":
            smooth_poses = _direction_only_path(
                planner,
                smooth_start,
                smooth_goal,
                1,
                float(
                    smooth_connection.get(
                        "turning_radius_cm",
                        network["turning_radius_cm"],
                    )
                ),
            ).poses
        else:
            smooth_poses = _smooth_bezier_path(
                smooth_start,
                smooth_goal,
                1,
                float(planner.path_output_step_cm),
                float(smooth_connection["start_handle_cm"]),
                float(smooth_connection["end_handle_cm"]),
            )
        if not planner.is_path_collision_free(smooth_poses):
            raise RuntimeError(
                f"smooth road connection collides: {source} -> {target}"
            )
        rows.extend(
            {
                "x_cm": pose.x_cm,
                "y_cm": pose.y_cm,
                "yaw_rad": pose.yaw_rad,
                "direction": 1,
            }
            for pose in smooth_poses[1:]
        )
        rows.extend(
            dict(row)
            for row in centerline[smooth_merge_index + 1 : target_index + 1]
        )
    elif custom_waypoints is None:
        rows.extend(
            dict(row)
            for row in centerline[source_index + 1 : target_index + 1]
        )
    else:
        source_attachment = centerline[source_index]
        merge_point = (
            config.get("route_waypoint_merge_points", {})
            .get(source, {})
            .get(target)
        )
        merge_index = target_index
        if merge_point is not None:
            merge_pose = _pose([*merge_point, 0.0]) if len(merge_point) == 2 else _pose(merge_point)
            merge_index = min(
                range(source_index, target_index + 1),
                key=lambda index: (
                    (float(centerline[index]["x_cm"]) - merge_pose[0]) ** 2
                    + (float(centerline[index]["y_cm"]) - merge_pose[1]) ** 2
                ),
            )
        target_attachment = centerline[merge_index]
        custom_road_rows = _rounded_centerline(
            [
                (
                    float(source_attachment["x_cm"]),
                    float(source_attachment["y_cm"]),
                    float(source_attachment["yaw_rad"]),
                ),
                *(_pose(values) for values in custom_waypoints),
                (
                    float(target_attachment["x_cm"]),
                    float(target_attachment["y_cm"]),
                    float(target_attachment["yaw_rad"]),
                ),
            ],
            float(network["turning_radius_cm"]),
            float(planner.path_output_step_cm),
        )
        rows.extend(dict(row) for row in custom_road_rows[1:])
        rows.extend(
            dict(row)
            for row in centerline[merge_index + 1 : target_index + 1]
        )

    if "goal" in endpoints[target]:
        entry_poses = entry_connectors[target]
        rows.extend(
            {
                "x_cm": pose.x_cm,
                "y_cm": pose.y_cm,
                "yaw_rad": pose.yaw_rad,
                "direction": pose.direction,
            }
            for pose in entry_poses[1:]
        )
        _validate_final_straight(
            rows,
            _pose(endpoints[target]["goal"]),
            float(endpoints[target].get("entry_final_straight_cm", 0.0)),
            source,
            target,
        )

    _validate_route(
        rows,
        planner,
        min(
            float(network["turning_radius_cm"]),
            float(
                endpoints[target].get(
                    "entry_turning_radius_cm",
                    planner.minimum_turning_radius_cm,
                )
            ),
        ),
        source,
        target,
    )
    return rows


def _vehicle_center_rows(
    rows: list[dict],
    rear_axle_to_center_cm: float,
) -> list[dict]:
    """Validated rear-axle geometry를 vehicle-center trajectory로 변환한다."""
    if rear_axle_to_center_cm < 0.0:
        raise ValueError("rear_axle_to_center_cm must not be negative")
    return [
        {
            **row,
            "x_cm": float(row["x_cm"])
            + rear_axle_to_center_cm * math.cos(float(row["yaw_rad"])),
            "y_cm": float(row["y_cm"])
            + rear_axle_to_center_cm * math.sin(float(row["yaw_rad"])),
        }
        for row in rows
    ]


def save_route(
    rows: list[dict],
    source: str,
    target: str,
    rear_axle_to_center_cm: float,
) -> Path:
    output_dir = PROJECT_ROOT / "output"
    output_dir.mkdir(exist_ok=True)
    stem = f"fixed_route_{source.lower()}_to_{target.lower()}"
    csv_path = output_dir / f"{stem}.csv"
    center_rows = _vehicle_center_rows(rows, rear_axle_to_center_cm)
    with csv_path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(
            file,
            fieldnames=["index", "x_cm", "y_cm", "yaw_rad", "direction"],
            lineterminator="\n",
        )
        writer.writeheader()
        writer.writerows(
            {"index": index, **row}
            for index, row in enumerate(center_rows)
        )
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
    if config.get("trajectory_reference") != "vehicle_center":
        raise ValueError("fixed routes must be exported in vehicle_center reference")
    rear_axle_to_center_cm = float(config["rear_axle_to_center_cm"])
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
                        candidate_rows,
                        candidate_source,
                        candidate_target,
                        rear_axle_to_center_cm,
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
                    lineterminator="\n",
                )
                writer.writeheader()
                writer.writerows(manifest_rows)
            print(f"saved manifest: {manifest_path}")
        action = "generated" if args.generate_all else "validated"
        print(f"{action} fixed mission transitions: {count}")
        return 0

    rows = build_route(config, planner, source, target, road_network)
    csv_path = save_route(rows, source, target, rear_axle_to_center_cm)
    print(f"fixed route: {source} -> {target}")
    print(f"path points: {len(rows)}")
    print(f"saved csv: {csv_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
