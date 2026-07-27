"""Draw and footprint-check the calibrated one-way road loop for C2."""

from pathlib import Path
import sys

import cv2
import numpy as np
import yaml


SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parent
sys.path.extend((str(SCRIPT_DIR), str(PROJECT_ROOT / "src")))

from plan_from_live_vision import derive_reverse_parking_goal, load_planner_stack  # noqa: E402
from t_parking_planner import plan_reverse_only_maneuver  # noqa: E402
from visualization import draw_visible_polyline  # noqa: E402


def _camera_pixels(points_cm: list[tuple[float, float]], registration_path: Path) -> list[tuple[float, float]]:
    with np.load(registration_path) as registration:
        affine = np.asarray(registration["affine_matrix"], dtype=np.float64)
        cm_per_pixel = float(registration["resolution"]) * 100.0
    inverse_linear = np.linalg.inv(affine[:, :2])
    return [
        tuple(inverse_linear @ (np.asarray(point) / cm_per_pixel - affine[:, 2]))
        for point in points_cm
    ]


def _assert_loop_is_collision_free(planner, points_cm: list[tuple[float, float]]) -> None:
    for first, second in zip(points_cm, points_cm[1:] + points_cm[:1]):
        delta = np.asarray(second) - np.asarray(first)
        length = float(np.linalg.norm(delta))
        yaw = float(np.arctan2(delta[1], delta[0]))
        for distance_cm in np.arange(0.0, length + 0.001, 1.0):
            point = np.asarray(first) + delta * min(distance_cm / length, 1.0)
            if planner.is_pose_collision(float(point[0]), float(point[1]), yaw):
                raise RuntimeError(
                    f"road loop footprint collision at ({point[0]:.1f}, {point[1]:.1f})"
                )


def _assert_polyline_is_collision_free(planner, points_cm: list[tuple[float, float]]) -> None:
    """Validate a forward centre-line connector at 1 cm footprint intervals."""
    for first, second in zip(points_cm, points_cm[1:]):
        delta = np.asarray(second) - np.asarray(first)
        length = float(np.linalg.norm(delta))
        yaw = float(np.arctan2(delta[1], delta[0]))
        for distance_cm in np.arange(0.0, length + 0.001, 1.0):
            point = np.asarray(first) + delta * min(distance_cm / length, 1.0)
            if planner.is_pose_collision(float(point[0]), float(point[1]), yaw):
                raise RuntimeError(
                    f"approach footprint collision at ({point[0]:.1f}, {point[1]:.1f})"
                )


def main() -> int:
    with (PROJECT_ROOT / "config/c2_road_loop.yaml").open(encoding="utf-8") as file:
        payload = yaml.safe_load(file)
    nodes = payload["loop_nodes"]
    loop_cm = [(float(node["x_cm"]), float(node["y_cm"])) for node in nodes]
    entrance = payload["start_points"]["entrance"]
    start_cm = (float(entrance["x_cm"]), float(entrance["y_cm"]))

    _, planner, _, _, _ = load_planner_stack()
    grid_map, planner, _, _, _ = load_planner_stack()
    goal, _, _ = derive_reverse_parking_goal(planner, "C2", grid_map.resolution_cm)
    maneuver = plan_reverse_only_maneuver(planner, goal)
    entry_loop_node = entrance["entry_loop_node"]
    entry_index = next(
        index for index, node in enumerate(nodes) if node["id"] == entry_loop_node
    )
    c2_staging_index = next(
        index for index, node in enumerate(nodes) if node["id"] == "c2_staging"
    )
    approach_cm = [start_cm]
    index = entry_index
    while True:
        approach_cm.append(loop_cm[index])
        if index == c2_staging_index:
            break
        index = (index + 1) % len(loop_cm)
    _assert_polyline_is_collision_free(planner, approach_cm)

    central_root = PROJECT_ROOT.parent
    camera_path = central_root / "camera_tools/first_map/camera_bev.png"
    registration_path = central_root / "camera_tools/first_map/camera_to_lidar_rigid_registration.npz"
    image = cv2.imread(str(camera_path), cv2.IMREAD_COLOR)
    if image is None:
        raise RuntimeError(f"failed to load Camera BEV: {camera_path}")
    approach_px = _camera_pixels(approach_cm, registration_path)
    maneuver_px = _camera_pixels(
        [(point.x_cm, point.y_cm) for point in maneuver.path], registration_path
    )
    canvas = image.copy()
    draw_visible_polyline(canvas, approach_px, color=(0, 255, 0), thickness=5)
    draw_visible_polyline(canvas, maneuver_px, color=(0, 0, 255), thickness=4)
    cv2.circle(canvas, tuple(map(int, map(round, approach_px[0]))), 8, (0, 255, 0), -1)
    for node in nodes:
        if node["id"] == "c2_staging":
            pixel = _camera_pixels(
                [(float(node["x_cm"]), float(node["y_cm"]))], registration_path
            )[0]
            cv2.putText(canvas, "C2 staging", tuple(map(int, map(round, pixel))), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA)
    output_path = PROJECT_ROOT / "output/c2_road_loop_preview.png"
    output_path.parent.mkdir(exist_ok=True)
    if not cv2.imwrite(str(output_path), canvas):
        raise RuntimeError(f"failed to save preview: {output_path}")
    print(f"approach road nodes: {len(approach_cm) - 1}")
    print("approach footprint check: passed")
    print(f"entrance start: ({start_cm[0]:.1f}, {start_cm[1]:.1f}) cm")
    print(f"entrance yaw: {float(entrance['yaw_deg']):.1f} deg")
    print(f"loop join node: {nodes[entry_index]['id']}")
    print(f"saved preview: {output_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
