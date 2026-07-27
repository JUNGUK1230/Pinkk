"""Regression test for C2's single-gear reverse-only final maneuver."""

from pathlib import Path
import sys

import cv2
import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parent
sys.path.extend((str(SCRIPT_DIR), str(PROJECT_ROOT / "src")))

from plan_from_live_vision import derive_reverse_parking_goal, load_planner_stack  # noqa: E402
from t_parking_planner import plan_reverse_only_maneuver  # noqa: E402
from trajectory_profile import build_trajectory_profile  # noqa: E402
from trajectory_validator import validate_trajectory  # noqa: E402
from visualization import draw_path_on_image  # noqa: E402


def _lidar_cm_to_camera_px(
    path_cm: list[tuple[float, float]], registration_path: Path
) -> list[tuple[float, float]]:
    """Invert Camera-BEV-pixel -> LiDAR-map-pixel registration for drawing."""
    with np.load(registration_path) as registration:
        affine = np.asarray(registration["affine_matrix"], dtype=np.float64)
        cm_per_lidar_px = float(registration["resolution"]) * 100.0
    inverse_linear = np.linalg.inv(affine[:, :2])
    translation = affine[:, 2]
    return [
        tuple(inverse_linear @ (np.asarray(point) / cm_per_lidar_px - translation))
        for point in path_cm
    ]


def main() -> int:
    grid_map, planner, profile_config, limits, _ = load_planner_stack()
    goal, entrance_edge, clearance_cm = derive_reverse_parking_goal(
        planner, "C2", grid_map.resolution_cm
    )
    maneuver = plan_reverse_only_maneuver(planner, goal)
    trajectory = build_trajectory_profile(
        maneuver.path,
        wheelbase_cm=planner.wheelbase_cm,
        max_steer_rad=max(abs(value) for value in planner.steer_set_rad),
        additional_stop_indices=set(maneuver.stop_indices),
        **profile_config,
    )
    validation = validate_trajectory(
        trajectory,
        limits,
        collision_checker=planner.is_pose_collision,
        required_final_direction=-1,
        min_final_direction_distance_cm=10.0,
    )
    assert validation.valid, validation.issues
    assert all(point.direction == -1 for point in trajectory)

    camera_bev_path = PROJECT_ROOT.parent / "camera_tools/first_map/camera_bev.png"
    registration_path = (
        PROJECT_ROOT.parent / "camera_tools/first_map/camera_to_lidar_rigid_registration.npz"
    )
    image = cv2.imread(str(camera_bev_path), cv2.IMREAD_COLOR)
    if image is None:
        raise RuntimeError(f"failed to load Camera BEV image: {camera_bev_path}")
    camera_path = _lidar_cm_to_camera_px(
        [(point.x_cm, point.y_cm) for point in trajectory], registration_path
    )
    output_path = PROJECT_ROOT / "output/c2_reverse_only_on_camera_bev.png"
    skipped = draw_path_on_image(
        image, camera_path, camera_path[0], camera_path[-1], str(output_path)
    )

    print("C2 reverse-only maneuver regression passed")
    print(f"C2 entrance edge: {entrance_edge}, clearance: {clearance_cm:.1f} cm")
    print(
        "Staging pose: "
        f"({maneuver.staging_pose[0]:.2f}, {maneuver.staging_pose[1]:.2f}, "
        f"{maneuver.staging_pose[2]:.4f} rad)"
    )
    print(f"Reverse-only length: {maneuver.total_length_cm:.2f} cm")
    print(f"Trajectory points: {len(trajectory)}")
    print(f"Steering reset stop indices: {list(maneuver.stop_indices)}")
    print(f"Camera BEV overlay: {output_path} (out-of-bounds points: {skipped})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
