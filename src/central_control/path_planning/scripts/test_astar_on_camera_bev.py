"""Project an inflated-grid A* path onto a Camera BEV using rigid registration."""

from pathlib import Path
import sys
from typing import Sequence

import cv2
import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parent
sys.path.append(str(PROJECT_ROOT / "src"))
sys.path.append(str(SCRIPT_DIR))

from astar_planner import AStarPlanner  # noqa: E402
from occupancy_grid import OccupancyGridMap  # noqa: E402
from test_astar import (  # noqa: E402
    find_nearest_free,
    find_nearest_reachable_free,
    resolve_map_image,
)
from visualization import draw_path_on_image  # noqa: E402

GridPoint = tuple[int, int]
PixelPoint = tuple[float, float]

BEV_WIDTH = 1600
BEV_HEIGHT = 800
INFLATION_RADIUS_CM = 7.0
BEV_IMAGE_CANDIDATES = (
    "bev_result.png",
    "latest_bev.png",
    "camera_bev.png",
    "live_bev.png",
    "undistorted_bev.png",
)


def transform_points_affine(
    points: Sequence[GridPoint], matrix: np.ndarray
) -> list[PixelPoint]:
    """Apply a 2x3 affine matrix to LiDAR/grid points."""
    if matrix.shape != (2, 3):
        raise ValueError(f"Affine matrix must have shape (2, 3), got {matrix.shape}")
    if not points:
        return []

    point_array = np.asarray(points, dtype=np.float32).reshape(-1, 1, 2)
    transformed = cv2.transform(point_array, matrix).reshape(-1, 2)
    return [(float(x), float(y)) for x, y in transformed]


def load_camera_bev(first_map_dir: Path) -> tuple[np.ndarray, Path | None]:
    """Load the first available BEV candidate or return a white 1600x800 canvas."""
    for filename in BEV_IMAGE_CANDIDATES:
        candidate = first_map_dir / filename
        if not candidate.exists():
            continue
        image = cv2.imread(str(candidate), cv2.IMREAD_COLOR)
        if image is None:
            print(f"WARNING: Failed to read BEV image candidate: {candidate}")
            continue
        return image, candidate

    blank = np.full((BEV_HEIGHT, BEV_WIDTH, 3), 255, dtype=np.uint8)
    return blank, None


def main() -> None:
    """Plan in LiDAR coordinates and project the result onto Camera BEV pixels."""
    first_map_dir = PROJECT_ROOT.parent / "camera_tools" / "first_map"
    pgm_path = first_map_dir / "my_test_map0710.pgm"
    yaml_path = first_map_dir / "my_test_map0710.yaml"
    registration_path = first_map_dir / "camera_to_lidar_rigid_registration.npz"

    if not registration_path.exists():
        raise FileNotFoundError(f"Registration NPZ not found: {registration_path}")
    with np.load(registration_path) as registration:
        if "affine_matrix" not in registration:
            raise KeyError(f"affine_matrix is missing from {registration_path}")
        lidar_from_camera = registration["affine_matrix"].astype(np.float32)
        if lidar_from_camera.shape != (2, 3):
            raise ValueError(
                f"affine_matrix must have shape (2, 3), got {lidar_from_camera.shape}"
            )
        rmse_cm = float(registration["rmse_cm"]) if "rmse_cm" in registration else float("nan")

    # Stored registration maps Camera BEV -> LiDAR, so path projection needs its inverse.
    camera_from_lidar = cv2.invertAffineTransform(lidar_from_camera)
    print(f"Loaded NPZ path: {registration_path}")
    print("Affine matrix (Camera BEV -> LiDAR):")
    print(lidar_from_camera)
    print("Inverse affine matrix (LiDAR -> Camera BEV):")
    print(camera_from_lidar)
    print(f"RMSE: {rmse_cm:.3f} cm")

    bev_image, selected_bev_path = load_camera_bev(first_map_dir)
    if selected_bev_path is None:
        print("WARNING: No Camera BEV image candidate found; using a blank 1600x800 image.")
        print("Selected BEV image path: <blank image>")
    else:
        print(f"Selected BEV image path: {selected_bev_path}")
    bev_height, bev_width = bev_image.shape[:2]
    print(f"BEV image size: {bev_width}x{bev_height}")
    if (bev_width, bev_height) != (BEV_WIDTH, BEV_HEIGHT):
        print(
            f"WARNING: Selected BEV image is {bev_width}x{bev_height}, "
            f"expected {BEV_WIDTH}x{BEV_HEIGHT}."
        )

    grid_map = OccupancyGridMap(
        str(resolve_map_image(pgm_path, yaml_path)),
        str(yaml_path),
        block_outside_area=True,
    )
    grid_map.inflate_obstacles(INFLATION_RADIUS_CM, grid_map.resolution_cm)
    planning_grid = (
        grid_map.inflated_grid
        if grid_map.inflated_grid is not None
        else grid_map.get_grid()
    )
    height, width = planning_grid.shape

    original_start = (20, 20)
    start = find_nearest_free(planning_grid, original_start)
    original_goal = (200, 180)
    bounded_goal = (
        min(max(original_goal[0], 0), max(0, width - 20)),
        min(max(original_goal[1], 0), max(0, height - 20)),
    )
    nearby_goal = find_nearest_free(planning_grid, bounded_goal)
    goal = find_nearest_reachable_free(planning_grid, start, nearby_goal)

    print(f"Original start: {original_start}")
    print(f"Adjusted start: {start}")
    print(f"Original goal: {original_goal}")
    print(f"Adjusted goal: {goal}")

    planner = AStarPlanner(
        planning_grid,
        allow_diagonal=True,
        prevent_corner_cutting=True,
    )
    result = planner.plan(start, goal)
    if not result.success:
        print(f"A* failed: no path from {start} to {goal}")
        return

    path_px = transform_points_affine(result.path, camera_from_lidar)
    start_px = transform_points_affine([start], camera_from_lidar)[0]
    goal_px = transform_points_affine([goal], camera_from_lidar)[0]
    output_path = PROJECT_ROOT / "output" / "astar_on_camera_bev.png"
    skipped_count = draw_path_on_image(
        bev_image,
        path_px,
        start_px,
        goal_px,
        str(output_path),
    )

    print("Path found")
    print(f"Path length: {len(result.path)}")
    print(f"Total cost: {result.total_cost:.3f}")
    print(f"Out-of-bounds path points: {skipped_count}/{len(result.path)}")
    if skipped_count:
        print(f"WARNING: Skipped {skipped_count} path points outside the BEV image.")
    print(f"Saved output path: {output_path}")


if __name__ == "__main__":
    main()
