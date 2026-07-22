"""Plan an inflated-grid A* path and draw it on the camera/LiDAR overlay."""

from pathlib import Path
import sys

import cv2

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.append(str(PROJECT_ROOT / "src"))

from astar_planner import AStarPlanner  # noqa: E402
from coordinate_transform import BevConfig, CoordinateTransform  # noqa: E402
from occupancy_grid import OccupancyGridMap  # noqa: E402
from test_astar import (  # noqa: E402
    find_nearest_free,
    find_nearest_reachable_free,
    resolve_map_image,
)
from visualization import draw_path_on_image  # noqa: E402

GridPoint = tuple[int, int]
PixelPoint = tuple[float, float]

EXPECTED_BEV_SIZE = (1600, 800)
SCALE_PX_PER_CM = 8.0
INFLATION_RADIUS_CM = 7.0


def grid_path_to_bev_pixels(
    path: list[GridPoint],
    transform: CoordinateTransform,
    resolution_cm: float,
) -> list[PixelPoint]:
    """Convert grid cells through world centimetres into BEV pixels."""
    path_px: list[PixelPoint] = []
    for gx, gy in path:
        x_cm, y_cm = transform.grid_to_world_cm(gx, gy, resolution_cm)
        path_px.append(transform.world_cm_to_bev_px(x_cm, y_cm))
    return path_px


def is_in_image(point: PixelPoint, width: int, height: int) -> bool:
    """Return whether the rounded pixel lies inside the image."""
    x, y = int(round(point[0])), int(round(point[1]))
    return 0 <= x < width and 0 <= y < height


def main() -> None:
    """Generate an inflated A* path and save it on the rigid overlay image."""
    first_map_dir = PROJECT_ROOT.parent / "camera_tools" / "first_map"
    pgm_path = first_map_dir / "my_test_map0710.pgm"
    yaml_path = first_map_dir / "my_test_map0710.yaml"
    overlay_path = first_map_dir / "camera_lidar_rigid_overlay.png"

    if not overlay_path.exists():
        raise FileNotFoundError(f"Overlay image not found: {overlay_path}")
    overlay = cv2.imread(str(overlay_path), cv2.IMREAD_COLOR)
    if overlay is None:
        raise RuntimeError(f"Failed to read overlay image: {overlay_path}")

    overlay_height, overlay_width = overlay.shape[:2]
    if (overlay_width, overlay_height) != EXPECTED_BEV_SIZE:
        print(
            "WARNING: Overlay image size is "
            f"{overlay_width}x{overlay_height}, expected "
            f"{EXPECTED_BEV_SIZE[0]}x{EXPECTED_BEV_SIZE[1]}."
        )

    grid_map = OccupancyGridMap(
        str(resolve_map_image(pgm_path, yaml_path)),
        str(yaml_path),
        block_outside_area=True,
    )
    grid_map.inflate_obstacles(INFLATION_RADIUS_CM, grid_map.resolution_cm)
    # Use inflation when available, while retaining a safe fallback for future callers.
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

    if (overlay_width, overlay_height) == (width, height):
        # The current registration tool saves its result in LiDAR-map coordinates.
        # In that case occupancy grid[y, x] already addresses the same overlay pixel.
        print("WARNING: Overlay is LiDAR-grid-sized; using direct grid pixel alignment.")
        path_px = [(float(x), float(y)) for x, y in result.path]
        start_px = (float(start[0]), float(start[1]))
        goal_px = (float(goal[0]), float(goal[1]))
    else:
        # A camera BEV overlay uses the requested left-bottom world origin and 8 px/cm.
        transform = CoordinateTransform(
            BevConfig(overlay_width, overlay_height, SCALE_PX_PER_CM)
        )
        path_px = grid_path_to_bev_pixels(
            result.path, transform, grid_map.resolution_cm
        )
        start_px = grid_path_to_bev_pixels(
            [start], transform, grid_map.resolution_cm
        )[0]
        goal_px = grid_path_to_bev_pixels(
            [goal], transform, grid_map.resolution_cm
        )[0]

    output_path = PROJECT_ROOT / "output" / "astar_on_overlay.png"
    skipped_count = draw_path_on_image(
        overlay,
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
        print(f"WARNING: Skipped {skipped_count} path points outside the overlay image.")
    if not is_in_image(start_px, overlay_width, overlay_height):
        print(f"WARNING: Start pixel {start_px} is outside the overlay image.")
    if not is_in_image(goal_px, overlay_width, overlay_height):
        print(f"WARNING: Goal pixel {goal_px} is outside the overlay image.")
    print("Overlay image saved: output/astar_on_overlay.png")


if __name__ == "__main__":
    main()
