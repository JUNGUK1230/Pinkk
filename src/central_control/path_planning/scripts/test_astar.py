"""Load the LiDAR map, run A*, and save the planned path visualization."""

from collections import deque
from pathlib import Path
import sys

import numpy as np
import yaml

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.append(str(PROJECT_ROOT / "src"))

from astar_planner import AStarPlanner  # noqa: E402
from occupancy_grid import OccupancyGridMap  # noqa: E402
from visualization import draw_grid_with_path, draw_inflation_comparison  # noqa: E402

GridPoint = tuple[int, int]


def resolve_map_image(pgm_path: Path, yaml_path: Path) -> Path:
    """Use the requested PGM, or the image declared by YAML when PGM is absent."""
    if pgm_path.exists():
        return pgm_path
    with yaml_path.open("r", encoding="utf-8") as file:
        image_name = (yaml.safe_load(file) or {}).get("image")
    yaml_image = yaml_path.parent / image_name if image_name else None
    if yaml_image is not None and yaml_image.exists():
        print(f"PGM not found; using YAML map image instead: {yaml_image}")
        return yaml_image
    return pgm_path


def find_nearest_free(
    grid: np.ndarray, point: GridPoint, max_radius: int = 30
) -> GridPoint:
    """Find the nearest free in-bounds cell around point using square rings."""
    height, width = grid.shape
    px = min(max(point[0], 0), width - 1)
    py = min(max(point[1], 0), height - 1)

    for radius in range(max_radius + 1):
        candidates: list[GridPoint] = []
        for y in range(max(0, py - radius), min(height, py + radius + 1)):
            for x in range(max(0, px - radius), min(width, px + radius + 1)):
                # Inspect only the current square ring, avoiding repeated inner cells.
                if max(abs(x - px), abs(y - py)) == radius and grid[y, x] < 50:
                    candidates.append((x, y))
        if candidates:
            return min(candidates, key=lambda p: (p[0] - px) ** 2 + (p[1] - py) ** 2)

    raise ValueError(f"No free cell found within radius {max_radius} of {point}")


def find_nearest_reachable_free(
    grid: np.ndarray, start: GridPoint, point: GridPoint
) -> GridPoint:
    """Return the cell in start's free component that is closest to point."""
    height, width = grid.shape
    queue: deque[GridPoint] = deque([start])
    visited = np.zeros(grid.shape, dtype=bool)
    visited[start[1], start[0]] = True
    reachable: list[GridPoint] = []

    while queue:
        x, y = queue.popleft()
        reachable.append((x, y))
        for nx, ny in ((x - 1, y), (x + 1, y), (x, y - 1), (x, y + 1)):
            if (
                0 <= nx < width
                and 0 <= ny < height
                and not visited[ny, nx]
                and grid[ny, nx] < 50
            ):
                visited[ny, nx] = True
                queue.append((nx, ny))

    if not reachable:
        raise ValueError(f"No reachable free cell from start {start}")

    # A nearby but wall-separated free cell is not a valid goal. Restricting the
    # adjustment to this component prevents A* from being asked for an impossible path.
    return min(
        reachable,
        key=lambda p: ((p[0] - point[0]) ** 2 + (p[1] - point[1]) ** 2, p[1], p[0]),
    )


def main() -> None:
    """Run the A* smoke test against the project LiDAR map."""
    first_map_dir = PROJECT_ROOT.parent / "camera_tools" / "first_map"
    pgm_path = first_map_dir / "my_test_map0710.pgm"
    yaml_path = first_map_dir / "my_test_map0710.yaml"
    occupancy_map = OccupancyGridMap(
        str(resolve_map_image(pgm_path, yaml_path)),
        str(yaml_path),
        block_outside_area=True,
    )
    original_grid = occupancy_map.get_grid()
    occupancy_map.save_debug_image(str(PROJECT_ROOT / "output" / "debug_occupancy_grid.png"))

    inflation_radius_cm = 7.0
    inflated_grid = occupancy_map.inflate_obstacles(
        radius_cm=inflation_radius_cm,
        resolution_cm=occupancy_map.resolution_cm,
    )
    inflated_debug_path = PROJECT_ROOT / "output" / "debug_inflated_grid.png"
    occupancy_map.save_inflated_debug_image(str(inflated_debug_path))
    radius_cells = occupancy_map.inflation_radius_cells
    height, width = inflated_grid.shape

    original_start = (20, 20)
    start = find_nearest_free(inflated_grid, original_start)
    # Keep the requested goal when possible; otherwise move it inside with a margin.
    original_goal = (200, 180)
    bounded_goal = (
        min(max(original_goal[0], 0), max(0, width - 20)),
        min(max(original_goal[1], 0), max(0, height - 20)),
    )
    nearby_goal = find_nearest_free(inflated_grid, bounded_goal)
    goal = find_nearest_reachable_free(inflated_grid, start, nearby_goal)
    print(f"Original start: {original_start}")
    print(f"Adjusted start: {start}")
    print(f"Original goal: {original_goal}")
    print(f"Adjusted goal: {goal}")
    print(f"Inflation radius: {inflation_radius_cm} cm")
    print(f"Inflation radius cells: {radius_cells}")

    planner = AStarPlanner(inflated_grid, allow_diagonal=True, prevent_corner_cutting=True)
    result = planner.plan(start, goal)
    if not result.success:
        print(f"A* failed: no path from {start} to {goal}")
        return

    astar_path = PROJECT_ROOT / "output" / "astar_result.png"
    comparison_path = PROJECT_ROOT / "output" / "astar_inflation_comparison.png"
    draw_grid_with_path(inflated_grid, result.path, start, goal, str(astar_path))
    draw_inflation_comparison(
        original_grid,
        inflated_grid,
        result.path,
        start,
        goal,
        str(comparison_path),
    )
    print("Path found")
    print(f"Path length: {len(result.path)}")
    print(f"Total cost: {result.total_cost:.3f}")
    print("Saved:")
    print("- output/debug_inflated_grid.png")
    print("- output/astar_result.png")
    print("- output/astar_inflation_comparison.png")


if __name__ == "__main__":
    main()
