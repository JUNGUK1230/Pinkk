"""Interactively select A* start/goal points on a Camera BEV image.

Left click selects start and then goal. Press r to reset, s to save the current
result, and q or ESC to quit.
"""

import csv
import json
from pathlib import Path
import sys

import cv2
import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parent
sys.path.append(str(PROJECT_ROOT / "src"))
sys.path.append(str(SCRIPT_DIR))

from astar_planner import AStarPlanner  # noqa: E402
from occupancy_grid import OccupancyGridMap  # noqa: E402
from path_postprocess import build_path_rows, rdp_simplify  # noqa: E402
from test_astar import find_nearest_free, resolve_map_image  # noqa: E402
from test_astar_on_camera_bev import transform_points_affine  # noqa: E402
from visualization import draw_visible_polyline  # noqa: E402

GridPoint = tuple[int, int]
PixelPoint = tuple[float, float]

WINDOW_NAME = "Click A* on Camera BEV"
INFLATION_RADIUS_CM = 7.0
ORIGINAL_POINT_RADIUS = 4
ADJUSTED_POINT_RADIUS = 10
RDP_EPSILON_CM = 3.0


class InteractiveAStarApp:
    """Manage mouse selections, coordinate transforms, planning, and rendering."""

    def __init__(
        self,
        camera_bev: np.ndarray,
        planning_grid: np.ndarray,
        lidar_from_camera: np.ndarray,
        camera_from_lidar: np.ndarray,
        output_path: Path,
        resolution_cm: float = 1.0,
    ) -> None:
        self.base_image = camera_bev
        self.planning_grid = planning_grid
        self.lidar_from_camera = lidar_from_camera
        self.camera_from_lidar = camera_from_lidar
        self.output_path = output_path
        if resolution_cm <= 0:
            raise ValueError("resolution_cm must be positive")
        self.resolution_cm = resolution_cm

        self.camera_clicks: list[GridPoint] = []
        self.adjusted_lidar_points: list[GridPoint] = []
        self.path_lidar_grid: list[GridPoint] = []
        self.path_camera_px: list[PixelPoint] = []
        self.display_image = self.base_image.copy()

    def handle_click(self, camera_point: GridPoint) -> None:
        """Record start/goal clicks and plan after the second click."""
        if len(self.camera_clicks) >= 2:
            print("Start and goal are already selected. Press 'r' to reset.")
            return

        self.camera_clicks.append(camera_point)
        if len(self.camera_clicks) == 1:
            print(f"Camera start: {camera_point}")
            self._render()
            return

        print(f"Camera goal: {camera_point}")
        self._plan_from_clicks()

    def _plan_from_clicks(self) -> None:
        """Transform both clicks to LiDAR cells, adjust them, and run A*."""
        lidar_float = transform_points_affine(
            self.camera_clicks, self.lidar_from_camera
        )
        lidar_before = [
            (int(round(x)), int(round(y))) for x, y in lidar_float
        ]
        start_before, goal_before = lidar_before
        print(f"LiDAR start before adjustment: {start_before}")
        print(f"LiDAR goal before adjustment: {goal_before}")

        for label, point in (("start", start_before), ("goal", goal_before)):
            if not self._grid_contains(point):
                print(f"WARNING: LiDAR {label} {point} is outside the grid.")

        try:
            start = find_nearest_free(self.planning_grid, start_before)
            goal = find_nearest_free(self.planning_grid, goal_before)
        except ValueError as error:
            print(f"A* input adjustment failed: {error}")
            self.adjusted_lidar_points = []
            self.path_lidar_grid = []
            self.path_camera_px = []
            self._render()
            return

        self.adjusted_lidar_points = [start, goal]
        print(f"LiDAR start adjusted: {start}")
        print(f"LiDAR goal adjusted: {goal}")

        planner = AStarPlanner(
            self.planning_grid,
            allow_diagonal=True,
            prevent_corner_cutting=True,
        )
        result = planner.plan(start, goal)
        if not result.success:
            self.path_lidar_grid = []
            self.path_camera_px = []
            self._render()
            print(f"A* failed: no path from {start} to {goal}")
            return

        # Keep the authoritative LiDAR path; every controller export derives from it.
        self.path_lidar_grid = list(result.path)
        self.path_camera_px = transform_points_affine(
            self.path_lidar_grid, self.camera_from_lidar
        )
        skipped_count = self._render()
        print("Path found")
        print(f"Path length: {len(result.path)}")
        print(f"Total cost: {result.total_cost:.3f}")
        print(f"Out-of-bounds path points: {skipped_count}/{len(result.path)}")

    def _render(self) -> int:
        """Rebuild the display from the immutable Camera BEV image."""
        canvas = self.base_image.copy()
        skipped_count = draw_visible_polyline(
            canvas, self.path_camera_px, color=(0, 0, 255), thickness=3
        )

        # Draw adjusted points as large rings, then original clicks as small dots.
        if self.adjusted_lidar_points:
            adjusted_px = transform_points_affine(
                self.adjusted_lidar_points, self.camera_from_lidar
            )
            self._draw_marker(canvas, adjusted_px[0], (0, 255, 0), ADJUSTED_POINT_RADIUS, 3)
            self._draw_marker(canvas, adjusted_px[1], (255, 0, 0), ADJUSTED_POINT_RADIUS, 3)
        if self.camera_clicks:
            self._draw_marker(canvas, self.camera_clicks[0], (0, 255, 0), ORIGINAL_POINT_RADIUS, -1)
        if len(self.camera_clicks) == 2:
            self._draw_marker(canvas, self.camera_clicks[1], (255, 0, 0), ORIGINAL_POINT_RADIUS, -1)

        self.display_image = canvas
        return skipped_count

    @staticmethod
    def _draw_marker(
        image: np.ndarray,
        point: PixelPoint,
        color: tuple[int, int, int],
        radius: int,
        thickness: int,
    ) -> None:
        x, y = int(round(point[0])), int(round(point[1]))
        height, width = image.shape[:2]
        if 0 <= x < width and 0 <= y < height:
            cv2.circle(image, (x, y), radius, color, thickness)

    def _grid_contains(self, point: GridPoint) -> bool:
        x, y = point
        height, width = self.planning_grid.shape
        return 0 <= x < width and 0 <= y < height

    def reset(self) -> None:
        """Clear clicks and path while preserving the loaded BEV image."""
        self.camera_clicks.clear()
        self.adjusted_lidar_points.clear()
        self.path_lidar_grid.clear()
        self.path_camera_px.clear()
        self._render()
        print("Selections and path reset")

    def save(self) -> None:
        """Save the result image and controller path files for the current path."""
        if not self.path_lidar_grid:
            print("No path to save.")
            return

        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        if not cv2.imwrite(str(self.output_path), self.display_image):
            print(f"ERROR: Failed to save: {self.output_path}")
            return

        output_dir = self.output_path.parent
        lidar_csv_path = output_dir / "path_lidar_grid.csv"
        camera_csv_path = output_dir / "path_camera_bev.csv"
        world_csv_path = output_dir / "path_world_cm.csv"
        world_json_path = output_dir / "path_world_cm.json"
        raw_world_csv_path = output_dir / "path_world_cm_raw.csv"
        simplified_world_csv_path = output_dir / "path_world_cm_simplified.csv"
        simplified_world_json_path = output_dir / "path_world_cm_simplified.json"

        raw_world_points = self._world_points()
        world_path = build_path_rows(raw_world_points)
        simplified_points = rdp_simplify(raw_world_points, RDP_EPSILON_CM)
        simplified_world_path = build_path_rows(simplified_points)

        try:
            with lidar_csv_path.open("w", newline="", encoding="utf-8") as file:
                writer = csv.writer(file)
                writer.writerow(["index", "x_grid", "y_grid"])
                for index, (x_grid, y_grid) in enumerate(self.path_lidar_grid):
                    writer.writerow([index, x_grid, y_grid])

            with camera_csv_path.open("w", newline="", encoding="utf-8") as file:
                writer = csv.writer(file)
                writer.writerow(["index", "x_px", "y_px"])
                for index, (x_px, y_px) in enumerate(self.path_camera_px):
                    writer.writerow([index, f"{x_px:.6f}", f"{y_px:.6f}"])

            self._write_world_csv(world_csv_path, world_path)
            # Keep a clearly named raw backup while preserving the legacy filename.
            self._write_world_csv(raw_world_csv_path, world_path)
            self._write_world_csv(simplified_world_csv_path, simplified_world_path)

            payload = {
                "frame": "lidar_map_cm",
                "resolution_cm": self.resolution_cm,
                "planner": "astar",
                "path": world_path,
            }
            with world_json_path.open("w", encoding="utf-8") as file:
                json.dump(payload, file, indent=2, ensure_ascii=False)
                file.write("\n")

            simplified_payload = {
                "frame": "lidar_map_cm",
                "resolution_cm": self.resolution_cm,
                "planner": "astar",
                "postprocess": {
                    "method": "rdp",
                    "epsilon_cm": RDP_EPSILON_CM,
                },
                "path": simplified_world_path,
            }
            with simplified_world_json_path.open("w", encoding="utf-8") as file:
                json.dump(simplified_payload, file, indent=2, ensure_ascii=False)
                file.write("\n")
        except OSError as error:
            print(f"ERROR: Failed to save path files: {error}")
            return

        print("Saved image: output/click_astar_on_camera_bev.png")
        print("Saved LiDAR grid path: output/path_lidar_grid.csv")
        print("Saved Camera BEV path: output/path_camera_bev.csv")
        print("Saved world cm path: output/path_world_cm.csv")
        print("Saved world cm json: output/path_world_cm.json")
        print(f"Raw path points: {len(world_path)}")
        print(f"Simplified path points: {len(simplified_world_path)}")
        print("Saved raw world path: output/path_world_cm_raw.csv")
        print("Saved simplified world path: output/path_world_cm_simplified.csv")
        print("Saved simplified world json: output/path_world_cm_simplified.json")

    def _world_points(self) -> list[tuple[float, float]]:
        """Convert the authoritative LiDAR grid path into world centimetres."""
        return [
            (x_grid * self.resolution_cm, y_grid * self.resolution_cm)
            for x_grid, y_grid in self.path_lidar_grid
        ]

    @staticmethod
    def _write_world_csv(
        path: Path, rows: list[dict[str, int | float]]
    ) -> None:
        """Write controller world-path rows with a stable column order."""
        columns = [
            "index", "x_cm", "y_cm", "yaw_rad", "yaw_deg", "direction"
        ]
        with path.open("w", newline="", encoding="utf-8") as file:
            writer = csv.DictWriter(file, fieldnames=columns)
            writer.writeheader()
            writer.writerows(rows)


def mouse_callback(
    event: int, x: int, y: int, flags: int, app: InteractiveAStarApp
) -> None:
    """Forward left-button clicks from OpenCV to the application."""
    del flags
    if event == cv2.EVENT_LBUTTONDOWN:
        app.handle_click((x, y))


def load_registration(path: Path) -> tuple[np.ndarray, np.ndarray]:
    """Load Camera-to-LiDAR affine registration and create its inverse."""
    if not path.exists():
        raise FileNotFoundError(f"Registration file not found: {path}")
    with np.load(path) as data:
        if "affine_matrix" not in data:
            raise KeyError(
                f"affine_matrix is missing from {path}; available keys: {list(data.files)}"
            )
        lidar_from_camera = data["affine_matrix"].astype(np.float32)
    if lidar_from_camera.shape != (2, 3):
        raise ValueError(
            f"affine_matrix must have shape (2, 3), got {lidar_from_camera.shape}"
        )
    return lidar_from_camera, cv2.invertAffineTransform(lidar_from_camera)


def main() -> int:
    """Load project assets and run the interactive OpenCV event loop."""
    first_map_dir = PROJECT_ROOT.parent / "camera_tools" / "first_map"
    camera_bev_path = first_map_dir / "camera_bev.png"
    registration_path = first_map_dir / "camera_to_lidar_rigid_registration.npz"
    yaml_path = first_map_dir / "my_test_map0710.yaml"
    pgm_path = first_map_dir / "my_test_map0710.pgm"

    if not camera_bev_path.exists():
        print(f"ERROR: Camera BEV image not found: {camera_bev_path}")
        return 1
    camera_bev = cv2.imread(str(camera_bev_path), cv2.IMREAD_COLOR)
    if camera_bev is None:
        print(f"ERROR: Failed to read Camera BEV image: {camera_bev_path}")
        return 1

    try:
        lidar_from_camera, camera_from_lidar = load_registration(registration_path)
        grid_map = OccupancyGridMap(
            str(resolve_map_image(pgm_path, yaml_path)),
            str(yaml_path),
            block_outside_area=True,
        )
    except (FileNotFoundError, KeyError, ValueError, RuntimeError, OSError) as error:
        print(f"ERROR: {error}")
        return 1

    grid_map.inflate_obstacles(INFLATION_RADIUS_CM, grid_map.resolution_cm)
    planning_grid = (
        grid_map.inflated_grid
        if grid_map.inflated_grid is not None
        else grid_map.get_grid()
    )
    print(f"Loaded camera BEV: {camera_bev_path}")
    print(f"Loaded registration: {registration_path}")
    print("Left click: start then goal | r: reset | s: save | q/ESC: quit")

    app = InteractiveAStarApp(
        camera_bev,
        planning_grid,
        lidar_from_camera,
        camera_from_lidar,
        PROJECT_ROOT / "output" / "click_astar_on_camera_bev.png",
        resolution_cm=grid_map.resolution_cm,
    )
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    cv2.setMouseCallback(WINDOW_NAME, mouse_callback, app)

    try:
        while True:
            cv2.imshow(WINDOW_NAME, app.display_image)
            key = cv2.waitKey(20) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("r"):
                app.reset()
            elif key == ord("s"):
                app.save()
    finally:
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
