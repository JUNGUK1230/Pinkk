"""Select start/goal poses on Camera BEV and run heading-aware Hybrid A*."""

import csv
import json
import math
from pathlib import Path
import sys

import cv2
import numpy as np
import yaml

SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parent
sys.path.append(str(PROJECT_ROOT / "src"))
sys.path.append(str(SCRIPT_DIR))

from hybrid_astar_planner import HybridAStarPlanner, HybridState  # noqa: E402
from occupancy_grid import OccupancyGridMap  # noqa: E402
from test_astar import resolve_map_image  # noqa: E402
from test_astar_on_camera_bev import transform_points_affine  # noqa: E402
from click_astar_on_camera_bev import load_registration  # noqa: E402
from trajectory_profile import (  # noqa: E402
    TrajectoryPoint,
    build_trajectory_profile,
)
from visualization import draw_visible_polyline  # noqa: E402

GridPoint = tuple[int, int]
PixelPoint = tuple[float, float]

WINDOW_NAME = "Click Hybrid A* on Camera BEV"
POSE_ADJUSTMENT_RADIUS_CM = 30.0
POSE_ARROW_LENGTH_CM = 12.0
CLICK_LABELS = (
    "start position",
    "start heading point",
    "goal position",
    "goal heading point",
)


class InteractiveHybridAStarApp:
    """Collect two clicked poses and visualize a kinematically feasible path."""

    def __init__(
        self,
        camera_bev: np.ndarray,
        planning_grid: np.ndarray,
        lidar_from_camera: np.ndarray,
        camera_from_lidar: np.ndarray,
        planner: HybridAStarPlanner,
        trajectory_profile_config: dict[str, float],
        resolution_cm: float,
        output_dir: Path,
    ) -> None:
        self.base_image = camera_bev
        self.planning_grid = planning_grid
        self.lidar_from_camera = lidar_from_camera
        self.camera_from_lidar = camera_from_lidar
        self.planner = planner
        self.trajectory_profile_config = trajectory_profile_config
        self.resolution_cm = resolution_cm
        self.output_dir = output_dir
        self.camera_clicks: list[GridPoint] = []
        self.adjusted_positions: list[GridPoint] = []
        self.requested_yaws: list[float] = []
        self.path_states: list[HybridState] = []
        self.trajectory_points: list[TrajectoryPoint] = []
        self.path_camera_px: list[PixelPoint] = []
        self.total_cost = math.inf
        self.display_image = camera_bev.copy()

    def handle_click(self, point: GridPoint) -> None:
        """Record four clicks: start position/direction then goal position/direction."""
        if len(self.camera_clicks) >= 4:
            print("Both poses are already selected. Press 'r' to reset.")
            return
        label = CLICK_LABELS[len(self.camera_clicks)]
        self.camera_clicks.append(point)
        print(f"Camera {label}: {point}")
        if len(self.camera_clicks) == 4:
            self._plan()
        else:
            print(f"Next click: {CLICK_LABELS[len(self.camera_clicks)]}")
            self._render()

    def _plan(self) -> None:
        lidar_clicks = transform_points_affine(
            self.camera_clicks, self.lidar_from_camera
        )
        start_float, start_heading, goal_float, goal_heading = lidar_clicks
        start_before = (round(start_float[0]), round(start_float[1]))
        goal_before = (round(goal_float[0]), round(goal_float[1]))
        try:
            start_yaw = self._heading(start_float, start_heading, "start")
            goal_yaw = self._heading(goal_float, goal_heading, "goal")
            start_cm = self.planner.find_nearest_valid_pose(
                start_before[0] * self.resolution_cm,
                start_before[1] * self.resolution_cm,
                start_yaw,
                POSE_ADJUSTMENT_RADIUS_CM,
            )
            goal_cm = self.planner.find_nearest_valid_pose(
                goal_before[0] * self.resolution_cm,
                goal_before[1] * self.resolution_cm,
                goal_yaw,
                POSE_ADJUSTMENT_RADIUS_CM,
            )
            start = tuple(round(value / self.resolution_cm) for value in start_cm)
            goal = tuple(round(value / self.resolution_cm) for value in goal_cm)
        except ValueError as error:
            print(f"Hybrid A* input error: {error}")
            self._clear_path()
            self._render()
            return

        self.adjusted_positions = [start, goal]
        self.requested_yaws = [start_yaw, goal_yaw]
        print(f"LiDAR start adjusted: {start}, yaw={math.degrees(start_yaw):.1f} deg")
        print(f"LiDAR goal adjusted: {goal}, yaw={math.degrees(goal_yaw):.1f} deg")

        result = self.planner.plan(
            (
                start[0] * self.resolution_cm,
                start[1] * self.resolution_cm,
                start_yaw,
            ),
            (
                goal[0] * self.resolution_cm,
                goal[1] * self.resolution_cm,
                goal_yaw,
            ),
        )
        if not result.success:
            self._clear_path()
            self._render()
            print(
                f"Hybrid A* failed: {result.message}, "
                f"expanded nodes={result.expanded_nodes}"
            )
            return

        try:
            trajectory_points = build_trajectory_profile(
                result.path,
                wheelbase_cm=self.planner.wheelbase_cm,
                max_steer_rad=max(abs(value) for value in self.planner.steer_set_rad),
                **self.trajectory_profile_config,
            )
        except (TypeError, ValueError) as error:
            self._clear_path()
            self._render()
            print(f"Trajectory profile failed: {error}")
            return

        self.path_states = result.path
        self.trajectory_points = trajectory_points
        self.total_cost = result.total_cost
        lidar_path_px = [
            (state.x_cm / self.resolution_cm, state.y_cm / self.resolution_cm)
            for state in result.path
        ]
        self.path_camera_px = transform_points_affine(
            lidar_path_px, self.camera_from_lidar
        )
        skipped = self._render()
        reverse_count = sum(state.direction < 0 for state in result.path)
        stop_count = sum(point.stop_required for point in trajectory_points)
        nonzero_speeds = [
            abs(point.target_speed_mps)
            for point in trajectory_points
            if abs(point.target_speed_mps) > 1e-12
        ]
        print("Hybrid A* path found")
        print(f"Path poses: {len(result.path)}")
        print(f"Total cost: {result.total_cost:.3f}")
        print(f"Expanded nodes: {result.expanded_nodes}")
        print(f"Reverse poses: {reverse_count}")
        print(f"Required stops: {stop_count}")
        if nonzero_speeds:
            print(
                "Target speed range: "
                f"{min(nonzero_speeds):.3f} to {max(nonzero_speeds):.3f} m/s"
            )
        print(f"Out-of-bounds path points: {skipped}/{len(result.path)}")

    @staticmethod
    def _heading(
        position: PixelPoint, heading_point: PixelPoint, label: str
    ) -> float:
        dx = heading_point[0] - position[0]
        dy = heading_point[1] - position[1]
        if math.hypot(dx, dy) < 1e-6:
            raise ValueError(f"{label} heading point must differ from its position")
        return math.atan2(dy, dx)

    def _render(self) -> int:
        canvas = self.base_image.copy()
        skipped = draw_visible_polyline(
            canvas, self.path_camera_px, color=(0, 0, 255), thickness=3
        )

        # Clicked pose arrows show exactly what the operator requested.
        if len(self.camera_clicks) >= 2:
            self._draw_arrow(canvas, self.camera_clicks[0], self.camera_clicks[1], (0, 255, 0))
        elif self.camera_clicks:
            cv2.circle(canvas, self.camera_clicks[0], 5, (0, 255, 0), -1)
        if len(self.camera_clicks) >= 4:
            self._draw_arrow(canvas, self.camera_clicks[2], self.camera_clicks[3], (255, 0, 0))
        elif len(self.camera_clicks) >= 3:
            cv2.circle(canvas, self.camera_clicks[2], 5, (255, 0, 0), -1)

        # Large adjusted pose arrows reveal obstacle-based position corrections.
        if self.adjusted_positions and self.requested_yaws:
            self._draw_lidar_pose(canvas, self.adjusted_positions[0], self.requested_yaws[0], (0, 255, 0))
            self._draw_lidar_pose(canvas, self.adjusted_positions[1], self.requested_yaws[1], (255, 0, 0))
        self.display_image = canvas
        return skipped

    @staticmethod
    def _draw_arrow(
        image: np.ndarray, start: GridPoint, end: GridPoint, color: tuple[int, int, int]
    ) -> None:
        cv2.circle(image, start, 5, color, -1)
        cv2.arrowedLine(image, start, end, color, 2, tipLength=0.2)

    def _draw_lidar_pose(
        self,
        image: np.ndarray,
        position: GridPoint,
        yaw_rad: float,
        color: tuple[int, int, int],
    ) -> None:
        x_px, y_px = float(position[0]), float(position[1])
        end_px = (
            x_px + POSE_ARROW_LENGTH_CM / self.resolution_cm * math.cos(yaw_rad),
            y_px + POSE_ARROW_LENGTH_CM / self.resolution_cm * math.sin(yaw_rad),
        )
        camera_points = transform_points_affine(
            [(x_px, y_px), end_px], self.camera_from_lidar
        )
        start = tuple(round(value) for value in camera_points[0])
        end = tuple(round(value) for value in camera_points[1])
        cv2.circle(image, start, 10, color, 3)
        cv2.arrowedLine(image, start, end, color, 3, tipLength=0.2)

    def _clear_path(self) -> None:
        self.adjusted_positions.clear()
        self.requested_yaws.clear()
        self.path_states.clear()
        self.trajectory_points.clear()
        self.path_camera_px.clear()
        self.total_cost = math.inf

    def reset(self) -> None:
        self.camera_clicks.clear()
        self._clear_path()
        self._render()
        print("Hybrid pose selections and path reset")
        print(f"Next click: {CLICK_LABELS[0]}")

    def save(self) -> None:
        """Save the Hybrid path with planner-generated yaw and direction."""
        if not self.path_states or not self.trajectory_points:
            print("No Hybrid A* path to save.")
            return
        self.output_dir.mkdir(parents=True, exist_ok=True)
        image_path = self.output_dir / "click_hybrid_astar_on_camera_bev.png"
        world_csv_path = self.output_dir / "hybrid_path_world_cm.csv"
        camera_csv_path = self.output_dir / "hybrid_path_camera_bev.csv"
        json_path = self.output_dir / "hybrid_path_world_cm.json"
        if not cv2.imwrite(str(image_path), self.display_image):
            print(f"ERROR: Failed to save image: {image_path}")
            return

        columns = [
            "index",
            "x_cm",
            "y_cm",
            "yaw_rad",
            "yaw_deg",
            "direction",
            "steer_deg",
            "curvature_1pm",
            "target_speed_mps",
            "target_angular_z_radps",
            "stop_required",
        ]
        rows = [
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
            for index, point in enumerate(self.trajectory_points)
        ]
        try:
            with world_csv_path.open("w", newline="", encoding="utf-8") as file:
                writer = csv.DictWriter(file, fieldnames=columns)
                writer.writeheader()
                writer.writerows(rows)
            with camera_csv_path.open("w", newline="", encoding="utf-8") as file:
                writer = csv.writer(file)
                writer.writerow(["index", "x_px", "y_px"])
                for index, (x_px, y_px) in enumerate(self.path_camera_px):
                    writer.writerow([index, f"{x_px:.6f}", f"{y_px:.6f}"])
            payload = {
                "frame": "lidar_map_cm",
                "resolution_cm": self.resolution_cm,
                "planner": "hybrid_astar",
                "path_sampling": {
                    "method": "kinematic_bicycle",
                    "output_step_cm": self.planner.path_output_step_cm,
                },
                "steering_constraints": {
                    "candidates_deg": [
                        math.degrees(value)
                        for value in self.planner.steer_set_rad
                    ],
                    "max_change_deg_per_primitive": math.degrees(
                        self.planner.max_steer_change_rad
                    ),
                },
                "trajectory_profile": {
                    **self.trajectory_profile_config,
                    "reference_max_steer_deg": math.degrees(
                        max(abs(value) for value in self.planner.steer_set_rad)
                    ),
                },
                "path": rows,
            }
            with json_path.open("w", encoding="utf-8") as file:
                json.dump(payload, file, indent=2, ensure_ascii=False)
                file.write("\n")
        except OSError as error:
            print(f"ERROR: Failed to save Hybrid path: {error}")
            return
        print("Saved image: output/click_hybrid_astar_on_camera_bev.png")
        print("Saved Hybrid world path: output/hybrid_path_world_cm.csv")
        print("Saved Hybrid Camera path: output/hybrid_path_camera_bev.csv")
        print("Saved Hybrid world JSON: output/hybrid_path_world_cm.json")


def mouse_callback(
    event: int, x: int, y: int, flags: int, app: InteractiveHybridAStarApp
) -> None:
    del flags
    if event == cv2.EVENT_LBUTTONDOWN:
        app.handle_click((x, y))


def load_settings() -> tuple[dict[str, object], dict[str, object]]:
    """Load vehicle and Hybrid A* settings from project YAML files."""
    with (PROJECT_ROOT / "config" / "vehicle_config.yaml").open(encoding="utf-8") as file:
        vehicle = (yaml.safe_load(file) or {})["vehicle"]
    with (PROJECT_ROOT / "config" / "planner_config.yaml").open(encoding="utf-8") as file:
        planner_config = yaml.safe_load(file) or {}
    return vehicle, planner_config


def main() -> int:
    first_map_dir = PROJECT_ROOT.parent / "camera_tools" / "first_map"
    camera_path = first_map_dir / "camera_bev.png"
    registration_path = first_map_dir / "camera_to_lidar_rigid_registration.npz"
    yaml_path = first_map_dir / "my_test_map0710.yaml"
    pgm_path = first_map_dir / "my_test_map0710.pgm"
    camera_bev = cv2.imread(str(camera_path), cv2.IMREAD_COLOR)
    if camera_bev is None:
        print(f"ERROR: Camera BEV image not found or unreadable: {camera_path}")
        return 1

    try:
        lidar_from_camera, camera_from_lidar = load_registration(registration_path)
        vehicle, config = load_settings()
        grid_map = OccupancyGridMap(
            str(resolve_map_image(pgm_path, yaml_path)), str(yaml_path), True
        )
        hybrid = config["hybrid_astar"]
        cost = config["cost"]
        trajectory_profile_config = {
            key: float(value)
            for key, value in config["trajectory_profile"].items()
        }
        safety_margin_cm = float(hybrid["footprint_safety_margin_cm"])
        planning_grid = grid_map.inflate_obstacles(
            safety_margin_cm, grid_map.resolution_cm
        )
        planner = HybridAStarPlanner(
            planning_grid,
            resolution_cm=grid_map.resolution_cm,
            wheelbase_cm=float(vehicle["wheelbase_cm"]),
            vehicle_length_cm=float(vehicle["length_cm"]),
            vehicle_width_cm=float(vehicle["width_cm"]),
            rear_overhang_cm=float(vehicle["rear_overhang_cm"]),
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
        )
    except (FileNotFoundError, KeyError, TypeError, ValueError, OSError) as error:
        print(f"ERROR: Failed to initialize Hybrid A*: {error}")
        return 1

    print(f"Loaded Camera BEV: {camera_path}")
    print(f"Planner: Hybrid A*, reverse={planner.allow_reverse}")
    print(
        f"Vehicle footprint: {planner.vehicle_length_cm:.1f} x "
        f"{planner.vehicle_width_cm:.1f} cm, safety margin={safety_margin_cm:.1f} cm"
    )
    print(f"Control path output step: {planner.path_output_step_cm:.2f} cm")
    print(
        "Steering candidates: "
        + ", ".join(
            f"{math.degrees(value):.0f} deg" for value in planner.steer_set_rad
        )
    )
    print(
        "Maximum steering change per primitive: "
        f"{math.degrees(planner.max_steer_change_rad):.1f} deg"
    )
    print(
        "Trajectory speed limits: forward="
        f"{trajectory_profile_config['max_forward_speed_mps']:.3f} m/s, "
        "reverse="
        f"{trajectory_profile_config['max_reverse_speed_mps']:.3f} m/s"
    )
    print("Click order: start position -> start heading -> goal position -> goal heading")
    print("r: reset | s: save | q/ESC: quit")
    app = InteractiveHybridAStarApp(
        camera_bev,
        planning_grid,
        lidar_from_camera,
        camera_from_lidar,
        planner,
        trajectory_profile_config,
        grid_map.resolution_cm,
        PROJECT_ROOT / "output",
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
