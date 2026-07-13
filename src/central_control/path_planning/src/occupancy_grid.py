from collections import deque
import math
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import yaml


class OccupancyGridMap:
    """
    ROS map_server 형식의 PGM + YAML을 읽어서
    경로계획용 occupancy grid로 변환하는 클래스.

    grid 값:
        0   = free
        100 = obstacle
    """

    def __init__(
        self,
        pgm_path: str,
        yaml_path: str,
        block_outside_area: bool = True,
    ) -> None:
        self.pgm_path = Path(pgm_path)
        self.yaml_path = Path(yaml_path)

        if not self.pgm_path.exists():
            raise FileNotFoundError(f"PGM not found: {self.pgm_path}")

        if not self.yaml_path.exists():
            raise FileNotFoundError(f"YAML not found: {self.yaml_path}")

        with self.yaml_path.open("r", encoding="utf-8") as file:
            self.meta: dict[str, Any] = yaml.safe_load(file) or {}

        self.resolution_m = float(self.meta.get("resolution", 0.01))
        self.resolution_cm = self.resolution_m * 100.0
        self.origin = self.meta.get("origin", [0.0, 0.0, 0.0])
        self.occupied_thresh = float(self.meta.get("occupied_thresh", 0.65))
        self.free_thresh = float(self.meta.get("free_thresh", 0.196))
        self.block_outside_area = block_outside_area
        self.obstacle_threshold = 50

        self.raw = cv2.imread(str(self.pgm_path), cv2.IMREAD_GRAYSCALE)

        if self.raw is None:
            raise RuntimeError(f"Failed to read PGM: {self.pgm_path}")

        self.height, self.width = self.raw.shape
        self.grid = self._convert_to_occupancy(self.raw)
        if self.block_outside_area:
            self.grid = self._block_outside_area(self.grid)
        # Inflation is generated on demand so the original occupancy data remains intact.
        self.inflated_grid: np.ndarray | None = None
        self.inflation_radius_cells: int | None = None

    def _convert_to_occupancy(self, img: np.ndarray) -> np.ndarray:
        """
        PGM gray image를 binary occupancy grid로 변환.
        일반 ROS map:
            검은색에 가까움 = occupied
            흰색에 가까움 = free
        """
        grid = np.zeros_like(img, dtype=np.uint8)

        # 이 단계에서는 ROS 확률 계산 대신 명세의 고정 밝기 기준을 사용한다.
        grid[img < 100] = 100

        # 나머지는 free
        grid[img >= 100] = 0

        return grid

    def _block_outside_area(
        self,
        grid: np.ndarray,
        kernel_size: int = 5,
        iterations: int = 2,
    ) -> np.ndarray:
        """Mark border-connected free space as an outside obstacle.

        Morphological closing is applied only to the temporary detection mask. This
        seals small breaks in the outer wall without thickening parking lines or
        walls in the final occupancy grid.
        """
        obstacle_mask = (grid >= self.obstacle_threshold).astype(np.uint8)
        kernel = np.ones((kernel_size, kernel_size), dtype=np.uint8)
        obstacle_closed = cv2.morphologyEx(
            obstacle_mask,
            cv2.MORPH_CLOSE,
            kernel,
            iterations=iterations,
        )
        free_for_fill = obstacle_closed == 0
        height, width = free_for_fill.shape
        outside = np.zeros((height, width), dtype=np.uint8)
        queue: deque[tuple[int, int]] = deque()

        # Flood fill must start from every free border pixel because the outside
        # region can be split into several components by walls touching the border.
        border_points = (
            [(x, 0) for x in range(width)]
            + [(x, height - 1) for x in range(width)]
            + [(0, y) for y in range(1, height - 1)]
            + [(width - 1, y) for y in range(1, height - 1)]
        )
        for x, y in border_points:
            if free_for_fill[y, x] and outside[y, x] == 0:
                outside[y, x] = 1
                queue.append((x, y))

        while queue:
            x, y = queue.popleft()
            for nx, ny in ((x - 1, y), (x + 1, y), (x, y - 1), (x, y + 1)):
                if (
                    0 <= nx < width
                    and 0 <= ny < height
                    and free_for_fill[ny, nx]
                    and outside[ny, nx] == 0
                ):
                    outside[ny, nx] = 1
                    queue.append((nx, ny))

        # Preserve all original obstacles and add only detected outside cells.
        final_grid = grid.copy()
        final_grid[outside == 1] = 100
        return final_grid

    def get_grid(self) -> np.ndarray:
        """Return the binary occupancy grid (0: free, 100: obstacle)."""
        return self.grid

    def inflate_obstacles(
        self, radius_cm: float, resolution_cm: float | None = None
    ) -> np.ndarray:
        """Return a grid whose obstacles are expanded by the requested safe radius."""
        if radius_cm < 0:
            raise ValueError("radius_cm must be zero or positive")
        effective_resolution = self.resolution_cm if resolution_cm is None else resolution_cm
        if effective_resolution <= 0:
            raise ValueError("resolution_cm must be positive")

        # Rounding upward guarantees the requested physical clearance is not reduced.
        radius_cells = int(math.ceil(radius_cm / effective_resolution))
        kernel_size = 2 * radius_cells + 1
        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (kernel_size, kernel_size)
        )
        obstacle_mask = (self.grid >= self.obstacle_threshold).astype(np.uint8)
        inflated_mask = cv2.dilate(obstacle_mask, kernel, iterations=1)

        inflated_grid = np.zeros_like(self.grid, dtype=np.uint8)
        inflated_grid[inflated_mask > 0] = 100
        self.inflated_grid = inflated_grid
        self.inflation_radius_cells = radius_cells
        return inflated_grid

    def save_debug_image(self, save_path: str) -> None:
        """
        obstacle = black, free = white 로 저장.
        """
        vis = np.full_like(self.grid, 255, dtype=np.uint8)
        vis[self.grid >= 50] = 0
        output_path = Path(save_path)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        if not cv2.imwrite(str(output_path), vis):
            raise RuntimeError(f"Failed to save debug image: {output_path}")

    def save_inflated_debug_image(self, save_path: str) -> None:
        """Save the most recently generated inflated occupancy grid."""
        if self.inflated_grid is None:
            raise RuntimeError("inflate_obstacles() must be called before saving")

        vis = np.full_like(self.inflated_grid, 255, dtype=np.uint8)
        vis[self.inflated_grid >= self.obstacle_threshold] = 0
        output_path = Path(save_path)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        if not cv2.imwrite(str(output_path), vis):
            raise RuntimeError(f"Failed to save inflated debug image: {output_path}")

    def print_info(self) -> None:
        """Print map metadata and occupancy statistics."""
        print("=" * 60)
        print("Occupancy Grid Map")
        print("=" * 60)
        print(f"PGM path      : {self.pgm_path}")
        print(f"YAML path     : {self.yaml_path}")
        print(f"size          : {self.width} x {self.height} px")
        print(f"resolution    : {self.resolution_m:.4f} m/px")
        print(f"resolution    : {self.resolution_cm:.2f} cm/px")
        print(f"origin        : {self.origin}")
        print(f"outside block : {self.block_outside_area}")
        print(f"occupied cells: {np.sum(self.grid >= 50)}")
        print(f"free cells    : {np.sum(self.grid < 50)}")
