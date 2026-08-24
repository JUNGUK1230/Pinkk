"""OpenCV helpers for visualizing occupancy-grid planning results."""

from pathlib import Path
from typing import Sequence

import cv2
import numpy as np

GridPoint = tuple[int, int]
PixelPoint = tuple[float, float]


def draw_visible_polyline(
    image: np.ndarray,
    path_px: Sequence[PixelPoint],
    color: tuple[int, int, int] = (0, 0, 255),
    thickness: int = 3,
) -> int:
    """Draw only consecutive in-bounds path segments and return skipped count."""
    if image.ndim != 3 or image.shape[2] != 3:
        raise ValueError("image must be a three-channel BGR image")
    if thickness <= 0:
        raise ValueError("thickness must be positive")

    height, width = image.shape[:2]
    skipped_count = 0
    current_segment: list[tuple[int, int]] = []
    segments: list[list[tuple[int, int]]] = []

    for x_float, y_float in path_px:
        x, y = int(round(x_float)), int(round(y_float))
        if not (0 <= x < width and 0 <= y < height):
            skipped_count += 1
            # Closing the segment here prevents a line from jumping across the image.
            if current_segment:
                segments.append(current_segment)
                current_segment = []
            continue
        current_segment.append((x, y))
    if current_segment:
        segments.append(current_segment)

    for segment in segments:
        if len(segment) >= 2:
            points = np.asarray(segment, dtype=np.int32).reshape((-1, 1, 2))
            cv2.polylines(image, [points], False, color, thickness)
        elif segment:
            cv2.circle(image, segment[0], max(1, thickness // 2), color, -1)
    return skipped_count


def draw_grid_with_path(
    grid: np.ndarray,
    path: Sequence[GridPoint],
    start: GridPoint,
    goal: GridPoint,
    save_path: str,
    scale: int = 4,
) -> None:
    """Save a BGR image containing obstacles, path, start, and goal."""
    if grid.ndim != 2:
        raise ValueError("grid must be a two-dimensional array")
    if scale <= 0:
        raise ValueError("scale must be positive")

    # A white canvas is used so occupied cells can be painted black directly.
    image = np.full((*grid.shape, 3), 255, dtype=np.uint8)
    image[grid >= 50] = (0, 0, 0)
    for x, y in path:
        if 0 <= x < grid.shape[1] and 0 <= y < grid.shape[0]:
            image[y, x] = (0, 0, 255)  # BGR red

    # Nearest-neighbor scaling preserves discrete cells and obstacle boundaries.
    image = cv2.resize(image, None, fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)
    radius = max(2, scale * 2)
    cv2.circle(image, (start[0] * scale + scale // 2, start[1] * scale + scale // 2), radius, (0, 255, 0), -1)
    cv2.circle(image, (goal[0] * scale + scale // 2, goal[1] * scale + scale // 2), radius, (255, 0, 0), -1)

    output_path = Path(save_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(output_path), image):
        raise RuntimeError(f"Failed to save visualization: {output_path}")


def draw_inflation_comparison(
    original_grid: np.ndarray,
    inflated_grid: np.ndarray,
    path: Sequence[GridPoint],
    start: GridPoint,
    goal: GridPoint,
    save_path: str,
    scale: int = 4,
) -> None:
    """Save a view distinguishing original obstacles from inflation-only cells."""
    if original_grid.ndim != 2 or original_grid.shape != inflated_grid.shape:
        raise ValueError("original_grid and inflated_grid must have the same 2D shape")
    if scale <= 0:
        raise ValueError("scale must be positive")

    image = np.full((*original_grid.shape, 3), 255, dtype=np.uint8)
    original_obstacle = original_grid >= 50
    inflated_only = (inflated_grid >= 50) & ~original_obstacle
    image[inflated_only] = (160, 160, 160)  # Safety margin added by inflation.
    image[original_obstacle] = (0, 0, 0)
    for x, y in path:
        if 0 <= x < original_grid.shape[1] and 0 <= y < original_grid.shape[0]:
            image[y, x] = (0, 0, 255)

    image = cv2.resize(image, None, fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)
    radius = max(2, scale * 2)
    cv2.circle(image, (start[0] * scale + scale // 2, start[1] * scale + scale // 2), radius, (0, 255, 0), -1)
    cv2.circle(image, (goal[0] * scale + scale // 2, goal[1] * scale + scale // 2), radius, (255, 0, 0), -1)

    output_path = Path(save_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(output_path), image):
        raise RuntimeError(f"Failed to save inflation comparison: {output_path}")


def draw_path_on_image(
    image: np.ndarray,
    path_px: Sequence[PixelPoint],
    start_px: PixelPoint,
    goal_px: PixelPoint,
    save_path: str,
    line_thickness: int = 3,
    point_radius: int = 7,
) -> int:
    """Draw in-bounds path segments on an image and return skipped point count."""
    if image.ndim != 3 or image.shape[2] != 3:
        raise ValueError("image must be a three-channel BGR image")
    if line_thickness <= 0 or point_radius <= 0:
        raise ValueError("line_thickness and point_radius must be positive")

    canvas = image.copy()
    height, width = canvas.shape[:2]

    def to_visible_pixel(point: PixelPoint) -> tuple[int, int] | None:
        x, y = int(round(point[0])), int(round(point[1]))
        return (x, y) if 0 <= x < width and 0 <= y < height else None

    skipped_count = draw_visible_polyline(
        canvas, path_px, color=(0, 0, 255), thickness=line_thickness
    )

    start_visible = to_visible_pixel(start_px)
    goal_visible = to_visible_pixel(goal_px)
    if start_visible is not None:
        cv2.circle(canvas, start_visible, point_radius, (0, 255, 0), -1)
    if goal_visible is not None:
        cv2.circle(canvas, goal_visible, point_radius, (255, 0, 0), -1)

    output_path = Path(save_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(output_path), canvas):
        raise RuntimeError(f"Failed to save path overlay: {output_path}")
    return skipped_count
