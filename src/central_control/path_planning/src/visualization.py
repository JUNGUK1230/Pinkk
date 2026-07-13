"""OpenCV helpers for visualizing occupancy-grid planning results."""

from pathlib import Path
from typing import Sequence

import cv2
import numpy as np

GridPoint = tuple[int, int]


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
