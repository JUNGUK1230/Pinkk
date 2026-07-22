"""Path simplification and heading helpers for controller exports."""

import math
from typing import Sequence

WorldPoint = tuple[float, float]
PathRow = dict[str, int | float]


def rdp_simplify(points: Sequence[WorldPoint], epsilon: float = 3.0) -> list[WorldPoint]:
    """Simplify a 2D polyline with the Ramer-Douglas-Peucker algorithm."""
    if epsilon < 0:
        raise ValueError("epsilon must be zero or positive")
    if len(points) <= 2:
        return list(points)

    start = points[0]
    end = points[-1]
    max_distance = -1.0
    split_index = 0
    for index in range(1, len(points) - 1):
        distance = _point_to_segment_distance(points[index], start, end)
        if distance > max_distance:
            max_distance = distance
            split_index = index

    if max_distance > epsilon:
        left = rdp_simplify(points[: split_index + 1], epsilon)
        right = rdp_simplify(points[split_index:], epsilon)
        # The split point is the last of left and first of right; keep it only once.
        return left[:-1] + right
    return [start, end]


def build_path_rows(
    points: Sequence[WorldPoint], direction: int = 1
) -> list[PathRow]:
    """Attach segment heading and direction fields to world-coordinate points."""
    if not points:
        return []

    yaw_values: list[float] = []
    for index in range(len(points) - 1):
        x_cm, y_cm = points[index]
        next_x_cm, next_y_cm = points[index + 1]
        yaw_values.append(math.atan2(next_y_cm - y_cm, next_x_cm - x_cm))
    # A single point has no heading; a terminal point inherits its incoming heading.
    yaw_values.append(yaw_values[-1] if yaw_values else 0.0)

    return [
        {
            "index": index,
            "x_cm": float(x_cm),
            "y_cm": float(y_cm),
            "yaw_rad": yaw_values[index],
            "yaw_deg": math.degrees(yaw_values[index]),
            "direction": direction,
        }
        for index, (x_cm, y_cm) in enumerate(points)
    ]


def _point_to_segment_distance(
    point: WorldPoint, start: WorldPoint, end: WorldPoint
) -> float:
    """Return the Euclidean distance from a point to a finite line segment."""
    px, py = point
    sx, sy = start
    ex, ey = end
    dx, dy = ex - sx, ey - sy
    length_squared = dx * dx + dy * dy
    if length_squared == 0.0:
        return math.hypot(px - sx, py - sy)

    projection = ((px - sx) * dx + (py - sy) * dy) / length_squared
    projection = min(1.0, max(0.0, projection))
    nearest_x = sx + projection * dx
    nearest_y = sy + projection * dy
    return math.hypot(px - nearest_x, py - nearest_y)
