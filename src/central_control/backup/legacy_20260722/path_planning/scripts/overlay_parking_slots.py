"""Draw parking-slot BEV coordinates on a Camera BEV image."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import numpy as np


CORNER_KEYS = (
    "top_left_bev",
    "top_right_bev",
    "bottom_right_bev",
    "bottom_left_bev",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("coordinates", type=Path, help="Parking-slot JSON file")
    parser.add_argument("image", type=Path, help="1600x800 Camera BEV image")
    parser.add_argument("output", type=Path, help="Output overlay image")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    with args.coordinates.open(encoding="utf-8") as file:
        parking_slots = json.load(file)

    image = cv2.imread(str(args.image), cv2.IMREAD_COLOR)
    if image is None:
        raise RuntimeError(f"Failed to read image: {args.image}")

    overlay = image.copy()
    polygons: list[tuple[str, np.ndarray, tuple[int, int]]] = []
    height, width = image.shape[:2]

    for name, slot in parking_slots.items():
        points = np.rint([slot[key] for key in CORNER_KEYS]).astype(np.int32)
        center = tuple(np.rint(slot["center_bev"]).astype(int))
        if np.any(points[:, 0] < 0) or np.any(points[:, 0] >= width):
            raise ValueError(f"{name} has an x coordinate outside the image")
        if np.any(points[:, 1] < 0) or np.any(points[:, 1] >= height):
            raise ValueError(f"{name} has a y coordinate outside the image")
        cv2.fillPoly(overlay, [points], (0, 210, 255))
        polygons.append((name, points, center))

    result = cv2.addWeighted(overlay, 0.28, image, 0.72, 0.0)
    for name, points, center in polygons:
        cv2.polylines(result, [points], True, (0, 255, 255), 3, cv2.LINE_AA)
        cv2.circle(result, center, 5, (0, 0, 255), -1, cv2.LINE_AA)
        label = name.replace("parking_", "P")
        cv2.putText(
            result,
            label,
            (center[0] - 18, center[1] + 7),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (20, 20, 20),
            3,
            cv2.LINE_AA,
        )
        cv2.putText(
            result,
            label,
            (center[0] - 18, center[1] + 7),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )

    args.output.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(args.output), result):
        raise RuntimeError(f"Failed to save image: {args.output}")
    print(f"Saved {len(polygons)} parking slots to {args.output}")


if __name__ == "__main__":
    main()
