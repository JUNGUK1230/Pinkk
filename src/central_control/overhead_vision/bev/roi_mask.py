"""Polygon ROI Mask 모듈."""

from __future__ import annotations

import cv2
import numpy as np


def apply_polygon_mask(image: np.ndarray, polygon_points: np.ndarray) -> np.ndarray:
    mask = np.zeros(image.shape[:2], dtype=np.uint8)
    points = np.asarray(polygon_points, dtype=np.int32)
    cv2.fillPoly(mask, [points], 255)
    return cv2.bitwise_and(image, image, mask=mask)
