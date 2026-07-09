"""Homography 기반 Bird's-Eye View 변환 모듈."""

from __future__ import annotations

import cv2
import numpy as np


def transform_to_bev(frame: np.ndarray, homography: np.ndarray, output_size: tuple[int, int]) -> np.ndarray:
    width, height = output_size
    return cv2.warpPerspective(frame, homography, (width, height))
