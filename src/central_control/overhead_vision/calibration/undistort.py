"""렌즈 왜곡 보정 모듈."""

from __future__ import annotations

import cv2
import numpy as np


def undistort_frame(frame: np.ndarray, camera_matrix: np.ndarray, dist_coeffs: np.ndarray) -> np.ndarray:
    return cv2.undistort(frame, camera_matrix, dist_coeffs)
