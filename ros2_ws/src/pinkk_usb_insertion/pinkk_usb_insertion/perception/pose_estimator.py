"""USB 네 모서리로 카메라 기준 포트 자세를 계산한다."""

from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np

from ..geometry.transforms import validate_transform


@dataclass(frozen=True)
class PoseEstimate:
    camera_to_port: np.ndarray
    reprojection_error_px: float
    depth_m: float


def usb_port_object_points(width_m: float, height_m: float) -> np.ndarray:
    if width_m <= 0.0 or height_m <= 0.0:
        raise ValueError('USB 포트 폭과 높이는 0보다 커야 합니다')
    return np.array(
        [
            [-width_m / 2.0, -height_m / 2.0, 0.0],
            [width_m / 2.0, -height_m / 2.0, 0.0],
            [width_m / 2.0, height_m / 2.0, 0.0],
            [-width_m / 2.0, height_m / 2.0, 0.0],
        ],
        dtype=np.float64,
    )


def estimate_port_pose(
    image_points_px: np.ndarray,
    camera_matrix: np.ndarray,
    distortion_coefficients: np.ndarray,
    width_m: float,
    height_m: float,
) -> PoseEstimate:
    image_points = np.asarray(image_points_px, dtype=np.float64)
    if image_points.shape != (4, 2) or not np.all(np.isfinite(image_points)):
        raise ValueError('image_points_px는 유효한 (4, 2) 배열이어야 합니다')
    intrinsic = np.asarray(camera_matrix, dtype=np.float64).reshape(3, 3)
    distortion = np.asarray(distortion_coefficients, dtype=np.float64).reshape(-1)
    object_points = usb_port_object_points(width_m, height_m)

    success, rvec, tvec = cv2.solvePnP(
        object_points,
        image_points,
        intrinsic,
        distortion,
        flags=cv2.SOLVEPNP_IPPE,
    )
    if not success:
        raise RuntimeError('solvePnP가 자세를 계산하지 못했습니다')
    rotation, _ = cv2.Rodrigues(rvec)
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = rotation
    transform[:3, 3] = tvec.reshape(3)
    transform = validate_transform(transform)

    projected, _ = cv2.projectPoints(object_points, rvec, tvec, intrinsic, distortion)
    error = np.linalg.norm(projected.reshape(-1, 2) - image_points, axis=1)
    return PoseEstimate(
        camera_to_port=transform,
        reprojection_error_px=float(np.sqrt(np.mean(error * error))),
        depth_m=float(tvec.reshape(3)[2]),
    )
