"""4×4 homogeneous transform을 일관된 단위와 방향으로 계산한다."""

from __future__ import annotations

from collections.abc import Sequence
import math

import numpy as np


def normalize_quaternion(values: Sequence[float]) -> np.ndarray:
    quaternion = np.asarray(values, dtype=np.float64).reshape(4)
    if not np.all(np.isfinite(quaternion)):
        raise ValueError('quaternion에 NaN 또는 inf가 있습니다')
    norm = float(np.linalg.norm(quaternion))
    if norm < 1e-12:
        raise ValueError('길이가 0인 quaternion입니다')
    return quaternion / norm


def quaternion_to_rotation(values: Sequence[float]) -> np.ndarray:
    x, y, z, w = normalize_quaternion(values)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def rotation_to_quaternion(rotation: np.ndarray) -> np.ndarray:
    matrix = np.asarray(rotation, dtype=np.float64).reshape(3, 3)
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = 2.0 * np.sqrt(trace + 1.0)
        quaternion = np.array(
            [
                (matrix[2, 1] - matrix[1, 2]) / scale,
                (matrix[0, 2] - matrix[2, 0]) / scale,
                (matrix[1, 0] - matrix[0, 1]) / scale,
                0.25 * scale,
            ]
        )
    else:
        index = int(np.argmax(np.diag(matrix)))
        if index == 0:
            scale = 2.0 * np.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2])
            quaternion = np.array(
                [0.25 * scale, (matrix[0, 1] + matrix[1, 0]) / scale,
                 (matrix[0, 2] + matrix[2, 0]) / scale,
                 (matrix[2, 1] - matrix[1, 2]) / scale]
            )
        elif index == 1:
            scale = 2.0 * np.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2])
            quaternion = np.array(
                [(matrix[0, 1] + matrix[1, 0]) / scale, 0.25 * scale,
                 (matrix[1, 2] + matrix[2, 1]) / scale,
                 (matrix[0, 2] - matrix[2, 0]) / scale]
            )
        else:
            scale = 2.0 * np.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1])
            quaternion = np.array(
                [(matrix[0, 2] + matrix[2, 0]) / scale,
                 (matrix[1, 2] + matrix[2, 1]) / scale, 0.25 * scale,
                 (matrix[1, 0] - matrix[0, 1]) / scale]
            )
    return normalize_quaternion(quaternion)


def rotation_to_rpy_degrees(rotation: np.ndarray) -> tuple[float, float, float]:
    """회전행렬을 intrinsic ZYX Roll/Pitch/Yaw degree로 변환한다."""
    matrix = np.asarray(rotation, dtype=np.float64).reshape(3, 3)
    if not np.all(np.isfinite(matrix)):
        raise ValueError('회전행렬에 NaN 또는 inf가 있습니다')
    pitch = math.asin(float(np.clip(-matrix[2, 0], -1.0, 1.0)))
    if abs(math.cos(pitch)) < 1e-8:
        roll = math.atan2(-float(matrix[0, 1]), float(matrix[1, 1]))
        yaw = 0.0
    else:
        roll = math.atan2(float(matrix[2, 1]), float(matrix[2, 2]))
        yaw = math.atan2(float(matrix[1, 0]), float(matrix[0, 0]))
    return tuple(
        (math.degrees(value) + 180.0) % 360.0 - 180.0
        for value in (roll, pitch, yaw)
    )


def rpy_degrees_to_rotation(
    roll_deg: float,
    pitch_deg: float,
    yaw_deg: float,
) -> np.ndarray:
    """intrinsic ZYX Roll/Pitch/Yaw degree를 회전행렬로 변환한다."""
    values = np.asarray(
        (roll_deg, pitch_deg, yaw_deg),
        dtype=np.float64,
    )
    if not np.all(np.isfinite(values)):
        raise ValueError('RPY에 NaN 또는 inf가 있습니다')
    roll, pitch, yaw = np.radians(values)
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array(
        (
            (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
            (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
            (-sp, cp * sr, cp * cr),
        ),
        dtype=np.float64,
    )


def make_transform(translation: Sequence[float], quaternion_xyzw: Sequence[float]) -> np.ndarray:
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = quaternion_to_rotation(quaternion_xyzw)
    transform[:3, 3] = np.asarray(translation, dtype=np.float64).reshape(3)
    return validate_transform(transform)


def validate_transform(transform: np.ndarray) -> np.ndarray:
    matrix = np.asarray(transform, dtype=np.float64).reshape(4, 4)
    if not np.all(np.isfinite(matrix)):
        raise ValueError('transform에 NaN 또는 inf가 있습니다')
    if not np.allclose(matrix[3], [0.0, 0.0, 0.0, 1.0], atol=1e-9):
        raise ValueError('homogeneous transform의 마지막 행이 잘못됐습니다')
    rotation = matrix[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6):
        raise ValueError('회전행렬이 직교행렬이 아닙니다')
    if not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-6):
        raise ValueError('회전행렬 determinant가 1이 아닙니다')
    return matrix.copy()


def compose(*transforms: np.ndarray) -> np.ndarray:
    result = np.eye(4, dtype=np.float64)
    for transform in transforms:
        result = result @ validate_transform(transform)
    return validate_transform(result)


def inverse(transform: np.ndarray) -> np.ndarray:
    matrix = validate_transform(transform)
    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = matrix[:3, :3].T
    result[:3, 3] = -result[:3, :3] @ matrix[:3, 3]
    return result


def approach_transform(standoff_m: float) -> np.ndarray:
    if not np.isfinite(standoff_m) or standoff_m <= 0.0:
        raise ValueError('standoff_m은 0보다 큰 유한값이어야 합니다')
    result = np.eye(4, dtype=np.float64)
    result[2, 3] = -float(standoff_m)
    return result
