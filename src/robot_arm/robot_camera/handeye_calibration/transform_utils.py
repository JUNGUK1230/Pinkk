"""로봇 pose와 동차변환행렬 처리 함수."""

from collections.abc import Sequence

import numpy as np
from scipy.spatial.transform import Rotation

from . import config


def validate_rotation_matrix(matrix: np.ndarray, atol: float = 1e-6) -> None:
    """3x3 행렬이 유효한 SO(3) 회전행렬인지 검사한다."""
    matrix = np.asarray(matrix, dtype=float)
    if matrix.shape != (3, 3) or not np.isfinite(matrix).all():
        raise ValueError("회전행렬은 유한한 3x3 행렬이어야 합니다")
    if not np.allclose(matrix.T @ matrix, np.eye(3), atol=atol):
        raise ValueError("회전행렬이 정규직교 행렬이 아닙니다")
    if not np.isclose(np.linalg.det(matrix), 1.0, atol=atol):
        raise ValueError("회전행렬 determinant가 +1이 아닙니다")


def make_transform(rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    """R_B2A와 t_B2A로 p_A=T_A_B@p_B를 만족하는 4x4 행렬을 만든다."""
    rotation = np.asarray(rotation, dtype=float)
    translation = np.asarray(translation, dtype=float).reshape(-1)
    validate_rotation_matrix(rotation)
    if translation.shape != (3,) or not np.isfinite(translation).all():
        raise ValueError("이동 벡터는 유한한 값 3개여야 합니다")
    transform = np.eye(4, dtype=float)
    transform[:3, :3] = rotation
    transform[:3, 3] = translation
    return transform


def validate_transform(transform: np.ndarray) -> None:
    """4x4 강체 동차변환행렬의 shape, 유한값, 마지막 행과 회전을 검사한다."""
    transform = np.asarray(transform, dtype=float)
    if transform.shape != (4, 4) or not np.isfinite(transform).all():
        raise ValueError("변환행렬은 유한한 4x4 행렬이어야 합니다")
    if not np.allclose(transform[3], [0, 0, 0, 1], atol=1e-8):
        raise ValueError("변환행렬 마지막 행이 [0, 0, 0, 1]이 아닙니다")
    validate_rotation_matrix(transform[:3, :3])


def robot_coords_to_T_base_flange(
    coords: Sequence[float],
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """[x,y,z,rx,ry,rz]를 T_base_flange로 변환한다.

    입력 위치 단위는 mm, 각도는 degree이며 출력 translation 단위는 meter다.
    Elephant Robotics 공식 정의에 따라 intrinsic ZYX와 각도 [rz, ry, rx]를 사용한다.
    """
    if not config.ROBOT_EULER_CONVENTION_VERIFIED or not config.ROBOT_EULER_SEQUENCE:
        raise RuntimeError("ROBOT EULER CONVENTION NOT VERIFIED: config.py에서 검증 후 활성화하세요")
    values = np.asarray(coords, dtype=float).reshape(-1)
    if values.shape != (6,) or not np.isfinite(values).all():
        raise ValueError("get_coords()는 유한한 숫자 6개를 반환해야 합니다")
    rx_deg, ry_deg, rz_deg = values[3:]
    rotation = Rotation.from_euler(
        config.ROBOT_EULER_SEQUENCE,
        [rz_deg, ry_deg, rx_deg],
        degrees=True,
    ).as_matrix()
    translation = values[:3] / 1000.0
    return make_transform(rotation, translation), rotation, translation.reshape(3, 1)


def pose_difference(first: np.ndarray, second: np.ndarray) -> tuple[float, float]:
    """두 pose의 이동 거리[m]와 상대 회전각[deg]을 반환한다."""
    validate_transform(first)
    validate_transform(second)
    distance = float(np.linalg.norm(first[:3, 3] - second[:3, 3]))
    relative = first[:3, :3].T @ second[:3, :3]
    angle = float(np.degrees(Rotation.from_matrix(relative).magnitude()))
    return distance, angle
