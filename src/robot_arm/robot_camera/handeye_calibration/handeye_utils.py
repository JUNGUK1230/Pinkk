"""OpenCV Hand-Eye 계산과 고정 보드 일관성 검증."""

from dataclasses import dataclass

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

from .transform_utils import make_transform, validate_transform

METHODS = {
    "TSAI": cv2.CALIB_HAND_EYE_TSAI,
    "PARK": cv2.CALIB_HAND_EYE_PARK,
    "HORAUD": cv2.CALIB_HAND_EYE_HORAUD,
    "ANDREFF": cv2.CALIB_HAND_EYE_ANDREFF,
    "DANIILIDIS": cv2.CALIB_HAND_EYE_DANIILIDIS,
}


@dataclass
class HandEyeResult:
    """한 계산 방법의 T_flange_camera와 고정 보드 검증 통계."""

    method: str
    R_cam2flange: np.ndarray
    t_cam2flange: np.ndarray
    T_flange_camera: np.ndarray
    position_mean_mm: float
    position_max_mm: float
    rotation_mean_deg: float
    rotation_max_deg: float
    xyz_mean_m: np.ndarray
    xyz_std_m: np.ndarray


def validate_fixed_board(
    T_base_flange: np.ndarray,
    T_camera_charuco: np.ndarray,
    T_flange_camera: np.ndarray,
) -> dict[str, np.ndarray | float]:
    """각 sample의 T_base_charuco가 얼마나 일정한지 계산한다."""
    validate_transform(T_flange_camera)
    board_poses = []
    for base_flange, camera_charuco in zip(T_base_flange, T_camera_charuco, strict=True):
        validate_transform(base_flange)
        validate_transform(camera_charuco)
        board_poses.append(base_flange @ T_flange_camera @ camera_charuco)
    poses = np.asarray(board_poses)
    positions = poses[:, :3, 3]
    xyz_mean = positions.mean(axis=0)
    position_errors = np.linalg.norm(positions - xyz_mean, axis=1)
    rotations = Rotation.from_matrix(poses[:, :3, :3])
    mean_rotation = rotations.mean()
    rotation_errors = np.degrees((mean_rotation.inv() * rotations).magnitude())
    return {
        "position_mean_mm": float(position_errors.mean() * 1000.0),
        "position_max_mm": float(position_errors.max() * 1000.0),
        "rotation_mean_deg": float(rotation_errors.mean()),
        "rotation_max_deg": float(rotation_errors.max()),
        "xyz_mean_m": xyz_mean,
        "xyz_std_m": positions.std(axis=0),
    }


def calibrate_all_methods(samples: dict[str, np.ndarray]) -> tuple[list[HandEyeResult], dict[str, str]]:
    """5개 방법을 독립적으로 계산한다. 한 방법 실패가 나머지를 막지 않는다.

    OpenCV 입력 gripper2base=T_base_flange, target2cam=T_camera_charuco이며
    출력 cam2gripper가 이 프로젝트에서 원하는 T_flange_camera다.
    """
    if not hasattr(cv2, "calibrateHandEye"):
        raise RuntimeError("cv2.calibrateHandEye가 없습니다. opencv-contrib-python을 확인하세요")
    base_flange = np.asarray(samples["T_base_flange"], dtype=float)
    camera_charuco = np.asarray(samples["T_camera_charuco"], dtype=float)
    results: list[HandEyeResult] = []
    failures: dict[str, str] = {}
    for name, method in METHODS.items():
        try:
            rotation, translation = cv2.calibrateHandEye(
                list(base_flange[:, :3, :3]),
                list(base_flange[:, :3, 3]),
                list(camera_charuco[:, :3, :3]),
                list(camera_charuco[:, :3, 3]),
                method=method,
            )
            transform = make_transform(rotation, translation)
            metrics = validate_fixed_board(base_flange, camera_charuco, transform)
            results.append(
                HandEyeResult(name, rotation, translation.reshape(3, 1), transform, **metrics)
            )
        except (cv2.error, TypeError, ValueError) as error:
            failures[name] = str(error)
    return results, failures


def result_score(result: HandEyeResult) -> float:
    """자동 추천용 단순 점수: 위치 평균[mm] + 회전 평균[deg]."""
    return result.position_mean_mm + result.rotation_mean_deg
