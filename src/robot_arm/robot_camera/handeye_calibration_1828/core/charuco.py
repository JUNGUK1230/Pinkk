"""ChArUco 보드 검출과 T_camera_charuco pose 추정."""

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import cv2
import numpy as np

from ..config import settings as config
from .transforms import make_transform


@dataclass
class CharucoPose:
    """카메라 한 frame에서 얻은 ChArUco pose 결과."""

    success: bool
    detected_corner_count: int = 0
    reprojection_error: float = float("inf")
    R_target2cam: np.ndarray | None = None
    t_target2cam: np.ndarray | None = None
    T_camera_charuco: np.ndarray | None = None
    corners: np.ndarray | None = None
    ids: np.ndarray | None = None


def require_opencv_features() -> None:
    """ArUco와 Hand-Eye에 필요한 OpenCV 기능 존재 여부를 검사한다."""
    if not hasattr(cv2, "aruco"):
        raise RuntimeError("cv2.aruco가 없습니다. opencv-contrib-python을 설치하세요")


def load_intrinsics(path: Path) -> tuple[np.ndarray, np.ndarray, tuple[int, int] | None]:
    """내부 파라미터 npz를 읽고 필수 key 및 값의 유효성을 검사한다."""
    if not path.exists():
        raise FileNotFoundError(f"내부 캘리브레이션 파일이 없습니다: {path}")
    with np.load(path, allow_pickle=False) as data:
        missing = {"camera_matrix", "dist_coeffs"}.difference(data.files)
        if missing:
            raise KeyError(f"필수 key {sorted(missing)}가 없습니다. 현재 key: {data.files}")
        camera_matrix = np.asarray(data["camera_matrix"], dtype=float)
        dist_coeffs = np.asarray(data["dist_coeffs"], dtype=float)
        size = None
        if {"image_width", "image_height"}.issubset(data.files):
            size = (int(data["image_width"]), int(data["image_height"]))
    if camera_matrix.shape != (3, 3) or not np.isfinite(camera_matrix).all():
        raise ValueError("camera_matrix가 유효한 3x3 행렬이 아닙니다")
    if not np.isfinite(dist_coeffs).all():
        raise ValueError("dist_coeffs에 NaN 또는 inf가 있습니다")
    return camera_matrix, dist_coeffs, size


def create_board_and_detector() -> tuple[Any, Any]:
    """config 값으로 calib.io ChArUco 보드와 detector를 만든다."""
    require_opencv_features()

    try:
        dictionary_id = getattr(
            cv2.aruco,
            config.ARUCO_DICTIONARY_NAME,
        )
    except AttributeError as error:
        raise RuntimeError(
            f"지원하지 않는 ArUco dictionary: "
            f"{config.ARUCO_DICTIONARY_NAME}"
        ) from error

    dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)

    board = cv2.aruco.CharucoBoard(
        (
            config.CHARUCO_SQUARES_X,
            config.CHARUCO_SQUARES_Y,
        ),
        config.SQUARE_LENGTH_M,
        config.MARKER_LENGTH_M,
        dictionary,
    )

    # calib.io에서 생성한 짝수 행 보드는 OpenCV의 legacy 배치를 사용한다.
    if config.CHARUCO_LEGACY_PATTERN:
        if not hasattr(board, "setLegacyPattern"):
            raise RuntimeError(
                "현재 OpenCV가 CharucoBoard.setLegacyPattern()을 "
                "지원하지 않습니다"
            )
        board.setLegacyPattern(True)

    detector = cv2.aruco.CharucoDetector(board)
    return board, detector

def estimate_charuco_pose(
    frame: np.ndarray,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    board: Any,
    detector: Any,
) -> CharucoPose:
    """검출 코너로 solvePnP를 수행해 T_camera_charuco를 반환한다.

    solvePnP의 object-to-camera 출력은 보드 기준 점을 camera 기준으로 바꾸므로
    OpenCV calibrateHandEye의 target2cam 입력에 역변환 없이 그대로 사용한다.
    """
    if frame is None or frame.ndim not in (2, 3):
        raise ValueError("올바른 카메라 frame이 아닙니다")
    corners, ids, _, _ = detector.detectBoard(frame)
    count = 0 if ids is None else int(len(ids))
    result = CharucoPose(False, count, corners=corners, ids=ids)
    if count < config.MIN_CHARUCO_CORNERS:
        return result
    object_points, image_points = board.matchImagePoints(corners, ids)
    success, rvec, tvec = cv2.solvePnP(
        object_points, image_points, camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_ITERATIVE
    )
    if not success:
        return result
    rotation, _ = cv2.Rodrigues(rvec)
    projected, _ = cv2.projectPoints(object_points, rvec, tvec, camera_matrix, dist_coeffs)
    residual = projected.reshape(-1, 2) - np.asarray(image_points).reshape(-1, 2)
    error = float(np.sqrt(np.mean(np.sum(residual * residual, axis=1))))
    if not np.isfinite(error):
        return result
    result.success = True
    result.reprojection_error = error
    result.R_target2cam = rotation
    result.t_target2cam = np.asarray(tvec, dtype=float).reshape(3, 1)
    result.T_camera_charuco = make_transform(rotation, result.t_target2cam)
    return result
