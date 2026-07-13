"""카메라 실행과 Hand-Eye sample 파일 입출력."""

from pathlib import Path
from typing import Any

import cv2
import numpy as np


def camera_arg(value: str) -> int | str:
    """숫자는 카메라 index로, 나머지는 `/dev/video*` 경로로 해석한다."""
    return int(value) if value.isdigit() else value


def open_camera(device: int | str, width: int, height: int) -> cv2.VideoCapture:
    """SSH 접속 대상인 로봇 PC의 로컬 카메라를 OpenCV로 연다."""
    capture = cv2.VideoCapture(device)
    if width > 0:
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    if height > 0:
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    if not capture.isOpened():
        raise RuntimeError(f"카메라를 열 수 없습니다: {device!r}")
    return capture


def verify_resolution(capture: cv2.VideoCapture, expected: tuple[int, int] | None) -> None:
    """현재 카메라와 내부 캘리브레이션 해상도가 같은지 확인한다."""
    actual = (
        int(capture.get(cv2.CAP_PROP_FRAME_WIDTH)),
        int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT)),
    )
    print(f"카메라 해상도: {actual[0]} x {actual[1]}")
    if expected is not None and actual != expected:
        raise RuntimeError(f"현재 해상도 {actual}와 내부 캘리브레이션 해상도 {expected}가 다릅니다")


def save_samples(path: Path, samples: list[dict[str, Any]]) -> None:
    """sample을 임시 npz에 쓴 뒤 교체하여 중간 종료 시 손상을 줄인다."""
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp.npz")
    keys = (
        "sample_index", "robot_coords_raw", "T_base_flange", "R_gripper2base",
        "t_gripper2base", "T_camera_charuco", "R_target2cam", "t_target2cam",
        "reprojection_error", "detected_corner_count", "timestamp",
    )
    np.savez_compressed(
        temporary,
        **{key: np.asarray([sample[key] for sample in samples]) for key in keys},
    )
    temporary.replace(path)


def load_samples(path: Path) -> dict[str, np.ndarray]:
    """pickle 없이 sample을 읽고 필수 변환 쌍의 개수를 검사한다."""
    if not path.exists():
        raise FileNotFoundError(f"sample 파일이 없습니다: {path}")
    with np.load(path, allow_pickle=False) as data:
        samples = {key: np.asarray(data[key]) for key in data.files}
    missing = {"T_base_flange", "T_camera_charuco"}.difference(samples)
    if missing:
        raise KeyError(f"sample 파일에 필수 key가 없습니다: {sorted(missing)}")
    if len(samples["T_base_flange"]) != len(samples["T_camera_charuco"]):
        raise ValueError("로봇 pose와 카메라 pose의 개수가 다릅니다")
    return samples
