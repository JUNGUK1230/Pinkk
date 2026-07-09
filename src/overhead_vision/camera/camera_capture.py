"""USB 상단 카메라 영상 입력 모듈."""

from __future__ import annotations

from pathlib import Path

import cv2
import yaml


def load_camera_config(config_path: str | Path) -> dict:
    path = Path(config_path)
    if not path.exists():
        raise FileNotFoundError(f"카메라 설정 파일이 없습니다: {path}")

    with path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f)

    if "camera" not in data:
        raise ValueError("설정 파일에 'camera' 항목이 없습니다.")

    return data["camera"]


def open_camera(config: dict) -> cv2.VideoCapture:
    device_id = int(config.get("device_id", 0))
    cap = cv2.VideoCapture(device_id, cv2.CAP_V4L2)

    fourcc = str(config.get("fourcc", "MJPG"))
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(config.get("width", 1920)))
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(config.get("height", 1080)))
    cap.set(cv2.CAP_PROP_FPS, int(config.get("fps", 30)))

    if not cap.isOpened():
        cap.release()
        raise RuntimeError(f"카메라를 열 수 없습니다. device_id={device_id}")

    return cap


def main() -> None:
    config = load_camera_config("config/camera/camera.yaml")
    cap = open_camera(config)

    print("'q' 키를 누르면 종료됩니다.")

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("프레임 취득 실패")
                break

            cv2.imshow("Overhead Camera", frame)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
