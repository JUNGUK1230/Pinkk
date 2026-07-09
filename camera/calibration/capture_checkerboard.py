import cv2
import os
from pathlib import Path

CAMERA_ID = 2

WIDTH = 1920
HEIGHT = 1080
FPS = 30

CAMERA_DIR = Path(__file__).resolve().parent
SAVE_DIR = CAMERA_DIR / "calibration_images"
SAVE_DIR.mkdir(parents=True, exist_ok=True)


def main():
    cap = cv2.VideoCapture(CAMERA_ID, cv2.CAP_V4L2)

    if not cap.isOpened():
        raise RuntimeError(
            f"카메라를 열 수 없습니다. CAMERA_ID={CAMERA_ID}"
        )

    # MJPG 설정
    fourcc = cv2.VideoWriter_fourcc(*"MJPG")
    cap.set(cv2.CAP_PROP_FOURCC, fourcc)

    # 해상도 / FPS
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, FPS)

    actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    actual_fps = cap.get(cv2.CAP_PROP_FPS)

    print(f"Camera ID : {CAMERA_ID}")
    print(f"Resolution: {actual_width} x {actual_height}")
    print(f"FPS       : {actual_fps}")
    print()
    print("[SPACE] 이미지 저장")
    print("[Q] 종료")

    image_count = 0

    while True:
        ret, frame = cap.read()

        if not ret:
            print("프레임 읽기 실패")
            break

        display = frame.copy()

        cv2.putText(
            display,
            f"Saved: {image_count}",
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (0, 255, 0),
            2
        )

        cv2.imshow("Checkerboard Capture", display)

        key = cv2.waitKey(1) & 0xFF

        if key == ord(" "):
            filename = SAVE_DIR / f"checker_{image_count:03d}.jpg"

            success = cv2.imwrite(str(filename), frame)

            if success:
                print(f"저장 완료: {filename}")
                image_count += 1
            else:
                print(f"저장 실패: {filename}")

        elif key == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
