import cv2
import numpy as np
from pathlib import Path

CAMERA_ID = 2

WIDTH = 1920
HEIGHT = 1080
FPS = 30


# ============================================================
# 캘리브레이션 데이터 로드
# ============================================================

CAMERA_DIR = Path(__file__).resolve().parent
data = np.load(CAMERA_DIR / "camera_calibration.npz")

camera_matrix = data["camera_matrix"]
dist_coeffs = data["dist_coeffs"]

calib_width = int(data["image_width"])
calib_height = int(data["image_height"])


print("=" * 60)
print("CALIBRATION INFO")
print("=" * 60)

print(f"Calibration Resolution: {calib_width} x {calib_height}")

print("\nCamera Matrix:")
print(camera_matrix)

print("\nDistortion Coefficients:")
print(dist_coeffs)


# ============================================================
# 카메라 연결
# ============================================================

cap = cv2.VideoCapture(
    CAMERA_ID,
    cv2.CAP_V4L2
)

if not cap.isOpened():
    raise RuntimeError(
        f"카메라 {CAMERA_ID}번을 열 수 없습니다."
    )


# MJPG 설정
fourcc = cv2.VideoWriter_fourcc(*"MJPG")

cap.set(cv2.CAP_PROP_FOURCC, fourcc)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
cap.set(cv2.CAP_PROP_FPS, FPS)


# ============================================================
# 실제 카메라 설정 확인
# ============================================================

actual_width = int(
    cap.get(cv2.CAP_PROP_FRAME_WIDTH)
)

actual_height = int(
    cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
)

actual_fps = cap.get(
    cv2.CAP_PROP_FPS
)


print("\n" + "=" * 60)
print("RUNTIME CAMERA INFO")
print("=" * 60)

print(
    f"Actual Resolution: "
    f"{actual_width} x {actual_height}"
)

print(
    f"Actual FPS: "
    f"{actual_fps}"
)


# ============================================================
# 해상도 검사
# ============================================================

if (
    calib_width != actual_width
    or calib_height != actual_height
):
    print("\n[ERROR]")
    print("캘리브레이션 해상도와 현재 카메라 해상도가 다릅니다.")

    print(
        f"Calibration: "
        f"{calib_width} x {calib_height}"
    )

    print(
        f"Runtime: "
        f"{actual_width} x {actual_height}"
    )

    cap.release()

    raise RuntimeError(
        "해상도가 다르므로 왜곡 보정을 중단합니다."
    )


# ============================================================
# 실시간 테스트
# ============================================================

while True:

    ret, frame = cap.read()

    if not ret:
        print("프레임 읽기 실패")
        break


    # --------------------------------------------------------
    # 가장 기본적인 왜곡 보정
    # new_camera_matrix 사용하지 않음
    # --------------------------------------------------------

    undistorted = cv2.undistort(
        frame,
        camera_matrix,
        dist_coeffs
    )


    # --------------------------------------------------------
    # 표시용 축소
    # --------------------------------------------------------

    original_display = cv2.resize(
        frame,
        (960, 540)
    )

    undistorted_display = cv2.resize(
        undistorted,
        (960, 540)
    )


    # 텍스트 표시
    cv2.putText(
        original_display,
        "ORIGINAL",
        (20, 40),
        cv2.FONT_HERSHEY_SIMPLEX,
        1.0,
        (0, 0, 255),
        2
    )

    cv2.putText(
        undistorted_display,
        "UNDISTORTED",
        (20, 40),
        cv2.FONT_HERSHEY_SIMPLEX,
        1.0,
        (0, 255, 0),
        2
    )


    cv2.imshow(
        "Original",
        original_display
    )

    cv2.imshow(
        "Undistorted",
        undistorted_display
    )


    key = cv2.waitKey(1) & 0xFF

    if key == ord("q"):
        break


cap.release()
cv2.destroyAllWindows()
