import cv2
import numpy as np
from pathlib import Path


# ============================================================
# 기본 설정
# ============================================================

CAMERA_ID = 2

CAMERA_WIDTH = 1920
CAMERA_HEIGHT = 1080
FPS = 30


# ============================================================
# 파일 경로
# ============================================================

BEV_DIR = Path(__file__).resolve().parent
CALIBRATION_DIR = BEV_DIR.parent / "calibration"

CALIBRATION_FILE = CALIBRATION_DIR / "camera_calibration.npz"

HOMOGRAPHY_FILE = Path(
    BEV_DIR / "bev_homography.npz"
)


# ============================================================
# 파일 존재 확인
# ============================================================

if not CALIBRATION_FILE.exists():
    raise FileNotFoundError(
        f"캘리브레이션 파일이 없습니다: "
        f"{CALIBRATION_FILE}"
    )

if not HOMOGRAPHY_FILE.exists():
    raise FileNotFoundError(
        f"Homography 파일이 없습니다: "
        f"{HOMOGRAPHY_FILE}"
    )


# ============================================================
# Calibration 데이터 로드
# ============================================================

calib_data = np.load(
    CALIBRATION_FILE
)

camera_matrix = calib_data[
    "camera_matrix"
]

dist_coeffs = calib_data[
    "dist_coeffs"
]


# ============================================================
# Homography 데이터 로드
# ============================================================

bev_data = np.load(
    HOMOGRAPHY_FILE
)

homography_matrix = bev_data[
    "homography_matrix"
]

bev_width = int(
    bev_data["bev_width"]
)

bev_height = int(
    bev_data["bev_height"]
)


# ============================================================
# 실제 크기 정보 로드
# ============================================================

if "rect_width_cm" not in bev_data.files:
    raise KeyError(
        "bev_homography.npz 안에 "
        "'rect_width_cm' 값이 없습니다."
    )

if "rect_height_cm" not in bev_data.files:
    raise KeyError(
        "bev_homography.npz 안에 "
        "'rect_height_cm' 값이 없습니다."
    )


rect_width_cm = float(
    bev_data["rect_width_cm"]
)

rect_height_cm = float(
    bev_data["rect_height_cm"]
)


# ============================================================
# X/Y 스케일 계산
#
# 굳이 하나의 pixels_per_cm 값만 믿지 않고
# X축, Y축 각각 계산
# ============================================================

pixels_per_cm_x = (
    bev_width
    /
    rect_width_cm
)

pixels_per_cm_y = (
    bev_height
    /
    rect_height_cm
)


print("=" * 60)
print("BEV Coordinate System")
print("=" * 60)

print(
    f"BEV Size       : "
    f"{bev_width} x {bev_height} px"
)

print(
    f"Real Size      : "
    f"{rect_width_cm} x "
    f"{rect_height_cm} cm"
)

print(
    f"Scale X        : "
    f"{pixels_per_cm_x:.4f} px/cm"
)

print(
    f"Scale Y        : "
    f"{pixels_per_cm_y:.4f} px/cm"
)


# ============================================================
# 화면 표시 크기
#
# 실제 BEV는 1600x800
# 화면에는 1200x600 정도로 표시
# ============================================================

MAX_DISPLAY_WIDTH = 1200
MAX_DISPLAY_HEIGHT = 700


display_scale = min(
    MAX_DISPLAY_WIDTH / bev_width,
    MAX_DISPLAY_HEIGHT / bev_height,
    1.0
)


display_width = int(
    bev_width * display_scale
)

display_height = int(
    bev_height * display_scale
)


print(
    f"Display Size   : "
    f"{display_width} x "
    f"{display_height}"
)

print(
    f"Display Scale  : "
    f"{display_scale:.4f}"
)


# ============================================================
# 클릭 정보 저장
# ============================================================

last_clicked_pixel = None

last_clicked_world = None


# ============================================================
# BEV 픽셀 → 실제 cm 좌표
#
# 좌표계:
#
# 실제 맵:
#
# y ↑
#   |
#   |
#   +------→ x
# (0,0)
#
# 즉 좌하단이 원점
# ============================================================

def bev_pixel_to_world_cm(
    px,
    py
):
    # X축
    x_cm = (
        px
        /
        pixels_per_cm_x
    )

    # 이미지 Y축은 아래로 증가하므로 뒤집음
    y_cm = (
        bev_height - py
    ) / pixels_per_cm_y

    return (
        x_cm,
        y_cm
    )


# ============================================================
# 실제 cm 좌표 → BEV 픽셀
# ============================================================

def world_cm_to_bev_pixel(
    x_cm,
    y_cm
):
    px = int(
        x_cm
        *
        pixels_per_cm_x
    )

    py = int(
        bev_height
        -
        (
            y_cm
            *
            pixels_per_cm_y
        )
    )

    return (
        px,
        py
    )


# ============================================================
# 마우스 콜백
# ============================================================

def mouse_callback(
    event,
    x,
    y,
    flags,
    param
):
    global last_clicked_pixel
    global last_clicked_world


    if event != cv2.EVENT_LBUTTONDOWN:
        return


    # --------------------------------------------------------
    # 화면 표시 좌표 → 실제 BEV 좌표
    # --------------------------------------------------------

    bev_x = int(
        x / display_scale
    )

    bev_y = int(
        y / display_scale
    )


    # 범위 제한
    bev_x = max(
        0,
        min(
            bev_x,
            bev_width - 1
        )
    )

    bev_y = max(
        0,
        min(
            bev_y,
            bev_height - 1
        )
    )


    # --------------------------------------------------------
    # BEV pixel → 실제 cm
    # --------------------------------------------------------

    world_x_cm, world_y_cm = (
        bev_pixel_to_world_cm(
            bev_x,
            bev_y
        )
    )


    # 저장
    last_clicked_pixel = (
        bev_x,
        bev_y
    )

    last_clicked_world = (
        world_x_cm,
        world_y_cm
    )


    # --------------------------------------------------------
    # 터미널 출력
    # --------------------------------------------------------

    print()
    print("=" * 60)
    print("Clicked Coordinate")
    print("=" * 60)

    print(
        f"Display Pixel : "
        f"({x}, {y})"
    )

    print(
        f"BEV Pixel     : "
        f"({bev_x}, {bev_y})"
    )

    print(
        f"World cm      : "
        f"({world_x_cm:.2f}, "
        f"{world_y_cm:.2f})"
    )


# ============================================================
# 카메라 연결
# ============================================================

cap = cv2.VideoCapture(
    CAMERA_ID,
    cv2.CAP_V4L2
)

if not cap.isOpened():
    raise RuntimeError(
        f"카메라 {CAMERA_ID}번을 "
        f"열 수 없습니다."
    )


# ============================================================
# MJPG 설정
# ============================================================

fourcc = cv2.VideoWriter_fourcc(
    *"MJPG"
)

cap.set(
    cv2.CAP_PROP_FOURCC,
    fourcc
)

cap.set(
    cv2.CAP_PROP_FRAME_WIDTH,
    CAMERA_WIDTH
)

cap.set(
    cv2.CAP_PROP_FRAME_HEIGHT,
    CAMERA_HEIGHT
)

cap.set(
    cv2.CAP_PROP_FPS,
    FPS
)


# ============================================================
# 실제 카메라 설정 확인
# ============================================================

actual_width = int(
    cap.get(
        cv2.CAP_PROP_FRAME_WIDTH
    )
)

actual_height = int(
    cap.get(
        cv2.CAP_PROP_FRAME_HEIGHT
    )
)

actual_fps = cap.get(
    cv2.CAP_PROP_FPS
)


print()
print("=" * 60)
print("Runtime Camera")
print("=" * 60)

print(
    f"Camera ID : "
    f"{CAMERA_ID}"
)

print(
    f"Resolution: "
    f"{actual_width} x "
    f"{actual_height}"
)

print(
    f"FPS       : "
    f"{actual_fps}"
)


# ============================================================
# 해상도 검사
# ============================================================

if (
    actual_width != CAMERA_WIDTH
    or
    actual_height != CAMERA_HEIGHT
):
    cap.release()

    raise RuntimeError(
        "카메라가 1920x1080으로 "
        "열리지 않았습니다."
    )


# ============================================================
# OpenCV Window
# ============================================================

cv2.namedWindow(
    "BEV Coordinate Test"
)

cv2.setMouseCallback(
    "BEV Coordinate Test",
    mouse_callback
)


# ============================================================
# 메인 루프
# ============================================================

while True:

    # --------------------------------------------------------
    # 1. 카메라 프레임 읽기
    # --------------------------------------------------------

    ret, frame = cap.read()

    if not ret:
        print(
            "프레임 읽기 실패"
        )
        break


    # --------------------------------------------------------
    # 2. 렌즈 왜곡 보정
    # --------------------------------------------------------

    undistorted = cv2.undistort(
        frame,
        camera_matrix,
        dist_coeffs
    )


    # --------------------------------------------------------
    # 3. Homography 적용
    # --------------------------------------------------------

    bev = cv2.warpPerspective(
        undistorted,
        homography_matrix,
        (
            bev_width,
            bev_height
        )
    )


    # --------------------------------------------------------
    # 4. 화면 표시용 축소
    # --------------------------------------------------------

    display = cv2.resize(
        bev,
        (
            display_width,
            display_height
        )
    )


    # --------------------------------------------------------
    # 5. 좌표축 안내
    # --------------------------------------------------------

    cv2.putText(
        display,
        "Click anywhere on BEV",
        (20, 40),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (0, 255, 255),
        2
    )

    cv2.putText(
        display,
        "Origin: bottom-left (0, 0) cm",
        (20, 75),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (0, 255, 255),
        2
    )

    cv2.putText(
        display,
        "Q: Quit",
        (20, 110),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (0, 255, 255),
        2
    )


    # --------------------------------------------------------
    # 6. 마지막 클릭점 표시
    # --------------------------------------------------------

    if (
        last_clicked_pixel is not None
        and
        last_clicked_world is not None
    ):

        bev_x, bev_y = (
            last_clicked_pixel
        )

        world_x_cm, world_y_cm = (
            last_clicked_world
        )


        # BEV 좌표 → 표시 좌표
        display_x = int(
            bev_x
            *
            display_scale
        )

        display_y = int(
            bev_y
            *
            display_scale
        )


        # 점 표시
        cv2.circle(
            display,
            (
                display_x,
                display_y
            ),
            8,
            (0, 0, 255),
            -1
        )


        # 좌표 문자열
        coordinate_text = (
            f"px=({bev_x},{bev_y}) "
            f"cm=({world_x_cm:.1f},"
            f"{world_y_cm:.1f})"
        )


        # 텍스트 위치
        text_x = min(
            display_x + 15,
            display_width - 350
        )

        text_y = max(
            display_y - 15,
            30
        )


        cv2.putText(
            display,
            coordinate_text,
            (
                text_x,
                text_y
            ),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 255, 0),
            2
        )


    # --------------------------------------------------------
    # 7. 화면 출력
    # --------------------------------------------------------

    cv2.imshow(
        "BEV Coordinate Test",
        display
    )


    # --------------------------------------------------------
    # 8. 키 입력
    # --------------------------------------------------------

    key = cv2.waitKey(1) & 0xFF

    if key == ord("q"):
        break


# ============================================================
# 종료 처리
# ============================================================

cap.release()

cv2.destroyAllWindows()
