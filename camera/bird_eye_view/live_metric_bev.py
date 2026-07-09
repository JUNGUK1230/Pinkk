import cv2
import numpy as np
from pathlib import Path


# ============================================================
# 1. Camera 설정
# ============================================================

CAMERA_ID = 2

CAMERA_WIDTH = 1920
CAMERA_HEIGHT = 1080
FPS = 30


# ============================================================
# 2. 프로젝트 경로
# ============================================================

BEV_DIR = Path(__file__).resolve().parent
CALIBRATION_DIR = BEV_DIR.parent / "calibration"

CALIBRATION_FILE = (
    CALIBRATION_DIR
    / "camera_calibration.npz"
)

WORLD_H_FILE = (
    BEV_DIR
    / "camera_to_world_homography.npz"
)


# ============================================================
# 3. 파일 확인
# ============================================================

if not CALIBRATION_FILE.exists():

    raise FileNotFoundError(
        f"Calibration 파일 없음: "
        f"{CALIBRATION_FILE}"
    )


if not WORLD_H_FILE.exists():

    raise FileNotFoundError(
        f"Camera->World 파일 없음: "
        f"{WORLD_H_FILE}"
    )


# ============================================================
# 4. Camera Calibration 로드
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
# 5. Camera -> World Homography 로드
# ============================================================

world_data = np.load(
    WORLD_H_FILE
)

camera_to_bev_h = world_data[
    "camera_to_bev_homography"
]

world_width_cm = float(
    world_data[
        "world_width_cm"
    ]
)

world_height_cm = float(
    world_data[
        "world_height_cm"
    ]
)

bev_width = int(
    world_data[
        "bev_width"
    ]
)

bev_height = int(
    world_data[
        "bev_height"
    ]
)

pixels_per_cm_x = float(
    world_data[
        "pixels_per_cm_x"
    ]
)

pixels_per_cm_y = float(
    world_data[
        "pixels_per_cm_y"
    ]
)


# ============================================================
# 6. 로드 정보 출력
# ============================================================

print("=" * 70)
print("Metric BEV System Loaded")
print("=" * 70)

print(
    f"World Size : "
    f"{world_width_cm} x "
    f"{world_height_cm} cm"
)

print(
    f"BEV Size   : "
    f"{bev_width} x "
    f"{bev_height} px"
)

print(
    f"Scale X    : "
    f"{pixels_per_cm_x:.6f} px/cm"
)

print(
    f"Scale Y    : "
    f"{pixels_per_cm_y:.6f} px/cm"
)

print()
print("Camera -> BEV Homography:")
print(
    camera_to_bev_h
)


# ============================================================
# 7. BEV Pixel -> World cm
#
# parking_world:
#
# y ↑
#   |
#   |
#   +-------> x
#
# 원점 = 좌하단
# ============================================================

def bev_pixel_to_world_cm(
    px,
    py
):

    x_cm = (
        px
        /
        pixels_per_cm_x
    )

    y_cm = (
        bev_height - py
    ) / pixels_per_cm_y


    return (
        x_cm,
        y_cm
    )


# ============================================================
# 8. World cm -> BEV Pixel
# ============================================================

def world_cm_to_bev_pixel(
    x_cm,
    y_cm
):

    px = int(
        round(
            x_cm
            *
            pixels_per_cm_x
        )
    )

    py = int(
        round(
            bev_height
            -
            (
                y_cm
                *
                pixels_per_cm_y
            )
        )
    )


    return (
        px,
        py
    )


# ============================================================
# 9. 마지막 클릭 정보
# ============================================================

last_clicked_bev = None

last_clicked_world = None


# ============================================================
# 10. Mouse Callback
# ============================================================

def mouse_callback(
    event,
    x,
    y,
    flags,
    param
):

    global last_clicked_bev
    global last_clicked_world


    if event != cv2.EVENT_LBUTTONDOWN:
        return


    # 현재 창이 실제 1200x600 BEV 크기 그대로이므로
    # x, y 자체가 BEV pixel
    bev_x = x
    bev_y = y


    # 범위 체크
    if (
        bev_x < 0
        or
        bev_x >= bev_width
        or
        bev_y < 0
        or
        bev_y >= bev_height
    ):

        return


    world_x_cm, world_y_cm = (
        bev_pixel_to_world_cm(
            bev_x,
            bev_y
        )
    )


    last_clicked_bev = (
        bev_x,
        bev_y
    )

    last_clicked_world = (
        world_x_cm,
        world_y_cm
    )


    print()
    print("=" * 70)
    print("Clicked Position")
    print("=" * 70)

    print(
        f"BEV Pixel : "
        f"({bev_x}, {bev_y})"
    )

    print(
        f"World cm  : "
        f"({world_x_cm:.2f}, "
        f"{world_y_cm:.2f})"
    )


# ============================================================
# 11. Camera 연결
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
# 12. MJPG 설정
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
# 13. 실제 Camera 설정 확인
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
print("=" * 70)
print("Runtime Camera")
print("=" * 70)

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
# 14. Window 설정
# ============================================================

WINDOW_NAME = (
    "Live Metric BEV"
)


cv2.namedWindow(
    WINDOW_NAME
)

cv2.setMouseCallback(
    WINDOW_NAME,
    mouse_callback
)


# ============================================================
# 15. Main Loop
# ============================================================

while True:

    # --------------------------------------------------------
    # Camera frame
    # --------------------------------------------------------

    ret, frame = cap.read()


    if not ret:

        print(
            "프레임 읽기 실패"
        )

        break


    # --------------------------------------------------------
    # 1. 렌즈 왜곡 보정
    # --------------------------------------------------------

    undistorted = cv2.undistort(
        frame,
        camera_matrix,
        dist_coeffs
    )


    # --------------------------------------------------------
    # 2. Camera -> Metric BEV
    # --------------------------------------------------------

    metric_bev = cv2.warpPerspective(
        undistorted,
        camera_to_bev_h,
        (
            bev_width,
            bev_height
        )
    )


    # --------------------------------------------------------
    # 3. Debug용 복사
    # --------------------------------------------------------

    display = (
        metric_bev.copy()
    )


    # --------------------------------------------------------
    # 4. 10 cm Grid 표시
    # --------------------------------------------------------

    # X 방향
    for x_cm in range(
        0,
        int(world_width_cm) + 1,
        10
    ):

        px, _ = (
            world_cm_to_bev_pixel(
                x_cm,
                0
            )
        )


        cv2.line(
            display,
            (
                px,
                0
            ),
            (
                px,
                bev_height - 1
            ),
            (
                0,
                255,
                255
            ),
            1
        )


    # Y 방향
    for y_cm in range(
        0,
        int(world_height_cm) + 1,
        10
    ):

        _, py = (
            world_cm_to_bev_pixel(
                0,
                y_cm
            )
        )


        cv2.line(
            display,
            (
                0,
                py
            ),
            (
                bev_width - 1,
                py
            ),
            (
                0,
                255,
                255
            ),
            1
        )


    # --------------------------------------------------------
    # 5. World 좌표축 표시
    # --------------------------------------------------------

    origin_px = (
        world_cm_to_bev_pixel(
            0,
            0
        )
    )


    x_axis_end = (
        world_cm_to_bev_pixel(
            30,
            0
        )
    )


    y_axis_end = (
        world_cm_to_bev_pixel(
            0,
            30
        )
    )


    # X axis
    cv2.arrowedLine(
        display,
        origin_px,
        x_axis_end,
        (
            0,
            0,
            255
        ),
        3,
        tipLength=0.1
    )


    # Y axis
    cv2.arrowedLine(
        display,
        origin_px,
        y_axis_end,
        (
            0,
            255,
            0
        ),
        3,
        tipLength=0.1
    )


    cv2.putText(
        display,
        "X",
        (
            x_axis_end[0] - 20,
            x_axis_end[1] - 10
        ),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (
            0,
            0,
            255
        ),
        2
    )


    cv2.putText(
        display,
        "Y",
        (
            y_axis_end[0] + 10,
            y_axis_end[1] + 20
        ),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (
            0,
            255,
            0
        ),
        2
    )


    # --------------------------------------------------------
    # 6. 마지막 클릭점 표시
    # --------------------------------------------------------

    if (
        last_clicked_bev is not None
        and
        last_clicked_world is not None
    ):

        px, py = (
            last_clicked_bev
        )

        world_x_cm, world_y_cm = (
            last_clicked_world
        )


        cv2.circle(
            display,
            (
                px,
                py
            ),
            8,
            (
                0,
                0,
                255
            ),
            -1
        )


        text = (
            f"({world_x_cm:.1f}, "
            f"{world_y_cm:.1f}) cm"
        )


        text_x = min(
            px + 15,
            bev_width - 250
        )

        text_y = max(
            py - 15,
            30
        )


        cv2.putText(
            display,
            text,
            (
                text_x,
                text_y
            ),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (
                0,
                255,
                0
            ),
            2
        )


    # --------------------------------------------------------
    # 7. 상태 표시
    # --------------------------------------------------------

    cv2.putText(
        display,
        "parking_world: 200 x 100 cm",
        (
            20,
            35
        ),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.75,
        (
            255,
            255,
            0
        ),
        2
    )


    cv2.putText(
        display,
        "Click: World coordinate | Q: Quit",
        (
            20,
            70
        ),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        (
            255,
            255,
            0
        ),
        2
    )


    # --------------------------------------------------------
    # 8. Show
    # --------------------------------------------------------

    cv2.imshow(
        WINDOW_NAME,
        display
    )


    # --------------------------------------------------------
    # 9. Key
    # --------------------------------------------------------

    key = cv2.waitKey(1) & 0xFF


    if key == ord("q"):

        break


# ============================================================
# 16. 종료
# ============================================================

cap.release()

cv2.destroyAllWindows()
