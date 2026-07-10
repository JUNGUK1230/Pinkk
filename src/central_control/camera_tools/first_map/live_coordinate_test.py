import cv2
import numpy as np
from pathlib import Path

from coordinate_transformer import CoordinateTransformer


# ============================================================
# 기본 설정
# ============================================================

CAMERA_ID = 2

CAMERA_WIDTH = 1920
CAMERA_HEIGHT = 1080
FPS = 30


PROJECT_DIR = Path(
    "/home/junguk/pinkk/src/central_control/camera_tools/first_map"
)


CALIBRATION_FILE = (
    PROJECT_DIR
    / "camera_calibration.npz"
)

BEV_FILE = (
    PROJECT_DIR
    / "bev_homography.npz"
)

REGISTRATION_FILE = (
    PROJECT_DIR
    / "camera_to_lidar_rigid_registration.npz"
)


# ============================================================
# Calibration 로드
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
# BEV 로드
# ============================================================

bev_data = np.load(
    BEV_FILE
)

bev_homography = bev_data[
    "homography_matrix"
]

bev_width = int(
    bev_data["bev_width"]
)

bev_height = int(
    bev_data["bev_height"]
)


# ============================================================
# Coordinate Transformer
# ============================================================

transformer = CoordinateTransformer(
    REGISTRATION_FILE
)


transformer.print_info()


# ============================================================
# 표시 크기
# ============================================================

DISPLAY_WIDTH = 1200
DISPLAY_HEIGHT = 600


display_scale_x = (
    DISPLAY_WIDTH
    /
    bev_width
)

display_scale_y = (
    DISPLAY_HEIGHT
    /
    bev_height
)


# ============================================================
# 마지막 클릭 정보
# ============================================================

last_bev_point = None
last_lidar_point = None
last_map_point = None


# ============================================================
# Mouse callback
# ============================================================

def mouse_callback(
    event,
    x,
    y,
    flags,
    param
):

    global last_bev_point
    global last_lidar_point
    global last_map_point


    if event != cv2.EVENT_LBUTTONDOWN:
        return


    # --------------------------------------------------------
    # 표시 좌표 -> 원래 1600x800 BEV
    # --------------------------------------------------------

    bev_x = (
        x
        /
        display_scale_x
    )

    bev_y = (
        y
        /
        display_scale_y
    )


    # --------------------------------------------------------
    # BEV -> LiDAR pixel
    # --------------------------------------------------------

    lidar_x, lidar_y = (
        transformer.bev_to_lidar_pixel(
            bev_x,
            bev_y
        )
    )


    # --------------------------------------------------------
    # LiDAR pixel -> ROS /map
    # --------------------------------------------------------

    map_x, map_y = (
        transformer.lidar_pixel_to_map(
            lidar_x,
            lidar_y
        )
    )


    # --------------------------------------------------------
    # 저장
    # --------------------------------------------------------

    last_bev_point = (
        bev_x,
        bev_y
    )

    last_lidar_point = (
        lidar_x,
        lidar_y
    )

    last_map_point = (
        map_x,
        map_y
    )


    # --------------------------------------------------------
    # 터미널 출력
    # --------------------------------------------------------

    print()
    print("=" * 70)
    print("Clicked Coordinate")
    print("=" * 70)

    print(
        f"Display Pixel : "
        f"({x}, {y})"
    )

    print(
        f"BEV Pixel     : "
        f"({bev_x:.2f}, "
        f"{bev_y:.2f})"
    )

    print(
        f"LiDAR Pixel   : "
        f"({lidar_x:.2f}, "
        f"{lidar_y:.2f})"
    )

    print(
        f"ROS /map      : "
        f"({map_x:.4f}, "
        f"{map_y:.4f}) m"
    )

    print(
        f"Inside map    : "
        f"{transformer.is_inside_lidar_map(
            lidar_x,
            lidar_y
        )}"
    )


# ============================================================
# Camera 연결
# ============================================================

cap = cv2.VideoCapture(
    CAMERA_ID,
    cv2.CAP_V4L2
)


if not cap.isOpened():

    raise RuntimeError(
        f"카메라 {CAMERA_ID}번 "
        f"열기 실패"
    )


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
# Window
# ============================================================

WINDOW_NAME = (
    "BEV Coordinate Test"
)


cv2.namedWindow(
    WINDOW_NAME
)


cv2.setMouseCallback(
    WINDOW_NAME,
    mouse_callback
)


# ============================================================
# Main loop
# ============================================================

while True:

    ret, frame = cap.read()


    if not ret:

        print(
            "프레임 읽기 실패"
        )

        break


    # --------------------------------------------------------
    # 왜곡 보정
    # --------------------------------------------------------

    undistorted = cv2.undistort(
        frame,
        camera_matrix,
        dist_coeffs
    )


    # --------------------------------------------------------
    # 1600x800 BEV
    # --------------------------------------------------------

    bev = cv2.warpPerspective(
        undistorted,
        bev_homography,
        (
            bev_width,
            bev_height
        )
    )


    # --------------------------------------------------------
    # 표시용 1200x600
    # --------------------------------------------------------

    display = cv2.resize(
        bev,
        (
            DISPLAY_WIDTH,
            DISPLAY_HEIGHT
        )
    )


    # --------------------------------------------------------
    # 클릭점 표시
    # --------------------------------------------------------

    if last_bev_point is not None:

        bev_x, bev_y = (
            last_bev_point
        )

        map_x, map_y = (
            last_map_point
        )


        display_x = int(
            bev_x
            *
            display_scale_x
        )

        display_y = int(
            bev_y
            *
            display_scale_y
        )


        cv2.circle(
            display,
            (
                display_x,
                display_y
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
            f"/map=({map_x:.3f}, "
            f"{map_y:.3f})m"
        )


        text_x = max(
            10,
            min(
                display_x + 15,
                DISPLAY_WIDTH - 350
            )
        )

        text_y = max(
            30,
            display_y - 15
        )


        cv2.putText(
            display,
            text,
            (
                text_x,
                text_y
            ),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (
                0,
                255,
                255
            ),
            2
        )


    # --------------------------------------------------------
    # 안내
    # --------------------------------------------------------

    cv2.putText(
        display,
        "Click BEV point -> ROS /map",
        (
            20,
            40
        ),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (
            0,
            255,
            255
        ),
        2
    )


    cv2.putText(
        display,
        "Q: Quit",
        (
            20,
            75
        ),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (
            0,
            255,
            255
        ),
        2
    )


    cv2.imshow(
        WINDOW_NAME,
        display
    )


    key = cv2.waitKey(1) & 0xFF


    if key == ord("q"):

        break


# ============================================================
# 종료
# ============================================================

cap.release()

cv2.destroyAllWindows()