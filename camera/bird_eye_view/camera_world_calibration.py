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
# 2. 화면 표시 크기
# ============================================================

DISPLAY_WIDTH = 1280
DISPLAY_HEIGHT = 720


# ============================================================
# 3. 실제 parking_world 크기
# ============================================================

WORLD_WIDTH_CM = 200.0
WORLD_HEIGHT_CM = 100.0


# ============================================================
# 4. Metric BEV 크기
#
# 200 cm -> 1200 px
# 100 cm ->  600 px
#
# 1 cm = 6 px
# ============================================================

BEV_WIDTH = 1200
BEV_HEIGHT = 600


PIXELS_PER_CM_X = (
    BEV_WIDTH / WORLD_WIDTH_CM
)

PIXELS_PER_CM_Y = (
    BEV_HEIGHT / WORLD_HEIGHT_CM
)


# ============================================================
# 5. 파일 경로
# ============================================================

BEV_DIR = Path(__file__).resolve().parent
CALIBRATION_DIR = BEV_DIR.parent / "calibration"

CALIBRATION_FILE = (
    CALIBRATION_DIR
    / "camera_calibration.npz"
)

OUTPUT_FILE = (
    BEV_DIR
    / "camera_to_world_homography.npz"
)

PREVIEW_FILE = (
    BEV_DIR
    / "camera_world_bev_preview.png"
)


# ============================================================
# 6. 실제 스티커 World 좌표
#
# 원점:
# 주차장 좌하단 = (0, 0)
#
# 클릭 순서:
# A -> B -> C -> D
# ============================================================

WORLD_POINTS_CM = np.float32([
    [30.0, 30.0],    # A
    [170.0, 30.0],   # B
    [170.0, 70.0],   # C
    [30.0, 70.0]     # D
])


WORLD_POINT_NAMES = [
    "A (30,30)",
    "B (170,30)",
    "C (170,70)",
    "D (30,70)"
]


# ============================================================
# 7. 파일 확인
# ============================================================

if not CALIBRATION_FILE.exists():

    raise FileNotFoundError(
        f"캘리브레이션 파일이 없습니다: "
        f"{CALIBRATION_FILE}"
    )


# ============================================================
# 8. Camera Calibration 로드
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


print("=" * 70)
print("Camera Calibration Loaded")
print("=" * 70)

print(
    f"File: {CALIBRATION_FILE}"
)

print()

print("Camera Matrix:")
print(camera_matrix)

print()

print("Distortion Coefficients:")
print(dist_coeffs)


# ============================================================
# 9. World 정보 출력
# ============================================================

print()
print("=" * 70)
print("parking_world")
print("=" * 70)

print(
    f"World Size : "
    f"{WORLD_WIDTH_CM} x "
    f"{WORLD_HEIGHT_CM} cm"
)

print(
    f"BEV Size   : "
    f"{BEV_WIDTH} x "
    f"{BEV_HEIGHT} px"
)

print(
    f"Scale X    : "
    f"{PIXELS_PER_CM_X:.3f} px/cm"
)

print(
    f"Scale Y    : "
    f"{PIXELS_PER_CM_Y:.3f} px/cm"
)

print()

print("Sticker World Coordinates:")

for name, point in zip(
    WORLD_POINT_NAMES,
    WORLD_POINTS_CM
):
    print(
        f"{name}: "
        f"{point}"
    )


# ============================================================
# 10. 전역 상태
# ============================================================

clicked_camera_points = []

is_frozen = False

frozen_frame = None

camera_to_bev_h = None

last_metric_bev = None


# ============================================================
# 11. World cm -> BEV pixel
#
# World:
#
# y ↑
#   |
#   |
#   +------> x
#
# Image:
#
# +------> x
# |
# |
# v y
#
# 따라서 y축 반전
# ============================================================

def world_cm_to_bev_pixel(
    x_cm,
    y_cm
):
    px = (
        x_cm
        *
        PIXELS_PER_CM_X
    )

    py = (
        BEV_HEIGHT
        -
        (
            y_cm
            *
            PIXELS_PER_CM_Y
        )
    )

    return (
        px,
        py
    )


# ============================================================
# 12. BEV pixel -> World cm
# ============================================================

def bev_pixel_to_world_cm(
    px,
    py
):
    x_cm = (
        px
        /
        PIXELS_PER_CM_X
    )

    y_cm = (
        BEV_HEIGHT - py
    ) / PIXELS_PER_CM_Y

    return (
        x_cm,
        y_cm
    )


# ============================================================
# 13. 목적지 BEV 좌표 계산
# ============================================================

BEV_DESTINATION_POINTS = np.float32([
    world_cm_to_bev_pixel(
        x_cm,
        y_cm
    )
    for x_cm, y_cm
    in WORLD_POINTS_CM
])


print()
print("=" * 70)
print("Destination BEV Pixels")
print("=" * 70)

for name, point in zip(
    WORLD_POINT_NAMES,
    BEV_DESTINATION_POINTS
):
    print(
        f"{name} -> "
        f"({point[0]:.1f}, "
        f"{point[1]:.1f})"
    )


# ============================================================
# 14. Homography 계산
# ============================================================

def calculate_world_homography():

    global camera_to_bev_h


    if len(clicked_camera_points) != 4:

        print(
            "[ERROR] 정확히 4개의 "
            "Camera 점이 필요합니다."
        )

        return False


    source_points = np.float32(
        clicked_camera_points
    )


    camera_to_bev_h = (
        cv2.getPerspectiveTransform(
            source_points,
            BEV_DESTINATION_POINTS
        )
    )


    print()
    print("=" * 70)
    print("Camera -> parking_world Homography")
    print("=" * 70)

    print(
        camera_to_bev_h
    )


    return True


# ============================================================
# 15. 마우스 콜백
# ============================================================

def mouse_callback(
    event,
    x,
    y,
    flags,
    param
):

    if event != cv2.EVENT_LBUTTONDOWN:
        return


    if not is_frozen:

        print(
            "[WARNING] 먼저 S 키를 눌러 "
            "영상을 고정하세요."
        )

        return


    if len(clicked_camera_points) >= 4:

        print(
            "[WARNING] 이미 4개 점을 "
            "선택했습니다."
        )

        return


    # --------------------------------------------------------
    # 표시 좌표 -> 원본 1920x1080 좌표
    # --------------------------------------------------------

    scale_x = (
        CAMERA_WIDTH
        /
        DISPLAY_WIDTH
    )

    scale_y = (
        CAMERA_HEIGHT
        /
        DISPLAY_HEIGHT
    )


    original_x = (
        x * scale_x
    )

    original_y = (
        y * scale_y
    )


    clicked_camera_points.append(
        [
            original_x,
            original_y
        ]
    )


    index = (
        len(clicked_camera_points)
        - 1
    )


    print()
    print("=" * 70)

    print(
        f"Point {index + 1}: "
        f"{WORLD_POINT_NAMES[index]}"
    )

    print("=" * 70)

    print(
        f"Display Pixel : "
        f"({x}, {y})"
    )

    print(
        f"Camera Pixel  : "
        f"({original_x:.2f}, "
        f"{original_y:.2f})"
    )

    print(
        f"World Target  : "
        f"({WORLD_POINTS_CM[index, 0]:.1f}, "
        f"{WORLD_POINTS_CM[index, 1]:.1f}) cm"
    )


    # 4개 완료
    if len(clicked_camera_points) == 4:

        print()
        print(
            "4개 World 대응점 선택 완료"
        )

        calculate_world_homography()


# ============================================================
# 16. Camera 연결
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


# MJPG
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
# 17. 실제 Camera 설정 확인
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
    f"Camera ID : {CAMERA_ID}"
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
# 18. Window
# ============================================================

WINDOW_NAME = "Undistorted World Calibration"


cv2.namedWindow(
    WINDOW_NAME
)

cv2.setMouseCallback(
    WINDOW_NAME,
    mouse_callback
)


# ============================================================
# 19. 사용법 출력
# ============================================================

print()
print("=" * 70)
print("Controls")
print("=" * 70)

print(
    "S : Freeze frame"
)

print(
    "Mouse Left : Click A -> B -> C -> D"
)

print(
    "U : Undo last point"
)

print(
    "R : Reset all"
)

print(
    "W : Save calibration"
)

print(
    "Q : Quit"
)

print()

print(
    "IMPORTANT CLICK ORDER:"
)

print(
    "1. A = (30, 30)"
)

print(
    "2. B = (170, 30)"
)

print(
    "3. C = (170, 70)"
)

print(
    "4. D = (30, 70)"
)


# ============================================================
# 20. Main Loop
# ============================================================

while True:

    # --------------------------------------------------------
    # LIVE
    # --------------------------------------------------------

    if not is_frozen:

        ret, frame = cap.read()


        if not ret:

            print(
                "프레임 읽기 실패"
            )

            break


        # -----------------------------------------------
        # 왜곡 보정
        # -----------------------------------------------

        undistorted = cv2.undistort(
            frame,
            camera_matrix,
            dist_coeffs
        )


        current_frame = (
            undistorted
        )


    # --------------------------------------------------------
    # FROZEN
    # --------------------------------------------------------

    else:

        current_frame = (
            frozen_frame.copy()
        )


    # ========================================================
    # 표시 영상
    # ========================================================

    display = cv2.resize(
        current_frame,
        (
            DISPLAY_WIDTH,
            DISPLAY_HEIGHT
        )
    )


    # ========================================================
    # 클릭점 표시
    # ========================================================

    for i, point in enumerate(
        clicked_camera_points
    ):

        ox, oy = point


        dx = int(
            ox
            *
            DISPLAY_WIDTH
            /
            CAMERA_WIDTH
        )

        dy = int(
            oy
            *
            DISPLAY_HEIGHT
            /
            CAMERA_HEIGHT
        )


        cv2.circle(
            display,
            (
                dx,
                dy
            ),
            8,
            (
                0,
                0,
                255
            ),
            -1
        )


        label = (
            f"{i + 1}: "
            f"{WORLD_POINT_NAMES[i]}"
        )


        cv2.putText(
            display,
            label,
            (
                dx + 12,
                dy - 12
            ),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (
                0,
                255,
                255
            ),
            2
        )


    # ========================================================
    # Metric BEV 생성
    # ========================================================

    if camera_to_bev_h is not None:

        metric_bev = cv2.warpPerspective(
            current_frame,
            camera_to_bev_h,
            (
                BEV_WIDTH,
                BEV_HEIGHT
            )
        )


        last_metric_bev = (
            metric_bev.copy()
        )


        # -----------------------------------------------
        # World grid 표시
        # 10 cm 간격
        # -----------------------------------------------

        bev_debug = (
            metric_bev.copy()
        )


        # 세로선
        for x_cm in range(
            0,
            201,
            10
        ):

            px = int(
                x_cm
                *
                PIXELS_PER_CM_X
            )


            cv2.line(
                bev_debug,
                (
                    px,
                    0
                ),
                (
                    px,
                    BEV_HEIGHT - 1
                ),
                (
                    0,
                    255,
                    255
                ),
                1
            )


        # 가로선
        for y_cm in range(
            0,
            101,
            10
        ):

            py = int(
                BEV_HEIGHT
                -
                (
                    y_cm
                    *
                    PIXELS_PER_CM_Y
                )
            )


            cv2.line(
                bev_debug,
                (
                    0,
                    py
                ),
                (
                    BEV_WIDTH - 1,
                    py
                ),
                (
                    0,
                    255,
                    255
                ),
                1
            )


        # -----------------------------------------------
        # 스티커 목표 위치 표시
        # -----------------------------------------------

        for i, point in enumerate(
            BEV_DESTINATION_POINTS
        ):

            px = int(
                round(
                    point[0]
                )
            )

            py = int(
                round(
                    point[1]
                )
            )


            cv2.circle(
                bev_debug,
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


            cv2.putText(
                bev_debug,
                WORLD_POINT_NAMES[i],
                (
                    px + 10,
                    py - 10
                ),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (
                    0,
                    255,
                    0
                ),
                2
            )


        cv2.imshow(
            "Metric BEV 1200x600",
            bev_debug
        )


    # ========================================================
    # 상태 표시
    # ========================================================

    if is_frozen:

        state_text = (
            "FROZEN - Click A -> B -> C -> D"
        )

    else:

        state_text = (
            "LIVE - Press S to freeze"
        )


    cv2.putText(
        display,
        state_text,
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


    next_index = len(
        clicked_camera_points
    )


    if next_index < 4:

        next_text = (
            f"NEXT: "
            f"{WORLD_POINT_NAMES[next_index]}"
        )

    else:

        next_text = (
            "4 points complete - Press W to save"
        )


    cv2.putText(
        display,
        next_text,
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


    cv2.putText(
        display,
        "S:Freeze U:Undo R:Reset W:Save Q:Quit",
        (
            20,
            110
        ),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
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


    # ========================================================
    # Key
    # ========================================================

    key = cv2.waitKey(1) & 0xFF


    # --------------------------------------------------------
    # Q
    # --------------------------------------------------------

    if key == ord("q"):

        break


    # --------------------------------------------------------
    # S: Freeze
    # --------------------------------------------------------

    elif key == ord("s"):

        if not is_frozen:

            frozen_frame = (
                current_frame.copy()
            )

            is_frozen = True


            print()
            print("=" * 70)
            print("Frame Frozen")
            print("=" * 70)

            print(
                "이제 A -> B -> C -> D "
                "순서로 클릭하세요."
            )


    # --------------------------------------------------------
    # U: Undo
    # --------------------------------------------------------

    elif key == ord("u"):

        if len(
            clicked_camera_points
        ) > 0:

            removed = (
                clicked_camera_points.pop()
            )


            camera_to_bev_h = None


            print()
            print(
                f"Removed: {removed}"
            )

            print(
                f"Next: "
                f"{WORLD_POINT_NAMES[
                    len(clicked_camera_points)
                ]}"
            )


    # --------------------------------------------------------
    # R: Reset
    # --------------------------------------------------------

    elif key == ord("r"):

        clicked_camera_points.clear()

        is_frozen = False

        frozen_frame = None

        camera_to_bev_h = None

        last_metric_bev = None


        try:

            cv2.destroyWindow(
                "Metric BEV 1200x600"
            )

        except cv2.error:

            pass


        print()
        print(
            "전체 초기화 완료"
        )


    # --------------------------------------------------------
    # W: Save
    # --------------------------------------------------------

    elif key == ord("w"):

        if camera_to_bev_h is None:

            print(
                "[WARNING] 먼저 4개 점을 "
                "선택하세요."
            )

            continue


        OUTPUT_FILE.parent.mkdir(
            parents=True,
            exist_ok=True
        )


        np.savez(
            OUTPUT_FILE,

            camera_to_bev_homography=(
                camera_to_bev_h
            ),

            camera_points=np.array(
                clicked_camera_points,
                dtype=np.float32
            ),

            world_points_cm=(
                WORLD_POINTS_CM
            ),

            bev_destination_points=(
                BEV_DESTINATION_POINTS
            ),

            world_width_cm=(
                WORLD_WIDTH_CM
            ),

            world_height_cm=(
                WORLD_HEIGHT_CM
            ),

            bev_width=(
                BEV_WIDTH
            ),

            bev_height=(
                BEV_HEIGHT
            ),

            pixels_per_cm_x=(
                PIXELS_PER_CM_X
            ),

            pixels_per_cm_y=(
                PIXELS_PER_CM_Y
            )
        )


        if last_metric_bev is not None:

            cv2.imwrite(
                str(PREVIEW_FILE),
                last_metric_bev
            )


        print()
        print("=" * 70)
        print("Camera -> parking_world Saved")
        print("=" * 70)

        print(
            f"Calibration: "
            f"{OUTPUT_FILE}"
        )

        print(
            f"Preview    : "
            f"{PREVIEW_FILE}"
        )

        print()

        print(
            "World Frame:"
        )

        print(
            "Origin = bottom-left"
        )

        print(
            "X = 0 ~ 200 cm"
        )

        print(
            "Y = 0 ~ 100 cm"
        )

        print(
            "Scale = 6 px/cm"
        )


# ============================================================
# 종료
# ============================================================

cap.release()

cv2.destroyAllWindows()
