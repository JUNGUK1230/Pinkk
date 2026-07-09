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

DISPLAY_WIDTH = 1280
DISPLAY_HEIGHT = 720

BEV_DIR = Path(__file__).resolve().parent
CALIBRATION_DIR = BEV_DIR.parent / "calibration"

CALIBRATION_FILE = Path(
    CALIBRATION_DIR / "camera_calibration.npz"
)

HOMOGRAPHY_FILE = Path(
    BEV_DIR / "bev_homography.npz"
)


# ============================================================
# 실제 기준 직사각형 크기
# ============================================================

RECT_WIDTH_CM = 200.0
RECT_HEIGHT_CM = 100.0

# 실제 비율 2:1 유지
BEV_WIDTH = 1600
BEV_HEIGHT = 800


# ============================================================
# 파일 존재 확인
# ============================================================

if not CALIBRATION_FILE.exists():
    raise FileNotFoundError(
        f"캘리브레이션 파일이 없습니다: "
        f"{CALIBRATION_FILE}"
    )


# ============================================================
# 캘리브레이션 데이터 로드
# ============================================================

data = np.load(
    CALIBRATION_FILE
)

camera_matrix = data["camera_matrix"]
dist_coeffs = data["dist_coeffs"]


print("=" * 60)
print("Calibration Data Loaded")
print("=" * 60)

print(f"Calibration File: {CALIBRATION_FILE}")

print()
print("Camera Matrix:")
print(camera_matrix)

print()
print("Distortion Coefficients:")
print(dist_coeffs)

print()
print("=" * 60)
print("Reference Rectangle")
print("=" * 60)

print(
    f"Real Size : "
    f"{RECT_WIDTH_CM} cm × "
    f"{RECT_HEIGHT_CM} cm"
)

print(
    f"BEV Size  : "
    f"{BEV_WIDTH} px × "
    f"{BEV_HEIGHT} px"
)

print(
    f"Scale     : "
    f"{BEV_WIDTH / RECT_WIDTH_CM:.2f} px/cm"
)


# ============================================================
# 전역 변수
# ============================================================

clicked_points = []

frozen_frame = None

is_frozen = False

homography_matrix = None

bev_size = None


# ============================================================
# Homography 계산
# ============================================================

def calculate_homography():
    global homography_matrix
    global bev_size

    if len(clicked_points) != 4:
        print(
            "[ERROR] 4개 점이 필요합니다."
        )
        return

    # 실제 기준 직사각형 비율 사용
    bev_width = BEV_WIDTH
    bev_height = BEV_HEIGHT

    bev_size = (
        bev_width,
        bev_height
    )

    print()
    print("=" * 60)
    print("BEV Size")
    print("=" * 60)

    print(
        f"Width : {bev_width} px"
    )

    print(
        f"Height: {bev_height} px"
    )

    print(
        "Real Ratio: 200:100 = 2:1"
    )

    # --------------------------------------------------------
    # 원본 영상의 4개 점
    #
    # 순서:
    # TL -> TR -> BR -> BL
    # --------------------------------------------------------

    src_points = np.float32(
        clicked_points
    )

    # --------------------------------------------------------
    # 목적지 좌표
    # 1600 × 800
    # --------------------------------------------------------

    dst_points = np.float32([
        [0, 0],

        [
            bev_width - 1,
            0
        ],

        [
            bev_width - 1,
            bev_height - 1
        ],

        [
            0,
            bev_height - 1
        ]
    ])

    # --------------------------------------------------------
    # Homography 계산
    # --------------------------------------------------------

    homography_matrix = (
        cv2.getPerspectiveTransform(
            src_points,
            dst_points
        )
    )

    print()
    print("=" * 60)
    print("Homography Matrix")
    print("=" * 60)

    print(
        homography_matrix
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
    global clicked_points

    if event != cv2.EVENT_LBUTTONDOWN:
        return

    # 프레임 고정 상태에서만 클릭 허용
    if not is_frozen:
        print(
            "[WARNING] 먼저 S 키를 눌러 "
            "화면을 고정하세요."
        )
        return

    # 최대 4점
    if len(clicked_points) >= 4:
        print(
            "[WARNING] 이미 4개 점이 "
            "선택되었습니다."
        )
        return

    # --------------------------------------------------------
    # 표시 좌표 → 원본 1920x1080 좌표 변환
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

    original_x = int(
        x * scale_x
    )

    original_y = int(
        y * scale_y
    )

    clicked_points.append(
        (
            original_x,
            original_y
        )
    )

    point_number = len(
        clicked_points
    )

    print()
    print(
        f"Point {point_number}"
    )

    print(
        f"Display Coordinate: "
        f"({x}, {y})"
    )

    print(
        f"Original Coordinate: "
        f"({original_x}, {original_y})"
    )

    # 4개 선택 완료 시 Homography 계산
    if len(clicked_points) == 4:

        print()
        print(
            "4개 점 선택 완료"
        )

        calculate_homography()


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
    f"Camera ID : {CAMERA_ID}"
)

print(
    f"Resolution: "
    f"{actual_width} x {actual_height}"
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
# OpenCV 창
# ============================================================

cv2.namedWindow(
    "Undistorted"
)

cv2.setMouseCallback(
    "Undistorted",
    mouse_callback
)


# ============================================================
# 메인 루프
# ============================================================

while True:

    # --------------------------------------------------------
    # LIVE 모드
    # --------------------------------------------------------

    if not is_frozen:

        ret, frame = cap.read()

        if not ret:
            print(
                "프레임 읽기 실패"
            )
            break

        # ----------------------------------------------------
        # 왜곡 보정
        # ----------------------------------------------------

        undistorted = cv2.undistort(
            frame,
            camera_matrix,
            dist_coeffs
        )

        current_frame = (
            undistorted
        )

    # --------------------------------------------------------
    # FROZEN 모드
    # --------------------------------------------------------

    else:

        current_frame = (
            frozen_frame.copy()
        )


    # ========================================================
    # 표시용 영상 축소
    # ========================================================

    display = cv2.resize(
        current_frame,
        (
            DISPLAY_WIDTH,
            DISPLAY_HEIGHT
        )
    )


    # ========================================================
    # 클릭점 표시용 스케일
    # ========================================================

    scale_x = (
        DISPLAY_WIDTH
        /
        CAMERA_WIDTH
    )

    scale_y = (
        DISPLAY_HEIGHT
        /
        CAMERA_HEIGHT
    )


    # ========================================================
    # 클릭 포인트 표시
    # ========================================================

    display_points = []

    for i, point in enumerate(
        clicked_points
    ):

        original_x, original_y = point

        display_x = int(
            original_x * scale_x
        )

        display_y = int(
            original_y * scale_y
        )

        display_points.append(
            (
                display_x,
                display_y
            )
        )

        # 포인트
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

        # 번호
        cv2.putText(
            display,
            str(i + 1),
            (
                display_x + 12,
                display_y - 12
            ),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 0),
            2
        )


    # ========================================================
    # 클릭점 연결선
    # ========================================================

    if len(display_points) >= 2:

        for i in range(
            len(display_points) - 1
        ):

            cv2.line(
                display,
                display_points[i],
                display_points[i + 1],
                (255, 0, 0),
                2
            )


    # 4개 선택 시 마지막과 첫 점 연결
    if len(display_points) == 4:

        cv2.line(
            display,
            display_points[3],
            display_points[0],
            (255, 0, 0),
            2
        )


    # ========================================================
    # BEV 생성
    # ========================================================

    if (
        homography_matrix is not None
        and
        bev_size is not None
    ):

        bev_width, bev_height = (
            bev_size
        )

        bev = cv2.warpPerspective(
            current_frame,
            homography_matrix,
            (
                bev_width,
                bev_height
            )
        )

        # ----------------------------------------------------
        # BEV 표시 크기
        # 실제 결과는 1600x800 유지
        # 화면에는 1200x600으로 축소
        # ----------------------------------------------------

        bev_display = cv2.resize(
            bev,
            (
                1200,
                600
            )
        )

        cv2.imshow(
            "BEV",
            bev_display
        )


    # ========================================================
    # 상태 문구
    # ========================================================

    if is_frozen:

        status_text = (
            "FROZEN - Click: TL -> TR -> BR -> BL"
        )

    else:

        status_text = (
            "LIVE - Press S to freeze"
        )


    cv2.putText(
        display,
        status_text,
        (20, 40),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (0, 255, 255),
        2
    )


    cv2.putText(
        display,
        "S: Freeze  R: Reset  W: Save H  Q: Quit",
        (20, 75),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (0, 255, 255),
        2
    )


    # ========================================================
    # 화면 출력
    # ========================================================

    cv2.imshow(
        "Undistorted",
        display
    )


    # ========================================================
    # 키 입력
    # ========================================================

    key = cv2.waitKey(1) & 0xFF


    # Q: 종료
    if key == ord("q"):
        break


    # S: 현재 프레임 고정
    elif key == ord("s"):

        if not is_frozen:

            frozen_frame = (
                current_frame.copy()
            )

            is_frozen = True

            print()
            print("=" * 60)
            print("Frame Frozen")
            print("=" * 60)

            print(
                "클릭 순서:"
            )

            print(
                "1. TL 좌상단"
            )

            print(
                "2. TR 우상단"
            )

            print(
                "3. BR 우하단"
            )

            print(
                "4. BL 좌하단"
            )


    # R: 전체 초기화
    elif key == ord("r"):

        clicked_points.clear()

        frozen_frame = None

        is_frozen = False

        homography_matrix = None

        bev_size = None

        try:
            cv2.destroyWindow(
                "BEV"
            )

        except cv2.error:
            pass

        print()
        print(
            "Reset 완료"
        )


    # W: Homography 저장
    elif key == ord("w"):

        if (
            homography_matrix is None
            or
            bev_size is None
        ):

            print(
                "[WARNING] 먼저 4개 점을 "
                "선택하세요."
            )

        else:

            HOMOGRAPHY_FILE.parent.mkdir(
                parents=True,
                exist_ok=True
            )

            np.savez(
                HOMOGRAPHY_FILE,

                homography_matrix=(
                    homography_matrix
                ),

                source_points=np.array(
                    clicked_points,
                    dtype=np.float32
                ),

                bev_width=bev_size[0],

                bev_height=bev_size[1],

                rect_width_cm=(
                    RECT_WIDTH_CM
                ),

                rect_height_cm=(
                    RECT_HEIGHT_CM
                ),

                pixels_per_cm=(
                    BEV_WIDTH
                    /
                    RECT_WIDTH_CM
                )
            )

            print()
            print("=" * 60)
            print("Homography 저장 완료")
            print("=" * 60)

            print(
                f"파일: "
                f"{HOMOGRAPHY_FILE}"
            )

            print(
                f"Scale: "
                f"{BEV_WIDTH / RECT_WIDTH_CM:.2f} px/cm"
            )


# ============================================================
# 종료 처리
# ============================================================

cap.release()
cv2.destroyAllWindows()
