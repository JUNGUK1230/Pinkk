import cv2
import numpy as np
from pathlib import Path


# ============================================================
# 1. 기본 설정
# ============================================================

CAMERA_ID = 2

CAMERA_WIDTH = 1920
CAMERA_HEIGHT = 1080
FPS = 30


PROJECT_DIR = Path(__file__).resolve().parent


# ============================================================
# 2. 파일 경로
# ============================================================

CALIBRATION_FILE = (
    PROJECT_DIR
    / "camera_calibration.npz"
)

BEV_HOMOGRAPHY_FILE = (
    PROJECT_DIR
    / "bev_homography.npz"
)

REGISTRATION_FILE = (
    PROJECT_DIR
    / "camera_to_lidar_rigid_registration.npz"
)

LIDAR_MAP_FILE = (
    PROJECT_DIR
    / "my_test_map0710.png"
)


# ============================================================
# 3. 파일 존재 확인
# ============================================================

required_files = [
    CALIBRATION_FILE,
    BEV_HOMOGRAPHY_FILE,
    REGISTRATION_FILE,
    LIDAR_MAP_FILE
]


for file_path in required_files:

    if not file_path.exists():

        raise FileNotFoundError(
            f"파일 없음: {file_path}"
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
# 5. BEV Homography 로드
# ============================================================

bev_data = np.load(
    BEV_HOMOGRAPHY_FILE
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
# 6. Camera -> LiDAR Registration 로드
# ============================================================

registration_data = np.load(
    REGISTRATION_FILE
)

affine_matrix = registration_data[
    "affine_matrix"
].astype(
    np.float32
)


lidar_width = int(
    registration_data[
        "lidar_width"
    ]
)

lidar_height = int(
    registration_data[
        "lidar_height"
    ]
)


rotation_deg = float(
    registration_data[
        "rotation_deg"
    ]
)


rmse_cm = float(
    registration_data[
        "rmse_cm"
    ]
)


resolution = float(
    registration_data[
        "resolution"
    ]
)


origin = registration_data[
    "origin"
]


# ============================================================
# 7. LiDAR Map 로드
# ============================================================

lidar_map = cv2.imread(
    str(LIDAR_MAP_FILE),
    cv2.IMREAD_GRAYSCALE
)


if lidar_map is None:

    raise RuntimeError(
        f"LiDAR map 읽기 실패: "
        f"{LIDAR_MAP_FILE}"
    )


# ============================================================
# 8. 실제 LiDAR map 크기 확인
# ============================================================

map_height, map_width = (
    lidar_map.shape
)


if (
    map_width != lidar_width
    or
    map_height != lidar_height
):

    raise RuntimeError(
        "저장된 정합 파일의 LiDAR 크기와 "
        "현재 맵 이미지 크기가 다릅니다.\n"
        f"Registration: "
        f"{lidar_width} x {lidar_height}\n"
        f"Map Image   : "
        f"{map_width} x {map_height}"
    )


# ============================================================
# 9. 시스템 정보 출력
# ============================================================

print("=" * 70)
print("Live Camera-LiDAR Registration")
print("=" * 70)

print(
    f"Camera ID      : "
    f"{CAMERA_ID}"
)

print(
    f"Camera Input   : "
    f"{CAMERA_WIDTH} x "
    f"{CAMERA_HEIGHT}"
)

print(
    f"BEV Size       : "
    f"{bev_width} x "
    f"{bev_height}"
)

print(
    f"LiDAR Map Size : "
    f"{lidar_width} x "
    f"{lidar_height}"
)

print(
    f"Resolution     : "
    f"{resolution} m/px"
)

print(
    f"Origin         : "
    f"{origin}"
)

print(
    f"Rotation       : "
    f"{rotation_deg:.4f} deg"
)

print(
    f"Registration RMSE: "
    f"{rmse_cm:.3f} cm"
)

print()

print(
    "Affine Matrix:"
)

print(
    affine_matrix
)


# ============================================================
# 10. LiDAR 컬러 변환
# ============================================================

lidar_color = cv2.cvtColor(
    lidar_map,
    cv2.COLOR_GRAY2BGR
)


# ============================================================
# 11. 화면 표시 배율
# ============================================================

MAX_MAP_DISPLAY_WIDTH = 900
MAX_MAP_DISPLAY_HEIGHT = 750


map_display_scale = min(
    MAX_MAP_DISPLAY_WIDTH
    /
    lidar_width,

    MAX_MAP_DISPLAY_HEIGHT
    /
    lidar_height
)


map_display_width = int(
    lidar_width
    *
    map_display_scale
)

map_display_height = int(
    lidar_height
    *
    map_display_scale
)


# ============================================================
# 12. 카메라 연결
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


# ============================================================
# 13. MJPG 설정
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
# 14. 실제 카메라 설정 확인
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
# 15. 메인 루프
# ============================================================

while True:

    # --------------------------------------------------------
    # STEP 1
    # 카메라 프레임 읽기
    # --------------------------------------------------------

    ret, frame = cap.read()


    if not ret:

        print(
            "프레임 읽기 실패"
        )

        break


    # --------------------------------------------------------
    # STEP 2
    # 렌즈 왜곡 보정
    # --------------------------------------------------------

    undistorted = cv2.undistort(
        frame,
        camera_matrix,
        dist_coeffs
    )


    # --------------------------------------------------------
    # STEP 3
    # Camera BEV 생성
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
    # STEP 4
    # BEV -> LiDAR Map 좌표계 변환
    #
    # 출력 크기:
    # LiDAR map 248 x 218
    # --------------------------------------------------------

    registered_camera = cv2.warpAffine(
        bev,
        affine_matrix,
        (
            lidar_width,
            lidar_height
        )
    )


    # --------------------------------------------------------
    # STEP 5
    # Overlay
    # --------------------------------------------------------

    overlay = cv2.addWeighted(
        lidar_color,
        0.5,

        registered_camera,
        0.5,

        0
    )


    # --------------------------------------------------------
    # STEP 6
    # 표시용 확대
    # --------------------------------------------------------

    registered_display = cv2.resize(
        registered_camera,
        (
            map_display_width,
            map_display_height
        ),
        interpolation=cv2.INTER_LINEAR
    )


    overlay_display = cv2.resize(
        overlay,
        (
            map_display_width,
            map_display_height
        ),
        interpolation=cv2.INTER_LINEAR
    )


    lidar_display = cv2.resize(
        lidar_color,
        (
            map_display_width,
            map_display_height
        ),
        interpolation=cv2.INTER_NEAREST
    )


    # --------------------------------------------------------
    # STEP 7
    # 상태 텍스트
    # --------------------------------------------------------

    cv2.putText(
        overlay_display,
        (
            f"RMSE: "
            f"{rmse_cm:.2f} cm"
        ),
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
        overlay_display,
        (
            f"Rotation: "
            f"{rotation_deg:.2f} deg"
        ),
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
        overlay_display,
        "Q: Quit",
        (
            20,
            110
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
    # STEP 8
    # 화면 출력
    # --------------------------------------------------------

    cv2.imshow(
        "LiDAR Map",
        lidar_display
    )


    cv2.imshow(
        "Registered Camera",
        registered_display
    )


    cv2.imshow(
        "Live Camera-LiDAR Overlay",
        overlay_display
    )


    # --------------------------------------------------------
    # 종료
    # --------------------------------------------------------

    key = cv2.waitKey(1) & 0xFF


    if key == ord("q"):

        break


# ============================================================
# 16. 종료 처리
# ============================================================

cap.release()

cv2.destroyAllWindows()
