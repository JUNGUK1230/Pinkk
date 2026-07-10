import cv2
import numpy as np
import yaml
from pathlib import Path


# ============================================================
# 1. 기본 설정
# ============================================================

CAMERA_ID = 2

CAMERA_WIDTH = 1920
CAMERA_HEIGHT = 1080
FPS = 30


PROJECT_DIR = Path(
    "/home/junguk/project"
)


CALIBRATION_FILE = (
    PROJECT_DIR
    / "camera_calibration.npz"
)

HOMOGRAPHY_FILE = (
    PROJECT_DIR
    / "bev_homography.npz"
)

# 새 YAML
YAML_FILE = (
    PROJECT_DIR
    / "my_test_map0710.yaml"
)

OUTPUT_FILE = (
    PROJECT_DIR
    / "camera_to_lidar_rigid_registration.npz"
)

OVERLAY_FILE = (
    PROJECT_DIR
    / "camera_lidar_rigid_overlay.png"
)


# ============================================================
# 2. Camera BEV가 표현하는 실제 영역
#
# 현재 프로젝트 전체 맵:
# 200 cm × 100 cm
# ============================================================

REAL_WIDTH_CM = 200.0
REAL_HEIGHT_CM = 100.0


# ============================================================
# 3. 파일 확인
# ============================================================

for file_path in [
    CALIBRATION_FILE,
    HOMOGRAPHY_FILE,
    YAML_FILE
]:
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
    HOMOGRAPHY_FILE
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


print("=" * 70)
print("Camera BEV")
print("=" * 70)

print(
    f"BEV Size: "
    f"{bev_width} x {bev_height}"
)


# ============================================================
# 6. Camera BEV 스케일 계산
#
# 예:
# 1200 px / 200 cm = 6 px/cm
# 600 px / 100 cm  = 6 px/cm
# ============================================================

camera_px_per_cm_x = (
    bev_width
    /
    REAL_WIDTH_CM
)

camera_px_per_cm_y = (
    bev_height
    /
    REAL_HEIGHT_CM
)


print(
    f"Camera X scale: "
    f"{camera_px_per_cm_x:.6f} px/cm"
)

print(
    f"Camera Y scale: "
    f"{camera_px_per_cm_y:.6f} px/cm"
)


# ============================================================
# 7. 새 LiDAR YAML 로드
# ============================================================

with open(
    YAML_FILE,
    "r",
    encoding="utf-8"
) as f:

    map_info = yaml.safe_load(f)


image_name = map_info["image"]

resolution_m_per_px = float(
    map_info["resolution"]
)

origin = np.array(
    map_info["origin"],
    dtype=np.float64
)


negate = int(
    map_info.get(
        "negate",
        0
    )
)

occupied_thresh = float(
    map_info.get(
        "occupied_thresh",
        0.65
    )
)

free_thresh = float(
    map_info.get(
        "free_thresh",
        0.196
    )
)


# ============================================================
# 8. PGM 경로
#
# YAML:
# image: my_test_map0710.pgm
# ============================================================

PGM_FILE = (
    YAML_FILE.parent
    /
    image_name
).resolve()


if not PGM_FILE.exists():

    raise FileNotFoundError(
        f"PGM 파일 없음: {PGM_FILE}"
    )


# ============================================================
# 9. LiDAR PGM 로드
# ============================================================

lidar_map = cv2.imread(
    str(PGM_FILE),
    cv2.IMREAD_GRAYSCALE
)


if lidar_map is None:

    raise RuntimeError(
        f"PGM 읽기 실패: {PGM_FILE}"
    )


lidar_height, lidar_width = (
    lidar_map.shape
)


# ============================================================
# 10. LiDAR resolution 변환
#
# 0.01 m/px
# =
# 1 cm/px
# ============================================================

lidar_cm_per_px = (
    resolution_m_per_px
    *
    100.0
)


print()
print("=" * 70)
print("LiDAR Map")
print("=" * 70)

print(
    f"PGM File    : "
    f"{PGM_FILE}"
)

print(
    f"Size        : "
    f"{lidar_width} x "
    f"{lidar_height} px"
)

print(
    f"Resolution  : "
    f"{resolution_m_per_px} m/px"
)

print(
    f"Resolution  : "
    f"{lidar_cm_per_px:.6f} cm/px"
)

print(
    f"Origin      : "
    f"{origin}"
)

print(
    f"Real Width  : "
    f"{lidar_width * resolution_m_per_px:.3f} m"
)

print(
    f"Real Height : "
    f"{lidar_height * resolution_m_per_px:.3f} m"
)

print(
    f"Negate      : "
    f"{negate}"
)

print(
    f"Occupied th.: "
    f"{occupied_thresh}"
)

print(
    f"Free th.    : "
    f"{free_thresh}"
)


# ============================================================
# 11. 고정 스케일 계산
#
# Camera:
# 1200 px / 200 cm
# = 6 px/cm
#
# LiDAR:
# 0.01 m/px
# = 1 cm/px
#
# Camera BEV px -> LiDAR px:
#
# 1 / (6 × 1)
# = 1/6
# ============================================================

fixed_scale_x = (
    1.0
    /
    (
        camera_px_per_cm_x
        *
        lidar_cm_per_px
    )
)

fixed_scale_y = (
    1.0
    /
    (
        camera_px_per_cm_y
        *
        lidar_cm_per_px
    )
)


print()
print("=" * 70)
print("Fixed Scale")
print("=" * 70)

print(
    f"Scale X: "
    f"{fixed_scale_x:.9f}"
)

print(
    f"Scale Y: "
    f"{fixed_scale_y:.9f}"
)


# 1200 × 600 BEV이면 이론상 1/6
if (
    bev_width == 1200
    and
    bev_height == 600
):

    print(
        "Expected Scale: "
        "0.166666667 = 1/6"
    )


# ============================================================
# 12. 표시 설정
# ============================================================

BEV_DISPLAY_WIDTH = 1200
BEV_DISPLAY_HEIGHT = 600


bev_display_scale_x = (
    BEV_DISPLAY_WIDTH
    /
    bev_width
)

bev_display_scale_y = (
    BEV_DISPLAY_HEIGHT
    /
    bev_height
)


# LiDAR PGM 표시 확대
#
# 새 맵은 이전보다 resolution이 높아서
# PGM 크기가 훨씬 커질 수 있으므로
# 고정 10배 대신 자동 계산
# ============================================================

MAX_LIDAR_DISPLAY_WIDTH = 900
MAX_LIDAR_DISPLAY_HEIGHT = 750


lidar_display_scale = min(
    MAX_LIDAR_DISPLAY_WIDTH
    /
    lidar_width,

    MAX_LIDAR_DISPLAY_HEIGHT
    /
    lidar_height,

    4.0
)


# 너무 작아지는 것 방지
lidar_display_scale = max(
    lidar_display_scale,
    1.0
)


lidar_display_width = int(
    lidar_width
    *
    lidar_display_scale
)

lidar_display_height = int(
    lidar_height
    *
    lidar_display_scale
)


print()
print("=" * 70)
print("Display")
print("=" * 70)

print(
    f"LiDAR Display Scale: "
    f"{lidar_display_scale:.4f}"
)

print(
    f"LiDAR Display Size : "
    f"{lidar_display_width} x "
    f"{lidar_display_height}"
)


# ============================================================
# 13. 대응점 저장
# ============================================================

camera_points = []

lidar_points = []

frozen_bev = None


# ============================================================
# 14. Camera 클릭
# ============================================================

def camera_mouse_callback(
    event,
    x,
    y,
    flags,
    param
):

    if event != cv2.EVENT_LBUTTONDOWN:
        return


    if frozen_bev is None:

        print(
            "[WARNING] 먼저 S 키를 눌러 "
            "BEV를 고정하세요."
        )

        return


    # 표시 좌표 -> 실제 BEV 좌표
    bev_x = (
        x
        /
        bev_display_scale_x
    )

    bev_y = (
        y
        /
        bev_display_scale_y
    )


    camera_points.append(
        [
            bev_x,
            bev_y
        ]
    )


    print()
    print(
        f"[Camera Point "
        f"{len(camera_points)}]"
    )

    print(
        f"BEV = "
        f"({bev_x:.3f}, "
        f"{bev_y:.3f})"
    )


# ============================================================
# 15. LiDAR 클릭
# ============================================================

def lidar_mouse_callback(
    event,
    x,
    y,
    flags,
    param
):

    if event != cv2.EVENT_LBUTTONDOWN:
        return


    # 확대 표시 좌표 -> 원본 PGM 좌표
    lidar_x = (
        x
        /
        lidar_display_scale
    )

    lidar_y = (
        y
        /
        lidar_display_scale
    )


    lidar_points.append(
        [
            lidar_x,
            lidar_y
        ]
    )


    print()
    print(
        f"[LiDAR Point "
        f"{len(lidar_points)}]"
    )

    print(
        f"PGM = "
        f"({lidar_x:.3f}, "
        f"{lidar_y:.3f})"
    )


# ============================================================
# 16. 고정 스케일 Rigid Transform
#
# 허용:
# - 고정 scale
# - rotation
# - translation
#
# 금지:
# - perspective deformation
# - shear
# - 자유 scale 변경
# ============================================================

def estimate_fixed_scale_rigid_transform(
    src_points,
    dst_points,
    scale_x,
    scale_y
):

    src = np.asarray(
        src_points,
        dtype=np.float64
    )

    dst = np.asarray(
        dst_points,
        dtype=np.float64
    )


    if src.shape != dst.shape:

        raise ValueError(
            "Camera/LiDAR point shape 불일치"
        )


    if len(src) < 2:

        raise ValueError(
            "최소 2개 대응점 필요"
        )


    # --------------------------------------------------------
    # Camera BEV pixel
    # -> LiDAR pixel scale
    # --------------------------------------------------------

    src_scaled = src.copy()

    src_scaled[:, 0] *= scale_x
    src_scaled[:, 1] *= scale_y


    # --------------------------------------------------------
    # 중심점
    # --------------------------------------------------------

    src_center = np.mean(
        src_scaled,
        axis=0
    )

    dst_center = np.mean(
        dst,
        axis=0
    )


    src_centered = (
        src_scaled
        -
        src_center
    )

    dst_centered = (
        dst
        -
        dst_center
    )


    # --------------------------------------------------------
    # SVD 기반 최적 회전
    # --------------------------------------------------------

    covariance = (
        src_centered.T
        @
        dst_centered
    )


    U, _, Vt = np.linalg.svd(
        covariance
    )


    R = (
        Vt.T
        @
        U.T
    )


    # reflection 방지
    if np.linalg.det(R) < 0:

        Vt[-1, :] *= -1

        R = (
            Vt.T
            @
            U.T
        )


    # --------------------------------------------------------
    # 이동
    # --------------------------------------------------------

    t = (
        dst_center
        -
        R
        @
        src_center
    )


    # --------------------------------------------------------
    # Scale matrix
    # --------------------------------------------------------

    S = np.array(
        [
            [
                scale_x,
                0.0
            ],

            [
                0.0,
                scale_y
            ]
        ],
        dtype=np.float64
    )


    # Rotation × Scale
    A = (
        R
        @
        S
    )


    # 최종 2×3 affine matrix
    M = np.array(
        [
            [
                A[0, 0],
                A[0, 1],
                t[0]
            ],

            [
                A[1, 0],
                A[1, 1],
                t[1]
            ]
        ],
        dtype=np.float64
    )


    # --------------------------------------------------------
    # 대응점 예측
    # --------------------------------------------------------

    src_h = np.hstack(
        [
            src,

            np.ones(
                (
                    len(src),
                    1
                )
            )
        ]
    )


    predicted = (
        M
        @
        src_h.T
    ).T


    # --------------------------------------------------------
    # 오차
    # --------------------------------------------------------

    errors = np.linalg.norm(
        predicted
        -
        dst,
        axis=1
    )


    rmse = np.sqrt(
        np.mean(
            errors ** 2
        )
    )


    return (
        M,
        R,
        t,
        predicted,
        errors,
        rmse
    )


# ============================================================
# 17. 카메라 연결
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
# 18. 실제 카메라 설정 확인
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
# 19. Window 생성
# ============================================================

CAMERA_WINDOW = (
    "Camera BEV"
)

LIDAR_WINDOW = (
    "LiDAR PGM"
)


cv2.namedWindow(
    CAMERA_WINDOW
)

cv2.namedWindow(
    LIDAR_WINDOW
)


cv2.setMouseCallback(
    CAMERA_WINDOW,
    camera_mouse_callback
)

cv2.setMouseCallback(
    LIDAR_WINDOW,
    lidar_mouse_callback
)


# ============================================================
# 20. LiDAR 표시 이미지
#
# 클릭 정확도를 위해 INTER_NEAREST
# ============================================================

lidar_display_base = cv2.resize(
    lidar_map,
    (
        lidar_display_width,
        lidar_display_height
    ),
    interpolation=cv2.INTER_NEAREST
)


lidar_display_base = cv2.cvtColor(
    lidar_display_base,
    cv2.COLOR_GRAY2BGR
)


# ============================================================
# 21. 조작법
# ============================================================

print()
print("=" * 70)
print("Controls")
print("=" * 70)

print(
    "S : Freeze BEV"
)

print(
    "U : Undo Camera point"
)

print(
    "I : Undo LiDAR point"
)

print(
    "R : Reset all"
)

print(
    "W : Estimate + Save"
)

print(
    "Q : Quit"
)


# ============================================================
# 22. 메인 루프
# ============================================================

while True:

    # --------------------------------------------------------
    # Camera
    # --------------------------------------------------------

    if frozen_bev is None:

        ret, frame = cap.read()


        if not ret:

            print(
                "프레임 읽기 실패"
            )

            break


        # 렌즈 왜곡 보정
        undistorted = cv2.undistort(
            frame,
            camera_matrix,
            dist_coeffs
        )


        # Camera BEV
        bev = cv2.warpPerspective(
            undistorted,
            bev_homography,
            (
                bev_width,
                bev_height
            )
        )


        current_bev = bev


    else:

        current_bev = (
            frozen_bev.copy()
        )


    # --------------------------------------------------------
    # Camera 표시
    # --------------------------------------------------------

    camera_display = cv2.resize(
        current_bev,
        (
            BEV_DISPLAY_WIDTH,
            BEV_DISPLAY_HEIGHT
        )
    )


    for i, point in enumerate(
        camera_points
    ):

        bx, by = point


        dx = int(
            bx
            *
            bev_display_scale_x
        )

        dy = int(
            by
            *
            bev_display_scale_y
        )


        cv2.circle(
            camera_display,
            (
                dx,
                dy
            ),
            7,
            (
                0,
                0,
                255
            ),
            -1
        )


        cv2.putText(
            camera_display,
            str(i + 1),
            (
                dx + 10,
                dy - 10
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
        camera_display,
        (
            f"Camera pts: "
            f"{len(camera_points)}"
        ),
        (
            20,
            35
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


    # --------------------------------------------------------
    # LiDAR 표시
    # --------------------------------------------------------

    lidar_display = (
        lidar_display_base.copy()
    )


    for i, point in enumerate(
        lidar_points
    ):

        lx, ly = point


        dx = int(
            lx
            *
            lidar_display_scale
        )

        dy = int(
            ly
            *
            lidar_display_scale
        )


        cv2.circle(
            lidar_display,
            (
                dx,
                dy
            ),
            7,
            (
                0,
                0,
                255
            ),
            -1
        )


        cv2.putText(
            lidar_display,
            str(i + 1),
            (
                dx + 10,
                dy - 10
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
        lidar_display,
        (
            f"LiDAR pts: "
            f"{len(lidar_points)}"
        ),
        (
            20,
            35
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


    # --------------------------------------------------------
    # Show
    # --------------------------------------------------------

    cv2.imshow(
        CAMERA_WINDOW,
        camera_display
    )

    cv2.imshow(
        LIDAR_WINDOW,
        lidar_display
    )


    # --------------------------------------------------------
    # Key
    # --------------------------------------------------------

    key = cv2.waitKey(1) & 0xFF


    # Q 종료
    if key == ord("q"):

        break


    # S 고정
    elif key == ord("s"):

        if frozen_bev is None:

            frozen_bev = (
                current_bev.copy()
            )

            print()
            print(
                "BEV Frozen"
            )


    # U Camera 마지막 점 삭제
    elif key == ord("u"):

        if len(camera_points) > 0:

            removed = (
                camera_points.pop()
            )

            print(
                f"Camera removed: "
                f"{removed}"
            )


    # I LiDAR 마지막 점 삭제
    elif key == ord("i"):

        if len(lidar_points) > 0:

            removed = (
                lidar_points.pop()
            )

            print(
                f"LiDAR removed: "
                f"{removed}"
            )


    # R 초기화
    elif key == ord("r"):

        camera_points.clear()

        lidar_points.clear()

        frozen_bev = None


        print()
        print(
            "전체 초기화 완료"
        )


    # W 계산 및 저장
    elif key == ord("w"):

        if len(camera_points) < 3:

            print(
                "[ERROR] 최소 3개 "
                "Camera 점 필요"
            )

            continue


        if len(lidar_points) < 3:

            print(
                "[ERROR] 최소 3개 "
                "LiDAR 점 필요"
            )

            continue


        if (
            len(camera_points)
            !=
            len(lidar_points)
        ):

            print(
                "[ERROR] 대응점 개수 불일치"
            )

            continue


        camera_array = np.array(
            camera_points,
            dtype=np.float64
        )

        lidar_array = np.array(
            lidar_points,
            dtype=np.float64
        )


        try:

            (
                M,
                R,
                t,
                predicted,
                errors,
                rmse
            ) = (
                estimate_fixed_scale_rigid_transform(
                    camera_array,
                    lidar_array,
                    fixed_scale_x,
                    fixed_scale_y
                )
            )


        except Exception as e:

            print(
                f"[ERROR] 변환 계산 실패: "
                f"{e}"
            )

            continue


        # ----------------------------------------------------
        # 회전각
        # ----------------------------------------------------

        theta_rad = np.arctan2(
            R[1, 0],
            R[0, 0]
        )

        theta_deg = np.degrees(
            theta_rad
        )


        # ----------------------------------------------------
        # 결과 출력
        # ----------------------------------------------------

        print()
        print("=" * 70)
        print(
            "Fixed Scale Rigid Registration"
        )
        print("=" * 70)

        print(
            f"Rotation: "
            f"{theta_deg:.6f} deg"
        )

        print(
            f"Translation: "
            f"tx={t[0]:.6f}, "
            f"ty={t[1]:.6f}"
        )

        print(
            f"RMSE: "
            f"{rmse:.6f} LiDAR px"
        )

        print(
            f"RMSE physical: "
            f"{rmse * lidar_cm_per_px:.3f} cm"
        )


        print()
        print(
            "Affine Matrix:"
        )

        print(
            M
        )


        print()
        print(
            "Per-point errors:"
        )


        for i, error in enumerate(
            errors
        ):

            print(
                f"{i + 1:02d}: "
                f"{error:.4f} px "
                f"= "
                f"{error * lidar_cm_per_px:.2f} cm"
            )


        # ----------------------------------------------------
        # Camera BEV -> LiDAR Map
        # ----------------------------------------------------

        warped_camera = cv2.warpAffine(
            frozen_bev,
            M.astype(
                np.float32
            ),
            (
                lidar_width,
                lidar_height
            )
        )


        # ----------------------------------------------------
        # Overlay
        # ----------------------------------------------------

        lidar_color = cv2.cvtColor(
            lidar_map,
            cv2.COLOR_GRAY2BGR
        )


        overlay = cv2.addWeighted(
            lidar_color,
            0.5,
            warped_camera,
            0.5,
            0
        )


        overlay_display = cv2.resize(
            overlay,
            (
                lidar_display_width,
                lidar_display_height
            ),
            interpolation=cv2.INTER_NEAREST
        )


        cv2.imshow(
            "Rigid Registration Overlay",
            overlay_display
        )


        # ----------------------------------------------------
        # 대응점 오차 Debug
        #
        # Green = 실제 LiDAR 대응점
        # Red   = Camera 변환 예측점
        # ----------------------------------------------------

        debug = cv2.cvtColor(
            lidar_map,
            cv2.COLOR_GRAY2BGR
        )


        for i in range(
            len(lidar_array)
        ):

            actual_x = int(
                round(
                    lidar_array[i, 0]
                )
            )

            actual_y = int(
                round(
                    lidar_array[i, 1]
                )
            )


            pred_x = int(
                round(
                    predicted[i, 0]
                )
            )

            pred_y = int(
                round(
                    predicted[i, 1]
                )
            )


            # 실제 LiDAR 점
            cv2.circle(
                debug,
                (
                    actual_x,
                    actual_y
                ),
                3,
                (
                    0,
                    255,
                    0
                ),
                -1
            )


            # Camera 예측점
            cv2.circle(
                debug,
                (
                    pred_x,
                    pred_y
                ),
                3,
                (
                    0,
                    0,
                    255
                ),
                -1
            )


            # 오차 연결선
            cv2.line(
                debug,
                (
                    actual_x,
                    actual_y
                ),
                (
                    pred_x,
                    pred_y
                ),
                (
                    255,
                    0,
                    255
                ),
                1
            )


        debug_display = cv2.resize(
            debug,
            (
                lidar_display_width,
                lidar_display_height
            ),
            interpolation=cv2.INTER_NEAREST
        )


        cv2.imshow(
            "Registration Error Debug",
            debug_display
        )


        # ----------------------------------------------------
        # 저장
        # ----------------------------------------------------

        np.savez(
            OUTPUT_FILE,

            affine_matrix=M,

            rotation_matrix=R,

            translation=t,

            rotation_deg=theta_deg,

            fixed_scale_x=fixed_scale_x,

            fixed_scale_y=fixed_scale_y,

            camera_points=camera_array,

            lidar_points=lidar_array,

            predicted_points=predicted,

            point_errors=errors,

            rmse_lidar_px=rmse,

            rmse_cm=(
                rmse
                *
                lidar_cm_per_px
            ),

            lidar_width=lidar_width,

            lidar_height=lidar_height,

            resolution=resolution_m_per_px,

            origin=origin
        )


        cv2.imwrite(
            str(OVERLAY_FILE),
            overlay
        )


        print()
        print("=" * 70)
        print("Saved")
        print("=" * 70)

        print(
            f"Matrix: "
            f"{OUTPUT_FILE}"
        )

        print(
            f"Overlay: "
            f"{OVERLAY_FILE}"
        )


# ============================================================
# 23. 종료
# ============================================================

cap.release()

cv2.destroyAllWindows()