import cv2
import numpy as np
from pathlib import Path

from ultralytics import YOLO

from coordinate_transformer import (
    CoordinateTransformer
)


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

BEV_FILE = (
    PROJECT_DIR
    / "bev_homography.npz"
)

REGISTRATION_FILE = (
    PROJECT_DIR
    / "camera_to_lidar_rigid_registration.npz"
)

MODEL_FILE = Path(
    "/home/junguk/charging_station_vision/yolo11s.pt"
)

LIDAR_MAP_FILE = (
    PROJECT_DIR
    / "my_test_map0710.png"
)


# ============================================================
# 3. 파일 확인
# ============================================================

required_files = [
    CALIBRATION_FILE,
    BEV_FILE,
    REGISTRATION_FILE,
    MODEL_FILE,
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
# 6. Coordinate Transformer
# ============================================================

transformer = CoordinateTransformer(
    REGISTRATION_FILE
)


# ============================================================
# 7. Registration 정보 로드
#
# LiDAR overlay 표시용
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


# ============================================================
# 8. LiDAR Map 로드
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


map_height, map_width = (
    lidar_map.shape
)


if (
    map_width != lidar_width
    or
    map_height != lidar_height
):

    raise RuntimeError(
        "LiDAR map 크기 불일치\n"
        f"Registration: "
        f"{lidar_width} x {lidar_height}\n"
        f"Image       : "
        f"{map_width} x {map_height}"
    )


lidar_color = cv2.cvtColor(
    lidar_map,
    cv2.COLOR_GRAY2BGR
)


# ============================================================
# 9. YOLO 모델 로드
# ============================================================

print("=" * 70)
print("YOLO Model Loading")
print("=" * 70)

print(
    f"Model: {MODEL_FILE}"
)


model = YOLO(
    str(MODEL_FILE)
)


print(
    "YOLO Model Loaded"
)


# ============================================================
# 10. 검출 설정
# ============================================================

# 처음 테스트는 조금 낮게
CONF_THRESHOLD = 0.05


# ============================================================
# 11. 화면 표시 크기
# ============================================================

BEV_DISPLAY_WIDTH = 1200
BEV_DISPLAY_HEIGHT = 600


MAP_DISPLAY_SCALE = min(
    900 / lidar_width,
    750 / lidar_height
)


MAP_DISPLAY_WIDTH = int(
    lidar_width
    *
    MAP_DISPLAY_SCALE
)


MAP_DISPLAY_HEIGHT = int(
    lidar_height
    *
    MAP_DISPLAY_SCALE
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
# 15. 메인 루프
# ============================================================

while True:

    # --------------------------------------------------------
    # STEP 1
    # Camera frame
    # --------------------------------------------------------

    ret, frame = cap.read()


    if not ret:

        print(
            "프레임 읽기 실패"
        )

        break


    # --------------------------------------------------------
    # STEP 2
    # Lens undistortion
    # --------------------------------------------------------

    undistorted = cv2.undistort(
        frame,
        camera_matrix,
        dist_coeffs
    )


    # --------------------------------------------------------
    # STEP 3
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
    # STEP 4
    # YOLO inference
    #
    # 중요:
    # BEV 원본 배열을 그대로 넣음
    # --------------------------------------------------------

    results = model.predict(
        source=bev,
        conf=CONF_THRESHOLD,
        verbose=False
    )


    result = results[0]


    # --------------------------------------------------------
    # STEP 5
    # 표시 영상 준비
    # --------------------------------------------------------

    annotated_bev = (
        bev.copy()
    )


    lidar_detection_view = (
        lidar_color.copy()
    )


    vehicle_count = 0


    # --------------------------------------------------------
    # STEP 6
    # Detection loop
    # --------------------------------------------------------

    if result.boxes is not None:

        for box in result.boxes:

            # -----------------------------------------------
            # class ID
            # -----------------------------------------------

            class_id = int(
                box.cls[0].item()
            )


            class_name = (
                result.names[
                    class_id
                ]
            )


            confidence = float(
                box.conf[0].item()
            )


            # -----------------------------------------------
            # 차량 클래스만 사용
            # -----------------------------------------------

#           if (
#               class_name
#               not in
#               VEHICLE_CLASS_NAMES
#           ):
#
#               continue


            # -----------------------------------------------
            # bbox
            #
            # YOLO 결과는 현재 입력한
            # BEV 영상 좌표 기준
            # -----------------------------------------------

            x1, y1, x2, y2 = (
                box.xyxy[0]
                .cpu()
                .numpy()
                .astype(float)
            )


            # -----------------------------------------------
            # bbox center
            # -----------------------------------------------

            cx = (
                x1 + x2
            ) / 2.0


            cy = (
                y1 + y2
            ) / 2.0


            # -----------------------------------------------
            # BEV -> LiDAR pixel
            # -----------------------------------------------

            lidar_x, lidar_y = (
                transformer
                .bev_to_lidar_pixel(
                    cx,
                    cy
                )
            )


            # -----------------------------------------------
            # BEV -> ROS /map
            # -----------------------------------------------

            map_x, map_y = (
                transformer
                .bev_to_map(
                    cx,
                    cy
                )
            )


            vehicle_count += 1


            # ===============================================
            # BEV 화면 표시
            # ===============================================

            x1_i = int(
                round(x1)
            )

            y1_i = int(
                round(y1)
            )

            x2_i = int(
                round(x2)
            )

            y2_i = int(
                round(y2)
            )


            cx_i = int(
                round(cx)
            )

            cy_i = int(
                round(cy)
            )


            # bbox
            cv2.rectangle(
                annotated_bev,
                (
                    x1_i,
                    y1_i
                ),
                (
                    x2_i,
                    y2_i
                ),
                (
                    0,
                    255,
                    0
                ),
                3
            )


            # 중심점
            cv2.circle(
                annotated_bev,
                (
                    cx_i,
                    cy_i
                ),
                8,
                (
                    0,
                    0,
                    255
                ),
                -1
            )


            # label
            label = (
                f"{class_name} "
                f"{confidence:.2f}"
            )


            cv2.putText(
                annotated_bev,
                label,
                (
                    x1_i,
                    max(
                        y1_i - 10,
                        25
                    )
                ),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (
                    0,
                    255,
                    0
                ),
                2
            )


            # /map 좌표
            map_text = (
                f"map "
                f"({map_x:.3f}, "
                f"{map_y:.3f})m"
            )


            cv2.putText(
                annotated_bev,
                map_text,
                (
                    x1_i,
                    min(
                        y2_i + 30,
                        bev_height - 10
                    )
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


            # ===============================================
            # LiDAR map에 중심점 표시
            # ===============================================

            if (
                transformer
                .is_inside_lidar_map(
                    lidar_x,
                    lidar_y
                )
            ):

                lx_i = int(
                    round(lidar_x)
                )

                ly_i = int(
                    round(lidar_y)
                )


                cv2.circle(
                    lidar_detection_view,
                    (
                        lx_i,
                        ly_i
                    ),
                    4,
                    (
                        0,
                        0,
                        255
                    ),
                    -1
                )


                cv2.putText(
                    lidar_detection_view,
                    f"V{vehicle_count}",
                    (
                        lx_i + 5,
                        ly_i - 5
                    ),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.35,
                    (
                        0,
                        0,
                        255
                    ),
                    1
                )


            # ===============================================
            # 터미널 출력
            # ===============================================

            print(
                f"[Vehicle {vehicle_count}] "
                f"class={class_name} "
                f"conf={confidence:.2f} | "
                f"BEV=({cx:.1f}, {cy:.1f}) | "
                f"LiDAR=({lidar_x:.2f}, "
                f"{lidar_y:.2f}) | "
                f"/map=({map_x:.3f}, "
                f"{map_y:.3f}) m"
            )


    # --------------------------------------------------------
    # STEP 7
    # 상태 표시
    # --------------------------------------------------------

    cv2.putText(
        annotated_bev,
        (
            f"Vehicles: "
            f"{vehicle_count}"
        ),
        (
            20,
            40
        ),
        cv2.FONT_HERSHEY_SIMPLEX,
        1.0,
        (
            0,
            255,
            255
        ),
        2
    )


    cv2.putText(
        annotated_bev,
        "Q: Quit",
        (
            20,
            80
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
    # STEP 8
    # 표시용 resize
    # --------------------------------------------------------

    bev_display = cv2.resize(
        annotated_bev,
        (
            BEV_DISPLAY_WIDTH,
            BEV_DISPLAY_HEIGHT
        ),
        interpolation=cv2.INTER_LINEAR
    )


    lidar_display = cv2.resize(
        lidar_detection_view,
        (
            MAP_DISPLAY_WIDTH,
            MAP_DISPLAY_HEIGHT
        ),
        interpolation=cv2.INTER_NEAREST
    )


    # --------------------------------------------------------
    # STEP 9
    # Show
    # --------------------------------------------------------

    cv2.imshow(
        "YOLO BEV Detection",
        bev_display
    )


    cv2.imshow(
        "Vehicle Position on LiDAR Map",
        lidar_display
    )


    # --------------------------------------------------------
    # STEP 10
    # Key
    # --------------------------------------------------------

    key = cv2.waitKey(1) & 0xFF


    if key == ord("q"):

        break


# ============================================================
# 16. 종료
# ============================================================

cap.release()

cv2.destroyAllWindows()
