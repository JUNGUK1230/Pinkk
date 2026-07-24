import json
import os
import threading
import time

import cv2
import numpy as np
from pathlib import Path

from ultralytics import YOLO

from central_control.camera_tools.first_map.coordinate_transformer import (
    CoordinateTransformer
)


# ============================================================
# 1. 기본 설정
# ============================================================

# USB 재연결 시 /dev/video 번호는 바뀔 수 있다. 고정된 by-id 경로를 기본값으로
# 사용해, 같은 LKZC USB 카메라라면 다시 연결한 뒤에도 자동으로 찾는다.
CAMERA_DEVICE = os.getenv(
    "PINKK_CAMERA_DEVICE",
    "/dev/v4l/by-id/usb-LKZC_USB_Camera_200901010001-video-index0",
)
CAMERA_WIDTH = 1920
CAMERA_HEIGHT = 1080
FPS = 30


PROJECT_DIR = Path(__file__).resolve().parent
CENTRAL_CONTROL_DIR = PROJECT_DIR.parent
CAMERA_DATA_DIR = CENTRAL_CONTROL_DIR / "camera_tools" / "first_map"


# ============================================================
# 2. 파일 경로
# ============================================================

CALIBRATION_FILE = (
    CAMERA_DATA_DIR
    / "camera_calibration.npz"
)

BEV_FILE = (
    CAMERA_DATA_DIR
    / "bev_homography.npz"
)

REGISTRATION_FILE = (
    CAMERA_DATA_DIR
    / "camera_to_lidar_rigid_registration.npz"
)

MODEL_FILE = (
    CENTRAL_CONTROL_DIR
    / "models"
    / "best.pt"
)

PARKING_SLOTS_FILE = (
    CENTRAL_CONTROL_DIR
    / "config"
    / "map"
    / "parking_slots_bev.json"
)

LIDAR_MAP_FILE = (
    CAMERA_DATA_DIR
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
    PARKING_SLOTS_FILE,
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
# 6. 주차면 polygon 로드
# ============================================================

with PARKING_SLOTS_FILE.open(encoding="utf-8") as file:

    parking_slot_points = {
        name: np.asarray(points, dtype=np.int32)
        for name, points in json.load(file).items()
    }


parking_slot_masks = {}

for name, points in parking_slot_points.items():

    slot_mask = np.zeros(
        (bev_height, bev_width),
        dtype=np.uint8
    )

    cv2.fillPoly(slot_mask, [points], 255)
    parking_slot_masks[name] = slot_mask


# ============================================================
# 7. Coordinate Transformer
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


if model.task != "segment":

    raise RuntimeError(
        "Segmentation 모델이 아닙니다: "
        f"task={model.task}"
    )


print(
    "YOLO11 Segmentation Model Loaded"
)


# ============================================================
# 10. 검출 설정
# ============================================================

# 처음 테스트는 조금 낮게
CONF_THRESHOLD = 0.05
OCCUPANCY_THRESHOLD = 0.1


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
    CAMERA_DEVICE,
    cv2.CAP_V4L2
)


if not cap.isOpened():

    raise RuntimeError(
        f"카메라 {CAMERA_DEVICE} "
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
    f"Camera device : {CAMERA_DEVICE}"
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


# ============================================================
# 15. 프레임 처리
# ============================================================

def process_frame(frame):
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
    # Segmentation mask overlay
    # --------------------------------------------------------

    vehicle_mask = np.zeros(
        (bev_height, bev_width),
        dtype=np.uint8
    )

    if result.masks is not None:

        mask_overlay = annotated_bev.copy()

        for polygon in result.masks.xy:

            polygon_i = np.round(
                polygon
            ).astype(np.int32)

            if len(polygon_i) >= 3:

                cv2.fillPoly(
                    vehicle_mask,
                    [polygon_i],
                    255
                )

                cv2.fillPoly(
                    mask_overlay,
                    [polygon_i],
                    (
                        255,
                        0,
                        255
                    )
                )

        annotated_bev = cv2.addWeighted(
            mask_overlay,
            0.35,
            annotated_bev,
            0.65,
            0.0
        )


    # --------------------------------------------------------
    # 주차면별 차량 마스크 점유율
    # --------------------------------------------------------

    parking_overlay = annotated_bev.copy()

    for slot_name, slot_mask in parking_slot_masks.items():

        overlap = cv2.bitwise_and(
            slot_mask,
            vehicle_mask
        )

        slot_area = cv2.countNonZero(slot_mask)
        occupied_area = cv2.countNonZero(overlap)
        occupancy_ratio = occupied_area / max(slot_area, 1)
        occupied = occupancy_ratio >= OCCUPANCY_THRESHOLD
        points = parking_slot_points[slot_name]

        color = (
            (0, 0, 255)
            if occupied
            else (0, 200, 0)
        )

        cv2.fillPoly(
            parking_overlay,
            [points],
            color
        )

        cv2.polylines(
            annotated_bev,
            [points],
            True,
            color,
            3,
            cv2.LINE_AA
        )

        center = np.mean(
            points,
            axis=0
        ).astype(int)

        status = "OCCUPIED" if occupied else "EMPTY"
        label = (
            f"{slot_name} {status} "
            f"{occupancy_ratio:.0%}"
        )

        cv2.putText(
            annotated_bev,
            label,
            (center[0] - 65, center[1]),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (255, 255, 255),
            2,
            cv2.LINE_AA
        )

    annotated_bev = cv2.addWeighted(
        parking_overlay,
        0.16,
        annotated_bev,
        0.84,
        0.0
    )


    # --------------------------------------------------------
    # STEP 6
    # Segmentation loop
    # --------------------------------------------------------

    if result.boxes is not None:

        for index, box in enumerate(result.boxes):

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
            # segmentation mask와 중심점
            # -----------------------------------------------

            polygon_i = None

            if (
                result.masks is not None
                and index < len(result.masks.xy)
            ):

                polygon_i = np.round(
                    result.masks.xy[index]
                ).astype(np.int32)


            if polygon_i is not None and len(polygon_i) >= 3:

                moments = cv2.moments(polygon_i)

                if moments["m00"] != 0:

                    cx = moments["m10"] / moments["m00"]
                    cy = moments["m01"] / moments["m00"]

                else:

                    cx = (x1 + x2) / 2.0
                    cy = (y1 + y2) / 2.0

            else:

                cx = (x1 + x2) / 2.0
                cy = (y1 + y2) / 2.0


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


            # segmentation 윤곽선
            if polygon_i is not None and len(polygon_i) >= 3:

                cv2.polylines(
                    annotated_bev,
                    [polygon_i],
                    True,
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
                    f"V{vehicle_count} ({map_x:.2f}, {map_y:.2f})",
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

    # BEV 화면과 LiDAR 맵 화면을 각각 반환
    return bev_display, lidar_display


# ============================================================
# 16. Flask MJPEG 스트리밍
#
# 카메라는 한 번만 읽고, 처리된 BEV와 LiDAR 화면을 각각 캐시한다.
# /video_feed와 /map_feed가 동시에 접속해도 카메라를 서로 빼앗지 않는다.
# ============================================================

_frame_lock = threading.Lock()
_latest_bev_jpeg = None
_latest_lidar_jpeg = None
_latest_raw_bev_jpeg = None
_processor_thread = None
_processor_started = False


def _frame_processor():
    global _latest_bev_jpeg
    global _latest_lidar_jpeg
    global _latest_raw_bev_jpeg

    while True:
        ret, frame = cap.read()

        if not ret:
            time.sleep(0.02)
            continue

        try:
            undistorted = cv2.undistort(frame, camera_matrix, dist_coeffs)
            raw_bev = cv2.warpPerspective(undistorted, bev_homography, (bev_width, bev_height))
            bev_display, lidar_display = process_frame(frame)

            bev_success, bev_buffer = cv2.imencode(
                ".jpg",
                bev_display
            )

            raw_success, raw_buffer = cv2.imencode(".jpg", raw_bev)

            lidar_success, lidar_buffer = cv2.imencode(
                ".jpg",
                lidar_display
            )

            if not bev_success or not lidar_success or not raw_success:
                continue

            with _frame_lock:
                _latest_bev_jpeg = bev_buffer.tobytes()
                _latest_lidar_jpeg = lidar_buffer.tobytes()
                _latest_raw_bev_jpeg = raw_buffer.tobytes()

        except Exception as error:
            print(f"프레임 처리 오류: {error}")
            time.sleep(0.05)


def _ensure_processor_started():
    global _processor_thread
    global _processor_started

    if _processor_started:
        return

    with _frame_lock:
        if _processor_started:
            return

        _processor_thread = threading.Thread(
            target=_frame_processor,
            daemon=True,
            name="parking-camera-processor"
        )
        _processor_thread.start()
        _processor_started = True


def _mjpeg_stream(frame_type):
    _ensure_processor_started()

    while True:
        with _frame_lock:
            if frame_type == "bev":
                jpeg = _latest_bev_jpeg
            elif frame_type == "raw_bev":
                jpeg = _latest_raw_bev_jpeg
            else:
                jpeg = _latest_lidar_jpeg

        if jpeg is None:
            time.sleep(0.03)
            continue

        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n"
            + jpeg
            + b"\r\n"
        )

        time.sleep(0.03)


def generate_frames():
    """YOLO BEV 화면용 Flask 스트림."""
    yield from _mjpeg_stream("bev")


def generate_map_frames():
    """LiDAR 맵 화면용 Flask 스트림."""
    yield from _mjpeg_stream("lidar")


# ============================================================
# 17. 단독 실행 테스트
# ============================================================

def run_desktop():
    while True:
        ret, frame = cap.read()

        if not ret:
            continue

        undistorted = cv2.undistort(frame, camera_matrix, dist_coeffs)
        bev_display, lidar_display = process_frame(frame)

        cv2.imshow("YOLO BEV", bev_display)
        cv2.imshow("LiDAR MAP", lidar_display)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()

def generate_raw_bev_frames():
    """YOLO 없는 BEV 화면."""
    yield from _mjpeg_stream("raw_bev")

if __name__ == "__main__":
    run_desktop()
