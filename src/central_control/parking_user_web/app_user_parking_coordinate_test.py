from __future__ import annotations

import atexit
import sys
import threading
import time
from pathlib import Path

import cv2
import numpy as np
from flask import Flask, Response, jsonify, render_template, request
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

# ============================================================
# 경로 설정
# ============================================================

BASE_DIR = Path(__file__).resolve().parent
SRC_DIR = Path(__file__).resolve().parents[2]

TEMPLATE_DIR = BASE_DIR / "templates"

sys.path.insert(0, str(SRC_DIR))

FIRST_MAP_DIR = SRC_DIR / "central_control" / "camera_tools" / "first_map"

CALIBRATION_FILE = FIRST_MAP_DIR / "camera_calibration.npz"
HOMOGRAPHY_FILE = FIRST_MAP_DIR / "bev_homography.npz"

CALIBRATION_CANDIDATES = [CALIBRATION_FILE]
HOMOGRAPHY_CANDIDATES = [HOMOGRAPHY_FILE]

# ============================================================
# 카메라 설정
# ============================================================

CAMERA_ID = 2
CAMERA_WIDTH = 1920
CAMERA_HEIGHT = 1080
CAMERA_FPS = 30

ROBOT_NAMESPACE = "/pinky1"
BATTERY_PERCENT_TOPIC = f"{ROBOT_NAMESPACE}/battery/percent"
BATTERY_VOLTAGE_TOPIC = f"{ROBOT_NAMESPACE}/battery/voltage"

app = Flask(
    __name__,
    template_folder=str(TEMPLATE_DIR),
)

camera_lock = threading.Lock()
camera: cv2.VideoCapture | None = None

camera_matrix: np.ndarray | None = None
dist_coeffs: np.ndarray | None = None
homography_matrix: np.ndarray | None = None
bev_width = 1600
bev_height = 800

system_state = {
    "vehicle_id": "PINKY_01",
    "battery": None,
    "battery_voltage": None,
    "battery_connected": False,
    "charging": False,
    "parking_slot": "P3",
    "location": "P3 주차구역",
    "state": "주차 완료",
    "request_state": "대기 중",
    "estimated_time": "-",
    "progress": 100,
    "camera_mode": "확인 중",
}

battery_lock = threading.Lock()
battery_last_update = 0.0


class BatterySubscriber(Node):
    def __init__(self):
        super().__init__("parking_user_web_battery_subscriber")
        self.create_subscription(
            Float32,
            BATTERY_PERCENT_TOPIC,
            self.percent_callback,
            10,
        )
        self.create_subscription(
            Float32,
            BATTERY_VOLTAGE_TOPIC,
            self.voltage_callback,
            10,
        )

    def percent_callback(self, msg):
        global battery_last_update

        percent = max(0.0, min(100.0, float(msg.data)))
        with battery_lock:
            system_state["battery"] = round(percent, 1)
            system_state["battery_connected"] = True
            battery_last_update = time.time()

    def voltage_callback(self, msg):
        global battery_last_update

        with battery_lock:
            system_state["battery_voltage"] = round(float(msg.data), 2)
            system_state["battery_connected"] = True
            battery_last_update = time.time()


def run_battery_subscriber() -> None:
    try:
        rclpy.init(args=None)
        node = BatterySubscriber()
        rclpy.spin(node)
        node.destroy_node()
    except Exception as error:
        print(f"[배터리 구독 오류] {error}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


def refresh_battery_connection_state() -> None:
    while True:
        with battery_lock:
            if battery_last_update == 0.0 or time.time() - battery_last_update > 15.0:
                system_state["battery_connected"] = False
        time.sleep(2.0)


def find_existing_file(candidates: list[Path], label: str) -> Path:
    for path in candidates:
        if path.exists():
            print(f"{label}: {path}")
            return path

    searched = "\n".join(f"  - {path}" for path in candidates)
    raise FileNotFoundError(
        f"{label} 파일을 찾을 수 없습니다.\n검색 위치:\n{searched}"
    )


def load_bev_files() -> None:
    global camera_matrix
    global dist_coeffs
    global homography_matrix
    global bev_width
    global bev_height

    calibration_path = find_existing_file(
        CALIBRATION_CANDIDATES,
        "Calibration",
    )
    homography_path = find_existing_file(
        HOMOGRAPHY_CANDIDATES,
        "Homography",
    )

    calibration_data = np.load(calibration_path)
    bev_data = np.load(homography_path)

    # 기존 파일 키 이름 차이를 둘 다 지원
    if "camera_matrix" in calibration_data.files:
        camera_matrix = calibration_data["camera_matrix"]
    elif "cameraMatrix" in calibration_data.files:
        camera_matrix = calibration_data["cameraMatrix"]
    else:
        raise KeyError(
            "camera_calibration.npz에 camera_matrix 또는 cameraMatrix가 없습니다."
        )

    if "dist_coeffs" in calibration_data.files:
        dist_coeffs = calibration_data["dist_coeffs"]
    elif "dist" in calibration_data.files:
        dist_coeffs = calibration_data["dist"]
    else:
        raise KeyError(
            "camera_calibration.npz에 dist_coeffs 또는 dist가 없습니다."
        )

    homography_matrix = bev_data["homography_matrix"]
    bev_width = int(bev_data["bev_width"])
    bev_height = int(bev_data["bev_height"])

    print("=" * 64)
    print("BEV 설정 로드 완료")
    print(f"BEV 크기: {bev_width} x {bev_height}")
    print("=" * 64)


def open_camera() -> bool:
    global camera

    with camera_lock:
        if camera is not None and camera.isOpened():
            return True

        cap = cv2.VideoCapture(CAMERA_ID, cv2.CAP_V4L2)

        if not cap.isOpened():
            system_state["camera_mode"] = "연결 끊김"
            print(f"[경고] 카메라 {CAMERA_ID}번을 열 수 없습니다.")
            return False

        cap.set(
            cv2.CAP_PROP_FOURCC,
            cv2.VideoWriter_fourcc(*"MJPG"),
        )
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
        cap.set(cv2.CAP_PROP_FPS, CAMERA_FPS)

        # 실제 프레임 수신까지 확인
        received = False
        for _ in range(10):
            ok, frame = cap.read()
            if ok and frame is not None and frame.size > 0:
                received = True
                break
            time.sleep(0.05)

        if not received:
            cap.release()
            system_state["camera_mode"] = "연결 끊김"
            print("[경고] 카메라는 열렸지만 프레임을 받지 못했습니다.")
            return False

        camera = cap
        if (
            camera_matrix is not None
            and dist_coeffs is not None
            and homography_matrix is not None
        ):
            system_state["camera_mode"] = "실시간 BEV"
        else:
            system_state["camera_mode"] = "카메라 연결됨 / BEV 파일 없음"

        actual_width = int(camera.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(camera.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = camera.get(cv2.CAP_PROP_FPS)

        print("=" * 64)
        print("Runtime Camera")
        print(f"Camera ID : {CAMERA_ID}")
        print(f"Resolution: {actual_width} x {actual_height}")
        print(f"FPS       : {actual_fps}")
        print("=" * 64)

        return True


def close_camera() -> None:
    global camera

    with camera_lock:
        if camera is not None:
            camera.release()
            camera = None


atexit.register(close_camera)


def read_bev_frame() -> np.ndarray | None:
    if camera_matrix is None or dist_coeffs is None or homography_matrix is None:
        return None

    if not open_camera():
        return None

    with camera_lock:
        if camera is None:
            return None

        ok, frame = camera.read()

    if not ok or frame is None:
        system_state["camera_mode"] = "연결 끊김"
        close_camera()
        return None

    undistorted = cv2.undistort(
        frame,
        camera_matrix,
        dist_coeffs,
    )

    bev = cv2.warpPerspective(
        undistorted,
        homography_matrix,
        (bev_width, bev_height),
    )

    return bev


def generate_video():
    while True:
        frame = read_bev_frame()
        if frame is None:
            return

        ok, encoded = cv2.imencode(
            ".jpg",
            frame,
            [cv2.IMWRITE_JPEG_QUALITY, 85],
        )
        if not ok:
            continue

        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n"
            + encoded.tobytes()
            + b"\r\n"
        )

        time.sleep(1 / CAMERA_FPS)


@app.route("/")
def index():
    return render_template("index_user.html")


@app.route("/video_feed")
def video_feed():
    if camera_matrix is None or homography_matrix is None or not open_camera():
        return Response("BEV 영상 스트림을 사용할 수 없습니다.", status=503)

    return Response(
        generate_video(),
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )


@app.route("/api/status")
def api_status():
    with battery_lock:
        return jsonify(system_state)


@app.route("/api/battery")
def api_battery():
    with battery_lock:
        return jsonify(
            {
                "success": True,
                "percent": system_state["battery"],
                "voltage": system_state["battery_voltage"],
                "connected": system_state["battery_connected"],
            }
        )


@app.route("/api/route/<command>", methods=["POST"])
def route_request(command: str):
    if command not in {"entry", "exit"}:
        return jsonify(
            {
                "ok": False,
                "message": "지원하지 않는 명령입니다.",
            }
        ), 400

    payload = request.get_json(silent=True) or {}
    vehicle_id = payload.get("vehicle_id", system_state["vehicle_id"])

    if command == "entry":
        system_state.update(
            {
                "vehicle_id": vehicle_id,
                "state": "입차 경로 생성 완료",
                "request_state": "입차 요청 완료",
                "estimated_time": "약 1분",
                "progress": 15,
                "location": "입구 대기구역",
            }
        )
        message = "입차 요청이 중앙 관제 시스템에 전달되었습니다."
        destination = system_state["parking_slot"]
    else:
        system_state.update(
            {
                "vehicle_id": vehicle_id,
                "state": "출차 준비 완료",
                "request_state": "출차 요청 완료",
                "estimated_time": "약 1분",
                "progress": 15,
                "location": f"{system_state['parking_slot']} 출차 준비",
            }
        )
        message = "출차 요청이 중앙 관제 시스템에 전달되었습니다."
        destination = "출구"

    # 현재는 웹 연동 확인용 상태 변경이다.
    # 나중에 여기서 ROS2 서비스 또는 토픽을 호출하면 됨.
    return jsonify(
        {
            "ok": True,
            "message": message,
            "state": system_state["state"],
            "estimated_time": system_state["estimated_time"],
            "destination": destination,
            "progress": system_state["progress"],
        }
    )


if __name__ == "__main__":
    try:
        load_bev_files()
    except (FileNotFoundError, KeyError) as error:
        print(error)
        print("[경고] BEV 영상 스트림을 시작할 수 없습니다.")
        system_state["camera_mode"] = "사용 불가"

    open_camera()
    threading.Thread(target=run_battery_subscriber, daemon=True).start()
    threading.Thread(target=refresh_battery_connection_state, daemon=True).start()

    print("=" * 64)
    print("사용자 주차 서비스 웹 서버 시작")
    print("접속 주소: http://127.0.0.1:5002")
    print(f"카메라 모드: {system_state['camera_mode']}")
    print("=" * 64)

    app.run(
        host="0.0.0.0",
        port=5002,
        debug=True,
        threaded=True,
        use_reloader=False,
    )
