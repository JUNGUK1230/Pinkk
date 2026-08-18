from __future__ import annotations

import atexit
import json
import sys
import threading
import time
from pathlib import Path

import cv2
import numpy as np
from flask import Flask, jsonify, redirect, render_template, request
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String

# ============================================================
# 경로 설정
# ============================================================

BASE_DIR = Path(__file__).resolve().parent
SRC_DIR = Path(__file__).resolve().parents[2]

TEMPLATE_DIR = BASE_DIR / "templates"

sys.path.insert(0, str(SRC_DIR))

from central_control.vehicle_registry import VEHICLES as VEHICLE_REGISTRY

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

VEHICLES = {
    index: {
        "vehicle_id": vehicle.vehicle_id,
        "namespace": vehicle.ros_namespace,
        "ros_namespace": vehicle.ros_namespace,
        "controller_id": vehicle.controller_id,
        "hardware_serial": vehicle.hardware_serial,
        "display_name": vehicle.display_name,
    }
    for index, vehicle in enumerate(VEHICLE_REGISTRY.values(), start=1)
}
ROBOT_NAMESPACES = {
    robot: vehicle["namespace"] for robot, vehicle in VEHICLES.items()
}
CENTRAL_CONTROL_TOPIC = "/pinkk/web/control"
LOCALIZATION_IMAGE_TOPIC = "/pinkk/localization/image"
VIDEO_SERVER_PORT = 8080

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

def initial_vehicle_state(robot: int) -> dict:
    return {
        "robot": robot,
        **VEHICLES[robot],
        "battery": None,
        "battery_voltage": None,
        "battery_connected": False,
        "charging": False,
        "parking_slot": "P3" if robot == 1 else "P4",
        "location": f"P{robot + 2} 주차구역",
        "state": "주차 완료",
        "request_state": "대기 중",
        "estimated_time": "-",
        "progress": 100,
        "camera_mode": "관제 영상 공유",
    }


system_states = {robot: initial_vehicle_state(robot) for robot in ROBOT_NAMESPACES}


def set_camera_mode(mode: str) -> None:
    for state in system_states.values():
        state["camera_mode"] = mode

battery_lock = threading.Lock()
battery_last_updates = {robot: 0.0 for robot in ROBOT_NAMESPACES}
ros_node_lock = threading.Lock()
ros_node: ParkingUserRosNode | None = None


class ParkingUserRosNode(Node):
    def __init__(self):
        super().__init__("parking_user_web_battery_subscriber")
        self.central_control_publisher = self.create_publisher(
            String, CENTRAL_CONTROL_TOPIC, 10
        )
        self.lcd_status_publishers = {}
        self._subscriptions = []
        for robot, namespace in ROBOT_NAMESPACES.items():
            self._subscriptions.append(self.create_subscription(
                Float32,
                f"{namespace}/battery/percent",
                lambda msg, robot=robot: self.percent_callback(robot, msg),
                10,
            ))
            self._subscriptions.append(self.create_subscription(
                Float32,
                f"{namespace}/battery/voltage",
                lambda msg, robot=robot: self.voltage_callback(robot, msg),
                10,
            ))
            self.lcd_status_publishers[robot] = self.create_publisher(
                String, f"{namespace}/lcd_status", 10
            )

    def percent_callback(self, robot: int, msg):
        percent = max(0.0, min(100.0, float(msg.data)))
        with battery_lock:
            system_states[robot]["battery"] = round(percent, 1)
            system_states[robot]["battery_connected"] = True
            battery_last_updates[robot] = time.time()

    def voltage_callback(self, robot: int, msg):
        with battery_lock:
            system_states[robot]["battery_voltage"] = round(float(msg.data), 2)
            system_states[robot]["battery_connected"] = True
            battery_last_updates[robot] = time.time()

    def publish_user_request(
        self,
        robot: int,
        command: str,
        requested_by: str,
    ) -> None:
        vehicle = VEHICLES[robot]
        request_message = String()
        request_message.data = json.dumps(
            {
                "vehicle_id": vehicle["vehicle_id"],
                "robot_id": vehicle["vehicle_id"],
                "controller_id": vehicle["controller_id"],
                "hardware_serial": vehicle["hardware_serial"],
                "ros_namespace": vehicle["ros_namespace"],
                "command": command,
                "priority": 6 if command == "entry" else 4,
                "source": "parking_user_web",
                "requested_by": requested_by,
                "requested_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
            },
            ensure_ascii=False,
        )
        self.central_control_publisher.publish(request_message)

        lcd_message = String()
        lcd_message.data = "입차 중" if command == "entry" else "출차 중"
        self.lcd_status_publishers[robot].publish(lcd_message)


def run_ros_node() -> None:
    global ros_node

    try:
        rclpy.init(args=None)
        node = ParkingUserRosNode()
        with ros_node_lock:
            ros_node = node
        rclpy.spin(node)
        node.destroy_node()
    except Exception as error:
        print(f"[배터리 구독 오류] {error}")
    finally:
        with ros_node_lock:
            ros_node = None
        if rclpy.ok():
            rclpy.shutdown()


def refresh_battery_connection_state() -> None:
    while True:
        with battery_lock:
            for robot, last_update in battery_last_updates.items():
                if last_update == 0.0 or time.time() - last_update > 15.0:
                    system_states[robot]["battery_connected"] = False
        time.sleep(2.0)


def requested_robot(payload: dict | None = None) -> int:
    raw = (payload or {}).get("robot", request.args.get("robot", 1))
    try:
        robot = int(raw)
    except (TypeError, ValueError) as error:
        raise ValueError("robot은 1 또는 2여야 합니다.") from error
    if robot not in ROBOT_NAMESPACES:
        raise ValueError("robot은 1 또는 2여야 합니다.")
    return robot


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
            set_camera_mode("연결 끊김")
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
            set_camera_mode("연결 끊김")
            print("[경고] 카메라는 열렸지만 프레임을 받지 못했습니다.")
            return False

        camera = cap
        if (
            camera_matrix is not None
            and dist_coeffs is not None
            and homography_matrix is not None
        ):
            set_camera_mode("실시간 BEV")
        else:
            set_camera_mode("카메라 연결됨 / BEV 파일 없음")

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
        set_camera_mode("연결 끊김")
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
    # USB 카메라는 중앙 localization 프로세스 하나만 점유한다. 사용자웹은
    # 동일 ROS 영상을 web_video_server에서 받아 카메라 충돌을 방지한다.
    browser_host = request.host.split(":", 1)[0]
    return redirect(
        f"http://{browser_host}:{VIDEO_SERVER_PORT}/stream"
        f"?topic={LOCALIZATION_IMAGE_TOPIC}"
    )


@app.route("/api/status")
def api_status():
    try:
        robot = requested_robot()
    except ValueError as error:
        return jsonify({"ok": False, "message": str(error)}), 400
    with battery_lock:
        return jsonify(system_states[robot])


@app.route("/api/vehicles")
def api_vehicles():
    return jsonify({"vehicles": list(VEHICLES.values())})


@app.route("/api/battery")
def api_battery():
    try:
        robot = requested_robot()
    except ValueError as error:
        return jsonify({"success": False, "message": str(error)}), 400
    with battery_lock:
        state = system_states[robot]
        return jsonify(
            {
                "success": True,
                "percent": state["battery"],
                "voltage": state["battery_voltage"],
                "connected": state["battery_connected"],
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
    try:
        robot = requested_robot(payload)
    except ValueError as error:
        return jsonify({"ok": False, "message": str(error)}), 400
    state = system_states[robot]
    vehicle_id = payload.get("vehicle_id", state["vehicle_id"])

    with ros_node_lock:
        node = ros_node
    if node is None:
        return jsonify(
            {
                "ok": False,
                "message": "ROS 연결이 준비되지 않아 요청을 전송하지 못했습니다.",
            }
        ), 503

    requested_by = str(payload.get("requested_by") or "parking_user").strip()
    node.publish_user_request(robot, command, requested_by)

    if command == "entry":
        state.update(
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
        destination = state["parking_slot"]
    else:
        state.update(
            {
                "vehicle_id": vehicle_id,
                "state": "출차 준비 완료",
                "request_state": "출차 요청 완료",
                "estimated_time": "약 1분",
                "progress": 15,
                "location": f"{state['parking_slot']} 출차 준비",
            }
        )
        message = "출차 요청이 중앙 관제 시스템에 전달되었습니다."
        destination = "출구"

    return jsonify(
        {
            "ok": True,
            "message": message,
            "state": state["state"],
            "estimated_time": state["estimated_time"],
            "destination": destination,
            "progress": state["progress"],
        }
    )

if __name__ == "__main__":
    threading.Thread(target=run_ros_node, daemon=True).start()
    threading.Thread(target=refresh_battery_connection_state, daemon=True).start()

    print("=" * 64)
    print("사용자 주차 서비스 웹 서버 시작")
    print("접속 주소: http://127.0.0.1:5002")
    print(f"영상 모드: {system_states[1]['camera_mode']}")
    print("=" * 64)

    app.run(
        host="0.0.0.0",
        port=5002,
        debug=True,
        threaded=True,
        use_reloader=False,
    )
