from __future__ import annotations

import json
import os
import sys
import threading
import time
from pathlib import Path

from flask import Flask, jsonify, render_template, request
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

VIDEO_ENABLED = os.environ.get("PINKK_VIDEO_ENABLED", "true").lower() not in {
    "0",
    "false",
    "no",
}
VIDEO_TOPIC = os.environ.get(
    "PINKK_VIDEO_TOPIC",
    "/pinkk/camera_bev/image",
)
VIDEO_PORT = int(os.environ.get("PINKK_VIDEO_PORT", "8080"))

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

app = Flask(
    __name__,
    template_folder=str(TEMPLATE_DIR),
)

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
        "camera_mode": "순수 BEV 실시간 영상" if VIDEO_ENABLED else "영상 비활성",
    }


system_states = {robot: initial_vehicle_state(robot) for robot in ROBOT_NAMESPACES}


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


@app.route("/")
def index():
    return render_template(
        "index_user.html",
        video_enabled=VIDEO_ENABLED,
        video_topic=VIDEO_TOPIC,
        video_port=VIDEO_PORT,
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
    print(f"카메라 모드: {system_states[1]['camera_mode']}")
    print("=" * 64)

    app.run(
        host="0.0.0.0",
        port=5002,
        debug=False,
        threaded=True,
        use_reloader=False,
    )
