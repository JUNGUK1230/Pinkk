import os
from pathlib import Path
import threading
import time

import sys

from flask import Flask, Response, jsonify, render_template, request

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

SRC_DIR = Path(__file__).resolve().parents[2]

sys.path.insert(0, str(SRC_DIR))

VISION_ENABLED = os.getenv("PINKK_ENABLE_VISION", "1").lower() in {
    "1",
    "true",
    "yes",
}

generate_frames = None
generate_map_frames = None
generate_raw_bev_frames = None

if VISION_ENABLED:
    try:
        from central_control.parking_management_web.live_yolo_bev_map_web_separate import (
            generate_frames,
            generate_map_frames,
            generate_raw_bev_frames,
        )
    except (ImportError, FileNotFoundError) as error:
        print(f"[비전 기능 비활성화] {error}")


# ============================================================
# 1. Flask 기본 설정
# ============================================================

BASE_DIR = Path(__file__).resolve().parent

app: Flask = Flask(
    __name__,
    template_folder=str(BASE_DIR / "templates")
)


# ============================================================
# 2. 임시 차량 상태
# 나중에 ROS2 토픽 데이터로 교체
# ============================================================

vehicle_status = {
    "vehicle_id": "PINKY_01",
    "battery": None,
    "battery_voltage": None,
    "battery_connected": False,
    "charging": False,
    "driving_status": "대기 중",
    "current_location": "입구 대기구역",
    "destination": None,
    "route_state": "생성 전",
    "last_command": None
}


# ============================================================
# 3. ROS2 배터리 구독
# /battery/percent, /battery/voltage
# ============================================================

battery_lock = threading.Lock()
battery_last_update = 0.0


class BatterySubscriber(Node):
    def __init__(self):
        super().__init__("parking_web_battery_subscriber")

        self.create_subscription(
            Float32,
            "/battery/percent",
            self.percent_callback,
            10,
        )

        self.create_subscription(
            Float32,
            "/battery/voltage",
            self.voltage_callback,
            10,
        )

    def percent_callback(self, msg):
        global battery_last_update

        percent = max(0.0, min(100.0, float(msg.data)))

        with battery_lock:
            vehicle_status["battery"] = round(percent, 1)
            vehicle_status["battery_connected"] = True
            battery_last_update = time.time()

            if percent <= 20.0:
                vehicle_status["driving_status"] = "충전 필요"

    def voltage_callback(self, msg):
        global battery_last_update

        with battery_lock:
            vehicle_status["battery_voltage"] = round(float(msg.data), 2)
            vehicle_status["battery_connected"] = True
            battery_last_update = time.time()


def run_battery_subscriber():
    try:
        rclpy.init(args=None)
        node = BatterySubscriber()
        rclpy.spin(node)
        node.destroy_node()
    except Exception as exc:
        print(f"[배터리 구독 오류] {exc}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


def refresh_battery_connection_state():
    while True:
        with battery_lock:
            if battery_last_update == 0.0 or time.time() - battery_last_update > 15.0:
                vehicle_status["battery_connected"] = False
        time.sleep(2.0)


# ============================================================
# 4. 메인 웹 화면
# ============================================================

@app.route("/")
def index():
    return render_template("index_management_dual_view_battery.html")


# ============================================================
# 5. 카메라 스트리밍
# ============================================================

@app.route("/video_feed")
def video_feed():
    if generate_frames is None:
        return Response("YOLO 영상 스트림을 사용할 수 없습니다.", status=503)

    return Response(
        generate_frames(),
        mimetype="multipart/x-mixed-replace; boundary=frame"
    )


@app.route("/bev_feed")
def bev_feed():
    if generate_raw_bev_frames is None:
        return Response("BEV 영상 스트림을 사용할 수 없습니다.", status=503)

    return Response(
        generate_raw_bev_frames(),
        mimetype="multipart/x-mixed-replace; boundary=frame"
    )


@app.route("/map_feed")
def map_feed():
    if generate_map_frames is None:
        return Response("LiDAR 맵 스트림을 사용할 수 없습니다.", status=503)

    return Response(
        generate_map_frames(),
        mimetype="multipart/x-mixed-replace; boundary=frame"
    )



# ============================================================
# 6. 차량 상태 조회 API
# ============================================================

@app.route("/api/vehicle/status", methods=["GET"])
def get_vehicle_status():
    return jsonify({
        "success": True,
        **vehicle_status
    })


# ============================================================
# 7. 실시간 배터리 조회 API
# ============================================================

@app.route("/api/battery", methods=["GET"])
def get_battery():
    with battery_lock:
        return jsonify({
            "success": True,
            "percent": vehicle_status["battery"],
            "voltage": vehicle_status["battery_voltage"],
            "connected": vehicle_status["battery_connected"],
        })


# ============================================================
# 8. 배터리 상태 변경 API
# 현재는 테스트용
# 나중에 ROS2 /battery/percent 토픽과 연결
# ============================================================

@app.route("/api/vehicle/battery", methods=["POST"])
def update_battery():
    data = request.get_json(silent=True) or {}

    battery = data.get("battery")

    if battery is None:
        return jsonify({
            "success": False,
            "message": "battery 값이 필요합니다."
        }), 400

    try:
        battery = int(battery)
    except (TypeError, ValueError):
        return jsonify({
            "success": False,
            "message": "battery 값은 숫자여야 합니다."
        }), 400

    battery = max(0, min(100, battery))

    vehicle_status["battery"] = battery

    if battery <= 20:
        vehicle_status["driving_status"] = "충전 필요"

    return jsonify({
        "success": True,
        "battery": battery,
        "message": "배터리 상태가 변경되었습니다."
    })


# ============================================================
# 8. 입차 경로 생성 요청
# ============================================================

@app.route("/api/route/entry", methods=["POST"])
def create_entry_route():
    data = request.get_json(silent=True) or {}

    vehicle_id = data.get(
        "vehicle_id",
        vehicle_status["vehicle_id"]
    )

    # 지금은 주차공간을 임시로 parking_1에 배정
    # 나중에 빈 주차공간 판단 코드와 연결
    selected_parking_slot = "parking_1"

    vehicle_status["vehicle_id"] = vehicle_id
    vehicle_status["destination"] = selected_parking_slot
    vehicle_status["route_state"] = "경로 생성 완료"
    vehicle_status["driving_status"] = "입차 준비"
    vehicle_status["last_command"] = "entry"

    print("=" * 60)
    print("[입차 명령 수신]")
    print("차량:", vehicle_id)
    print("목적지:", selected_parking_slot)
    print("명령: ENTRY")
    print("=" * 60)

    # 나중에 들어갈 부분
    # publish_route_command(
    #     command="entry",
    #     destination=selected_parking_slot
    # )

    return jsonify({
        "success": True,
        "command": "entry",
        "vehicle_id": vehicle_id,
        "destination": selected_parking_slot,
        "route_state": "경로 생성 완료",
        "message": (
            f"{selected_parking_slot}까지 "
            "입차 경로가 생성되었습니다."
        )
    })


# ============================================================
# 9. 출차 경로 생성 요청
# ============================================================

@app.route("/api/route/exit", methods=["POST"])
def create_exit_route():
    data = request.get_json(silent=True) or {}

    vehicle_id = data.get(
        "vehicle_id",
        vehicle_status["vehicle_id"]
    )

    exit_destination = "exit_waiting_area"

    vehicle_status["vehicle_id"] = vehicle_id
    vehicle_status["destination"] = exit_destination
    vehicle_status["route_state"] = "경로 생성 완료"
    vehicle_status["driving_status"] = "출차 준비"
    vehicle_status["last_command"] = "exit"

    print("=" * 60)
    print("[출차 명령 수신]")
    print("차량:", vehicle_id)
    print("목적지:", exit_destination)
    print("명령: EXIT")
    print("=" * 60)

    # 나중에 들어갈 부분
    # publish_route_command(
    #     command="exit",
    #     destination=exit_destination
    # )

    return jsonify({
        "success": True,
        "command": "exit",
        "vehicle_id": vehicle_id,
        "destination": "출차 대기구역",
        "route_state": "경로 생성 완료",
        "message": "출차 대기구역까지 경로가 생성되었습니다."
    })


# ============================================================
# 10. 경로 생성 상태 초기화
# ============================================================

@app.route("/api/route/reset", methods=["POST"])
def reset_route():
    vehicle_status["destination"] = None
    vehicle_status["route_state"] = "생성 전"
    vehicle_status["driving_status"] = "대기 중"
    vehicle_status["last_command"] = None

    return jsonify({
        "success": True,
        "message": "경로 상태가 초기화되었습니다."
    })


# ============================================================
# 11. Flask 실행
# ============================================================

if __name__ == "__main__":
    threading.Thread(
        target=run_battery_subscriber,
        daemon=True,
    ).start()

    threading.Thread(
        target=refresh_battery_connection_state,
        daemon=True,
    ).start()

    print("=" * 60)
    print("스마트 주차 관제 웹 서버 시작")
    print("접속 주소: http://127.0.0.1:5001")
    print(f"YOLO 비전 모드: {'활성' if generate_frames else '사용 불가'}")
    print("=" * 60)

    app.run(
        host="0.0.0.0",
        port=5001,
        debug=True,
        threaded=True,
        use_reloader=False
    )
