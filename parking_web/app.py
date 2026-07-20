from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
from flask import Flask, Response, jsonify, render_template, request


# ============================================================
# 1. Flask 기본 설정
# ============================================================

BASE_DIR = Path(__file__).resolve().parent

app = Flask(
    __name__,
    template_folder=str(BASE_DIR / "templates")
)


# ============================================================
# 2. 임시 차량 상태
# 나중에 ROS2 토픽 데이터로 교체
# ============================================================

vehicle_status = {
    "vehicle_id": "PINKY_01",
    "battery": 78,
    "charging": False,
    "driving_status": "대기 중",
    "current_location": "입구 대기구역",
    "destination": None,
    "route_state": "생성 전",
    "last_command": None
}


# ============================================================
# 3. 임시 카메라 화면 생성
# 현재는 맥북 카메라를 사용하지 않음
# ============================================================

def create_placeholder_frame():
    """
    실제 상단 카메라를 연결하기 전까지
    웹 화면에 표시할 임시 영상을 생성한다.
    """

    width = 1280
    height = 720

    frame = np.zeros((height, width, 3), dtype=np.uint8)

    # 배경
    frame[:] = (20, 28, 45)

    # 주차장 형태의 임시 구역
    cv2.rectangle(
        frame,
        (80, 100),
        (1200, 650),
        (100, 116, 139),
        3
    )

    # 주차공간 예시
    start_x = 150
    start_y = 180
    slot_width = 170
    slot_height = 200
    gap = 30

    for index in range(5):
        x1 = start_x + index * (slot_width + gap)
        y1 = start_y

        x2 = x1 + slot_width
        y2 = y1 + slot_height

        cv2.rectangle(
            frame,
            (x1, y1),
            (x2, y2),
            (71, 85, 105),
            2
        )

        cv2.putText(
            frame,
            f"P{index + 1}",
            (x1 + 60, y1 + 110),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (148, 163, 184),
            2
        )

    # 안내 문구
    cv2.putText(
        frame,
        "TOP CAMERA NOT CONNECTED",
        (350, 500),
        cv2.FONT_HERSHEY_SIMPLEX,
        1.2,
        (226, 232, 240),
        3
    )

    cv2.putText(
        frame,
        "Waiting for external parking camera",
        (390, 550),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (148, 163, 184),
        2
    )

    # 현재 시간
    current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    cv2.putText(
        frame,
        current_time,
        (880, 690),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (148, 163, 184),
        1
    )

    return frame


def generate_placeholder_video():
    """
    임시 화면을 MJPEG 영상 형식으로 계속 전송한다.
    맥북 내장 카메라에는 접근하지 않는다.
    """

    while True:
        frame = create_placeholder_frame()

        success, buffer = cv2.imencode(".jpg", frame)

        if not success:
            continue

        frame_bytes = buffer.tobytes()

        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n"
            + frame_bytes
            + b"\r\n"
        )


# ============================================================
# 4. 메인 웹 화면
# ============================================================

@app.route("/")
def index():
    return render_template("index.html")


# ============================================================
# 5. 임시 카메라 스트리밍
# ============================================================

@app.route("/video_feed")
def video_feed():
    return Response(
        generate_placeholder_video(),
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
# 7. 배터리 상태 변경 API
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
    print("=" * 60)
    print("스마트 주차 관제 웹 서버 시작")
    print("접속 주소: http://127.0.0.1:5001")
    print("현재 카메라 모드: 임시 화면")
    print("=" * 60)

    app.run(
        host="0.0.0.0",
        port=5001,
        debug=True,
        threaded=True,
        use_reloader=False
    )