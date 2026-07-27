# 주차 관제 Flask 웹 서버

상단 카메라 영상에 왜곡 보정, BEV 변환 및 YOLO 차량 인식을 적용하고
LiDAR 맵과 함께 Flask 웹페이지로 전송합니다.

## Git 제외 항목

가상환경과 `pip`으로 설치되는 라이브러리는 저장소에 커밋하지 않습니다.

- `venv/`, `.venv/`, `env/`, `.env/`
- `site-packages/`, `*.dist-info/`
- Python 캐시와 빌드 결과
- YOLO 모델 파일(`*.pt`, `*.onnx`, `*.engine`)

특히 `best.pt`는 Git에서 제외되므로 장비에 별도로 복사해야 합니다.

## 필수 입력 파일

YOLO 비전 기능을 실행하려면 다음 파일이 모두 필요합니다.

| 용도 | 경로 |
|---|---|
| YOLO 모델 | `src/central_control/models/best.pt` |
| 카메라 보정 | `src/central_control/camera_tools/first_map/camera_calibration.npz` |
| BEV 변환 | `src/central_control/camera_tools/first_map/bev_homography.npz` |
| 카메라-LiDAR 정합 | `src/central_control/camera_tools/first_map/camera_to_lidar_rigid_registration.npz` |
| LiDAR 배경 맵 | `src/central_control/camera_tools/first_map/my_test_map0710.png` |
| 주차면 좌표 | `src/central_control/config/map/parking_slots_bev.json` |

현재 카메라 장치 번호는
`live_yolo_bev_map_web_separate.py`의 `CAMERA_ID`에서 설정합니다.

## 가상환경 생성

저장소 루트에서 실행합니다.

```bash
python3 -m venv src/central_control/parking_management_web/venv
```

ROS 2 환경을 먼저 불러와야 합니다.

```bash
source /opt/ros/jazzy/setup.bash
```

## Flask 기본 설치

웹 UI와 HTTP 로봇 상태 API만 사용할 때 설치합니다. 중앙 서버에는 ROS 2가
설치되어 있지 않아도 됩니다.

```bash
src/central_control/parking_management_web/venv/bin/pip install \
  -r src/central_control/parking_management_web/requirements.txt
```

## YOLO 비전 설치

실제 YOLO 및 BEV 스트림을 사용할 때 추가로 설치합니다. CPU 전용
PyTorch를 사용하므로 대용량 CUDA 라이브러리는 설치하지 않습니다.

```bash
src/central_control/parking_management_web/venv/bin/pip install \
  -r requirements-vision.txt
```

## Flask 서버 실행

먼저 API 키를 설정합니다. `.env.example`은 참고용이며 Flask가 자동으로
읽지 않으므로 실제 운영 환경이나 systemd에 환경변수를 설정해야 합니다.

```bash
export PINKK_API_KEY='충분히-긴-임의의-비밀값'
export PINKK_ADMIN_KEY='로봇-API키와-다른-관리자-비밀값'
PINKK_ENABLE_VISION=1 \
src/central_control/parking_management_web/venv/bin/python \
src/central_control/parking_management_web/app_management_battery.py
```

브라우저에서 다음 주소로 접속합니다.

```text
http://127.0.0.1:5001
```

다른 장치에서는 `127.0.0.1` 대신 서버의 IP 주소를 사용합니다.

공개 서버에서는 Flask 개발 서버 대신 HTTPS를 종료하는 리버스 프록시
뒤에서 Gunicorn을 실행합니다.

```bash
PYTHONPATH=src gunicorn --workers 1 --threads 4 --bind 127.0.0.1:5001 \
  central_control.parking_management_web.app_management_battery:app
```

`PINKK_ALLOWED_ORIGINS`에는 필요할 때만 쉼표로 구분한 웹 출처를 넣습니다.
동일 출처의 기본 관제 화면은 CORS 설정이 필요하지 않습니다.

## 운영 노트북 배터리 전송

기존 코드에서 사용하는 실제 토픽은 `std_msgs/msg/Float32` 타입의
`/battery/percent`, `/battery/voltage`입니다. 각 PinkyPro 운영 노트북에서
ROS 2 Jazzy 환경을 불러오고 HTTP 의존성을 설치합니다.

```bash
python3 -m pip install -r robot_client/requirements.txt
source /opt/ros/jazzy/setup.bash
export ROBOT_ID=pinky1                 # 두 번째 노트북은 pinky2
export CENTRAL_SERVER_URL=https://parking.example.com
export PINKK_API_KEY='서버와-같은-비밀값'
python3 robot_client/battery_bridge.py
```

토픽 이름이 장비에서 다르면 `PINKK_BATTERY_PERCENT_TOPIC`과
`PINKK_BATTERY_VOLTAGE_TOPIC`으로 변경할 수 있습니다. 전송은 기본 1.5초
간격이며 HTTP 처리는 별도 스레드에서 실행되어 서버 장애가 ROS 구독을
중단시키지 않습니다. 요청은 3초 타임아웃과 최대 3회 재시도를 사용합니다.

ROS 없이 전체 HTTP 흐름을 시험하려면 같은 환경변수로 실행합니다.

```bash
python3 robot_client/fake_battery_sender.py
```

상태 수신은 `POST /api/robots/status`, 웹 조회는
`GET /api/robots/status`입니다. 마지막 수신 후 5초부터 `delayed`, 10초부터
`offline`으로 표시됩니다. 현재 저장소는 초기 단계의 메모리 방식이므로
서버 재시작 시 상태가 초기화됩니다. 또한 여러 프로세스가 메모리를 공유하지
않으므로 현재 단계에서는 Gunicorn worker를 1개만 사용하고, 다중 worker가
필요해질 때 SQLite나 외부 데이터베이스로 저장소를 교체해야 합니다.

## 입차·출차·긴급정지 명령

관제 화면에서 대상 로봇과 `PINKK_ADMIN_KEY` 값을 입력하고 입차, 출차 또는
긴급정지를 누르면 명령이 SQLite의
`instance/robot_commands.db`에 `pending` 상태로 저장됩니다. 운영 노트북은
기본 1초마다 다음 API를 조회합니다.

```text
POST /api/robots/{robot_id}/commands
GET  /api/robots/{robot_id}/commands/next
POST /api/robots/{robot_id}/commands/{command_id}/result
GET  /api/robots/commands/history
```

운영 노트북이 가져간 명령은 JSON 문자열로 `/pinkk/command` 토픽에
발행됩니다. 다른 토픽을 사용하려면 `PINKK_COMMAND_TOPIC`을 설정합니다.
메시지에는 `command_id`, `robot_id`, `command`, `parameters`, 생성 시각이
포함됩니다. 차량 제어 노드는 이 토픽을 구독해 `entry`, `exit`,
`emergency_stop`을 실제 동작과 연결해야 합니다.

긴급정지는 대기열에서 다른 명령보다 먼저 전달되지만 네트워크, 서버 또는
운영 노트북 장애 시 도달하지 않을 수 있습니다. 물리 버튼이나 차량 내부
watchdog 같은 로컬 안전정지는 반드시 별도로 유지해야 합니다.

## 비전 처리 파일 단독 실행

```bash
PYTHONPATH=src \
src/central_control/parking_management_web/venv/bin/python \
src/central_control/parking_management_web/live_yolo_bev_map_web_separate.py
```

단독 실행은 데스크톱 OpenCV 창을 사용하며, 종료 키는 `q`입니다.

## 문제 확인

```bash
# Flask 확인
src/central_control/parking_management_web/venv/bin/python \
  -c "from flask import Flask; print(Flask)"

# OpenCV 확인
src/central_control/parking_management_web/venv/bin/python \
  -c "import cv2; print(cv2.__version__)"

# YOLO 확인
src/central_control/parking_management_web/venv/bin/python \
  -c "from ultralytics import YOLO; print(YOLO)"
```

`FileNotFoundError: .../models/best.pt`가 발생하면 필수 모델 파일이 아직
복사되지 않은 상태입니다.
