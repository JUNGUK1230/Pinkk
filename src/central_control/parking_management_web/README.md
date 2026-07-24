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

웹 UI와 ROS 배터리 상태 API만 사용할 때 설치합니다.

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

```bash
PINKK_ENABLE_VISION=1 \
src/central_control/parking_management_web/venv/bin/python \
src/central_control/parking_management_web/app_management_battery.py
```

브라우저에서 다음 주소로 접속합니다.

```text
http://127.0.0.1:5001
```

다른 장치에서는 `127.0.0.1` 대신 서버의 IP 주소를 사용합니다.

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
