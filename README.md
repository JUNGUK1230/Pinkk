# Smart Parking System

Python 기반 스마트 주차 및 자동 충전 프로젝트 기본 구조입니다.

## 주요 모듈

- `src/overhead_vision`: 상단 카메라, BEV, LiDAR 맵 정합, YOLO, 경로 생성
- `src/vehicle_control`: 차량 제어 담당 코드
- `src/robot_arm/robot_camera`: 로봇팔 장착 카메라 및 비전 처리
- `src/robot_arm/motion_control`: 로봇팔 관절 및 충전 동작 제어
- `src/system_manager`: 전체 시스템 통합 및 상태 관리
- `config`: 실행 파라미터
- `camera/calibration`: 상단 카메라 촬영 및 렌즈 보정 도구
- `camera/bird_eye_view`: 상단 카메라 BEV 변환 및 좌표 도구
- `map/lidar_map`: LiDAR 맵 촬영 결과를 확인하는 작업 공간
- `maps`: LiDAR / Camera / Fused Map
- `models`: YOLO 모델 가중치 위치
- `scripts`: 실행 스크립트
- `tests`: 테스트 코드
- `output`: 실행 결과 저장 경로

## 설치

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## 실행 예시

카메라 확인:

```bash
python3 -m src.overhead_vision.camera.camera_capture
```

전체 파이프라인:

```bash
python3 main.py
```

## 주의

- YOLO `.pt` 모델 파일은 기본적으로 Git에 올리지 않도록 설정되어 있습니다.
- LiDAR 대용량 맵 및 실행 결과는 필요에 따라 별도 관리하세요.
