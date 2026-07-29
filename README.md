# Smart Parking System

Python 기반 스마트 주차 및 자동 충전 프로젝트 기본 구조입니다.

## 주요 모듈

- `src/robot_arm`: 로봇팔 카메라, 동작 제어 및 설정
- `src/central_control`: 상단 카메라, BEV, 지도, 경로 계획, 모델, 시스템 통합 및 설정
- `src/vehicle_control`: 차량 제어 코드 및 설정
- `ros2_ws/src/pinkk_usb_insertion`: USB-A 포트 인식, 접근, 정렬 및 삽입 제어
- `ros2_ws/src/pinkk_usb_insertion_interfaces`: YOLO keypoint와 포트 관측 ROS 메시지

로봇팔 USB 정렬 제어의 실행 방법은
[`pinkk_usb_insertion/README.md`](ros2_ws/src/pinkk_usb_insertion/README.md)를
참고하세요. 캘리브레이션 패키지와 실제 운용 제어 패키지는 분리해서 관리합니다.
현재 완료된 기능, 문제와 다음 계획은
[`USB 포트 정렬 개발일지`](ros2_ws/src/pinkk_usb_insertion/docs/DEVELOPMENT_LOG_KO.md)에서
관리합니다.

프로젝트 루트에는 안내 문서인 `README.md`와 라이브러리 의존성 파일인
`requirements.txt`만 두고, 실행 코드와 데이터는 위 세 영역 안에서 관리합니다.

## 설치

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## 실행 예시

카메라 확인:

```bash
python3 -m src.central_control.overhead_vision.camera.camera_capture
```

전체 파이프라인:

```bash
python3 -m src.central_control.main
```

## 주의

- YOLO 모델은 `src/central_control/models`에서 관리합니다.
- LiDAR 대용량 맵 및 실행 결과는 필요에 따라 별도 관리하세요.
