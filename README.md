# Smart Parking System

상단 카메라 기반 주차 감지·BEV/LiDAR 지도 정합·경로 계획과 ROS 2 차량 제어를 위한 프로젝트입니다. 프로젝트는 단일 ROS 2 Python 패키지 `pinkk`로 구성되어 있습니다.

## 구성

- `src/central_control`: 상단 카메라, 보정·BEV, YOLO 주차 감지, 지도 정합 및 경로 계획
- `src/vehicle_control`: 고정 경로와 차량 중심 pose를 받아 차동구동 명령을
  계산하는 MPC 경로 추종 노드
- `src/robot_arm`: 로봇팔 카메라·동작 제어 관련 코드와 설정

## Python 의존성 설치

카메라·YOLO·경로 계획 도구를 실행할 때 필요합니다.

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## ROS 2 빌드 및 실행

ROS 2 Jazzy 환경에서 프로젝트 루트에서 실행합니다.

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 차량 MPC 경로 추종

상단 카메라 x/y와 LiDAR map heading을 결합한
`/pinkk/fused_vehicle_pose`를 MPC가 사용합니다. MPC는
`/pinkk/planned_trajectory`, `/pinkk/fused_vehicle_pose`, `/scan`을 구독하고
실차 `/cmd_vel`을 발행합니다. 다른 속도 제어기는 반드시 종료해야 합니다.

```bash
cd ~/PINKK
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=36

ros2 run pinkk mpc_path_follower \
  --ros-args \
  --params-file src/vehicle_control/config/mpc/mpc.yaml
```

먼저 오프라인 시뮬레이션을 실행합니다.

```bash
.venv/bin/python src/vehicle_control/tests/test_mpc_controller.py
```

실차 `/cmd_vel` 연결 방법과 안전 조건은
[`src/vehicle_control/README.md`](src/vehicle_control/README.md)를 참고하세요.

### 중앙 제어 진입점

```bash
ros2 run pinkk central_control
```

현재 중앙 제어 파이프라인은 모듈을 연결하기 위한 골격으로, 실행 시 안내 메시지만 출력합니다.

## 개발 도구 실행

프로젝트 루트에서 실행합니다.

상단 카메라 미리보기(기본 장치 번호·해상도는 `src/central_control/config/camera/camera.yaml`에서 설정):

```bash
python3 -m src.central_control.overhead_vision.camera.camera_capture
```

YOLO 세그멘테이션·BEV·LiDAR 지도 표시:

```bash
./src/central_control/scripts/run_yolo_seg.sh
```

이 스크립트는 `.venv`와 `/dev/video2`를 확인합니다. YOLO 가중치 파일은 `src/central_control/models/best.pt`에 두어야 하며, 모델 파일은 Git에서 제외됩니다.

### 상단 카메라 실시간 차량 위치·주차면 좌표

새 localization 파이프라인은 USB 카메라 원본을 왜곡 보정한 뒤 `1600×800` BEV로 변환하고, `best.pt` 차량 segmentation mask를 Camera BEV에서 LiDAR map 좌표로 변환합니다. 차량 위치와 제어 경로는 **차량 중심 cm pose**로 통일하며, 고정 주차면은 차량 mask와 겹친 비율로 점유 여부를 판단합니다.

```bash
cd ~/PINKK
.venv/bin/python -m src.central_control.overhead_vision.localization.live_localization
```

직접 파일 실행도 지원하지만 YOLO·OpenCV가 설치된 프로젝트 `.venv`의 module 실행을 권장합니다. 시스템 `python3`를 사용하면 환경에 따라 `ultralytics`가 없을 수 있습니다.

최신 결과는 `src/central_control/path_planning/output/live_vision_scene.json`에 매 프레임 원자적으로 저장됩니다. 다른 터미널에서 경로계획 입력을 확인할 수 있습니다.

```bash
cd ~/PINKK/src/central_control/path_planning
python3 scripts/read_live_vision_scene.py
```

차량이 발견된 fresh scene으로 한 번 자동 경로를 생성하려면 localization을 계속 실행한 상태에서 다음 명령을 사용합니다. 현재 운영 슬롯은 상단 `P5~P8`, 하단 `P1~P4`, 충전 구역 `C1/C2`입니다.

실시간 계획은 전체 기본 30초, goal 후보당 기본 5초로 제한됩니다. 탐색 중에는 후보 번호와 확장 노드 수가 출력되며, 진단 실행에서만 `--planning-timeout-sec 0 --candidate-timeout-sec 0`으로 무제한을 지정합니다.

Hybrid 탐색은 차량 rectangle 바깥에 2cm obstacle inflation을 추가해, 폭 10cm 차량의 중심 경로가 측면 장애물에서 약 7cm 이상 떨어지도록 합니다. 폭 20cm 주차면 안에서 양쪽 안전공간을 유지하면서 smoothing 경로가 벽을 침범하지 않게 확보하는 여유입니다.

```bash
cd ~/PINKK/src/central_control/path_planning
python3 scripts/plan_from_live_vision.py
```

생성 파일은 `output/live_hybrid_path_world_cm.csv`, `output/live_hybrid_path_camera_bev.csv`, `output/live_hybrid_path_world_cm.json` 및 `output/live_hybrid_path_on_camera_bev.png`입니다. 경로 이미지는 최신 `live_camera_bev.png` 위에 빨간 경로, 초록 start heading, 파란 goal heading을 표시합니다. 이번 단계는 한 번 계획하고 종료하는 기본 파이프라인이며 제어기로 전송하지 않습니다. 실패 시 이전 자동 경로와 overlay를 제거하고 `output/live_hybrid_planning_status.json`에 차단 이유를 기록합니다.

화면에 차량이 여러 대면 localization 창에서 ego 차량을 선택해 해당 차량의 현재 섹션과 고정 Yaw를 기준으로 경로를 선택합니다.

저장된 BEV 이미지로 카메라 없이 전체 변환을 확인할 수도 있습니다.

```bash
cd ~/PINKK
.venv/bin/python -m src.central_control.overhead_vision.localization.live_localization \
  --bev-image src/central_control/camera_tools/first_map/camera_bev.png \
  --initial-ego-center 67 585 \
  --initial-yaw-deg -117 \
  --no-display
```

`yolo11l.pt` 기본 가중치는 detection 모델이므로 mask 기반 헤딩·주차면 점유 계산에는 사용할 수 없습니다. 라벨링과 재학습이 끝나면 동일한 `car` class를 가진 YOLO11 Large **segmentation** 가중치로 `model_path`만 교체합니다.

기타 카메라 보정·BEV·지도 정합·경로 계획 도구의 사용법은 [중앙 제어 도구 문서](src/central_control/camera_tools/README.md), [LiDAR 지도 문서](src/central_control/map/lidar_map/README.md), [경로 계획 문서](src/central_control/path_planning/README.md)를 참고하세요.

## 참고

- ROS 2 패키지 메타데이터는 `src/package.xml`, 패키지 실행 명령은 `src/setup.py`에서 관리합니다.
- 카메라 장치 번호는 일반 카메라 미리보기 설정(`camera.yaml`)과 YOLO 실행 스크립트(`/dev/video2`)가 서로 다를 수 있으므로, 연결된 장치를 확인한 뒤 맞춰 사용하세요.
