# Smart Parking System

상단 카메라 기반 주차 감지·BEV/LiDAR 지도 정합·경로 계획과 ROS 2 차량 제어를 위한 프로젝트입니다. 프로젝트는 단일 ROS 2 Python 패키지 `pinkk`로 구성되어 있습니다.

## 구성

- `src/central_control`: 상단 카메라, 보정·BEV, YOLO 주차 감지, 지도 정합 및 경로 계획
- `src/vehicle_control`: `/odom`을 받아 `/cmd_vel`을 발행하는 PID 경로 추종 노드
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

### 차량 PID 경로 추종

`pid_path_follower`는 기본적으로 `/odom`을 구독하고 `/cmd_vel`을 발행합니다. 웨이포인트와 제어 이득은 현재 노드 코드에 정의돼 있습니다.

```bash
ros2 run pinkk pid_path_follower
```

토픽을 바꾸려면 ROS 파라미터를 전달합니다.

```bash
ros2 run pinkk pid_path_follower --ros-args \
  -p odom_topic:=/odom \
  -p cmd_vel_topic:=/cmd_vel
```

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

새 localization 파이프라인은 USB 카메라 원본을 왜곡 보정한 뒤 `1600×800` BEV로 변환하고, `best.pt` 차량 segmentation mask를 Camera BEV에서 LiDAR map 좌표로 변환합니다. 차량 위치는 Hybrid A*와 같은 **rear axle 중심 cm pose**로 출력하며, 고정 주차면은 차량 mask와 겹친 비율로 점유 여부를 판단합니다.

```bash
cd ~/PINKK
.venv/bin/python -m src.central_control.overhead_vision.localization.live_localization
```

최신 결과는 `src/central_control/path_planning/output/live_vision_scene.json`에 매 프레임 원자적으로 저장됩니다. 다른 터미널에서 경로계획 입력을 확인할 수 있습니다.

```bash
cd ~/PINKK/src/central_control/path_planning
python3 scripts/read_live_vision_scene.py
```

차량이 발견된 fresh scene으로 한 번 자동 경로를 생성하려면 localization을 계속 실행한 상태에서 다음 명령을 사용합니다. 가장 가까운 빈 주차면의 두 진입 헤딩을 순서대로 시도하며, footprint 보정·곡률 smoothing·속도 프로파일·trajectory validator를 모두 통과한 경로만 저장합니다.

```bash
cd ~/PINKK/src/central_control/path_planning
python3 scripts/plan_from_live_vision.py
```

생성 파일은 `output/live_hybrid_path_world_cm.csv`, `output/live_hybrid_path_camera_bev.csv`, `output/live_hybrid_path_world_cm.json`입니다. 이번 단계는 한 번 계획하고 종료하는 기본 파이프라인이며 제어기로 전송하지 않습니다. 실패 시 이전 자동 경로 파일을 제거하고 `output/live_hybrid_planning_status.json`에 차단 이유를 기록합니다.

현재 차량 segmentation 장축만으로는 앞/뒤가 180° 모호하며, 화면에 차량이 여러 대면 ego 차량을 처음부터 구분할 단서가 필요합니다. 최초 설치 시 `src/central_control/config/yolo/realtime_localization.yaml`에 `initial_ego_center_bev_px`와 LiDAR 좌표계 기준 `initial_ego_yaw_deg`를 설정하십시오. 값이 없거나 검출 결과가 모호하면 화면과 JSON에는 관측 좌표를 남기지만 `planning_ready=false`로 만들어 경로계획 입력을 차단합니다.

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
