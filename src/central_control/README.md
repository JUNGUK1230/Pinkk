# PINKK Central Control

상단 카메라에서 차량과 주차면을 인식하고, LiDAR 지도에서 T자 후진주차
경로를 생성해 ROS 2 토픽으로 전달하는 중앙제어 모듈입니다.

## 설치

기준 환경은 Ubuntu 24.04, Python 3.12, ROS 2 Jazzy입니다. PINKK 프로젝트
루트에서 Python 가상환경과 중앙제어 의존성을 설치합니다.

```bash
cd ~/PINKK
python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install --upgrade pip
python3 -m pip install -r requirements.txt
```

실시간 YOLO segmentation 학습 가중치는 다음 위치에 `best.pt`라는 이름으로
둡니다.

```text
src/central_control/models/best.pt
```

카메라 실행 전 다음 보정·정합 파일이 준비되어 있어야 합니다.

```text
src/central_control/camera_tools/first_map/camera_calibration.npz
src/central_control/camera_tools/first_map/bev_homography.npz
src/central_control/camera_tools/first_map/camera_to_lidar_rigid_registration.npz
src/central_control/camera_tools/first_map/my_test_map0710.yaml
src/central_control/camera_tools/first_map/my_test_map0710.png
```

ROS 2 publisher를 사용할 때는 프로젝트를 빌드합니다.

```bash
cd ~/PINKK
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 실행

아래 프로세스는 각각 별도 터미널에서 실행합니다.

### 1. 상단 카메라 YOLO localization

```bash
cd ~/PINKK
.venv/bin/python -m src.central_control.overhead_vision.localization.live_localization
```

창에서 `h`를 누르고 차량 앞쪽을 클릭하면 현재 차량 heading이 지정됩니다.
`x`는 heading 초기화, `q` 또는 `ESC`는 종료입니다.

### 2. T자 후진주차 경로 생성

```bash
cd ~/PINKK/src/central_control/path_planning
~/PINKK/.venv/bin/python scripts/plan_from_live_vision.py
```

최신 차량 pose와 선택된 빈 주차면으로 경로를 한 번 생성합니다. 충돌 검사와
trajectory validator를 통과한 결과만 `output/live_hybrid_path_world_cm.json`에
저장됩니다.

### 3. 검증된 경로 토픽 발행

```bash
cd ~/PINKK
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run pinkk trajectory_publisher
```

### 4. 현재 차량 pose 토픽 발행

```bash
cd ~/PINKK
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run pinkk vehicle_pose_publisher
```

발행 토픽은 다음과 같습니다.

- `/pinkk/planned_path`: `nav_msgs/Path`, m 단위, `lidar_map` frame
- `/pinkk/planned_trajectory`: 전후진·속도·조향·정지 정보
- `/pinkk/vehicle_pose`: `geometry_msgs/PoseStamped`, 현재 rear-axle pose

### Camera BEV 다시 저장

```bash
cd ~/PINKK/src/central_control/camera_tools/first_map
~/PINKK/.venv/bin/python capture_camera_bev.py
```

`s`를 누르면 `camera_bev.png`를 저장하고 `q` 또는 `ESC`로 종료합니다.

## 알고리즘 개요

1. USB 카메라를 별도 스레드에서 계속 읽고, 밀린 프레임은 버린 뒤 가장
   최신 프레임만 렌즈 왜곡 보정하여 `1600×800` Camera BEV로 변환합니다.
   화면의 `capture age`로 촬영부터 표시까지의 지연을 확인할 수 있습니다.
2. YOLO segmentation과 ByteTrack으로 차량별 `track_id`를 유지하고,
   처음 선택한 ego ID가 잠시 가려졌을 때 다른 차량으로 바뀌지 않게 합니다.
   Camera–LiDAR affine 정합으로 해당 ego의 rear-axle 위치를 LiDAR map cm
   좌표로 변환합니다.
3. 입구 단계에서는 P6~P10의 빈 주차면을 입구에서 먼 순서로 선택합니다.
   현재 충전 이동 단계에서는 C2를 우선 선택하고, C2 점유 시 C1을 선택합니다.
4. 2D A* 통로 guide를 짧은 Hybrid A* 구간으로 나눠 장거리 탐색량을
   줄이고, 각 구간 경계에서 정지·조향 재설정점을 만듭니다.
5. Occupancy grid의 검은 영역을 벽으로 사용하고, 차량 `12×10 cm` 회전
   footprint와 전진·후진을 고려하는 Hybrid A*로 경로를 탐색합니다.
6. 주차면 앞 staging pose까지 접근한 뒤 정지하고, T자 maneuver로 마지막
   구간을 후진해 주차합니다.
7. 경로를 0.5cm 이하 간격으로 생성하고 곡률, 목표속도, 조향각, 정지점을
   계산합니다.
8. 충돌·간격·yaw·조향·속도 제한을 통과한 trajectory만 ROS 2로 발행합니다.
