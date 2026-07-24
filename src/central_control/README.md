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

상단 카메라·YOLO·Hybrid A*·ROS 2 토픽 발행은 한 프로세스로 실행합니다.

```bash
cd ~/PINKK
source /opt/ros/jazzy/setup.bash
source install/setup.bash
.venv/bin/python -m src.central_control.overhead_vision.localization.live_localization
```

창에서 `h`를 누르고 차량 앞쪽을 클릭하면 현재 차량 heading이 지정됩니다.
그러면 검증된 Hybrid A*가 별도 작업 스레드에서 바로 시작되고, 성공 경로는
화면에 빨간 선으로 표시되며 ROS 2 토픽으로 즉시 발행됩니다. `p`는 같은
차량·목표로 재계획, `x`는 heading 초기화, `q` 또는 `ESC`는 종료입니다.

발행 토픽은 다음과 같습니다.

- `/pinkk/planned_path`: `nav_msgs/Path`, m 단위, `lidar_map` frame
- `/pinkk/planned_trajectory`: 전후진·속도·조향·정지 정보
- `/pinkk/vehicle_pose`: `geometry_msgs/PoseStamped`, 현재 rear-axle pose

기본 통합 실행은 trajectory CSV·JSON·Camera overlay 파일을 저장하지
않습니다. 진단용 파일 저장이 필요할 때만
`config/yolo/realtime_localization.yaml`의 `write_runtime_files`를 `true`로
바꿉니다. 파일 기반 단독 진단은 기존
`path_planning/scripts/plan_from_live_vision.py`를 사용합니다.

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
3. 각 `track_id` 차량은 중심이 포함된 주차칸으로 `entry_or_transit`,
   `waiting_for_charge`(P6~P10), `charging`(C1·C2),
   `charged_waiting_exit`(P1~P5) 상태로 분류해 메모리 scene에 유지합니다.
   짧은 가림은 기본 2초 동안 유지하며, 진단 파일 저장 모드에서만 JSON에도
   기록합니다.
4. 충전 Episode는 `entry_or_transit`·`waiting_for_charge` 차량을 입차 시각
   기준 FIFO로 선택합니다. 빈 충전칸은 C2를 우선하고 C2 점유 시 C1을 쓰며,
   배정된 ego만 기존 경로 계획 입력을 생성합니다.
5. 입구 단계에서 충전칸이 모두 차 있으면 P6~P10의 빈 주차면을 입구에서 먼
   순서로 선택합니다. 충전 완료·출차 대기 단계의 실제 전환은 아직 외부
   충전 완료 신호를 연결하기 전의 다음 작업입니다.
6. 2D A* 통로 guide를 짧은 Hybrid A* 구간으로 나눠 장거리 탐색량을
   줄이고, 각 구간 경계에서 정지·조향 재설정점을 만듭니다.
7. Occupancy grid의 검은 영역을 벽으로 사용하고, 차량 `12×10 cm` 회전
   footprint와 전진·후진을 고려하는 Hybrid A*로 경로를 탐색합니다.
8. 주차면 앞 staging pose까지 접근한 뒤 정지하고, T자 maneuver로 마지막
   구간을 후진해 주차합니다.
9. 경로를 0.5cm 이하 간격으로 생성하고 곡률, 목표속도, 조향각, 정지점을
   계산합니다.
10. 충돌·간격·yaw·조향·속도 제한을 통과한 trajectory만 파일 중계 없이
    ROS 2 토픽으로 직접 발행합니다.
