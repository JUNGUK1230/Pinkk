# PINKK Central Control

상단 카메라로 차량의 현재 section을 찾고, 미리 생성한 주차장 고정 경로를
ROS 2 토픽으로 전달하는 중앙제어 모듈입니다. 실시간 운행에서는 차량 heading을
클릭하지 않으며, 차량 마스크 장축을 Camera–LiDAR 정합으로 변환해 yaw를
측정합니다. `START`와 각 주차면의 고정 yaw는 앞·뒤 판별 기준으로 사용합니다.

## 설치

기준 환경은 Ubuntu 24.04, Python 3.12, ROS 2 Jazzy입니다.

```bash
cd ~/PINKK
python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install --upgrade pip
python3 -m pip install -r requirements.txt
```

YOLO segmentation 가중치는 다음 위치에 둡니다.

```text
src/central_control/models/best.pt
```

카메라 실행 전 다음 보정·정합 파일이 필요합니다.

```text
src/central_control/camera_tools/first_map/camera_calibration.npz
src/central_control/camera_tools/first_map/bev_homography.npz
src/central_control/camera_tools/first_map/camera_to_lidar_rigid_registration.npz
src/central_control/camera_tools/first_map/my_test_map0710.yaml
src/central_control/camera_tools/first_map/my_test_map0710.png
```

## 고정 경로 준비

운영 경로는 `path_planning/output/fixed_route_*_to_*.csv` 32개입니다. 경로
설정이나 주차장 구조를 변경한 경우에만 다시 생성합니다.

```bash
cd ~/PINKK/src/central_control/path_planning
python3 scripts/test_fixed_mission_routes.py --generate-all
python3 scripts/test_fixed_mission_routes.py --check-all
python3 scripts/test_fixed_route_selector.py
python3 scripts/test_fixed_live_route_bridge.py
```

허용 이동 관계는 다음과 같습니다.

- `START` → `P8`~`P5`, `C1`, `C2`
- `P8`~`P5` → `C1`, `C2`
- `C1`, `C2` → `P1`~`P4`
- `P1`~`P4` → `EXIT`

동일 라인 내부 전이(`P8`→`P7`, `C1`→`C2`, `P1`→`P2` 등)는 허용하지
않으며, 다른 차량의 충전 배정은 현재 ego의 `TARGET`으로 표시하지 않습니다.

## 실시간 실행

```bash
cd ~/PINKK
source /opt/ros/jazzy/setup.bash
.venv/bin/python -m src.central_control.overhead_vision.localization.live_localization
```

YOLO/ByteTrack 검출, 현재 section 판별, 목표 배정, 고정 CSV 선택과 ROS 2
발행이 한 프로세스에서 동작합니다. 차량은 `START` 또는 주차면에 정차한
상태에서만 새 경로를 선택합니다. 도로를 주행 중인 `TRANSIT` 상태에서는
경로를 중간부터 잘라 새로 발행하지 않습니다.

창 조작:

- `e`: 현재 ego 차량을 화면의 다음 ByteTrack ID 차량으로 전환
- `SPACE`: C1/C2 충전 완료 처리 후 P1~P4 목표 배정
- `p`: 현재 운행 단계의 고정 경로 다시 선택
- `q` 또는 `ESC`: 종료

heading은 차량 마스크에서 자동 측정하므로 마우스 클릭과 `h`/`x` 키는 없습니다.

## ROS 2 토픽

- `/pinkk/vehicle_N/localization_pose`: `geometry_msgs/PoseStamped`, 차량별로 확정한
  차체 중심 x/y와 LiDAR map 좌표계의 측정 yaw
- `/pinkk/vehicle_N/path`: `nav_msgs/Path`, m 단위, 차량별 map frame
- `/pinkk/planned_trajectory`: `std_msgs/Float64MultiArray`
- `/pinkk/path_valid`: `std_msgs/Bool`, ego 전환 시 이전 경로 즉시 무효화

`/pinkk/planned_trajectory`의 point 필드는 다음 네 개뿐입니다.

```text
x_m, y_m, yaw_rad, direction
```

`direction`은 전진 `1`, 후진 `-1`입니다. speed, steering angle, angular
velocity, stop flag는 보내지 않으며 차량 주행 코드가 경로를 바탕으로 직접
계산합니다. 경로와 trajectory 토픽은 reliable/transient-local QoS로 마지막
경로를 유지합니다. 새 경로는 즉시 발행하고 같은 경로를 기본 1초 간격으로
재발행하며, pose 토픽은 최신 차량 위치를 연속 발행합니다. 재발행 주기는
`config/yolo/realtime_localization.yaml`의
`route_republish_period_sec`에서 변경합니다.

## 동작 흐름

1. USB 카메라의 최신 프레임을 `1600×800` Camera BEV로 변환합니다.
2. YOLO segmentation과 ByteTrack으로 ego 차량을 추적합니다.
3. Camera–LiDAR affine 정합으로 차량 rear-axle 위치를 `lidar_map_cm`으로
   변환합니다.
4. 차량 마스크 장축을 LiDAR map yaw로 변환하고 endpoint 고정 yaw로 앞·뒤를
   판별합니다.
5. 입차·충전·출차 상태와 빈 주차면을 기준으로 목표 section을 배정합니다.
6. 현재 section과 목표 section에 대응하는 CSV 전체를 읽어 경로 토픽으로
   즉시 발행합니다.

충전칸이 모두 사용 중이면 P5~P8의 빈 면을 대기 위치로 사용합니다. 충전
완료 후에는 빈 P1~P4 중 출구에 가까운 면을 배정합니다.

기본 실행은 runtime CSV/JSON/PNG를 저장하지 않습니다.
`config/yolo/realtime_localization.yaml`의 `write_runtime_files: true`는
scene/BEV 파일이 필요한 진단 상황에서만 사용합니다.

상세 경로 설정과 테스트 방법은
[`path_planning/README.md`](path_planning/README.md)를 참고합니다.

## Camera BEV 다시 저장

```bash
cd ~/PINKK/src/central_control/camera_tools/first_map
~/PINKK/.venv/bin/python capture_camera_bev.py
```

`s`를 누르면 `camera_bev.png`를 저장하고 `q` 또는 `ESC`로 종료합니다.

Hybrid A*, T-parking, velocity/steering profile 코드는 오프라인 실험과 회귀
진단용으로 남아 있으며 현재 실시간 고정 경로 발행에는 사용하지 않습니다.
