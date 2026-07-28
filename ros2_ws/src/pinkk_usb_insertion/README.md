# Pinkk USB 삽입 제어

`pinkk_usb_insertion`은 로봇팔이 USB-A 포트를 인식하고, 포트 전방으로 접근한 뒤,
영상 기반 미세 정렬과 저속 삽입을 수행하기 위한 ROS 2 패키지입니다.

이 패키지는 기존 캘리브레이션 코드를 확장한 것이 아닙니다. 카메라 내부
파라미터와 Hand-eye 결과를 **입력 데이터**로만 사용하고 실제 운용 제어를 새로
구성합니다.

## 현재 상태

현재 버전은 USB 카메라와 CUDA YOLO Pose 추론, solvePnP, PBVS 목표 계산까지 실제
데이터로 확인했습니다. 로봇 명령 큐 초기화와 실행 게이트도 실기에서 확인했지만,
MyCobot의 직접 `send_coords(mode=1)`는 X 1 mm 시험에서 Z가 4.5 mm 이탈하여
고정-Z 제어 경로로 사용을 중단했습니다. 다음 실행 백엔드는 MoveIt IK와 검증된
관절 waypoint 방식으로 교체할 예정입니다.

| 영역 | 현재 상태 |
|---|---|
| 카메라 내부 파라미터 | 실제 보정 결과 반영 |
| Hand-eye 변환 | Easy Handeye2 결과 반영 |
| YOLO keypoint 입력 | `/dev/video2`, `usb_01.pt`, CUDA 추론과 ROS 토픽 연결 확인 |
| 수동 네 점 입력 | 동일 detection 메시지를 만드는 테스트 노드 지원 |
| solvePnP | 18×8 mm 포트 모델로 유효 관측 발행 확인 |
| 좌표변환 | Hand-eye와 로봇 TF를 포함한 PBVS DRY RUN 확인 |
| 고정-Z PBVS XY 정렬 | 목표 계산 확인, 실제 실행 백엔드는 교체 필요 |
| IBVS | 초기 XY P 제어 계산 함수 구현 |
| 상태 머신 | DRY RUN 경로 구현 |
| MyCobot 명령 큐 안전 | fresh mode, stop, 정지 상태 조회와 실행 게이트 실기 확인 |
| 직접 Cartesian 실제 실행 | `send_coords(mode=1)`에서 Z 이탈 확인, 사용 중단 |
| MoveIt IK 실행 | 다음 구현 단계, 아직 실제 PBVS에 연결되지 않음 |
| Plug TCP | 미보정 |
| YOLO 추론 모델 | 약 500장 학습 모델 연결, 추가 데이터 평가 필요 |
| 실제 삽입 및 접촉 감지 | 미구현 |

현재 기본 실행 모드는 `DRY RUN`입니다. 관절과 Cartesian 실행 게이트는 기본적으로
모두 닫혀 있습니다. 직접 Cartesian action은 코드와 안전 감시 검증용으로 남겨
두지만, MoveIt IK 백엔드가 구현되고 1 mm 실기 검증을 통과하기 전에는 다시
활성화하지 않습니다.

## 전체 처리 흐름

```text
카메라 영상
→ USB 네 모서리 특징점
→ solvePnP
→ T_camera_port
→ Hand-eye와 현재 flange TF 결합
→ T_base_port
→ PBVS 사전 접근 자세
→ 근거리 재검출
→ 필요 시 IBVS 미세 정렬
→ 저속 삽입
→ 성공 확인 또는 후퇴
```

현재 구현 범위는 수동 또는 YOLO 검출 메시지 수신, `solvePnP`, 좌표 계산,
고정-Z PBVS 목표 발행과 DRY RUN 상태 머신까지입니다. YOLO 추론은
`yolo_keypoint_node`, 실제 좌표 계산은 `port_pose_node`, 로봇 기준 변환과 PBVS
목표 계산은 `pbvs_alignment_node`가 각각 담당합니다. 실제 이동 백엔드는
MoveIt IK 방식으로 교체 중입니다.

## 제어 전 인지·좌표 계산 순서

이 구간은 영상을 받아 USB 포트 좌표와 PBVS 목표를 계산하는 단계입니다. 로봇
이동 명령을 보내지 않으며, 현재 실기에서 확인된 인지 파이프라인입니다.

```text
/dev/video2
   │
   ▼
camera_publisher_node
   ├─ /camera/image_raw
   └─ /camera/camera_info  ← 카메라 내부행렬 K, 왜곡계수 D
          │
          ▼
yolo_keypoint_node        ← usb_01.pt, CUDA
   │
   └─ /robot_arm/perception/usb_port/detections
          │                 bbox + USB 네 모서리 픽셀 + confidence
          ▼
port_pose_node
   ├─ 검출 시간·class·confidence·해상도·frame 검사
   ├─ 18×8 mm 포트 모델과 네 픽셀점으로 solvePnP
   └─ T_camera_port 계산
          │
          ├─ /robot_arm/perception/usb_port/observation
          └─ /robot_arm/perception/usb_port/pose_camera
                     │
                     ▼
pbvs_alignment_node
   ├─ T_base_flange       ← /joint_states 기반 로봇 TF
   ├─ T_flange_camera     ← Hand-eye YAML
   ├─ T_camera_port       ← solvePnP
   └─ T_base_port와 고정-Z PBVS 목표 계산
          │
          ├─ /robot_arm/pbvs/port_pose_base
          ├─ /robot_arm/pbvs/target_flange_pose
          ├─ /robot_arm/pbvs/converged
          └─ /robot_arm/pbvs/status
```

### 1. 카메라 영상과 내부 파라미터

[`camera_publisher_node.py`](pinkk_usb_insertion/camera_publisher_node.py)는 USB
카메라 영상을 원본 해상도로 발행합니다.

| 출력 토픽 | 형식 | 내용 |
|---|---|---|
| `/camera/image_raw` | `sensor_msgs/Image` | YOLO가 사용할 원본 BGR 영상 |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | 내부행렬 `K`, 왜곡계수 `D`, 해상도 |

내부 파라미터는 [`camera_intrinsics.yaml`](config/camera_intrinsics.yaml)에서
읽습니다. YOLO keypoint와 `CameraInfo`는 반드시 같은 원본 영상 좌표계를 사용해야
합니다.

### 2. YOLO Pose keypoint 검출

[`yolo_keypoint_node.py`](pinkk_usb_insertion/yolo_keypoint_node.py)는
`/camera/image_raw`를 구독하고 `usb_01.pt`를 CUDA에서 실행합니다. 출력은 USB
포트의 bbox와 네 모서리 픽셀입니다.

```text
0 ───────── 1
│           │
3 ───────── 2
```

중심점은 별도 keypoint로 학습하지 않고 필요할 때 네 점의 평균으로 계산합니다.
YOLO 출력은 다음 custom 메시지 토픽으로 전달됩니다.

```text
/robot_arm/perception/usb_port/detections
pinkk_usb_insertion_interfaces/msg/UsbPortDetectionArray
```

각 detection에는 다음이 포함됩니다.

- 원본 영상 timestamp와 `camera_optical_frame`
- bbox 중심과 크기
- 객체 confidence
- index 0~3의 픽셀 `x`, `y`, confidence와 visibility
- 원본 영상 폭과 높이

RQT에서 보는 `/robot_arm/perception/usb_port/debug_image`는 확인용 영상이며,
좌표 계산 입력은 `debug_image`가 아니라 `detections` 토픽입니다.

### 3. 검출 선택과 안전 검사

[`port_pose_node.py`](pinkk_usb_insertion/port_pose_node.py)는 YOLO detections와
CameraInfo를 함께 구독합니다. 다음 조건을 통과한 검출만 solvePnP에 사용합니다.

- 최근 timestamp이며 최대 age를 넘지 않음
- class가 `usb_port`
- 객체와 네 keypoint confidence가 기준 이상
- keypoint index 0~3이 모두 유효
- 검출 영상과 CameraInfo의 해상도·frame이 일치

기준값은 [`insertion_control.yaml`](config/insertion_control.yaml)의
`pose_estimation`과 `safety`에서 관리합니다. 검출이 없거나 조건을 통과하지 못하면
`valid=false` 관측을 발행하고 제어 목표를 만들지 않습니다.

### 4. solvePnP로 카메라 기준 포트 pose 계산

실제 `cv2.solvePnP()` 호출은
[`perception/pose_estimator.py`](pinkk_usb_insertion/perception/pose_estimator.py)에
있습니다. 입력은 다음 네 가지입니다.

```text
YOLO 네 모서리 픽셀 좌표
+ 카메라 내부행렬 K
+ 카메라 왜곡계수 D
+ 포트 평면 모델 18 mm × 8 mm
```

3D 포트 모델점은 포트 중심을 원점으로 둡니다.

```text
(-9mm, -4mm, 0)   (+9mm, -4mm, 0)
(-9mm, +4mm, 0)   (+9mm, +4mm, 0)
```

solvePnP 결과인 `rvec`, `tvec`을 4×4 변환행렬로 바꾸면 다음 값이 됩니다.

```text
T_camera_port
= camera_optical_frame 기준 USB 포트의 위치와 방향
```

결과는 다음 두 토픽으로 발행합니다.

| 출력 토픽 | 내용 |
|---|---|
| `/robot_arm/perception/usb_port/observation` | pose, keypoint, confidence, 깊이, 재투영 오차, valid 여부 |
| `/robot_arm/perception/usb_port/pose_camera` | 카메라 기준 `PoseStamped` |

`observation.pose.position`의 단위는 meter이며 frame은
`camera_optical_frame`입니다. 이 값은 아직 로봇 base 좌표가 아닙니다.

### 5. 로봇 base 기준 포트 좌표

[`pbvs_alignment_node.py`](pinkk_usb_insertion/pbvs_alignment_node.py)는 solvePnP
결과에 Hand-eye와 현재 로봇 TF를 결합합니다.

```text
T_base_port
= T_base_flange
× T_flange_camera
× T_camera_port
```

| 행렬 | 출처 |
|---|---|
| `T_base_flange` | `/joint_states`와 `g_base → joint6_flange` TF |
| `T_flange_camera` | [`handeye.yaml`](config/handeye.yaml) |
| `T_camera_port` | solvePnP `UsbPortObservation` |

계산된 로봇 기준 USB 포트 pose는 다음 토픽에서 확인합니다.

```text
/robot_arm/pbvs/port_pose_base
```

### 6. PBVS 목표 계산과 제어 경계

PBVS 노드는 카메라 기준 X/Y 오차를 로봇 base 방향으로 변환하고 한 번의 제한된
보정 목표를 만듭니다.

```text
현재 flange pose
+ 제한된 base X/Y 보정량
→ target_flange_pose
```

이 단계의 출력은 계산 결과일 뿐 실제 로봇 명령이 아닙니다.

```text
/robot_arm/pbvs/target_flange_pose
/robot_arm/pbvs/converged
/robot_arm/pbvs/status
```

현재 제어 경계는 다음과 같습니다.

```text
[검증 완료]
카메라 → YOLO → solvePnP → T_base_port → PBVS 목표

[다음 구현]
PBVS 목표 → MoveIt IK → 안전 검사 → 관절 waypoint → 실제 로봇
```

MyCobot 직접 `send_coords()` 경로는 Z 이탈 실기 결과 때문에 이 경계 뒤에서
사용하지 않습니다.

### 인지 단계 확인 명령

```bash
# YOLO 픽셀 검출
ros2 topic echo \
  /robot_arm/perception/usb_port/detections \
  --once

# solvePnP 카메라 기준 좌표
ros2 topic echo \
  /robot_arm/perception/usb_port/observation \
  --once

# 카메라 기준 PoseStamped
ros2 topic echo \
  /robot_arm/perception/usb_port/pose_camera \
  --once

# Hand-eye와 로봇 TF까지 결합한 base 기준 포트 좌표
ros2 topic echo \
  /robot_arm/pbvs/port_pose_base \
  --once

# 실제 이동 전 PBVS 목표와 상태
ros2 topic echo /robot_arm/pbvs/target_flange_pose --once
ros2 topic echo /robot_arm/pbvs/status --once
```

문제 위치를 구분할 때는 다음 순서로 확인합니다.

```text
detections 없음      → 카메라 또는 YOLO 문제
observation invalid  → confidence, timestamp, CameraInfo 또는 solvePnP 문제
pose_camera 정상     → 카메라 기준 인지는 정상
port_pose_base 없음  → Hand-eye 또는 로봇 TF 문제
PBVS target 없음     → 기준 pose capture 또는 PBVS 안전 조건 문제
```

## 패키지 구성

```text
pinkk_usb_insertion/
├── config/                 행렬, 제어값, 실행 안전 설정
├── docs/                   설계 문서
├── launch/                 통합 launch
├── pinkk_usb_insertion/
│   ├── perception/         YOLO 검출 선택·검증과 solvePnP
│   ├── geometry/           좌표변환
│   ├── control/            PBVS, IBVS, 안전 검사
│   ├── state_machine/      전체 절차 상태 관리
│   ├── port_pose_node.py
│   ├── pbvs_alignment_node.py
│   ├── arm_motion_node.py
│   └── usb_insertion_node.py
└── test/                   로봇 없이 실행하는 단위 테스트
```

## 문서 안내

설계를 변경하기 전 다음 문서를 순서대로 확인합니다.

1. [전체 구조](docs/01_ARCHITECTURE_KO.md)
2. [좌표계와 행렬 방향](docs/02_COORDINATE_FRAMES_KO.md)
3. [노드와 토픽](docs/03_NODES_AND_TOPICS_KO.md)
4. [실행 안전 조건](docs/04_SAFETY_KO.md)
5. [진행 현황과 전체 개발 체크리스트](docs/05_DEVELOPMENT_ROADMAP_KO.md)
6. [설정 파일 관리](config/README_KO.md)

## 빌드

```bash
cd ~/Desktop/Pinkk-robot-arm
source /opt/ros/jazzy/setup.bash
colcon build \
  --base-paths ros2_ws/src \
  --packages-select pinkk_usb_insertion \
  --symlink-install
source install/setup.bash
```

기존 `~/mycobot_moveit_ws` overlay에 빌드할 경우 해당 작업공간에서 이 패키지
경로를 인식하도록 구성해야 합니다. 어느 설치본이 실행되는지는 반드시 확인합니다.

```bash
ros2 pkg prefix pinkk_usb_insertion
```

## YOLO 노드 연결

운영 실행에서는 YOLO 노드가 다음 토픽을 발행해야 합니다.

```text
/camera/image_raw                             sensor_msgs/Image
/camera/camera_info                          sensor_msgs/CameraInfo
/robot_arm/perception/usb_port/detections    UsbPortDetectionArray
```

YOLO 출력 좌표는 letterbox나 resize된 추론 이미지가 아니라 `CameraInfo`와 같은
원본 영상 좌표로 복원해서 발행해야 합니다. 메시지 정의는
[`pinkk_usb_insertion_interfaces`](../pinkk_usb_insertion_interfaces/README.md)를
참고합니다.

## DRY RUN 실행

```bash
ros2 launch pinkk_usb_insertion usb_insertion.launch.py
```

YOLO 모델이 아직 없을 때만 수동 입력 테스트 노드를 함께 실행합니다.

```bash
ros2 launch pinkk_usb_insertion usb_insertion.launch.py use_manual_input:=true
```

수동 특징점은 다음 순서로 8개 값을 발행합니다.

```text
[u1, v1, u2, v2, u3, v3, u4, v4]
```

```bash
ros2 topic pub --once /robot_arm/perception/usb_port/manual_keypoints \
  std_msgs/msg/Float64MultiArray \
  "{data: [300.0, 220.0, 360.0, 220.0, 360.0, 245.0, 300.0, 245.0]}"
```

자세 추정 결과를 확인합니다.

```bash
ros2 topic echo /robot_arm/perception/usb_port/observation
ros2 topic echo /robot_arm/perception/usb_port/pose_camera
```

## 현재 조건에서 PBVS 시험

현재 PBVS는 TCP가 정해지기 전의 **카메라 중심 정렬 시험**입니다. `solvePnP`로 얻은
`T_camera_port`와 Easy Handeye2의 `T_flange_camera`, 로봇 TF의
`T_base_flange`를 다음처럼 결합합니다.

```text
T_base_port = T_base_flange × T_flange_camera × T_camera_port
```

그 뒤 포트의 camera X/Y 오차를 base 좌표계로 회전하고 base X/Y 이동량만
계산합니다. 캡처한 초기 flange의 base Z와 회전은 목표에 다시 사용합니다. 한 번에
계산되는 XY 이동은 최대 10 mm이며, XY 오차가 3 mm 이하면
`converged=true`입니다. 이 값은 TCP 없는 카메라 중심 정렬 시험 기준입니다.

```text
UsbPortObservation
  ├─ pose(T_camera_port) ─┐
  ├─ timestamp ───────────┼→ pbvs_alignment_node
로봇 TF(T_base_flange) ───┤     ├→ T_base_port
Hand-eye(T_flange_camera) ┘     ├→ target_flange_pose (Z·자세 유지)
                                └→ converged/status
```

통합 launch에는 PBVS 노드가 포함되어 있습니다. 실제 로봇 TF가 실행 중이면 바로
수동 keypoint를 발행하고 다음 토픽을 확인합니다.

```bash
ros2 topic echo /robot_arm/pbvs/port_pose_base
ros2 topic echo /robot_arm/pbvs/target_flange_pose
ros2 topic echo /robot_arm/pbvs/converged
ros2 topic echo /robot_arm/pbvs/status
```

로봇 없이 계산 경로만 시험할 때는 별도 터미널에서 임시 정적 TF를 발행할 수
있습니다. 실제 로봇 TF가 발행 중일 때는 같은 frame의 정적 TF를 중복 발행하면
안 됩니다.

```bash
ros2 run tf2_ros static_transform_publisher \
  --x 0.30 --y 0.00 --z 0.40 \
  --qx 0.0 --qy 0.0 --qz 0.0 --qw 1.0 \
  --frame-id g_base --child-frame-id joint6_flange
```

`/robot_arm/pbvs/target_flange_pose`는 검증용 출력입니다. 현재 노드는 이 값을
`/robot_arm/target_pose`로 전달하지 않으며 MoveIt이나 로봇 bridge를 호출하지
않습니다. 따라서 토픽을 확인하는 것만으로 로봇이 움직이지 않습니다.

### USB 영상 클릭으로 시험

숫자를 직접 토픽에 입력하는 대신 기존 `manual_usb_tf` OpenCV 창에서 USB 네
모서리를 클릭할 수 있습니다. 유효한 네 번째 클릭이 들어오면 클릭 프로그램이
`/robot_arm/perception/usb_port/manual_keypoints`를 자동으로 한 번 발행합니다.

먼저 이 패키지를 `~/mycobot_moveit_ws/install_pinkk`에 빌드한 뒤 PBVS 노드를
실행합니다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/mycobot_moveit_ws/install/setup.bash
source ~/mycobot_moveit_ws/install_pinkk/setup.bash
export ROS_DOMAIN_ID=36
export ROS_LOCALHOST_ONLY=0
ros2 launch pinkk_usb_insertion usb_insertion.launch.py use_manual_input:=true
```

다른 노트북 터미널에서 클릭 창을 실행합니다.

```bash
cd ~/Desktop/Pinkk-robot-arm
bash scripts/calibration/laptop_manual_usb_tf.sh \
  http://192.168.6.1:5000/stream
```

OpenCV 창에서 `f`를 눌러 영상을 고정하고 다음 순서로 클릭합니다.

```text
1: USB 긴 변 시작점
2: 같은 긴 변 끝점
3: 인접한 짧은 변 끝점
4: 마지막 모서리
```

세 번째 터미널에서 결과를 확인합니다.

```bash
ros2 topic echo /robot_arm/pbvs/status
ros2 topic echo /robot_arm/pbvs/target_flange_pose
ros2 topic echo /robot_arm/pbvs/converged
```

로봇팔을 실제로 움직였다면 클릭 창에서 `r`로 이전 점을 지우고, 로봇이 완전히
멈춘 새 영상을 `f`로 고정하여 다시 클릭해야 합니다. 오래된 클릭점을 다음 자세에
재사용하면 안 됩니다.

수동 입력 launch는 컴퓨터 사이의 작은 clock 차이로 TF extrapolation이 발생하지
않도록 정지 상태의 최신 `g_base → joint6_flange` TF를 사용합니다. 상태에는
`tf_mode=latest_stationary`가 표시됩니다. 이 모드는 로봇이 완전히 정지한
stop-and-go 시험에서만 사용합니다. YOLO 운용에서는 영상 timestamp에 해당하는
TF를 사용하므로 두 컴퓨터의 clock 동기화가 필요합니다.

### 직접 `send_coords()` 시험 결과와 현재 금지 상태

2026-07-27 실기에서 X +1 mm 목표를 `send_coords(mode=1)`로 보냈을 때
MyCobot의 실제 Z가 4.5 mm 이탈했습니다. 정지 상태 `get_coords()`를 30회 읽은
결과 모든 축의 span이 0이어서 측정 노이즈가 아니라 실제 제어 경로 문제로
판정했습니다. 브리지의 Z 이탈 감시는 정상적으로 정지 명령을 수행했습니다.

따라서 다음 항목을 적용합니다.

- `cartesian_execution_enabled=false` 유지
- `enable_pbvs_test_execution=false` 유지
- Z/Roll/Pitch 허용오차를 늘려 같은 시험을 우회하지 않음
- `cartesian_smoke_test`는 DRY RUN 좌표 확인에만 사용
- 실제 PBVS 이동은 MoveIt IK 백엔드가 준비된 후 재개

다음 백엔드는 현재 TF에서 X/Y 목표를 만들고, Z와 Roll/Pitch를 시작값으로 고정한
뒤 MoveIt IK·충돌·관절 점프 검사를 통과한 짧은 관절 waypoint만 실행합니다.
실행 중에는 실제 TF를 감시하고 Z 또는 Roll/Pitch가 제한을 넘으면 action을
취소합니다. X/Y 각각 1 mm 왕복 시험을 통과하기 전에는 5 mm, 10 mm 또는 PBVS
폐루프 시험으로 확대하지 않습니다.

Yaw PBVS용 평면 긴 축 오차와 1회 2도 제한 계산은 준비되어 있지만 현재
`yaw_pbvs.enabled=false`입니다. YOLO keypoint 순서와 실제 `T_flange_plug`를
확정한 뒤 포트 긴 축과 플러그 긴 축을 연결하기 전까지 Yaw 실제 명령은
발행하지 않습니다. 최종 제어 자유도는 `X/Y/Yaw 허용, Z/Roll/Pitch 고정`입니다.

### 나중에 교체할 입력

- YOLO가 준비되면 `manual_detection_node`만 끄고 같은
  `/robot_arm/perception/usb_port/detections`를 YOLO 노드가 발행합니다.
- TCP가 측정되면 카메라 중심 정렬 결과에서 바로 flange를 움직이는 방식 대신
  `T_flange_plug`를 포함한 plug tip 목표 역산을 추가합니다.
- 현재 고정-Z PBVS가 검증된 뒤 IBVS-PD 미세 XY 정렬과 수직 저속 삽입을 순서대로
  연결합니다.

유효한 검출 결과가 들어온 뒤 DRY RUN 상태 머신을 시작합니다.

```bash
ros2 topic pub --once /robot_arm/usb_insertion/command \
  std_msgs/msg/String "{data: start}"
ros2 topic echo /robot_arm/usb_insertion/state
```

## 중요한 제한

- YOLO 라벨 index와 USB 3D 모델점 순서가 반드시 같아야 합니다.
- YOLO keypoint는 원본 영상 좌표계로 복원해서 발행해야 합니다.
- `tool.calibrated=false`이므로 현재 값은 실제 plug tip을 뜻하지 않습니다.
- 현재 PBVS 목표는 plug tip 정렬이 아니라 카메라 광축 중심 정렬입니다.
- 카메라가 크게 기울어진 자세에서는 base Z 고정 때문에 영상 오차 전체를 한 번에
  제거할 수 없으므로 반복 관측으로 수렴 여부를 확인해야 합니다.
- 실제 실행은 launch의 `enable_pbvs_test_execution=true`를 명시해야만 허용됩니다.
- 직접 Cartesian 실행은 실기에서 Z 이탈이 확인되어 현재 사용 금지입니다.
- 다음 실기 검증은 MoveIt IK 백엔드의 X/Y 1 mm 왕복 시험부터 다시 시작합니다.
- 기존 `usb_pre_approach`는 좌표 검증 실험이며 이 패키지의 실행기가 아닙니다.
