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

PBVS 노드는 유효한 YOLO keypoint와 SolvePnP 결과를 base 좌표계로 변환한 뒤
같은 자세를 `g_base → usb_port` 동적 TF로도 발행한다. 이 TF는 RViz
시각화용이며 로봇 이동 명령이 아니다. 검출이 없거나 관측이 거부된 동안에는
새 TF를 발행하지 않는다. 포트 TF에는 OBSERVE_POSE 확인이 필요하지 않지만,
PBVS 목표는 현재 관절이 설정된 OBSERVE_POSE에서 최대 3도 안인지 확인된 뒤에만
발행된다. 별도의 수동 `capture` 명령은 사용하지 않는다.

노트북에서 perception과 로봇의 `g_base → joint6_flange` TF가 들어오는 상태에서
다음 노드를 실행한다.

```bash
ros2 run pinkk_usb_insertion pbvs_alignment_node \
  --ros-args -p use_latest_flange_tf:=true
```

별도 터미널에서 수치와 TF 연결을 확인한다.

```bash
ros2 run tf2_ros tf2_echo g_base usb_port
```

RViz2의 `Global Options → Fixed Frame`을 `g_base`로 지정하고 `TF` display를
추가한다. 포트 축만 크게 보고 싶으면 `Axes` display를 추가해
`Reference Frame=usb_port`로 지정한다. 축의 원점은 YOLO 네 keypoint로 구한
18×8mm 포트 모델의 중심이며, 방향은 keypoint 순서와 SolvePnP 결과를 따른다.

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
PBVS target 없음     → OBSERVE_POSE 관절 검사 또는 PBVS 안전 조건 문제
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

### MoveIt IK 고정-Z DRY RUN

`moveit_ik_step_executor`는 위 실행 백엔드의 첫 단계인 계산 전용 검사기입니다.
현재 `g_base → joint6_flange` 자세를 기준으로 base X 또는 Y만 바꾸며, Z와
quaternion 전체를 그대로 복사합니다. 각 waypoint마다 다음 순서를 수행합니다.

```text
/joint_states와 최신 flange TF 읽기
  → 1 mm 간격의 고정-Z XY waypoint 생성
  → /compute_ik (직전 해를 다음 seed로 사용)
  → 인접 IK 관절 점프 검사
  → /check_state_validity 충돌·관절 제한 검사
  → /compute_fk로 목표 Z·자세 역검증
  → 결과 출력 후 종료
```

이 노드는 trajectory action client, Cartesian action client 및 로봇 명령
publisher를 만들지 않습니다. 따라서 `DRY RUN 통과`는 계산 경로가 안전 조건을
통과했다는 뜻일 뿐, 로봇이 이동했다는 뜻이 아닙니다.

노트북에서 변경분을 먼저 빌드합니다.

```bash
cd ~/Desktop/Pinkk-robot-arm
source /opt/ros/jazzy/setup.bash
source ~/mycobot_moveit_ws/install/setup.bash

colcon build \
  --base-paths ros2_ws/src \
  --packages-select pinkk_usb_insertion \
  --build-base ~/mycobot_moveit_ws/build_pinkk \
  --install-base ~/mycobot_moveit_ws/install_pinkk \
  --symlink-install
```

터미널 1의 로봇 PC에서는 `/joint_states`만 필요하므로 관절·Cartesian 실행
게이트를 모두 닫은 bridge를 유지합니다.

```bash
cd ~/Pinkk-robot-arm
bash scripts/calibration/robot_start_bridge.sh \
  5 3.5 false 0.0015 false
```

터미널 2의 노트북에서는 MoveIt 계산 service를 실행합니다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/mycobot_moveit_ws/install/setup.bash
source ~/mycobot_moveit_ws/install_pinkk/setup.bash

export ROS_DOMAIN_ID=36
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
unset ROS_LOCALHOST_ONLY

ros2 launch pinkk_mycobot_bridge planning_only.launch.py
```

터미널 3의 노트북에서 먼저 X +1 mm를 계산합니다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/mycobot_moveit_ws/install/setup.bash
source ~/mycobot_moveit_ws/install_pinkk/setup.bash

export ROS_DOMAIN_ID=36
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
unset ROS_LOCALHOST_ONLY

ros2 run pinkk_usb_insertion moveit_ik_step_executor \
  --axis x --distance-mm 1
```

X -1 mm와 Y ±1 mm도 같은 방식으로 계산할 수 있습니다.

```bash
ros2 run pinkk_usb_insertion moveit_ik_step_executor \
  --axis x --distance-mm -1
ros2 run pinkk_usb_insertion moveit_ik_step_executor \
  --axis y --distance-mm 1
ros2 run pinkk_usb_insertion moveit_ik_step_executor \
  --axis y --distance-mm -1
```

기본 제한은 총 이동 10 mm 이하, waypoint 간격 1 mm 이하, 인접 waypoint의
최대 관절 변화 5도 이하입니다. 실제 이동 기능은 아직 없으므로 이 단계에서는
bridge의 `joint_enabled=false`와 `cartesian_enabled=false`를 바꾸지 않습니다.
출력의 `현재 관절`, `IK 관절`, `관절 차이`, `FK Z 오차`, `자세 오차`를 네
방향 모두 기록한 뒤에만 검증된 관절 trajectory 실행 단계를 추가합니다.

관절 완료 허용오차는 용도별로 구분합니다. 관측 자세 복귀의 1.5~3.5도는
카메라 시야를 되찾는 근사 기준이고, 1mm PBVS 이동 성공 기준이 아닙니다.
1mm MoveIt IK 단발 시험에서는 관절 목표를 통과해도 이동 후 TF의 Z 1mm와
자세 1도 제한을 별도로 통과해야 합니다. 이후 거친 PBVS에는 정지 후 Z 2mm와
자세 2도 제한을 적용합니다. 하드웨어 관절 오차를 숨기기 위해 이 완료 기준을
더 늘리지 않습니다.

### MoveIt IK 1mm 실제 단발 시험

네 방향 DRY RUN이 모두 통과한 뒤에만 `moveit_ik_step_execute`를 사용합니다.
이 명령은 직접 Cartesian `send_coords()`를 호출하지 않습니다. 현재 자세에서
MoveIt IK·충돌·FK 검사를 다시 수행하고, 최종 관절값 하나를
`FollowJointTrajectory`로 bridge에 전달합니다.

로봇 PC에서 실행 중인 차단 bridge를 `Ctrl+C`로 종료한 뒤 관절 실행만
허용하여 다시 시작합니다. 1mm X 이동의 예상 관절 변화가 약 0.75도이므로
3.5도 허용오차는 사용할 수 없습니다. 첫 시험은 0.3도로 제한합니다.

```bash
cd ~/Pinkk-robot-arm

bash scripts/calibration/robot_start_bridge.sh \
  5 0.3 false 0.0015 true
```

인자의 의미는 순서대로 다음과 같습니다.

```text
5       : MyCobot 관절 이동 속도
0.3     : 목표 관절 도달 허용오차(deg)
false   : 직접 Cartesian send_coords 실행 차단
0.0015  : Cartesian 제한값(차단 상태라 사용하지 않음)
true    : FollowJointTrajectory 관절 실행 허용
```

노트북의 `planning_only.launch.py`는 그대로 유지합니다. 다른 노트북 터미널에서
로봇 주변을 비우고 비상 정지 준비 후 X +1mm 한 번만 실행합니다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/mycobot_moveit_ws/install/setup.bash
source ~/mycobot_moveit_ws/install_pinkk/setup.bash

export ROS_DOMAIN_ID=36
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
unset ROS_LOCALHOST_ONLY

ros2 run pinkk_usb_insertion moveit_ik_step_execute \
  --axis x --distance-mm 1 --move-seconds 5 --execute
```

실행기는 다음 조건을 모두 강제합니다.

- 절대 이동 명령 1mm 이하
- 전체 관절 변화 1도 이하
- MoveIt IK·충돌·FK 사전검사 통과
- 실제 명령 전 3초 경고
- 이동 후 실제 TF가 요청 방향으로 최소 0.25mm 이동
- 목표 대비 위치 오차 0.75mm 이하
- 목표 대비 Z 오차 1mm 이하
- 목표 대비 자세 오차 1도 이하

성공 로그의 `측정 이동 dx/dy/dz`를 기록합니다. 실패하면 반대 명령이나 다음 축을
연속 실행하지 말고 현재 관절과 TF부터 확인합니다. X +1mm 결과가 정상일 때만
현재 위치 기준 X -1mm를 별도로 승인하여 원래 X 위치로 복귀합니다.

```bash
ros2 run pinkk_usb_insertion moveit_ik_step_execute \
  --axis x --distance-mm -1 --move-seconds 5 --execute
```

그다음 Y +1mm와 Y -1mm를 같은 방식으로 한 번씩 실행합니다. 실행기는 실패 시
임의의 자동 복귀를 하지 않습니다. 잘못된 자동 복귀가 두 번째 위험 이동이 될 수
있기 때문에 결과 확인 후 사용자가 반대 방향을 명시해야 합니다.

### YOLO PBVS 실제 3mm 단발 실행

`moveit_pbvs_step_execute`는 YOLO keypoint와 solvePnP를 통해 발행된 최신
`/robot_arm/pbvs/target_flange_pose`를 실제 관절 명령으로 한 번만 실행합니다.
첫 실기 상한은 3mm이며, 한 번 실행 후 반드시 새 YOLO 관측을 기다리는
stop-and-go 방식입니다.

```text
YOLO keypoint
→ solvePnP 포트 자세
→ PBVS 고정-Z XY 목표
→ 목표 age·frame·converged 검사
→ MoveIt IK·충돌·FK 검사
→ FollowJointTrajectory 한 번
→ 실제 TF의 이동 방향·Z·자세 검사
→ 종료하고 새 관측 대기
```

로봇 PC bridge는 직접 Cartesian을 차단하고 관절 명령을 한 번만 보냅니다.

```bash
bash scripts/calibration/robot_start_bridge.sh \
  5 0.5 false 0.0015 true 3 true 0.8 1.0 2.0
```

마지막 네 값은 관절 오차 보상 허용, 보상 gain, 관절별 1회 보상 제한(deg),
원래 MoveIt 목표로부터 관절별 누적 보상 제한(deg)입니다. 보상은 로봇이
정지했는데도 원래 목표 오차가 0.5도보다 클 때만 실행되며, 오차가 감소하지
않으면 남은 횟수와 관계없이 중단합니다.

노트북에서는 MoveIt, USB 인지, PBVS 목표 계산을 각각 실행합니다.

```bash
ros2 launch pinkk_mycobot_bridge planning_only.launch.py
```

```bash
ros2 launch pinkk_usb_insertion usb_perception.launch.py \
  camera_device:=/dev/video2 \
  model_path:=/home/juwon/Desktop/usb_01.pt \
  inference_device:=cuda:0
```

```bash
ros2 run pinkk_usb_insertion pbvs_alignment_node \
  --ros-args -p use_latest_flange_tf:=true
```

PBVS 상태가 `converged=False`이고 목표가 최대 3mm인지 먼저 확인합니다.

```bash
ros2 topic echo /robot_arm/pbvs/status --once
```

주변을 비우고 명시적으로 한 번만 승인합니다.

```bash
ros2 run pinkk_usb_insertion moveit_pbvs_step_execute \
  --move-seconds 8 --execute
```

실행기는 1초보다 오래된 PBVS 목표, 3mm 초과 이동, Z 0.5mm 초과 목표 변화,
자세 0.5도 초과 목표 변화 및 이미 수렴한 목표를 거부합니다. 성공해도 자동으로
다음 PBVS 명령을 실행하지 않으며 새 YOLO 관측의 오차 감소 여부로 다음 단계를
결정합니다.

거친 PBVS와 최종 삽입 직전의 안전 기준은 분리합니다. 관절 공간 보간 중에는
기기 오차만으로 불필요하게 action을 취소하지 않도록 시작 Z 10mm, 자세 5도,
계획 XY보다 10mm 과주행, 요청 반대 방향 3mm를 비상 취소 기준으로 사용합니다.
로봇이 정지한 뒤에는 Z 오차 2mm, 자세 오차 2도를 적용하며 하나라도 넘으면
해당 이동을 실패로 처리하고 다음 PBVS step을 보내지 않습니다. 최종 삽입
직전에는 Z 1mm, 자세 1도 기준으로 다시 전환해야 합니다.

거친 PBVS 실행기는 PBVS 목표에서 base X/Y만 사용하고 각 단발 실행 직전의
현재 Z와 quaternion을 목표에 복사합니다. 따라서 기기 오차로 초기 기준 자세와
조금 달라져도 다음 XY 보정을 계속할 수 있습니다. 초기 관측 Z와 Roll/Pitch를
되찾는 동작은 XY가 수렴한 뒤 PRE_INSERT 단계에서 별도로 수행합니다.

시리얼 큐 준비나 `send_angles()` 호출 중에는 TF 갱신 자체가 지연될 수 있으므로
이 감시는 물리적 비상정지를 대신하지 않습니다.

Yaw PBVS용 평면 긴 축 오차와 1회 2도 제한 계산은 준비되어 있지만 현재
`yaw_pbvs.enabled=false`입니다. YOLO keypoint 순서와 실제 `T_flange_plug`를
확정한 뒤 포트 긴 축과 플러그 긴 축을 연결하기 전까지 Yaw 실제 명령은
발행하지 않습니다. 최종 제어 자유도는 `X/Y/Yaw 허용, Z/Roll/Pitch 고정`입니다.

### YOLO X/Y/Yaw 실제 stop-and-go 폐루프

`moveit_pbvs_closed_loop_execute`는 한 번의 명시적 실행 승인 뒤 아래 순서를
제한 횟수만 반복한다.

```text
안정된 YOLO 5개 확인
→ SolvePnP와 base-frame X/Y/Yaw 오차 계산
→ X/Y 최대 3mm, Yaw 최대 2도 목표
→ MoveIt IK·충돌·FK 검사
→ FollowJointTrajectory 한 번
→ 로봇 정지와 실제 TF 확인
→ 1초 대기
→ 이동 후 새 YOLO 5개로 오차 감소 확인
```

먼저 X/Y만 실제 폐루프로 확인한다. `pbvs_alignment_node`는
`enable_yaw_pbvs`를 주지 않고 실행한다. 로봇이 OBSERVE_POSE에 있으면
`/joint_states` 검사 후 현재 TF가 자동 저장되므로 별도 capture 명령은 없다.

```bash
# 실제 X/Y 폐루프
ros2 run pinkk_usb_insertion moveit_pbvs_closed_loop_execute \
  --max-steps 12 \
  --move-seconds 8 \
  --max-total-xy-mm 40 \
  --execute
```

Yaw에는 TCP의 **위치 offset**까지는 필요하지 않다. 이 프로젝트에서는 충전기를
고정한 뒤에도 플러그 Yaw를 `joint6_flange`의 그리퍼 Yaw와 동일하게 사용한다.
따라서 `flange_to_plug_yaw_offset_deg: 0.0`, `flange_long_axis: x`로 두고
포트 18mm 장축인 `port_long_axis: x`와 맞춘다. 실제 실행 전에는 YOLO
keypoint 0→1 방향이 포트 18mm 장축과 일치하는지만 확인하고 정렬 노드와
실행기 양쪽에서 Yaw를 동시에 승인한다.

```bash
# PBVS 목표 계산 터미널
ros2 run pinkk_usb_insertion pbvs_alignment_node \
  --ros-args \
  -p use_latest_flange_tf:=true \
  -p enable_yaw_pbvs:=true

# 실제 폐루프 실행 터미널
ros2 run pinkk_usb_insertion moveit_pbvs_closed_loop_execute \
  --enable-yaw \
  --max-steps 12 \
  --move-seconds 8 \
  --max-total-xy-mm 40 \
  --execute
```

폐루프는 다음 경우 다음 명령을 보내지 않고 실패 종료한다.

- 안정된 새 검출을 8초 안에 받지 못함
- X/Y 영상 오차가 직전보다 3mm 이상 증가
- Yaw 영상 오차가 직전보다 1.5도 이상 증가
- 한 번의 MoveIt·충돌·FK·실제 이동 검사 실패
- 최대 12 step, 300초 또는 누적 XY 40mm 도달

정상 종료 조건은 X/Y 3mm 이내이며 Yaw 활성 시 Yaw 1도 이내도 동시에
만족하는 것이다. 정상 종료 후에도 로봇은 그 자리에서 정지할 뿐이고 Z 하강,
PRE_INSERT 이동, 삽입 및 자동 복귀는 실행하지 않는다.

나중에 측정할 TCP translation은 `T_flange_plug_tip`의 X/Y/Z 위치에만 적용한다.
현재 확정한 Yaw offset 0도는 유지하므로 TCP 추가 뒤에도 같은 Yaw PBVS 계산을
재사용한다.

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
