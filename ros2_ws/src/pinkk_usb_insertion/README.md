# Pinkk USB 삽입 제어

`pinkk_usb_insertion`은 로봇팔이 USB-A 포트를 인식하고, 포트 전방으로 접근한 뒤,
영상 기반 미세 정렬과 저속 삽입을 수행하기 위한 ROS 2 패키지입니다.

이 패키지는 기존 캘리브레이션 코드를 확장한 것이 아닙니다. 카메라 내부
파라미터와 Hand-eye 결과를 **입력 데이터**로만 사용하고 실제 운용 제어를 새로
구성합니다.

## 현재 상태

현재 버전은 인지 인터페이스, solvePnP, PBVS 계산과 승인형 Cartesian 실행 기반까지
구현한 상태입니다. 실제 로봇 브리지와 이후 단계는 아래 상태를 구분해 확인합니다.

| 영역 | 현재 상태 |
|---|---|
| 카메라 내부 파라미터 | 실제 보정 결과 반영 |
| Hand-eye 변환 | Easy Handeye2 결과 반영 |
| YOLO keypoint 입력 | custom detection 토픽 구독 구조 구현 |
| 수동 네 점 입력 | 동일 detection 메시지를 만드는 테스트 노드 지원 |
| solvePnP | 구현 |
| 좌표변환 | 구현 |
| 고정-Z PBVS XY 정렬 | 계산 노드와 승인형 최대 10 mm Cartesian 단발 실행기 구현 |
| IBVS | 초기 XY P 제어 계산 함수 구현 |
| 상태 머신 | DRY RUN 경로 구현 |
| PBVS Cartesian 실제 실행 | `send_coords()` 연결, 노트북 테스트 완료, 실기 미검증 |
| MoveIt 일반 trajectory 실행 | 마지막 관절 목표 방식이며 Cartesian 경로 추종용이 아님 |
| Plug TCP | 미보정 |
| YOLO 추론 모델 | 외부 노드 연결 필요 |
| 실제 삽입 및 접촉 감지 | 미구현 |

현재 기본 실행 모드는 `DRY RUN`입니다. PBVS Cartesian 경로는 소프트웨어로
연결됐지만 launch에서 명시적으로 실행을 허용하지 않으면 로봇 bridge에 목표를
보내지 않습니다. 새 브리지는 실제 로봇에서 아직 검증되지 않았습니다.

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
고정-Z PBVS 목표 발행, 승인형 Cartesian 실행 경로와 DRY RUN 상태 머신까지입니다.
YOLO 모델 추론 자체는 별도 노드가 담당합니다.

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

### 사용자 승인형 최대 10 mm Cartesian 실제 시험

실제 시험 launch는 명시적으로 실행기를 허용해야 합니다. 기본값은 항상
`false`입니다.

```bash
ros2 launch pinkk_usb_insertion usb_insertion.launch.py \
  use_manual_input:=true \
  enable_pbvs_test_execution:=true
```

로봇이 안전한 초기 관측 자세에 완전히 정지한 뒤 Z와 초기 자세 기준을 명시적으로
한 번 캡처합니다. 기준이 없으면 PBVS 계산은 자동 거부됩니다.

```bash
ros2 topic pub --once /robot_arm/pbvs/reference_command \
  std_msgs/msg/String "{data: capture}"
```

캡처한 `z_lock`과 초기 자세는 모든 PBVS 반복에서 재사용하므로 매 이동의 작은
Z·자세 오차가 다음 기준으로 누적되지 않습니다. 새 작업을 시작하거나 관측 자세를
다시 잡았으면 먼저 초기화한 뒤 다시 캡처합니다.

```bash
ros2 topic pub --once /robot_arm/pbvs/reference_command \
  std_msgs/msg/String "{data: reset}"
```

새 클릭으로 `converged=false` 목표가 나온 뒤 30초 안에 다음 명령을 한 번
발행하면 최신 목표 하나만 실행합니다.

```bash
ros2 topic pub --once /robot_arm/pbvs/step_command \
  std_msgs/msg/String "{data: execute_once}"
```

실행 상태를 클릭 전에 모니터링합니다.

```bash
ros2 topic echo /robot_arm/pbvs/execution_status
```

실행기는 현재 TF와 목표를 다시 비교하여 XY 10 mm, Z 변화 0.5 mm, 자세 변화
0.5도 제한을 검사합니다. 목표까지 최대 1 mm 간격의 고정-Z·고정자세 Cartesian
waypoint를 만들고, 각 점에서 MoveIt 충돌검사 IK와 관절 점프 5도 제한을 모두
사전검사합니다. 이 waypoint들은 안전 검사에만 사용하며 한 점씩 정지 이동하지
않습니다.

검사를 모두 통과하면 최종 `g_base` 기준 flange pose 하나를
`/robot_arm/cartesian_move` action으로 보내고, 로봇 PC 브리지가
`send_coords(..., speed=10, mode=1)`로 실행합니다. 브리지는 이동 중 실제
`get_coords()`를 읽어 Z 2 mm 또는 Roll/Pitch 3도 이상 이탈하면 `stop()`하고,
PBVS 실행기도 완료 후 TF의 Z·자세를 다시 검사합니다. 실행 후 목표를 폐기하므로
다음 단계에는 반드시 새 YOLO 관측이 필요합니다. 현재 수동 클릭 입력은 기능 확인
용도일 뿐 운영 입력으로 사용하지 않습니다.

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
- Cartesian 실행은 노트북 단위 테스트만 통과했으며 실제 `pymycobot`과 로봇에서
  아직 검증되지 않았습니다. 첫 검증은 장애물이 없는 자세에서 1 mm로 수행합니다.
- 기존 `usb_pre_approach`는 좌표 검증 실험이며 이 패키지의 실행기가 아닙니다.
