# Pinkk USB 삽입 제어

`pinkk_usb_insertion`은 로봇팔이 USB-A 포트를 인식하고, 포트 전방으로 접근한 뒤,
영상 기반 미세 정렬과 저속 삽입을 수행하기 위한 ROS 2 패키지입니다.

이 패키지는 기존 캘리브레이션 코드를 확장한 것이 아닙니다. 카메라 내부
파라미터와 Hand-eye 결과를 **입력 데이터**로만 사용하고 실제 운용 제어를 새로
구성합니다.

## 현재 상태

현재 버전은 전체 구조의 첫 단계입니다.

| 영역 | 현재 상태 |
|---|---|
| 카메라 내부 파라미터 | 실제 보정 결과 반영 |
| Hand-eye 변환 | Easy Handeye2 결과 반영 |
| YOLO keypoint 입력 | custom detection 토픽 구독 구조 구현 |
| 수동 네 점 입력 | 동일 detection 메시지를 만드는 테스트 노드 지원 |
| solvePnP | 구현 |
| 좌표변환 | 구현 |
| PBVS 접근 자세 계산 | 순수 계산 함수 구현 |
| IBVS | 초기 XY P 제어 계산 함수 구현 |
| 상태 머신 | DRY RUN 경로 구현 |
| MoveIt 실제 실행 | 미연결 |
| Plug TCP | 미보정 |
| YOLO 추론 모델 | 외부 노드 연결 필요 |
| 실제 삽입 및 접촉 감지 | 미구현 |

현재 기본 실행 모드는 `DRY RUN`입니다. 포트 자세 계산과 상태 전이는 수행하지만
MoveIt 또는 로봇 bridge에 이동 목표를 보내지 않습니다.

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
→ IBVS 미세 정렬
→ 저속 삽입
→ 성공 확인 또는 후퇴
```

현재 구현 범위는 YOLO 검출 메시지 수신, `solvePnP`, 좌표 계산 모듈과 DRY RUN
상태 머신까지입니다. YOLO 모델 추론 자체는 별도 노드가 담당합니다.

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
5. [단계별 개발 순서](docs/05_DEVELOPMENT_ROADMAP_KO.md)
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
- `execution_enabled=true`로 바꿔도 현재 이동 백엔드는 명령을 거부합니다.
- 실제 로봇 실행 코드는 좌표 반복성, 작업영역, TCP 검증 후 연결합니다.
- 기존 `usb_pre_approach`는 좌표 검증 실험이며 이 패키지의 실행기가 아닙니다.
