# 전체 구조

## 1. 설계 목표

캘리브레이션 과정과 실제 운용 제어를 분리합니다. 운용 패키지는 캘리브레이션
알고리즘을 다시 실행하지 않고 검증된 결과 행렬만 읽습니다.

```text
캘리브레이션 도구                       실제 운용 패키지
------------------                     ----------------
카메라 내부 보정 ── K, D ─────────────→ port_pose_node
Hand-eye 보정 ── T_flange_camera ─────→ geometry/control
TCP 측정 ── T_flange_plug ────────────→ PBVS/삽입 제어
```

## 2. 계층 분리

### Perception

별도 YOLO 노드가 USB 포트의 네 모서리와 신뢰도를 검출 메시지로 발행합니다.
`port_pose_node`는 메시지를 구독해 후보 선택, 검증과 `T_camera_port` 계산을
담당합니다. 수동 입력은 같은 검출 메시지를 발행하는 테스트 노드로 분리합니다.

### Geometry

행렬의 방향, 합성, 역변환을 담당합니다. ROS 노드나 카메라 입력에 의존하지 않아
로봇 없이 단위 테스트할 수 있습니다.

### Control

- PBVS: 포트 전방 사전 접근 자세 계산
- IBVS: 현재 픽셀과 teach pose 픽셀 사이의 근거리 오차 보정
- Safety: 검출 품질, 시간, 실행 설정 검사

### State machine

인식, 접근, 정렬, 삽입, 검증, 후퇴 순서를 관리합니다. 각 알고리즘이 직접 다음
동작을 호출하지 않고 상태 머신이 전체 순서를 결정합니다.

### ROS nodes

순수 계산 모듈을 ROS 메시지와 연결합니다. 실제 로봇으로 나가는 모든 명령은
`arm_motion_node` 한 곳을 통과해야 합니다.

## 3. 노드 책임

```text
YOLO keypoint detections
          │
          ▼
  port_pose_node ─────→ UsbPortObservation / pose_camera
          │
          ▼
 usb_insertion_node ──→ target_pose
          │
          ▼
   arm_motion_node ───→ MoveIt/robot bridge (향후 연결)
          │
          └───────────→ motion_done / motion_status
```

## 4. 의도적으로 분리한 기존 코드

다음 코드는 실제 운용 패키지에서 import하지 않습니다.

- `pinkk_handeye_automation`: Hand-eye 수집과 좌표 검증
- `pinkk_mycobot_bridge`: 관절 상태와 trajectory 연결
- `manual_usb_tf`: 캘리브레이션 이후 좌표 정확도 실험
- `usb_pre_approach`: flange 기준 시험 이동

`pinkk_mycobot_bridge`는 향후 실행 백엔드로 연결할 수 있지만 운용 상태 머신과
하나의 패키지로 합치지 않습니다.

## 5. 데이터 흐름 원칙

- 길이 단위는 내부에서 모두 미터를 사용합니다.
- 각도는 내부에서 radian 또는 quaternion을 사용합니다.
- 이미지 특징점만 pixel 단위를 사용합니다.
- 행렬 이름은 `T_parent_child` 규칙을 사용합니다.
- 오래된 검출 결과로 다음 이동을 실행하지 않습니다.
- 포트 pose와 로봇 pose의 시간 관계를 기록합니다.
