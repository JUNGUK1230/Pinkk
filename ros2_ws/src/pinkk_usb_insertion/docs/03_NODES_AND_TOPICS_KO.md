# 노드와 토픽

## 1. 외부 `yolo_keypoint_node`

카메라 영상을 구독하고 USB-A 포트 후보와 네 keypoint를 발행합니다. 이 노드는
현재 패키지에 포함하지 않으며 YOLO 모델이 준비될 때 별도로 구현합니다.

### 입력

| 토픽 | 형식 | 설명 |
|---|---|---|
| `/camera/image_raw` | `sensor_msgs/Image` | 원본 카메라 영상 |

### 출력

| 토픽 | 형식 | 설명 |
|---|---|---|
| `/robot_arm/perception/usb_port/detections` | `UsbPortDetectionArray` | 같은 영상에서 검출된 포트 후보 목록 |

출력 `header`는 입력 영상의 header를 그대로 사용합니다. keypoint 좌표는 YOLO
letterbox 또는 resize 좌표가 아니라 원본 영상 좌표로 복원해야 합니다.

## 2. `port_pose_node`

YOLO 검출과 카메라 정보를 구독해 후보를 선택하고 `T_camera_port`를 계산합니다.

### 입력

| 토픽 | 형식 | 설명 |
|---|---|---|
| `/robot_arm/perception/usb_port/detections` | `UsbPortDetectionArray` | YOLO 포트 후보 목록 |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | 원본 영상 K, D, frame과 해상도 |

### 출력

| 토픽 | 형식 | 설명 |
|---|---|---|
| `/robot_arm/perception/usb_port/observation` | `UsbPortObservation` | keypoint, pose, 품질과 유효 여부 |
| `/robot_arm/perception/usb_port/pose_camera` | `PoseStamped` | 시각화 및 TF 연결용 pose |

후보 선택 시 다음을 검사합니다.

- `class_name == usb_port`
- 객체 confidence 기준
- keypoint 0, 1, 2, 3의 중복·누락 여부
- 각 keypoint의 visible 및 confidence
- keypoint가 원본 영상 범위 안인지
- YOLO 원본 해상도와 CameraInfo 해상도가 같은지
- YOLO header와 CameraInfo frame이 같은지
- solvePnP 깊이와 재투영 오차

## 3. `manual_detection_node`

YOLO 모델이 준비되지 않았을 때만 사용하는 개발용 입력 노드입니다. 운영
`port_pose_node`를 바꾸지 않고 YOLO와 동일한 검출 메시지를 만듭니다.

| 방향 | 토픽 | 형식 |
|---|---|---|
| 입력 | `/robot_arm/perception/usb_port/manual_keypoints` | `Float64MultiArray` |
| 출력 | `/robot_arm/perception/usb_port/detections` | `UsbPortDetectionArray` |
| 출력 | `/camera/camera_info` | `CameraInfo` |

운영 환경에서는 실행하지 않습니다. 통합 launch의 `use_manual_input` 기본값은
false입니다.

## 4. `pbvs_alignment_node`

유효한 포트 관측, 현재 flange TF, Hand-eye 행렬을 결합해 base 기준 포트 pose와
다음 XY 정렬 목표를 계산합니다. 현재 구현은 flange의 Z와 회전을 보존합니다.

| 방향 | 토픽 | 형식 | 설명 |
|---|---|---|---|
| 입력 | `/robot_arm/perception/usb_port/observation` | `UsbPortObservation` | solvePnP 관측 |
| 입력 | `/joint_states` | `JointState` | OBSERVE_POSE 근접 검사와 자동 기준 저장 |
| TF 입력 | `g_base → joint6_flange` | TF2 | 관측 시각의 flange pose |
| 출력 | `/robot_arm/pbvs/port_pose_base` | `PoseStamped` | base 기준 포트 pose |
| 출력 | `/robot_arm/pbvs/target_flange_pose` | `PoseStamped` | 고정-Z 다음 XY 목표 |
| 출력 TF | `g_base → usb_port` | 동적 TF | 유효한 YOLO·SolvePnP 포트 자세의 RViz 표시 |
| 출력 | `/robot_arm/pbvs/converged` | `Bool` | XY 허용오차 도달 여부 |
| 출력 | `/robot_arm/pbvs/error` | `Vector3Stamped` | base X/Y 오차(m), Yaw 오차(rad) |
| 출력 | `/robot_arm/pbvs/yaw_enabled` | `Bool` | 목표 자세에 Yaw step 포함 여부 |
| 출력 | `/robot_arm/pbvs/status` | `String` | 오차, 제한된 step, 거부 이유 |

이 출력은 `/robot_arm/target_pose`와 분리된 DRY RUN 결과입니다. PBVS 노드는 실제
이동 명령을 발행하지 않습니다. 처음 유효한 관측에서 현재 관절이 설정된
OBSERVE_POSE로부터 최대 3.5도 안이면 현재 flange TF를 정렬 기준으로 자동
저장합니다. 기준 관절 범위를 벗어나면 포트 TF만 발행하고 PBVS 목표는
차단하며, 수동 `capture` 토픽은 사용하지 않습니다.

## 5. `pbvs_step_executor_node`

명시적인 `execute_once` 명령이 들어왔을 때만 최신 PBVS 목표를 검사합니다. 최대
1 mm 간격 waypoint의 MoveIt IK·충돌·관절 점프 사전검사를 통과하면 waypoint를
한 점씩 실행하지 않고 최종 pose를 Cartesian action으로 한 번 보냅니다.

| 방향 | 이름 | 형식 | 설명 |
|---|---|---|---|
| 입력 | `/robot_arm/pbvs/step_command` | `String` | `execute_once`만 허용 |
| Action 출력 | `/robot_arm/cartesian_move` | `CartesianMove` | 고정 Z/Roll/Pitch flange 목표 |
| 출력 | `/robot_arm/pbvs/execution_status` | `String` | 사전검사·실행·거부 이유 |

실행 launch 인자 `enable_pbvs_test_execution`의 기본값은 `false`입니다.

## 6. `moveit_pbvs_closed_loop_execute`

유효한 새 YOLO 관측을 여러 개 확인한 다음 `MoveIt IK → 관절 action → 정지 →
새 YOLO 관측`을 반복합니다. 이 노드가 실행하는 것은 X/Y와 명시적으로 승인한
Yaw뿐이며 Z 하강과 삽입은 포함하지 않습니다.

다음 조건에서는 다음 이동을 보내지 않고 종료합니다.

- YOLO/PBVS 관측 소실 또는 불안정
- 실행기와 정렬 노드의 Yaw 활성 상태 불일치
- 새 영상 XY 오차가 직전보다 3mm 이상 증가
- 새 영상 Yaw 오차가 직전보다 1.5도 이상 증가
- 최대 step, 최대 실행 시간 또는 누적 XY 제한 도달
- MoveIt IK·충돌·FK 또는 실제 TF 이동 검사 실패
- 정지 후 Z 오차 2mm 또는 자세 오차 2도 초과
- 폐루프 시작 Z 대비 누적 이탈 5mm 또는 시작 자세 대비 2도 초과

기본 수렴 조건은 X/Y 3mm 이내이며 Yaw를 켠 경우 Yaw 1도 이내도 함께
만족해야 합니다.

폐루프 시작 시 실제 flange Z와 자세를 한 번 저장하며, 이후 모든 step은
직전 결과가 아니라 이 시작 기준으로 복귀하도록 IK 목표를 계산합니다. 따라서
단발마다 발생한 Z 오차를 다음 step의 정상 기준으로 누적하지 않습니다.

## 7. `arm_motion_node`

상위 제어기와 실제 로봇 실행기의 경계입니다.

### 입력

| 토픽 | 형식 | 설명 |
|---|---|---|
| `/robot_arm/target_pose` | `PoseStamped` | base 기준 목표 flange pose |

### 출력

| 토픽 | 형식 | 설명 |
|---|---|---|
| `/robot_arm/motion_done` | `Bool` | 목표 실행 완료 여부 |
| `/robot_arm/motion_status` | `String` | DRY RUN, 거부 또는 실행 상태 |

현재는 목표를 받아 안전 설정을 검사하고 실제 백엔드 호출을 거부합니다. 이후
MoveIt action 연동은 이 노드 내부에만 추가합니다.

## 7. `usb_insertion_node`

전체 작업 순서를 관리합니다.

### 입력

| 토픽 | 형식 | 값 |
|---|---|---|
| `/robot_arm/usb_insertion/command` | `String` | `start`, `reset` |
| `/robot_arm/perception/usb_port/observation` | `UsbPortObservation` | 포트 pose와 유효 여부 |

### 출력

| 토픽 | 형식 | 설명 |
|---|---|---|
| `/robot_arm/usb_insertion/state` | `String` | 현재 상태 이름 |
| `/robot_arm/usb_insertion/status` | `String` | 전이 결과 또는 실패 이유 |

## 7. Custom interface

`pinkk_usb_insertion_interfaces` 패키지에 다음 메시지를 정의합니다.

```text
Keypoint2D
  index
  x, y
  confidence
  visible

UsbPortDetection
  header
  detection_id
  class_name
  object_confidence
  bbox
  source_image_width, source_image_height
  keypoints[4]

UsbPortDetectionArray
  header
  detections[]

UsbPortObservation
  header
  detection_id
  pose
  keypoints[4]
  object_confidence
  reprojection_error_px
  depth_m
  valid
  rejection_reason
```

Pose와 `valid`를 별도 토픽으로 나누지 않아 서로 다른 영상 프레임의 값이 섞이지
않게 합니다.

## 8. 향후 제어 인터페이스

실제 로봇 이동을 연결할 때 다음 custom message 또는 action을 별도로 정의합니다.

```text
CartesianStep
  header
  frame_id
  delta_xyz
  delta_rpy
  max_speed
  command_id

InsertionResult
  success
  final_state
  retry_count
  failure_reason
```

명령과 완료 응답은 `command_id`로 대응시켜 오래된 완료 신호가 다음 명령에
사용되지 않게 해야 합니다.
