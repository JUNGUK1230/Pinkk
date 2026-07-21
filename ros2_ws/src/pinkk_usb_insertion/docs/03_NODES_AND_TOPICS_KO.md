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

## 4. `arm_motion_node`

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

## 5. `usb_insertion_node`

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

## 6. Custom interface

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

## 7. 향후 제어 인터페이스

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
