# Pinkk USB Port Alignment

YOLO/SolvePnP로 USB 포트를 찾고 PBVS 거친 이동과 영상 기반 XY/Yaw
미세 보정을 PyMyCobot bridge로 실행하는 ROS 2 패키지다.

## 설정 파일

- 노트북 전체 설정: `config/hybrid_runtime.yaml`
- 인식·PBVS 공통값: `config/insertion_control.yaml`
- 카메라 내부 보정: `config/camera_intrinsics.yaml`
- Hand-eye: `config/handeye.yaml`
- 로봇 PC bridge: `pinkk_mycobot_bridge/config/trajectory_bridge.yaml`

평소에는 터미널에서 파라미터를 입력하지 않고 YAML을 수정한다.

## 빌드

```bash
cd ~/Desktop/Pinkk-robot-arm/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install \
  --packages-select pinkk_usb_insertion_interfaces \
  pinkk_usb_insertion
```

로봇 PC에서는 동기화된 소스에서 제공 스크립트로 세 패키지를
`~/mycobot_moveit_ws/install_pinkk`에 빌드한다.

```bash
cd ~/Pinkk-robot-arm
./scripts/calibration/robot_build_pinkk.sh
```

## 실행

로봇 PC 터미널:

```bash
cd ~/Pinkk-robot-arm
./scripts/run_robot_bridge.sh
```

이 명령은 통합 trajectory bridge를 시작하고 2초 뒤 초기 관측 자세로
한 번 복귀한다. bridge는 이후 세션 동안 계속 실행된다.

노트북:

```bash
cd ~/Desktop/Pinkk-robot-arm
./scripts/run_laptop_alignment.sh
```

노트북 launch는 robot_state_publisher, 카메라, YOLO, SolvePnP, PBVS
계산기와 단발 실행기를 함께 시작한다.

현재 권장 실기 경로는 초기 PBVS 목표를 한 번 저장한 뒤 XY/Yaw를 맞추고,
관절 Jacobian Z와 Cartesian XY/Roll-Pitch를 혼합해 포트 기반 목표 Z의
15mm 안전 여유까지 접근하는 frozen-target 시험이다. 기존 alignment
스크립트와 동시에 실행하지 않는다.

```bash
./scripts/run_laptop_frozen_target_test.sh
```

상세 흐름은 `docs/FROZEN_TARGET_TEST_KO.md`, 실제 실패 원인과 해결 기록은
`docs/TROUBLESHOOTING_KO.md`를 참고한다.

세션 도중 bridge는 이미 실행 중이고 초기 자세 복귀만 다시 필요하면:

```bash
ros2 launch pinkk_usb_insertion return_to_observe.launch.py
```

## 단발 명령

```bash
ros2 topic pub --once /robot_arm/hybrid/command \
  std_msgs/msg/String "{data: coarse_xy}"
```

`coarse_xy_then_yaw` 한 명령은 coarse XY/Z 이동, 포트 소실 시 mode 1
Z 복구, 새 관측 기반 `refine_xy` 반복, Joint6 Yaw를 순서대로 실행한다.
XY 오차가 `yaw_start_xy_tolerance_m` 안에 들어오지 않으면 YAML의
`automatic_refine_xy_max_cycles`까지만 보정하고 Yaw 없이 정지한다:

```bash
ros2 topic pub --once /robot_arm/hybrid/command \
  std_msgs/msg/String "{data: coarse_xy_then_yaw}"
```

XY와 Z 복구는 mode 1 직선 이동을 요청하고 Roll/Pitch 목표는 초기 관측
기준을 사용한다. 이동 중 시작 Roll/Pitch에서 bridge 허용값을 넘으면
중단한다. Yaw는 이동 직전 현재값에서 Joint6만 회전하며, Yaw 후 포트
재관측은 요구하지 않고 정지한다.

```bash
ros2 topic pub --once /robot_arm/hybrid/command \
  std_msgs/msg/String "{data: refine_xy}"
```

```bash
ros2 topic pub --once /robot_arm/hybrid/command \
  std_msgs/msg/String "{data: refine_yaw}"
```

포트가 화면에서 사라졌으면 초기 관절자세로 복귀하기 전에 Z-only 복구를
한 단계 시험할 수 있다. 현재 X/Y를 목표로 유지하고 Z는 YAML의
`z_recovery_step_m`만큼 초기 관측 Z 방향으로 올린다. 초기 관측 Z를 넘지는
않는다. 일반 XY 이동은 `cartesian_mode`를 사용하고 Z-only 복구만
`z_recovery_cartesian_mode`를 사용한다. 기본 시험값은 직선 좌표 이동을
요청하는 mode 1이며, 목표 Roll/Pitch는 초기 관측값을 사용하고 Yaw는
이동 직전 현재값을 유지한다.

```bash
ros2 topic pub --once /robot_arm/hybrid/command \
  std_msgs/msg/String "{data: recover_z_once}"
```

`coarse_xy_then_yaw`는 재관측 timeout이 발생하면 같은 복구를
`z_recovery_max_attempts` 횟수까지 자동 실행한다. Z는 경로 잠금을 사용하지
않지만 Roll/Pitch는 이동 중 bridge 허용값으로 감시하며, 매 복구 이동 후
기존 PBVS 목표를 폐기하고 새 검출을 기다린다.

### waypoint_pbvs_align: 마지막 가시 자세 복귀 + 제한된 waypoint stop-and-go

PBVS 절대 목표까지 한 번에 이동하지 않고, 목표 방향으로 YAML의
`waypoint_maximum_xy_step_m`(15mm)/`waypoint_maximum_z_step_m`(10mm)까지만
이동한 뒤 `waypoint_settle_seconds`(0.8초) 대기하고 재관측하는 것을
`waypoint_max_cycles`(6회)까지 자동 반복하는 명령이다.

```bash
ros2 topic pub --once /robot_arm/hybrid/command \
  std_msgs/msg/String "{data: waypoint_pbvs_align}"
```

동작:
- 매 waypoint 이동 성공(재관측 성공) 후 `last_visible_flange_pose`를
  방금 도착한 실제 자세로 갱신한다.
- 재관측에 실패하면(포트가 화면에서 사라지면) 임의로 Z를 올리지 않고
  `last_visible_flange_pose`(이번 명령이 방금 실제로 지나온 도달 가능한
  자세)로 Cartesian 복귀한 뒤 재관측하고 자동 연속 이동을 중단한다.
  `enable_last_visible_pose_recovery: false`로 끄면 옛 방식처럼 즉시
  REJECTED로 끝난다.
- 재관측 XY 잔여오차가 `yaw_start_xy_tolerance_m`(5mm) 이하가 되면 XY
  이동을 멈추고 `refine_yaw`와 같은 로직으로 Joint6 Yaw를 한 번 실행한다.
- Yaw 후 반드시 새 관측으로 XY/Yaw를 다시 확인한다. 이 XY가 5mm를
  넘으면 `refine_xy`를 한 번 실행하고 끝낸다.
- `waypoint_max_cycles`에 도달하거나 누적 이동량이
  `waypoint_max_cycles × waypoint_maximum_xy_step_m`을 넘으면 XY가 아직
  수렴하지 않아도 자동 이동을 멈추고 상태를 보고한다. 실제 삽입은
  이 명령의 범위에 포함하지 않는다.

상태:

```bash
ros2 topic echo /robot_arm/hybrid/status
```

각 명령은 새 관측을 기다리며 진행한다(waypoint_pbvs_align은 한 명령 안에서
여러 waypoint를 자동 반복). Z-only 복구(`recover_z_once`,
`coarse_xy_then_yaw`의 자동 복구)가 최대 횟수 후에도 실패하면 추가
이동을 중단한다. 현재는 시험 단계이며 실제 삽입은 구현하지 않았다.

진행 과정은 `docs/DEVELOPMENT_LOG_KO.md`, 현재 실행 순서는
`docs/FROZEN_TARGET_TEST_KO.md`, 제어 문제 해결 이력은
`docs/TROUBLESHOOTING_KO.md`에 정리되어 있다.
