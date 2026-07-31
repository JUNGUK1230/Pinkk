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

세션 도중 bridge는 이미 실행 중이고 초기 자세 복귀만 다시 필요하면:

```bash
ros2 launch pinkk_usb_insertion return_to_observe.launch.py
```

## 단발 명령

```bash
ros2 topic pub --once /robot_arm/hybrid/command \
  std_msgs/msg/String "{data: coarse_xy}"
```

초기 관측의 keypoint Yaw를 저장하고 coarse XY 이동 완료 후 Joint6 Yaw를
한 번 이어서 실행:

```bash
ros2 topic pub --once /robot_arm/hybrid/command \
  std_msgs/msg/String "{data: coarse_xy_then_yaw}"
```

```bash
ros2 topic pub --once /robot_arm/hybrid/command \
  std_msgs/msg/String "{data: refine_xy}"
```

```bash
ros2 topic pub --once /robot_arm/hybrid/command \
  std_msgs/msg/String "{data: refine_yaw}"
```

상태:

```bash
ros2 topic echo /robot_arm/hybrid/status
```

각 명령은 한 번만 움직인 뒤 새 관측을 기다린다. 현재는 시험 단계이며
자동 연속 실행과 실제 삽입은 구현하지 않았다.

진행 과정과 알려진 문제는 `docs/DEVELOPMENT_LOG_KO.md`에 정리되어 있다.
