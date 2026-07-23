# Pinkk MyCobot ROS 2 브리지

캘리브레이션 관련 실행과 진단은
[`HAND_EYE_HANDOFF_KO.md`](HAND_EYE_HANDOFF_KO.md)를 참고합니다.

현재 표준은 `trajectory_bridge`가 실제 관절각을 `/joint_states`로 발행하고
MoveIt의 `FollowJointTrajectory` 목표를 로봇에 전달하는 구성입니다.
`joint_state_bridge`와 `planning_only`는 읽기 전용 진단이 필요할 때만 사용합니다.

노트북과 로봇 PC에서 동일한 ROS 네트워크 설정을 사용합니다.

```bash
export ROS_DOMAIN_ID=36
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

로봇 PC:

```bash
cd ~/Pinkk-robot-arm
bash scripts/calibration/robot_start_bridge.sh 5 5.0
```

노트북:

```bash
cd ~/Desktop/Pinkk-robot-arm
bash scripts/calibration/laptop_start_moveit.sh
```

브리지와 동시에 `sync_plan`, `sync_plan_arduino`, Jupyter 로봇 제어 코드 또는
`/dev/ttyUSB0`를 여는 다른 프로그램을 실행하면 안 됩니다. 로봇 시리얼 포트는
항상 하나의 프로세스만 사용해야 합니다.

## ChArUco 타깃 TF

카메라 노드는 기존 calib.io 11×8 보드 설정을 사용합니다. ChArUco 코너가 25개
이상이고 재투영 오차가 0.7 px 이하인 유효 검출만 다음 TF로 발행합니다.

```text
camera_optical_frame -> charuco_board
```

```bash
ros2 launch pinkk_mycobot_bridge charuco_tf_bridge.launch.py
```

OpenCV 카메라 좌표계는 ROS optical frame과 동일하게 X축은 영상 오른쪽, Y축은
아래쪽, Z축은 카메라 정면을 향합니다. `camera_link`가 optical 축으로 정의된
프레임이 아니라면 부모 프레임 이름을 `camera_link`로 변경하면 안 됩니다.

## 실제 MoveIt 실행 브리지

`trajectory_bridge`는 `/joint_states`를 발행하면서 두 action을 제공합니다.

```text
/arm_group_controller/follow_joint_trajectory
/robot_arm/cartesian_move
```

이 브리지는 MoveIt trajectory의 마지막 관절 자세를 MyCobot `send_angles()`로
전달합니다. 실행할 때는 `/dev/ttyUSB0`를 단독으로 사용해야 하므로 읽기 전용
`joint_state_bridge`를 먼저 종료합니다.

로봇 PC:

```bash
ros2 launch pinkk_mycobot_bridge trajectory_bridge.launch.py speed:=10
```

위 기본 명령은 joint action은 기존 방식으로 유지하지만 새 Cartesian action의 실제
`send_coords()` 실행은 차단합니다. 브리지 로그에는 다음이 표시됩니다.

```text
Cartesian send_coords action API 준비, 실행 차단
```

Cartesian 실기 시험에서만 다음처럼 최종 실행 게이트를 명시적으로 엽니다. 첫
시험은 브리지 자체의 이동 상한도 1.5 mm로 낮춥니다.

```bash
ros2 launch pinkk_mycobot_bridge trajectory_bridge.launch.py \
  speed:=5 \
  cartesian_execution_enabled:=true \
  cartesian_max_translation_m:=0.0015
```

노트북:

```bash
ros2 launch pinkk_mycobot_bridge real_execution.launch.py
```

이 구현은 중간 trajectory point를 시간에 맞춰 재생하지 않고 마지막 관절 자세만
전달합니다. 캘리브레이션용 자세 이동에는 사용할 수 있지만, MoveIt이 계획한
장애물 회피 경로를 실제 로봇이 그대로 따라가는 실행기는 아닙니다.

`/robot_arm/cartesian_move`는 `pinkk_usb_insertion_interfaces/action/CartesianMove`
형식의 `g_base` 기준 flange 목표를 받아 MyCobot `send_coords()`로 전달합니다.
ROS의 meter/quaternion은 로봇의 mm/[rx, ry, rz] degree로 변환하며, Hand-Eye에서
확인한 intrinsic ZYX 규약을 사용합니다. 브리지는 시작할 때 다음 조건을 모두
확인하지 못하면 Cartesian goal을 거부합니다.

- `get_coords()`와 `send_coords()` API가 존재하는가?
- reference frame이 base(0)인가?
- end type이 flange(0)인가?
- `cartesian_execution_enabled=true`를 명시했는가?
- 목표 frame이 `g_base`인가?
- 이동량과 회전량이 각각 10.5 mm, 2.1도 제한 이내인가?
- 요청한 speed와 mode가 유효한가?

PBVS는 `lock_z=true`, `lock_roll_pitch=true`로 요청합니다. 실행 중 10 Hz로
`get_coords()`를 읽어 시작 Z에서 2 mm, 시작 Roll/Pitch에서 3도 이상 벗어나면
`stop()` 후 action을 실패 처리합니다. 최종 허용오차는 위치 0.5 mm, 자세 1도이며
15초 안에 도달하지 못해도 정지합니다. 이 감시는 명령 후 이탈을 감지하는
소프트웨어 보호이며, 펌웨어 수준의 실시간 안전 제어를 대신하지 않습니다.

Cartesian action도 같은 serial 객체를 사용합니다. joint trajectory action과
Cartesian action이 동시에 들어오면 나중 요청을 거부하므로 별도 `pymycobot`
프로그램에서 `/dev/ttyUSB0`를 열면 안 됩니다.

처음에는 `speed:=5` 또는 `speed:=10`으로 실행하고 작은 이동으로 통신과 관절
방향을 확인합니다. 실제 MyCobot 이동 속도는 이 `speed` 파라미터가 결정합니다.
PBVS Cartesian speed는 USB 삽입 패키지의 `pbvs_test_execution.cartesian_speed`가
결정합니다.
