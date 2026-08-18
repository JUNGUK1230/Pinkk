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

위 기본 실행은 관절 `send_angles()`와 Cartesian `send_coords()`를 모두
차단합니다. `/joint_states`와 action server는 제공하지만 실제 이동 goal은
명시적인 실행 게이트가 없으면 거부합니다.

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

위 기본 명령은 관절과 Cartesian 실제 실행을 모두 차단합니다. 브리지 로그에는
다음 두 줄이 표시됩니다.

```text
관절 send_angles action 실행 차단
Cartesian send_coords action API 준비, 실행 차단
```

관측 자세처럼 검증된 관절 목표를 실제 실행할 때만 마지막 인자로 관절 실행을
엽니다. 초기 검증 허용오차는 1도로 낮춥니다.

```bash
bash scripts/calibration/robot_start_bridge.sh \
  5 1.0 false 0.0015 true
```

관절 이동 완료는 단일 오차 표본으로 판정하지 않습니다. 목표 오차와 표본간
변화량이 기준 안이고 `is_moving()==false`인 상태가 5회 연속 확인되어야 합니다.
완료 직전에 `stop()`을 호출하고 2초 동안 목표 자세가 유지되는지도 검사합니다.
취소, timeout, 검증 실패와 bridge 종료에서도 `stop()`을 호출합니다.

Cartesian 실기 시험에서만 다음처럼 최종 실행 게이트를 명시적으로 엽니다. 첫
시험은 브리지 자체의 이동 상한도 1.5 mm로 낮춥니다.

```bash
ros2 launch pinkk_mycobot_bridge trajectory_bridge.launch.py \
  speed:=5 \
  cartesian_execution_enabled:=true \
  cartesian_max_translation_m:=0.0015
```

저장소 스크립트를 사용할 때 같은 명령은 다음과 같습니다.

```bash
bash scripts/calibration/robot_start_bridge.sh 5 5.0 true 0.0015
```

인자는 차례대로 joint speed, joint tolerance, Cartesian 실행 허용, Cartesian 최대
translation, joint 실행 허용, 최대 관절 명령 횟수, 관절 오차 보상 허용,
보상 gain, 관절별 1회 보상 제한, 관절별 누적 보상 제한입니다. 세 번째와
다섯 번째 인자를 생략하면 두 실행 경로 모두 기본 차단되고, 여섯 번째 인자를
생략하면 관절 목표는 기존처럼 한 번만 전송합니다. 일곱 번째 관절 오차 보상도
기본값은 `false`입니다.

하드웨어 오차 진단이나 제한 재보정이 필요할 때만 마지막 인자를 2 또는 3으로
지정합니다.

```bash
bash scripts/calibration/robot_start_bridge.sh \
  5 0.5 false 0.0015 true 3 true 0.8 1.0 2.0
```

재전송은 로봇이 정지했고 관절 표본이 연속으로 안정됐지만 목표 오차가 남았을
때만 수행합니다. 이전 시도보다 최대 관절 오차가 0.1도 이상 감소하지 않으면
남은 횟수가 있어도 중단합니다. 보상을 켜면 `target-actual` 오차의 0.8배를
다음 하드웨어 명령에 더하되, 관절별 한 번에 1도와 원래 MoveIt 목표로부터
누적 2도를 넘지 않습니다. 최종 성공 여부는 보상 명령이 아니라 원래 MoveIt
목표에 대한 실제 관절 오차로 판정합니다. 최대 횟수는 3으로 제한됩니다.
로그에는 매 시도마다 다음 내용이 출력됩니다.

```text
queue 준비 소요시간
send_angles 호출 소요시간과 응답
목표/실제 관절각
관절별 target-actual 오차
적용 보상량과 누적 보상량
최대 관절 오차
재전송 또는 중단 이유
```

관측 자세 복귀에는 1.5~3도처럼 현실적인 관절 허용오차를 사용할 수 있지만,
1mm PBVS에서는 이 값만으로 성공을 판정하면 안 됩니다. 1mm 이동의 관절 변화가
허용오차보다 작을 수 있기 때문입니다. PBVS 실제 성공은 관절 검사와 별도로
이동 후 flange TF의 X/Y 이동, Z 변화, Roll/Pitch 변화를 모두 검사합니다.

노트북:

```bash
ros2 launch pinkk_mycobot_bridge real_execution.launch.py
```

기본 설정은 bridge 시작 3초 후 그리퍼에 `value=0`, `speed=20`을 한 번
보내 완전 닫힘 상태로 고정합니다. PyMyCobot 기준으로 `0`은 완전
닫힘, `100`은 완전 열림입니다. 경고 대기 중 손과 물체를 치워야 하며,
이 명령은 이동 중 반복하지 않습니다. 자동 고정이 필요 없으면
`gripper_initialize_on_startup: false`로 끈니다.

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

PBVS는 `lock_z=true`, `lock_roll_pitch=true`로 요청합니다. 실행 중
`get_coords()`를 읽어 요청한 Z/Roll/Pitch lock과 목표 도달을 감시합니다.
현재 coarse/pre-approach 설정은 위치 5 mm, bridge 자세 6도로 action을
종료한 뒤, 상위 frozen-target 실행기가 실제 자세를 5도 기준으로
다시 판정합니다. 실제 삽입 전에는 별도 프로파일에서 이 허용값을
줄여야 합니다. 이 감시는 명령 후 이탈을 감지하는
소프트웨어 보호이며, 펌웨어 수준의 실시간 안전 제어를 대신하지 않습니다.

Cartesian action도 같은 serial 객체를 사용합니다. joint trajectory action과
Cartesian action이 동시에 들어오면 나중 요청을 거부하므로 별도 `pymycobot`
프로그램에서 `/dev/ttyUSB0`를 열면 안 됩니다.

처음에는 `speed:=5` 또는 `speed:=10`으로 실행하고 작은 이동으로 통신과 관절
방향을 확인합니다. 실제 MyCobot 이동 속도는 이 `speed` 파라미터가 결정합니다.
PBVS Cartesian speed는 USB 삽입 패키지의 `pbvs_test_execution.cartesian_speed`가
결정합니다.
