# Pinkk MyCobot ROS 2 브리지

현재 구성 상태와 다음 작업의 상세 절차는
[`HAND_EYE_HANDOFF_KO.md`](HAND_EYE_HANDOFF_KO.md)를 참고합니다.

첫 번째 연동 단계는 안전을 위해 읽기 전용으로 구성되어 있습니다. 로봇 PC는
`MyCobot280.get_angles()`로 실제 관절각을 읽어 `/joint_states`를 발행하며,
로봇 이동 API는 호출하지 않습니다. MoveIt과 RViz는 가짜 ros2_control 하드웨어
없이 노트북에서 실행합니다.

노트북과 로봇 PC에서 동일한 ROS 네트워크 설정을 사용합니다.

```bash
export ROS_DOMAIN_ID=36
export ROS_LOCALHOST_ONLY=0
```

로봇 PC:

```bash
ros2 launch pinkk_mycobot_bridge joint_state_bridge.launch.py
```

노트북:

```bash
ros2 launch pinkk_mycobot_bridge planning_only.launch.py
```

브리지와 동시에 `sync_plan`, `sync_plan_arduino`, Jupyter 로봇 제어 코드 또는
`/dev/ttyUSB0`를 여는 다른 프로그램을 실행하면 안 됩니다. 로봇 시리얼 포트는
항상 하나의 프로세스만 사용해야 합니다.

## ChArUco 타깃 TF

카메라 노드는 기존 calib.io 11×8 보드 설정을 사용합니다. ChArUco 코너가 35개
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

`trajectory_bridge`는 `/joint_states`를 발행하면서 다음 action을 제공합니다.

```text
/arm_group_controller/follow_joint_trajectory
```

이 브리지는 MoveIt trajectory의 마지막 관절 자세를 MyCobot `send_angles()`로
전달합니다. 실행할 때는 `/dev/ttyUSB0`를 단독으로 사용해야 하므로 읽기 전용
`joint_state_bridge`를 먼저 종료합니다.

로봇 PC:

```bash
ros2 launch pinkk_mycobot_bridge trajectory_bridge.launch.py speed:=10
```

노트북:

```bash
ros2 launch pinkk_mycobot_bridge real_execution.launch.py
```

이 구현은 중간 trajectory point를 시간에 맞춰 재생하지 않고 마지막 관절 자세만
전달합니다. 캘리브레이션용 자세 이동에는 사용할 수 있지만, MoveIt이 계획한
장애물 회피 경로를 실제 로봇이 그대로 따라가는 실행기는 아닙니다.

처음에는 `speed:=5` 또는 `speed:=10`으로 실행하고 작은 이동으로 통신과 관절
방향을 확인합니다. 실제 MyCobot 이동 속도는 이 `speed` 파라미터가 결정합니다.
