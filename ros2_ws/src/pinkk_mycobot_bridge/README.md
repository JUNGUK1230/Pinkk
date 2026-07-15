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
