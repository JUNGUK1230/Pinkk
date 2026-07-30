# Vehicle Control

## MPC 경로 추종기

`fused_pose_estimator.py`는 상단 카메라 차체 중심 x/y와 차량 LiDAR scan을
고정 맵에 정합해 heading이 포함된 pose를 만듭니다. `mpc_path_follower.py`는
이 pose와 중앙제어 고정 경로로 차동구동 차량의 선속도와 각속도를 계산합니다.

입출력:

- 입력 `/pinkk/planned_trajectory`: `x_m, y_m, yaw_rad, direction`
- 위치 입력 `/pinkk/vehicle_pose`: 상단 카메라의 `lidar_map` 차체 중심 x/y
- 센서 입력 `/scan`: LiDAR map 절대 yaw 정합
- 융합 출력 `/pinkk/fused_vehicle_pose`: MPC가 사용하는 rear-axle pose
- 실차 출력 `/cmd_vel`: `geometry_msgs/Twist`

MPC는 signed speed와 curvature를 최적화하고 `angular.z = speed × curvature`로
계산합니다. `angular_command_sign`은 Pinky 실차 좌우 회전 시험 결과에 맞춰
`1.0`을 사용합니다. 따라서 속도가 0이면 각속도도 항상 0이며 제자리
회전하지 않습니다.
전진·후진 direction block 사이에서는 정지 시간을 둡니다.

안전 동작:

- pose 또는 경로 timeout 시 0 명령
- solver 실패나 비정상 경로 수신 시 0 명령
- `/scan` timeout, invalid sector 또는 진행 방향 장애물 감지 시 0 명령
- 다른 노드가 `/cmd_vel`을 발행하면 충돌 상태로 정지
- 선속도·가속도·곡률·각속도 제한
- 직선 구간 곡률을 제한해 횡오차를 완만하게 보정
- 직선 경로와 heading이 25° 이상 어긋나면 `HEADING_ERROR_TOO_LARGE` 정지
- 동일 경로의 주기 재발행은 기존 progress를 초기화하지 않음
- 새 경로에서만 MPC progress와 warm start 초기화
- 상단 카메라에서 `e`로 ego 차량을 바꾸면 `/pinkk/path_valid=false`를
  받아 이전 차량 경로를 즉시 폐기하고 정지

실차 설정은 `/cmd_vel`을 직접 사용하므로 기존 Pure Pursuit/Stanley/PID
제어기를 반드시 종료해야 합니다.

## 오프라인 검사

```bash
cd ~/PINKK
.venv/bin/python src/vehicle_control/tests/test_mpc_controller.py
```

이 검사는 실제 `START→C2` CSV를 차동구동 운동학으로 주행해 전진, 정지,
후진, 종점 도착과 무제자리회전 조건을 확인합니다.

## ROS 2 실차 실행

먼저 Pinky에서 기본 bringup을 같은 `ROS_DOMAIN_ID`로 실행하고 `/scan`이
발행되는지 확인합니다. 현재 기본 설정은 IMU를 사용하지 않습니다.

```bash
export ROS_DOMAIN_ID=36
ros2 launch pinky_bringup bringup_robot.launch.xml
```

상단 카메라 localization을 실행한 다음 중앙제어 PC에서 fused pose를 먼저
실행합니다.

```bash
cd ~/PINKK
source /opt/ros/jazzy/setup.bash
source ~/pinky_pro/install/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=36

ros2 run pinkk fused_pose_estimator \
  --ros-args \
  --params-file src/vehicle_control/config/localization/fused_pose.yaml
```

`Fused pose status: TRACKING`을 확인한 후 MPC를 실행합니다.

```bash
cd ~/PINKK
source /opt/ros/jazzy/setup.bash
source ~/pinky_pro/install/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=36

ros2 run pinkk mpc_path_follower \
  --ros-args \
  --params-file src/vehicle_control/config/mpc/mpc.yaml
```

실행 전 `/cmd_vel` publisher가 없는지 확인합니다.

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=36

ros2 topic info -v /cmd_vel
```

MPC 실행 전 `Publisher count: 0`, 실행 후 `Publisher count: 1`이어야 합니다.
처음에는 바퀴를 띄우거나 모터 비상정지를 준비한 상태에서 검증해야 합니다.

heading 진단값은 `/pinkk/heading_diagnostics`에 다음 순서로 발행됩니다.
IMU를 사용하지 않으므로 `imu_yaw_rad` 값은 `NaN`입니다.

```text
[x_m, y_m, imu_yaw_rad, fused_map_yaw_rad,
 lidar_map_yaw_rad, lidar_match_score_m, distinct_margin_m]
```

주요 파라미터는 `config/mpc/mpc.yaml`에 있습니다. 현재 기본값은 4 Hz,
10 step, 2.5초 prediction horizon, 전진 0.06 m/s, 후진 0.02 m/s입니다.
초반 직선에서 카메라 위치 노이즈를 과도하게 따라가지 않도록 곡률 변화량과
곡률 변화 비용을 제한하면서, 주차 구간의 최대 곡률은 기존 값으로 유지합니다.

## 현재 제한

이 1차 버전은 장애물을 MPC 최적화 문제에 포함하지 않지만 `/scan`의 차체
전방 18°, 후방 30° sector에 대해 각각 15cm/12cm 비상정지를 적용합니다.
Pinky URDF의 LiDAR 180° 장착 방향도 반영합니다. 기존 차량 안전 계층을
유지하고, 첫 실차 연결에서는 `/pinkk/heading_diagnostics`와 실제 차체 방향을
대조해야 합니다.
