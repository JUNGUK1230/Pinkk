# Vehicle Control

## MPC 경로 추종기

`fused_pose_estimator.py`는 상단 카메라 차체 중심 x/y, 고정 경로 시작 yaw와
차량 wheel odometry를 결합해 heading이 포함된 pose를 만듭니다. 카메라 차량
마스크 장축 yaw와 LiDAR map 정합은 odometry 예측과 10도 이내로 일치할 때만
미세 보정에 사용합니다.
`mpc_path_follower.py`는 이 pose와 중앙제어 고정 경로로 차동구동 차량의
선속도와 각속도를 계산합니다.
상단 카메라 x/y에는 짧은 저역통과 필터를 적용해 차량 mask 중심의 프레임별
흔들림이 조향 명령으로 바로 전달되지 않게 합니다.

입출력:

- 입력 `/pinkk/planned_trajectory`: `x_m, y_m, yaw_rad, direction`
- 위치 입력 `/pinkk/vehicle_pose`: 상단 카메라의 `lidar_map` 차체 중심 x/y
- 센서 입력 `/odom`: camera yaw 사이의 상대 회전
- 센서 입력 `/scan`: LiDAR map yaw 검증·제한적 보정 및 장애물 안전정지
- 융합 출력 `/pinkk/fused_vehicle_pose`: MPC가 사용하는 rear-axle pose
- 실차 출력 `/cmd_vel`: `geometry_msgs/Twist`

MPC는 signed speed와 curvature를 최적화하고 `angular.z = speed × curvature`로
계산합니다. `angular_command_sign`은 Pinky 실차 좌우 회전 시험 결과에 맞춰
설정 파일에서 `-1.0`을 사용합니다. 실차 회전 응답 보정을 위해
`angular_command_gain=1.35`를 적용하고 최종 출력은 `0.35rad/s`로 제한합니다.
따라서 속도가 0이면 각속도도 항상 0이며 제자리
회전하지 않습니다.
위치 비용은 가까운 점까지의 단순 x/y 거리가 아니라 경로 접선 기준 횡오차와
진행방향 오차를 분리합니다. 횡오차에 더 큰 비용을 주므로 중심선을 넘었다가
반대로 복귀하는 점 추종 진동 없이 경로를 따라 수렴합니다.
참조에는 현재 위치 바로 다음 경로점 하나만 사용합니다. 현재 경로 접선의
signed 횡오차와 heading 오차 크기에 비례해 추가 곡률을 계산하며, 경로 자체
곡률은 feed-forward로 적용합니다.
현재 위치 검색은 direction 구간 전체에서 수행하므로 카메라 프레임이 일부
건너뛰어도 뒤쪽 경로점을 계속 쫓지 않습니다. 3cm를 넘는 큰 횡이탈에서는
비선형 이득과 복귀 모드를 사용하고, 0.5cm 안으로 돌아온 뒤 일반 헤딩
안전범위로 복귀합니다.
전진·후진 direction block 사이에서는 정지 시간을 둡니다.

안전 동작:

- pose 또는 경로 timeout 시 0 명령
- solver 실패나 비정상 경로 수신 시 0 명령
- `/scan` timeout, invalid sector 또는 진행 방향 장애물 감지 시 0 명령
- 다른 노드가 `/cmd_vel`을 발행하면 충돌 상태로 정지
- 선속도·가속도·곡률·각속도 제한
- 직선 구간 곡률을 제한해 횡오차를 완만하게 보정
- 직선 경로와 heading이 25° 이상 어긋나면 `HEADING_ERROR_TOO_LARGE` 정지
- 직선·곡선 구분 없이 heading 오류가 한 번 25°를 넘으면 안전정지를 latch하며,
  경로 무효화 또는 MPC 재시작 전에는 자동 재출발하지 않음
- 동일 경로의 주기 재발행은 기존 progress를 초기화하지 않음
- 새 경로에서만 MPC progress와 warm start 초기화
- MPC를 staging에서 재시작하면 첫 pose에 한해 전체 경로의 nearest point를
  찾아 전진 구간을 다시 주행하지 않고 현재 cusp에서 후진을 재개
- 전진→후진 전환 뒤 nearest-point 탐색을 후진 direction block 안으로 제한해
  공간상 가까운 전진 cusp로 되돌아가지 않음
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
 lidar_map_yaw_rad, lidar_match_score_m, distinct_margin_m,
 camera_map_yaw_rad, odom_yaw_rad]
```

주요 파라미터는 `config/mpc/mpc.yaml`에 있습니다. MPC가 실행 중일 때 이
파일을 저장하면 1초 안에 전체 값을 원자적으로 검증해 자동 반영합니다.
정상 반영되면 `Reloaded MPC tuning file`이 출력됩니다. 범위를 벗어난 값은
전체 변경이 거부되므로 기존 설정으로 계속 동작합니다. 토픽 이름과
`tuning_file`, `tuning_reload_period_sec`는 재생성이 필요하므로 변경 후
노드를 재시작해야 합니다.

조정 순서는 다음을 권장합니다.

- 선 복귀가 느리면 `weight_position`을 조금 올리거나
  `weight_curvature_rate`를 조금 내립니다.
- 좌우 진동이 크면 `weight_curvature_rate`와
  `max_curvature_rate_1pmps` 제한을 강화합니다.
- 코너 반응이 늦으면 `curvature_smoothing_points` 또는
  `straight_history_points`를 줄입니다.
- 코너를 너무 일찍 돌면 `straight_lookahead_points`를 줄입니다.
- 속도는 `forward_speed_mps`, `reverse_speed_mps`부터 조정하고 각각의
  `max_*_speed_mps`보다 크게 설정하지 않습니다.

한 번에 한 종류의 값만 20~30% 이내로 바꾸고 저속으로 확인하십시오.
obstacle, pose/scan timeout 또는 heading 오류에서는 설정과 관계없이 즉시
0속도로 정지합니다.

## 현재 제한

이 1차 버전은 장애물을 MPC 최적화 문제에 포함하지 않지만 `/scan`의 차체
전방 18°, 후방 30° sector에 대해 각각 15cm/5cm 비상정지를 적용합니다.
Pinky URDF의 LiDAR 180° 장착 방향도 반영합니다. 기존 차량 안전 계층을
유지하고, 첫 실차 연결에서는 `/pinkk/heading_diagnostics`와 실제 차체 방향을
대조해야 합니다.
