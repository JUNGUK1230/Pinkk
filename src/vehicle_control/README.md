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
- 위치 입력 `localization_pose`: 차량 namespace의 확정 pose
- 센서 입력 `/odom`: camera yaw 사이의 상대 회전
- 센서 입력 `/scan`: LiDAR map yaw 검증·제한적 보정 및 장애물 안전정지
- 융합 출력 `localization_pose`: MPC가 사용하는 차량별 rear-axle pose
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

주요 파라미터는 `config/mpc/mpc.yaml`에 있습니다. 현재 실차 검증 기본값은
5 Hz, 1 step, 0.2초 prediction horizon, 전진 0.025 m/s, 후진
0.015 m/s입니다. 전진 코너가 horizon에 들어오면 직선용 곡률 제한을 미리
해제해 코너 진입 전에 조향을 시작합니다. 0.5cm 간격 경로의 nearest progress
검색은 현재 direction 구간 전체에서 가장 가까운 점을 찾습니다. 이는 차량의
현재 위치를 찾는 과정일 뿐이며, 제어 참조는 그 바로 다음 한 점만 사용합니다.

C2 후진 구간은 전진 staging pose 2.5cm 이내에서 0.7초 정지한 뒤 약
29.3cm를 20cm 회전반경으로 연속 후진합니다. 최종 pose의 지도상 후방
여유가 약 5.5~7.2cm이므로
rear LiDAR 안전정지는 5cm로 설정합니다. 후진 최적화가 0속도 local minimum에
빠지지 않도록 TRACKING 중에만 0.002m/s의 최소 후진 속도를 적용합니다.
종점 10cm 전부터 기준 속도와 horizon 진행량을 함께 줄이고, 카메라 위치
분해능을 고려해 종점 4cm 이내에서 정지합니다.

직선 횡오차 복귀 곡률은 최대 `2.3 1/m`, 전체 각속도는 최대
`0.25 rad/s`로 제한합니다. 경로 기준 곡률은 feed-forward로 먼저 적용하고
추가 횡오차 보정만 변화율을 제한합니다. obstacle, pose/scan timeout 또는 heading 오류에서는
최소속도와 관계없이 즉시 0속도로 정지합니다.
초반 직선에서 카메라 위치 노이즈를 과도하게 따라가지 않도록 곡률 변화량과
곡률 변화 비용을 제한하면서, 주차 구간의 최대 곡률은 기존 값으로 유지합니다.

## 현재 제한

이 1차 버전은 장애물을 MPC 최적화 문제에 포함하지 않지만 `/scan`의 차체
전방 18°, 후방 30° sector에 대해 각각 15cm/5cm 비상정지를 적용합니다.
Pinky URDF의 LiDAR 180° 장착 방향도 반영합니다. 기존 차량 안전 계층을
유지하고, 첫 실차 연결에서는 `/pinkk/heading_diagnostics`와 실제 차체 방향을
대조해야 합니다.

## 배터리 상태 LED·LCD와 웹 긴급정지

Pinky에서는 `pinky_status_led.py`와 `pinky_status_lcd.py`를 동시에 실행합니다.
관제 웹의 긴급정지 버튼이
`/pinkk/vehicle_1/set_emergency_stop` 서비스를 호출합니다. 노드는 정지 상태를
래치하고 그때만 `/pinkk/vehicle_1/cmd_vel` publisher를 만들어 0속도를 20Hz로 계속
발행하며 LED를 빨간색으로 점멸합니다.
평상시에는 추가 `/cmd_vel` publisher가 없어 기존 주행 제어기와 충돌하지
않습니다.

두 노드는 차량 namespace의 상대 토픽 `battery/percent`를 함께 구독합니다.
LCD에는 배터리 퍼센트와 버튼 상태가 표시되고, LED는 배터리를 3등분해
66.7% 이상 노랑, 33.3% 이상 66.7% 미만 주황, 33.3% 미만 빨강으로 표시합니다.
긴급정지 때는 LCD에 `긴급정지`가 나오며 LED도 빨간색으로 점멸합니다.
웹에서 충전을 요청하면 LED가 초록색으로 두 번 점멸한 뒤 현재 배터리 구간
색으로 자동 복귀합니다. 긴급정지는 해제될 때까지 빨간색 점멸을 계속하며,
긴급정지 중에는 충전 점멸 요청을 무시합니다.
일시정지는 다음 주행 상태 요청이 들어올 때까지 노란색 점멸을 계속합니다.
긴급정지가 들어오면 일시정지의 노란 점멸보다 빨간 점멸이 우선합니다.

`run_parking_management.sh`는 두 상태 노드와 `run_pinky_services.sh`를 Pinky 홈에
복사하고 namespaced bringup, LED와 LCD 노드를 자동 실행합니다. 수동 실행은
Pinky에서 다음과 같이 합니다.
노드는 현재 래치 상태를 transient-local
`/pinkk/vehicle_1/emergency_stop_state` 토픽으로 발행하며, 웹은 이 상태를 구독해
새로고침 후에도 긴급정지 잠금을 복원합니다.

```bash
ROS_DOMAIN_ID=36 ~/run_pinky_services.sh
```

서비스를 직접 시험하려면 다음 명령을 사용합니다.

```bash
ros2 service call /pinkk/vehicle_1/set_emergency_stop \
  std_srvs/srv/SetBool "{data: true}"
```

해제는 반드시 주변 안전을 확인한 뒤 수행합니다.
관제 웹에서는 해당 Pinky의 `경로 재생성` 버튼을 누르면 확인 후 같은 해제
서비스를 호출하고, 해제 성공 뒤에만 경로 재생성 요청을 발행합니다.

```bash
ros2 service call /pinkk/vehicle_1/set_emergency_stop \
  std_srvs/srv/SetBool "{data: false}"
```
