# MyCobot Hand-eye 브리지 참고

이 문서는 Hand-eye 과정에서 `pinkk_mycobot_bridge`가 제공하는 ROS 연결과 빠른
진단만 설명합니다.

전체 캘리브레이션 실행 순서는
[`scripts/calibration/README_KO.md`](../../../scripts/calibration/README_KO.md)를
따릅니다. 과거 USB 수동 클릭, Jupyter 직접 이동과 임시 PRE 접근 절차는 최종
자동 USB/PBVS 흐름에서 사용하지 않으므로 이 문서에서 제거했습니다.

## 1. 브리지 역할

```text
MyCobot serial /dev/ttyUSB0
  ↕
pinkk_mycobot_bridge
  ├─ /joint_states
  └─ /arm_group_controller/follow_joint_trajectory
       ↕
노트북 MoveIt/RViz 및 Hand-eye 자동화
```

- 실제 관절각을 `/joint_states`로 발행합니다.
- 노트북의 `FollowJointTrajectory` 목표를 실제 MyCobot 명령으로 변환합니다.
- `/dev/ttyUSB0`는 bridge 한 프로세스만 열어야 합니다.
- 캘리브레이션 계산과 결과 저장은 `pinkk_handeye_automation`이 담당합니다.

## 2. 고정 네트워크 설정

로봇 PC와 노트북 모두 다음 값을 사용합니다.

```bash
export ROS_DOMAIN_ID=36
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
```

스크립트의 기본 Domain은 36이며 `ROS_DOMAIN_ID` 환경변수로 덮어쓸 수 있습니다.

## 3. 로봇 PC 실행

bridge:

```bash
# 저장소 루트에서 실행
bash scripts/calibration/robot_start_bridge.sh 5 5.0
```

ChArUco 화면 포함:

```bash
# 저장소 루트에서 실행
bash scripts/calibration/robot_start_charuco.sh true
```

화면 없이:

```bash
bash scripts/calibration/robot_start_charuco.sh false
```

정상 bridge 로그:

```text
실제 실행 브리지 준비:
port=/dev/ttyUSB0, baud=1000000
action=/arm_group_controller/follow_joint_trajectory
관절 send_angles action 실행 차단
Cartesian send_coords action 실행 차단
```

기본 bridge는 상태만 읽고 두 이동 action을 모두 차단합니다. 관측 자세를 실제로
검증할 때만 다음처럼 관절 실행을 명시적으로 엽니다.

```bash
bash scripts/calibration/robot_start_bridge.sh \
  5 1.0 false 0.0015 true
```

정상 ChArUco 로그:

```text
ChArUco TF:
camera_optical_frame -> charuco_board
```

## 4. 노트북 실행

MoveIt/RViz:

```bash
# 저장소 루트에서 실행
bash scripts/calibration/laptop_start_moveit.sh
```

연결 확인:

```bash
ros2 topic echo /joint_states --once
ros2 action list | grep follow_joint_trajectory
ros2 run tf2_ros tf2_echo g_base joint6_flange
ros2 run tf2_ros tf2_echo camera_optical_frame charuco_board
```

## 5. 좌표계

```text
g_base
  └─ joint6_flange
       └─ camera_optical_frame
            └─ charuco_board
```

Eye-in-hand 결과는 다음 변환입니다.

```text
T_flange_camera
= joint6_flange → camera_optical_frame
```

고정 보드 검증:

```text
T_base_board =
  T_base_flange × T_flange_camera × T_camera_board
```

활성 Hand-eye 값은 하드코딩하지 않고 다음 명령으로 선택합니다.

```bash
bash scripts/calibration/laptop_handeye_data.sh activate RUN
bash scripts/calibration/laptop_publish_handeye_tf.sh
```

## 6. 빌드

로봇 PC:

```bash
# 저장소 루트에서 실행
bash scripts/calibration/robot_build_pinkk.sh
```

노트북:

```bash
# 저장소 루트에서 실행
bash scripts/calibration/laptop_build_pinkk.sh
```

소스 변경 후에는 다시 빌드하고 새 터미널을 사용합니다.

## 7. 빠른 장애 진단

### `/joint_states`가 보이지 않음

1. 로봇 PC bridge가 살아 있는지 확인합니다.
2. 두 PC의 Domain 38과 RMW를 맞춥니다.
3. 오래된 ROS daemon을 종료한 뒤 다시 확인합니다.

```bash
ros2 daemon stop
ros2 topic echo /joint_states --once
```

### serial disconnected 또는 multiple access

```bash
sudo lsof /dev/ttyUSB0
```

bridge 외의 Jupyter와 pymycobot 프로세스를 종료합니다.

### 카메라를 열지 못함

```bash
sudo lsof /dev/video0
```

Flask, 카메라 테스트 프로그램과 이전 ChArUco 노드 중 하나만 남깁니다.

### ChArUco TF가 없음

- 보드가 화면에 충분히 크게 보이는지 확인합니다.
- 현재 intrinsic이 640×480 영상용인지 확인합니다.
- `intrinsics.npz`의 키가 `camera_matrix`, `dist_coeffs`인지 확인합니다.
- X11 문제라면 우선 preview를 `false`로 실행해 TF 자체를 확인합니다.

### DDS symbol lookup 또는 payload 오류

두 PC의 ROS Jazzy 패키지 버전과 `rmw_fastrtps_cpp` 사용 여부를 맞춘 뒤 모든 ROS
프로세스를 재시작합니다. 한 터미널에서 여러 overlay를 역순으로 중복 source하지
않습니다.

## 8. 안전

- bridge 로그에서 목표가 수락된 뒤 로봇 주변에 손을 넣지 않습니다.
- 첫 시험은 speed 5와 충분한 공간에서 수행합니다.
- 자동 수집과 비교는 `check`를 먼저 통과한 후 `execute`합니다.
- 로봇을 움직이는 프로세스를 종료한 뒤 bridge를 마지막에 종료합니다.
