# MyCobot Hand-eye 캘리브레이션 작업 인수인계

기준일: 2026-07-14  
구성: Eye-in-hand (카메라가 로봇 플랜지에 고정됨)

## 1. 오늘 완료된 범위

- 노트북의 ROS 2 Jazzy + MoveIt 2에서 MyCobot280 모델 로드 완료
- MoveIt `Plan` 동작 확인
- 로봇 PC의 실제 관절각을 `/joint_states`로 발행 완료
- 노트북에서 실제 관절각과 `g_base -> joint6_flange` TF 수신 확인
- 로봇 PC에서 ChArUco 보드 검출 완료
- `camera_optical_frame -> charuco_board` TF 발행 준비 완료
- 실제 검출 결과: 41 corners, reprojection error 0.381 px

아직 구현하지 않은 범위:

- MoveIt 궤적을 실제 MyCobot에 전달하는 실행 브리지
- Easy Handeye2 설치 및 캘리브레이션 GUI 설정
- 자동 포즈 이동 및 자동 샘플 수집
- 최종 `T_flange_camera` 저장 및 별도 검증

현재 로봇 브리지는 **읽기 전용**이다. `get_angles()`만 호출하며 로봇에
이동 명령을 보내지 않는다. MoveIt의 실제 Execute도 비활성화되어 있다.

## 2. 사용 프레임과 데이터 흐름

```text
로봇 PC: MyCobot encoder
  -> /joint_states
  -> 노트북 robot_state_publisher
  -> g_base -> joint6_flange

로봇 PC: USB camera + ChArUco solvePnP
  -> camera_optical_frame -> charuco_board

Easy Handeye2 입력
  -> g_base -> joint6_flange
  -> camera_optical_frame -> charuco_board

최종 출력
  -> joint6_flange -> camera_optical_frame
  -> 프로젝트 표기: T_flange_camera
```

Easy Handeye2의 Eye-in-hand 설정값:

| 역할 | 프레임 |
| --- | --- |
| robot base | `g_base` |
| robot effector | `joint6_flange` |
| tracking base/camera | `camera_optical_frame` |
| tracking marker/target | `charuco_board` |

## 3. 고정 설정

- ROS domain ID: `36` (노트북과 로봇 PC가 반드시 같아야 함)
- ROS localhost only: `0`
- 로봇 포트: `/dev/ttyUSB0`
- MyCobot baud rate: `1000000`
- 카메라: `/dev/video0` 또는 index `0`
- 카메라 해상도: `640 x 480`
- ChArUco: 11 x 8 squares
- square: 15 mm
- marker: 11 mm
- dictionary: `DICT_4X4_1000`
- OpenCV legacy ChArUco pattern: enabled
- 유효 검출: corners 35개 이상, reprojection error 0.7 px 이하

내부 캘리브레이션 파일:

```text
~/Pinkk-robot-arm/src/robot_arm/robot_camera/camera_calibration/results/intrinsics.npz
```

## 4. 내일 시작 전 물리 고정 확인

1. ChArUco 보드를 로봇 베이스와 같은 작업대에 단단히 고정한다.
2. 수집이 끝날 때까지 보드를 움직이거나 회전하지 않는다.
3. 카메라와 플랜지 체결부가 흔들리지 않는지 확인한다.
4. 카메라 초점, 해상도 및 USB 포트를 바꾸지 않는다.
5. 로봇 주변의 충돌 물체를 치우고 비상 정지 수단을 준비한다.
6. 자동 이동을 처음 시험할 때 속도는 5~10%로 제한한다.

보드가 영상 전체에 모두 보일 필요는 없지만, 각 자세에서 최소 35개 이상의
ChArUco corner와 0.7 px 이하 reprojection error가 안정적으로 유지되어야 한다.

## 5. 로봇 PC 시작 절차

### 터미널 A: 실제 관절각 발행

다른 Jupyter kernel 또는 pymycobot 프로그램이 로봇 포트를 사용하지 않는지
먼저 확인한다.

```bash
sudo lsof /dev/ttyUSB0
```

출력이 없어야 한다. 그다음:

```bash
source /opt/ros/jazzy/setup.bash
source ~/venv/mycobot/bin/activate
source ~/mycobot_moveit_ws/install/setup.bash
source ~/mycobot_moveit_ws/install_bridge/setup.bash

export PYTHONPATH="$HOME/venv/mycobot/lib/python3.12/site-packages${PYTHONPATH:+:$PYTHONPATH}"
export ROS_DOMAIN_ID=36
export ROS_LOCALHOST_ONLY=0

ros2 launch pinkk_mycobot_bridge joint_state_bridge.launch.py
```

정상 로그:

```text
READ-ONLY robot connection: port=/dev/ttyUSB0, baud=1000000
```

### 터미널 B: ChArUco TF 발행

영상 창이 필요하면 노트북에서 X11 forwarding으로 접속한다.

```bash
ssh -Y -C jetcobot@192.168.6.1
echo "$DISPLAY"
```

`localhost:10.0`과 비슷한 값이 나와야 한다. 환경 설정 후:

```bash
source /opt/ros/jazzy/setup.bash
source ~/venv/mycobot/bin/activate
source ~/mycobot_moveit_ws/install/setup.bash
source ~/mycobot_moveit_ws/install_bridge/setup.bash

export PYTHONPATH="$HOME/venv/mycobot/lib/python3.12/site-packages${PYTHONPATH:+:$PYTHONPATH}"
export ROS_DOMAIN_ID=36
export ROS_LOCALHOST_ONLY=0

ros2 launch pinkk_mycobot_bridge charuco_tf_bridge.launch.py
```

화면 없는 SSH 터미널에서는:

```bash
ros2 launch pinkk_mycobot_bridge charuco_tf_bridge.launch.py \
  show_preview:=false
```

정상 로그 예시:

```text
ChArUco DETECTED: corners=41, error=0.381px
```

`collect_samples`, `verify` 또는 다른 OpenCV 프로그램을 동시에 실행하지 않는다.
카메라는 한 프로세스만 열어야 한다.

## 6. 노트북 시작 절차

기존 fake-controller `demo.launch.py`가 실행 중이면 먼저 종료한다. 새 터미널에서:

```bash
source /opt/ros/jazzy/setup.bash
source ~/mycobot_moveit_ws/install/setup.bash

export ROS_DOMAIN_ID=36
export ROS_LOCALHOST_ONLY=0

ros2 launch pinkk_mycobot_bridge planning_only.launch.py
```

이 런치는 실제 실행 없이 RViz, robot_state_publisher와 MoveIt planning만 시작한다.

다른 노트북 터미널에서 입력 TF 두 개를 확인한다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/mycobot_moveit_ws/install/setup.bash

export ROS_DOMAIN_ID=36
export ROS_LOCALHOST_ONLY=0

ros2 topic echo /joint_states --once
ros2 run tf2_ros tf2_echo g_base joint6_flange
ros2 run tf2_ros tf2_echo camera_optical_frame charuco_board
```

두 `tf2_echo`가 유효한 translation과 rotation을 계속 출력해야 한다.

## 7. 시간 동기화 확인

노트북과 로봇 PC에서 각각 다음 명령을 거의 동시에 실행한다.

```bash
date +%s.%N
```

가능하면 차이를 0.1초 이하로 유지한다. 시간 차이가 크면 TF lookup의 extrapolation
오류와 잘못된 자세 쌍이 발생할 수 있으므로 캘리브레이션 전에 NTP/chrony를 맞춘다.

## 8. 내일 구현 순서

### 단계 1: 안전한 실제 실행 브리지

기존 읽기 전용 노드를 확장하거나 단일 serial-owner 노드로 교체해 다음 기능을
구현한다.

- `/arm_group_controller/follow_joint_trajectory` action server
- 목표 관절 이름과 6개 관절값 검증
- 관절 범위 검증
- 낮은 속도 및 가속도 제한
- trajectory 시간 순서 검증
- 명령 중 encoder 상태 지속 발행
- 통신 실패/취소/timeout 시 정지
- 하나의 프로세스만 `/dev/ttyUSB0` 소유

공식 저장소의 `sync_plan.py`와 `sync_plan_arduino.py`는 사용하지 않는다. baud rate,
명령 방식 및 안전 검증이 현재 구성과 맞지 않는다.

### 단계 2: 실제 이동 최소 시험

1. 보드/카메라와 무관한 안전 자세에서 시작한다.
2. MoveIt 속도와 가속도를 0.05~0.10으로 제한한다.
3. 관절 하나만 약 3~5도 이동하는 경로를 계획한다.
4. 계획 궤적을 확인한 뒤 한 번만 Execute한다.
5. 실제 로봇과 RViz의 최종 각도가 일치하는지 확인한다.
6. 취소 및 통신 timeout 시 로봇이 정지하는지 확인한다.

이 시험을 통과하기 전에는 자동 캘리브레이션 포즈 실행을 활성화하지 않는다.

### 단계 3: Easy Handeye2 설치 및 설정

- 노트북의 별도 ROS 2 workspace에 Easy Handeye2를 설치한다.
- calibration type: `eye_in_hand`
- robot base: `g_base`
- robot effector: `joint6_flange`
- tracking base: `camera_optical_frame`
- tracking marker: `charuco_board`
- MoveIt planning group: `arm_group`

처음에는 GUI에서 현재 자세 샘플을 수동으로 한 개 받아 TF 이름과 방향을 검증한다.

### 단계 4: 자동 포즈 범위 설정

- 중앙 안전 자세 1개를 기준으로 시작한다.
- roll/pitch/yaw가 모두 변하도록 자세를 구성한다.
- 한 축 회전만 반복하지 않는다.
- 초기 이동 범위는 작게 하고 충돌 없는 범위에서 점차 확장한다.
- 각 자세에서 로봇 정지와 진동 감쇠를 기다린다.
- ChArUco 유효 검출일 때만 샘플을 저장한다.
- 목표 샘플 수는 15~25개로 시작한다.

### 단계 5: 계산과 독립 검증

- Easy Handeye2 결과를 저장한다.
- 결과 방향이 `joint6_flange -> camera_optical_frame`인지 확인한다.
- 프로젝트의 `T_flange_camera` 형식과 단위(m)가 같은지 확인한다.
- 수집에 사용하지 않은 별도 자세에서 고정 보드의 base 좌표 편차를 측정한다.
- 목표 기준은 우선 위치 평균 5 mm 이하, 회전 평균 1도 이하로 잡고 실제 로봇
  반복 정밀도에 맞춰 판단한다.

## 9. 빌드 및 배포 메모

로봇 PC에서는 `--symlink-install`과 setuptools가 충돌해
`option --editable not recognized`가 발생했다. 브리지는 별도 일반 설치 overlay로
빌드했다.

```bash
cd ~/mycobot_moveit_ws
source /opt/ros/jazzy/setup.bash
source ~/venv/mycobot/bin/activate
source ~/mycobot_moveit_ws/install/setup.bash

colcon --log-base log_bridge build \
  --build-base build_bridge \
  --install-base install_bridge \
  --base-paths \
    ~/mycobot_moveit_ws/src \
    ~/Pinkk-robot-arm/ros2_ws/src/pinkk_mycobot_bridge \
  --packages-select pinkk_mycobot_bridge
```

빌드 후에는 반드시 다음 overlay를 source한다.

```bash
source ~/mycobot_moveit_ws/install_bridge/setup.bash
```

ROS 실행 파일이 venv의 pymycobot/OpenCV를 찾도록 `PYTHONPATH`도 설정해야 한다.

## 10. 문제 발생 시 빠른 확인

```bash
echo "$ROS_DOMAIN_ID"                 # 양쪽 모두 36
echo "$DISPLAY"                       # GUI 사용 시 비어 있으면 안 됨
sudo lsof /dev/ttyUSB0                 # serial owner는 하나
sudo lsof /dev/video0                  # camera owner는 하나
ros2 pkg prefix pinkk_mycobot_bridge
ros2 pkg executables pinkk_mycobot_bridge
ros2 topic info /joint_states -v       # publisher 중복 확인
```

`ROS_LOCALHOST_ONLY is deprecated` 경고는 현재 실행 실패 원인이 아니며, 양쪽에서
`ROS_LOCALHOST_ONLY=0`이면 네트워크 discovery는 계속 동작한다.
