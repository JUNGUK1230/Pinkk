# MyCobot Hand-eye 캘리브레이션 작업 인수인계

기준일: 2026-07-15

구성: Eye-in-hand (카메라가 로봇 플랜지에 고정됨)

## 1. 현재 완료된 범위

- 노트북의 ROS 2 Jazzy + MoveIt 2에서 MyCobot280 모델 로드 완료
- MoveIt `Plan` 및 실제 `Execute` 브리지 동작 확인
- 로봇 PC의 실제 관절각 `/joint_states` 발행과 궤적 실행 완료
- 노트북에서 실제 관절각과 `g_base -> joint6_flange` TF 수신 확인
- 로봇 PC에서 ChArUco 보드 검출 및 TF 발행 완료
- 자동 관측 자세 생성, 이동, 유효 검출 확인 및 샘플 수집 완료
- Easy Handeye2로 Eye-in-hand 계산 및 결과 저장 완료
- 최종 `joint6_flange -> camera_optical_frame` static TF 검증 완료
- Flask MJPEG 영상에서 USB 포트 네 점 클릭 및 solvePnP 완료
- `camera_optical_frame -> usb_port` TF 발행과 `g_base -> usb_port` 조회 완료
- 로봇 PC Jupyter에서 TF 기반 PRE 위치 이동 시험 진행

남은 범위:

- 그리퍼와 USB 충전기의 실제 TCP 변환 측정
- 가까운 거리에서 USB 포트 재검출 및 PBVS/IBVS 미세 보정
- 힘/접촉 조건을 포함한 실제 USB 삽입
- 반복 실험을 통한 최종 위치·각도 오차 통계

`trajectory_bridge`는 `/joint_states`를 발행하면서 MoveIt의 마지막 관절 목표를
실제 MyCobot에 전달한다. 로봇 PC Jupyter에서 `pymycobot`을 직접 사용할 때는
반드시 이 브리지를 먼저 종료해야 한다. `/dev/ttyUSB0`의 소유자는 항상 하나여야
한다.

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

## 4. 작업 시작 전 물리 고정 확인

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

## 8. 2026-07-14 작성 계획 기록

아래 단계는 최초 연동 시 작성한 계획이다. 2026-07-15 기준으로 실제 실행 브리지,
Easy Handeye2, 자동 수집과 최종 Hand-eye 저장까지 완료했다. 현재 최신 실행 절차는
11절부터 따른다.

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

## 11. 최종 Hand-eye 결과

Easy Handeye2 자동 수집은 유효 샘플 20개로 완료했다. 최종 변환 방향은 다음과
같다.

```text
joint6_flange -> camera_optical_frame
= T_flange_camera
```

저장된 Easy Handeye2 결과:

```text
~/.ros2/easy_handeye2/calibrations/pinkk_eye_in_hand.calib
```

프로젝트 NumPy 파일:

```text
~/Pinkk-robot-arm/src/robot_arm/robot_camera/handeye_calibration_1828/data/T_flange_camera.npy
```

최종 translation과 quaternion:

```text
translation [m]
[-0.032326655, -0.040054972, 0.030691236]

quaternion xyzw
[-0.008700106, 0.002006141, -0.374364741, 0.927238548]
```

노트북에서 실제 Hand-eye TF를 발행하는 명령:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=36
export ROS_LOCALHOST_ONLY=0

ros2 run tf2_ros static_transform_publisher \
  --x -0.032326655 \
  --y -0.040054972 \
  --z 0.030691236 \
  --qx -0.008700106 \
  --qy 0.002006141 \
  --qz -0.374364741 \
  --qw 0.927238548 \
  --frame-id joint6_flange \
  --child-frame-id camera_optical_frame
```

Easy Handeye2를 동시에 실행하면 같은 부모·자식 TF가 중복될 수 있으므로 종료한다.

## 12. USB 포트 수동 검출 및 TF 실행 절차

현재 단계의 전체 계산은 다음과 같다.

```text
T_base_usb
= T_base_flange @ T_flange_camera @ T_camera_usb
```

`T_camera_usb`는 USB-A 포트의 네 모서리를 클릭한 뒤 `solvePnP`로 구한다. 화면
방향이 아니라 USB 실제 규격 축을 기준으로 클릭한다.

```text
1→2 = USB 긴 변 11.5 mm
2→3 = 인접한 짧은 변 4.5 mm
3→4→1 = 나머지 둘레
```

USB 긴 방향이 영상에서 세로라면 `좌상단 → 좌하단 → 우하단 → 우상단`처럼
클릭한다. 긴 변과 짧은 변을 바꾸면 평면 PnP의 재투영 오차가 작아도 깊이와
기울기가 크게 틀릴 수 있다.

### 12.1 로봇 PC 터미널 A: 실제 로봇 브리지

```bash
source /opt/ros/jazzy/setup.bash
source ~/venv/mycobot/bin/activate
source ~/mycobot_moveit_ws/install/setup.bash
source ~/mycobot_moveit_ws/install_bridge/setup.bash

export PYTHONPATH="$HOME/venv/mycobot/lib/python3.12/site-packages${PYTHONPATH:+:$PYTHONPATH}"
export ROS_DOMAIN_ID=36
export ROS_LOCALHOST_ONLY=0

ros2 launch pinkk_mycobot_bridge trajectory_bridge.launch.py speed:=10
```

### 12.2 로봇 PC 터미널 B: Flask 카메라

기존 Flask MJPEG 서버를 실행한다. 노트북에서 다음 주소가 열려야 한다.

```text
http://192.168.6.1:5000/stream
```

`charuco_tf_bridge`와 Flask를 동시에 실행하면 `/dev/video0`가 충돌할 수 있으므로
ChArUco 노드는 종료한다.

### 12.3 노트북 터미널 A: MoveIt과 RViz

```bash
source /opt/ros/jazzy/setup.bash
source ~/mycobot_moveit_ws/install/setup.bash
source ~/mycobot_moveit_ws/install_bridge/setup.bash

export ROS_DOMAIN_ID=36
export ROS_LOCALHOST_ONLY=0

ros2 launch pinkk_mycobot_bridge real_execution.launch.py
```

### 12.4 노트북 터미널 B: Hand-eye static TF

11절의 `static_transform_publisher` 명령을 실행하고 계속 켜 둔다.

### 12.5 노트북 터미널 C: USB 클릭 TF

```bash
source /opt/ros/jazzy/setup.bash
source ~/mycobot_moveit_ws/install/setup.bash

export ROS_DOMAIN_ID=36
export ROS_LOCALHOST_ONLY=0

cd ~/Desktop/Pinkk-robot-arm

python3 -m \
  src.robot_arm.robot_camera.handeye_calibration_1828.applications.manual_usb_tf \
  --url http://192.168.6.1:5000/stream
```

OpenCV 창에서 `f`로 화면을 고정하고 네 점을 클릭한다. `r`은 이전 결과 초기화,
`q`는 종료다. 로봇을 이동한 뒤에는 반드시 `r`을 누르고 새 영상에서 다시
클릭한다.

### 12.6 노트북 터미널 D: 최종 base 좌표 확인

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=36
export ROS_LOCALHOST_ONLY=0

ros2 run tf2_ros tf2_echo g_base usb_port
```

2026-07-15 시험 측정값:

```text
Translation [m]: [0.242, -0.008, -0.070]
Quaternion xyzw: [0.911, -0.341, -0.090, 0.215]
RPY [degree]: [153.088, 0.988, -41.309]
```

USB 포트는 작업대에 평평하게 고정되어 있으므로 현재 시험에서는 solvePnP의
roll/pitch를 로봇 목표 자세로 사용하지 않는다. 다음 규칙을 사용한다.

```text
RX = -180 deg
RY = 0 deg
RZ = USB Yaw + GRIPPER_YAW_OFFSET
```

현재 육안 정렬로 구한 방향 차이는 다음과 같다.

```text
GRIPPER_YAW_OFFSET = -37.961 deg
현재 USB Yaw       = -41.309 deg
현재 목표 RZ        = -79.270 deg
```

이 값은 위치/TCP 오프셋이 아니라 USB 좌표축과 그리퍼 방향 사이의 시험용 각도
차이다. 그리퍼 장착 방향이 바뀌면 다시 측정한다.

## 13. 로봇 PC Jupyter 직접 이동 절차

이 절차는 MoveIt이 아니라 로봇 PC Jupyter의 `pymycobot`으로 직접 이동하는
현재 시험 방식이다. 시작 전에 `trajectory_bridge`를 `Ctrl+C`로 종료하고 다음
명령에서 출력이 없는지 확인한다.

```bash
sudo lsof /dev/ttyUSB0
```

Jupyter kernel만 `/dev/ttyUSB0`를 열어야 한다. Flask는 `/dev/video0`를 사용하므로
계속 실행할 수 있다.

### 13.1 현재 TF를 입력한 PRE 이동 코드

아래 값은 12.6절의 현재 USB 측정에만 대응한다. USB를 다시 검출하면
`USB_X_M`, `USB_Y_M`, `USB_Z_M`, `USB_YAW_DEG`를 새 TF 값으로 교체한다.

```python
import time

USB_X_M = 0.242
USB_Y_M = -0.008
USB_Z_M = -0.070
USB_YAW_DEG = -41.309

FIXED_RX = -180.0
FIXED_RY = 0.0
GRIPPER_YAW_OFFSET_DEG = -37.961

TEST_STANDOFF_MM = 220.0
XY_CLEARANCE_MM = 30.0

XY_SPEED = 10
YAW_SPEED = 5
Z_SPEED = 5


def get_valid_coords(mc, attempts=15):
    for _ in range(attempts):
        coords = mc.get_coords()
        if isinstance(coords, list) and len(coords) == 6:
            return [float(value) for value in coords]
        time.sleep(0.2)
    raise RuntimeError("로봇 좌표 읽기 실패")


usb_x_mm = USB_X_M * 1000.0
usb_y_mm = USB_Y_M * 1000.0
usb_z_mm = USB_Z_M * 1000.0
target_rz = (
    USB_YAW_DEG + GRIPPER_YAW_OFFSET_DEG + 180.0
) % 360.0 - 180.0

pre_target = [
    usb_x_mm,
    usb_y_mm,
    usb_z_mm + TEST_STANDOFF_MM,
    FIXED_RX,
    FIXED_RY,
    target_rz,
]
xy_approach_z = pre_target[2] + XY_CLEARANCE_MM

current = get_valid_coords(mc)
xy_target = [
    pre_target[0], pre_target[1], xy_approach_z,
    FIXED_RX, FIXED_RY, current[5],
]

print("1. XY 목표:", xy_target)
mc.send_coords(xy_target, XY_SPEED, 0)
time.sleep(4.0)

current = get_valid_coords(mc)
yaw_target = [
    current[0], current[1], current[2],
    FIXED_RX, FIXED_RY, target_rz,
]

print("2. Yaw 목표:", yaw_target)
mc.send_coords(yaw_target, YAW_SPEED, 0)
time.sleep(4.0)

z_target = [
    pre_target[0], pre_target[1], pre_target[2],
    FIXED_RX, FIXED_RY, target_rz,
]

print("3. PRE Z 목표:", z_target)
mc.send_coords(z_target, Z_SPEED, 1)
time.sleep(4.0)
print("최종 pose:", get_valid_coords(mc))
```

현재 입력의 계산 결과:

```text
PRE target   = [242.0, -8.0, 150.0, -180.0, 0.0, -79.27]
XY approach Z = 180.0 mm
```

`XY_CLEARANCE_MM`는 PRE보다 높은 곳에서 먼저 XY를 맞추기 위한 값이므로
`pre_target[2] + XY_CLEARANCE_MM`으로 계산한다.

### 13.2 PRE에서 천천히 수직 하강

공구 TCP를 아직 측정하지 않았으므로 아래 기본값은 동작 확인용으로 20 mm만
하강한다. USB 표면 좌표까지 직접 내려가지 않는다.

```python
import math
import time


def descend_z_slowly(mc, distance_mm=20.0, step_mm=2.0, speed=3):
    if distance_mm <= 0.0 or step_mm <= 0.0:
        raise ValueError("거리와 step은 0보다 커야 합니다")

    start = get_valid_coords(mc)
    start_z = start[2]
    steps = math.ceil(distance_mm / step_mm)

    try:
        for index in range(1, steps + 1):
            moved = min(index * step_mm, distance_mm)
            target_z = start_z - moved
            target = [
                pre_target[0], pre_target[1], target_z,
                FIXED_RX, FIXED_RY, target_rz,
            ]
            print(f"[{index}/{steps}] 목표 Z={target_z:.2f} mm")
            mc.send_coords(target, speed, 1)
            time.sleep(1.0)
            print("실제 pose:", get_valid_coords(mc))
    except KeyboardInterrupt:
        if hasattr(mc, "stop"):
            mc.stop()
        print("사용자 중지 pose:", get_valid_coords(mc))
        return None

    return get_valid_coords(mc)


descend_z_slowly(mc, distance_mm=20.0, step_mm=2.0)
```

## 14. 직접 Jupyter 제어 후 TF를 다시 사용할 때

Jupyter가 `/dev/ttyUSB0`를 직접 소유하는 동안에는 `trajectory_bridge`가 꺼져 있어
실제 `/joint_states`와 `g_base -> joint6_flange`가 갱신되지 않는다. 따라서 직접
이동한 뒤 화면에 남아 있는 `g_base -> usb_port`를 새 측정값으로 믿으면 안 된다.

다시 검출하려면 다음 순서를 따른다.

1. Jupyter의 로봇 명령 실행을 끝내고 kernel의 시리얼 연결을 종료한다.
2. `sudo lsof /dev/ttyUSB0`로 포트가 비었는지 확인한다.
3. 로봇 PC에서 `trajectory_bridge.launch.py`를 다시 실행한다.
4. 노트북에서 `/joint_states`와 `g_base -> joint6_flange` 갱신을 확인한다.
5. USB 클릭 창에서 `r`을 누른 후 새 영상의 네 모서리를 다시 클릭한다.
6. 새 `g_base -> usb_port`를 읽고 Jupyter 입력값을 갱신한다.

반복 PBVS/IBVS 단계에서는 이 재시작 과정이 비효율적이므로 최종 구현은 Jupyter가
시리얼을 직접 열지 않고 ROS의 `/compute_ik`와
`/arm_group_controller/follow_joint_trajectory`를 호출하는 구조로 전환한다.
