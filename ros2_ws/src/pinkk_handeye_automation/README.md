# Pinkk Hand-eye 자동화와 좌표 정확도 검증

카메라 내부 캘리브레이션부터 결과 파일 선택까지의 대표 흐름은
[`../../../src/robot_arm/robot_camera/CALIBRATION_GUIDE_KO.md`](../../../src/robot_arm/robot_camera/CALIBRATION_GUIDE_KO.md)를
먼저 확인합니다.

이 ROS 2 패키지에는 성격이 다른 두 실험 도구가 들어 있습니다.

| 실행 파일 | 목적 | 상태 |
|---|---|---|
| `auto_collect` | ChArUco 관측 자세 이동과 Easy Handeye2 샘플 자동 수집 | 캘리브레이션 도구 |
| `usb_pre_approach` | Hand-eye·SolvePnP·TF 좌표가 실제 로봇 위치와 맞는지 확인 | 정확도 검증용 실험 |

`usb_pre_approach`는 이름에 `approach`가 있지만 실제 USB 삽입 프로그램이
아닙니다. 그리퍼/충전기 TCP, 충돌 회피, 힘 제어, PBVS/IBVS가 아직 포함되지
않았으므로 측정 좌표의 오차를 눈으로 확인하는 용도로만 사용합니다.

실제 로봇·MoveIt·Hand-eye TF·카메라 터미널 구성은
[`../pinkk_mycobot_bridge/HAND_EYE_HANDOFF_KO.md`](../pinkk_mycobot_bridge/HAND_EYE_HANDOFF_KO.md)의
USB 포트 수동 검출 절을 참고합니다.

## 1. Hand-eye 자동 캘리브레이션

현재 `g_base -> joint6_flange`를 기준 자세로 저장한 뒤, local X/Y/Z 회전을
조합한 후보 자세를 생성합니다.

```text
MoveIt IK
→ FollowJointTrajectory 이동
→ 정지 대기
→ 유효 ChArUco TF 확인
→ Easy Handeye2 TakeSample
→ 계산 및 저장
```

ChArUco TF는 코너 35개 이상, 재투영 오차 0.7 px 이하일 때만 유효합니다.
후보 30개 중 유효 샘플 목표는 15개이며 최소 12개 이상일 때만 결과를
계산합니다.

이동 없는 IK 검사:

```bash
ros2 launch pinkk_handeye_automation auto_calibrate.launch.py
```

실제 자동 수집:

```bash
ros2 launch pinkk_handeye_automation auto_calibrate.launch.py execute:=true
```

샘플 수 변경:

```bash
ros2 launch pinkk_handeye_automation auto_calibrate.launch.py \
  execute:=true target_samples:=18 minimum_samples:=15
```

## 2. USB 좌표 정확도 검증 실험

### 2.1 검증하려는 좌표 체인

```text
T_base_usb
= T_base_flange × T_flange_camera × T_camera_usb
```

- `T_base_flange`: 실제 관절각과 URDF FK
- `T_flange_camera`: Hand-eye 결과
- `T_camera_usb`: 내부 파라미터와 수동 네 점 SolvePnP

실험 목표는 이 체인으로 얻은 USB X/Y/Z/Yaw가 실제 USB 위치와 얼마나 맞는지
확인하는 것입니다.

### 2.2 현재 고정 시험값

| 항목 | 값 | 의미 |
|---|---:|---|
| 초기 관절각 | `[-1.66, -8.08, -36.65, -39.9, 0, 45]°` | USB 관측 자세 |
| transit Z | `150 mm` | 높은 Z에서 XY IK가 실패해 먼저 이동하는 중간 높이 |
| 최종 Z | `USB Z + 100 mm` | flange 기준 검증 종료 높이 |
| Roll/Pitch | `-180° / 0°` | 아래쪽을 보는 시험 자세 |
| Yaw offset | `+129.782°` | 고정 클릭 축과 그리퍼 정렬 관계 |
| XY 분할 | 없음 | 목표 XY를 한 번에 전송 |
| 회전 분할 | 실행 시 `180°` | 목표 회전을 한 번에 전송 |
| 최종 Z 분할 | `10 mm` | 마지막 접근만 상대적으로 천천히 확인 |

Yaw offset은 USB 위치를 다시 측정할 때마다 바꾸는 값이 아닙니다. 클릭 좌표축,
카메라·그리퍼 장착 또는 충전기 고정 방향이 바뀔 때만 다시 측정합니다.

### 2.3 USB 클릭 규칙

화면의 가로·세로가 아니라 USB의 물리적 변을 기준으로 합니다.

```text
1→2: USB 긴 변 11.5 mm
2→3: 인접한 짧은 변 4.5 mm
3→4→1: 같은 방향으로 나머지 둘레
```

Yaw 방향까지 반복하려면 같은 물리적 시작 모서리를 사용해야 합니다. 한쪽에
작은 표시를 붙이고 그 점을 항상 1번으로 사용하는 방법이 가장 확실합니다.

카메라 기준 SolvePnP 깊이는 실제 렌즈–USB 거리 약 260 mm와 비슷해야 합니다.
깊이가 크게 다르면 로봇을 이동하지 않습니다.

### 2.4 표준 실행 순서

초기 관측 자세 이동:

```bash
ros2 run pinkk_handeye_automation usb_pre_approach observe --execute
```

로봇 정지 후 `manual_usb_tf` 화면에서 기존 결과를 지우고 다시 선택합니다.

```text
r → f → 물리적 긴 변부터 네 점 클릭
```

TF 확인:

```bash
ros2 run tf2_ros tf2_echo camera_optical_frame usb_port
ros2 run tf2_ros tf2_echo g_base usb_port
```

로봇을 움직이지 않는 IK/경로 검사:

```bash
ros2 run pinkk_handeye_automation usb_pre_approach run \
  --transit-z-mm 150 \
  --standoff-mm 100 \
  --angle-step-deg 180 \
  --final-z-step-mm 10
```

모든 단계의 IK가 성공한 경우에만 실제 검증 이동을 실행합니다.

```bash
ros2 run pinkk_handeye_automation usb_pre_approach run \
  --transit-z-mm 150 \
  --standoff-mm 100 \
  --angle-step-deg 180 \
  --final-z-step-mm 10 \
  --execute
```

예상 단계 수:

```text
TRANSIT_Z=1
XY=1
ROT=1
Z=거리/10 mm
```

실제 MyCobot 속도는 로봇 PC의 bridge에서 정합니다.

```bash
ros2 launch pinkk_mycobot_bridge trajectory_bridge.launch.py \
  speed:=50 goal_tolerance_deg:=5.0
```

`usb_pre_approach`의 `--motion-seconds`는 현재 bridge에서 실제 속도보다 목표 도달
timeout 계산에 주로 사용됩니다.

### 2.5 현재 실험의 한계

- `joint6_flange`를 목표점으로 사용하며 그리퍼/충전기 TCP 오프셋이 없습니다.
- 충돌 검사는 기본적으로 비활성화되어 있습니다.
- XY와 transit Z는 한 관절 목표로 보내므로 완전한 Cartesian 직선을 보장하지
  않습니다.
- 마지막 Z는 작은 IK 목표들로 직선을 근사하며 연속 servo 제어가 아닙니다.
- 이동 후 기존 `camera_optical_frame -> usb_port`는 오래된 관측이므로 반드시
  다시 클릭해야 합니다.
- 실제 삽입에는 TCP 보정, 힘/접촉 감지와 PBVS/IBVS 폐루프가 추가로 필요합니다.

## 3. 결과 해석

이 실험에서 확인할 것은 “USB에 삽입했는가”가 아니라 다음 항목입니다.

- SolvePnP 깊이가 실제 거리와 일치하는가
- RViz의 USB 위치가 실제 작업대 위치와 일치하는가
- 목표 XY 이동 후 flange가 USB 중심 근처에 오는가
- 고정 Yaw offset으로 반복 측정해도 같은 방향 정렬이 나오는가
- 위치를 바꿨을 때 오차가 특정 방향으로 누적되는가

이 결과를 기록한 뒤 TCP와 PBVS/IBVS 단계로 넘어갑니다.
