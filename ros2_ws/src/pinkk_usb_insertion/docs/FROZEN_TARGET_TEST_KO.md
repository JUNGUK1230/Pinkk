# 초기 관측 고정목표 혼합 PBVS 시험

이 문서는 현재 사용하는 USB 포트 정렬 시험 하나만 설명한다. 별도의
`frozen_target_p` 실행기는 제거했으며 다음 스크립트만 사용한다.

```bash
./scripts/run_laptop_frozen_target_test.sh
```

## 제어 목적과 범위

초기 관측에서 YOLO keypoint, SolvePnP, Hand-eye 변환을 이용해 포트의 base
좌표를 구한다. 이 관측으로 목표 XY, 포트 Z, 초기 Roll/Pitch와 영상 Yaw를
한 번 저장한다. 큰 이동 뒤 포트가 화각에서 사라지는 문제를 피하기 위해 이후
하강 제어에는 카메라 재관측을 요구하지 않고 제조사 `get_coords()`와
`/joint_states`를 사용한다.

현재 범위는 다음과 같다.

- 초기 PBVS 목표 XY/pre-approach로 이동
- 저장한 영상각으로 Joint6 Yaw 정렬
- 초기 관측 Roll/Pitch와 XY 결합 재확인
- 포트 기반 최종 flange Z의 15mm 위까지 혼합 stop-and-go 하강
- 마지막 15mm와 실제 접촉·삽입은 자동 실행하지 않음

힘/토크 또는 접촉 센서가 없으므로 포트 내부까지 자동 삽입하는 기능으로
간주하면 안 된다.

## 좌표와 최종 Z

최종 flange 목표는 초기 관측 포트 Z와 장착할 도구 길이로 계산한다.

```text
target_flange_z
= frozen_port_z + final_tcp_offset_z_m - final_port_insertion_depth_m
```

현재 YAML 시험값은 flange에서 USB 끝까지 100mm, 삽입 깊이 10mm다.

```text
target_flange_z = frozen_port_z + 90mm
```

실제 충전기를 장착한 뒤 반드시 `final_tcp_offset_z_m`을 다시 측정해야 한다.

## 전체 제어 순서

### 1. `execute_once`: 초기 목표 고정 및 XY/Yaw 정렬

1. fresh PBVS target, port pose, observation reference와 keypoint angle을 저장한다.
2. 포트 Z보다 `absolute_pre_approach_height_m`만큼 높은 Z와 PBVS XY로
   `send_coords()` coarse 이동한다.
3. 초기 관측 Roll/Pitch를 복구하고 저장 XY 오차를 stop-and-go로 줄인다.
4. 저장한 영상각으로 Joint6 Yaw를 `send_angles()`로 보정한다.
5. Yaw 뒤 XY와 초기 Roll/Pitch를 다시 맞춘다.
6. 제조사 좌표의 정렬 완료 XY, 초기 Roll/Pitch, 포트 기반 최종 Z를 저장한다.

카메라 관측은 여기까지 사용한다.

### 2. `descend_joint_z_once`: 혼합 제어 한 사이클

한 번 승인할 때 다음 단계만 수행한다.

```text
URDF Jacobian 관절 Z 하강(send_angles)
→ 실제 get_coords 측정
→ 필요 시 Cartesian XY 보정(send_coords)
→ 실제 get_coords 측정
→ 필요 시 Cartesian 초기 Roll/Pitch 보정(send_coords)
→ 실제 get_coords 최종 측정
```

- Z 명령은 남은 절대 Z 오차에 P gain을 적용하고 최대 3mm로 제한한다.
- Jacobian 관절 하나의 계산 step은 최대 2도다.
- XY와 Roll/Pitch는 앞선 정렬에서 더 잘 동작한 Cartesian 좌표 제어를 쓴다.
- XY 5mm, Roll/Pitch 7도 안이면 다음 사이클을 허용한다.
- 모든 단계는 이전 action 종료와 정지 자세 측정 후 진행한다.

### 3. `descend_joint_z_to_guard`: 안전 여유까지 자동 반복

위 혼합 사이클을 최대 8회 반복한다. 매 사이클마다 실제 Z를 다시 읽고
포트 기반 절대 목표까지 남은 거리를 재계산한다.

- XY/Roll-Pitch 보정 중 Z 결합 이동 5mm 초과는 경고로 기록한다.
- 한 사이클 총하강 30mm 초과 또는 목표 Z 아래로 이동하면 중단한다.
- 목표 Z까지 15mm 이내이면 추가 Roll/Pitch와 다음 Z 사이클을 차단한다.
- 15mm guard 이후 마지막 삽입은 이 명령의 범위가 아니다.

3mm 명령에 실제 약 9.5mm, Cartesian 자세 복구 중 약 11.9mm의 Z 결합
이동이 관측됐기 때문에 guard를 임의로 0으로 낮추면 안 된다.

## 실행 순서

### 로봇 PC

```bash
cd ~/Pinkk-robot-arm
./scripts/run_robot_bridge.sh
```

브리지가 이미 실행 중이면 중복 실행하지 않는다. `/dev/ttyUSB0`는 한
프로세스만 열어야 한다.

### 노트북

```bash
cd ~/Desktop/Pinkk-robot-arm
./scripts/run_laptop_frozen_target_test.sh
```

다른 터미널에서 상태를 확인한다.

```bash
ros2 topic echo /robot_arm/frozen_target/status
```

초기 관측 자세에서 포트를 검출한 뒤 정렬을 한 번 승인한다.

```bash
ros2 topic pub --once \
  /robot_arm/frozen_target/command \
  std_msgs/msg/String \
  "{data: execute_once}"
```

다음 상태를 확인한다.

```text
EXECUTED: 초기 관측 고정목표 XY/Yaw 완료
```

혼합 하강 한 사이클만 시험하려면:

```bash
ros2 topic pub --once \
  /robot_arm/frozen_target/command \
  std_msgs/msg/String \
  "{data: descend_joint_z_once}"
```

15mm guard까지 자동 반복하려면:

```bash
ros2 topic pub --once \
  /robot_arm/frozen_target/command \
  std_msgs/msg/String \
  "{data: descend_joint_z_to_guard}"
```

자동 명령 실행 중에는 다른 이동 명령을 보내지 않는다.

## 정상 판정 로그

각 사이클에서 다음 값을 확인한다.

```text
command / actual / overshoot
xy / roll / pitch
rp_z_drift / total_dz / remaining_z
```

다음 메시지는 추가 사이클을 허용한다.

```text
EXECUTED: ... 다음 descend_joint_z_once 승인 대기
```

다음 메시지는 guard 도달이며 같은 Z 명령을 반복하지 않는다.

```text
EXECUTED: 자동 혼합 P제어가 포트 기반 최종 Z 안전 여유에 도달했습니다
```

`REJECTED`가 나오면 바로 재전송하지 말고 로봇 PC bridge 로그와 상태 토픽의
마지막 한 사이클 전체를 확인한다.

## 주요 YAML 파라미터

설정 파일은 `config/hybrid_runtime.yaml`이다.

| 파라미터 | 현재값 | 의미 |
|---|---:|---|
| `absolute_pre_approach_height_m` | 0.15 | 포트 위 초기 coarse 높이 |
| `final_tcp_offset_z_m` | 0.100 | flange에서 USB 끝까지 거리 |
| `final_port_insertion_depth_m` | 0.010 | 목표 삽입 깊이 |
| `joint_vertical_z_kp` | 0.40 | 남은 Z 오차 비율 |
| `joint_vertical_z_step_m` | 0.003 | 관절 Z 1회 명령 상한 |
| `joint_vertical_max_joint_step_deg` | 2.0 | 관절별 계산 증분 상한 |
| `joint_vertical_xy_tolerance_m` | 0.005 | 다음 사이클 XY 허용오차 |
| `joint_vertical_roll_pitch_tolerance_deg` | 7.0 | 다음 사이클 자세 허용오차 |
| `joint_vertical_final_z_guard_m` | 0.015 | 자동 하강 종료 안전 여유 |
| `joint_vertical_max_cycles` | 8 | 한 자동 명령의 반복 상한 |

YAML만 수정하면 노트북 코드를 다시 빌드할 필요는 없지만 실행 중인 launch를
재시작해야 한다. Python/launch/setup.py/package.xml을 수정하면 노트북 패키지를
다시 빌드한다.

상세 문제 원인과 해결 이력은 `docs/TROUBLESHOOTING_KO.md`를 참고한다.
