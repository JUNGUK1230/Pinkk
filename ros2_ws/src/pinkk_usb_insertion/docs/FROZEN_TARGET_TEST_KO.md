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
- guard 도달 후 XY/Roll/Pitch를 보정하지 않고 Z만 상대 10mm 추가 하강
- 마지막 Z 이동 후 제조사 actual XYZ/Yaw에서 초기 관측 Roll/Pitch 전체값을
  한 번 적용하며, 이 단계의 추가 Z 결합 이동은 제한하지 않음

마지막 Z-only 단계에도 힘/토크 또는 접촉 감지가 없다. 실기에서 명령보다
큰 Z 이동이 관측됐으므로 비상 정지 수단을 준비하고 낮은 위치에 장애물이
없는 조건에서 시험한다.

## 좌표와 최종 Z

최종 flange 목표는 초기 관측 포트 Z와 장착할 도구 길이로 계산한다.

```text
target_flange_z
= frozen_port_z + final_tcp_offset_z_m - final_port_insertion_depth_m
```

현재 YAML 시험값은 완전히 닫힌 그리퍼에서 flange부터 USB 끝까지 130mm,
삽입 깊이 10mm다. USB-A와 flange의 X/Y 방향은 동일하고 X/Y 편심은
0으로 가정한다.

```text
target_flange_z = frozen_port_z + 110mm
```

충전기를 다시 장착하거나 고정 위치가 변하면 `final_tcp_offset_z_m`을
다시 측정해야 한다.

## 전체 제어 순서

### 1. `execute_once`: 초기 목표 고정 및 XY/Yaw 정렬

1. fresh PBVS target, port pose, observation reference와 keypoint angle을 저장한다.
2. 포트 Z보다 `absolute_pre_approach_height_m`만큼 높은 Z와 PBVS XY로
   `send_coords()` coarse 이동한다.
3. Z/Roll/Pitch 경로 lock 없이 이동한 뒤 초기 관측 Roll/Pitch를 복구한다.
   XY refine은 저장한 frozen 절대 X/Y와 제조사 actual Z/RPY를 결합해,
   큰 coarse 뒤 실제로 내려간 Z에서 높은 TF Z를 다시 요구하지 않는다.
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

- Z 명령은 남은 절대 Z 오차에 P gain을 적용하고 최대 6mm로 제한한다.
- Jacobian 관절 하나의 계산 step은 최대 2도다.
- XY와 Roll/Pitch는 앞선 정렬에서 더 잘 동작한 Cartesian 좌표 제어를 쓴다.
- XY 5mm, Roll/Pitch 5도 안이면 다음 사이클을 허용한다.
- 모든 단계는 이전 action 종료와 정지 자세 측정 후 진행한다.

### 3. `descend_joint_z_to_guard`: 안전 여유까지 자동 반복

위 혼합 사이클을 최대 8회 반복한다. 매 사이클마다 실제 Z를 다시 읽고
포트 기반 절대 목표까지 남은 거리를 재계산한다.

- XY/Roll-Pitch 보정 중 Z 결합 이동 5mm 초과는 경고로 기록한다.
- 한 사이클 총하강 30mm 초과 또는 목표 Z 아래로 이동하면 중단한다.
- 목표 Z까지 15mm 이내에서도 Roll/Pitch가 5도를 넘으면 Cartesian
  자세 보정은 수행하고 다음 Z 사이클은 차단한다.
- 15mm guard 이후에는 `insert_step_once`를 한 번씩 승인해 0.5mm씩
  삽입한다. 힘/접촉 센서가 없으므로 자동 반복하지 않는다.
- 기존 통합 시험 흐름에서는 guard 이후 `insert_final_z_once`로 설정된
  상대 Z를 한 번에 실행할 수도 있지만, 접촉 감지가 없으므로 실기에서는
  `insert_step_once` 방식이 더 안전하다.

3mm 명령에 실제 약 9.5mm, Cartesian 자세 복구 중 약 11.9mm의 Z 결합
이동이 관측됐기 때문에 guard를 임의로 0으로 낮추면 안 된다.

### 4. `insert_final_z_once`: Z-only 10mm 추가 하강

guard에 도달한 실제 자세를 시작점으로 Z만 상대 10mm 내린다. 관절
Jacobian에 Z 오차만 입력하며 이 단계에서는 XY와 Roll/Pitch 명령을 만들지
않는다. 각 이동 뒤 XY/Roll/Pitch는 상태 로그에만 기록한다.

- Z −10mm task error를 계산해 관절 목표를 한 번만 전송한다.
- 이동 후 실제 Z와 XY/Roll/Pitch를 한 번 측정하며 추가 보정하지 않는다.
- 한 번의 실제 하강 15mm를 하드 한계로 둔다.
- 포트 기반 최종 flange Z보다 아래를 목표로 만들지는 않는다.

Z-only 명령 뒤에는 초기 관측 Roll/Pitch를 한 번 적용한다. 이 후속 자세
복구는 X/Y/Z/Yaw 목표를 보정 직전 제조사 actual 값으로 만들지만, 실기의
축 결합으로 발생하는 추가 Z 하강은 제한하거나 실패 조건으로 사용하지 않는다.

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

guard 도달 후 XY/Roll/Pitch 보정 없이 Z만 10mm 내리려면:

```bash
ros2 topic pub --once \
  /robot_arm/frozen_target/command \
  std_msgs/msg/String \
  "{data: insert_final_z_once}"
```

자동 명령 실행 중에는 다른 이동 명령을 보내지 않는다.

### 전체 과정을 한 명령으로 실행

초기 포트 관측이 안정적이고 로봇이 초기 관측 자세에서 정지했으면 다음
스크립트 하나로 초기 정렬부터 Z 안전 여유와 마지막 Z-only 10mm 하강까지
실행할 수 있다.

```bash
ROS_DOMAIN_ID=36 ./scripts/execute_frozen_target_full_sequence.sh
```

내부 순서는 다음과 같다.

```text
execute_once 전체 정렬
→ descend_joint_z_to_guard 자동 반복
→ insert_final_z_once (XY/Roll/Pitch 보정 없음)
→ 초기 관측 Roll/Pitch 복구
→ Z-only 5mm 추가 하강
→ 제조사 실제 좌표 최종 측정
→ X/Y/XY/Z/Roll/Pitch/Yaw 오차 보고
```

완료 시 `FINAL_ERROR_REPORT`와
`EXECUTED: execute_full_sequence_with_final_z 완료`가 순서대로 발행된다.
중간 단계가 실패하면 다음 단계로 진행하지 않는다. 안전 여유까지만 실행하려면
`execute_full_sequence`를 직접 발행한다.

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

TCP 측정값과 정렬을 확인한 뒤 YAML의 `enable_final_insertion`을
`true`로 바꾸고 launch를 재시작한다. 단발 삽입 명령은 다음과 같다.

```bash
ros2 topic pub --once /robot_arm/frozen_target/command \
  std_msgs/msg/String "{data: insert_step_once}"
```

각 명령 후 실제 USB 접촉·변형과 status의 `actual`, `remaining`, `xy`,
`roll`, `pitch`를 직접 확인한 뒤만 다음 0.5mm를 승인한다.

`REJECTED`가 나오면 바로 재전송하지 말고 로봇 PC bridge 로그와 상태 토픽의
마지막 한 사이클 전체를 확인한다.

## 주요 YAML 파라미터

설정 파일은 `config/hybrid_runtime.yaml`이다.

| 파라미터 | 현재값 | 의미 |
|---|---:|---|
| `initial_observation_sample_count` | 5 | 초기 목표 집계에 사용할 최근 관측 수 |
| `initial_observation_aggregation_method` | median | XY/Z/Yaw 이상값에 강한 중앙값 집계 |
| `vertical_z_control_backend` | joint | 통합 Z 단계에 Jacobian/send_angles 사용 |
| `vertical_z_cartesian_mode` | 1 | 통합 Z 단계의 send_coords mode |
| `absolute_pre_approach_height_m` | 0.18 | 자세 결합 하강을 고려한 포트 위 초기 coarse 높이 |
| `robot_xy_tracking_tolerance_m` | 0.005 | 초기 XY/Roll/Pitch 결합 정렬의 로봇좌표 XY 허용오차 |
| `cartesian_mode` | 1 | XY/Roll-Pitch용 제조사 직선 Cartesian 모드 |
| `use_fixed_roll_pitch_target` | true | 초기 실측 대신 고정 R/P 기준 사용 |
| `fixed_roll_target_deg` | -180.0 | 전체 제어 Roll 목표 |
| `fixed_pitch_target_deg` | 0.0 | 전체 제어 Pitch 목표 |
| `pitch_correction_gain` | 1.70 | Pitch 오차에만 적용하는 보정 배율 |
| `final_tcp_offset_z_m` | 0.130 | 닫힌 그리퍼의 flange에서 USB 끝까지 거리 |
| `final_port_insertion_depth_m` | 0.010 | 목표 삽입 깊이 |
| `enable_final_insertion` | false | 최종 단발 삽입 안전 스위치 |
| `final_insertion_step_m` | 0.0005 | 승인 1회당 삽입 명령량 |
| `joint_vertical_z_kp` | 0.40 | 남은 Z 오차 비율 |
| `joint_vertical_z_step_m` | 0.006 | 관절 Z 1회 명령 상한 |
| `joint_vertical_max_joint_step_deg` | 2.0 | 관절별 계산 증분 상한 |
| `joint_vertical_xy_tolerance_m` | 0.005 | 다음 사이클 XY 허용오차 |
| `joint_vertical_roll_pitch_tolerance_deg` | 5.0 | 다음 사이클 자세 허용오차 |
| `joint_vertical_final_z_guard_m` | 0.015 | 자동 하강 종료 안전 여유 |
| `joint_vertical_max_cycles` | 8 | 한 자동 명령의 반복 상한 |
| `final_insertion_relative_distance_m` | 0.010 | guard 이후 Z-only 상대 하강 거리 |
| `final_insertion_hard_maximum_total_descent_m` | 0.015 | 마지막 단발 실제 하강 하드 한계 |
| `enable_post_recovery_final_z` | true | 삽입 후 R/P 복구 뒤 추가 Z 하강 사용 |
| `post_recovery_final_z_distance_m` | 0.005 | R/P 복구 뒤 Z-only 추가 하강 거리 |

이 작업공간에서는 YAML data file이 install에 복사될 수 있으므로 YAML만
수정해도 노트북 패키지를 재빌드하고 launch를 재시작한다. Python/launch/
setup.py/package.xml 수정도 동일하게 재빌드한다.

상세 문제 원인과 해결 이력은 `docs/TROUBLESHOOTING_KO.md`를 참고한다.
