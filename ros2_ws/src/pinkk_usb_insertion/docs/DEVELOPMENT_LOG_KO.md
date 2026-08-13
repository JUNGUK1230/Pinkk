# USB 포트 정렬 개발일지

## 2026-07-29 현재 목표

JetCobot 카메라로 USB 포트를 검출하고, 제조사 PyMyCobot API로 포트
근처까지 이동한 뒤 영상 피드백으로 남은 XY/Yaw 오차를 줄이는 것이
현재 목표다. 충전기 장착과 실제 삽입은 아직 범위에 포함하지 않는다.

## 확인된 동작

1. YOLO Pose가 USB 포트와 keypoint 4개를 검출한다.
2. SolvePnP가 `camera_optical_frame → usb_port` 자세를 계산한다.
3. Hand-eye와 flange TF를 사용해 포트 위치를 `g_base`로 변환한다.
4. PBVS 절대 목표로 포트 XY 근처까지 큰 이동이 가능했다.
5. 실제 이동은 MoveIt이 아니라 ROS bridge를 거쳐 PyMyCobot
   `send_coords()`로 실행한다.
6. 영상 장축각에 따른 Joint6 보정 방향은 `joint6_direction=-1`로
   확인했다.

## 시험에서 발견된 문제

### Z와 재관측

- `port Z + pre-approach 높이`로 이동하면 포트가 카메라 화각에서
  사라지는 경우가 있다.
- 새 XY에서 초기 flange Z로 복귀하는 Cartesian 자세가 항상
  도달 가능한 것은 아니다.
- 따라서 고정된 초기 Z를 재관측 자세로 사용하는 방식은 중단해야 한다.

### Yaw

- SolvePnP 기반 절대 Yaw는 keypoint 노이즈, 장축의 180도 대칭,
  카메라/플랜지 축 차이 때문에 실제 오차가 컸다.
- Joint6를 돌리면 영상각뿐 아니라 포트 중심 u/v도 이동했다.
- 카메라가 Joint6 축과 동축이 아니므로 XY와 Yaw를 독립적으로
  보정할 수 없다.
- 충전기가 장착되지 않아 현재 화면 목표각 0도는 임시 기준일 뿐이다.

### 제조사 API와 하드웨어

- `send_coords()` 순수 Yaw 명령이 움직이지 않고 action timeout으로
  끝난 사례가 있었다.
- `send_angles()`의 목표 허용오차 2도는 1~3도 미세 Yaw 보정에 크다.
- 관절 백래시, 브래킷 유격, 케이블 장력과 실제 정지각을 별도로
  측정해야 한다.

## 현재 제어 구조

```text
camera
  → YOLO keypoints
  → SolvePnP
  → PBVS target
  → coarse_xy 또는 refine_xy
  → PyMyCobot send_coords

keypoint image angle
  → refine_yaw
  → Joint6 trajectory
  → PyMyCobot send_angles
```

실행기는 자동 연속 이동을 하지 않는다. 사용자가 한 번의 명령을 보낼
때 한 번만 이동하고 새 관측을 기다린다.

로봇 PC의 `observe_session.launch.py`는 통합 trajectory bridge를 먼저
시작하고 초기 관측 관절 자세로 한 번 복귀한다. 별도 joint-state bridge는
사용하지 않는다. 세션 중 재복귀만 필요하면 `return_to_observe.launch.py`를
사용한다.

## 남겨 둔 명령

| 명령 | 역할 |
|---|---|
| `coarse_xy` | 포트의 절대 XY와 pre-approach Z로 큰 이동 |
| `coarse_xy_then_yaw` | coarse XY/Z → 자동 Z 가시성 복구 → 새 관측 refine XY(제한 반복) → Joint6 Yaw를 한 명령으로 실행 |
| `refine_xy` | 현재 Z/자세를 유지하고 새 관측의 XY만 적용 |
| `refine_yaw` | keypoint 영상각으로 Joint6만 제한 회전 |
| `recover_z_once` | 현재 X/Y를 목표로 유지하고 mode 1로 초기 관측 Z 방향 상승 (수동 및 coarse 자동 가시성 복구) |
| `waypoint_pbvs_align` | 마지막 가시 자세 복귀 + 제한된 waypoint stop-and-go PBVS (아래 참고) |

XY 및 Z 복구 목표의 Roll/Pitch는 초기 관측 기준을 다시 사용하고, Yaw는
이동 직전 현재값을 유지한다. Cartesian 이동 중 Roll/Pitch가 bridge 경로
허용값을 넘으면 중단하고 정지 후에도 초기 기준과의 오차를 확인한다.
Joint6 Yaw는 새 PBVS 목표의 XY 잔여오차가 허용값 안일 때만 실행하며,
Yaw 후 포트 재관측은 요구하지 않는다.

## waypoint_pbvs_align (2026-08-03 추가)

기존 문제:
- Z-only 30mm 복구는 현재 XY에서 IK가 없어 움직이지 않는 경우가 많았다.
- 포트 방향으로 60~70mm를 한 번에 이동하면 이동 후 XY 오차와 화각
  소실이 커졌다.

새 흐름:
1. 명령 시작 시 PBVS target이 fresh하면 현재 flange를
   `last_visible_flange_pose`로 저장한다(포트가 지금 보인다는 뜻).
2. PBVS 절대 목표까지 한 번에 이동하지 않고, 목표 방향으로 XY 15mm/
   Z 10mm까지만 이동하는 waypoint를 계산해 mode 0 `send_coords`로
   한 번만 이동한다.
3. 0.8초 정지 후 재관측한다.
   - 성공: `last_visible_flange_pose`를 방금 도착한 실제 자세로 갱신하고
     다음 waypoint를 다시 계산한다.
   - 실패: 임의로 Z를 올리지 않고 `last_visible_flange_pose`(방금 실제로
     지나온 도달 가능한 자세)로 Cartesian 복귀한 뒤 재관측하고 자동
     연속 이동을 중단한다.
4. 재관측 XY 잔여오차가 5mm 이하가 되면 XY 이동을 멈추고 Joint6 Yaw를
   한 번 실행한 뒤 반드시 새 관측으로 XY/Yaw를 다시 확인한다. XY가
   5mm를 넘으면 `refine_xy`를 한 번 실행하고 끝낸다.
5. `waypoint_max_cycles`(기본 6회) 또는 누적 이동량 한도에 도달하면
   XY가 수렴하지 않아도 자동 이동을 멈춘다.

`last_visible_flange_pose`는 세션 시작 시 한 번만 잠기는 기존
`observation_reference_pose`(Roll/Pitch 기준용)와는 다른, 재검출마다
계속 갱신되는 상태다. 실제 삽입은 이 명령의 범위에 포함하지 않는다.

## 제거한 실험 경로

- MoveIt IK 단발 실행기
- MoveIt PBVS step/closed-loop 실행기
- 축 응답 전용 실행기
- Cartesian smoke test
- 수동 keypoint 입력 노드
- 사용하지 않는 삽입 상태머신과 arm motion stub
- 위 실행기에만 사용되던 중복 안전/복구 모듈과 테스트

로봇 정지, 입력 최신성, 동시 실행 차단, 최대 이동량, action timeout,
Joint6 범위 제한은 실제 장비 실행에 필요한 최소 보호로 유지한다.

## 다음 개발 순서

### 1. 재관측 복구 (완료: waypoint_pbvs_align)

- 유효한 포트 관측마다 `last_visible_flange_pose`를 저장한다.
- 이동 후 포트가 사라지면 관절 자세가 아니라 방금 실제로 지나온
  Cartesian flange 자세로 복귀한다(위 `waypoint_pbvs_align` 참고).

### 2. 도달 가능한 관측 자세 검색

- 초기 flange Z 하나를 고정하지 않는다.
- 포트 기준 카메라 거리 후보를 여러 개 만든다.
- MoveIt은 실행기가 아니라 충돌검사를 끈 IK 가능 여부 확인에만
  선택적으로 사용한다.
- 전체 XY 목표가 불가능하면 75%, 50%, 25% 순서로 이동량을 줄인다.

### 3. 안정 관측

- 로봇 정지 후 0.5~1초 기다린다.
- 유효한 5~10프레임의 u/v/장축각/SolvePnP 깊이 중앙값을 사용한다.
- 분산이 큰 관측에서는 이동하지 않는다.

### 4. 결합 IBVS

실제 하드웨어에서 다음 관계를 측정한다.

```text
[Δu, Δv, Δθ] = J × [ΔX, ΔY, ΔJoint6]
```

Joint6 회전으로 생기는 화면 중심 이동까지 포함한 로컬 영상 자코비안을
사용해 XY와 Yaw를 함께 보정한다. 관측 거리가 바뀌면 자코비안을 다시
추정하거나 SolvePnP 깊이로 스케일을 갱신한다.

### 5. 충전기 장착 이후

- flange에서 충전기 끝까지 TCP를 측정한다.
- 충전기와 포트가 정렬됐을 때의 목표 영상각을 보정한다.
- XY/Yaw가 수렴한 뒤 포트 법선 방향 pre-approach를 계산한다.
- 실제 삽입에는 저속 접근과 기계적 컴플라이언스 또는 힘 감지가
  필요하다.

## 완료 기준

- 여러 포트 위치에서 PBVS coarse 이동 후 포트를 다시 검출한다.
- 포트 소실 시 마지막 가시 flange 자세로 반복 복귀한다.
- 정지 영상 중심과 Yaw 오차가 정한 허용 범위에서 유지된다.
- 도달 불가능한 재관측 자세를 실행 전에 걸러낸다.
- 충전기 TCP 보정 후에만 삽입 단계로 넘어간다.

## 진단: 로봇 실행 오차와 PBVS 인식 오차 분리

실기에서 로봇 PC bridge가 보고하는 `xy_residual`(실제 `get_coords()` 기준
정지 오차, 현재 3mm 이내 확인)과 waypoint 이동 후 노트북이 다시 계산하는
PBVS XY 오차(약 19.1mm 관측 사례)가 크게 다르면, 원인이 로봇 실행이
아니라 인식/보정 쪽에 있을 가능성이 크다. 다음 순서로 분리해서 확인한다.

1. **로봇 실행 오차**: 로봇 PC bridge 로그의
   `Cartesian 진행 상태: ... xy_residual=...mm, orientation_residual=...`
   와 최종 `Cartesian 목표 자세 도달: xy_residual=...` 메시지를 사용한다.
   이 값이 이미 3mm 안이면 로봇 자체의 위치 추종은 문제가 아니다.
2. **PBVS 재인식 오차**: waypoint 이동 후 `_wait_for_reobservation()`으로
   받은 새 `/robot_arm/pbvs/target_flange_pose`와 실제 flange TF의 XY
   차이(`_require_yaw_xy_ready()`가 계산하는 값, 상태 토픽에 로그됨)를
   비교한다. 이 값이 bridge `xy_residual`보다 훨씬 크면 다음을 의심한다.
   - 카메라 intrinsic/distortion (`config/camera_intrinsics.yaml`)
   - SolvePnP 포트 모델 크기 (`insertion_control.yaml`의 포트 치수)
   - YOLO keypoint 편향 (RQT `debug_image`에서 keypoint가 실제 모서리에
     정확히 붙는지 확인)
   - flange-camera Hand-eye 변환 (`config/handeye.yaml`)
   - 카메라 장착부 흔들림 (이동/정지 시 카메라가 물리적으로 흔들리는지)
   - TCP offset 미적용 (현재 미보정 상태, 충전기 장착 전이므로 당연히
     남아 있는 오차)

### 고정 포트 기준 `port_pose_base` 안정성 시험 방법

포트를 움직이지 않고 로봇(카메라)만 이동시켰을 때 계산된 포트 절대
위치(`/robot_arm/pbvs/port_pose_base`)가 이상적으로는 변하지 않아야
한다. 실제로 변한다면 그 변화량이 위 원인들(주로 Hand-eye/카메라
흔들림/SolvePnP)이 만드는 오차의 상한이다.

1. 포트를 고정한 채 초기 관측 자세에서
   `ros2 topic echo /robot_arm/pbvs/port_pose_base --once`로 base XYZ를
   기록한다.
2. `waypoint_pbvs_align` 등으로 로봇을 다른 자세(다른 XY/Z)로 이동시킨다.
3. 같은 포트가 여전히 보이는 새 자세에서 다시
   `ros2 topic echo /robot_arm/pbvs/port_pose_base --once`로 기록한다.
4. 두 base XYZ 차이를 계산한다. 포트는 실제로 움직이지 않았으므로 이
   차이가 곧 카메라 이동 전후 인식 파이프라인의 절대 오차다.
   `xy_residual`(로봇 실행 오차)보다 이 값이 훨씬 크면 로봇이 아니라
   인식/보정 체인이 원인이다.
5. 여러 자세 쌍에서 반복해 편차가 특정 방향(예: Z가 커질수록 XY 오차
   증가)으로 나타나는지 확인하면 Hand-eye 회전 오차인지 SolvePnP 깊이
   오차인지 좁힐 수 있다.

## 2026-08-06: frozen-target 혼합 하강으로 주 경로 통합

현재 실기 주 경로를 하나로 정리했다.

```text
초기 YOLO/SolvePnP/PBVS 관측
→ frozen XY/pre-approach coarse
→ 초기 Roll/Pitch와 Joint6 Yaw 정렬
→ 관절 Jacobian Z
→ Cartesian XY
→ Cartesian 초기 Roll/Pitch
→ 제조사 실제 pose로 절대 포트 Z 잔여거리 재계산
```

실측 결과:

- 관절 Jacobian Z 명령 3mm에서 실제 약 9.5mm 하강
- 관절 Jacobian R/P 보정은 자세가 약 1.7도 악화되고 Z가 6.8mm 결합 이동
- Cartesian R/P 보정은 Roll 9.13→3.65도, Pitch 8.79→3.33도로 개선
- 같은 Cartesian R/P 보정에서 Z가 11.9mm 결합 이동

따라서 Jacobian은 Z에만 사용하고 XY/R/P는 Cartesian 제어로 통합했다.
상대 Z drift 경고만으로 이미 끝난 이동을 폐기하지 않고 포트 기반 절대 목표
Z까지 남은 거리를 다음 P 사이클 입력으로 사용한다. 단일 Z 실제 하강 15mm,
한 혼합 사이클 총하강 30mm를 하드 한계로 두고 목표 15mm 위에서 자동 반복을
끝낸다.

중복된 `frozen_target_p` 노드·launch·스크립트·문서는 기본 실행기에 기능이
통합되어 제거했다. 현재 사용 명령은 `execute_once`, `yaw_only_once`,
`descend_joint_z_once`, `descend_joint_z_to_guard`다. 이전 Cartesian Z 반복
메서드는 비교 이력으로 코드에 남아 있지만 ROS 명령 입력에서는 차단했다.

상세 원인과 해결 근거는 `TROUBLESHOOTING_KO.md`, 현재 실행 절차와 YAML
파라미터는 `FROZEN_TARGET_TEST_KO.md`에 분리했다.

## 2026-08-10: 다른 포트 각도 XY 시험과 경로 제약 완화

### 시험 목적

기존에 시험한 포트 자세와 다른 각도에서도 초기 SolvePnP/PBVS 절대 XY로
이동하고, frozen-target의 flange 좌표 오차 보정이 같은 수준으로 수렴하는지
확인했다.

### 관측 결과

- 큰 coarse 이동은 초기 XY 오차를 약 73mm에서 7~9mm까지 줄였다. 따라서
  PBVS 목표 방향과 base XY 변환 자체는 대체로 맞았다.
- mode 1 coarse 잔여오차 약 8.7mm, mode 0 비교 시험 약 6.9mm가 관측됐다.
  mode 0이 조금 나았지만 최종 5mm 기준을 안정적으로 만족하지 못해 mode만의
  문제로 판단하지 않았다.
- Roll/Pitch 복구 뒤 여러 각도에서 XY가 약 4.5~5mm 수준에 반복 정지했다.
  하드웨어 반복오차 근처의 추가 refine이 XY보다 Z 여유를 더 소모했다.
- 잔여오차 18.9mm에서 P gain 0.7이 적용된 약 13.2mm mode 1 refine 목표는
  전혀 움직이지 않고 `error=32: 역기구학 해 없음`으로 종료됐다.

### 적용한 변경

- frozen-target 로봇좌표 XY 통과 기준을 4mm에서 5mm로 완화했다.
- 모든 이동 전 경고 대기를 3초에서 1.5초로 줄였다.
- mode 0을 비교한 뒤 최종 시험 설정은 mode 1로 되돌렸다.
- coarse와 XY refine의 경로 감시 설정을 `lock_z=false`,
  `lock_roll_pitch=false`로 변경했다.
- XY refine은 현재 Z와 현재 자세를 복사하고 X/Y만 바꾸도록 분리했다.
  이동 정지 후 기존 Roll/Pitch 복구를 실행하고 실제 XY를 다시 측정한다.

현재 초기 정렬 순서는 다음과 같다.

```text
PBVS coarse (mode 1, path lock 없음)
→ 정지 후 초기 Roll/Pitch 확인/복구
→ frozen XY refine (현재 Z/RPY 유지 목표, path lock 없음)
→ 정지 후 Roll/Pitch 확인/복구
→ 실제 XY 재측정
→ Joint6 Yaw
→ 최종 XY/Roll/Pitch 결합 확인
```

### 검증

- `pinkk_usb_insertion` 패키지 재빌드 완료
- configuration/frozen-target 단위 테스트 12개 통과
- 설치 YAML에서 `warning_delay_seconds=1.5`,
  `cartesian_mode=1`, `lock_z=false`, `lock_roll_pitch=false` 확인

### 보류 및 미적용

- TCP XYZ를 삽입 pose로 역산하는 절차는 검토했지만 이번 작업에서는 측정하거나
  제어에 적용하지 않았다. 현재 코드는 기존 scalar TCP Z 시험값 120mm만 쓴다.
- 최종 Z guard 15mm는 제거하지 않았다. 실측에서 3~5mm Z 명령이 실제
  9~19.5mm 이동했으므로 포트 내부 10mm 자동 삽입은 현재 범위 밖이다.
- 다음 XY 개선 후보인 제조사 실제 `cartesian_pose_actual` 기반 refine 시작
  자세와 8mm 이하 waypoint 분할은 비교 후 되돌려 현재 적용하지 않았다.

## 2026-08-11: guard 이후 최종 Z-only 10mm 단계

- 기존 `execute_full_sequence`는 15mm guard에서 끝나는 안전 시험으로 유지했다.
- `insert_final_z_once`는 guard 도달 자세에서 XY/Roll/Pitch 보정 없이 Z만
  상대 10mm 하강한다.
- `execute_full_sequence_with_final_z`는 초기 정렬, guard 접근, 마지막 Z-only
  하강과 최종 오차 보고를 한 명령으로 실행한다.
- 마지막 단계는 Z −10mm task error로 관절 목표를 한 번만 전송하고 실제
  자세를 한 번 측정한다. XY/Roll/Pitch 추가 보정은 하지 않으며 실제 하강
  15mm를 하드 한계로 사용한다.
- 단위 테스트 15개와 노트북 패키지 빌드를 통과했다.

## 2026-08-13: 초기 5장 평균과 Z-R/P-Z 마무리 시험

- frozen-target 실행기의 최근 PBVS target, port Z, keypoint Yaw를 각각 5장
  모아 초기 고정 목표를 계산한다.
- 기본 집계 방식을 median으로 변경했다. XY/Z는 성분별 중앙값, Yaw는
  ±180도 경계를 풀어낸 원형 중앙값을 사용한다. YAML에서 mean 비교도 가능하다.
- 평균에서 벗어난 최대 편차가 XY 8mm, Z 15mm, Yaw 8도를 넘으면 불안정한
  관측으로 보고 이동 전에 거부한다.
- 반복 이동 전 경고 대기는 1.5초에서 0.5초, 이동 후 측정 안정화 대기는
  0.8초에서 0.5초로 줄였다. 브리지의 실제 정지 확인은 유지한다.
- 최종 순서를 `Z-only 삽입 → 초기 Roll/Pitch 복구 → Z-only 3mm 추가 하강`
  으로 확장했다. 추가 3mm는 한 번의 관절 Jacobian 명령으로 실행한다.
- 비교 시험을 위해 `vertical_z_control_backend=cartesian`을 추가했다. 이
  설정에서는 통합 실행의 guard 하강, 최종 10mm 및 R/P 뒤 3mm를 모두
  제조사 `send_coords(mode=1)`로 실행하고 최종 실제 오차를 보고한다.
- 다음 비교 시험은 같은 통합 흐름에서 backend만 `joint`로 되돌려 Jacobian
  관절 증분과 제조사 `send_angles` 결과를 비교한다.
- 이 비교에서는 coarse/XY/Roll-Pitch용 `cartesian_mode`도 0으로 둔다.
  따라서 Z는 send_angles, 나머지 Cartesian 보정은 send_coords mode 0이다.
- 고정 자세 비교에서는 `cartesian_mode=1`, Z backend는 `joint`로 복귀하고,
  초기 실측 R/P 대신 전체 제어 기준을 Roll -180도, Pitch 0도로 고정한다.
- Pitch 보정 미달을 비교하기 위해 Roll gain은 유지하고 Pitch 오차에만 1.20
  배율을 적용한다. 최초 coarse 목표는 정확한 고정 R/P이며, 반복 R/P 복구와
  Z 사이클 및 삽입 후 복구 명령에만 이 배율을 적용한다.
