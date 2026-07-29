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
| `refine_xy` | 현재 Z/자세를 유지하고 새 관측의 XY만 적용 |
| `refine_yaw` | keypoint 영상각으로 Joint6만 제한 회전 |

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

### 1. 재관측 복구

- 유효한 포트 관측마다 `last_visible_joint_pose`를 저장한다.
- 이동 후 포트가 사라지면 Cartesian 좌표를 새로 풀지 않고 저장한
  관절 자세로 복귀한다.

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
- 포트 소실 시 마지막 가시 관절 자세로 반복 복귀한다.
- 정지 영상 중심과 Yaw 오차가 정한 허용 범위에서 유지된다.
- 도달 불가능한 재관측 자세를 실행 전에 걸러낸다.
- 충전기 TCP 보정 후에만 삽입 단계로 넘어간다.
