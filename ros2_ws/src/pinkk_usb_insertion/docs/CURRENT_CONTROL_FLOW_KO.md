# 현재 USB 포트 제어 방식

## 1. 사용 중인 방식

현재 방식은 IBVS가 아니라 **초기 관측 frozen-target PBVS + 로봇좌표
stop-and-go 폐루프 + 관절/Cartesian 혼합 하강**입니다.

```text
카메라 영상
→ YOLO Pose: 포트 외곽 keypoint 4개
→ SolvePnP: camera_optical_frame 기준 포트 3D pose
→ Hand-eye와 flange TF: g_base 기준 usb_port pose
→ 초기 유효 관측 여러 장의 중앙값
→ g_base 기준 목표 XY/Z/Yaw 고정
→ 이후 카메라 재관측 대신 실제 flange 좌표로 오차 보정
```

IBVS의 픽셀 interaction matrix, 영상 Jacobian 또는 고주기 pixel-error
servo는 사용하지 않습니다. YOLO 픽셀은 포트 pose와 초기 Yaw를 얻는 관측에만
사용합니다.

## 2. 자동 시작

`UsbPortObservation`의 네 keypoint 중심, SolvePnP depth, keypoint 장축각을
5초 동안 검사합니다. 다음을 모두 만족하면 차량과 포트가 정지했다고 판단합니다.

- 유효 관측 30개 이상
- 포트 중심이 영상 `(320, 240)`에서 80px 이내
- 포트 중심 최대 흔들림 5px 이하
- SolvePnP depth 최대 흔들림 5mm 이하
- keypoint 장축각 최대 흔들림 2도 이하
- 관측 사이 공백 0.5초 이하
- 로봇이 초기 관측 자세로 확인되어 PBVS 기준 pose가 발행된 상태

조건 충족 시 `execute_full_sequence_with_final_z`와 같은 통합 흐름을 한 번
자동 실행합니다. 같은 프로세스에서는 다시 트리거하지 않으므로 새 차량 시험은
노트북 런치를 재시작해 다시 arm합니다.

## 3. 초기 목표 생성

초기 관측 5장을 최근 1초 시간창에서 모아 중앙값을 사용합니다.

- 포트 모델: 장축 18mm × 단축 12mm
- 카메라 pose: `camera_optical_frame → usb_port`
- 로봇 pose: `g_base → joint6_flange → camera_optical_frame → usb_port`
- coarse Z: `port_z + 180mm` pre-approach
- 최종 flange Z: `port_z + TCP_Z - insertion_depth`
- 현재 TCP Z: 130mm
- 현재 포트 삽입 깊이: 10mm

따라서 현재 최종 목표는 다음과 같습니다.

```text
target_flange_z = frozen_port_z + 130mm - 10mm
```

## 4. XY와 자세 정렬

1. frozen 목표 XY/pre-approach Z로 coarse `send_coords`를 보냅니다.
2. 제조사 `get_coords()` 실제 pose와 frozen XY의 오차를 계산합니다.
3. XY 허용오차 5mm 밖이면 최대 3회 stop-and-go 보정합니다.
4. 초기 관측 Roll/Pitch 목표와 현재 자세 차이가 5도보다 크면 Cartesian
   Roll/Pitch 복구 후 XY를 다시 계산합니다.
5. 저장한 초기 keypoint 장축각을 gain과 방향 부호로 Joint6 Yaw 명령으로
   변환합니다.
6. Yaw 뒤 XY와 Roll/Pitch를 다시 확인합니다.

이 구간은 MoveIt 경로계획이나 충돌검사를 사용하지 않습니다. 노트북 실행기가
ROS action 목표를 보내고 로봇 PC bridge가 PyMyCobot `send_coords()` 또는
`send_angles()`를 호출합니다.

## 5. Z 하강

현재 `vertical_z_control_backend: joint`를 사용합니다.

```text
현재 관절·flange pose 측정
→ URDF Jacobian으로 Z 방향 관절 증분 계산
→ send_angles
→ 실제 pose 재측정
→ 필요하면 Cartesian XY 보정
→ 필요하면 Cartesian Roll/Pitch 보정
→ 실제 최종 Z까지 남은 거리 재계산
```

- Z P gain: 0.40
- 한 사이클 최대 Z 요청: 6mm
- 관절 하나의 계산상 최대 변화: 2도
- 최종 목표 위 guard: 15mm
- 최대 혼합 사이클: 8회

기구 결합 때문에 6mm 명령이 실제로 정확히 6mm가 되지는 않습니다. 매 사이클
후 실제 좌표를 다시 읽고 절대 포트 목표 Z를 기준으로 남은 거리를 계산합니다.

## 6. 마지막 삽입

```text
guard 도달
→ 최종 Z-only 최대 10mm
→ 초기 Roll/Pitch 전체값 한 번 복구
→ Z-only 추가 5mm
→ FINAL_ERROR_REPORT
```

혼합 보정 중 이미 최종 목표 Z 아래로 내려갔다면 10mm 단계는 생략하고
Roll/Pitch 복구와 추가 5mm로 진행합니다. 현재 시험 설정은 이 과삽입을
경고로 기록하고 통합 실행을 계속합니다.

## 7. 오차의 의미

- `error_xy`: frozen 로봇좌표 목표와 실제 flange XY 차이
- `xy_norm`: 위 X/Y 오차의 크기
- `z_remaining = actual_flange_z - configured_target_flange_z`
  - 양수: 목표보다 위
  - 음수: 목표보다 아래, 즉 계산상 과삽입
- `roll`, `pitch`: 고정 목표 Roll/Pitch와 실제 자세 차이

물리적으로 삽입이 부족한데 `z_remaining`이 음수라면 추가 제어보다 TCP 길이,
SolvePnP 포트 크기/깊이, Hand-eye Z를 먼저 확인해야 합니다.

## 8. 남겨둔 실행 파일

- `frozen_target_alignment.launch.py`: 노트북 인식·목표 계산·통합 실행
- `observe_session.launch.py`: 로봇 bridge와 초기 관측 자세 복귀
- `return_to_observe.launch.py`: 실행 중 초기 관측 자세만 다시 요청
- `run_laptop_frozen_target_test.sh`: 로봇 프로필과 환경을 선택해 노트북 실행
- `run_robot_bridge.sh`: 로봇 프로필과 PyMyCobot 환경을 선택해 bridge 실행
- `execute_frozen_target_full_sequence.sh`: 자동 시작을 끈 경우 수동 통합 명령
