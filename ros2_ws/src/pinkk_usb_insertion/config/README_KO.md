# 설정 파일 관리

## `camera_intrinsics.yaml`

카메라 내부 행렬 K, 왜곡계수 D, 보정 해상도를 저장합니다. 카메라, 렌즈, 초점,
해상도 또는 crop 방식이 바뀌면 다시 보정해야 합니다.

현재 값은 640×480, RMS 약 0.345 px 결과입니다. 다른 해상도의 영상에 이 행렬을
그대로 사용하지 않습니다.

현재 YAML 값은 다음 원본 NPZ의 `camera_matrix`, `dist_coeffs`, 영상 크기와 RMS를
전체 정밀도로 옮긴 값입니다.

```text
src/robot_arm/robot_camera/camera_calibration/results/intrinsics.npz
```

## `handeye.yaml`

`T_flange_camera`를 저장합니다.

```text
parent: joint6_flange
child: camera_optical_frame
```

현재 YAML의 `matrix_4x4`와 translation/quaternion은 다음 활성 NPY에서 가져온
동일한 변환입니다.

```text
src/robot_arm/robot_camera/handeye_calibration_1828/data/active/T_flange_camera.npy
```

카메라 브래킷 위치나 각도가 바뀌면 다시 측정합니다. 충전기 TCP가 바뀌는 것은
Hand-eye 재보정 조건이 아닙니다.

검증된 run을 선택할 때 YAML을 직접 수정하지 않습니다.

```bash
bash scripts/calibration/laptop_handeye_data.sh activate RUN
```

이 명령이 활성 `.calib`, NPY와 이 YAML을 같은 값으로 동기화합니다.

## 충전기 TCP 설정

사용되지 않던 `tool_transform.yaml`과 과거 실행 gate는 제거했습니다. 현재 제어는
`hybrid_runtime.yaml`의 공통 TCP Z/삽입 깊이와 로봇별
`config/robots/<profile>/runtime.yaml`의 X/Y 편심을 직접 사용합니다.

- `final_tcp_offset_x_m`, `final_tcp_offset_y_m`: flange 로컬 측면 편심
- `final_tcp_offset_z_m`: flange에서 충전기 끝까지 길이
- `final_port_insertion_depth_m`: 포트 평면 안쪽 목표 깊이
- `final_port_target_offset_x_m`, `final_port_target_offset_y_m`: 포트 로컬 목표 편향

## `insertion_control.yaml`

포트 실제 크기, SolvePnP 품질 기준, PBVS 기준과 검출 유효시간을 관리합니다.

`pose_estimation`에는 YOLO 검출 선택 기준도 포함합니다.
현재 SolvePnP 포트 모델은 YOLO keypoint 라벨링 순서
`LEFT_TOP → RIGHT_TOP → RIGHT_BOTTOM → LEFT_BOTTOM`을 사용합니다. 실제 크기는
항상 `insertion_control.yaml`의 `width_m`, `height_m` 값을 기준으로 확인합니다.

```yaml
target_class_name: usb_port
target_detection_id: ''
minimum_object_confidence: 0.70
minimum_keypoint_confidence: 0.60
```

`target_detection_id`가 비어 있으면 품질 점수가 가장 높은 포트 후보를 선택합니다.
특정 포트를 추적할 때는 상위 상태 머신이 선택한 ID를 지정하도록 확장합니다.

`pbvs.maximum_xy_step_m`은 PBVS 목표 계산에서 한 번에 제한할 XY 이동량입니다.
실제 로봇 실행 허용, 자동 시작, frozen-target XY/RP 및 Z 하강값은
`hybrid_runtime.yaml`에서 관리합니다.

## 단위 규칙

| 값 | 단위 |
|---|---|
| translation, 거리, 속도 | m, m/s |
| 회전 내부 계산 | rad 또는 quaternion |
| 이미지 특징점과 오차 | pixel |
| 시간 | second |
