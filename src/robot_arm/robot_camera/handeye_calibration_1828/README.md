# 수동 OpenCV Hand-eye 복구 도구

이 폴더의 Python 모듈은 로봇 PC에서 `pymycobot + OpenCV`로 Hand-eye를 직접
수집·계산하는 복구 및 독립 검증 경로입니다.

일반 운용에서는 이 경로를 사용하지 않고 ROS2 Easy Handeye2 자동 흐름을
사용합니다.

```text
대표 실행 문서:
scripts/calibration/README_KO.md
```

## 폴더 역할

```text
config/      수동 경로 설정
core/        좌표 변환, ChArUco, calibrateHandEye 공통 로직
runtime/     수동 샘플 수집과 계산
validation/  고정 보드 검증
applications/과거 수동 USB 검증 코드
data/        자동/수동 Hand-eye 영구 기록과 활성값
```

`applications/manual_usb_tf.py`는 과거 검증용이며 최종 USB 시스템에서는 사용하지
않습니다. 최종 자동 인식과 PBVS는 `ros2_ws/src/pinkk_usb_insertion`에 있습니다.

## 수동 복구가 필요한 경우

- Easy Handeye2 또는 ROS2를 실행할 수 없음
- 로봇 PC에서 계산을 독립적으로 재현해야 함
- 여러 OpenCV Hand-eye 알고리즘을 별도로 비교해야 함

평상시 새 캘리브레이션이나 결과 비교는
`scripts/calibration/laptop_auto_handeye.sh`와
`laptop_compare_handeye.sh`를 사용합니다.

## 수동 실행 순서

세부 옵션은 각 하위 README를 확인합니다.

1. [`config/README.md`](config/README.md): serial, 카메라, 보드 설정
2. [`runtime/README.md`](runtime/README.md): 샘플 수집과 계산
3. [`validation/README.md`](validation/README.md): 고정 보드 검증

최소 명령:

```bash
# 저장소 루트에서 실행
source ~/venv/mycobot/bin/activate

python3 -m src.robot_arm.robot_camera.handeye_calibration_1828.runtime.collect_samples \
  --camera 0

python3 -m src.robot_arm.robot_camera.handeye_calibration_1828.runtime.calibrate

python3 -m src.robot_arm.robot_camera.handeye_calibration_1828.validation.verify \
  --camera 0
```

bridge와 수동 `pymycobot`을 동시에 실행하면 안 됩니다. `/dev/ttyUSB0`와
`/dev/video0`은 각각 한 프로세스만 사용합니다.

## 좌표계

```text
p_A = T_A_B @ p_B

T_base_flange    : flange → base
T_camera_charuco : ChArUco → camera
T_flange_camera  : camera → flange

T_base_charuco =
  T_base_flange @ T_flange_camera @ T_camera_charuco
```

OpenCV `calibrateHandEye()`에는 `gripper2base=T_base_flange`,
`target2cam=T_camera_charuco`를 입력하고 반환되는 `cam2gripper`를
`T_flange_camera`로 사용합니다.

## 데이터 보관

이전 설명과 달리 Hand-eye 데이터는 이제 Git에 포함됩니다.

```text
data/runs/         실행별 샘플과 결과
data/comparisons/  old/new 자동 자세 비교
data/active/       전체 시스템에서 사용하는 결과
```

수동 계산 결과도 기존 파일을 덮어쓰지 말고 새 run 폴더로 옮긴 후 비교합니다.
활성값 변경은 다음 명령으로만 수행합니다.

```bash
bash scripts/calibration/laptop_handeye_data.sh activate RUN
```

상세 보관 규칙은 [`data/README.md`](data/README.md)를 확인합니다.
