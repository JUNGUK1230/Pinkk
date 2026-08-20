# Pinkk robot-arm USB insertion

JetCobot/MyCobot280의 카메라로 USB-A 포트를 인식하고, 초기 관측에서 계산한
PBVS 목표를 기준으로 정렬·하강·삽입을 시험하는 ROS 2 Jazzy 저장소입니다.
현재 운용 경로는 MoveIt 충돌계획이 아닌 PyMyCobot 제조사 API 기반
frozen-target 제어 하나입니다.

## 저장소 구조

```text
ros2_ws/src/
  pinkk_usb_insertion/             카메라·YOLO·SolvePnP·PBVS·통합 실행기
  pinkk_usb_insertion_interfaces/  포트 관측 메시지와 Cartesian action
  pinkk_mycobot_bridge/            로봇 PC의 PyMyCobot action bridge
  pinkk_handeye_automation/        Hand-eye 자동 수집·비교 도구
scripts/
  run_robot_bridge.sh              로봇 PC 실행 진입점
  run_laptop_frozen_target_test.sh 노트북 통합 실행 진입점
  calibration/                     카메라·Hand-eye 보정 도구
models/                             로컬 YOLO weight 위치(PT 파일은 Git 제외)
src/robot_arm/calibration/         보정 원본 코드와 장비별 결과
```

`build/`, `install/`, `log/`, Python/pytest 캐시는 생성물이므로 Git에서 제외합니다.
카메라 `results/`는 Git에서 제외되지만 장비별 보정 자산이므로 임의로 지우지
않습니다.

## Robot A 실행

아래 명령은 노트북과 로봇 PC에서 각각 저장소 루트에서 실행합니다.

YOLO weight는 Git에 포함되지 않습니다. 학습 모델을
`models/usb_02.pt`에 복사하거나 다른 위치의 모델을 환경변수로 지정합니다.

```bash
export PINKK_YOLO_MODEL_PATH=/path/to/usb_model.pt
```

자세한 내용은 [`models/README.md`](models/README.md)를 참고합니다.

로봇 PC:

```bash
./scripts/run_robot_bridge.sh robot_a
```

노트북:

```bash
./scripts/run_laptop_frozen_target_test.sh robot_a
```

현재 설정은 포트가 영상 중앙 허용범위에서 5초 동안 안정되면 최종 Z까지 통합
제어를 한 번 자동 실행합니다. 수동 실행은 다음 스크립트를 사용합니다.

```bash
./scripts/execute_frozen_target_full_sequence.sh robot_a
```

상태 확인:

```bash
ros2 topic echo /robot_arm/frozen_target/status
```

## 설정 위치

- 공통 런타임: `ros2_ws/src/pinkk_usb_insertion/config/hybrid_runtime.yaml`
- 포트 규격/SolvePnP: `ros2_ws/src/pinkk_usb_insertion/config/insertion_control.yaml`
- 로봇 A/B 보정: `ros2_ws/src/pinkk_usb_insertion/config/robots/`
- 로봇 PC bridge: `ros2_ws/src/pinkk_mycobot_bridge/config/trajectory_bridge.yaml`

현재 제어 방식은
[`FROZEN_TARGET_TEST_KO.md`](ros2_ws/src/pinkk_usb_insertion/docs/FROZEN_TARGET_TEST_KO.md),
개발 진행 이력은
[`DEVELOPMENT_LOG_KO.md`](ros2_ws/src/pinkk_usb_insertion/docs/DEVELOPMENT_LOG_KO.md),
시험 중 문제와 해결 이력은
[`TROUBLESHOOTING_KO.md`](ros2_ws/src/pinkk_usb_insertion/docs/TROUBLESHOOTING_KO.md)를
참고합니다.

## 빌드

노트북 전체 워크스페이스:

```bash
set +u
source /opt/ros/jazzy/setup.bash
colcon --log-base ros2_ws/log build --symlink-install \
  --base-paths ros2_ws/src \
  --build-base ros2_ws/build \
  --install-base ros2_ws/install
```

로봇 PC 배포·빌드는 [`scripts/calibration/README_KO.md`](scripts/calibration/README_KO.md)의
로봇 PC 절차를 따릅니다.
