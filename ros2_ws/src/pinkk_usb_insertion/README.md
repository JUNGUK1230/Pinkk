# Pinkk USB Port Frozen-Target Control

YOLO keypoint, SolvePnP, Hand-eye로 USB 포트의 로봇 베이스 좌표를 구한 뒤,
초기 관측 목표를 고정하고 PyMyCobot bridge를 통해 정렬·하강하는 ROS 2
패키지입니다. 현재 실행 경로는 frozen-target 하나만 유지합니다.

## 실행

아래 명령은 각 장비의 저장소 루트에서 실행합니다.

로봇 A PC:

```bash
./scripts/run_robot_bridge.sh robot_a
```

노트북:

```bash
./scripts/run_laptop_frozen_target_test.sh robot_a
```

현재 YAML은 포트가 영상 중앙에서 5초 동안 안정되면 최종 Z를 포함한 통합
제어를 프로세스당 한 번 자동 실행합니다. 자동 시작을 끄려면
`config/hybrid_runtime.yaml`에서 다음을 수정합니다.

```yaml
auto_start_enabled: false
```

수동 실행은 다음 스크립트를 사용합니다.

```bash
./scripts/execute_frozen_target_full_sequence.sh robot_a
```

상태:

```bash
ros2 topic echo /robot_arm/frozen_target/status
```

초기 자세 복귀:

```bash
ros2 launch pinkk_usb_insertion return_to_observe.launch.py \
  robot_profile:=robot_a
```

## 설정

- 공통 런타임: `config/hybrid_runtime.yaml`
- 포트 모델·SolvePnP·PBVS: `config/insertion_control.yaml`
- 로봇별 카메라/Hand-eye/override: `config/robots/robot_a|robot_b`
- 로봇 PC bridge: `pinkk_mycobot_bridge/config/trajectory_bridge.yaml`

현재 실행 방식과 파라미터는
[`docs/FROZEN_TARGET_TEST_KO.md`](docs/FROZEN_TARGET_TEST_KO.md), 개발 진행은
[`docs/DEVELOPMENT_LOG_KO.md`](docs/DEVELOPMENT_LOG_KO.md), 문제 해결 이력은
[`docs/TROUBLESHOOTING_KO.md`](docs/TROUBLESHOOTING_KO.md)를 참고합니다.
