# Pinkk MyCobot ROS 2 브리지

PyMyCobot 시리얼 연결 하나로 실제 관절 상태와 Cartesian pose를 읽고,
ROS 2 action을 제조사 `send_angles()`·`send_coords()` 호출로 변환합니다.

## 표준 실행

로봇 PC의 저장소 루트에서 실행합니다.

```bash
./scripts/run_robot_bridge.sh robot_a  # ROS domain 36
./scripts/run_robot_bridge.sh robot_b  # ROS domain 38
```

스크립트는 자신의 사용자 홈을 기준으로 다음 환경을 찾습니다.

```text
${HOME}/venv/mycobot
${HOME}/mycobot_moveit_ws/install_pinkk/setup.bash
```

다른 위치를 사용하면 실행 전에 지정합니다.

```bash
export PINKK_ROBOT_VENV=/path/to/mycobot-venv
export PINKK_ROBOT_INSTALL_SETUP=/path/to/install_pinkk/setup.bash
./scripts/run_robot_bridge.sh robot_a
```

## 제공 인터페이스

```text
/joint_states
/robot_arm/cartesian_pose_actual
/arm_group_controller/follow_joint_trajectory
/robot_arm/cartesian_move
```

표준 `trajectory_bridge`가 상태 발행과 두 action을 모두 담당합니다. 제거된
단독 joint-state 실행기를 별도로 켤 필요가 없습니다.

## 설정

실제 로봇 PC 파라미터는 `config/trajectory_bridge.yaml`에서 관리합니다.

- 시리얼 port/baud와 상태 발행률
- 관절·Cartesian 실행 허용
- 위치·자세 완료 허용오차
- Cartesian 최대 이동량·timeout·무동작 판정
- 명령 큐 초기화 재사용
- 시작 시 그리퍼 닫기

소스 YAML 또는 Python을 바꾼 뒤에는 로봇 PC에서 다시 빌드하고 브리지를
재시작해야 합니다.

```bash
# 저장소 루트에서 실행
bash scripts/calibration/robot_build_pinkk.sh
./scripts/run_robot_bridge.sh robot_a
```

## 시리얼 단독 점유

`/dev/ttyUSB0`는 브리지 하나만 열어야 합니다. 별도 PyMyCobot 스크립트,
Jupyter 제어 셀 또는 이전 bridge가 남아 있으면 통신이 불안정해집니다.

```bash
fuser -v /dev/ttyUSB0
```

브리지를 시작하기 전에 기존 프로세스를 정상 종료하고, 프로세스가 남았다면
정확한 PID를 확인해 종료합니다.

## 판정 방식

- `send_angles()` 반환값 하나가 아니라 실제 관절각, 정지 표본과 자세 유지를
  확인합니다.
- `send_coords()`의 `-1`/`None`은 일부 펌웨어의 무응답일 수 있으므로 실제
  `get_coords()`, `is_moving()`과 오류 코드를 함께 사용합니다.
- action 하나가 실행 중일 때 다른 관절/Cartesian 목표는 거부합니다.
- bridge 위치·자세 허용오차는 action 종료 기준이며, 상위 frozen-target
  실행기가 최종 XY/Roll/Pitch/Z를 다시 측정합니다.

Hand-eye 캘리브레이션 절차는 `HAND_EYE_HANDOFF_KO.md`, 실기 오류 사례는
`pinkk_usb_insertion/docs/TROUBLESHOOTING_KO.md`를 참고합니다.
