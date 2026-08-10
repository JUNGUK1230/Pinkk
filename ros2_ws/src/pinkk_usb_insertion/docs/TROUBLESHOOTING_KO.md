# USB 포트 PBVS·JetCobot 제어 트러블슈팅

이 문서는 실기 시험에서 실제로 발생한 문제, 원인 판별 근거, 적용한 해결책과
재확인 방법을 기록한다. 현재 권장 실행 흐름은 `FROZEN_TARGET_TEST_KO.md`에
있다.

## 제어 방식 변경 이력과 의사결정

이 절은 현재 구조가 만들어진 과정을 시간 순서로 정리한다. 단순한 오류 목록이
아니라 각 시험에서 무엇을 확인했고 어떤 경로를 폐기하거나 유지했는지 설명한다.

### 단계 1. MoveIt 기반 PBVS 단발 이동

초기 구조는 YOLO keypoint → SolvePnP → PBVS 목표 → MoveIt IK/충돌검사 →
Cartesian 이동이었다. 실제 시험에서는 충돌검사가 필요하지 않았고, MoveIt
경로와 실제 JetCobot 펌웨어의 움직임 차이 때문에 원인 분리가 어려웠다.

결정:

- 포트 인식과 PBVS 목표 계산은 유지한다.
- 실제 실행은 제조사와 호환성이 높은 PyMyCobot API bridge로 분리한다.
- MoveIt IK/충돌 사전검사는 현재 실기 실행 경로에서 제외한다.

### 단계 2. 제조사 `send_coords()` 단발 XY

PBVS가 계산한 목표에서 최대 이동량을 제한하고 `execute_once` 한 번마다
`send_coords()`를 호출했다. 1mm step에서는 기기 데드밴드와 정지오차 때문에
영상 오차 변화가 불분명했지만 3~6mm에서는 대체로 목표 방향으로 XY 오차가
감소했다. 이 시험으로 다음을 확인했다.

- PBVS가 계산한 base XY 방향은 대체로 맞았다.
- 제조사 API로 목표 근처까지 coarse 이동하는 것은 가능했다.
- 1mm 이하 반복 보정은 통신 지연과 기계 반복오차에 비해 너무 작았다.

결정:

- coarse XY는 제조사 Cartesian 제어를 유지한다.
- 허용오차와 유효 step은 수 mm 수준으로 설정한다.
- 고주기 PID 대신 정지 후 측정하는 stop-and-go 제어를 사용한다.

### 단계 3. 고정 Z와 자유 Z 비교

초기 관측 Z를 Cartesian 목표와 경로 제약으로 고정하면 다음 문제가 발생했다.

```text
고정 Z 이탈 5.000mm가 허용값을 초과했습니다
```

실제 장비는 XY를 이동할 때도 Z가 결합 이동했고, 고정 XYZ/RPY 목표의 IK가
없는 경우가 많았다. 반대로 Z를 완전히 자유롭게 두면 목표 근처로 내려가면서
포트가 카메라 화각에서 사라졌다.

결정:

- 이동 중 Z를 정확히 고정하는 path constraint는 제거한다.
- 관측은 초기 관측 높이에서 하고 큰 이동 목표는 포트 위 pre-approach를 쓴다.
- 이동 뒤에는 명령 Z가 아니라 실제 Z를 다시 읽는다.

### 단계 4. pre-approach와 재관측 복구

포트 Z보다 100~150mm 높은 pre-approach를 시험했다. 큰 XY 이동 후 관측 Z로
복귀하면 해당 XYZ/RPY의 해가 없거나 포트가 화각에 들어오지 않았다. 현재
XY를 유지한 채 Z만 30mm 올리는 복구도 다음처럼 실패했다.

```text
Cartesian 명령 후 로봇 무동작
error=32: 역기구학 해 없음
```

waypoint stop-and-go와 마지막 가시 자세 복귀도 구현했지만, 매 자세에서 포트가
보인다는 보장이 없고 SolvePnP/Hand-eye 변화가 새 오차로 들어왔다.

결정:

- waypoint/Z-only 복구는 레거시 PBVS 진단 경로로 남긴다.
- 현재 주 경로는 초기 관측 목표를 한 번 고정한 frozen-target 방식으로 전환한다.

### 단계 5. frozen-target XY 제어

초기 관측 PBVS 목표 XY를 저장하고 이후 카메라 재관측 대신 flange/base 좌표
오차를 사용했다. coarse 이동 뒤 실제 좌표에서 목표까지 남은 XY를 최대 횟수만
stop-and-go로 보정했다.

관측 결과:

- 첫 coarse에서 약 50~90mm 이동 후 목표 근처까지 도달했다.
- bridge의 `get_coords()` 잔여오차가 약 3~5mm 수준에서 더 줄지 않는 경우가
  있었다.
- 오차 개선량이 일시적으로 음수여도 다음 반복에서 다시 감소할 수 있었다.

결정:

- `robot_xy_minimum_improvement_m` 미달을 무조건 중단 조건으로 쓰지 않는다.
- 하드웨어 반복오차를 고려해 XY 완료 허용값을 수 mm로 둔다.
- 카메라 재관측 오차와 로봇 좌표 추종오차를 별도로 기록한다.

### 단계 6. Yaw 보정

초기에는 keypoint 장축각과 Joint6 변화가 1:1이라고 가정했지만 실제 영상각
변화가 달랐다. 실측 예시는 다음과 같다.

```text
보정 전 keypoint angle: 약 +6.95deg
보정 후 keypoint angle: 약 -1.97deg
```

또한 포트가 화면에서 사라지면 Yaw 후 keypoint 토픽이 더 이상 나오지 않았다.

결정:

- Yaw는 영상각 오차와 실측 gain으로 Joint6만 움직인다.
- 한 번의 Yaw 상한과 Joint6 절대 한계를 둔다.
- frozen-target 경로는 초기 관측 영상각을 저장해 XY 도착 후 적용한다.
- 실제 충전기 장착 방향이 확정되면 목표 영상각과 gain을 다시 측정한다.

### 단계 7. XY/Yaw 뒤 Roll/Pitch 결합 보정

XY를 맞추면 Roll/Pitch가 변하고 Roll/Pitch를 초기 관측값으로 복구하면 XY가
다시 변했다. 한 축의 단발 성공만으로 전체 정렬 완료를 판단할 수 없었다.

결정:

```text
XY 보정
→ 초기 관측 Roll/Pitch 복구
→ XY 재계산
→ Yaw
→ XY와 Roll/Pitch 최종 결합 검사
```

이 과정의 Cartesian Roll/Pitch 복구는 pre-approach 영역에서 비교적 잘
작동했으므로 이후 혼합 하강에서도 재사용한다.

### 단계 8. Cartesian Z P제어 실패

정렬 후 `send_coords()`로 Z를 5mm씩 내리고 XY/Roll-Pitch를 보정하려 했다.
mode 1과 mode 0을 모두 비교했지만 다음과 같은 실제 하강이 반복됐다.

```text
command=5.0mm, actual=18.8mm
command=5.0mm, actual=19.5mm
```

`send_coords()` 반환 `-1`은 실제 이동 여부와 일치하지 않았으며, 자세를
유지하는 목표도 Z 결합 이동을 막지 못했다.

결정:

- Cartesian Z 하강을 현재 권장 경로에서 사용하지 않는다.
- P제어는 명령값 누적이 아니라 매번 측정한 절대 Z 오차로 계산해야 한다.
- Z를 관절각 기반으로 분리하는 시험으로 전환한다.

### 단계 9. URDF Jacobian 관절 Z 하강

URDF에서 base→flange geometric Jacobian을 계산하고 damped least-squares로
수직 Z 관절 증분을 만들었다. 제조사 `send_angles()` bridge를 그대로
사용했다. 모델 예측과 실측 예시는 다음과 같다.

```text
Z command: 3.0mm
Jacobian predicted Z: 약 2.5mm
actual Z: 약 9.5mm
```

Joint5가 0도 부근이고 Jacobian 최소 singular value가 작아 자세가 특이점에
가까웠다. 초기 관절 자세를 유지한다는 요구 때문에 시작 자세 자체는 바꾸지
않고, 관절 step·하드 한계와 실측 재계산으로 대응했다.

결정:

- Z에는 관절 Jacobian을 사용하되 한 단계 최대 3mm/관절 최대 2도로 제한한다.
- 실제 하강 15mm를 넘으면 해당 사이클을 중단한다.
- 작은 예측값을 실제 이동량으로 간주하지 않고 반드시 `get_coords()`를 읽는다.

### 단계 10. 관절 Jacobian Roll/Pitch 보정 실패

Z 뒤 초기 Roll/Pitch도 Jacobian으로 보정했을 때 다음 결과가 나왔다.

```text
보정 전: roll=7.00deg, pitch=6.69deg
보정 후: roll=8.71deg, pitch=8.34deg
predicted Z change=0.2mm, actual Z drift=6.8mm
```

명령 관절 변화가 `0.858, -2.0, 1.86deg`처럼 bridge의 2도 관절 완료
허용값과 비슷했다. 일부 관절이 계산 비율대로 움직이지 않으면 Jacobian의
상쇄가 무너져 자세와 Z가 모두 악화됐다.

결정:

- Jacobian은 Z에만 사용한다.
- XY와 Roll/Pitch는 기존에 잘 동작한 Cartesian 좌표 보정으로 되돌린다.

### 단계 11. 현재 혼합 제어와 절대 Z P판정

현재 주 제어는 다음과 같다.

```text
관절 Jacobian Z
→ get_coords 측정
→ Cartesian XY
→ get_coords 측정
→ Cartesian 초기 Roll/Pitch
→ get_coords 측정
→ 포트 기반 절대 Z 잔여거리 재계산
```

Cartesian Roll/Pitch는 다음처럼 자세를 실제로 개선했다.

```text
roll: 9.13deg → 3.65deg
pitch: 8.79deg → 3.33deg
```

하지만 동시에 Z가 11.9mm 내려갔다. 처음에는 5mm Z drift 한계를 넘으면
실패시켰지만, 이미 이동이 끝난 뒤 결과를 폐기해도 실제 위치는 되돌아오지
않는다. 따라서 상대 drift만 보는 대신 다음 절대값으로 판정을 변경했다.

```text
remaining_z = actual_flange_z - port_based_target_flange_z
```

- 목표보다 충분히 높으면 새 실제 Z를 다음 P 사이클 시작값으로 수용한다.
- 목표 아래로 내려가면 중단한다.
- 혼합 한 사이클 총하강 30mm를 하드 한계로 둔다.
- 실측 결합 이동 9~12mm를 고려해 목표 15mm 위에서 자동 반복을 종료한다.

### 유지·레거시·미구현 구분

| 구분 | 경로 | 상태 |
|---|---|---|
| 주 사용 | frozen target `execute_once` | 유지 |
| 주 사용 | `descend_joint_z_once` | 유지, 단일 혼합 사이클 |
| 주 사용 | `descend_joint_z_to_guard` | 유지, 15mm guard까지 자동 반복 |
| 진단 | `yaw_only_once` | 유지 |
| 레거시 | PBVS `recover_z_once`, waypoint | 코드 유지, 현재 주 시험 아님 |
| 비권장 | Cartesian Z 반복 | 실측 오버슈트 때문에 주 경로에서 제외 |
| 제거 | 별도 `frozen_target_p` 실행기/launch | 기본 실행기에 통합되어 삭제 |
| 미구현 | 마지막 15mm 접촉 삽입 | 힘/컴플라이언스 확보 후 진행 |

## 1. `pymycobot을 찾을 수 없습니다`

### 증상

```text
ModuleNotFoundError: No module named 'pymycobot'
RuntimeError: pymycobot을 찾을 수 없습니다
```

### 원인

로봇 PC에서 ROS 2가 사용하는 Python과 `pymycobot`이 설치된 가상환경의
Python 경로가 달랐다. 단순히 쉘 프롬프트에 `(mycobot)`이 표시되는 것만으로
ROS launch 하위 프로세스의 import 경로가 보장되지 않는다.

### 해결

`scripts/run_robot_bridge.sh`가 가상환경 Python으로 `pymycobot`의
site-packages 경로를 구해 `PYTHONPATH`에 추가한다. 스크립트로 실행한다.

```bash
cd ~/Pinkk-robot-arm
./scripts/run_robot_bridge.sh
```

확인:

```bash
source ~/venv/mycobot/bin/activate
python -c "import pymycobot; print(pymycobot.__file__)"
```

## 2. ROS setup 소싱 시 `unbound variable`

### 증상

```text
AMENT_TRACE_SETUP_FILES: unbound variable
COLCON_CURRENT_PREFIX: unbound variable
```

### 원인

실행 스크립트의 `set -u`가 활성화된 상태에서 ROS/colcon setup 스크립트가
아직 정의되지 않은 환경변수를 참조했다.

### 해결

setup을 읽는 동안만 `set +u`로 전환하고 이후 다시 `set -u`를 적용한다.
현재 실행 스크립트의 `source_environment()`가 이 처리를 한다. 수동 소싱은:

```bash
set +u
source /opt/ros/jazzy/setup.bash
source ~/mycobot_moveit_ws/install/setup.bash
source ~/Pinkk-robot-arm/ros2_ws/install/setup.bash
set -u
```

## 3. 시리얼 장치 중복 접근

### 증상

```text
device reports readiness to read but returned no data
device disconnected or multiple access on port?
```

### 원인

기존 Python 또는 trajectory bridge 프로세스가 `/dev/ttyUSB0`를 열고 있는
상태에서 새 브리지를 시작했다.

### 확인과 해결

```bash
fuser -v /dev/ttyUSB0
```

표시된 기존 브리지 터미널을 정상적으로 `Ctrl+C` 종료하고 다시 확인한다.
포트를 사용하는 프로세스가 없을 때 브리지를 하나만 실행한다. PID를 확인하지
않은 광범위한 강제 종료 명령은 사용하지 않는다.

## 4. trajectory action이 없음

### 증상

```text
통합 trajectory bridge action이 없습니다:
/arm_group_controller/follow_joint_trajectory
```

### 원인

대부분 앞 단계에서 bridge가 `pymycobot` import 또는 시리얼 중복 접근으로
종료된 결과다. 복귀 노드 자체의 문제가 아니다.

### 확인

```bash
ros2 node list | grep pinkk_mycobot_trajectory_bridge
ros2 action list | grep follow_joint_trajectory
```

bridge 오류를 먼저 해결한 뒤 초기 관측 복귀를 다시 실행한다.

## 5. `joint6_flange` TF가 없음

### 증상

```text
lookupTransform argument source_frame does not exist: joint6_flange
```

### 원인

robot_state_publisher 또는 static virtual joint TF가 실행되지 않았거나,
노트북과 로봇 PC의 ROS domain/RMW 설정이 달랐다.

### 확인

```bash
echo "$ROS_DOMAIN_ID"
ros2 node list
ros2 run tf2_ros tf2_echo g_base joint6_flange
```

현재 노트북 frozen-target launch는 robot description 관련 launch를 함께
시작한다. 서로 다른 터미널에서 `ROS_DOMAIN_ID`, discovery range와 RMW를
같게 맞춘다.

## 6. 카메라 포트와 RQT 영상 문제

### 증상

- `/dev/video0`을 열 수 없음
- `/dev/video2`는 웹캠이며 로봇팔 카메라가 아님
- RQT에는 raw만 보이거나 영상이 없음

### 해결 순서

```bash
v4l2-ctl --list-devices
ls -l /dev/v4l/by-id/ 2>/dev/null
```

장치를 하나씩 확인한 뒤 `hybrid_runtime.yaml`의 `camera_device`를 수정한다.
가능하면 `/dev/videoN` 대신 `/dev/v4l/by-id/...` 경로를 사용한다.

```bash
ros2 topic list | grep image
ros2 topic hz /camera/image_raw
ros2 topic hz /robot_arm/perception/usb_port/debug_image
```

RQT가 없다면 `ros-jazzy-rqt-image-view` 설치 여부와 현재 ROS 환경을 확인한다.
YOLO 표시 영상은 `/robot_arm/perception/usb_port/debug_image`다.

## 7. YOLO 검출 시간이 오래됨

### 증상

```text
포트 자세 추정 거부: YOLO 검출 시간이 유효하지 않습니다: age=...
```

### 원인과 해결

노드 시작 직후 모델 로딩, GPU 초기화 또는 카메라/YOLO 주파수 저하로 첫
검출이 늦을 수 있다. 지속될 때는 다음을 확인한다.

```bash
ros2 topic hz /camera/image_raw
ros2 topic hz /robot_arm/perception/usb_port/keypoints
nvidia-smi
```

YOLO 로그의 `device=cuda:0`와 모델 경로를 확인한다. 잠깐의 첫 경고는 fresh
검출이 들어오면 사라지지만 계속되면 카메라 토픽과 모델 추론을 먼저 고친다.

## 8. 고정 관측 Z 또는 Z-only Cartesian 목표의 해가 없음

### 증상

- XY 목표는 유효해 보이지만 로봇이 움직이지 않음
- `error=32: 역기구학 해 없음`
- 현재 XY를 고정한 30mm Z 상승이 실행되지 않음

### 원인

6축 로봇에서 XYZ와 RPY를 모두 고정한 Cartesian 목표는 현재 관절 자세에서
해가 없을 수 있다. 특히 Joint5가 0도 부근인 자세는 Jacobian이 나빠져 작은
수직 이동에도 큰 관절 변화가 필요했다. `mode 0/1`을 바꾸는 것만으로 항상
해결되지 않는다.

### 적용한 해결

- 초기 큰 이동은 포트 위 pre-approach Z를 사용한다.
- 이동 경로의 Z 고정 제약을 사용하지 않는다.
- 최종 하강 Z만 URDF damped Jacobian으로 관절 증분을 계산한다.
- XY와 Roll/Pitch는 앞선 실기에서 더 잘 동작한 Cartesian 제어를 사용한다.

제조사 `solve_inv_kinematics()`의 `-1`은 펌웨어 무응답과 구분하기 어려워
사전 도달성 판정에서 제거했다. 반면 명령 후 실제 무동작과 firmware error 32가
함께 나오면 해당 목표가 실기에서 실행되지 않았다는 진단 근거로 사용한다.

## 9. `send_coords()` 응답 `-1`

### 오해하기 쉬운 점

일부 PyMyCobot/펌웨어 조합은 명령을 받아 실제로 움직여도 `-1` 또는 `None`을
반환한다. 반환값 하나만으로 IK 실패라고 판정하면 안 된다.

### 현재 판정

bridge는 명령 후 `get_coords()`, `is_moving()`, 실제 위치·자세 변화를
감시한다. 5초 동안 0.25mm/0.25도 이상의 변화가 없을 때만 무동작으로
종료하고 `get_error_information()`을 진단에 덧붙인다.

## 10. Cartesian 목표 전송 후 계속 진행 상태만 출력

### 증상

```text
moving=False, xy_residual=2.780mm,
orientation_residual=4.648deg
```

또는 허용값 경계에서 60초 timeout.

### 원인

로봇은 정지했지만 목표 완료 허용오차 중 하나를 조금 넘었다. coarse 시험에서
0.5mm/1도는 하드웨어 반복오차보다 작아 계속 timeout이 발생했다.

### 해결

bridge의 coarse 시험 허용값을 위치 5mm, 자세 5도로 완화하고 진행 로그에
실제 좌표를 포함했다. frozen 실행기는 coarse 부분 도달과 자세 복구 timeout을
받아 실제 pose를 다시 검사한다. 최종 삽입 정확도와 이 coarse 허용값은 별도
프로파일로 분리해야 한다.

## 11. Z 3~5mm 명령이 실제 9~19mm 이동

### 관측

- Cartesian Z 명령 5mm, 실제 약 18.8~19.5mm
- 관절 Jacobian Z 명령 3mm, 예측 약 2.5mm, 실제 약 9.5mm

### 원인 판단

단순 P gain 문제만은 아니다. 제조사 관절 정지 오차, 2도 관절 완료 허용값,
백래시·케이블 장력과 특이점 부근 Jacobian 민감도가 함께 작용한다. 작은 여러
관절 증분 중 일부가 목표 비율대로 실행되지 않으면 계산된 상쇄가 깨진다.

### 현재 해결과 한계

- 매 action 뒤 제조사 실제 pose를 새로 측정한다.
- 다음 Z는 이전 명령값이 아니라 포트 기반 절대 Z의 남은 거리로 다시 계산한다.
- Z 한 단계 하드 한계 15mm, 혼합 한 사이클 총하강 하드 한계 30mm를 둔다.
- 최종 목표 15mm 위에서 자동 반복을 종료한다.

이 조치는 오차를 측정 기반으로 흡수하지만 마지막 15mm를 안전하게 해결하지
않는다. 정확한 삽입에는 더 나은 관절 서보 인터페이스, 컴플라이언스 또는
힘/접촉 감지가 필요하다.

## 12. Z 하강 후 Roll/Pitch 악화

### 실패한 방법

관절 Jacobian으로 초기 Roll/Pitch까지 동시에 복구했을 때 계산상 Z 변화는
약 0.2mm였지만 실제 Z가 6.8mm 변하고 자세 오차가 약 1.7도 증가했다.

### 원인

최대 2도 관절 증분과 bridge의 2도 완료 허용오차가 같은 크기였다. 여러 관절의
미세 조합을 실기가 정확히 재현하지 못했고 특이점 부근 결합 이동이 커졌다.

### 해결

- 관절 Jacobian은 Z에만 사용한다.
- XY와 Roll/Pitch는 기존 정렬에서 검증된 `send_coords()`를 사용한다.
- Roll/Pitch 5도 이하는 불필요한 보정을 생략한다. 마지막 안전 여유 도달
  사이클에서도 5도를 넘으면 초기 관측 Roll/Pitch로 보정한 뒤 최종 측정한다.
- 보정 후 XY, Roll/Pitch, 절대 Z를 다시 측정한다.

Cartesian Roll/Pitch 복구는 예를 들어 Roll 9.13→3.65도, Pitch
8.79→3.33도로 개선했지만 Z가 11.9mm 함께 내려갔다. 따라서 Z drift만으로
폐기하지 않고 포트 기반 절대 목표까지 남은 거리를 사용하되 15mm guard를
유지한다.

## 13. YAML 수정이 적용되지 않음

소스의 `config/hybrid_runtime.yaml`을 수정해도 이미 실행 중인 노드는 값을
다시 읽지 않는다. launch를 재시작해야 한다.

- YAML만 수정: 빌드 불필요, launch 재시작 필요
- Python/launch/setup.py/package.xml 수정: 노트북 패키지 재빌드 필요
- robot bridge Python/YAML 수정: 로봇 PC pull, robot build, bridge 재시작 필요

노트북 빌드:

```bash
cd ~/Desktop/Pinkk-robot-arm
bash scripts/calibration/laptop_build_pinkk.sh
```

로봇 PC 빌드:

```bash
cd ~/Pinkk-robot-arm
bash scripts/calibration/robot_build_pinkk.sh
```

실제 로드값 확인:

```bash
ros2 param get /pinkk_frozen_target_executor_node joint_vertical_z_step_m
ros2 param get /pinkk_frozen_target_executor_node joint_vertical_final_z_guard_m
```

## 14. ROS domain 전환

스크립트는 이미 설정된 `ROS_DOMAIN_ID`를 우선하고 없으면 36을 사용한다.

```bash
ROS_DOMAIN_ID=38 ./scripts/run_laptop_frozen_target_test.sh
```

노트북과 로봇 PC의 domain이 반드시 같아야 한다. `.bashrc`를 자주 고치기보다
시험 명령 앞에 일시적으로 지정하는 방법을 권장한다.

## 15. Git 변경 파일이 수백~수천 개로 보임

원인은 대부분 colcon `build/install/log`, Python cache 또는 pytest cache다.
저장소 `.gitignore`는 다음을 제외한다.

```text
/build/ /install/ /log/
/ros2_ws/build/ /ros2_ws/install/ /ros2_ws/log/
.venv/ __pycache__/ .pytest_cache/ *.py[cod]
```

이미 추적 중인 산출물은 `.gitignore`만으로 사라지지 않으므로 Git index에서
별도 정리가 필요하다. 현재 소스 저장소에는 빌드 결과를 커밋하지 않는다.

## 16. 문제 보고 시 함께 남길 정보

다음 정보를 한 묶음으로 보관하면 원인 분리가 쉽다.

```bash
ros2 topic echo /robot_arm/frozen_target/status
ros2 topic echo /robot_arm/cartesian_pose_actual --once
ros2 topic echo /joint_states --once
ros2 node list
ros2 action list
```

추가로 로봇 PC bridge의 목표 좌표, 실제 진행 로그, 최종 error code와
노트북 상태 토픽의 한 사이클 전체를 함께 기록한다.
