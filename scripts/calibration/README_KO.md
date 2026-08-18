# Hand-eye 자동 캘리브레이션 운영 가이드

이 문서는 `robot_arm_1828` 브랜치에서 Hand-eye 캘리브레이션을 새로 수집하고,
과거 결과와 비교한 뒤 전체 USB/YOLO/PBVS 시스템의 활성값을 선택하는 대표
실행 문서입니다.

```text
자동 수집 → run 영구 보관 → 이전 run과 비교 → 검증 결과 선택 → active 반영
```

최종 시스템은 USB 수동 클릭을 사용하지 않습니다. `manual_usb_tf`와
`usb_pre_approach`는 과거 검증용 코드로만 남아 있으며 이 문서의 표준 실행
순서에는 포함하지 않습니다.

## 1. 고정 환경

| 항목 | 값 |
|---|---|
| 최종 통합 브랜치 | `robot_arm_1828` |
| ROS 2 | Jazzy |
| ROS domain | `36` |
| RMW | `rmw_fastrtps_cpp` |
| 로봇 PC 계정 | `jetcobot@raspi` |
| 로봇 serial | `/dev/ttyUSB0`, `1000000 baud` |
| 카메라 | `/dev/video0`, 640×480 |
| Hand-eye 형식 | Eye-in-hand |
| 로봇 프레임 | `g_base → joint6_flange` |
| 검출 프레임 | `camera_optical_frame → charuco_board` |
| ChArUco 유효 기준 | 코너 25개 이상, 재투영 오차 0.7 px 이하 |
| 자동 관측 자세 | 최대 30개 |

스크립트는 domain 36과 Fast DDS를 기본값으로 사용하므로 일반적으로 별도 `export`가 필요하지 않습니다.
직접 ROS 명령을 실행할 때는 두 PC에서 다음 값을 동일하게 사용합니다.

```bash
export ROS_DOMAIN_ID=36
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
```

## 2. 데이터 보존 원칙

Hand-eye 실행 한 번을 하나의 `run`으로 저장합니다.

```text
src/robot_arm/robot_camera/handeye_calibration_1828/data/
  runs/
    YYYYMMDD_HHMMSS_<label>/
      metadata.json
      samples.samples
      calibration.calib
      T_flange_camera.npy
  comparisons/
    YYYYMMDD_HHMMSS_<old>_vs_<new>/
      metadata.json
      measurements.csv
      measurements.summary.json
  active/
    manifest.json
    calibration.calib
    T_flange_camera.npy
```

- `runs/`와 `comparisons/`는 과거 이력이며 덮어쓰거나 재사용하지 않습니다.
- `.samples`, `.calib`, `.npy`, `.csv`, `.json`은 Git ignore 대상이 아닙니다.
- Easy Handeye2가 홈 폴더의 같은 파일명을 덮어써도 wrapper가 매 실행 결과를 새
  run 폴더에 복사합니다.
- 새 수집에서 Easy Handeye 서버 메모리에 과거 샘플이 있으면 실행을 거부합니다.
- `active/`는 비교가 끝난 후 명시적으로 선택한 결과만 담습니다.

상세 파일 규칙은
[`handeye_calibration_1828/data/README.md`](../../src/robot_arm/robot_camera/handeye_calibration_1828/data/README.md)를
참고합니다.

## 3. 새 PC 또는 pull 후 한 번 수행

두 PC에서 같은 브랜치를 사용합니다.

로봇 PC:

```bash
cd ~/Pinkk-robot-arm
git switch robot_arm_1828
git pull origin robot_arm_1828
bash scripts/calibration/robot_build_pinkk.sh
```

노트북:

```bash
cd ~/Desktop/Pinkk-robot-arm
git switch robot_arm_1828
git pull origin robot_arm_1828
bash scripts/calibration/laptop_build_pinkk.sh
```

빌드 후 새 터미널을 사용합니다. 노트북에서 overlay를 확인할 수 있습니다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/mycobot_moveit_ws/install/setup.bash
source ~/mycobot_moveit_ws/install_pinkk/setup.bash

ros2 pkg prefix pinkk_mycobot_bridge
ros2 pkg prefix pinkk_handeye_automation
ros2 pkg prefix pinkk_usb_insertion
```

모두 `~/mycobot_moveit_ws/install_pinkk` 아래여야 합니다.

## 4. 캘리브레이션 시작 전 확인

- 로봇 비상 정지와 작업 공간을 확인합니다.
- 카메라 브래킷과 ChArUco 보드를 단단히 고정합니다.
- 비교가 끝날 때까지 보드를 움직이지 않습니다.
- `/dev/ttyUSB0`는 bridge 한 프로세스만 엽니다.
- `/dev/video0`은 ChArUco 노드 한 프로세스만 엽니다.
- Flask 카메라, USB 수동 검출, Hand-eye static TF는 종료합니다.
- 두 PC 모두 Domain 38인지 확인합니다.

점유 프로세스 확인:

```bash
sudo lsof /dev/ttyUSB0
sudo lsof /dev/video0
```

`lsof`의 `fuse.portal` 경고는 장치 점유 결과와 무관합니다.

## 5. 새 Hand-eye run 수집

아래 네 터미널을 순서대로 실행하고 계속 켜 둡니다.

### 5.1 로봇 PC 터미널 1 — bridge

초기 검증에서는 낮은 속도 5를 권장합니다.

```bash
cd ~/Pinkk-robot-arm
bash scripts/calibration/robot_start_bridge.sh 5 5.0
```

정상 로그:

```text
실제 실행 브리지 준비: port=/dev/ttyUSB0 ...
관절 send_angles action 실행 차단
Cartesian send_coords action API 준비, 실행 차단
```

이 기본 명령은 `/joint_states`를 발행하지만 관절과 Cartesian 실제 이동은 모두
차단합니다. 검증된 관측 자세 이동에서만 다음처럼 관절 실행을 명시적으로 엽니다.

```bash
bash scripts/calibration/robot_start_bridge.sh \
  5 1.0 false 0.0015 true
```

노트북에서는 `observe`로 목표를 먼저 확인하고 `observe-execute`에서만 실제
이동합니다.

```bash
bash scripts/calibration/laptop_usb_accuracy_check.sh observe
bash scripts/calibration/laptop_usb_accuracy_check.sh observe-execute
```

### 5.2 로봇 PC 터미널 2 — ChArUco

SSH X11 화면을 보면서 실행:

```bash
cd ~/Pinkk-robot-arm
bash scripts/calibration/robot_start_charuco.sh true
```

화면이 필요 없으면:

```bash
bash scripts/calibration/robot_start_charuco.sh false
```

`ChArUco DETECTED`가 안정적으로 나오고 보드가 화면 밖으로 잘리지 않는지
확인합니다.

### 5.3 노트북 터미널 1 — MoveIt/RViz

```bash
cd ~/Desktop/Pinkk-robot-arm
bash scripts/calibration/laptop_start_moveit.sh
```

### 5.4 노트북 터미널 2 — Easy Handeye2

매 run마다 서버를 새로 시작합니다.

```bash
cd ~/Desktop/Pinkk-robot-arm
bash scripts/calibration/laptop_start_easy_handeye.sh
```

GUI의 `Load Samples`를 누르지 않습니다. 새 run은 빈 샘플 목록에서 시작해야
합니다.

### 5.5 노트북 터미널 3 — 이동 없는 검사

```bash
cd ~/Desktop/Pinkk-robot-arm
bash scripts/calibration/laptop_auto_handeye.sh check 30 20
```

- `30`: 목표 유효 샘플 수
- `20`: 계산을 허용하는 최소 샘플 수
- check는 IK와 서비스만 검사하며 로봇을 움직이지 않습니다.
- 현재 자동 자세가 30개이므로 목표를 30보다 크게 지정하지 않습니다.

### 5.6 실제 수집과 자동 보관

실행 이름은 장착 상태나 변경 이유를 알 수 있게 적습니다.

```bash
bash scripts/calibration/laptop_auto_handeye.sh execute 30 20 bracket_fixed
```

실행 과정:

```text
새 run 폴더 생성
→ 최대 30개 회전 자세로 이동
→ 정지 및 ChArUco 검출 확인
→ Easy Handeye2 sample 추가
→ 샘플/결과 저장
→ T_flange_camera.npy 생성
→ metadata와 SHA-256 기록
→ 최초 홈 자세 복귀
```

성공 후 출력된 run 이름을 기록합니다. 계산이 실패하고 샘플만 저장됐다면
metadata의 상태가 `samples_only`로 남으므로 원본 샘플은 유실되지 않습니다.
이 경우 로봇을 다시 움직여 수집하지 말고 보존된 샘플을 계산합니다.

```bash
bash scripts/calibration/laptop_handeye_data.sh compute RUN
```

`compute`는 Easy Handeye2와 같은 OpenCV Tsai–Lenz 계산을 사용하며 기존
`calibration.calib`은 덮어쓰지 않습니다.

## 6. 저장된 run 확인

```bash
cd ~/Desktop/Pinkk-robot-arm
bash scripts/calibration/laptop_handeye_data.sh list
```

예:

```text
RUN                                              STATUS             SAMPLES
20260715_baseline_old                            calibration_only   -
20260723_auto_30samples                          complete           30
20260723_150000_bracket_fixed                    complete           30
```

상태 의미:

| 상태 | 의미 |
|---|---|
| `complete` | samples, calibration, NPY가 모두 있음 |
| `samples_only` | 샘플은 보존됐지만 계산/저장이 끝나지 않음 |
| `calibration_only` | 결과만 보존된 과거 run |
| `no_new_artifacts` | 이번 실행에서 새 파일이 생성되지 않음 |

## 7. 이전 run과 자동 비교

비교할 때는 다음 프로세스만 유지합니다.

```text
로봇 PC: bridge + ChArUco
노트북: MoveIt/RViz
```

Easy Handeye2 서버와 모든 Hand-eye static TF publisher는 종료합니다.
ChArUco 보드는 수집 때와 같은 위치에 고정합니다.

이동 없는 검사:

```bash
bash scripts/calibration/laptop_compare_handeye.sh check 15 \
  20260715_baseline_old 20260723_150000_bracket_fixed 5
```

실제 비교:

```bash
bash scripts/calibration/laptop_compare_handeye.sh execute 15 \
  20260715_baseline_old 20260723_150000_bracket_fixed 5
```

run은 전체 이름 또는 유일하게 구분되는 일부 문자열을 사용할 수 있습니다.
두 번째 인자는 앞에서 시도할 자세 수가 아니라 **목표 유효 자세 수**입니다.
노드는 최대 30개 후보를 순회하고 ChArUco 측정에 성공한 자세가 목표 수에
도달하면 즉시 종료합니다. 마지막 인자는 자세당 반복 측정 횟수이며 권장값은
5회입니다. 실행 시간을 줄인 권장 비교는 다음과 같습니다.

```bash
bash scripts/calibration/laptop_compare_handeye.sh execute 15 \
  OLD_RUN NEW_RUN 5
```

한 번의 로봇 이동과 동일한 원본 TF에 두 행렬을 적용합니다.

```text
T_base_board_old = T_base_flange × T_flange_camera_old × T_camera_board
T_base_board_new = T_base_flange × T_flange_camera_new × T_camera_board
```

따라서 두 static TF를 동시에 publish하지 않으며 TF 충돌이 없습니다.

### 비교 지표

`measurements.summary.json`에서 다음 값을 봅니다.

| 지표 | 의미 |
|---|---|
| `position_rms_mm` | 고정 보드 위치의 전체 자세 RMS 산포 |
| `position_max_mm` | 가장 큰 위치 편차 |
| `rotation_rms_deg` | 고정 보드 회전의 전체 자세 RMS 산포 |
| `rotation_max_deg` | 가장 큰 회전 편차 |

네 값이 전반적으로 작은 run이 자세 변화에 더 일관적입니다. 이 비교는 고정 보드
일관성 검사이며 절대 위치 정확도나 실제 삽입 성공을 단독으로 보장하지 않습니다.

## 8. 검증 결과를 전체 시스템에 적용

목록 확인:

```bash
bash scripts/calibration/laptop_handeye_data.sh list
```

선택한 run 활성화:

```bash
bash scripts/calibration/laptop_handeye_data.sh activate 20260715_baseline_old
bash scripts/calibration/laptop_handeye_data.sh show-active
```

`activate`가 다음 파일을 같은 값으로 동기화합니다.

```text
data/active/calibration.calib
data/active/T_flange_camera.npy
data/T_flange_camera.npy
data/T_flange_camera_easy_handeye.npy
ros2_ws/src/pinkk_usb_insertion/config/handeye.yaml
install_pinkk/.../pinkk_usb_insertion/config/handeye.yaml
```

활성값을 static TF로 발행:

```bash
bash scripts/calibration/laptop_publish_handeye_tf.sh
```

활성값을 바꾸지 않고 특정 run을 임시 발행:

```bash
bash scripts/calibration/laptop_publish_handeye_tf.sh 20260723_auto_30samples
```

## 9. Git에 실험 이력 저장

새 run, 비교 결과와 active 변경을 확인합니다.

```bash
git status --short
git add \
  src/robot_arm/robot_camera/handeye_calibration_1828/data \
  ros2_ws/src/pinkk_usb_insertion/config/handeye.yaml
git commit -m "Record hand-eye calibration run"
git push origin robot_arm_1828
```

실험 데이터가 보이지 않으면 다음 명령으로 ignore 여부를 확인합니다.

```bash
git check-ignore -v \
  src/robot_arm/robot_camera/handeye_calibration_1828/data/runs/*/*
```

정상 상태에서는 아무것도 출력되지 않습니다.

## 10. 자주 발생하는 문제

### `/joint_states`를 받지 못함

- 로봇 PC bridge가 켜져 있는지 확인합니다.
- bridge와 MoveIt을 모두 Domain 38로 다시 시작합니다.
- 두 PC에서 `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`를 맞춥니다.

```bash
ros2 topic echo /joint_states --once
```

### `/dev/ttyUSB0` multiple access

bridge 외의 Jupyter, pymycobot Python 프로그램을 종료합니다. 한 프로세스만 serial
포트를 열어야 합니다.

### `/dev/video0`을 열지 못함

Flask, OpenCV 테스트 프로그램, 이전 ChArUco 노드를 모두 종료한 후 하나만 다시
실행합니다.

### X11 `xcb` 오류 또는 화면이 안 뜸

노트북 로컬 터미널에서 `echo "$DISPLAY"`를 확인한 뒤 `ssh -Y -C`로 로봇 PC에
다시 접속합니다. 화면이 필요하지 않으면 `robot_start_charuco.sh false`를
사용합니다.

### Easy Handeye service 부족

`get_sample_list` 하나만 보이거나 `take_sample`이 없으면 TF 준비가 끝나지 않은
상태입니다. ChArUco와 MoveIt TF를 먼저 확인한 후 Easy Handeye2 서버를
재시작합니다.

### 기존 샘플이 있다는 오류

Easy Handeye2 서버를 종료하고 새로 실행합니다. GUI에서 `Load Samples`를 누르지
않습니다. 과거 디스크 파일을 삭제할 필요는 없습니다.

### `cv2.calibrateHandEye`가 없음

`laptop_start_easy_handeye.sh`는 `~/.local`의 pip OpenCV를 제외하고 ROS
시스템 OpenCV를 검사한 뒤 서버를 시작합니다. 이미 `samples_only`가 됐다면
재수집하지 말고 다음 명령으로 계산을 복구합니다.

```bash
bash scripts/calibration/laptop_handeye_data.sh compute RUN
```

### 소스를 수정했는데 이전 코드가 실행됨

`laptop_build_pinkk.sh` 또는 `robot_build_pinkk.sh`로 다시 빌드하고 새 터미널을
엽니다.

## 11. 종료

각 터미널에서 `Ctrl+C`를 누릅니다. 로봇을 움직이는 작업이 끝나면 자동 수집과
MoveIt을 먼저 종료하고 bridge를 마지막에 종료합니다.

다음 작업 전에는 `laptop_handeye_data.sh list`와 `show-active`로 저장 및 활성
상태를 확인합니다.
