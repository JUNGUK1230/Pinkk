# 캘리브레이션 및 USB 좌표 검증 실행 스크립트

이 폴더의 스크립트는 자신의 위치에서 저장소 루트를 자동 계산합니다. 따라서
노트북의 `~/Desktop/Pinkk-robot-arm`과 로봇 PC의 `~/Pinkk-robot-arm`처럼 clone
경로가 달라도 저장소 안에서 스크립트를 실행하면 됩니다.

이 폴더에는 **Easy Handeye2 자동 캘리브레이션**과 **이미 계산된 결과를 이용한
USB TF 정확도 검증**이라는 두 실행 흐름이 있습니다. USB 검증 이동은 실제 USB
삽입 절차가 아닙니다.

최종 통합 작업 브랜치는 `robot_arm_1828`입니다. Hand-eye 실행 이력과 활성값,
USB/YOLO/PBVS 설정도 이 브랜치에서 함께 관리합니다.

## 처음 한 번: pull 후 로컬 패키지 다시 빌드

소스가 바뀌어도 ROS 2가 이전 설치본을 실행할 수 있으므로 `git pull` 뒤에는
프로젝트 전용 overlay인 `~/mycobot_moveit_ws/install_pinkk`를 다시 빌드합니다.

로봇 PC:

```bash
cd ~/Pinkk-robot-arm
bash scripts/calibration/robot_build_pinkk.sh
```

노트북:

```bash
cd ~/Desktop/Pinkk-robot-arm
bash scripts/calibration/laptop_build_pinkk.sh
```

빌드가 끝나면 새 터미널을 열고 아래 실행 스크립트들을 시작합니다.

실제로 어느 소스가 실행되는지는 다음 명령으로 확인할 수 있습니다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/mycobot_moveit_ws/install/setup.bash
source ~/mycobot_moveit_ws/install_bridge/setup.bash 2>/dev/null || true
source ~/mycobot_moveit_ws/install_pinkk/setup.bash

ros2 pkg prefix pinkk_mycobot_bridge
ros2 pkg prefix pinkk_handeye_automation
ros2 pkg prefix pinkk_usb_insertion_interfaces
ros2 pkg prefix pinkk_usb_insertion
```

네 Pinkk 패키지 경로가 모두 `~/mycobot_moveit_ws/install_pinkk` 아래로 나오면
최신 프로젝트 overlay가 선택된 것입니다.

## 두 플로우를 섞지 않기

### A. Hand-eye를 새로 계산할 때

```text
로봇 bridge + ChArUco TF + MoveIt + Easy Handeye2 서버
→ 자동수집 check
→ 자동수집 execute
→ 결과 저장 및 별도 자세 검증
```

이때 Flask, USB 클릭 TF, 기존 Hand-eye static TF는 실행하지 않습니다.

### B. 이미 계산한 결과로 USB 좌표만 검증할 때

```text
1. 로봇 PC bridge
2. 노트북 MoveIt/RViz
3. 노트북 Hand-eye static TF
4. 로봇 PC Flask 카메라 서버
5. 노트북 USB 클릭 TF
6. 노트북 좌표 정확도 검증: observe -> 재클릭 -> check -> execute
```

이때 ChArUco TF와 Easy Handeye2 서버는 실행하지 않습니다.

모든 스크립트는 저장소 루트를 자동으로 찾고 ROS domain을 기본 `36`으로
설정합니다. 따라서 긴 `source` 명령이나 절대 프로젝트 경로를 매번 복사하지
않습니다.

## A. Hand-eye 자동 캘리브레이션

### A-1. 로봇 PC 터미널 1 — bridge

```bash
cd ~/Pinkk-robot-arm
bash scripts/calibration/robot_start_bridge.sh
```

기본값은 `speed=50`, 관절 도달 허용오차 `5.0°`입니다. 바꾸려면:

```bash
bash scripts/calibration/robot_start_bridge.sh 30 3.0
```

### A-2. 로봇 PC 터미널 2 — ChArUco TF

화면 없이 실행:

```bash
cd ~/Pinkk-robot-arm
bash scripts/calibration/robot_start_charuco.sh false
```

X11 접속으로 검출 화면도 볼 때는 마지막 값을 `true`로 바꿉니다.

### A-3. 노트북 터미널 1 — MoveIt/RViz

```bash
cd ~/Desktop/Pinkk-robot-arm
bash scripts/calibration/laptop_start_moveit.sh
```

### A-4. 노트북 터미널 2 — Easy Handeye2 서버

```bash
cd ~/Desktop/Pinkk-robot-arm
bash scripts/calibration/laptop_start_easy_handeye.sh
```

### A-5. 노트북 터미널 3 — 자동 수집

먼저 이동 없는 검사:

```bash
cd ~/Desktop/Pinkk-robot-arm
bash scripts/calibration/laptop_auto_handeye.sh check 30 20
```

IK 후보를 확인한 다음 실제 수집:

```bash
bash scripts/calibration/laptop_auto_handeye.sh execute 30 20 bracket_fixed
```

`30`은 목표 유효 샘플 수, `20`은 계산을 허용하는 최소 유효 샘플 수,
`bracket_fixed`는 이번 실행을 구분하는 이름입니다. 현재 자동 자세 목록은 최대
30개이므로 목표를 30보다 크게 지정하지 않습니다.

`execute`를 시작할 때 다음 폴더가 먼저 만들어집니다.

```text
src/robot_arm/robot_camera/handeye_calibration_1828/data/runs/
  YYYYMMDD_HHMMSS_bracket_fixed/
```

수집이 끝나면 Easy Handeye2 홈 폴더에서 다음 파일을 자동 보관합니다.

```text
metadata.json
samples.samples
calibration.calib
T_flange_camera.npy
```

계산이 실패해도 새로 저장된 샘플이 있으면 `samples_only` 상태로 남습니다. 기존
run은 덮어쓰거나 삭제하지 않습니다.

### A-6. 기존/신규 Hand-eye 결과를 같은 자동 자세에서 비교

이 비교는 새 샘플을 추가하거나 `.calib` 파일을 덮어쓰지 않습니다. 고정된
ChArUco 보드를 30개 자세에서 관측하고, 같은 원본 TF에 기존/신규 행렬을 각각
적용해 `g_base` 기준 보드 자세의 산포를 계산합니다. 위치 산포(mm)와 회전
산포(deg)가 작은 결과가 자세가 바뀌어도 더 일관적인 결과입니다.

다음 세 프로세스만 유지합니다.

```text
로봇 PC: bridge + ChArUco TF
노트북: MoveIt/RViz
```

Easy Handeye2 서버와 기존/신규 Hand-eye static TF publisher는 모두 종료합니다.
ChArUco 보드는 비교가 끝날 때까지 절대 움직이지 않습니다.

먼저 이동 없는 IK 검사:

```bash
cd ~/Desktop/Pinkk-robot-arm
export ROS_DOMAIN_ID=38
bash scripts/calibration/laptop_compare_handeye.sh check 30
```

IK 결과를 확인한 뒤 실제 비교:

```bash
bash scripts/calibration/laptop_compare_handeye.sh execute 30
```

기본 비교 run은 저장소에 보관된 다음 두 결과입니다.

```text
OLD: 20260715_baseline_old
NEW: 20260723_auto_30samples
```

결과는 기본적으로 저장소에 영구 보관됩니다.

```text
src/robot_arm/robot_camera/handeye_calibration_1828/data/comparisons/
  YYYYMMDD_HHMMSS_<old>_vs_<new>/
    metadata.json
    measurements.csv
    measurements.summary.json
```

CSV에는 각 자세의 old/new 보드 좌표가 들어가며, JSON에는 전체 자세의 위치
RMS/최대 산포와 회전 RMS/최대 산포가 들어갑니다. 별도 파일을 비교하려면:

```bash
bash scripts/calibration/laptop_compare_handeye.sh check 30 \
  20260715_baseline_old 20260723_auto_30samples

bash scripts/calibration/laptop_compare_handeye.sh execute 30 \
  20260715_baseline_old 20260723_auto_30samples
```

run은 전체 폴더 이름 대신 유일하게 구분되는 일부 문자열도 사용할 수 있습니다.

### A-7. 결과 목록과 활성값 선택

저장된 결과 목록:

```bash
bash scripts/calibration/laptop_handeye_data.sh list
```

검증에서 선택한 run을 전체 시스템 활성값으로 적용:

```bash
bash scripts/calibration/laptop_handeye_data.sh activate 20260715_baseline_old
bash scripts/calibration/laptop_handeye_data.sh show-active
```

`activate`는 다음 파일을 한 번에 동기화합니다.

```text
data/active/calibration.calib
data/active/T_flange_camera.npy
data/T_flange_camera.npy
data/T_flange_camera_easy_handeye.npy
ros2_ws/src/pinkk_usb_insertion/config/handeye.yaml
```

활성값을 static TF로 발행:

```bash
bash scripts/calibration/laptop_publish_handeye_tf.sh
```

특정 run을 활성화하지 않고 임시 발행할 수도 있습니다.

```bash
bash scripts/calibration/laptop_publish_handeye_tf.sh 20260723_auto_30samples
```

## B. USB 좌표 정확도 검증

### B-1. 로봇 PC 터미널 1 — bridge

A-1과 동일하게 `robot_start_bridge.sh`를 실행합니다.

### B-2. 노트북 터미널 1 — MoveIt/RViz

A-3과 동일하게 `laptop_start_moveit.sh`를 실행합니다.

### B-3. 노트북 터미널 2 — Hand-eye static TF

```bash
cd ~/Desktop/Pinkk-robot-arm
bash scripts/calibration/laptop_publish_handeye_tf.sh
```

### B-4. 노트북 터미널 3 — USB 클릭 TF

먼저 로봇 PC에서 기존 Flask MJPEG 카메라 서버를 실행하고
`http://192.168.6.1:5000/stream`이 노트북에서 열리는지 확인합니다. 카메라는 한
프로세스만 열어야 하므로 ChArUco TF 노드는 이때 종료합니다.

```bash
cd ~/Desktop/Pinkk-robot-arm
bash scripts/calibration/laptop_manual_usb_tf.sh
```

화면에서 `r → f`를 누른 뒤 `1→2=물리적 긴 변`, `2→3=짧은 변`이 되게 같은
방향으로 네 점을 클릭합니다. 카메라 기준 깊이가 실제 거리 약 260 mm와 비슷한지
확인합니다.

### B-5. 노트북 터미널 4 — 좌표 정확도 검증

초기 관측 자세 이동:

```bash
bash scripts/calibration/laptop_usb_accuracy_check.sh observe
```

로봇이 움직였으므로 터미널 4의 화면에서 USB를 다시 클릭합니다. 이후 목표 출력:

```bash
bash scripts/calibration/laptop_usb_accuracy_check.sh show
```

이동 없는 IK 검사:

```bash
bash scripts/calibration/laptop_usb_accuracy_check.sh check
```

모든 IK가 성공한 경우에만 실제 검증 이동:

```bash
bash scripts/calibration/laptop_usb_accuracy_check.sh execute
```

## 고정된 검증 파라미터

긴 명령을 매번 입력하지 않도록 다음 값을 스크립트에 고정했습니다.

```text
transit Z         150 mm
USB standoff      100 mm
Yaw offset        +129.782° (Python 기본 설정)
회전 waypoint     1회
마지막 Z 간격     10 mm
```

값을 변경할 때는
[`laptop_usb_accuracy_check.sh`](laptop_usb_accuracy_check.sh)의 `COMMON_ARGS`만
수정합니다. 같은 값을 여러 README나 터미널에 따로 복사하지 않습니다.

## 종료

각 터미널에서 `Ctrl+C`를 누릅니다. 로봇 bridge를 먼저 종료하면 이후 이동
명령은 실행되지 않습니다. `/dev/ttyUSB0`는 bridge 하나만 열어야 합니다.
