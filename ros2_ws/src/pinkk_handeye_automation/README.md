# pinkk_handeye_automation

ROS2 MoveIt을 이용해 Eye-in-hand 샘플을 자동 수집하고 두 calibration 결과를
동일한 로봇 자세에서 비교하는 패키지입니다.

실제 터미널 실행과 데이터 관리 순서는
[`scripts/calibration/README_KO.md`](../../../scripts/calibration/README_KO.md)를
따릅니다. 이 문서는 노드의 동작과 직접 launch 방법만 설명합니다.

## 실행 파일

| 실행 파일 | 역할 |
|---|---|
| `auto_collect` | 자동 관측 자세 이동, Easy Handeye2 수집·계산·저장 |
| `compare_calibrations` | 동일한 자세에서 두 결과의 고정 보드 산포 비교 |
| `usb_pre_approach` | 과거 수동 USB 검증용 레거시 노드 |

최종 USB 자동 인식과 PBVS는 `pinkk_usb_insertion` 패키지가 담당합니다.

## 자동 수집

시작 시점의 `g_base → joint6_flange` 위치를 기준으로 local Roll/Pitch/Yaw를
변경한 최대 30개의 관측 자세를 만듭니다.

```text
MoveIt IK
→ FollowJointTrajectory 이동
→ 정지 대기
→ 최신 ChArUco TF 확인
→ Easy Handeye2 TakeSample
→ samples 저장
→ Tsai-Lenz 계산
→ calibration 저장
→ 시작 자세 복귀
```

목표 회전의 IK나 검출이 실패하면 같은 회전 방향을 유지하며 다음 비율로
축소합니다.

```text
100% → 75% → 50% → 35%
```

새 수집을 시작할 때 Easy Handeye2 서버 메모리에 기존 샘플이 있으면 실행을
중단합니다. 서로 다른 run의 샘플을 자동으로 섞지 않습니다.

직접 launch:

```bash
# 이동 없는 검사
ros2 launch pinkk_handeye_automation auto_calibrate.launch.py \
  execute:=false target_samples:=30 minimum_samples:=20

# 실제 이동과 계산
ros2 launch pinkk_handeye_automation auto_calibrate.launch.py \
  execute:=true target_samples:=30 minimum_samples:=20
```

직접 launch는 Easy Handeye2 홈 폴더까지만 저장합니다. Git 추적 run 폴더까지
자동 보관하려면 운영 wrapper를 사용합니다.

```bash
bash scripts/calibration/laptop_auto_handeye.sh execute 30 20 LABEL
```

## 두 결과 비교

ChArUco 보드를 고정한 상태에서 old/new에 동일한 원본 TF를 적용합니다.

```text
T_base_board_old =
  T_base_flange × T_flange_camera_old × T_camera_board

T_base_board_new =
  T_base_flange × T_flange_camera_new × T_camera_board
```

두 Hand-eye static TF를 동시에 publish하지 않으므로 TF 이름 충돌이 없습니다.
카메라 검출 시각의 로봇 TF를 사용해 시간 정렬하며, 각 자세에서 서로 다른
ChArUco timestamp를 10회 측정합니다.

직접 launch:

```bash
# 이동 없는 IK 검사
ros2 launch pinkk_handeye_automation compare_calibrations.launch.py \
  execute:=false \
  old_calib_path:=/path/old.calib \
  new_calib_path:=/path/new.calib

# 실제 비교
ros2 launch pinkk_handeye_automation compare_calibrations.launch.py \
  execute:=true \
  old_calib_path:=/path/old.calib \
  new_calib_path:=/path/new.calib \
  output_csv:=/path/measurements.csv
```

운영 wrapper는 run 선택자를 받아 결과를 Git 추적 `data/comparisons/` 폴더에
자동 보관합니다.

```bash
bash scripts/calibration/laptop_compare_handeye.sh execute 30 OLD_RUN NEW_RUN
```

## 비교 결과

CSV는 각 자세의 old/new 보드 좌표와 자세 내부 측정 산포를 기록합니다. 요약
JSON은 다음 값을 기록합니다.

- `position_rms_mm`
- `position_max_mm`
- `rotation_rms_deg`
- `rotation_max_deg`
- 두 calibration 행렬 사이 translation/rotation 차이

고정 보드의 위치·회전 산포가 전반적으로 작은 결과가 자세 변화에 더
일관적입니다. 이 검사는 절대 위치 정확도를 단독으로 보장하지 않습니다.

## 주요 파라미터

### `auto_collect`

| 파라미터 | 기본값 | 의미 |
|---|---:|---|
| `execute` | `false` | 실제 이동 허용 |
| `target_samples` | `15` | 목표 유효 샘플 |
| `minimum_samples` | `12` | 계산 최소 샘플 |
| `settle_seconds` | `1.5` | 이동 후 정지 대기 |
| `detection_timeout_seconds` | `8.0` | 자세별 검출 대기 |
| `max_tf_age_seconds` | `0.4` | ChArUco TF 최대 나이 |
| `motion_seconds` | `4.0` | trajectory 계획 시간 |
| `motion_retry_count` | `2` | 동일 목표 재시도 |
| `return_home` | `true` | 종료 시 시작 자세 복귀 |

운영 wrapper 기본값은 목표 30, 최소 20입니다.

### `compare_calibrations`

| 파라미터 | 기본값 | 의미 |
|---|---:|---|
| `execute` | `false` | 실제 비교 이동 허용 |
| `pose_limit` | `30` | 사용할 자동 자세 수 |
| `measurement_count` | `10` | 자세별 고유 TF 측정 수 |
| `settle_seconds` | `1.5` | 이동 후 정지 대기 |
| `detection_timeout_seconds` | `8.0` | 연속 측정 대기 |
| `old_calib_path` | 필수 | 기준 `.calib` |
| `new_calib_path` | 필수 | 후보 `.calib` |
| `output_csv` | 자동 | 자세별 결과 파일 |

## 실행 전 요구 조건

- 로봇 PC bridge
- 로봇 PC ChArUco TF
- 노트북 MoveIt/RViz
- 자동 수집 시에만 Easy Handeye2 서버
- Domain 38과 동일한 Fast DDS 설정

비교 중에는 Easy Handeye2 서버와 Hand-eye static TF publisher를 종료합니다.
