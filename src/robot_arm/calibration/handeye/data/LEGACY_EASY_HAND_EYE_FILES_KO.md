# 이전 Easy Handeye2 결과 파일 안내

기존에 한 폴더에 보관하던 파일은 `data/runs/` 구조로 이동했습니다.

| 파일 | 내용 |
|---|---|
| run | 내용 |
|---|---|
| `runs/20260715_baseline_old/` | 2026-07-15 기존 기준 결과 |
| `runs/20260723_auto_30samples/` | 2026-07-23 수집한 30개 샘플과 결과 |

2026-07-23 결과는 기존 결과와 위치 약 19.1 mm, 회전 약 6.17° 차이가 있으므로
고정 ChArUco 보드의 `g_base -> charuco_board` 안정성 검증을 마친 뒤 활성 결과를
선택합니다.

저장된 샘플로 다시 계산할 때는 `.samples` 파일을 Easy Handeye2 샘플 폴더에
복사하고 같은 이름으로 서버를 시작합니다.

```bash
mkdir -p ~/.ros2/easy_handeye2/samples

cp \
  src/robot_arm/calibration/handeye/data/runs/20260723_auto_30samples/samples.samples \
  ~/.ros2/easy_handeye2/samples/pinkk_eye_in_hand_30samples_20260723.samples
```

계산 결과를 Easy Handeye2 publisher로 사용할 때는 `.calib` 파일을 calibration
폴더에 복사합니다.

```bash
mkdir -p ~/.ros2/easy_handeye2/calibrations

cp \
  src/robot_arm/calibration/handeye/data/runs/20260723_auto_30samples/calibration.calib \
  ~/.ros2/easy_handeye2/calibrations/pinkk_eye_in_hand_30samples_20260723.calib
```
