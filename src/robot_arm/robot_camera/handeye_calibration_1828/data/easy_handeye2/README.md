# Easy Handeye2 결과 보관

Easy Handeye2가 노트북의 `~/.ros2/easy_handeye2/`에 생성한 샘플과 계산 결과를
비교·재현할 수 있도록 날짜별로 보관합니다.

| 파일 | 내용 |
|---|---|
| `pinkk_eye_in_hand_20260715.calib` | 2026-07-15 기존 기준 결과 |
| `pinkk_eye_in_hand_30samples_20260723.samples` | 2026-07-23 수집한 30개 샘플 |
| `pinkk_eye_in_hand_30samples_20260723.calib` | 위 30개 샘플로 다시 계산한 결과 |

2026-07-23 결과는 기존 결과와 위치 약 19.1 mm, 회전 약 6.17° 차이가 있으므로
고정 ChArUco 보드의 `g_base -> charuco_board` 안정성 검증을 마친 뒤 활성 결과를
선택합니다.

저장된 샘플로 다시 계산할 때는 `.samples` 파일을 Easy Handeye2 샘플 폴더에
복사하고 같은 이름으로 서버를 시작합니다.

```bash
mkdir -p ~/.ros2/easy_handeye2/samples

cp \
  src/robot_arm/robot_camera/handeye_calibration_1828/data/easy_handeye2/pinkk_eye_in_hand_30samples_20260723.samples \
  ~/.ros2/easy_handeye2/samples/
```

계산 결과를 Easy Handeye2 publisher로 사용할 때는 `.calib` 파일을 calibration
폴더에 복사합니다.

```bash
mkdir -p ~/.ros2/easy_handeye2/calibrations

cp \
  src/robot_arm/robot_camera/handeye_calibration_1828/data/easy_handeye2/pinkk_eye_in_hand_30samples_20260723.calib \
  ~/.ros2/easy_handeye2/calibrations/
```
