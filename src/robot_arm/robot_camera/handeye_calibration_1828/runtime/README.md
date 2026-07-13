# sample 수집과 캘리브레이션 실행

이 폴더에는 실제로 실행하는 workflow 파일이 있습니다.

```text
collect_samples.py  로봇 pose와 ChArUco pose sample 수집
calibrate.py        저장된 sample로 Hand-Eye 계산
```

명령은 로봇 PC의 저장소 루트에서 실행합니다.

```bash
cd ~/Pinkk-robot-arm
source ~/venv/mycobot/bin/activate
```

## 이 단계의 위치

전체 플로우에서 이 폴더는 `sample 수집 -> Hand-Eye 계산` 단계를 담당합니다.
수집 전에 로봇 연결과 카메라 intrinsic 준비가 끝나 있어야 하고, 계산 후에는
[validation/README.md](../validation/README.md)의 실시간 검증으로 넘어갑니다.

## 1. sample 수집

ChArUco 보드를 완전히 고정하고 로봇을 여러 위치와 회전 자세로 이동합니다.

```bash
python3 -m src.robot_arm.robot_camera.handeye_calibration_1828.runtime.collect_samples \
  --camera 0
```

조작 키는 다음과 같습니다.

```text
S 또는 Space : 현재 sample 저장
Q 또는 ESC   : 종료
```

저장 전에는 로봇이 완전히 멈춰 있어야 합니다. 정면, 좌/우, 앞/뒤, 대각선 기울임을
포함해 `15~30개` sample을 권장합니다. 단순 이동만 반복하지 말고 회전 다양성을 반드시
확보합니다.

기존 sample 파일을 덮어쓰려면 다음 옵션을 사용합니다.

```bash
python3 -m src.robot_arm.robot_camera.handeye_calibration_1828.runtime.collect_samples \
  --camera 0 \
  --overwrite
```

저장 위치는 다음과 같습니다.

```text
src/robot_arm/robot_camera/handeye_calibration_1828/data/handeye_samples.npz
```

## 2. 계산과 방법 비교

sample 수집 후 다음 명령을 실행합니다.

```bash
python3 -m src.robot_arm.robot_camera.handeye_calibration_1828.runtime.calibrate
```

`TSAI`, `PARK`, `HORAUD`, `ANDREFF`, `DANIILIDIS` 방법을 모두 계산하고, 고정 보드의
base pose 일관성을 위치 평균/최대 `mm`와 회전 평균/최대 `deg`로 출력합니다. 값이
작을수록 안정적인 결과입니다.

특정 방법을 저장하려면 `--method`를 지정합니다.

```bash
python3 -m src.robot_arm.robot_camera.handeye_calibration_1828.runtime.calibrate \
  --method PARK
```

계산 결과는 다음 파일로 저장됩니다.

```text
src/robot_arm/robot_camera/handeye_calibration_1828/data/T_flange_camera.npy
src/robot_arm/robot_camera/handeye_calibration_1828/data/handeye_result.npz
```
