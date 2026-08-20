# sample 수집과 캘리브레이션 실행

이 폴더에는 실제로 실행하는 workflow 파일이 있습니다.

```text
collect_samples.py  로봇 pose와 ChArUco pose sample 수집
calibrate.py        저장된 sample로 Hand-Eye 계산
```

명령은 로봇 PC의 저장소 루트에서 실행합니다.

```bash
# 저장소 루트에서 실행
source ~/venv/mycobot/bin/activate
```

## 이 단계의 위치

전체 플로우에서 이 폴더는 `sample 수집 -> Hand-Eye 계산` 단계를 담당합니다.
수집 전에 로봇 연결과 카메라 intrinsic 준비가 끝나 있어야 하고, 계산 후에는
[validation/README.md](../validation/README.md)의 실시간 검증으로 넘어갑니다.

## SSH에서 OpenCV 창 표시

`collect_samples`와 실시간 검증은 `cv2.imshow()`로 카메라 창을 표시합니다. 일반 SSH나
VS Code Remote-SSH 터미널은 X11 전달 없이 연결될 수 있으므로, `DISPLAY`가 비어 있으면
다음과 같은 오류가 발생합니다.

```text
qt.qpa.xcb: could not connect to display
```

이 경우 로봇 PC에 접속된 터미널 안에서 다시 SSH를 실행하지 말고, **노트북의 일반
터미널**에서 X11 forwarding을 활성화해 새로 접속합니다.

```bash
ssh -Y -C jetcobot@192.168.6.1
```

- `-Y`: 신뢰된 X11 forwarding을 활성화해 로봇 PC의 OpenCV 창을 노트북에 표시합니다.
- `-C`: 전송 데이터를 압축합니다. 대문자 `-C`이며, 소문자 `-c`와 다릅니다.

접속 직후 `DISPLAY`가 자동으로 설정되었는지 확인합니다.

```bash
echo "$DISPLAY"
```

정상적인 값은 보통 `localhost:10.0`과 비슷합니다. 값이 비어 있다면 현재 연결에는 X11
forwarding이 적용되지 않은 것이므로 그 세션에서 수집 프로그램을 실행하지 않습니다.
기존 SSH 세션은 나중에 `-Y`를 추가할 수 없으므로 종료 후 노트북 터미널에서 다시
접속해야 합니다.

매번 옵션을 입력하지 않으려면 노트북의 `~/.ssh/config`에 다음 host를 등록할 수 있습니다.

```sshconfig
Host jetcobot-handeye
    HostName 192.168.6.1
    User jetcobot
    ForwardX11 yes
    ForwardX11Trusted yes
    Compression yes
```

이후에는 다음 명령으로 접속합니다.

```bash
ssh jetcobot-handeye
```

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
