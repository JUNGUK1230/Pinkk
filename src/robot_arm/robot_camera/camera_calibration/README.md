# 카메라 내부 캘리브레이션

이 폴더는 OpenCV를 이용한 카메라 내부 캘리브레이션 파일을 관리하는 공간입니다.

내부 캘리브레이션을 하면 다음 값을 얻을 수 있습니다.

```text
camera_matrix: 초점거리, 주점 등 카메라 내부 파라미터
dist_coeffs: 렌즈 왜곡 계수
```

이 값들은 나중에 YOLO로 찾은 USB 포트 픽셀 좌표를 보정하거나, 포트의 3D 위치를 추정할 때 사용합니다.

## 폴더 구조

```text
camera_calibration/
  images/
    raw/        # 체스보드 촬영 원본 이미지
    accepted/   # 필요하면 좋은 이미지만 따로 모아두는 폴더
  results/      # 캘리브레이션 결과 저장 폴더
  scripts/      # 실행 스크립트
```

## 진행 순서

1. OpenCV로 카메라 연결을 확인합니다.
2. 체스보드 패턴을 출력합니다.
3. 체스보드 한 칸의 실제 크기를 mm 단위로 측정합니다.
4. 여러 각도에서 체스보드 이미지를 20~40장 정도 촬영합니다.
5. OpenCV 내부 캘리브레이션을 실행합니다.
6. 재투영 오차와 왜곡 보정 결과를 확인합니다.

## 실행 위치

아래 명령어는 `camera_calibration` 폴더 안에서 실행하는 기준입니다.

```bash
cd src/robot_arm/robot_camera/camera_calibration
```

만약 저장소 루트가 아니라 다른 위치에서 실행한다면, 위 경로를 현재 위치에 맞게 조정하면 됩니다.

## OpenCV 카메라 연결 확인

먼저 카메라가 OpenCV에서 정상적으로 열리는지 확인합니다.

```bash
python3 scripts/test_opencv_camera.py --camera 2
```

USB 카메라가 몇 번 인덱스로 잡혔는지 모르면 먼저 스캔합니다.

```bash
python3 scripts/test_opencv_camera.py --scan
```

리눅스에서 USB 카메라 장치 경로를 직접 지정할 수도 있습니다.

```bash
python3 scripts/test_opencv_camera.py --camera /dev/video2
```

특정 해상도로 확인하고 싶으면 다음처럼 실행합니다.

```bash
python3 scripts/test_opencv_camera.py --camera 2 --width 1280 --height 720
```

실행 중 키 조작:

```text
q: 종료
```

## 이미지 촬영

```bash
python3 scripts/capture_checkerboard.py --camera 2
```

USB 카메라 장치 경로를 직접 지정하려면:

```bash
python3 scripts/capture_checkerboard.py --camera /dev/video2
```

실행 중 키 조작:

```text
s: 현재 프레임 저장
q: 종료
```

저장 위치:

```text
images/raw
```

## 캘리브레이션 실행

예시는 내부 코너가 `9 x 6`이고, 체스보드 한 칸 크기가 `25 mm`인 경우입니다.

주의: `cols`, `rows`는 체스보드 칸 수가 아니라 **내부 코너 개수**입니다.

```bash
python3 scripts/calibrate_intrinsics.py \
  --images images/raw \
  --cols 9 \
  --rows 6 \
  --square-size 25.0
```

주요 결과 파일:

```text
results/intrinsics.npz
results/intrinsics.yaml
results/calibration_report.txt
```

## 왜곡 보정 미리보기

캘리브레이션 결과를 이용해서 원본 영상과 왜곡 보정 영상을 나란히 확인합니다.

```bash
python3 scripts/preview_undistort.py \
  --calib results/intrinsics.npz \
  --camera 2
```

## 촬영 팁

- USB 포트 인식에 사용할 해상도와 같은 해상도로 촬영합니다.
- 체스보드를 화면 중앙뿐 아니라 모서리에도 위치시킵니다.
- 정면 사진만 찍지 말고 기울어진 각도도 포함합니다.
- 너무 흔들리거나 흐린 이미지는 제거합니다.
- 체스보드가 화면에서 너무 작게 나오지 않게 합니다.
- 조명 반사가 심하면 코너 검출이 실패할 수 있습니다.

## 결과 확인 기준

`calibration_report.txt`에서 `RMS reprojection error`를 확인합니다.

```text
0.2 ~ 0.5 px: 좋음
0.5 ~ 1.0 px: 사용 가능
1.0 px 이상: 이미지 재촬영 권장
```

## 이 프로젝트에서 사용하는 위치

USB 포트 인식 코드에서는 보통 아래 파일을 불러와 사용하면 됩니다.

```text
results/intrinsics.npz
```

예시:

```python
import numpy as np

data = np.load("results/intrinsics.npz")
camera_matrix = data["camera_matrix"]
dist_coeffs = data["dist_coeffs"]
```
