# 로봇 PC용 Eye-in-Hand Hand-Eye 캘리브레이션

이 패키지는 로봇 팔 끝단에 고정된 카메라의 `T_flange_camera`를 계산합니다.
모든 실행은 로봇 PC의 Linux 환경을 기준으로 하며, 카메라는 Flask 서버가 아니라
`cv2.VideoCapture()`로 직접 열고 로봇 pose는 `pymycobot`의 `mc.get_coords()`에서 읽습니다.

## 폴더 구성

```text
handeye_calibration_1828/
  config/      환경설정, 로봇 PC 준비, serial 연결 설정
  core/        좌표 변환, ChArUco 검출, Hand-Eye 계산 공통 로직
  runtime/     sample 수집과 최종 캘리브레이션 실행 파일
  validation/  로봇 연결과 계산 결과 검증 실행 파일
  data/        sample과 계산 결과 저장 위치
```

## 실행 순서

1. [config/README.md](config/README.md)를 보고 로봇 PC 환경, 카메라 intrinsic, serial port를 준비합니다.
2. [runtime/README.md](runtime/README.md)의 sample 수집 명령으로 여러 자세의 데이터를 저장합니다.
3. [runtime/README.md](runtime/README.md)의 계산 명령으로 `T_flange_camera`를 생성합니다.
4. [validation/README.md](validation/README.md)의 실시간 검증 명령으로 결과가 안정적인지 확인합니다.

## 전체 플로우

```text
1. 저장소 준비
   git clone 또는 git pull로 로봇 PC에 최신 코드를 준비합니다.

2. 실행 환경 준비
   가상환경을 활성화하고 OpenCV, NumPy, SciPy, pymycobot이 준비되어 있는지 확인합니다.

3. 카메라 intrinsic 준비
   camera_calibration/results/intrinsics.npz 파일이 있는지 확인합니다.

4. 설정값 확인
   config/settings.py에서 ChArUco 보드 실측값, 카메라 번호, 로봇 serial 기본값을 확인합니다.

5. 로봇 연결 확인
   validation/README.md의 읽기 전용 연결 검사로 get_coords(), reference frame, end type을 확인합니다.

6. sample 수집
   runtime.collect_samples를 실행하고, 로봇을 다양한 위치와 회전 자세로 움직이며 15~30개 sample을 저장합니다.

7. Hand-Eye 계산
   runtime.calibrate를 실행해 여러 방법의 결과를 비교하고 T_flange_camera를 저장합니다.

8. 실시간 검증
   validation.verify를 실행해 로봇 자세가 바뀌어도 고정 보드의 base 좌표가 안정적인지 확인합니다.

9. 결과 사용
   data/T_flange_camera.npy 또는 data/handeye_result.npz를 로봇-카메라 좌표 변환에 사용합니다.
```

최소 실행 명령만 모으면 다음 순서입니다.

```bash
cd ~/Pinkk-robot-arm
source ~/venv/mycobot/bin/activate

python3 -m src.robot_arm.robot_camera.handeye_calibration_1828.runtime.collect_samples \
  --camera 0

python3 -m src.robot_arm.robot_camera.handeye_calibration_1828.runtime.calibrate

python3 -m src.robot_arm.robot_camera.handeye_calibration_1828.validation.verify \
  --camera 0
```

## 좌표계 요약

```text
p_A = T_A_B @ p_B

T_base_flange    : flange 좌표 -> base 좌표
T_camera_charuco : ChArUco 좌표 -> camera 좌표
T_flange_camera  : camera 좌표 -> flange 좌표

T_base_charuco =
    T_base_flange @ T_flange_camera @ T_camera_charuco
```

OpenCV `calibrateHandEye()`에는 `gripper2base=T_base_flange`,
`target2cam=T_camera_charuco`를 넣고, 반환되는 `cam2gripper`를
`T_flange_camera`로 사용합니다.

자세한 좌표계와 모듈 설명은 [core/README.md](core/README.md)를 확인합니다.

## Git에 포함되지 않는 파일

측정 데이터와 계산 결과는 Git에 포함하지 않습니다.

```text
camera_calibration/results/*
handeye_calibration_1828/data/*
```

따라서 로봇 PC에는 카메라 intrinsic 파일을 다음 위치에 별도로 준비해야 합니다.

```text
src/robot_arm/robot_camera/camera_calibration/results/intrinsics.npz
```

현재 기준 intrinsic은 `640x480`, checkerboard square `28 mm`, RMS `0.345252 px`입니다.
