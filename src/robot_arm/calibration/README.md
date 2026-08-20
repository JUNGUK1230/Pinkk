# 로봇팔 카메라 캘리브레이션

로봇팔 Eye-in-hand 카메라의 내부 파라미터, Hand-eye 캘리브레이션과 비전 관련
코드를 관리합니다.

## 대표 문서

- 전체 캘리브레이션 구분:
  [`CALIBRATION_GUIDE_KO.md`](CALIBRATION_GUIDE_KO.md)
- 실제 자동 수집·비교·활성화:
  [`scripts/calibration/README_KO.md`](../../../scripts/calibration/README_KO.md)
- 내부 파라미터:
  [`camera_intrinsics/README.md`](camera_intrinsics/README.md)
- Hand-eye 영구 기록:
  [`handeye/data/README.md`](handeye/data/README.md)

## 폴더

```text
camera_intrinsics/          카메라 matrix, distortion, 촬영·진단
handeye/                    수집·계산·검증 코드와 영구 데이터
CALIBRATION_GUIDE_KO.md     전체 보정 순서와 활성값 관리 기준
```

실제 ROS 제어는 이 폴더의 NPZ/NPY를 실행 중 직접 읽지 않습니다. 검증하고
활성화한 숫자를 `ros2_ws/src/pinkk_usb_insertion/config/robots/`의 로봇별
YAML에 반영해 사용합니다.

과거 Flask 카메라와 USB 수동 클릭 절차는 최종 시스템의 표준 경로가 아닙니다.
카메라 장치는 동시에 한 프로세스만 열어야 합니다.
