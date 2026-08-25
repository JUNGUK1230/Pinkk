# Robot camera

로봇팔 Eye-in-hand 카메라의 내부 파라미터, Hand-eye 캘리브레이션과 비전 관련
코드를 관리합니다.

## 대표 문서

- 전체 캘리브레이션 구분:
  [`CALIBRATION_GUIDE_KO.md`](CALIBRATION_GUIDE_KO.md)
- 실제 자동 수집·비교·활성화:
  [`scripts/calibration/README_KO.md`](../../../scripts/calibration/README_KO.md)
- 내부 파라미터:
  [`camera_calibration/README.md`](camera_calibration/README.md)
- Hand-eye 영구 기록:
  [`handeye_calibration_1828/data/README.md`](handeye_calibration_1828/data/README.md)

## 폴더

```text
camera_calibration/         카메라 matrix, distortion, 진단
handeye_calibration_1828/   수동 복구 도구와 영구 데이터
ros2 pinkk_mycobot_bridge   ChArUco TF
ros2 handeye_automation     자동 수집과 old/new 비교
ros2 pinkk_usb_insertion    최종 USB 자동 인식과 PBVS
```

## 이전 Flask 웹캠 실행 방법

로봇 웹캠 서버(SSH 접속):

```bash
source ~/venv/mycobot/bin/activate
cd ~/venv/mycobot
python3 flask_server.py
```

웹캠에 원격 접속할 노트북:

```bash
cd ~/Download
python3 test_0.py
```

이 Flask 카메라와 USB 수동 클릭 절차는 이전 방식이며 최종 시스템의 표준 경로가
아닙니다. 카메라 장치는 동시에 한 프로세스만 열어야 합니다.
