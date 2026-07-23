# 레거시 수동 USB 검증

이 폴더는 USB-A 외곽 네 점을 사람이 클릭해 SolvePnP와 TF 체인을 점검하던 과거
검증 코드를 보관합니다.

최종 시스템에서는 사용하지 않습니다.

```text
최종 인식/제어:
ros2_ws/src/pinkk_usb_insertion

Hand-eye 자동 운영:
scripts/calibration/README_KO.md
```

`manual_usb_tf.py`는 회귀 분석이나 과거 실험 재현이 필요할 때만 사용합니다.
새 캘리브레이션 검증은 고정 ChArUco 보드의 자동 자세 비교로 수행하며, 실제 USB
정렬은 자동 검출과 PBVS를 사용합니다.

레거시 코드를 실행할 경우에도 다음 제한을 지킵니다.

- ChArUco와 Flask/OpenCV가 `/dev/video0`을 동시에 열지 않음
- bridge와 별도 pymycobot이 `/dev/ttyUSB0`을 동시에 열지 않음
- 클릭 TF 결과를 실제 삽입 정확도의 보장으로 취급하지 않음
