# Pinkk 카메라 캘리브레이션 대표 안내

이 문서는 카메라 관련 캘리브레이션의 **대표 진입점**입니다. 세부 명령과 구현
설명은 각 하위 README에 두고, 여기서는 어떤 방법을 언제 실행하고 어떤 결과를
사용하는지만 정리합니다.

## 1. 단계 구분

```text
카메라 내부 캘리브레이션
→ Hand-eye 캘리브레이션
→ 고정 보드로 Hand-eye 검증
→ USB SolvePnP/TF 좌표 검증
→ TCP 측정과 PBVS/IBVS
→ 실제 삽입
```

USB 좌표로 로봇을 이동하는 현재 실험은 캘리브레이션 결과의 정확도를 확인하는
단계입니다. 실제 삽입 제어와 동일하게 취급하지 않습니다.

## 2. 현재 사용하는 결과

| 종류 | 현재 기준 파일 | 용도 |
|---|---|---|
| 카메라 내부 파라미터 | `camera_calibration/results/intrinsics.npz` | 왜곡 보정과 모든 PnP |
| 활성 Hand-eye | `handeye_calibration_1828/data/T_flange_camera.npy` | camera 좌표를 flange로 변환 |
| Easy Handeye2 보관본 | `handeye_calibration_1828/data/T_flange_camera_easy_handeye.npy` | 현재 활성 결과의 원본 |
| 수동 ChArUco 비교본 | `handeye_calibration_1828/data/T_flange_camera_manual_charuco.npy` | 두 방법의 오차 비교 |

현재 `T_flange_camera.npy`와 `T_flange_camera_easy_handeye.npy`는 동일합니다. 즉,
프로젝트의 활성 Hand-eye 결과는 **ROS2 Easy Handeye2 결과**입니다. 수동 ChArUco
결과는 덮어쓰지 않고 비교본으로 유지합니다.

## 3. 폴더 역할

```text
camera_calibration/
  카메라 matrix와 distortion 계산 및 진단

handeye_calibration_1828/
  pymycobot + OpenCV 수동 Hand-eye 방식
  현재는 비교, 독립 검증, ROS2 장애 시 복구용

ros2_ws/src/pinkk_handeye_automation/
  MoveIt + Easy Handeye2 자동 관측/수집
  현재 권장 Hand-eye 계산 경로
  USB 좌표 정확도 검증용 시험 이동 포함

ros2_ws/src/pinkk_mycobot_bridge/
  실제 로봇 joint state와 trajectory 연결
  캘리브레이션 계산 자체를 담당하지 않음
```

`handeye_calibration_1828`의 숫자 이름은 기존 import와 로봇 PC 실행 명령을
유지하기 위한 호환 이름입니다. 현재 단계에서는 임의로 이름을 바꾸지 않습니다.

## 4. 언제 다시 실행하는가

### 내부 캘리브레이션을 다시 하는 조건

- 카메라 또는 렌즈 교체
- 초점 변경
- 사용하는 해상도 또는 crop/resize 방식 변경
- 카메라 matrix 진단 결과 악화

현재 기준은 640×480이며 RMS 재투영 오차는 약 0.345 px입니다.

세부 절차: [`camera_calibration/README.md`](camera_calibration/README.md)

### Hand-eye를 다시 하는 조건

- 카메라와 flange 체결 위치 또는 각도 변경
- 카메라 브래킷 흔들림
- flange/URDF 프레임 정의 변경
- 고정 보드 검증에서 base 위치가 자세에 따라 크게 변화

그리퍼 끝이나 USB 충전기 TCP 변경만으로는 `T_flange_camera`를 다시 계산하지
않습니다. 그 경우에는 별도의 flange/tool 변환을 측정합니다.

## 5. 권장 Hand-eye 경로

현재 권장 방식은 ROS2 Easy Handeye2 자동 수집입니다.

```text
robot bridge와 MoveIt 준비
→ ChArUco TF 확인
→ 자동 자세 IK DRY RUN
→ 유효 샘플 자동 수집
→ Easy Handeye2 계산/저장
→ 활성 T_flange_camera로 복사
→ 고정 보드 및 USB TF 검증
```

자동 수집 명령과 USB 검증 명령:
[`../../../ros2_ws/src/pinkk_handeye_automation/README.md`](../../../ros2_ws/src/pinkk_handeye_automation/README.md)

로봇 PC와 노트북 터미널 구성:
[`../../../ros2_ws/src/pinkk_mycobot_bridge/HAND_EYE_HANDOFF_KO.md`](../../../ros2_ws/src/pinkk_mycobot_bridge/HAND_EYE_HANDOFF_KO.md)

이미 계산된 결과로 USB 좌표 정확도를 검증하는 현재 표준 실행 스크립트:
[`../../../scripts/calibration/README_KO.md`](../../../scripts/calibration/README_KO.md)

## 6. 수동 Hand-eye 경로

ROS2와 Easy Handeye2를 사용하지 않고 로봇 PC에서 독립적으로 재현하거나 결과를
비교할 때만 사용합니다.

```text
pymycobot에서 T_base_flange 읽기
→ OpenCV에서 T_camera_charuco 계산
→ 15~30개 수동 샘플
→ calibrateHandEye 방법 5종 비교
→ 고정 보드 base pose 안정성 검증
```

세부 절차:
[`handeye_calibration_1828/README.md`](handeye_calibration_1828/README.md)

수동 계산 결과를 현재 활성 파일에 바로 덮어쓰지 않습니다. 먼저
`T_flange_camera_manual_charuco.npy`로 보관하고 Easy Handeye2 결과와 검증 오차를
비교한 뒤 활성 결과를 선택합니다.

## 7. 결과 검증 기준

### 내부 파라미터

- RMS reprojection: 0.5 px 이하 권장
- 캘리브레이션과 실제 PnP 영상의 해상도/crop이 동일
- 왜곡 보정 후 직선과 영상 모서리가 자연스러움

### Hand-eye

- ChArUco 보드를 움직이지 않음
- 로봇 자세를 바꿔도 `T_base_charuco`가 거의 일정
- 여러 방법과 반복 수집 결과가 비슷함
- 현재 실험에서는 위치 약 8 mm, 회전 약 1.7° 수준의 잔차를 확인했으며 실제
  작업 오차는 USB TF 반복 측정으로 별도 기록

### USB SolvePnP/TF

- 1→2는 USB 물리적 긴 변 11.5 mm
- 2→3은 짧은 변 4.5 mm
- 카메라 기준 깊이가 실제 렌즈–USB 거리와 비슷함
- 같은 USB를 반복 클릭했을 때 base XYZ/Yaw가 안정적

## 8. 섞어 쓰지 말아야 할 것

- 수동 샘플과 Easy Handeye2 샘플을 한 계산에 혼합하지 않음
- manual ChArUco 결과를 Easy Handeye2 결과 파일에 무조건 덮어쓰지 않음
- 캘리브레이션 중 Flask와 직접 `VideoCapture`가 같은 카메라를 동시에 열지 않음
- 실제 이동 bridge와 Jupyter/pymycobot이 `/dev/ttyUSB0`를 동시에 열지 않음
- USB PRE 검증 코드를 실제 삽입 완료 코드로 간주하지 않음

## 9. 다음 정리 대상

현재 기능을 유지한 채 문서의 대표 경로만 정리했습니다. 다음 구조 변경은 기존
실행 명령과 import 경로에 영향을 주므로 별도 승인 후 진행합니다.

- `handeye_calibration_1828` 이름 단순화
- `usb_pre_approach.py`를 `validation` 성격의 이름/폴더로 이동
- 긴 인수인계 문서에서 완료된 설치 기록을 `archive`로 분리
