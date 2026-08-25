# Pinkk 카메라 캘리브레이션 안내

카메라 캘리브레이션의 대표 실행 문서는
[`scripts/calibration/README_KO.md`](../../../scripts/calibration/README_KO.md)입니다.
이 문서는 내부 파라미터와 Hand-eye 결과의 역할 및 재측정 조건만 정리합니다.

## 1. 전체 좌표 흐름

```text
카메라 내부 캘리브레이션
→ Eye-in-hand 캘리브레이션
→ 고정 ChArUco 보드 자동 자세 비교
→ 검증된 run 활성화
→ USB 자동 인식/PBVS
```

최종 시스템은 USB 수동 클릭을 사용하지 않습니다. Hand-eye 검증 이후의 자동
USB 인식과 제어는 `pinkk_usb_insertion` 패키지가 담당합니다.

## 2. 현재 기준 파일

| 종류 | 파일 | 역할 |
|---|---|---|
| 카메라 내부 파라미터 | `camera_intrinsics/results/intrinsics.npz` | 왜곡 보정, ChArUco, PnP |
| 활성 Hand-eye 원본 | `handeye/data/active/calibration.calib` | Easy Handeye2 결과 |
| 활성 Hand-eye 행렬 | `handeye/data/active/T_flange_camera.npy` | camera → flange |
| 활성 run 정보 | `handeye/data/active/manifest.json` | 선택한 결과와 hash |
| 수집 이력 | `handeye/data/runs/` | 샘플, 결과, 실행 환경 |
| 비교 이력 | `handeye/data/comparisons/` | 자세별 CSV와 요약 |

기존 코드 호환용 `data/T_flange_camera.npy`와 USB 시스템의 `handeye.yaml`은
`laptop_handeye_data.sh activate RUN` 명령으로 활성값과 함께 동기화됩니다.

## 3. 다시 측정해야 하는 조건

### 카메라 내부 캘리브레이션

- 카메라 또는 렌즈 교체
- 초점 변경
- 해상도, crop 또는 resize 방식 변경
- 왜곡 보정이나 재투영 오차 악화

현재 기준 해상도는 640×480입니다. 내부 파라미터 절차는
[`camera_intrinsics/README.md`](camera_intrinsics/README.md)를 확인합니다.

### Hand-eye

- 카메라와 flange의 체결 위치 또는 각도 변경
- 카메라 브래킷 흔들림
- `joint6_flange` 또는 URDF 프레임 정의 변경
- 고정 보드 검증에서 자세별 산포 증가

그리퍼 끝이나 USB plug TCP만 바뀐 경우에는 Hand-eye가 아니라 별도의
`T_flange_tool`을 다시 측정합니다.

## 4. 좌표계

프로젝트 표기는 `p_A = T_A_B @ p_B`입니다.

```text
T_base_flange    : flange 좌표 → base 좌표
T_flange_camera  : camera 좌표 → flange 좌표
T_camera_board   : board 좌표 → camera 좌표

T_base_board =
  T_base_flange × T_flange_camera × T_camera_board
```

고정 보드가 실제로 움직이지 않았으므로 올바른 Hand-eye 결과라면 로봇 자세가
바뀌어도 `T_base_board`가 거의 일정해야 합니다.

## 5. 결과 선택 기준

자동 비교 JSON에서 다음 네 값을 확인합니다.

- `position_rms_mm`
- `position_max_mm`
- `rotation_rms_deg`
- `rotation_max_deg`

전반적으로 작은 결과가 자세 변화에 더 일관적입니다. 이 검사는 절대 위치
정확도나 삽입 성공을 단독으로 보장하지 않으므로 최종 시스템에서는 PBVS와
별도의 TCP/접촉 검증이 필요합니다.

현재 보관된 2026-07-23 비교에서는 `20260715_baseline_old`가 신규 30-sample
결과보다 네 지표 모두 작아 활성값으로 선택되어 있습니다.

## 6. 데이터 정책

- 서로 다른 run의 샘플을 한 계산에 섞지 않습니다.
- run과 comparison은 수정하거나 덮어쓰지 않습니다.
- 활성값은 파일을 직접 복사하지 않고 `activate RUN`으로 변경합니다.
- Hand-eye `.samples`, `.calib`, `.npy`, `.csv`, `.json`은 Git으로 추적합니다.
- 카메라 원본 이미지처럼 큰 파일은 `camera_intrinsics`의 별도 정책을 따릅니다.
- serial과 카메라는 각각 한 프로세스만 엽니다.

ROS2 장애 시에만
[`handeye/README.md`](handeye/README.md)의
수동 OpenCV 경로를 복구용으로 사용합니다.
