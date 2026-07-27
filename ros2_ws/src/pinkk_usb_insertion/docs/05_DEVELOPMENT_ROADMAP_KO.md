# 진행 현황과 개발 체크리스트

최종 갱신: 2026-07-27
대상 브랜치: `robot_arm_1828`

이 문서는 프로젝트의 현재 구현 범위, 아직 검증되지 않은 항목, 다음 작업 순서와
단계별 완료 조건을 한곳에서 관리합니다. 체크 표시는 소스 코드 기준이며 실제
로봇에서 검증되지 않은 기능은 구현됐더라도 별도로 표시합니다.

## 1. 최종 목표

```text
OBSERVE_POSE
→ YOLO USB 포트 keypoint 안정 검출
→ solvePnP 및 T_base_port 계산
→ PBVS X/Y/Yaw 폐루프 정렬
→ PRE_INSERT에서 관측 Roll/Pitch 복귀
→ 선택적 IBVS-PD 미세 정렬
→ 삽입축 1~2 mm 단계 전진
→ 연결 신호 확인
→ 성공 정지 또는 안전 후퇴
```

운용 제어 자유도는 다음을 목표로 합니다.

| 구간 | X/Y | Z | Roll/Pitch | Yaw |
|---|---|---|---|---|
| 초기 관측 | 현재값 저장 | 기준 높이 저장 | 기준값 저장 | 고정하지 않음 |
| PBVS 정렬 | 허용 | 관측 높이 유지 | 안전 범위 감시 | 포트 방향으로 보정 |
| PRE_INSERT 최종 확인 | 최종 보정 | 접근 높이 | 관측값으로 복귀 | 정렬값 유지 |
| 삽입 | 필요 시 극소 보정 | 삽입축으로만 이동 | 최종값 고정 | 최종값 고정 |

현재 PBVS 코드는 아직 초기 quaternion 전체를 고정하므로 Yaw 실행도 고정됩니다.
위 표는 TCP와 YOLO 방향 검증 후 바꿀 최종 정책입니다.

## 오늘 실기 검증 결과 (2026-07-27)

### 오늘 확인한 것

- [x] 로봇 PC와 노트북의 `robot_arm_1828` 동기화와 overlay 빌드
- [x] MyCobot 명령 큐 초기화, fresh mode와 정지 상태 확인
- [x] 관절/Cartesian 실행 게이트를 서로 독립적으로 차단
- [x] 초기 관측 자세 이동과 장시간 자세 유지
- [x] `/dev/video2` 카메라와 `usb_01.pt` CUDA 추론
- [x] YOLO 네 keypoint, solvePnP, 유효 `UsbPortObservation` 발행
- [x] RQT 디버그 영상과 ROS 토픽 통신
- [x] PBVS DRY RUN에서 안정적인 X/Y 오차와 제한 step 계산
- [x] ROS TF와 MyCobot `get_coords()` 비교
- [x] 직접 Cartesian X +1 mm 실기 시험과 Z 이탈 안전 정지

### 측정 결과

```text
ROS TF:       xyz ≈ [159, -66, 265] mm
MyCobot FK:   xyz = [159.3, -66.1, 261.6] mm
정적 차이:    Z 약 3.4 mm

send_coords(mode=1), X +1 mm 시험:
고정 Z 이탈 4.5 mm 감지 → 브리지 stop 및 실행 실패 처리

정지 후 get_coords 30회:
X/Y/Z/Roll/Pitch/Yaw span = 모두 0
```

따라서 4.5 mm는 측정 노이즈가 아니라 직접 Cartesian 실행에서 발생한 실제 좌표
변화로 판정합니다. `send_coords()`의 허용오차를 키워 재시도하지 않습니다.

### 현재 결정

```text
인지·solvePnP·PBVS 목표 계산: 계속 사용
MyCobot send_coords 실행: 중단
다음 실행 백엔드: MoveIt IK + 검증된 짧은 관절 waypoint
현재 실행 게이트: joint=false, cartesian=false
```

## 2. 현재 진행 현황

### 완료된 소프트웨어 작업

- [x] ROS 2 패키지와 custom interface 구조
- [x] 카메라 내부 파라미터 YAML 반영
- [x] Easy Handeye2의 `T_flange_camera` YAML 반영
- [x] `UsbPortDetectionArray`와 `UsbPortObservation` 메시지
- [x] 수동 네 점 입력을 YOLO와 같은 detection 메시지로 변환하는 개발용 노드
- [x] 네 keypoint와 실제 포트 치수를 이용한 `solvePnP`
- [x] 깊이·confidence·재투영 오차·키포인트 순서 검사
- [x] `T_base_flange × T_flange_camera × T_camera_port` 계산
- [x] 초기 기준 pose를 명시적으로 `capture/reset`하는 PBVS 인터페이스
- [x] 최대 10 mm 고정-Z PBVS X/Y 목표 계산
- [x] 목표까지 1 mm 간격 waypoint를 이용한 IK·충돌·관절 점프 사전검사
- [x] `/robot_arm/cartesian_move` action 정의
- [x] 브리지의 `send_coords()` 진단 경로와 실행 게이트
- [x] 실행 중 Z 및 Roll/Pitch 이탈 감시와 timeout 정지
- [x] joint action과 Cartesian action의 동시 실행 차단
- [x] Yaw 오차와 최대 2도 제한 계산 함수
- [x] 초기 IBVS XY P 제어 계산 함수
- [x] 상위 삽입 상태 이름과 DRY RUN 전이 골격
- [x] MyCobot 명령 큐 fresh mode·stop·정지 확인
- [x] CUDA YOLO Pose 노드와 디버그 영상
- [x] 노트북과 로봇 PC `install_pinkk` 빌드
- [x] bridge·insertion·hand-eye 단위 테스트 63개 통과

### 구현됐지만 실제 장비 검증이 필요한 작업

- [x] 로봇 PC `send_coords(..., mode=1)` 시험: Z 이탈로 사용 부적합 판정
- [x] ROS TF와 `get_coords()` 비교: Z 약 3.4 mm 정적 차이 확인
- [ ] Cartesian X/Y 1 mm 방향·반복성 검증
- [ ] Cartesian X/Y 5 mm 검증
- [ ] Cartesian X/Y 10 mm 검증
- [x] 이동 중 Z 이탈 감시와 실제 정지 확인
- [ ] action cancel과 timeout에서 `stop()` 동작 확인
- [ ] 이전에 발생한 전방 기울기와 Z 이동이 재발하지 않는지 확인

### 아직 구현 또는 연결되지 않은 작업

- [x] 약 500장 YOLO keypoint 데이터셋과 1차 모델 학습
- [x] `usb_01.pt` CUDA 추론과 `yolo_keypoint_node`
- [ ] 추가 촬영 세션을 포함한 모델 정량 평가와 재학습
- [ ] 다중 프레임 pose 안정화 필터
- [ ] 검출 소실 시 정지·후퇴 연동
- [ ] `T_flange_plug_tip` TCP 보정
- [ ] 카메라 중심이 아닌 plug tip 기준 PBVS
- [ ] Roll/Pitch와 Yaw를 분리한 자세 목표 생성
- [ ] 실제 Yaw PBVS 연결
- [ ] 오차 크기에 따른 적응형 PBVS step
- [ ] PRE_INSERT pose 계산과 실행
- [ ] IBVS-PD ROS 폐루프
- [ ] 삽입축 1~2 mm 단계 전진
- [ ] 연결 성공 신호 토픽
- [ ] 접촉·전류·힘 이상 감지
- [ ] 후퇴와 제한된 재시도
- [ ] 전체 자동 상태 머신

### 현재 안전 기본값

```yaml
execution:
  execution_enabled: false
  insertion_enabled: false

yaw_pbvs:
  enabled: false

ibvs:
  enabled: false
```

PBVS 실제 실행은 설정 파일과 별도로 launch 인자
`enable_pbvs_test_execution=true`를 명시해야 합니다. 실제 삽입은 구현되지 않았고
허용해서도 안 됩니다.

## 3. 다음 작업 순서

### A. 직접 Cartesian 경로 봉인

- [x] 기본 `cartesian_execution_enabled=false`
- [x] Z 이탈 시 stop과 실패 처리 확인
- [ ] 운영 launch와 실행 문서에서 `send_coords()`를 진단용으로만 표시
- [ ] PBVS step executor가 직접 Cartesian action을 호출하지 않도록 실행 경로 분리

### B. MoveIt IK 고정-Z 실행 백엔드 구현

```text
현재 g_base → joint6_flange TF
→ 목표 X/Y만 변경
→ 현재 Z/Roll/Pitch 복사
→ MoveIt IK
→ 충돌 및 관절 점프 검사
→ 짧은 FollowJointTrajectory
→ 실행 중 TF 감시
→ 정지 후 최종 pose 검증
```

- [ ] 현재 TF와 목표 TF를 함께 기록
- [ ] IK 해가 현재 관절 자세와 가까운지 검사
- [ ] 예상 FK의 Z/Roll/Pitch가 lock 기준 안인지 검사
- [ ] joint waypoint 최대 변화량과 속도 제한
- [ ] 실행 중 Z 1 mm, Roll/Pitch 1도 초기 감시 기준 적용
- [ ] TF stale, action timeout, 취소에서 즉시 stop
- [ ] 기본 DRY RUN과 이중 실행 승인 유지

### C. MoveIt IK 1 mm 실기 검증

- [ ] 장애물이 없는 초기 관측 자세에서 X +1 mm
- [ ] 초기 관측 자세 복귀 후 X -1 mm
- [ ] 초기 관측 자세 복귀 후 Y +1 mm
- [ ] 초기 관측 자세 복귀 후 Y -1 mm
- [ ] 각 시험의 실제 ΔX/Y/Z/Roll/Pitch/Yaw 기록
- [ ] 3회 이상 반복해 방향과 편차 확인
- [ ] 1 mm 시험 통과 전 5 mm와 PBVS 실행 금지

통과 기준:

- 명령 축과 실제 이동 방향 일치
- |ΔZ| ≤ 1 mm
- |ΔRoll|, |ΔPitch| ≤ 1도
- 급격한 IK branch 변경 없음
- 지연 이동과 잔류 명령 없음
- action 종료 후 자세 유지

### D. YOLO pose 안정화와 PBVS DRY RUN

- [ ] 정지 상태 100프레임 위치·Yaw 분산 기록
- [ ] confidence·재투영 오차·pose 이상치 필터
- [ ] 검출 소실과 stale observation 차단 시험
- [ ] X/Y 부호와 MoveIt IK 목표 방향 비교
- [ ] 한 프레임이 아닌 안정 구간으로 PBVS step 생성

### E. PBVS 실제 폐루프

- [ ] 최초 최대 step 1 mm로 시작
- [ ] 정지 → 새 YOLO → 재계산 방식 유지
- [ ] X/Y 오차 감소 방향을 매 step 확인
- [ ] 3회 이상 정상 감소 후 최대 3~5 mm 검토
- [ ] 카메라 중심 기준 1 mm 내 반복성 기록
- [ ] 통과 후 TCP, Yaw, PRE_INSERT 순서로 진행

### 이후 큰 흐름

```text
MoveIt IK 고정-Z 검증
→ YOLO pose 안정화
→ 카메라 중심 PBVS 폐루프
→ TCP offset 보정
→ X/Y/Yaw plug-tip PBVS
→ PRE_INSERT
→ 필요 시 IBVS-PD
→ 1~2 mm 단계 삽입
→ 연결 신호 또는 실패 후퇴
```

### 매 작업일 공통 시작 확인

- [ ] 양쪽 장치가 같은 `robot_arm_1828` 커밋인지 확인
- [ ] 로봇 PC에서 bridge/interface, 노트북에서 전체 overlay 빌드
- [ ] `ROS_DOMAIN_ID=36`, Fast DDS와 SUBNET discovery 일치
- [ ] `/dev/ttyUSB0`는 로봇 브리지 한 프로세스만 소유
- [ ] `/joint_states`와 `g_base → joint6_flange` TF 갱신 확인
- [ ] 실기 전 관절/Cartesian 실행 게이트가 모두 `false`인지 확인
- [ ] 필요한 시험 게이트 하나만 열고 종료 후 다시 모두 차단

## 4. YOLO keypoint 데이터셋

### 라벨 규칙

중심점을 별도로 라벨링하지 않고 네 물리적 모서리만 사용합니다.

```text
0 ───────── 1
│           │
3 ───────── 2
```

- [ ] 금속 외곽 또는 플라스틱 외곽 중 하나로 기준 통일
- [ ] 포트 자체의 물리적 방향을 기준으로 index 고정
- [ ] USB 내부 구조로 위·아래를 구분해 180도 뒤집힘 방지
- [ ] 보이지 않는 모서리를 임의로 추측하지 않기
- [ ] 네 점 중심은 코드에서 `(p0+p1+p2+p3)/4`로 계산

### 약 500장 수집

- [ ] 초기 관측 거리 약 100장
- [ ] 중간 접근 거리 약 100장
- [ ] PRE_INSERT 예상 거리 약 100장
- [ ] 좌우·상하·원근 시점 변화 약 80장
- [ ] 밝기·반사·그림자 변화 약 70장
- [ ] 부분 가림 약 30장
- [ ] 포트가 없는 negative 이미지 약 20장

연속 영상의 인접 프레임만 채우지 않고 촬영 세션을 분리합니다.

```text
train 350장 / validation 100장 / test 50장
```

- [ ] 같은 영상의 인접 프레임이 train과 test에 섞이지 않게 세션 단위 분리
- [ ] 흐림·심한 가림·중복 이미지 제거
- [ ] 최소 50장의 keypoint index를 사람이 재검수
- [ ] 좌우 반전 증강을 쓸 경우 keypoint index 변환 확인

### 모델 평가 완료 조건

- [ ] 별도 test 영상에서 포트 검출 성공률 기록
- [ ] 포트가 없는 영상의 오검출률 기록
- [ ] 정지 상태 중심점 픽셀 표준편차 기록
- [ ] 장축 각도 표준편차 기록
- [ ] keypoint index 뒤집힘이 없는지 확인
- [ ] 가까운 거리와 먼 거리 모두 평가

## 5. YOLO ROS 노드 연결

```text
/camera/image_raw
→ yolo_keypoint_node
→ /robot_arm/perception/usb_port/detections
→ port_pose_node
```

- [ ] 입력 영상 header와 timestamp 유지
- [ ] `camera_optical_frame` 사용
- [ ] letterbox 좌표를 원본 영상 좌표로 복원
- [ ] 객체 confidence와 각 keypoint confidence 발행
- [ ] 보이지 않는 keypoint의 visibility 처리
- [ ] `UsbPortDetectionArray` 메시지 순서 검증
- [ ] 수동 클릭 노드는 운영 launch에서 끄기

## 6. solvePnP와 pose 안정화

- [ ] USB 포트 실제 폭·높이 재확인
- [ ] YOLO 0~3 순서와 3D object point 순서 일치 확인
- [ ] 깊이 범위 검사
- [ ] 평균 재투영 오차 2 px 이하를 초기 기준으로 평가
- [ ] 정지 상태 10개 이상 관측 수집
- [ ] 위치·quaternion·Yaw 이상치 제거
- [ ] 중앙값 또는 confidence 가중 평균 적용
- [ ] 오래된 timestamp 거부
- [ ] 포트 pose의 180도 뒤집힘 감지
- [ ] RViz에서 `camera_optical_frame → usb_port` 확인
- [ ] RViz에서 `g_base → usb_port` 확인

한 프레임 결과로 로봇을 움직이지 않습니다. 검출이 끊기면 마지막 오차를 계속
적용하지 않고 새 명령을 차단합니다.

## 7. PBVS X/Y 검증

### DRY RUN

- [ ] OBSERVE_POSE에서 기준 pose 명시적 capture
- [ ] X/Y 오차 부호와 실제 이동 방향 확인
- [ ] 목표 Z가 기준 Z와 같은지 확인
- [ ] 목표 자세가 현재 정책과 일치하는지 확인
- [ ] 최대 이동량 10 mm 제한 확인
- [ ] stale observation과 TF 오류 거부 확인

### 실제 폐루프

```text
안정 검출
→ X/Y 오차 계산
→ 제한된 Cartesian 이동
→ 완전 정지
→ 새 YOLO 검출
→ 반복
```

목표 적응형 step:

| XY 오차 | 최대 1회 이동량 |
|---|---:|
| 20 mm 이상 | 10 mm |
| 10~20 mm | 5 mm |
| 3~10 mm | 1~3 mm |
| 1~3 mm | 0.5~1 mm |
| 1 mm 이하 | 최종 후보 |

- [ ] 적응형 P step 구현
- [ ] settle time을 실측해 불필요한 대기 축소
- [ ] 이동 전·중·후 Z/Roll/Pitch 기록
- [ ] 20~30회 반복 정렬 오차 통계 저장

TCP가 없을 때의 결과는 카메라 중심 정렬 시험이며 plug tip 정렬 정확도로
해석하지 않습니다.

## 8. TCP `T_flange_plug_tip`

TCP 프레임 권장 정의:

```text
원점: USB 플러그 삽입면 중심
X: 플러그 긴 변
Y: 플러그 짧은 변
Z: 플러그가 포트로 들어가는 삽입 방향
```

초기값을 얻는 방법은 프로젝트 상황에 맞게 선택합니다.

- [ ] 장착 지그 CAD와 캘리퍼스 X/Y/Z 실측
- [ ] 플러그 네 모서리 PnP와 Hand-eye를 이용한 비전 TCP
- [ ] 고정점 pivot calibration으로 tip translation 계산
- [ ] 캘리브레이션 포트 정렬 결과로 유효 TCP 보정

권장 절차:

```text
CAD/실측 또는 비전으로 초기값
→ RViz plug_tip 확인
→ 평면 또는 정렬 지그에서 안전거리 검증
→ 캘리브레이션 포트로 최종 offset 보정
→ 반복 장착 오차 평가
```

완료 조건:

- [ ] `tool_transform.yaml`에 측정 방법·날짜·값 기록
- [ ] 여러 로봇 자세에서 예측 plug tip 위치 일관성 확인
- [ ] translation과 rotation 반복 편차 기록
- [ ] 재장착 후에도 허용오차 안인지 확인
- [ ] 검증 전에는 `calibrated: false` 유지

## 9. PBVS Yaw와 최종 자세 정책

Yaw는 초기 관측값으로 고정하지 않습니다.

```text
PBVS X/Y 대략 정렬
→ 안전거리에서 Yaw 소량 보정
→ 새 YOLO 검출
→ X/Y 재정렬
→ Yaw 잔여 오차 보정
```

- [ ] 포트 장축의 base-frame Yaw 계산
- [ ] TCP에서 현재 플러그 장축 Yaw 계산
- [ ] 180도 대칭을 고려한 최단 각도 오차 사용
- [ ] 한 번의 Yaw 보정을 0.5~2도로 제한
- [ ] Yaw 회전 후 X/Y가 변하므로 반드시 재검출
- [ ] PBVS 이동 중 Roll/Pitch는 정확한 lock보다 안전 범위 감시
- [ ] PRE_INSERT 직전에 관측 Roll/Pitch로 복귀
- [ ] 정렬된 Yaw와 결합해 `q_insert_lock` 저장

현재 `yaw_pbvs.enabled=false`를 유지하며 TCP와 YOLO keypoint 방향을 확인한 뒤에만
실행합니다.

## 10. PRE_INSERT와 IBVS 선택

### PRE_INSERT

- [ ] 포트 삽입축 반대 방향으로 안전거리 정의
- [ ] plug tip 기준 PRE_INSERT pose 계산
- [ ] 카메라가 포트를 계속 볼 수 있는 최소 거리 확인
- [ ] PRE_INSERT 도착 후 YOLO 재검출
- [ ] Roll/Pitch 관측값 복귀 후 X/Y/Yaw 최종 재확인

### IBVS 추가 여부

IBVS는 필수가 아닙니다. TCP 적용 PBVS를 20~30회 반복해 결정합니다.

IBVS 없이 진행할 후보 조건:

- [ ] TCP-포트 XY 추정 오차가 반복적으로 약 1 mm 이내
- [ ] Yaw 오차가 정한 허용범위 이내
- [ ] PRE_INSERT 반복 편차가 삽입 공차보다 작음
- [ ] 기계적 가이드 또는 컴플라이언스로 잔여 오차 수용 가능

조건을 만족하지 않으면 다음을 구현합니다.

- [ ] 정렬된 teach pose에서 `desired_pixel` 저장
- [ ] 영상 해상도와 목표 중심·장축 각도 함께 저장
- [ ] 다중 프레임 필터 후 P 제어 방향 검증
- [ ] 픽셀 오차 변화율을 필터링한 D항 추가
- [ ] 0.1~0.5 mm 최대 step과 3 px 내외 수렴 기준 검증
- [ ] 근접 가림 시 즉시 정지·후퇴

## 11. 단계 삽입과 성공 판정

포트 삽입축이 base Z와 정확히 일치할 때만 단순 수직 하강을 사용합니다. 그렇지
않으면 `T_base_port`에서 계산한 삽입축으로 전진합니다.

```text
최종 정렬 확인
→ q_insert_lock 저장
→ 삽입축 1~2 mm 전진
→ 정지
→ 연결·접촉·검출 확인
→ 필요 시 미세 재정렬
→ 다시 전진
```

- [ ] 최대 삽입 거리 설정
- [ ] 삽입 속도 제한
- [ ] 단계당 1~2 mm 제한
- [ ] 연결 성공 Bool 토픽 정의
- [ ] 모터 전류·힘·접촉 중 가능한 안전 신호 연결
- [ ] 검출 소실 시 추가 전진 금지
- [ ] 성공 시 즉시 정지
- [ ] timeout 또는 최대 깊이에서 실패 처리
- [ ] 삽입축 반대 방향 후퇴
- [ ] 초기 자동 재시도 횟수 0으로 시작
- [ ] 후퇴 검증 후에만 제한된 재시도 허용

완료 기준은 단일 성공이 아니라 포트 위치와 초기 로봇 자세를 바꾼 반복 시험의
성공률로 평가합니다.

## 12. 통합 실행 안전 체크리스트

이 절은 실제 이동과 삽입 허용 여부를 판단하는 단일 안전 기준입니다. 설정값 하나를
바꿨다는 이유만으로 다음 단계가 자동 허용되지는 않습니다.

### 12.1 실행 스위치와 권한 분리

설정에는 의도가 다른 두 스위치가 있습니다.

```yaml
execution:
  execution_enabled: false
  insertion_enabled: false
```

- `execution_enabled`: 접근을 포함한 일반 실제 이동 허용 여부
- `insertion_enabled`: 포트 방향 저속 삽입을 별도로 허용하는 스위치
- `enable_pbvs_test_execution`: PBVS 단발 시험 실행을 명시적으로 여는 launch 인자

접근 시험에서는 다음과 같이 삽입을 계속 차단합니다.

```yaml
execution_enabled: true
insertion_enabled: false
```

PBVS 단발 시험도 launch 인자를 명시적으로 열기 전에는 실행되지 않습니다. 각
스위치의 책임이 다르므로 하나를 활성화했다고 다른 실행 권한까지 얻은 것으로
해석하지 않습니다.

로봇 PC 브리지는 별도의 최종 게이트를 가집니다.

```text
cartesian_execution_enabled=false  → Cartesian action goal 거부
cartesian_execution_enabled=true   → 제한 검사를 통과한 goal만 send_coords 실행
```

현재 `cartesian_execution_enabled=true`는 직접 Cartesian 경로의 진단에만 해당하며,
2026-07-27 Z 이탈 결과 때문에 운영 또는 PBVS 시험에서는 사용하지 않습니다.

체크리스트:

- [ ] 시험 목적에 필요한 최소 스위치만 활성화
- [ ] 로봇 PC `cartesian_execution_enabled`는 실기 시험 중에만 활성화
- [ ] PBVS 접근 시험 중 `insertion_enabled=false` 유지
- [ ] 새 실행 기능은 기본값 `false`로 추가
- [ ] 실행 종료 후 안전 스위치를 다시 `false`로 복원

### 12.2 TCP 안전 조건

```yaml
tool:
  calibrated: false
```

`require_calibrated_tool=true`일 때 TCP가 미보정이면 plug tip 접근과 삽입 명령을
거부해야 합니다. 카메라 중심 PBVS 시험과 실제 plug tip 정렬을 구분합니다.

- [ ] 충전기와 케이블이 최종 사용 상태로 단단히 고정됨
- [ ] `T_flange_plug_tip` 측정 방법과 날짜 기록
- [ ] translation 단위와 quaternion 방향 확인
- [ ] 여러 자세에서 plug tip 예측 위치 검증
- [ ] 재장착 반복 편차 확인
- [ ] 검증 완료 전 `calibrated: false` 유지

### 12.3 검출 안전 조건

이동 명령을 만들기 전에 최소한 다음을 모두 확인합니다.

- [ ] 네 keypoint가 모두 존재하고 index가 올바름
- [ ] keypoint 좌표가 유한하며 원본 영상 범위 안에 있음
- [ ] 객체 confidence와 keypoint confidence가 기준 이상
- [ ] 포트 깊이가 허용 범위 안에 있음
- [ ] solvePnP 재투영 오차가 기준 이하
- [ ] 영상과 CameraInfo의 frame·해상도가 일치
- [ ] 검출 timestamp가 최근 값이며 해당 시각 TF를 조회할 수 있음
- [ ] 여러 프레임의 위치·자세 분산이 허용범위 안에 있음
- [ ] 180도 pose 뒤집힘이나 순간적인 keypoint 순서 오류가 없음

한 프레임의 YOLO 결과만으로 이동하지 않습니다. 검출이 끊기거나 오래되면 마지막
오차를 계속 적용하지 않고 새 이동 명령을 차단합니다.

### 12.4 이동 안전 조건

- [ ] 목표 frame이 `g_base`인가?
- [ ] 목표가 설정된 작업영역 안인가?
- [ ] 현재 pose와 ROS TF 및 `get_coords()`가 합리적으로 일치하는가?
- [ ] 한 번의 Cartesian translation과 rotation이 제한 이하인가?
- [ ] 목표 Z와 자세가 현재 단계의 lock 정책에 맞는가?
- [ ] MoveIt IK와 충돌 사전검사를 통과했는가?
- [ ] 인접 IK 해의 관절 변화량이 제한 이하인가?
- [ ] 다른 action이나 이동 명령이 실행 중이지 않은가?
- [ ] 명령과 관측 timestamp가 timeout을 넘지 않았는가?
- [ ] `/dev/ttyUSB0`를 브리지 한 프로세스만 소유하는가?
- [ ] 실행 중 실제 TF와 `get_coords()`를 감시하는가?
- [ ] cancel과 timeout에서 `stop()`이 동작하는가?
- [ ] 작업자가 비상정지에 즉시 접근할 수 있는가?

목표 endpoint의 Z/Roll/Pitch가 같아도 중간 경로가 같다는 뜻은 아닙니다. 처음
검증하는 기능은 장애물이 없는 공간에서 1 mm 또는 0.5도부터 시작하고, 반복
검증을 통과한 뒤 5 mm와 10 mm로 확대합니다.

### 12.5 삽입 안전 조건

삽입은 접근보다 별도의 승인이 필요합니다.

- [ ] TCP 보정과 plug tip 기준 PBVS 완료
- [ ] IBVS를 쓴다면 teach pose와 desired feature 저장 완료
- [ ] IBVS를 쓰지 않는다면 PBVS 반복 오차와 기계적 허용공차 검증 완료
- [ ] 포트 중심과 Yaw 오차가 삽입 허용범위 이하
- [ ] 최종 Roll/Pitch/Yaw를 `q_insert_lock`으로 저장
- [ ] 실제 삽입축 방향 확인
- [ ] 단계당 전진 거리와 최대 전체 거리 설정
- [ ] 저속 제한과 timeout 설정
- [ ] 전류·힘·접촉 또는 연결 신호 중 가능한 피드백 존재
- [ ] 걸림·검출 소실·연결 실패 시 즉시 정지 가능
- [ ] 삽입축 반대 방향 후퇴 경로 검증
- [ ] 재시도 횟수 상한 설정

현재 패키지는 위 조건을 충족하지 않으며 실제 삽입 백엔드도 완성되지 않았습니다.
따라서 `insertion_enabled=false`를 유지합니다.

### 12.6 설정 변경 전 기록해야 할 결과

안전 스위치를 `true`로 바꾸기 전에 다음을 문서화합니다.

1. 좌표 방향과 반복성 측정 결과
2. TCP 측정 방법·값·반복 편차
3. 로봇 작업영역과 충돌 제한
4. 최소 속도 및 최소 이동량 시험 결과
5. Cartesian cancel·timeout·이탈 정지 시험 결과
6. 검출 소실 시 정지 시험 결과
7. 후퇴 경로 시험 결과
8. 비상정지 시험 결과
9. 실제 사용 Git 커밋과 YAML 버전

## 13. 항상 확인할 고려사항

### 좌표계와 단위

- 모든 변환 방향을 `T_A_B`로 기록
- ROS translation은 meter, MyCobot 좌표는 mm
- ROS rotation은 quaternion, MyCobot은 degree RPY
- `g_base`, `joint6_flange`, `camera_optical_frame`, `usb_port`, `plug_tip` 혼동 금지
- optical frame은 X 오른쪽, Y 아래, Z 카메라 전방

### 시간과 통신

- 노트북과 로봇 PC clock 동기화
- 영상 header timestamp를 YOLO 출력까지 유지
- YOLO 운용에서 임의의 최신 TF로 과거 영상을 변환하지 않기
- ROS domain과 discovery 설정 일치
- `/dev/ttyUSB0`는 항상 한 프로세스만 소유

### 시야와 기구

- Hand-in-eye 카메라가 근접할수록 포트가 가려질 가능성 평가
- 플러그 또는 손목이 keypoint를 가리는 거리 기록
- 마지막 비전 가능 거리 이후에는 이전 오차를 무제한 재사용하지 않기
- 케이블 장력과 플러그 재장착 오차 기록
- 차량 진동과 조명 변화 포함

### 추가 실행 고려사항

- endpoint 고정과 경로 전체 고정은 다르다는 점을 유지
- 실제 TF와 `get_coords()`를 실행 중 감시
- 비상정지에 즉시 접근 가능한 상태에서만 실기 시험
- 처음 검증하는 기능은 1 mm 또는 0.5도부터 시작
- 삽입 테스트 전 TCP, 후퇴, 성공 신호를 먼저 검증

## 14. 시험 기록 양식

각 실기 시험에서 다음을 기록합니다.

```text
날짜/커밋:
로봇 PC pymycobot 버전:
카메라 해상도:
모델 파일과 데이터셋 버전:
초기 flange pose:
목표 delta X/Y/Yaw:
실제 delta X/Y/Z/Roll/Pitch/Yaw:
YOLO confidence:
solvePnP 재투영 오차:
이동 시간:
정지 또는 거부 이유:
연결 성공 여부:
비고:
```

코드 변경과 캘리브레이션 값 변경을 같은 기록 없이 섞지 않습니다. 실제 실행에
사용한 Git 커밋과 YAML 파일을 반드시 함께 남깁니다.
