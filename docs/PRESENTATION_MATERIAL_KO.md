# PINKK 스마트 주차 발표 자료

작성 기준: 2026-08-26 현재 저장소와 회귀 검사 결과  
권장 발표 시간: 7~10분  
발표 범위: 중앙 관제, 상단 카메라 인식, 다중 차량 식별, 경로 계획, 위치 융합, MPC 주행, 웹 관제

## 발표 한 줄 요약

상단 카메라로 주차장 전체를 관측하고 차량별 LiDAR·odometry를 결합해 두 대의
차량을 구분한 뒤, 검증된 경로와 MPC로 입차·주차·출차를 수행하는 ROS 2 기반
스마트 주차 시스템이다.

## 핵심 수치

| 항목 | 현재 값 |
|---|---:|
| 운영 차량 | 2대 (`vehicle_1`, `vehicle_2`) |
| 상단 카메라 BEV | 1600 × 800 px |
| 주차·충전 위치 | 일반 주차 8면 + 충전 2면 |
| 검증된 mission 전이 | 32개 |
| 고정 경로 샘플 간격 | 0.5 cm |
| trajectory 필드 | `x`, `y`, `yaw`, `direction` 4개 |
| 개발 기간 기록 | 2026-07-09 ~ 2026-08-26 |
| 중앙 관제·차량 제어 관련 커밋 | 82개 |

> 현재 manifest와 회귀 검사는 32개 경로를 기준으로 한다. 과거 문서에 남은
> 26개라는 수치는 출차 직행 경로가 추가되기 전 기준이다.

## 슬라이드 1. 프로젝트 소개

### 화면에 넣을 내용

**PINKK: 상단 카메라와 LiDAR 기반 다중 차량 스마트 주차 시스템**

- 주차장 전체를 한눈에 보는 중앙 관제
- 두 대의 차량을 구분해 각자 경로 배정
- 입차 → 일반 주차 → 충전 → 재주차 → 출차 흐름 자동화

### 발표 멘트

“PINKK는 한 대의 상단 카메라로 주차장 전체를 관측하고, 차량별 LiDAR와
odometry를 결합해 두 대의 차량을 구분합니다. 이후 빈 주차면과 운행 단계에
맞는 경로를 선택하고 MPC로 실제 차량을 움직이는 스마트 주차 시스템입니다.”

### 추천 화면

![주차장 Camera BEV](../src/central_control/camera_tools/first_map/camera_bev.png)

## 슬라이드 2. 해결하려는 문제

### 화면에 넣을 내용

1. 제한된 센서로 주차장 전체와 여러 차량을 어떻게 파악할 것인가?
2. 좁은 주차 공간에서 전진·후진이 섞인 경로를 어떻게 안정적으로 주행할 것인가?
3. 인식 지연, 통신 단절, 장애물 상황에서 어떻게 안전하게 멈출 것인가?

### 발표 멘트

“단순히 차량 한 대를 움직이는 문제가 아니었습니다. 전체 공간을 보는 카메라의
장점과 각 차량 센서의 정확도를 결합해야 했고, 카메라의 임시 track ID를 실제
차량 ID로 연결해야 했습니다. 또한 좁은 공간의 후진 주차와 통신·센서 이상 시
정지까지 하나의 시스템으로 묶어야 했습니다.”

## 슬라이드 3. 전체 시스템 구조

### 화면에 넣을 내용

```text
상단 카메라
  ↓ 왜곡 보정 / Homography BEV
YOLO Segmentation + ByteTrack
  ↓ 차량 중심·방향·주차면 점유
차량별 LiDAR scan matching ──→ 차량 ID 확정
  ↓
운행 단계 판단 + 32개 검증 경로 중 선택 ← 웹 입·출차 요청
  ↓ trajectory
카메라 위치 + wheel odometry + LiDAR heading 융합
  ↓ fused pose
차동구동 MPC
  ↓ cmd_vel
Pinky 차량
```

### 발표 멘트

“중앙 PC는 인식과 경로 배정을 담당하고, 차량별 제어기는 융합된 자세와 경로를
받아 속도 명령을 계산합니다. 차량마다 ROS namespace를 분리했기 때문에 같은
구조를 두 대에 독립적으로 적용할 수 있습니다.”

## 슬라이드 4. 인식과 다중 차량 식별

### 화면에 넣을 내용

- 카메라 보정 후 1600×800 Bird's-Eye View 생성
- YOLO segmentation으로 차량 형상과 주차면 겹침 계산
- ByteTrack으로 프레임 간 차량 추적
- `/pinkk/vehicle_1/scan`, `/pinkk/vehicle_2/scan`을 각각 구독
- namespace별 LiDAR 위치와 카메라 track 위치를 비교
- 위치가 가장 잘 맞는 `track_id ↔ vehicle_id`를 연결
- 연결된 차량 namespace로 `localization_pose`, `path`, `trajectory` 발행
- 확신이 부족하면 경로를 발행하지 않는 fail-closed 정책

```text
/pinkk/vehicle_1/scan → LiDAR 지도 위치 ─┐
                                         ├→ 카메라 track 위치와 비교
/pinkk/vehicle_2/scan → LiDAR 지도 위치 ─┘

vehicle_1 위치 ≈ track_7 위치
→ vehicle_1 ↔ track_7 확정
→ /pinkk/vehicle_1/localization_pose
→ /pinkk/vehicle_1/path
→ /pinkk/vehicle_1/trajectory
```

### 발표 멘트

“ByteTrack ID는 화면에서 잠시 사라지면 바뀔 수 있어 영구적인 차량 ID로 쓸 수
없습니다. 반면 LiDAR scan은 차량별 namespace 토픽으로 들어오므로 어느 차량의
scan인지 알고 있습니다. 각 namespace의 scan을 지도에 정합한 위치와 카메라
track 위치를 비교해, 위치가 맞는 track을 해당 실제 차량으로 연결합니다. 연결
후 카메라 pose와 경로는 같은 차량 namespace로 발행합니다. 1·2순위 점수 차이와
연속 확인 조건까지 통과해야 확정하며, 확신이 부족하면 발행하지 않습니다. 현재
실차 동작 시험 기본값은 수동 연결 모드이며 자동 매칭도 선택할 수 있습니다.”

## 슬라이드 5. 경로 계획의 발전 과정

### 화면에 넣을 내용

```text
클릭 기반 A*
  → 차량 방향을 고려한 Hybrid A*
  → Reeds–Shepp + footprint 충돌 검사
  → smoothing + 속도 profile + trajectory validator
  → 실시간 운영은 사전 생성·검증한 고정 mission route 사용
```

- 연구·생성 도구: Hybrid A* 계열
- 실제 운영: 32개 고정 경로를 선택해 즉시 발행
- 전진·후진 방향과 cusp 정지를 경로에 명시

### 현재 고정 경로 생성 방식

```text
실측한 endpoint·staging·도로 기준점을 YAML에 정의
→ 공통 도로를 직선과 접선 원호로 생성
→ 특수 합류 구간에 5차 Bezier 또는 방향 제한 Reeds–Shepp 적용
→ 주차면별 전진 접근·후진 maneuver 연결
→ source exit + 공통 도로 + target entry 조립
→ footprint 충돌·곡률·최종 직선 후진 검사
→ rear-axle 경로를 차량 중심 경로로 변환
→ 0.5cm 간격 CSV 저장
```

- P5~P8: 최소 14cm 반경의 단일 연속 후진 진입
- P1~P4: 최소 20cm 반경의 단일 연속 후진 진입
- C1·C2: 후진→전진 정렬→44cm 직선 후진의 3-point 진입
- 현재 고정 CSV는 Hybrid A* 탐색 결과를 그대로 저장한 것이 아님

### 발표 멘트

“초기에는 온라인 A*와 Hybrid A*로 매번 경로를 탐색했습니다. 하지만 실차에서는
계산 성공 여부보다 매번 같은 안전한 궤적을 재현하는 것이 더 중요했습니다.
현재 고정 경로는 실제 도로 기준점과 주차 pose를 YAML에 정의하고, 직선·접선
원호·Bezier·방향 제한 Reeds–Shepp과 주차면별 후진 maneuver를 조합해 만듭니다.
차량 footprint와 곡률을 검사한 뒤 0.5cm 간격 CSV로 저장하며, 운영 중에는
검증된 32개 CSV 중 하나를 선택합니다. Hybrid A*는 형상 실험과 진단용으로
남겨 두었습니다.”

### 추천 화면

![충전면 진입 경로](../src/central_control/path_planning/output/c1_c2_camera_bev_route_comparison.png)

빨간색은 전진, 자홍색은 후진, 노란색은 방향 전환 지점이다.

## 슬라이드 6. 위치 융합과 MPC 제어

### 화면에 넣을 내용

**Pose fusion**

- 상단 카메라: 절대 x/y
- wheel odometry: 프레임 사이 상대 이동과 yaw
- LiDAR map matching: 출발 전 절대 heading 보정
- 카메라 지연 시간 동안의 이동량을 odometry로 보상

**MPC**

- signed speed와 curvature를 SLSQP로 최적화
- 전진/후진 gear block과 cusp 정지 관리
- 속도·각속도·가속도·곡률·휠 속도 제한
- 같은 경로 재발행 시 진행 위치와 warm start 보존

### 발표 멘트

“카메라는 절대 위치에 강하지만 지연이 있고, odometry는 빠르지만 누적 오차가
생깁니다. 둘을 결합하고 LiDAR heading으로 초기 방향을 보정했습니다. 출발한 뒤
LiDAR 보정이 갑자기 방향을 바꾸지 않도록 heading correction을 잠그고, MPC는
현재 기어 구간 안에서만 최근접점을 찾아 전진과 후진이 섞인 경로를 추종합니다.”

## 슬라이드 7. 관제 웹과 안전 구조

### 화면에 넣을 내용

- 관리자 웹: 영상, 차량 상태, 배터리, 경로 요청, 긴급정지
- 사용자 웹: 입차·출차 요청, 배터리, 순수 BEV 영상
- 차량별 ROS namespace와 긴급정지 service
- 출차 요청은 자동 주차·충전 배정보다 우선

**안전정지 조건**

- pose/path/scan timeout
- 진행 방향 장애물
- 비정상 trajectory 또는 MPC solver 실패
- 큰 heading 오차
- 예상하지 않은 `cmd_vel` publisher 충돌

### 발표 멘트

“웹은 차량을 직접 조종하지 않고 중앙 경로 요청만 보냅니다. 실제 속도 명령은
MPC 한 곳에서만 생성하며, 긴급정지는 별도 service로 0속도를 latch합니다. 센서가
오래되거나 경로가 유효하지 않으면 움직이는 대신 정지하는 구조입니다.”

## 슬라이드 8. 개발 중 핵심 문제와 해결

| 실제 문제 | 적용한 해결 |
|---|---|
| 카메라 track ID가 일시적이라 두 차량이 바뀔 수 있음 | 차량별 LiDAR scan-map 비용으로 영속 ID 연결, 연속 확인, 수동 fallback |
| Camera BEV와 LiDAR map의 좌표·방향이 다름 | homography와 rigid registration을 분리하고 최종 frame을 `lidar_map`으로 통일 |
| 카메라 지연 때문에 현재 위치보다 뒤를 보고 제어함 | 촬영 이후 odometry 이동량으로 x/y 지연 보상 |
| 주행 중 LiDAR heading 보정이 갑자기 경로 방향을 흔듦 | 경로 시작 후 heading correction lock |
| 좁은 주차면에서 차체가 벽·구획에 걸림 | 회전 footprint 검사, 전용 staging, 회전반경, 단일 연속 후진 진입 적용 |
| 온라인 탐색의 시간과 결과가 일정하지 않음 | 사전 생성·검증 경로를 운영에 사용하고 온라인 planner는 진단용으로 분리 |
| 경로 재발행 때 MPC 진행점이 초기화됨 | 동일 경로 판별과 progress/warm-start 보존 |
| 오래된 경로나 통신 단절에도 움직일 위험 | `path_valid` 무효화와 timeout 기반 fail-closed 정지 |

### 발표 멘트

“가장 큰 교훈은 알고리즘 하나의 성능보다 좌표계, 시간 지연, 차량 ID, 기구학,
안전 상태를 함께 맞춰야 실차가 움직인다는 점이었습니다. 문제를 센서·계획·제어
단계로 분리해 로그와 회귀 검사로 재현하면서 해결했습니다.”

## 슬라이드 9. 현재 성과와 검증 상태

### 화면에 넣을 내용

**구현·검증 완료**

- 상단 영상 보정, 차량 segmentation/tracking, 주차면 점유 판정
- 두 차량 namespace 분리와 LiDAR-camera 자동 식별 로직
- 입차·충전·재주차·출차를 포함한 32개 고정 mission route
- 위치·heading 융합, 전후진 MPC, 웹 요청과 긴급정지
- 1차 실차 통합 시험 및 운영 로그 수집

**2026-08-26 회귀 검사**

- 32개 고정 경로 validator 통과
- 고정 경로 selector 통과
- live route bridge와 trajectory 4필드 검사 통과
- heading fusion 검사 통과
- MPC 전체 회귀: 전진 재합류 1개 시나리오 미통과

### 발표 멘트

“현재 시스템의 주요 파이프라인은 하나로 연결되어 있고, 32개 경로와 selector,
bridge, heading 융합 검사는 통과했습니다. 다만 MPC 전체 회귀에서는 직선에서
2cm 벗어난 뒤 재합류할 때 중심선을 약 6.2mm 넘는 한 항목이 남아 있습니다.
따라서 완성이라고 과장하기보다, 통합은 완료했고 제어 정밀도 튜닝이 남은
상태라고 설명하는 것이 정확합니다.”

### 추천 화면

![C1에서 P1~P4로 이동하는 경로](../src/central_control/path_planning/output/p1_p4_far_reverse_start_preview.png)

## 슬라이드 10. 한계와 다음 단계

### 화면에 넣을 내용

1. MPC 전진 재합류 overshoot 보정 및 실차 재검증
2. 자동 LiDAR-camera 차량 식별을 기본 운영 모드로 전환
3. 고정 지도·보정 파일의 현장 변경 대응 자동화
4. 동적 장애물을 비상정지뿐 아니라 경로·MPC에 반영
5. 중앙 `pause` 중재와 management event 흐름 완성
6. Flask 시험 서버를 실제 운영용 배포 구조로 교체

### 마무리 멘트

“PINKK는 인식, 계획, 제어, 웹을 개별 기능으로 만든 데서 끝나지 않고 두 대의
차량이 함께 동작하는 통합 주차 흐름으로 연결했습니다. 다음 단계는 남은 MPC
정밀도 문제와 자동 차량 식별의 현장 신뢰도를 높여 반복 가능한 완전 자율
시연으로 만드는 것입니다.”

## 개발 타임라인 한 장 요약

| 시기 | 주요 진행 |
|---|---|
| 7/09~7/13 | 프로젝트 구조, 지도·BEV, 기본 A*와 경로 시각화 |
| 7/14~7/21 | 차량 방향·footprint를 고려한 Hybrid A*, Reeds–Shepp, smoothing, validator |
| 7/22~7/24 | 후진 주차, 자동 빈자리 배정, YOLO+ByteTrack, ROS 직접 발행, 웹 초안 |
| 7/27~7/31 | 고정 경로 운영 전환, 곡률 안전성, mission 단계 관리, 전후진 MPC |
| 8/05~8/14 | 실제 지도·충전면 보정, 점유 차단, 차량 namespace와 곡선 추종 개선 |
| 8/22 | LiDAR-camera 차량 연결, confidence 분리, heading 보정 lock, 주차 진입 개선 |
| 8/24~8/26 | 전체 파이프라인 연결, 출차 직행 경로, 사용자 BEV, MPC 시각화, 1차 시험 |

## 30초 소개문

“저희는 상단 카메라와 차량별 LiDAR를 이용한 ROS 2 기반 스마트 주차 시스템을
개발했습니다. 상단 영상에서 YOLO와 ByteTrack으로 차량과 빈자리를 찾고,
LaserScan 정합으로 두 차량의 신원을 구분합니다. 이후 32개의 사전 검증 경로 중
운행 단계에 맞는 경로를 선택하고, 카메라·odometry·LiDAR를 융합한 자세를 이용해
MPC가 전진과 후진 주차를 수행합니다. 웹 관제와 긴급정지까지 통합했으며, 현재는
실차 제어 정밀도와 자동 식별 신뢰도를 높이는 단계입니다.”

## 3분 시연 순서

1. 관리자 웹에서 두 차량 상태와 상단 영상 확인
2. BEV에서 주차면 점유와 차량 위치 표시 확인
3. 차량 하나를 선택해 입차 또는 목표 경로 요청
4. 경로 overlay에서 전진·후진 구간 설명
5. 차량의 `fused_pose`와 MPC 시각화 확인
6. 출차 요청이 기존 자동 배정보다 우선하는 장면 시연
7. 마지막에 긴급정지 latch와 0속도 상태 확인

시연 실패에 대비해 다음 세 가지는 녹화해 두는 것이 좋다.

- BEV 차량 검출·점유 판정 화면
- 충전면 또는 일반 주차면 후진 진입
- 웹 출차 요청과 긴급정지 동작

## 예상 질문과 답변

### Q. 왜 차량 자체 카메라가 아니라 상단 카메라를 사용했나요?

주차장 전체, 빈자리, 여러 차량을 한 화면에서 볼 수 있고 소형 차량에 많은 센서를
추가하지 않아도 되기 때문이다. 대신 카메라 가림과 지연 문제는 차량 odometry와
LiDAR를 결합해 보완했다.

### Q. 왜 실시간 Hybrid A*가 아니라 고정 경로인가요?

현재 주차장은 구조가 고정되어 있고 실차 시연에서는 계산 성공률과 결과의 일관성이
더 중요했다. 오프라인에서 footprint·곡률·기어 전환을 검증한 경로를 운영에 쓰면
즉시 발행할 수 있고 회귀 검사도 가능하다. Hybrid A*는 지도 변경 시 경로를 만드는
도구로 유지한다.

### Q. 왜 Nav2를 사용하지 않고 경로 생성기부터 MPC까지 직접 만들었나요?

Nav2를 적용해도 PINKK에 필요한 핵심 기능은 대부분 별도 구현해야 했다.

- 상단 카메라·odom·LiDAR를 결합한 pose를 Nav2 TF 구조로 바꾸는 adapter
- `x, y, yaw, direction`의 전진·후진 정보를 보존하는 planner/controller plugin
- cusp 완전 정지와 gear block 상태 관리
- 12×10cm 차량의 중심 offset, 휠 속도와 곡률 제한을 반영한 custom controller
- 두 차량의 ID 연결, 주차면·충전면 배정과 mission 단계용 중앙 coordinator

즉, 핵심 알고리즘을 모두 직접 만든 뒤 Nav2의 lifecycle·action·behavior tree
계층을 추가하는 형태가 된다. 반면 현재 환경은 약 2.5×2.2m의 고정 주차장으로,
범용 온라인 재탐색보다 32개 경로의 결정성과 0.5cm 간격 충돌 검증이 중요했다.
또한 pose나 차량 ID가 불확실한 상황에서는 자동 recovery보다 정지가 더 안전했다.

발표용 짧은 답변:

> “Nav2를 사용해도 외부 카메라 pose adapter, 전후진 경로 plugin, custom MPC,
> 다중 차량 coordinator를 다시 만들어야 했습니다. 고정된 초소형 주차장에서는
> 그 통합 비용보다 검증된 32개 경로의 결정성과 cm 단위 제어가 더 중요했습니다.
> 그래서 핵심 파이프라인을 직접 연결했고, 동적 환경으로 확장할 때 Nav2 도입을
> 다시 고려할 계획입니다.”

### Q. 두 차량을 어떻게 구분하나요?

카메라 track마다 각 차량의 LaserScan을 지도에 정합한 비용을 비교하고, 가능한
일대일 조합 중 최저 비용 조합을 선택한다. 점수 차이와 연속 확인 기준이 부족하면
차량을 확정하지 않는다. 현재 시험 모드에는 운영자 수동 연결도 제공한다.

### Q. 장애물이 나타나면 피해 가나요?

현재는 진행 방향 LaserScan sector에서 장애물을 감지하면 안전정지한다. 동적
장애물을 우회하도록 MPC나 경로를 온라인 갱신하는 기능은 다음 단계다.

### Q. 현재 완성도는 어느 정도인가요?

인지부터 경로 배정, pose 융합, MPC, 웹까지 통합되었고 32개 경로 검사는
통과했다. 다만 자동 차량 식별은 현재 수동 fallback을 기본으로 시험 중이며,
MPC 전진 재합류 회귀 한 항목과 반복 실차 검증이 남아 있다.

### Q. 안전은 어떻게 보장하나요?

센서 timeout, invalid path, 장애물, solver 실패, 큰 heading 오차, 제어 publisher
충돌 중 하나라도 발생하면 0속도를 내는 fail-closed 구조다. 웹 긴급정지는 차량별
service로 latch되며, 주변 안전을 확인한 뒤 명시적으로 해제해야 한다.

## 발표자가 피해야 할 표현

- “완전 자율주차가 완성됐다” 대신 “전체 파이프라인을 통합했고 반복 실차 검증과
  정밀도 튜닝이 남았다”라고 말한다.
- “동적 장애물을 회피한다” 대신 “현재는 LiDAR 기반 안전정지로 처리한다”라고
  말한다.
- “차량 자동 식별이 항상 된다” 대신 “자동 식별 로직이 구현됐고 현재 실차
  시험에서는 수동 fallback을 기본으로 사용한다”라고 말한다.
- “실시간으로 최적 경로를 탐색한다” 대신 “운영에서는 사전 검증 경로를 즉시
  선택하며 온라인 planner는 생성·진단용이다”라고 말한다.

## 발표용 원본 자료 위치

- 전체 설명: [`README.md`](../README.md)
- 중앙 관제 실행: [`src/central_control/README.md`](../src/central_control/README.md)
- 경로 계획: [`path_planning/README.md`](../src/central_control/path_planning/README.md)
- 차량 제어: [`src/vehicle_control/README.md`](../src/vehicle_control/README.md)
- 경로 설정: [`fixed_mission_routes.yaml`](../src/central_control/path_planning/config/fixed_mission_routes.yaml)
- Camera BEV: [`camera_bev.png`](../src/central_control/camera_tools/first_map/camera_bev.png)
- 충전면 경로 비교: [`c1_c2_camera_bev_route_comparison.png`](../src/central_control/path_planning/output/c1_c2_camera_bev_route_comparison.png)
- 일반 주차 경로 비교: [`p1_p4_far_reverse_start_preview.png`](../src/central_control/path_planning/output/p1_p4_far_reverse_start_preview.png)
