# 경로계획 중간 메모

## 2026-07-21: 좁은 주차면 충돌 조건 완화

- Hybrid A* 차량 크기: 길이 12cm, 폭 10cm, rear-axle 기준
- 추가 obstacle inflation: 2cm에서 0cm로 변경
- 실제 회전 차량 footprint와 원본 검은색 벽의 충돌 검사는 유지
- 고정 12cm 직선 후진 approach 조건 제거
- 고정거리 관련 설정과 live pipeline 실행 코드도 완전히 제거
- 최종 목표 pose까지 Hybrid A*가 전진·후진 조합을 직접 탐색
- 자동 주차의 마지막 이동 방향은 후진(`-1`)으로 유지
- 주차칸 이름에 따른 분기 없이 모든 칸에 같은 차량·지도·기구학 조건 적용

회귀 결과:

- C1: 성공, 약 0.56초
- P6: 성공, 약 0.16초
- P10: 성공, 약 0.21초
- P8/P9: 차체 충돌 즉시 거절은 해소됐지만 짧은 후보 timeout 안에서 조향 스무딩 경로 미확정

다음 작업은 P8/P9에서 생성된 Reeds–Shepp 후보가 조향 제한을 넘는 원인을 확인하고, 안전 충돌 검사를 유지한 채 smoothing 또는 goal heading을 조정하는 것이다.

## 2026-07-21: 자동 계획 시간 제한 해제

- live 자동 계획 전체 시간 제한: 30초에서 무제한(`0`)으로 변경
- goal 후보별 시간 제한: 5초에서 무제한(`0`)으로 변경
- 탐색 진행 로그는 1,000 expanded node마다 계속 출력
- 메모리·상태공간 폭증 보호용 `max_expanded_nodes: 50000`은 유지

## 2026-07-22: 실시간 목표 헤딩 조건 제거

- live planner는 primary/alternative rear-axle goal의 중점인 주차면 중심만 목표로 사용
- 목표 yaw 일치 조건과 heading 축 기준 주변 goal 반복 탐색을 제거
- Hybrid A* 내부의 시작 yaw, 조향, 차체 footprint 충돌 검사는 유지
- 마지막 이동 방향은 후진(`direction=-1`)으로 유지
- P5 heading-free 단일 목표 탐색은 50,000 node까지 확장했지만 해를 찾지 못했다. 목표 헤딩 제거만으로 전역 Hybrid A* 연산량 문제가 해결되지는 않는다.

## 2026-07-22: 공통 후진주차 입구 규칙 적용

- heading-free 목표 중심 방식은 후진 Reeds-Shepp 연결을 사용할 수 없어 자동 live planner에서 대체
- 각 주차칸 polygon의 짧은 두 변을 입구 후보로 생성
- occupancy map에서 바깥쪽 통로 여유가 더 큰 변을 입구로 자동 선택
- 차량 앞은 입구를 향하고 rear axle은 주차칸 안쪽에 배치
- 주차칸 이름별 조건문 없이 P1~P10, C1~C2 모두 footprint-valid goal을 생성하는 회귀 확인
- P5는 하나의 입구 heading으로 축소됐지만 전역 Hybrid A*가 8초/약 3만 node 안에 해를 찾지 못했다. 속도 문제는 별도의 전역 탐색 구조 개선이 필요하다.

## 2026-07-22: 입구 기준 첫 주차칸 자동 배정

- `config/map/parking_points.json`에 입구·출구의 BEV 좌표를 분리해 저장했다.
- 현재 활성 단계 `entry_to_parking`은 입구 기준으로 P6~P10의 빈 칸을 먼 순서로 정렬하고, 첫 번째 후보만 Hybrid A*에 전달한다.
- 점유된 칸은 후보에서 제외하므로 가장 먼 칸이 차 있으면 다음 거리 순위 칸으로 자동 변경된다.
- 후보 목록은 scene JSON에 함께 저장해 planner와 화면에서 선택 근거를 확인할 수 있다.
- 다음 단계는 `P6~P10 → C1/C2 → P1~P5 → exit` 상태 전환을 별도 episode controller로 구현하는 것이다. 아직 충전소·출구 단계는 선택하지 않는다.

## 2026-07-22: T자 후면주차 planner 분리

- 주차칸 입구 방향을 기준으로 15~30cm 앞 통로에 staging pose를 만든다.
- 첫 Hybrid A*는 현재 pose에서 staging pose까지 접근하고, 두 번째 Hybrid A*는 staging에서 final pose까지 T자 전진·후진 maneuver만 만든다.
- 두 구간 사이에는 연속 stop point를 넣어 정지 상태에서 조향을 바꿀 수 있게 했다.
- staging 목표의 yaw 허용오차도 5°로 제한했다. 이전 15° 허용오차에서는 접근 경로와 maneuver 사이에 yaw jump가 생길 수 있었다.
- P6 기준 회귀에서는 접근 약 200 node와 maneuver 약 80 node로, 마지막 후진 10cm 이상 조건을 통과했다.
- 현장 start에서 통로 접근이 3초 안에 실패하면 후보를 최대 4개까지만 검사하고 fail-closed 한다. 다음 개선은 2D 통로 guide를 Hybrid 접근 탐색의 heuristic으로 직접 사용하는 것이다.

## 2026-07-22: 검증된 trajectory ROS 2 발행

- `overhead_vision/path_planning/path_publisher.py`가 `live_hybrid_path_world_cm.json`을 감시해 ROS 2 토픽으로 발행한다.
- `/pinkk/planned_path`는 m 단위 `nav_msgs/Path`, `/pinkk/planned_trajectory`는 전후진·속도·조향·정지 정보를 포함한 7열 `Float64MultiArray`다.
- planner가 validator를 통과해 저장한 `planner: hybrid_astar` trajectory만 발행한다. `control_ready: false` 또는 validation 정보가 없는 파일은 fail-closed로 거부한다.
- 아직 제어기 subscriber는 연결하지 않았으며, 현재 단계는 토픽 규약·단위·발행 확인이다.

## 2026-07-22: 실시간 차량 pose ROS 2 발행

- `/pinkk/vehicle_pose`에 상단 카메라 localization의 rear-axle `(x_m, y_m, yaw)`를 `PoseStamped`로 발행한다.
- `live_vision_scene.json`의 새 frame만 감시하며, planning-ready가 아니거나 0.5초보다 오래된 scene은 발행하지 않는다.
- 경로 토픽과 현재 pose 토픽 모두 `lidar_map` frame과 m 단위를 사용해 PP/PI 제어기가 직접 비교할 수 있게 맞췄다.

## 2026-07-22: Hybrid A* 조향각 해상도 5° 적용

- 조향 물리 한계는 ±30°로 유지하고, 탐색 후보를 기존 10° 간격 7개에서 5° 간격 13개로 늘렸다.
- T자 주차 maneuver에서 더 완만한 곡선 후보를 찾을 수 있다.
- 3cm motion primitive당 최대 조향 변화 10° 제한은 유지해 조향각 순간 점프를 막는다.
- 후보 수 증가로 탐색량은 커질 수 있으므로 T자 staging 분리와 함께 실행 시간을 계속 확인한다.
