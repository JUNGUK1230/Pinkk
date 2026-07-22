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
