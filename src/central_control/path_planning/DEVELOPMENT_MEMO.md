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
