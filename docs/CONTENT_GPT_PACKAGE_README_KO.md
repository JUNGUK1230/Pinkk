# PINKK 콘텐츠 제작 인계 패키지

이 패키지는 PINKK 스마트 주차 프로젝트의 PPT·발표 대본·보고서 콘텐츠 제작을
다른 GPT에 넘기기 위한 자료다.

## 먼저 읽을 파일

1. `01_DETAILED_PROJECT_BRIEF.md`: 프로젝트의 상세 사실과 구현·검증 상태
2. `02_PROMPT_FOR_CONTENT_GPT.md`: 콘텐츠 GPT에 그대로 전달할 요청문
3. `03_PPT_ONLY_CONTENT.md`: 코드 없이 정리한 현재 PPT 핵심 문구

## 폴더 구성

- `assets`: PPT에 사용할 실제 주차장과 경로 이미지
- `references/docs`: 저장소의 원본 설명 문서
- `references/config`: 수치와 동작 기준의 근거 설정
- `references/source`: 차량 식별, 경로 생성, pose 융합, MPC 핵심 구현

## 발표 범위

- 상단 카메라 차량 검출과 주차면 점유
- LiDAR-camera 다중 차량 식별
- mission 배정과 32개 고정 경로
- pose 융합과 전진·후진 MPC
- 웹 관제와 안전정지

로봇팔과 USB 삽입 내용은 포함하지 않는다.

## 중요한 현재 상태

- 32개 고정 경로, 경로 선택, 경로 전달, heading 융합 검사는 통과했다.
- 자동 차량 식별 로직은 구현됐지만 현재 실차 시험 기본값은 수동 연결이다.
- MPC 전체 회귀는 전진 재합류 시나리오 한 항목이 남아 있다.
- 동적 장애물은 온라인 우회하지 않고 LiDAR 안전정지로 처리한다.
- 현재 고정 경로는 Hybrid A* 결과를 그대로 저장한 것이 아니다.

자료의 내용이 서로 다르면 기준일이 가장 최신인 `01_DETAILED_PROJECT_BRIEF.md`와
현재 설정·소스 파일을 우선한다.
