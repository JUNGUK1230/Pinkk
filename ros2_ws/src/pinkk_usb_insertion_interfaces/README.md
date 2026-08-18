# Pinkk USB 삽입 인터페이스

YOLO keypoint 검출 노드와 USB pose 계산 노드 사이의 ROS 2 메시지를 정의합니다.

```text
YOLO 노드
→ UsbPortDetectionArray
→ port_pose_node
→ UsbPortObservation
```

메시지에 영상의 `header`, 검출 ID, 객체 신뢰도, 네 keypoint의 좌표·신뢰도,
bounding box와 원본 해상도를 함께 넣어 서로 다른 프레임의 값이 섞이지 않게 합니다.

keypoint index는 다음 물리적 순서를 고정해서 사용합니다.

```text
0 → 1: YOLO 라벨 경계의 긴 변 18 mm
1 → 2: 인접한 짧은 변 12 mm
2 → 3 → 0: 같은 방향으로 나머지 둘레
```

YOLO 데이터셋 라벨 순서와 `port_pose_node`의 3D 모델점 순서가 반드시 같아야
합니다. 이 치수는 USB-A 규격 개구부가 아니라 데이터셋에서 일관되게 선택한
물리적 경계의 실측값입니다.

## Cartesian 이동 action

`CartesianMove.action`은 PBVS 실행기와 로봇 PC 브리지 사이의 승인형 이동
인터페이스입니다.

```text
목표: g_base 기준 flange PoseStamped, speed, mode, Z/Roll/Pitch lock 요청
결과: 성공 여부, 설명, 마지막 실제 pose
피드백: 실제 pose, 위치 오차, 자세 오차
```

로봇 전용 mm/degree 좌표는 메시지에 노출하지 않고 브리지 내부에서 변환합니다.
이 action 정의 자체는 실행 권한을 열지 않으며, PBVS launch의 실행 인자는 기본
`false`입니다.
