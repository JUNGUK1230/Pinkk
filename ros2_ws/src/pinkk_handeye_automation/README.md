# Pinkk Hand-eye 자동 캘리브레이션

이 패키지는 현재 `g_base -> joint6_flange` 자세를 홈으로 저장한 뒤, 플랜지 위치를
유지하면서 local X/Y/Z 회전을 조합한 관측 자세 15개를 자동으로 생성합니다.

각 자세에서 다음 작업을 자동으로 수행합니다.

```text
MoveIt IK 계산
→ FollowJointTrajectory 이동
→ 정지 대기
→ 최신 camera_optical_frame -> charuco_board TF 확인
→ Easy Handeye2 TakeSample 호출
```

ChArUco 노드는 코너 35개 이상, 재투영 오차 0.7 px 이하일 때만 TF를 발행하므로
최신 TF 검사는 검출 품질 조건을 함께 확인합니다. 전체 자세가 끝나면 홈으로
복귀하고, 유효 샘플이 10개 이상이면 계산 및 저장까지 수행합니다.

요청한 회전 자세의 IK가 실패하면 동일한 회전 방향을 유지하면서 각도를
`75%`, `50%`, `35%`로 줄여 자동 재시도합니다.

먼저 이동 없는 IK 점검을 실행합니다.

```bash
ros2 launch pinkk_handeye_automation auto_calibrate.launch.py
```

15개 자세의 IK가 충분히 성공하면 실제 자동 수집을 실행합니다.

```bash
ros2 launch pinkk_handeye_automation auto_calibrate.launch.py execute:=true
```
