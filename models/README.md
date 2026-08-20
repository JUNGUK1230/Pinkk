# YOLO model assets

USB 포트 YOLO Pose weight를 두는 로컬 디렉터리입니다.

현재 표준 파일명은 `usb_02.pt`이며 Git에는 포함하지 않습니다. 다른 모델을
시험할 때는 실행 전에 환경변수로 경로를 지정할 수 있습니다.

```bash
export PINKK_YOLO_MODEL_PATH=/path/to/model.pt
./scripts/run_laptop_frozen_target_test.sh robot_a
```

환경변수가 없으면 실행 스크립트는 저장소 루트 기준
`models/usb_02.pt`를 사용합니다.
