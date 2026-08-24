# Models 파일 구성

| 파일 | 역할 |
|---|---|
| `yolo11l.pt` | YOLO11 Large 기본 pretrained 가중치입니다. 학습 시작점으로 사용합니다. |
| `best.pt` | 현재 실시간 차량 segmentation에 사용하는 custom 학습 가중치입니다. |
| `README.md` | 이 폴더의 모델 파일 역할을 설명합니다. |

실행에 필수인 `best.pt`는 Git에 포함합니다. 그 외 학습용 또는 임시 모델 가중치
확장자 `.pt`, `.pth`, `.onnx`, `.engine`은 Git에서 제외됩니다.
