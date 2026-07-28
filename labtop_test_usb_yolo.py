from ultralytics import YOLO
import cv2


# YOLO 모델 경로
model = YOLO(
    "/home/kim-jayeon/Pinkk/ros2_ws/src/pinkk_usb_insertion/models/usb_01.pt"
)


# USB 카메라
cap = cv2.VideoCapture(2)


if not cap.isOpened():
    print("카메라 열기 실패")
    exit()


while True:

    ret, frame = cap.read()

    if not ret:
        print("프레임 읽기 실패")
        break


    # YOLO 추론
    results = model(frame)
    # bbox 출력
    boxes = results[0].boxes

    for box in boxes:
        xyxy = box.xyxy[0].cpu().numpy()
        conf = box.conf[0].cpu().numpy()

        x1, y1, x2, y2 = xyxy

        cx = (x1 + x2) / 2
        cy = (y1 + y2) / 2

        print(
            f"bbox: {xyxy}, center: ({cx:.1f}, {cy:.1f}), conf: {conf:.2f}"
        )

    # keypoint 출력
    if results[0].keypoints is not None:

        kpts = results[0].keypoints.xy

        if len(kpts) > 0:
            print("Keypoints:")
            print(kpts[0])


    # 화면 표시
    annotated = results[0].plot()

    cv2.imshow(
        "USB YOLO",
        annotated
    )


    # ESC 종료
    if cv2.waitKey(1) == 27:
        break


cap.release()
cv2.destroyAllWindows()