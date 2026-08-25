import argparse

import cv2


def camera_arg(value):
    if value.isdigit():
        return int(value)
    return value


def parse_args():
    parser = argparse.ArgumentParser(description="Test OpenCV camera connection.")
    parser.add_argument("--camera", type=camera_arg, default=0, help="Camera index or device path.")
    parser.add_argument("--scan", action="store_true", help="Scan camera indices from 0 to 10.")
    parser.add_argument("--width", type=int, default=0, help="Optional capture width.")
    parser.add_argument("--height", type=int, default=0, help="Optional capture height.")
    return parser.parse_args()


def open_camera(camera, width, height):
    cap = cv2.VideoCapture(camera)
    if width > 0:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    if height > 0:
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    return cap


def scan_cameras(width, height):
    print("Scanning camera indices 0 to 10...")
    found = []
    for index in range(11):
        cap = open_camera(index, width, height)
        if cap.isOpened():
            ok, _ = cap.read()
            actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            actual_fps = cap.get(cv2.CAP_PROP_FPS)
            status = "read ok" if ok else "opened, read failed"
            print(f"[{index}] {status} | {actual_width}x{actual_height} | fps {actual_fps:.2f}")
            found.append(index)
        cap.release()

    if not found:
        print("No OpenCV camera index found.")
    else:
        print(f"Available camera indices: {found}")


def main():
    args = parse_args()

    if args.scan:
        scan_cameras(args.width, args.height)
        return

    cap = open_camera(args.camera, args.width, args.height)

    if not cap.isOpened():
        raise RuntimeError(f"OpenCV could not open camera {args.camera}")

    actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    actual_fps = cap.get(cv2.CAP_PROP_FPS)

    print("OpenCV camera connected.")
    print(f"camera: {args.camera}")
    print(f"resolution: {actual_width} x {actual_height}")
    print(f"fps: {actual_fps:.2f}")
    print("Press 'q' to quit.")

    while True:
        ok, frame = cap.read()
        if not ok:
            print("Failed to read frame.")
            break

        cv2.putText(
            frame,
            f"camera {args.camera} | {actual_width}x{actual_height} | q: quit",
            (20, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )
        cv2.imshow("opencv_camera_test", frame)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
