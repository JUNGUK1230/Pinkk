import argparse
from datetime import datetime
from pathlib import Path

import cv2


def camera_arg(value):
    if value.isdigit():
        return int(value)
    return value


def parse_args():
    parser = argparse.ArgumentParser(description="Capture checkerboard images.")
    parser.add_argument("--camera", type=camera_arg, default=0, help="Camera index or device path.")
    parser.add_argument("--output", default="camera_calibration/images/raw", help="Output folder.")
    parser.add_argument("--width", type=int, default=0, help="Optional capture width.")
    parser.add_argument("--height", type=int, default=0, help="Optional capture height.")
    return parser.parse_args()


def main():
    args = parse_args()
    output_dir = Path(args.output)
    output_dir.mkdir(parents=True, exist_ok=True)

    cap = cv2.VideoCapture(args.camera)
    if args.width > 0:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    if args.height > 0:
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)

    if not cap.isOpened():
        raise RuntimeError(f"Could not open camera {args.camera}")

    saved_count = 0
    print("Press 's' to save, 'q' to quit.")

    while True:
        ok, frame = cap.read()
        if not ok:
            print("Failed to read frame.")
            break

        preview = frame.copy()
        cv2.putText(
            preview,
            f"saved: {saved_count} | s: save | q: quit",
            (20, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )
        cv2.imshow("capture_checkerboard", preview)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("s"):
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            path = output_dir / f"checkerboard_{timestamp}.jpg"
            cv2.imwrite(str(path), frame)
            saved_count += 1
            print(f"Saved {path}")
        elif key == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
