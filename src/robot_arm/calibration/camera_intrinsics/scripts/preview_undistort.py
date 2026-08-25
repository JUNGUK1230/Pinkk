import argparse
from pathlib import Path

import cv2
import numpy as np


def camera_arg(value):
    if value.isdigit():
        return int(value)
    return value


def parse_args():
    parser = argparse.ArgumentParser(description="Preview camera undistortion.")
    parser.add_argument("--calib", default="results/intrinsics.npz", help="Calibration npz file.")
    parser.add_argument("--camera", type=camera_arg, default=0, help="Camera index or device path.")
    parser.add_argument("--width", type=int, default=0, help="Optional capture width.")
    parser.add_argument("--height", type=int, default=0, help="Optional capture height.")
    return parser.parse_args()


def main():
    args = parse_args()
    calib_path = Path(args.calib)
    if not calib_path.exists():
        raise RuntimeError(f"Calibration file not found: {calib_path}")

    data = np.load(calib_path, allow_pickle=True)
    camera_matrix = data["camera_matrix"]
    dist_coeffs = data["dist_coeffs"]

    cap = cv2.VideoCapture(args.camera)
    if args.width > 0:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    if args.height > 0:
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)

    if not cap.isOpened():
        raise RuntimeError(f"Could not open camera {args.camera}")

    print("Press 'q' to quit.")
    while True:
        ok, frame = cap.read()
        if not ok:
            print("Failed to read frame.")
            break

        undistorted = cv2.undistort(frame, camera_matrix, dist_coeffs)
        combined = np.hstack((frame, undistorted))
        cv2.putText(
            combined,
            "left: raw | right: undistorted | q: quit",
            (20, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )
        cv2.imshow("preview_undistort", combined)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
