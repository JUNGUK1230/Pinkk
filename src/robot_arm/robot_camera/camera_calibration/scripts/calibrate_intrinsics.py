import argparse
from pathlib import Path

import cv2
import numpy as np


IMAGE_EXTENSIONS = {".jpg", ".jpeg", ".png", ".bmp"}


def parse_args():
    parser = argparse.ArgumentParser(description="Calibrate camera intrinsics with a checkerboard.")
    parser.add_argument("--images", default="camera_calibration/images/raw", help="Input image folder.")
    parser.add_argument("--cols", type=int, default=9, help="Number of inner corners per row.")
    parser.add_argument("--rows", type=int, default=6, help="Number of inner corners per column.")
    parser.add_argument("--square-size", type=float, default=25.0, help="Checkerboard square size in mm.")
    parser.add_argument("--output", default="camera_calibration/results", help="Output folder.")
    parser.add_argument("--show", action="store_true", help="Show detected corners.")
    return parser.parse_args()


def list_images(folder):
    image_dir = Path(folder)
    return sorted(path for path in image_dir.iterdir() if path.suffix.lower() in IMAGE_EXTENSIONS)


def build_object_points(cols, rows, square_size):
    objp = np.zeros((cols * rows, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    objp *= square_size
    return objp


def save_yaml(path, camera_matrix, dist_coeffs, image_size, rms, valid_count):
    fs = cv2.FileStorage(str(path), cv2.FILE_STORAGE_WRITE)
    fs.write("camera_matrix", camera_matrix)
    fs.write("dist_coeffs", dist_coeffs)
    fs.write("image_width", int(image_size[0]))
    fs.write("image_height", int(image_size[1]))
    fs.write("rms_reprojection_error", float(rms))
    fs.write("valid_image_count", int(valid_count))
    fs.release()


def main():
    args = parse_args()
    pattern_size = (args.cols, args.rows)
    output_dir = Path(args.output)
    output_dir.mkdir(parents=True, exist_ok=True)

    image_paths = list_images(args.images)
    if not image_paths:
        raise RuntimeError(f"No calibration images found in {args.images}")

    objp = build_object_points(args.cols, args.rows, args.square_size)
    objpoints = []
    imgpoints = []
    image_size = None
    accepted = []
    rejected = []

    criteria = (
        cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
        30,
        0.001,
    )

    for image_path in image_paths:
        image = cv2.imread(str(image_path))
        if image is None:
            rejected.append((image_path.name, "read_failed"))
            continue

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        image_size = gray.shape[::-1]
        found, corners = cv2.findChessboardCorners(gray, pattern_size, None)

        if not found:
            rejected.append((image_path.name, "corners_not_found"))
            continue

        corners_subpix = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        objpoints.append(objp)
        imgpoints.append(corners_subpix)
        accepted.append(image_path.name)

        if args.show:
            preview = image.copy()
            cv2.drawChessboardCorners(preview, pattern_size, corners_subpix, found)
            cv2.imshow("detected_corners", preview)
            cv2.waitKey(200)

    if args.show:
        cv2.destroyAllWindows()

    if len(objpoints) < 10:
        raise RuntimeError(f"Only {len(objpoints)} valid images. Capture at least 10, preferably 20 to 40.")

    rms, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        objpoints,
        imgpoints,
        image_size,
        None,
        None,
    )

    npz_path = output_dir / "intrinsics.npz"
    yaml_path = output_dir / "intrinsics.yaml"
    report_path = output_dir / "calibration_report.txt"

    np.savez(
        npz_path,
        camera_matrix=camera_matrix,
        dist_coeffs=dist_coeffs,
        image_width=image_size[0],
        image_height=image_size[1],
        rms_reprojection_error=rms,
        rvecs=np.array(rvecs, dtype=object),
        tvecs=np.array(tvecs, dtype=object),
    )
    save_yaml(yaml_path, camera_matrix, dist_coeffs, image_size, rms, len(accepted))

    report_lines = [
        f"RMS reprojection error: {rms:.6f}",
        f"Image size: {image_size[0]} x {image_size[1]}",
        f"Valid images: {len(accepted)}",
        f"Rejected images: {len(rejected)}",
        "",
        "Camera matrix:",
        str(camera_matrix),
        "",
        "Distortion coefficients:",
        str(dist_coeffs),
        "",
        "Accepted files:",
        *accepted,
        "",
        "Rejected files:",
        *(f"{name}: {reason}" for name, reason in rejected),
    ]
    report_path.write_text("\n".join(report_lines), encoding="utf-8")

    print(f"Saved {npz_path}")
    print(f"Saved {yaml_path}")
    print(f"Saved {report_path}")
    print(f"RMS reprojection error: {rms:.6f}")


if __name__ == "__main__":
    main()
