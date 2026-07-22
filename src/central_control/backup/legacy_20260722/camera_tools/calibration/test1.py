import cv2
import glob
import numpy as np
from pathlib import Path

CHECKERBOARD = (9, 6)

CAMERA_DIR = Path(__file__).resolve().parent
images = glob.glob(str(CAMERA_DIR / "calibration_images" / "*.jpg"))

for filename in images:
    image = cv2.imread(filename)

    if image is None:
        print(f"읽기 실패: {filename}")
        continue

    gray = cv2.cvtColor(
        image,
        cv2.COLOR_BGR2GRAY
    )

    found, corners = cv2.findChessboardCorners(
        gray,
        CHECKERBOARD,
        None
    )

    if found:
        cv2.drawChessboardCorners(
            image,
            CHECKERBOARD,
            corners,
            found
        )

        cv2.imshow("Detected Corners", image)
        cv2.waitKey(0)

cv2.destroyAllWindows()
