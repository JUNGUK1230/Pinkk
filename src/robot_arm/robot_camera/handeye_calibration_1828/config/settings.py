"""Hand-Eye 캘리브레이션 설정값.

좌표계 표기는 T_A_B이며 p_A = T_A_B @ p_B를 의미한다.
"""

from pathlib import Path

PACKAGE_DIR = Path(__file__).resolve().parents[1]
DATA_DIR = PACKAGE_DIR / "data"
INTRINSICS_PATH = PACKAGE_DIR.parent / "camera_calibration" / "results" / "intrinsics.npz"
SAMPLES_PATH = DATA_DIR / "handeye_samples.npz"
RESULT_MATRIX_PATH = DATA_DIR / "T_flange_camera.npy"
RESULT_NPZ_PATH = DATA_DIR / "handeye_result.npz"

# SSH로 접속한 로봇 PC에서 카메라를 직접 연다.
CAMERA_DEVICE: int | str = 0
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480

# 반드시 실제로 제작한 ChArUco 보드의 실측값과 일치시킨다. 단위는 meter다.
CHARUCO_SQUARES_X = 5
CHARUCO_SQUARES_Y = 7
SQUARE_LENGTH_M = 0.030
MARKER_LENGTH_M = 0.022
ARUCO_DICTIONARY_NAME = "DICT_4X4_50"
MIN_CHARUCO_CORNERS = 12
MAX_REPROJECTION_ERROR_PX = 0.7

TARGET_SAMPLE_COUNT = 20
MIN_CALIBRATION_SAMPLES = 10
CAPTURE_SETTLE_SECONDS = 0.25
MIN_TRANSLATION_DIFFERENCE_M = 0.010
MIN_ROTATION_DIFFERENCE_DEG = 5.0

# Elephant Robotics 공식 문서 기준:
# rx=roll, ry=pitch, rz=yaw이고 회전 순서는 local/body 좌표계 기준 intrinsic ZYX다.
# scipy에서는 Rotation.from_euler("ZYX", [rz, ry, rx], degrees=True)로 표현한다.
ROBOT_EULER_SEQUENCE = "ZYX"
ROBOT_EULER_CONVENTION_VERIFIED = True

# Hand-Eye 입력은 반드시 base 기준 flange pose여야 한다.
EXPECTED_REFERENCE_FRAME = 0  # 공식 API: 0=base, 1=tool
EXPECTED_END_TYPE = 0        # 공식 API: 0=flange, 1=tool
