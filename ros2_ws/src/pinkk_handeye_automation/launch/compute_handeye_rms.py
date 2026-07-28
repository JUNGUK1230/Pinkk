import itertools
import yaml
import numpy as np
from scipy.spatial.transform import Rotation as R

SAMPLES = "/home/kim-jayeon/.ros2/easy_handeye2/samples/pinkk_eye_in_hand.samples"
CALIB = "/home/kim-jayeon/.ros2/easy_handeye2/calibrations/pinkk_eye_in_hand.calib"


def pose_to_matrix(p):
    q = [
        p["rotation"]["x"],
        p["rotation"]["y"],
        p["rotation"]["z"],
        p["rotation"]["w"],
    ]
    t = [
        p["translation"]["x"],
        p["translation"]["y"],
        p["translation"]["z"],
    ]

    T = np.eye(4)
    T[:3, :3] = R.from_quat(q).as_matrix()
    T[:3, 3] = t
    return T


with open(SAMPLES) as f:
    sample_data = yaml.safe_load(f)

with open(CALIB) as f:
    calib = yaml.safe_load(f)

X = pose_to_matrix(calib["transform"])

samples = sample_data["samples"]

robot = [pose_to_matrix(s["robot"]) for s in samples]

# Easy Handeye2는 tracking을 Board -> Camera로 저장
# AX=XB 계산을 위해 Camera -> Board로 변환
tracking = [
    np.linalg.inv(pose_to_matrix(s["tracking"]))
    for s in samples
]

translation_errors = []
rotation_errors = []

# 모든 샘플쌍 사용 (30개면 435쌍)
for i, j in itertools.combinations(range(len(samples)), 2):

    # Robot relative motion
    A = np.linalg.inv(robot[i]) @ robot[j]

    # Camera relative motion
    B = np.linalg.inv(tracking[i]) @ tracking[j]

    lhs = A @ X
    rhs = X @ B

    E = np.linalg.inv(lhs) @ rhs

    translation_errors.append(
        np.linalg.norm(E[:3, 3]) * 1000.0
    )

    rotation_errors.append(
        np.degrees(
            np.linalg.norm(
                R.from_matrix(E[:3, :3]).as_rotvec()
            )
        )
    )

translation_errors = np.asarray(translation_errors)
rotation_errors = np.asarray(rotation_errors)

print()
print("========== Hand-Eye Residual ==========")
print(f"Motion pairs         : {len(translation_errors)}")
print()
print(f"Translation RMS      : {np.sqrt(np.mean(translation_errors**2)):.3f} mm")
print(f"Translation Mean     : {translation_errors.mean():.3f} mm")
print(f"Translation Median   : {np.median(translation_errors):.3f} mm")
print(f"Translation Max      : {translation_errors.max():.3f} mm")
print()
print(f"Rotation RMS         : {np.sqrt(np.mean(rotation_errors**2)):.3f} deg")
print(f"Rotation Mean        : {rotation_errors.mean():.3f} deg")
print(f"Rotation Median      : {np.median(rotation_errors):.3f} deg")
print(f"Rotation Max         : {rotation_errors.max():.3f} deg")
print("=======================================")