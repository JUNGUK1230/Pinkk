import cv2
import glob
import numpy as np
from pathlib import Path

# ============================================================
# 설정값
# ============================================================

# 체커보드 내부 코너 개수
# 반드시 네 체커보드와 맞춰야 함
CHECKERBOARD = (9, 6)

# 체커보드 한 칸 실제 크기 [m]
# 예: 7 cm = 0.07 m
SQUARE_SIZE = 0.07

CAMERA_DIR = Path(__file__).resolve().parent

# 이미지 폴더
IMAGE_DIR = CAMERA_DIR / "calibration_images"

# 결과 저장 파일
OUTPUT_FILE = CAMERA_DIR / "camera_calibration.npz"


# ============================================================
# Sub-pixel refinement 종료 조건
# ============================================================

criteria = (
    cv2.TERM_CRITERIA_EPS
    + cv2.TERM_CRITERIA_MAX_ITER,
    30,
    0.001
)


# ============================================================
# 체커보드 실제 3D 좌표 생성
#
# 예:
# (0, 0, 0)
# (0.025, 0, 0)
# (0.050, 0, 0)
# ...
# ============================================================

objp = np.zeros(
    (
        CHECKERBOARD[0] * CHECKERBOARD[1],
        3
    ),
    dtype=np.float32
)

objp[:, :2] = np.mgrid[
    0:CHECKERBOARD[0],
    0:CHECKERBOARD[1]
].T.reshape(-1, 2)

# 실제 크기 반영
objp *= SQUARE_SIZE


# ============================================================
# 모든 이미지의 대응점 저장
# ============================================================

# 실제 3D 좌표
object_points = []

# 이미지 2D 좌표
image_points = []


# ============================================================
# 이미지 불러오기
# ============================================================

image_paths = sorted(
    glob.glob(str(IMAGE_DIR / "*.jpg"))
)

if not image_paths:
    raise RuntimeError(
        f"이미지를 찾을 수 없습니다: {IMAGE_DIR}"
    )

print("=" * 60)
print("카메라 캘리브레이션 시작")
print("=" * 60)

print(f"전체 이미지 수: {len(image_paths)}")
print(f"체커보드 내부 코너: {CHECKERBOARD}")
print(f"한 칸 크기: {SQUARE_SIZE} m")
print()


image_size = None
success_count = 0


# ============================================================
# 각 이미지에서 체커보드 검출
# ============================================================

for image_path in image_paths:

    image = cv2.imread(image_path)

    if image is None:
        print(f"[읽기 실패] {image_path}")
        continue

    gray = cv2.cvtColor(
        image,
        cv2.COLOR_BGR2GRAY
    )

    current_size = gray.shape[::-1]

    # 첫 이미지에서 해상도 저장
    if image_size is None:
        image_size = current_size

    # 모든 이미지 해상도가 같은지 확인
    elif current_size != image_size:
        print(
            f"[해상도 불일치] {image_path}"
        )
        print(
            f"현재: {current_size}, "
            f"기준: {image_size}"
        )
        continue


    # 체커보드 코너 검출
    found, corners = cv2.findChessboardCorners(
        gray,
        CHECKERBOARD,
        None
    )


    if not found:
        print(f"[검출 실패] {image_path}")
        continue


    # Sub-pixel 정밀화
    refined_corners = cv2.cornerSubPix(
        gray,
        corners,
        (11, 11),
        (-1, -1),
        criteria
    )


    # 데이터 저장
    object_points.append(
        objp.copy()
    )

    image_points.append(
        refined_corners
    )

    success_count += 1

    print(
        f"[검출 성공 {success_count:02d}] "
        f"{image_path}"
    )


# ============================================================
# 결과 확인
# ============================================================

print()
print("=" * 60)
print("체커보드 검출 결과")
print("=" * 60)

print(f"전체 이미지 : {len(image_paths)}")
print(f"성공 이미지 : {success_count}")
print(
    f"실패 이미지 : "
    f"{len(image_paths) - success_count}"
)


if success_count < 10:
    raise RuntimeError(
        "유효한 이미지가 너무 적습니다. "
        "최소 10장 이상 확보하세요."
    )


# ============================================================
# Camera Calibration
# ============================================================

print()
print("cv2.calibrateCamera() 실행 중...")


rms_error, camera_matrix, dist_coeffs, rvecs, tvecs = (
    cv2.calibrateCamera(
        object_points,
        image_points,
        image_size,
        None,
        None
    )
)


# ============================================================
# 결과 출력
# ============================================================

print()
print("=" * 60)
print("캘리브레이션 완료")
print("=" * 60)


print()
print("[RMS Error]")
print(rms_error)


print()
print("[Camera Matrix K]")
print(camera_matrix)


print()
print("[Distortion Coefficients D]")
print(dist_coeffs)


# ============================================================
# 평균 Reprojection Error 계산
# ============================================================

total_error = 0.0


for i in range(len(object_points)):

    projected_points, _ = cv2.projectPoints(
        object_points[i],
        rvecs[i],
        tvecs[i],
        camera_matrix,
        dist_coeffs
    )

    error = cv2.norm(
        image_points[i],
        projected_points,
        cv2.NORM_L2
    ) / len(projected_points)

    total_error += error


mean_reprojection_error = (
    total_error / len(object_points)
)


print()
print("[Mean Reprojection Error]")
print(mean_reprojection_error)


# ============================================================
# 결과 저장
# ============================================================

np.savez(
    OUTPUT_FILE,

    camera_matrix=camera_matrix,
    dist_coeffs=dist_coeffs,

    image_width=image_size[0],
    image_height=image_size[1],

    rms_error=rms_error,
    mean_reprojection_error=mean_reprojection_error
)


print()
print("=" * 60)
print(f"저장 완료: {OUTPUT_FILE}")
print("=" * 60)
