#!/usr/bin/env python3

import cv2


# ============================================================
# 설정
# ============================================================

IMAGE_PATH = "pinkk/map/lidar_map/my_test_map0710_1.png"

WINDOW_NAME = "BEV Coordinate Picker"

# 네 BEV 기준
# 원점: 좌하단
# 단위: cm
MAP_WIDTH_CM = 200.0
MAP_HEIGHT_CM = 100.0


# ============================================================
# 전역 변수
# ============================================================

clicked_points = []

original_image = None
display_image = None

image_width = 0
image_height = 0


# ============================================================
# Pixel -> Metric Coordinate
#
# OpenCV:
#   원점 = 좌상단
#   x -> 오른쪽
#   y -> 아래쪽
#
# 우리가 원하는 좌표:
#   원점 = 좌하단
#   x -> 오른쪽
#   y -> 위쪽
# ============================================================

def pixel_to_metric(px, py):

    x_cm = (
        px
        / image_width
        * MAP_WIDTH_CM
    )

    y_cm = (
        (image_height - py)
        / image_height
        * MAP_HEIGHT_CM
    )

    return x_cm, y_cm


# ============================================================
# 화면 다시 그리기
# ============================================================

def redraw():

    global display_image

    display_image = original_image.copy()

    for index, point in enumerate(clicked_points):

        px = point["px"]
        py = point["py"]

        x_cm = point["x_cm"]
        y_cm = point["y_cm"]

        label = (
            f"P{index + 1} "
            f"({x_cm:.2f}, {y_cm:.2f})"
        )

        # 점
        cv2.circle(
            display_image,
            (px, py),
            6,
            (0, 0, 255),
            -1
        )

        # 좌표 텍스트
        cv2.putText(
            display_image,
            label,
            (px + 10, py - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 255, 0),
            2,
            cv2.LINE_AA
        )

    # 안내 문구
    cv2.putText(
        display_image,
        "Left Click: Add Point",
        (20, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        (0, 255, 255),
        2,
        cv2.LINE_AA
    )

    cv2.putText(
        display_image,
        "Right Click: Undo",
        (20, 60),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        (0, 255, 255),
        2,
        cv2.LINE_AA
    )

    cv2.putText(
        display_image,
        "S: Save CSV",
        (20, 90),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        (0, 255, 255),
        2,
        cv2.LINE_AA
    )

    cv2.putText(
        display_image,
        "C: Clear",
        (20, 120),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        (0, 255, 255),
        2,
        cv2.LINE_AA
    )

    cv2.putText(
        display_image,
        "Q: Quit",
        (20, 150),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        (0, 255, 255),
        2,
        cv2.LINE_AA
    )


# ============================================================
# Mouse Callback
# ============================================================

def mouse_callback(
    event,
    x,
    y,
    flags,
    param
):

    global clicked_points

    # --------------------------------------------------------
    # 좌클릭: 점 추가
    # --------------------------------------------------------

    if event == cv2.EVENT_LBUTTONDOWN:

        x_cm, y_cm = pixel_to_metric(
            x,
            y
        )

        point = {
            "px": x,
            "py": y,
            "x_cm": x_cm,
            "y_cm": y_cm
        }

        clicked_points.append(
            point
        )

        print(
            f"P{len(clicked_points)} | "
            f"pixel=({x}, {y}) | "
            f"metric=({x_cm:.3f}, {y_cm:.3f}) cm"
        )

        redraw()


    # --------------------------------------------------------
    # 우클릭: 마지막 점 삭제
    # --------------------------------------------------------

    elif event == cv2.EVENT_RBUTTONDOWN:

        if len(clicked_points) > 0:

            removed = clicked_points.pop()

            print(
                "Undo | "
                f"removed pixel="
                f"({removed['px']}, {removed['py']})"
            )

            redraw()


# ============================================================
# CSV 저장
# ============================================================

def save_points_csv():

    filename = "bev_points.csv"

    with open(
        filename,
        "w",
        encoding="utf-8"
    ) as file:

        file.write(
            "id,pixel_x,pixel_y,x_cm,y_cm\n"
        )

        for index, point in enumerate(
            clicked_points
        ):

            file.write(
                f"P{index + 1},"
                f"{point['px']},"
                f"{point['py']},"
                f"{point['x_cm']:.6f},"
                f"{point['y_cm']:.6f}\n"
            )

    print(
        f"Saved: {filename}"
    )


# ============================================================
# Main
# ============================================================

def main():

    global original_image
    global display_image
    global image_width
    global image_height
    global clicked_points


    # --------------------------------------------------------
    # 이미지 로드
    # --------------------------------------------------------

    original_image = cv2.imread(
        IMAGE_PATH,
        cv2.IMREAD_COLOR
    )

    if original_image is None:

        raise FileNotFoundError(
            f"Image not found: {IMAGE_PATH}"
        )


    image_height, image_width = (
        original_image.shape[:2]
    )


    print(
        "========================================"
    )

    print(
        f"Image: {IMAGE_PATH}"
    )

    print(
        f"Image size: "
        f"{image_width} x {image_height} px"
    )

    print(
        f"Metric size: "
        f"{MAP_WIDTH_CM} x {MAP_HEIGHT_CM} cm"
    )

    print(
        "Origin: bottom-left"
    )

    print(
        "========================================"
    )


    # --------------------------------------------------------
    # 초기 화면
    # --------------------------------------------------------

    redraw()


    cv2.namedWindow(
        WINDOW_NAME,
        cv2.WINDOW_NORMAL
    )

    cv2.setMouseCallback(
        WINDOW_NAME,
        mouse_callback
    )


    # --------------------------------------------------------
    # Loop
    # --------------------------------------------------------

    while True:

        cv2.imshow(
            WINDOW_NAME,
            display_image
        )


        key = (
            cv2.waitKey(20)
            &
            0xFF
        )


        # Q or ESC
        if (
            key == ord("q")
            or
            key == 27
        ):
            break


        # C: Clear
        elif key == ord("c"):

            clicked_points = []

            print(
                "All points cleared."
            )

            redraw()


        # S: Save
        elif key == ord("s"):

            save_points_csv()


    cv2.destroyAllWindows()


if __name__ == "__main__":

    main()