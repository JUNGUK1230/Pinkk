"""cv_bridge 없는 bgr8 ROS Image 변환을 검증한다."""

import numpy as np
import pytest
from sensor_msgs.msg import Image

from pinkk_usb_insertion.image_conversion import (
    array_to_bgr8_image,
    bgr8_image_to_array,
)


def test_bgr8_round_trip() -> None:
    """BGR 배열의 크기·dtype·픽셀값을 그대로 보존한다."""
    frame = np.arange(18, dtype=np.uint8).reshape(2, 3, 3)
    message = array_to_bgr8_image(frame)

    assert message.height == 2
    assert message.width == 3
    assert message.encoding == 'bgr8'
    assert message.step == 9
    np.testing.assert_array_equal(bgr8_image_to_array(message), frame)


def test_bgr8_input_removes_row_padding() -> None:
    """ROS Image의 행 끝 패딩은 OpenCV 배열에서 제외한다."""
    message = Image()
    message.height = 2
    message.width = 2
    message.encoding = 'bgr8'
    message.step = 8
    message.data = bytes(
        [1, 2, 3, 4, 5, 6, 99, 99, 7, 8, 9, 10, 11, 12, 99, 99]
    )

    frame = bgr8_image_to_array(message)

    np.testing.assert_array_equal(
        frame,
        np.array(
            [
                [[1, 2, 3], [4, 5, 6]],
                [[7, 8, 9], [10, 11, 12]],
            ],
            dtype=np.uint8,
        ),
    )


def test_rejects_unsupported_encoding() -> None:
    """색상 채널 해석이 다른 인코딩은 명확히 거부한다."""
    message = Image()
    message.height = 1
    message.width = 1
    message.encoding = 'rgb8'
    message.step = 3
    message.data = bytes([1, 2, 3])

    with pytest.raises(ValueError, match='bgr8'):
        bgr8_image_to_array(message)
