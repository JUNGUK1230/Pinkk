"""NumPy ABI에 묶인 cv_bridge 없이 bgr8 ROS Image를 변환한다."""

from __future__ import annotations

import numpy as np
from sensor_msgs.msg import Image


def bgr8_image_to_array(message: Image) -> np.ndarray:
    """bgr8 ROS Image를 행 패딩을 제거한 OpenCV 배열로 복사한다."""
    if message.encoding.lower() != 'bgr8':
        raise ValueError(
            f'bgr8 영상만 지원합니다: encoding={message.encoding!r}'
        )
    height = int(message.height)
    width = int(message.width)
    row_bytes = width * 3
    step = int(message.step)
    if height <= 0 or width <= 0:
        raise ValueError(f'영상 크기가 유효하지 않습니다: {width}x{height}')
    if step < row_bytes:
        raise ValueError(
            f'영상 step이 한 행보다 작습니다: step={step}, row={row_bytes}'
        )
    raw = np.frombuffer(message.data, dtype=np.uint8)
    required = height * step
    if raw.size < required:
        raise ValueError(
            f'영상 데이터가 부족합니다: actual={raw.size}, required={required}'
        )
    rows = raw[:required].reshape(height, step)
    return rows[:, :row_bytes].reshape(height, width, 3).copy()


def array_to_bgr8_image(
    frame: np.ndarray,
    *,
    header=None,
) -> Image:
    """연속 BGR uint8 배열을 bgr8 ROS Image로 변환한다."""
    array = np.asarray(frame)
    if array.dtype != np.uint8:
        raise ValueError(f'영상 dtype은 uint8이어야 합니다: {array.dtype}')
    if array.ndim != 3 or array.shape[2] != 3:
        raise ValueError(f'BGR 영상 shape이 아닙니다: {array.shape}')
    contiguous = np.ascontiguousarray(array)
    height, width, _ = contiguous.shape
    message = Image()
    if header is not None:
        message.header = header
    message.height = int(height)
    message.width = int(width)
    message.encoding = 'bgr8'
    message.is_bigendian = 0
    message.step = int(width * 3)
    message.data = contiguous.tobytes()
    return message
