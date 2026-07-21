from types import SimpleNamespace

import numpy as np
from pinkk_usb_insertion.perception.detection_selector import select_detection
import pytest


def _detection(
    detection_id: str,
    object_confidence: float,
    keypoint_confidence: float,
) -> SimpleNamespace:
    points = ((300.0, 220.0), (360.0, 220.0), (360.0, 245.0), (300.0, 245.0))
    keypoints = [
        SimpleNamespace(
            index=index,
            x=point[0],
            y=point[1],
            confidence=keypoint_confidence,
            visible=True,
        )
        for index, point in enumerate(points)
    ]
    return SimpleNamespace(
        detection_id=detection_id,
        class_name='usb_port',
        object_confidence=object_confidence,
        source_image_width=640,
        source_image_height=480,
        keypoints=keypoints,
    )


def test_selects_highest_combined_confidence() -> None:
    first = _detection('first', 0.95, 0.70)
    second = _detection('second', 0.90, 0.90)
    selected = select_detection([first, second], 0.7, 0.6, 'usb_port')
    assert selected.detection.detection_id == 'second'
    assert selected.ordered_points_px.shape == (4, 2)
    assert np.allclose(selected.ordered_points_px[0], (300.0, 220.0))


def test_rejects_low_keypoint_confidence() -> None:
    detection = _detection('weak', 0.95, 0.20)
    with pytest.raises(ValueError, match='사용 가능한 USB 포트 검출이 없습니다'):
        select_detection([detection], 0.7, 0.6, 'usb_port')


def test_orders_keypoints_by_index() -> None:
    detection = _detection('shuffled', 0.95, 0.90)
    detection.keypoints = list(reversed(detection.keypoints))
    selected = select_detection([detection], 0.7, 0.6, 'usb_port')
    assert np.allclose(selected.ordered_points_px[0], (300.0, 220.0))
