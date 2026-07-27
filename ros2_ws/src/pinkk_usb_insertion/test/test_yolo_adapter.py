"""YOLO Pose 배열 정규화의 형식과 visibility 처리를 검증한다."""

import numpy as np
import pytest

from pinkk_usb_insertion.perception.yolo_adapter import normalize_yolo_pose


def test_normalizes_one_four_keypoint_detection() -> None:
    """정상적인 4-keypoint 한 건을 순서대로 보존한다."""
    detections = normalize_yolo_pose(
        boxes_xywh=np.array([[320.0, 240.0, 120.0, 60.0]]),
        class_ids=np.array([0.0]),
        object_confidences=np.array([0.9]),
        keypoints_xy=np.array(
            [[[260.0, 210.0], [380.0, 210.0], [380.0, 270.0], [260.0, 270.0]]]
        ),
        keypoint_confidences=np.array([[0.95, 0.94, 0.93, 0.92]]),
        names={0: 'usb_port'},
        visibility_threshold=0.01,
    )

    assert len(detections) == 1
    assert detections[0].class_name == 'usb_port'
    assert detections[0].object_confidence == pytest.approx(0.9)
    assert [point.index for point in detections[0].keypoints] == [0, 1, 2, 3]
    assert all(point.visible for point in detections[0].keypoints)


def test_marks_low_confidence_keypoint_invisible() -> None:
    """Visibility 기준보다 낮은 keypoint는 보이지 않음으로 표시한다."""
    detections = normalize_yolo_pose(
        boxes_xywh=np.array([[320.0, 240.0, 120.0, 60.0]]),
        class_ids=np.array([0.0]),
        object_confidences=np.array([0.9]),
        keypoints_xy=np.zeros((1, 4, 2)),
        keypoint_confidences=np.array([[0.9, 0.9, 0.0, 0.9]]),
        names={0: 'usb_port'},
        visibility_threshold=0.01,
    )

    assert detections[0].keypoints[2].visible is False


def test_rejects_non_pose_shape() -> None:
    """네 점이 아닌 pose 출력은 ROS 메시지 생성 전에 거부한다."""
    with pytest.raises(ValueError, match='keypoint 4개'):
        normalize_yolo_pose(
            boxes_xywh=np.array([[320.0, 240.0, 120.0, 60.0]]),
            class_ids=np.array([0.0]),
            object_confidences=np.array([0.9]),
            keypoints_xy=np.zeros((1, 3, 2)),
            keypoint_confidences=np.zeros((1, 3)),
            names={0: 'usb_port'},
            visibility_threshold=0.01,
        )
