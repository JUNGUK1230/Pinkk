"""Ultralytics YOLO Pose 배열을 ROS 메시지로 옮기기 전 검증·정규화한다."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping, Sequence

import numpy as np


@dataclass(frozen=True)
class YoloKeypoint:
    """메시지로 변환하기 전 하나의 정규화된 keypoint."""

    index: int
    x: float
    y: float
    confidence: float
    visible: bool


@dataclass(frozen=True)
class YoloDetection:
    """메시지로 변환하기 전 하나의 정규화된 YOLO 검출."""

    class_name: str
    object_confidence: float
    center_x: float
    center_y: float
    width: float
    height: float
    keypoints: tuple[YoloKeypoint, ...]


def _class_name(
    names: Mapping[int, str] | Sequence[str],
    class_id: int,
) -> str:
    try:
        return str(names[class_id])
    except (IndexError, KeyError) as error:
        raise ValueError(f'YOLO class id {class_id}의 이름이 없습니다') from error


def _visible(
    point: np.ndarray,
    confidence: float,
    threshold: float,
) -> bool:
    return bool(np.all(np.isfinite(point)) and confidence >= threshold)


def normalize_yolo_pose(
    boxes_xywh: np.ndarray,
    class_ids: np.ndarray,
    object_confidences: np.ndarray,
    keypoints_xy: np.ndarray,
    keypoint_confidences: np.ndarray,
    names: Mapping[int, str] | Sequence[str],
    visibility_threshold: float,
) -> list[YoloDetection]:
    """YOLO 결과의 개수와 4-keypoint 형식을 검사해 불변 record로 변환한다."""
    boxes = np.asarray(boxes_xywh, dtype=np.float64)
    classes = np.asarray(class_ids, dtype=np.float64).reshape(-1)
    object_scores = np.asarray(
        object_confidences, dtype=np.float64
    ).reshape(-1)
    points = np.asarray(keypoints_xy, dtype=np.float64)
    point_scores = np.asarray(keypoint_confidences, dtype=np.float64)

    count = len(classes)
    if boxes.shape != (count, 4):
        raise ValueError(f'YOLO bbox 배열 형식 오류: {boxes.shape}')
    if object_scores.shape != (count,):
        raise ValueError(
            f'YOLO 객체 confidence 배열 형식 오류: {object_scores.shape}'
        )
    if points.shape != (count, 4, 2):
        raise ValueError(
            f'YOLO Pose는 검출마다 keypoint 4개가 필요합니다: {points.shape}'
        )
    if point_scores.shape != (count, 4):
        raise ValueError(
            f'YOLO keypoint confidence 배열 형식 오류: {point_scores.shape}'
        )
    if visibility_threshold < 0.0 or visibility_threshold > 1.0:
        raise ValueError('visibility_threshold는 0~1이어야 합니다')

    detections: list[YoloDetection] = []
    for detection_index in range(count):
        center_x, center_y, width, height = boxes[detection_index]
        object_confidence = float(object_scores[detection_index])
        class_id = int(classes[detection_index])
        keypoints = tuple(
            YoloKeypoint(
                index=keypoint_index,
                x=float(points[detection_index, keypoint_index, 0]),
                y=float(points[detection_index, keypoint_index, 1]),
                confidence=float(
                    point_scores[detection_index, keypoint_index]
                ),
                visible=_visible(
                    points[detection_index, keypoint_index],
                    float(point_scores[detection_index, keypoint_index]),
                    visibility_threshold,
                ),
            )
            for keypoint_index in range(4)
        )
        detections.append(
            YoloDetection(
                class_name=_class_name(names, class_id),
                object_confidence=object_confidence,
                center_x=float(center_x),
                center_y=float(center_y),
                width=float(width),
                height=float(height),
                keypoints=keypoints,
            )
        )
    return detections
