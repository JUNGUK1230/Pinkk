"""YOLO가 발행한 여러 USB 포트 후보를 검사하고 하나를 선택한다."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Sequence

import numpy as np


@dataclass(frozen=True)
class SelectedDetection:
    detection: Any
    ordered_points_px: np.ndarray
    score: float


def ordered_keypoints(
    detection: Any,
    minimum_keypoint_confidence: float,
) -> np.ndarray:
    width = int(detection.source_image_width)
    height = int(detection.source_image_height)
    if width <= 0 or height <= 0:
        raise ValueError('원본 영상 해상도가 유효하지 않습니다')
    if len(detection.keypoints) != 4:
        raise ValueError('USB 포트 keypoint는 정확히 4개여야 합니다')

    indexed: dict[int, Any] = {}
    for keypoint in detection.keypoints:
        index = int(keypoint.index)
        if index not in range(4) or index in indexed:
            raise ValueError('keypoint index는 중복 없이 0,1,2,3이어야 합니다')
        if not bool(keypoint.visible):
            raise ValueError(f'keypoint {index}가 보이지 않습니다')
        if float(keypoint.confidence) < minimum_keypoint_confidence:
            raise ValueError(f'keypoint {index} 신뢰도가 기준보다 낮습니다')
        x, y = float(keypoint.x), float(keypoint.y)
        if not np.isfinite(x) or not np.isfinite(y):
            raise ValueError(f'keypoint {index} 좌표가 유효하지 않습니다')
        if not 0.0 <= x < width or not 0.0 <= y < height:
            raise ValueError(f'keypoint {index}가 원본 영상 범위를 벗어났습니다')
        indexed[index] = keypoint
    if set(indexed) != set(range(4)):
        raise ValueError('keypoint index 0,1,2,3이 모두 필요합니다')
    return np.array(
        [[float(indexed[index].x), float(indexed[index].y)] for index in range(4)],
        dtype=np.float64,
    )


def select_detection(
    detections: Sequence[Any],
    minimum_object_confidence: float,
    minimum_keypoint_confidence: float,
    target_class_name: str,
    target_detection_id: str = '',
) -> SelectedDetection:
    candidates: list[SelectedDetection] = []
    rejection_reasons: list[str] = []
    for detection in detections:
        detection_id = str(detection.detection_id)
        try:
            if str(detection.class_name) != target_class_name:
                raise ValueError(f'대상 클래스가 아닙니다: {detection.class_name}')
            if target_detection_id and detection_id != target_detection_id:
                raise ValueError('선택된 detection_id가 아닙니다')
            object_confidence = float(detection.object_confidence)
            if object_confidence < minimum_object_confidence:
                raise ValueError('객체 신뢰도가 기준보다 낮습니다')
            points = ordered_keypoints(detection, minimum_keypoint_confidence)
            minimum_keypoint = min(float(point.confidence) for point in detection.keypoints)
            candidates.append(
                SelectedDetection(
                    detection=detection,
                    ordered_points_px=points,
                    score=object_confidence * minimum_keypoint,
                )
            )
        except ValueError as error:
            rejection_reasons.append(f'{detection_id or "unknown"}: {error}')

    if not candidates:
        details = '; '.join(rejection_reasons) if rejection_reasons else '검출 후보 없음'
        raise ValueError(f'사용 가능한 USB 포트 검출이 없습니다 ({details})')
    return max(candidates, key=lambda candidate: candidate.score)
