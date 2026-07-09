"""YOLO 기반 주차공간/차량 탐지 모듈."""

from __future__ import annotations

from pathlib import Path

from ultralytics import YOLO


class YoloDetector:
    def __init__(self, model_path: str | Path) -> None:
        self.model = YOLO(str(model_path))

    def predict(self, image):
        return self.model(image, verbose=False)
