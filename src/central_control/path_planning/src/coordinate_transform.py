"""Coordinate conversions shared by perception and path planning."""

from dataclasses import dataclass


@dataclass(frozen=True)
class BevConfig:
    """BEV image dimensions and pixel density."""

    width_px: int = 1600
    height_px: int = 800
    scale_px_per_cm: float = 8.0

    def __post_init__(self) -> None:
        if self.width_px <= 0 or self.height_px <= 0 or self.scale_px_per_cm <= 0:
            raise ValueError("BEV dimensions and scale must be positive")


class CoordinateTransform:
    """Convert between top-left BEV pixels and bottom-left world/grid coordinates."""

    def __init__(self, bev_config: BevConfig) -> None:
        self.bev = bev_config

    def bev_px_to_world_cm(self, x_px: float, y_px: float) -> tuple[float, float]:
        """Convert a BEV pixel into world centimetres."""
        x_cm = x_px / self.bev.scale_px_per_cm
        # BEV y grows downward, whereas world y grows upward from the image bottom.
        y_cm = (self.bev.height_px - y_px) / self.bev.scale_px_per_cm
        return x_cm, y_cm

    def world_cm_to_bev_px(self, x_cm: float, y_cm: float) -> tuple[float, float]:
        """Convert world centimetres into a BEV pixel."""
        x_px = x_cm * self.bev.scale_px_per_cm
        y_px = self.bev.height_px - y_cm * self.bev.scale_px_per_cm
        return x_px, y_px

    @staticmethod
    def world_cm_to_grid(
        x_cm: float, y_cm: float, resolution_cm: float = 1.0
    ) -> tuple[int, int]:
        """Convert world coordinates to the containing integer grid cell."""
        CoordinateTransform._validate_resolution(resolution_cm)
        # floor division maps a continuous position to the cell that contains it.
        return int(x_cm // resolution_cm), int(y_cm // resolution_cm)

    @staticmethod
    def grid_to_world_cm(
        gx: int, gy: int, resolution_cm: float = 1.0
    ) -> tuple[float, float]:
        """Convert a grid index to the cell's bottom-left world coordinate."""
        CoordinateTransform._validate_resolution(resolution_cm)
        return gx * resolution_cm, gy * resolution_cm

    @staticmethod
    def _validate_resolution(resolution_cm: float) -> None:
        if resolution_cm <= 0:
            raise ValueError("resolution_cm must be positive")
