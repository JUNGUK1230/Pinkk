from pathlib import Path
from typing import Tuple

import numpy as np


class CoordinateTransformer:
    """
    Coordinate pipeline:

    BEV pixel
        ->
    LiDAR map image pixel
        ->
    ROS /map coordinate [m]

    기준:
    - BEV: OpenCV image coordinate
    - LiDAR pixel: OpenCV image coordinate
    - /map: ROS world coordinate
    """

    def __init__(
        self,
        registration_file: str | Path
    ) -> None:

        self.registration_file = Path(
            registration_file
        )

        if not self.registration_file.exists():
            raise FileNotFoundError(
                f"Registration file not found: "
                f"{self.registration_file}"
            )

        data = np.load(
            self.registration_file
        )

        # ----------------------------------------------------
        # Camera BEV -> LiDAR image pixel
        # 2x3 Affine Matrix
        # ----------------------------------------------------

        self.affine_matrix = np.asarray(
            data["affine_matrix"],
            dtype=np.float64
        )

        if self.affine_matrix.shape != (2, 3):
            raise ValueError(
                "affine_matrix must have shape (2, 3), "
                f"got {self.affine_matrix.shape}"
            )

        # ----------------------------------------------------
        # LiDAR map image size
        # ----------------------------------------------------

        self.lidar_width = int(
            data["lidar_width"]
        )

        self.lidar_height = int(
            data["lidar_height"]
        )

        # ----------------------------------------------------
        # ROS map metadata
        # ----------------------------------------------------

        self.resolution = float(
            data["resolution"]
        )

        self.origin = np.asarray(
            data["origin"],
            dtype=np.float64
        )

        if self.origin.shape[0] < 2:
            raise ValueError(
                "origin must contain at least x and y"
            )

        self.origin_x = float(
            self.origin[0]
        )

        self.origin_y = float(
            self.origin[1]
        )

        # ----------------------------------------------------
        # Optional registration information
        # ----------------------------------------------------

        self.rmse_cm = (
            float(data["rmse_cm"])
            if "rmse_cm" in data.files
            else None
        )


    # ========================================================
    # 1. BEV pixel -> LiDAR image pixel
    # ========================================================

    def bev_to_lidar_pixel(
        self,
        bev_x: float,
        bev_y: float
    ) -> Tuple[float, float]:

        point = np.array(
            [
                float(bev_x),
                float(bev_y),
                1.0
            ],
            dtype=np.float64
        )

        result = (
            self.affine_matrix
            @
            point
        )

        lidar_x = float(
            result[0]
        )

        lidar_y = float(
            result[1]
        )

        return (
            lidar_x,
            lidar_y
        )


    # ========================================================
    # 2. LiDAR image pixel -> ROS /map [m]
    # ========================================================

    def lidar_pixel_to_map(
        self,
        lidar_x: float,
        lidar_y: float
    ) -> Tuple[float, float]:

        """
        OpenCV image:
            x -> right
            y -> down

        ROS map:
            x -> right
            y -> up

        pixel-center convention 사용.

        lidar_x, lidar_y는 연속 실수 좌표 가능.
        """

        map_x = (
            self.origin_x
            +
            (
                float(lidar_x)
                +
                0.5
            )
            *
            self.resolution
        )

        map_y = (
            self.origin_y
            +
            (
                self.lidar_height
                -
                float(lidar_y)
                -
                0.5
            )
            *
            self.resolution
        )

        return (
            float(map_x),
            float(map_y)
        )


    # ========================================================
    # 3. BEV pixel -> ROS /map [m]
    # ========================================================

    def bev_to_map(
        self,
        bev_x: float,
        bev_y: float
    ) -> Tuple[float, float]:

        lidar_x, lidar_y = (
            self.bev_to_lidar_pixel(
                bev_x,
                bev_y
            )
        )

        map_x, map_y = (
            self.lidar_pixel_to_map(
                lidar_x,
                lidar_y
            )
        )

        return (
            map_x,
            map_y
        )


    # ========================================================
    # 4. ROS /map [m] -> LiDAR image pixel
    # ========================================================

    def map_to_lidar_pixel(
        self,
        map_x: float,
        map_y: float
    ) -> Tuple[float, float]:

        lidar_x = (
            (
                float(map_x)
                -
                self.origin_x
            )
            /
            self.resolution
            -
            0.5
        )

        lidar_y = (
            self.lidar_height
            -
            (
                (
                    float(map_y)
                    -
                    self.origin_y
                )
                /
                self.resolution
            )
            -
            0.5
        )

        return (
            float(lidar_x),
            float(lidar_y)
        )


    # ========================================================
    # 5. 범위 검사
    # ========================================================

    def is_inside_lidar_map(
        self,
        lidar_x: float,
        lidar_y: float
    ) -> bool:

        return (
            0.0
            <= lidar_x
            < self.lidar_width
            and
            0.0
            <= lidar_y
            < self.lidar_height
        )


    # ========================================================
    # 6. 전체 변환 정보 출력
    # ========================================================

    def print_info(
        self
    ) -> None:

        print("=" * 70)
        print("Coordinate Transformer")
        print("=" * 70)

        print(
            f"Registration file : "
            f"{self.registration_file}"
        )

        print(
            f"LiDAR map size    : "
            f"{self.lidar_width} x "
            f"{self.lidar_height}"
        )

        print(
            f"Resolution        : "
            f"{self.resolution} m/px"
        )

        print(
            f"Origin            : "
            f"({self.origin_x:.6f}, "
            f"{self.origin_y:.6f})"
        )

        if self.rmse_cm is not None:

            print(
                f"Registration RMSE : "
                f"{self.rmse_cm:.3f} cm"
            )

        print()
        print("Affine Matrix:")
        print(
            self.affine_matrix
        )