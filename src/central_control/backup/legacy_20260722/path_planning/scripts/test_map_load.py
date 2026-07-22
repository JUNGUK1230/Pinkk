"""Load the LiDAR map and save a binary occupancy debug image."""

from pathlib import Path
import sys

import yaml

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.append(str(PROJECT_ROOT / "src"))

from occupancy_grid import OccupancyGridMap  # noqa: E402


def resolve_map_image(pgm_path: Path, yaml_path: Path) -> Path:
    """Use the requested PGM, or the YAML image when this map was saved as PNG."""
    if pgm_path.exists():
        return pgm_path
    with yaml_path.open("r", encoding="utf-8") as file:
        image_name = (yaml.safe_load(file) or {}).get("image")
    yaml_image = yaml_path.parent / image_name if image_name else None
    if yaml_image is not None and yaml_image.exists():
        print(f"PGM not found; using YAML map image instead: {yaml_image}")
        return yaml_image
    return pgm_path  # OccupancyGridMap raises the required descriptive error.


def main() -> None:
    """Run the map-loading smoke test."""
    first_map_dir = PROJECT_ROOT.parent / "camera_tools" / "first_map"
    pgm_path = first_map_dir / "my_test_map0710.pgm"
    yaml_path = first_map_dir / "my_test_map0710.yaml"

    occupancy_map = OccupancyGridMap(
        str(resolve_map_image(pgm_path, yaml_path)),
        str(yaml_path),
        block_outside_area=True,
    )
    occupancy_map.print_info()
    output_path = PROJECT_ROOT / "output" / "debug_occupancy_grid.png"
    occupancy_map.save_debug_image(str(output_path))
    print(f"Debug image saved: {output_path}")


if __name__ == "__main__":
    main()
