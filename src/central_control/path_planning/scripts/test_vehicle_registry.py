"""Persistent vehicle identity and topic routing regression test."""

from pathlib import Path
import sys


SRC_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(SRC_ROOT))

from central_control.vehicle_registry import get_vehicle  # noqa: E402


def main() -> int:
    first = get_vehicle("vehicle_1")
    second = get_vehicle("vehicle_2")
    assert first.topic("path") == "/pinkk/vehicle_1/path"
    assert second.topic("path") == "/pinkk/vehicle_2/path"
    assert second.topic("cmd_vel") == "/pinkk/vehicle_2/cmd_vel"
    assert second.topic("localization_pose") == "/pinkk/vehicle_2/localization_pose"
    assert second.topic("path_valid") == "/pinkk/vehicle_2/path_valid"
    assert second.topic("odom") == "/pinkk/vehicle_2/odom"
    assert second.topic("scan") == "/pinkk/vehicle_2/scan"
    assert first.topic("path") != second.topic("path")
    assert second.controller_id == "pinky_02"
    assert second.hardware_serial == "PINKY-002"
    try:
        get_vehicle("vehicle_2/../../vehicle_1")
    except ValueError:
        pass
    else:
        raise AssertionError("unknown vehicle IDs must fail closed")
    print("Vehicle registry and topic routing regression passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
