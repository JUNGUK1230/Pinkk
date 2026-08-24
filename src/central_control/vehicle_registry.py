"""Validated persistent vehicle identities and ROS topic construction."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import yaml


@dataclass(frozen=True)
class VehicleConfig:
    vehicle_id: str
    ros_namespace: str
    controller_id: str
    hardware_serial: str
    display_name: str

    def topic(self, relative_name: str) -> str:
        name = str(relative_name).strip("/")
        if not name or any(part in {"", ".", ".."} for part in name.split("/")):
            raise ValueError(f"invalid relative ROS topic: {relative_name!r}")
        return f"{self.ros_namespace}/{name}"

    @property
    def frame_prefix(self) -> str:
        return self.ros_namespace.lstrip("/")


def _load_registry() -> dict[str, VehicleConfig]:
    path = Path(__file__).resolve().parent / "config" / "vehicles.yaml"
    with path.open(encoding="utf-8") as file:
        payload = yaml.safe_load(file)
    rows = payload.get("vehicles") if isinstance(payload, dict) else None
    if not isinstance(rows, dict) or not rows:
        raise ValueError(f"vehicle registry is empty: {path}")
    vehicles = {}
    for key, row in rows.items():
        if not isinstance(row, dict) or row.get("vehicle_id") != key:
            raise ValueError(f"invalid vehicle registry row: {key!r}")
        namespace = str(row.get("ros_namespace", ""))
        if namespace != f"/pinkk/{key}":
            raise ValueError(f"unsafe namespace for {key}: {namespace!r}")
        vehicles[key] = VehicleConfig(
            vehicle_id=key,
            ros_namespace=namespace,
            controller_id=str(row["controller_id"]),
            hardware_serial=str(row["hardware_serial"]),
            display_name=str(row["display_name"]),
        )
    return vehicles


VEHICLES = _load_registry()


def get_vehicle(vehicle_id: str) -> VehicleConfig:
    """Return only allow-listed vehicles; never derive namespaces from input."""
    try:
        return VEHICLES[str(vehicle_id)]
    except KeyError as error:
        raise ValueError(f"unknown vehicle_id: {vehicle_id!r}") from error
