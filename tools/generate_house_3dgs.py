#!/usr/bin/env python3
"""Generate the deterministic House 3DGS fixture used by example 88.

The fixture is intentionally procedural rather than a scan.  It gives CI a
small, reproducible indoor room with semantic colour separation (floor, wall,
window, sofa, table, cabinet, plant, and rug) while exercising the same INRIA
Gaussian PLY layout consumed by ``wgpu-3dgs-viewer``.

Run from the repository root::

    python tools/generate_house_3dgs.py

The generator has no third-party dependencies and writes the PLY plus a
sidecar summary containing the byte/hash and semantic point counts.
"""

from __future__ import annotations

import hashlib
import json
import math
import struct
from collections import Counter
from dataclasses import dataclass
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
OUT_DIR = ROOT / "assets" / "environments" / "house_3dgs"
PLY_PATH = OUT_DIR / "house_3dgs_fixture.ply"
METADATA_PATH = OUT_DIR / "house_3dgs_fixture.metadata.json"
GENERATOR_PATH = Path(__file__).resolve()

SH_C0 = 0.2820948
PLY_HEADER = """ply
format binary_little_endian 1.0
element vertex {count}
property float x
property float y
property float z
property float nx
property float ny
property float nz
property float f_dc_0
property float f_dc_1
property float f_dc_2
property float opacity
property float scale_0
property float scale_1
property float scale_2
property float rot_0
property float rot_1
property float rot_2
property float rot_3
end_header
""".encode("ascii")


@dataclass(frozen=True)
class Point:
    x: float
    y: float
    z: float
    color: tuple[float, float, float]
    opacity: float = 0.96
    scale: tuple[float, float, float] = (0.055, 0.055, 0.055)
    group: str = "unknown"


class Cloud:
    def __init__(self) -> None:
        self.points: list[Point] = []

    def add(self, point: Point) -> None:
        self.points.append(point)

    def plane_grid(
        self,
        axis_a: str,
        axis_b: str,
        fixed_axis: str,
        fixed: float,
        a0: float,
        a1: float,
        b0: float,
        b1: float,
        step: float,
        color: tuple[float, float, float],
        group: str,
        opacity: float = 0.96,
        scale: float = 0.055,
    ) -> None:
        """Add a deterministic, axis-aligned rectangular surface."""
        a_count = max(1, int(round(abs(a1 - a0) / step)))
        b_count = max(1, int(round(abs(b1 - b0) / step)))
        for ia in range(a_count + 1):
            a = a0 + (a1 - a0) * ia / a_count
            for ib in range(b_count + 1):
                b = b0 + (b1 - b0) * ib / b_count
                coordinates = {axis_a: a, axis_b: b, fixed_axis: fixed}
                self.add(
                    Point(
                        coordinates["x"],
                        coordinates["y"],
                        coordinates["z"],
                        color,
                        opacity,
                        (scale, scale, scale),
                        group,
                    )
                )

    def box(
        self,
        center: tuple[float, float, float],
        size: tuple[float, float, float],
        color: tuple[float, float, float],
        group: str,
        step: float = 0.09,
        opacity: float = 0.96,
        scale: float | None = None,
    ) -> None:
        """Add all six surfaces of a box, retaining crisp furniture edges."""
        cx, cy, cz = center
        sx, sy, sz = (dimension / 2.0 for dimension in size)
        splat_scale = scale if scale is not None else min(step * 0.45, 0.06)
        self.plane_grid(
            "x",
            "y",
            "z",
            cz - sz,
            cx - sx,
            cx + sx,
            cy - sy,
            cy + sy,
            step,
            color,
            group,
            opacity,
            splat_scale,
        )
        self.plane_grid(
            "x",
            "y",
            "z",
            cz + sz,
            cx - sx,
            cx + sx,
            cy - sy,
            cy + sy,
            step,
            color,
            group,
            opacity,
            splat_scale,
        )
        self.plane_grid(
            "x",
            "z",
            "y",
            cy - sy,
            cx - sx,
            cx + sx,
            cz - sz,
            cz + sz,
            step,
            color,
            group,
            opacity,
            splat_scale,
        )
        self.plane_grid(
            "x",
            "z",
            "y",
            cy + sy,
            cx - sx,
            cx + sx,
            cz - sz,
            cz + sz,
            step,
            color,
            group,
            opacity,
            splat_scale,
        )
        self.plane_grid(
            "y",
            "z",
            "x",
            cx - sx,
            cy - sy,
            cy + sy,
            cz - sz,
            cz + sz,
            step,
            color,
            group,
            opacity,
            splat_scale,
        )
        self.plane_grid(
            "y",
            "z",
            "x",
            cx + sx,
            cy - sy,
            cy + sy,
            cz - sz,
            cz + sz,
            step,
            color,
            group,
            opacity,
            splat_scale,
        )

    def cylinder(
        self,
        center: tuple[float, float, float],
        radius: float,
        height: float,
        color: tuple[float, float, float],
        group: str,
        radial_steps: int = 18,
        height_steps: int = 5,
        opacity: float = 0.96,
        scale: float = 0.045,
    ) -> None:
        cx, cy, cz = center
        for iz in range(height_steps + 1):
            y = cy + height * iz / height_steps
            for ir in range(radial_steps):
                angle = 2.0 * math.pi * ir / radial_steps
                self.add(
                    Point(
                        cx + radius * math.cos(angle),
                        y,
                        cz + radius * math.sin(angle),
                        color,
                        opacity,
                        (scale, scale, scale),
                        group,
                    )
                )

    def sphere(
        self,
        center: tuple[float, float, float],
        radius: float,
        color: tuple[float, float, float],
        group: str,
        latitude_steps: int = 8,
        longitude_steps: int = 16,
        opacity: float = 0.96,
        scale: float = 0.06,
    ) -> None:
        cx, cy, cz = center
        for ilat in range(latitude_steps + 1):
            latitude = -math.pi / 2.0 + math.pi * ilat / latitude_steps
            ring_radius = radius * math.cos(latitude)
            y = cy + radius * math.sin(latitude)
            for ilon in range(longitude_steps):
                longitude = 2.0 * math.pi * ilon / longitude_steps
                self.add(
                    Point(
                        cx + ring_radius * math.cos(longitude),
                        y,
                        cz + ring_radius * math.sin(longitude),
                        color,
                        opacity,
                        (scale, scale, scale),
                        group,
                    )
                )


def add_house(cloud: Cloud) -> None:
    # Room shell: Y-up, metres.  The open front is intentional so a camera
    # orbit can see both the back window and the furniture arrangement.
    floor_light = (0.72, 0.48, 0.27)
    floor_dark = (0.43, 0.24, 0.13)
    wall = (0.84, 0.83, 0.78)
    wall_shadow = (0.66, 0.67, 0.64)
    cloud.plane_grid("x", "z", "y", 0.0, -3.2, 3.2, -3.2, 3.2, 0.12, floor_light, "floor", scale=0.06)
    # Deterministic alternating boards make the room legible in a small GIF.
    for ix in range(-16, 16):
        for iz in range(-16, 16):
            if (ix + iz) % 5 == 0:
                x0, x1 = ix * 0.2, (ix + 1) * 0.2
                z0, z1 = iz * 0.2, (iz + 1) * 0.2
                cloud.plane_grid("x", "z", "y", 0.008, x0, x1, z0, z1, 0.1, floor_dark, "floor", scale=0.045)

    # Back wall with a large window opening, plus side walls and ceiling edge.
    cloud.plane_grid("x", "y", "z", -3.2, -3.2, -1.35, 0.0, 3.0, 0.12, wall, "wall")
    cloud.plane_grid("x", "y", "z", -3.2, 1.35, 3.2, 0.0, 3.0, 0.12, wall, "wall")
    cloud.plane_grid("x", "y", "z", -3.2, -1.35, 1.35, 0.0, 0.95, 0.12, wall_shadow, "wall")
    cloud.plane_grid("x", "y", "z", -3.2, -1.35, 1.35, 2.45, 3.0, 0.12, wall_shadow, "wall")
    cloud.plane_grid("y", "z", "x", -3.2, 0.0, 3.0, -3.2, 3.2, 0.12, wall, "wall")
    cloud.plane_grid("y", "z", "x", 3.2, 0.0, 3.0, -3.2, 3.2, 0.12, wall_shadow, "wall")
    cloud.plane_grid("x", "z", "y", 3.0, -3.2, 3.2, -3.2, 3.2, 0.16, wall, "ceiling", opacity=0.7)

    # Window glazing and a dark wood frame; glazing is semi-transparent but
    # still carries a dense depth proxy for RGB-D captures.
    cloud.plane_grid("x", "y", "z", -3.16, -1.18, 1.18, 1.05, 2.38, 0.09, (0.20, 0.54, 0.76), "window", opacity=0.82, scale=0.05)
    frame = (0.16, 0.10, 0.07)
    cloud.box((-1.25, 1.72, -3.05), (0.12, 1.52, 0.24), frame, "window_frame", step=0.07)
    cloud.box((1.25, 1.72, -3.05), (0.12, 1.52, 0.24), frame, "window_frame", step=0.07)
    cloud.box((0.0, 1.02, -3.05), (2.62, 0.12, 0.24), frame, "window_frame", step=0.07)
    cloud.box((0.0, 2.42, -3.05), (2.62, 0.12, 0.24), frame, "window_frame", step=0.07)
    cloud.box((0.0, 1.72, -3.05), (0.08, 1.42, 0.24), frame, "window_frame", step=0.07)

    # A warm rug anchors the mobile manipulator's route.
    cloud.box((0.0, 0.025, 0.85), (2.55, 0.045, 1.65), (0.58, 0.18, 0.10), "rug", step=0.08, scale=0.045)
    cloud.box((0.0, 0.04, 0.85), (2.3, 0.035, 1.4), (0.76, 0.38, 0.15), "rug", step=0.08, scale=0.04)

    # Sofa with contrasting cushions.
    cloud.box((-1.55, 0.43, 0.55), (2.0, 0.62, 0.76), (0.14, 0.23, 0.29), "sofa", step=0.09)
    cloud.box((-1.55, 0.94, 0.84), (2.0, 0.75, 0.18), (0.11, 0.18, 0.23), "sofa", step=0.09)
    cloud.box((-1.55, 0.79, 0.42), (1.75, 0.13, 0.62), (0.78, 0.69, 0.48), "cushion", step=0.08)
    cloud.box((-2.34, 0.30, 0.55), (0.16, 0.62, 0.82), (0.10, 0.17, 0.21), "sofa", step=0.08)
    cloud.box((-0.76, 0.30, 0.55), (0.16, 0.62, 0.82), (0.10, 0.17, 0.21), "sofa", step=0.08)

    # Coffee table with four legs.
    cloud.box((0.10, 0.53, 0.75), (1.35, 0.13, 0.72), (0.33, 0.18, 0.09), "coffee_table", step=0.07)
    for x in (-0.43, 0.63):
        for z in (0.48, 1.02):
            cloud.box((x, 0.27, z), (0.09, 0.52, 0.09), (0.20, 0.10, 0.05), "coffee_table", step=0.06)

    # Kitchen island and stools in the back-right of the room.
    cloud.box((1.45, 0.56, -1.95), (2.25, 0.85, 0.78), (0.24, 0.27, 0.30), "kitchen_island", step=0.09)
    cloud.box((1.45, 1.02, -1.95), (2.38, 0.10, 0.88), (0.78, 0.80, 0.72), "kitchen_counter", step=0.07)
    for x in (0.75, 2.15):
        cloud.cylinder((x, 0.04, -1.10), 0.26, 0.72, (0.12, 0.15, 0.18), "stool", scale=0.045)
        cloud.cylinder((x, 0.73, -1.10), 0.36, 0.08, (0.70, 0.31, 0.10), "stool", scale=0.04)

    # Tall cabinet and television block on the right wall.
    cloud.box((2.48, 1.06, -2.72), (1.15, 2.1, 0.34), (0.34, 0.21, 0.12), "cabinet", step=0.09)
    cloud.box((2.48, 1.22, -2.51), (0.86, 0.78, 0.08), (0.04, 0.05, 0.06), "television", step=0.06, scale=0.035)
    cloud.box((2.48, 0.48, -2.49), (0.28, 0.09, 0.12), (0.75, 0.48, 0.12), "console", step=0.05)

    # Plant silhouette, intentionally point-dense so the window/plant colours
    # remain distinguishable in both the PLY viewer and proxy-depth raster.
    cloud.cylinder((-2.48, 0.05, -1.95), 0.34, 0.48, (0.25, 0.15, 0.08), "plant_pot", scale=0.045)
    cloud.sphere((-2.48, 1.18, -1.95), 0.62, (0.10, 0.36, 0.16), "plant_leaves", scale=0.055)
    cloud.sphere((-2.78, 1.38, -1.78), 0.38, (0.15, 0.46, 0.20), "plant_leaves", scale=0.05)
    cloud.sphere((-2.18, 1.44, -2.02), 0.34, (0.12, 0.42, 0.18), "plant_leaves", scale=0.05)


def encode_color(color: tuple[float, float, float]) -> tuple[float, float, float]:
    return tuple((channel - 0.5) / SH_C0 for channel in color)


def encode_alpha(opacity: float) -> float:
    bounded = min(1.0 - 1.0e-4, max(1.0e-4, opacity))
    return math.log(bounded / (1.0 - bounded))


def write_fixture(cloud: Cloud) -> dict[str, object]:
    header = PLY_HEADER.replace(b"{count}", str(len(cloud.points)).encode("ascii"))
    with PLY_PATH.open("wb") as handle:
        handle.write(header)
        for point in cloud.points:
            fdc = encode_color(point.color)
            payload = (*((point.x, point.y, point.z)), 0.0, 0.0, 1.0, *fdc, encode_alpha(point.opacity), *map(math.log, point.scale), 1.0, 0.0, 0.0, 0.0)
            handle.write(struct.pack("<17f", *payload))

    digest = hashlib.sha256(PLY_PATH.read_bytes()).hexdigest()
    groups = Counter(point.group for point in cloud.points)
    metadata = {
        "kind": "rne_house_3dgs_fixture_metadata",
        "schema_version": 1,
        "environment_id": "house.indoor.fixture.v1",
        "generator": "tools/generate_house_3dgs.py",
        "generator_sha256": hashlib.sha256(GENERATOR_PATH.read_bytes()).hexdigest(),
        "ply": PLY_PATH.name,
        "ply_sha256": digest,
        "ply_bytes": PLY_PATH.stat().st_size,
        "point_count": len(cloud.points),
        "semantic_groups": dict(sorted(groups.items())),
        "coordinate_system": "right-handed Y-up; metres; camera forward is -Z",
        "third_party_capture": False,
    }
    METADATA_PATH.write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
    return metadata


def main() -> None:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    cloud = Cloud()
    add_house(cloud)
    metadata = write_fixture(cloud)
    print(json.dumps(metadata, indent=2))


if __name__ == "__main__":
    main()
