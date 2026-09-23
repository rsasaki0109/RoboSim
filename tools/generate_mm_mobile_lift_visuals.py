#!/usr/bin/env python3
"""Generate the authored visual pack for ``mm_mobile_lift``.

The repository deliberately does not depend on Blender for its checked-in
visual evidence.  This small, deterministic glTF writer keeps the visual
asset reproducible on CI and in offline worktrees.  It generates link-scoped
binary glTF files with rounded/bevelled surfaces, multiple PBR material
slots, and embedded PNG maps.  The meshes are visual-only; the URDF remains
the owner of collision, joint, and inertia data.

Run from the repository root::

    python tools/generate_mm_mobile_lift_visuals.py

The output is intentionally stable: no timestamps, random IDs, host paths,
or floating-point serialization outside Python's deterministic ``repr`` are
written to the GLB files.
"""

from __future__ import annotations

import argparse
import binascii
import hashlib
import json
import math
import struct
import sys
import zlib
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Iterable, Sequence


ROOT = Path(__file__).resolve().parents[1]
OUTPUT_DIR = ROOT / "assets" / "robots" / "mm_mobile_lift" / "meshes"


Vec3 = tuple[float, float, float]
Vec2 = tuple[float, float]


@dataclass(frozen=True)
class MaterialSpec:
    name: str
    color: tuple[float, float, float, float]
    metallic: float
    roughness: float
    emissive: tuple[float, float, float]


MATERIALS: tuple[MaterialSpec, ...] = (
    MaterialSpec("anodized_navy", (0.055, 0.16, 0.38, 1.0), 0.72, 0.26, (0.0, 0.0, 0.0)),
    MaterialSpec("signal_orange", (0.92, 0.22, 0.035, 1.0), 0.40, 0.30, (0.0, 0.0, 0.0)),
    MaterialSpec("machined_aluminum", (0.54, 0.62, 0.70, 1.0), 0.88, 0.22, (0.0, 0.0, 0.0)),
    MaterialSpec("elastomer", (0.018, 0.024, 0.035, 1.0), 0.02, 0.80, (0.0, 0.0, 0.0)),
    MaterialSpec("status_cyan", (0.02, 0.30, 0.55, 1.0), 0.16, 0.20, (0.0, 0.09, 0.34)),
)


@dataclass
class MeshBuilder:
    """One material-homogeneous indexed mesh primitive."""

    positions: list[Vec3] = field(default_factory=list)
    normals: list[Vec3] = field(default_factory=list)
    texcoords: list[Vec2] = field(default_factory=list)
    indices: list[int] = field(default_factory=list)

    def _vertex(self, position: Vec3, normal: Vec3, uv: Vec2) -> int:
        self.positions.append(tuple(float(v) for v in position))
        self.normals.append(tuple(float(v) for v in normal))
        self.texcoords.append(tuple(float(v) for v in uv))
        return len(self.positions) - 1

    def _triangle(self, a: int, b: int, c: int) -> None:
        self.indices.extend((a, b, c))

    def _grid(
        self,
        rows: int,
        cols: int,
        position: Callable[[int, int], Vec3],
        normal: Callable[[int, int], Vec3],
    ) -> None:
        vertices: list[list[int]] = []
        for row in range(rows + 1):
            row_vertices: list[int] = []
            for col in range(cols + 1):
                uv = (col / max(cols, 1), row / max(rows, 1))
                row_vertices.append(self._vertex(position(row, col), normal(row, col), uv))
            vertices.append(row_vertices)
        for row in range(rows):
            for col in range(cols):
                a = vertices[row][col]
                b = vertices[row][col + 1]
                c = vertices[row + 1][col + 1]
                d = vertices[row + 1][col]
                self._triangle(a, b, c)
                self._triangle(a, c, d)

    def add_box(self, center: Vec3, size: Vec3) -> None:
        """Add a six-sided, flat-shaded box."""

        cx, cy, cz = center
        hx, hy, hz = (value / 2.0 for value in size)
        faces = (
            ((1.0, 0.0, 0.0), ((cx + hx, cy - hy, cz - hz), (cx + hx, cy + hy, cz - hz), (cx + hx, cy + hy, cz + hz), (cx + hx, cy - hy, cz + hz))),
            ((-1.0, 0.0, 0.0), ((cx - hx, cy - hy, cz + hz), (cx - hx, cy + hy, cz + hz), (cx - hx, cy + hy, cz - hz), (cx - hx, cy - hy, cz - hz))),
            ((0.0, 1.0, 0.0), ((cx - hx, cy + hy, cz - hz), (cx - hx, cy + hy, cz + hz), (cx + hx, cy + hy, cz + hz), (cx + hx, cy + hy, cz - hz))),
            ((0.0, -1.0, 0.0), ((cx - hx, cy - hy, cz + hz), (cx - hx, cy - hy, cz - hz), (cx + hx, cy - hy, cz - hz), (cx + hx, cy - hy, cz + hz))),
            ((0.0, 0.0, 1.0), ((cx - hx, cy - hy, cz + hz), (cx + hx, cy - hy, cz + hz), (cx + hx, cy + hy, cz + hz), (cx - hx, cy + hy, cz + hz))),
            ((0.0, 0.0, -1.0), ((cx + hx, cy - hy, cz - hz), (cx - hx, cy - hy, cz - hz), (cx - hx, cy + hy, cz - hz), (cx + hx, cy + hy, cz - hz))),
        )
        for normal, corners in faces:
            ids = [self._vertex(corner, normal, uv) for corner, uv in zip(corners, ((0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)))]
            self._triangle(ids[0], ids[1], ids[2])
            self._triangle(ids[0], ids[2], ids[3])

    def add_rounded_box(self, center: Vec3, size: Vec3, bevel: float, segments: int) -> None:
        """Add a watertight-looking box with planar faces and rounded edges."""

        cx, cy, cz = center
        hx, hy, hz = (value / 2.0 for value in size)
        b = min(bevel, hx * 0.48, hy * 0.48, hz * 0.48)
        face_steps = max(1, segments // 2)
        edge_steps = max(2, segments)
        sphere_steps = max(2, segments)

        # Six inset planar faces.
        self._grid(
            face_steps,
            face_steps,
            lambda r, c: (cx + hx, cy - hy + b + (2 * (hy - b)) * r / face_steps, cz - hz + b + (2 * (hz - b)) * c / face_steps),
            lambda _r, _c: (1.0, 0.0, 0.0),
        )
        self._grid(
            face_steps,
            face_steps,
            lambda r, c: (cx - hx, cy - hy + b + (2 * (hy - b)) * r / face_steps, cz + hz - b - (2 * (hz - b)) * c / face_steps),
            lambda _r, _c: (-1.0, 0.0, 0.0),
        )
        self._grid(
            face_steps,
            face_steps,
            lambda r, c: (cx - hx + b + (2 * (hx - b)) * c / face_steps, cy + hy, cz - hz + b + (2 * (hz - b)) * r / face_steps),
            lambda _r, _c: (0.0, 1.0, 0.0),
        )
        self._grid(
            face_steps,
            face_steps,
            lambda r, c: (cx - hx + b + (2 * (hx - b)) * c / face_steps, cy - hy, cz + hz - b - (2 * (hz - b)) * r / face_steps),
            lambda _r, _c: (0.0, -1.0, 0.0),
        )
        self._grid(
            face_steps,
            face_steps,
            lambda r, c: (cx - hx + b + (2 * (hx - b)) * c / face_steps, cy - hy + b + (2 * (hy - b)) * r / face_steps, cz + hz),
            lambda _r, _c: (0.0, 0.0, 1.0),
        )
        self._grid(
            face_steps,
            face_steps,
            lambda r, c: (cx + hx - b - (2 * (hx - b)) * c / face_steps, cy - hy + b + (2 * (hy - b)) * r / face_steps, cz - hz),
            lambda _r, _c: (0.0, 0.0, -1.0),
        )

        # Twelve quarter-cylinder edge strips.
        def edge_x(sy: float, sz: float) -> None:
            cy_edge, cz_edge = cy + sy * (hy - b), cz + sz * (hz - b)
            self._grid(
                1,
                edge_steps,
                lambda r, c: (
                    cx - hx + (2 * hx) * c / edge_steps,
                    cy_edge + sy * b * math.cos((math.pi / 2) * r),
                    cz_edge + sz * b * math.sin((math.pi / 2) * r),
                ),
                lambda r, _c: (0.0, sy * math.cos((math.pi / 2) * r), sz * math.sin((math.pi / 2) * r)),
            )

        def edge_y(sx: float, sz: float) -> None:
            cx_edge, cz_edge = cx + sx * (hx - b), cz + sz * (hz - b)
            self._grid(
                1,
                edge_steps,
                lambda r, c: (
                    cx_edge + sx * b * math.cos((math.pi / 2) * r),
                    cy - hy + (2 * hy) * c / edge_steps,
                    cz_edge + sz * b * math.sin((math.pi / 2) * r),
                ),
                lambda r, _c: (sx * math.cos((math.pi / 2) * r), 0.0, sz * math.sin((math.pi / 2) * r)),
            )

        def edge_z(sx: float, sy: float) -> None:
            cx_edge, cy_edge = cx + sx * (hx - b), cy + sy * (hy - b)
            self._grid(
                1,
                edge_steps,
                lambda r, c: (
                    cx_edge + sx * b * math.cos((math.pi / 2) * r),
                    cy_edge + sy * b * math.sin((math.pi / 2) * r),
                    cz - hz + (2 * hz) * c / edge_steps,
                ),
                lambda r, _c: (sx * math.cos((math.pi / 2) * r), sy * math.sin((math.pi / 2) * r), 0.0),
            )

        for sy in (-1.0, 1.0):
            for sz in (-1.0, 1.0):
                edge_x(sy, sz)
        for sx in (-1.0, 1.0):
            for sz in (-1.0, 1.0):
                edge_y(sx, sz)
        for sx in (-1.0, 1.0):
            for sy in (-1.0, 1.0):
                edge_z(sx, sy)

        # Eight spherical corner patches.
        for sx in (-1.0, 1.0):
            for sy in (-1.0, 1.0):
                for sz in (-1.0, 1.0):
                    corner = (cx + sx * (hx - b), cy + sy * (hy - b), cz + sz * (hz - b))

                    def corner_position(r: int, c: int, corner: Vec3 = corner, sx: float = sx, sy: float = sy, sz: float = sz) -> Vec3:
                        theta = (math.pi / 2) * r / sphere_steps
                        phi = (math.pi / 2) * c / sphere_steps
                        return (
                            corner[0] + sx * b * math.sin(theta) * math.cos(phi),
                            corner[1] + sy * b * math.sin(theta) * math.sin(phi),
                            corner[2] + sz * b * math.cos(theta),
                        )

                    def corner_normal(r: int, c: int, sx: float = sx, sy: float = sy, sz: float = sz) -> Vec3:
                        theta = (math.pi / 2) * r / sphere_steps
                        phi = (math.pi / 2) * c / sphere_steps
                        return (
                            sx * math.sin(theta) * math.cos(phi),
                            sy * math.sin(theta) * math.sin(phi),
                            sz * math.cos(theta),
                        )

                    self._grid(sphere_steps, sphere_steps, corner_position, corner_normal)

    def add_cylinder(self, center: Vec3, radius: float, length: float, axis: str, segments: int) -> None:
        """Add a capped cylinder aligned to one local coordinate axis."""

        cx, cy, cz = center
        half = length / 2.0
        side_vertices: list[list[int]] = [[], []]
        for end, coordinate in enumerate((-half, half)):
            for index in range(segments + 1):
                angle = 2 * math.pi * index / segments
                circle = (radius * math.cos(angle), radius * math.sin(angle))
                if axis == "x":
                    position = (cx + coordinate, cy + circle[0], cz + circle[1])
                    normal = (0.0, circle[0] / radius, circle[1] / radius)
                elif axis == "y":
                    position = (cx + circle[0], cy + coordinate, cz + circle[1])
                    normal = (circle[0] / radius, 0.0, circle[1] / radius)
                else:
                    position = (cx + circle[0], cy + circle[1], cz + coordinate)
                    normal = (circle[0] / radius, circle[1] / radius, 0.0)
                side_vertices[end].append(self._vertex(position, normal, (index / segments, end)))
        for index in range(segments):
            a, b = side_vertices[0][index], side_vertices[0][index + 1]
            c, d = side_vertices[1][index + 1], side_vertices[1][index]
            self._triangle(a, b, c)
            self._triangle(a, c, d)
        for end, coordinate in enumerate((-half, half)):
            normal_axis = -1.0 if end == 0 else 1.0
            if axis == "x":
                normal = (normal_axis, 0.0, 0.0)
                position_center = (cx + coordinate, cy, cz)
            elif axis == "y":
                normal = (0.0, normal_axis, 0.0)
                position_center = (cx, cy + coordinate, cz)
            else:
                normal = (0.0, 0.0, normal_axis)
                position_center = (cx, cy, cz + coordinate)
            center_id = self._vertex(position_center, normal, (0.5, 0.5))
            rim: list[int] = []
            for index in range(segments + 1):
                angle = 2 * math.pi * index / segments
                circle = (radius * math.cos(angle), radius * math.sin(angle))
                if axis == "x":
                    position = (cx + coordinate, cy + circle[0], cz + circle[1])
                elif axis == "y":
                    position = (cx + circle[0], cy + coordinate, cz + circle[1])
                else:
                    position = (cx + circle[0], cy + circle[1], cz + coordinate)
                rim.append(self._vertex(position, normal, (0.5 + 0.5 * math.cos(angle), 0.5 + 0.5 * math.sin(angle))))
            for index in range(segments):
                if end == 0:
                    self._triangle(center_id, rim[index + 1], rim[index])
                else:
                    self._triangle(center_id, rim[index], rim[index + 1])

    def add_uv_sphere(self, center: Vec3, radius: float, segments: int, rings: int) -> None:
        """Add a smooth UV sphere."""

        cx, cy, cz = center

        def position(row: int, col: int) -> Vec3:
            theta = math.pi * row / rings
            phi = 2 * math.pi * col / segments
            nx = math.sin(theta) * math.cos(phi)
            ny = math.cos(theta)
            nz = math.sin(theta) * math.sin(phi)
            return (cx + radius * nx, cy + radius * ny, cz + radius * nz)

        def normal(row: int, col: int) -> Vec3:
            theta = math.pi * row / rings
            phi = 2 * math.pi * col / segments
            return (math.sin(theta) * math.cos(phi), math.cos(theta), math.sin(theta) * math.sin(phi))

        self._grid(rings, segments, position, normal)

    def add_torus(self, center: Vec3, major_radius: float, minor_radius: float, segments: int, minor_segments: int) -> None:
        """Add a torus around the local Z axis."""

        cx, cy, cz = center

        def position(row: int, col: int) -> Vec3:
            u = 2 * math.pi * col / segments
            v = 2 * math.pi * row / minor_segments
            ring = major_radius + minor_radius * math.cos(v)
            return (cx + ring * math.cos(u), cy + ring * math.sin(u), cz + minor_radius * math.sin(v))

        def normal(row: int, col: int) -> Vec3:
            u = 2 * math.pi * col / segments
            v = 2 * math.pi * row / minor_segments
            return (math.cos(v) * math.cos(u), math.cos(v) * math.sin(u), math.sin(v))

        self._grid(minor_segments, segments, position, normal)


def add_gear(builder: MeshBuilder, center: Vec3, radius: float, thickness: float, axis: str, material: int, segments: int) -> None:
    """Add a compact toothed-looking actuator ring using a torus and hub."""

    # Rings are authored in XY and rotate with the link's joint frame.  A
    # second cylinder provides a clean metallic shoulder without introducing a
    # physics shape.
    if axis == "z":
        builder.add_torus(center, radius * 0.72, radius * 0.16, segments, max(4, segments // 6))
    else:
        builder.add_cylinder(center, radius * 0.72, thickness, axis, segments)
    builder.add_cylinder(center, radius * 0.34, thickness * 1.25, axis, segments)


def build_link(name: str, lod: int) -> dict[int, MeshBuilder]:
    """Build one link in its URDF link frame.

    ``lod=0`` keeps the curved edge detail used by the README hero.  ``lod=1``
    retains the silhouette and actuator cues while reducing radial samples.
    """

    coarse = lod == 1
    radial = 12 if coarse else 24
    smooth = 2 if coarse else 4
    builders = {index: MeshBuilder() for index in range(len(MATERIALS))}
    primary = builders[0]
    orange = builders[1]
    aluminum = builders[2]
    rubber = builders[3]
    cyan = builders[4]

    if name == "base_link":
        # The outer shell is deliberately layered around the existing URDF
        # envelope: the visual reads as a fabricated mobile base instead of a
        # single box, while collision and inertial data remain exclusively in
        # the physics asset.
        primary.add_rounded_box((0.0, 0.0, 0.0), (0.50, 0.30, 0.40), 0.035, smooth)
        primary.add_rounded_box((0.0, -0.115, 0.0), (0.47, 0.050, 0.34), 0.018, smooth)
        primary.add_rounded_box((0.0, 0.165, 0.0), (0.43, 0.026, 0.34), 0.008, smooth)
        rubber.add_rounded_box((0.0, -0.145, 0.0), (0.45, 0.026, 0.29), 0.010, smooth)
        rubber.add_rounded_box((-0.235, -0.020, 0.0), (0.022, 0.235, 0.29), 0.008, smooth)
        rubber.add_rounded_box((0.235, -0.020, 0.0), (0.022, 0.235, 0.29), 0.008, smooth)
        orange.add_rounded_box((-0.242, 0.015, 0.0), (0.025, 0.19, 0.35), 0.009, smooth)
        # Open lift tower: two spaced rails in local z, with visible upper
        # and lower crossmembers.  This keeps the visual mechanism legible in
        # the 3DGS capture instead of collapsing into one grey slab.
        for z in (-0.052, 0.052):
            aluminum.add_rounded_box((0.0, 0.45, z), (0.034, 0.70, 0.022), 0.006, smooth)
        aluminum.add_rounded_box((0.0, 0.125, 0.0), (0.10, 0.035, 0.14), 0.008, smooth)
        aluminum.add_rounded_box((0.0, 0.775, 0.0), (0.10, 0.035, 0.14), 0.008, smooth)
        rubber.add_rounded_box((0.028, 0.45, 0.0), (0.010, 0.62, 0.010), 0.003, smooth)
        orange.add_cylinder((0.028, 0.145, 0.0), 0.012, 0.030, "y", radial)
        orange.add_cylinder((0.028, 0.755, 0.0), 0.012, 0.030, "y", radial)
        aluminum.add_rounded_box((0.0, 0.265, 0.0), (0.16, 0.060, 0.15), 0.014, smooth)
        orange.add_rounded_box((0.0, 0.235, 0.0), (0.13, 0.018, 0.12), 0.004, smooth)
        cyan.add_rounded_box((-0.258, 0.05, 0.0), (0.008, 0.10, 0.20), 0.002, smooth)
        cyan.add_uv_sphere((-0.18, 0.18, 0.0), 0.018, radial, max(6, radial // 2))
        for z in (-0.13, 0.13):
            aluminum.add_cylinder((0.0, -0.025, z), 0.025, 0.010, "z", radial)
            orange.add_cylinder((0.0, -0.025, z), 0.010, 0.013, "z", radial)
        for x in (-0.13, -0.08, -0.03, 0.02, 0.07, 0.12):
            rubber.add_box((x, 0.181, 0.0), (0.018, 0.006, 0.23))
    elif name in ("left_wheel", "right_wheel"):
        rubber.add_cylinder((0.0, 0.0, 0.0), 0.100, 0.050, "z", radial)
        rubber.add_torus((0.0, 0.0, 0.0), 0.084, 0.012, radial, max(5, radial // 6))
        aluminum.add_cylinder((0.0, 0.0, 0.0), 0.057, 0.056, "z", radial)
        aluminum.add_torus((0.0, 0.0, 0.028), 0.045, 0.004, radial, max(4, radial // 8))
        orange.add_cylinder((0.0, 0.0, 0.0), 0.023, 0.060, "z", radial)
        for index in range(8 if coarse else 12):
            angle = 2 * math.pi * index / (8 if coarse else 12)
            x, y = 0.068 * math.cos(angle), 0.068 * math.sin(angle)
            rubber.add_box((x, y, 0.0), (0.010, 0.024, 0.055))
    elif name == "torso_link":
        primary.add_rounded_box((0.0, 0.0, 0.0), (0.14, 0.14, 0.14), 0.018, smooth)
        aluminum.add_rounded_box((0.0, 0.073, 0.0), (0.105, 0.018, 0.105), 0.004, smooth)
        orange.add_box((-0.073, 0.0, 0.0), (0.008, 0.09, 0.085))
        cyan.add_cylinder((0.0, 0.0, 0.0), 0.035, 0.152, "z", radial)
        aluminum.add_cylinder((0.0, 0.0, 0.0), 0.019, 0.17, "z", radial)
        for z in (-0.055, 0.055):
            orange.add_cylinder((0.0, 0.035, z), 0.010, 0.012, "x", radial)
    elif name == "upper_arm_link":
        orange.add_rounded_box((0.25, 0.0, 0.0), (0.50, 0.070, 0.070), 0.015, smooth)
        aluminum.add_rounded_box((0.25, 0.0, 0.038), (0.43, 0.022, 0.026), 0.006, smooth)
        aluminum.add_rounded_box((0.25, 0.0, -0.038), (0.43, 0.022, 0.026), 0.006, smooth)
        orange.add_rounded_box((0.25, 0.0, 0.052), (0.42, 0.016, 0.022), 0.004, smooth)
        primary.add_rounded_box((0.0, 0.0, 0.0), (0.13, 0.095, 0.105), 0.018, smooth)
        primary.add_rounded_box((0.50, 0.0, 0.0), (0.11, 0.088, 0.098), 0.016, smooth)
        aluminum.add_cylinder((0.0, 0.0, 0.0), 0.071, 0.10, "y", radial)
        aluminum.add_cylinder((0.50, 0.0, 0.0), 0.054, 0.085, "y", radial)
        add_gear(aluminum, (0.0, 0.0, 0.0), 0.069, 0.09, "y", 2, radial)
        cyan.add_box((0.25, -0.040, 0.0), (0.22, 0.008, 0.022))
        for x in (0.12, 0.22, 0.32, 0.42):
            aluminum.add_cylinder((x, 0.0, -0.040), 0.006, 0.008, "z", max(8, radial // 2))
    elif name == "forearm_link":
        orange.add_rounded_box((0.20, 0.0, 0.0), (0.40, 0.055, 0.055), 0.012, smooth)
        aluminum.add_rounded_box((0.20, 0.0, 0.031), (0.34, 0.018, 0.020), 0.005, smooth)
        aluminum.add_rounded_box((0.20, 0.0, -0.031), (0.34, 0.018, 0.020), 0.005, smooth)
        orange.add_rounded_box((0.21, 0.0, 0.039), (0.30, 0.012, 0.016), 0.003, smooth)
        primary.add_rounded_box((0.0, 0.0, 0.0), (0.11, 0.080, 0.090), 0.015, smooth)
        primary.add_rounded_box((0.40, 0.0, 0.0), (0.10, 0.074, 0.084), 0.014, smooth)
        aluminum.add_cylinder((0.0, 0.0, 0.0), 0.057, 0.080, "y", radial)
        aluminum.add_cylinder((0.40, 0.0, 0.0), 0.047, 0.073, "y", radial)
        add_gear(aluminum, (0.0, 0.0, 0.0), 0.055, 0.075, "y", 2, radial)
        rubber.add_cylinder((0.20, 0.0, 0.0), 0.014, 0.34, "x", radial)
        cyan.add_box((0.20, -0.032, 0.0), (0.18, 0.007, 0.014))
    elif name == "wrist_link":
        aluminum.add_cylinder((0.0, 0.0, 0.0), 0.035, 0.040, "z", radial)
        aluminum.add_torus((0.0, 0.0, -0.018), 0.030, 0.004, radial, max(4, radial // 8))
        aluminum.add_torus((0.0, 0.0, 0.018), 0.030, 0.004, radial, max(4, radial // 8))
        primary.add_rounded_box((0.0, 0.0, 0.0), (0.068, 0.058, 0.060), 0.010, smooth)
        primary.add_rounded_box((0.0, 0.0, 0.040), (0.082, 0.050, 0.032), 0.008, smooth)
        primary.add_rounded_box((0.0, 0.0, -0.040), (0.082, 0.050, 0.032), 0.008, smooth)
        orange.add_cylinder((0.0, 0.0, 0.0), 0.012, 0.048, "z", radial)
        cyan.add_uv_sphere((0.0, 0.031, 0.0), 0.008, radial, max(6, radial // 2))
    elif name == "gripper_base_link":
        primary.add_rounded_box((0.0, 0.0, 0.0), (0.060, 0.040, 0.120), 0.009, smooth)
        primary.add_rounded_box((0.0, -0.020, 0.0), (0.086, 0.050, 0.104), 0.012, smooth)
        aluminum.add_rounded_box((0.0, 0.024, 0.0), (0.043, 0.012, 0.094), 0.003, smooth)
        aluminum.add_rounded_box((0.0, -0.050, 0.0), (0.070, 0.018, 0.090), 0.005, smooth)
        aluminum.add_cylinder((0.0, -0.002, -0.052), 0.013, 0.048, "z", radial)
        aluminum.add_cylinder((0.0, -0.002, 0.052), 0.013, 0.048, "z", radial)
        orange.add_box((0.0, -0.022, 0.0), (0.045, 0.007, 0.076))
        cyan.add_box((-0.032, 0.002, 0.0), (0.005, 0.014, 0.050))
    elif name in ("left_finger_link", "right_finger_link"):
        # The URDF visual origin is y=-0.14 in each finger link frame.  Keep
        # that offset inside the authored mesh so the manifest can attach the
        # GLB at the identity transform and preserve joint synchronization.
        rubber.add_rounded_box((0.0, -0.14, 0.0), (0.10, 0.070, 0.020), 0.006, smooth)
        rubber.add_rounded_box((0.0, -0.178, 0.0), (0.086, 0.018, 0.026), 0.006, smooth)
        orange.add_rounded_box((0.0, -0.092, 0.0), (0.075, 0.018, 0.030), 0.005, smooth)
        aluminum.add_cylinder((0.0, -0.067, 0.0), 0.012, 0.052, "z", radial)
        for index in range(4 if coarse else 6):
            y = -0.168 + index * 0.014
            rubber.add_box((0.0, y, 0.011), (0.072, 0.004, 0.003))
        cyan.add_box((0.0, -0.141, 0.012), (0.050, 0.003, 0.002))
    else:
        raise ValueError(f"unknown mm_mobile_lift link: {name}")

    return {index: builder for index, builder in builders.items() if builder.indices}


def png_rgba(width: int, height: int, pixels: Iterable[tuple[int, int, int, int]]) -> bytes:
    """Encode a small deterministic RGBA PNG without external packages."""

    raw = bytearray()
    pixel_list = list(pixels)
    if len(pixel_list) != width * height:
        raise ValueError("PNG pixel count does not match dimensions")
    for row in range(height):
        raw.append(0)  # PNG filter byte: none.
        for red, green, blue, alpha in pixel_list[row * width : (row + 1) * width]:
            raw.extend((red, green, blue, alpha))

    def chunk(kind: bytes, payload: bytes) -> bytes:
        return struct.pack(">I", len(payload)) + kind + payload + struct.pack(">I", binascii.crc32(kind + payload) & 0xFFFFFFFF)

    return b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 6, 0, 0, 0)) + chunk(b"IDAT", zlib.compress(bytes(raw), level=9)) + chunk(b"IEND", b"")


def texture_maps() -> tuple[bytes, ...]:
    """Return base/metal-rough/normal/emissive/occlusion maps."""

    # A small authored tile is enough for these link-scale parts while still
    # giving the renderer useful roughness/normal variation at hero distance.
    # Keep it deterministic and deliberately neutral: material identity comes
    # from the per-primitive factors above, not from a hidden link texture.
    size = 16
    base = []
    metallic_roughness = []
    normal = []
    emissive = []
    occlusion = []
    for y in range(size):
        for x in range(size):
            checker = 5 if (x + y) % 2 else 0
            base.append((245 - checker, 245 - checker, 245 - checker, 255))
            metallic_roughness.append((0, 156 + ((x * 11 + y * 5) % 36), 176 + ((x + y) % 16), 255))
            normal.append((128 + ((x * 3 + y) % 5) - 2, 128 + ((x + y * 3) % 5) - 2, 255, 255))
            emissive.append((3 + (x % 4), 7 + (y % 5), 14 + ((x + y) % 4), 255))
            occlusion.append((228 - ((x * 3 + y) % 8),) * 3 + (255,))
    return tuple(
        png_rgba(size, size, pixels)
        for pixels in (base, metallic_roughness, normal, emissive, occlusion)
    )


def add_aligned(data: bytearray, payload: bytes, alignment: int = 4) -> int:
    while len(data) % alignment:
        data.append(0)
    offset = len(data)
    data.extend(payload)
    return offset


def pack_f32(values: Iterable[float]) -> bytes:
    values = tuple(values)
    return struct.pack("<" + "f" * len(values), *values)


def make_glb(link: str, lod: int) -> bytes:
    builders = build_link(link, lod)
    binary = bytearray()
    buffer_views: list[dict[str, int]] = []
    accessors: list[dict[str, object]] = []
    primitives: list[dict[str, object]] = []

    def attribute_accessor(values: Sequence[Sequence[float]], accessor_type: str, component_count: int) -> int:
        flattened = [component for value in values for component in value]
        payload = pack_f32(flattened)
        offset = add_aligned(binary, payload)
        view = len(buffer_views)
        buffer_views.append({"buffer": 0, "byteOffset": offset, "byteLength": len(payload), "target": 34962})
        mins = [min(value[index] for value in values) for index in range(component_count)]
        maxs = [max(value[index] for value in values) for index in range(component_count)]
        accessor = len(accessors)
        accessors.append({"bufferView": view, "componentType": 5126, "count": len(values), "type": accessor_type, "min": mins, "max": maxs})
        return accessor

    def index_accessor(indices: Sequence[int]) -> int:
        payload = struct.pack("<" + "I" * len(indices), *indices)
        offset = add_aligned(binary, payload)
        view = len(buffer_views)
        buffer_views.append({"buffer": 0, "byteOffset": offset, "byteLength": len(payload), "target": 34963})
        accessor = len(accessors)
        accessors.append({"bufferView": view, "componentType": 5125, "count": len(indices), "type": "SCALAR", "min": [min(indices)], "max": [max(indices)]})
        return accessor

    for material_index, builder in builders.items():
        position_accessor = attribute_accessor(builder.positions, "VEC3", 3)
        normal_accessor = attribute_accessor(builder.normals, "VEC3", 3)
        uv_accessor = attribute_accessor(builder.texcoords, "VEC2", 2)
        indices_accessor = index_accessor(builder.indices)
        primitives.append(
            {
                "attributes": {"POSITION": position_accessor, "NORMAL": normal_accessor, "TEXCOORD_0": uv_accessor},
                "indices": indices_accessor,
                "material": material_index,
            }
        )

    maps = texture_maps()
    image_views: list[int] = []
    for image in maps:
        offset = add_aligned(binary, image)
        image_views.append(len(buffer_views))
        buffer_views.append({"buffer": 0, "byteOffset": offset, "byteLength": len(image)})

    materials: list[dict[str, object]] = []
    for material in MATERIALS:
        materials.append(
            {
                "name": material.name,
                "pbrMetallicRoughness": {
                    "baseColorFactor": list(material.color),
                    "baseColorTexture": {"index": 0},
                    "metallicFactor": material.metallic,
                    "roughnessFactor": material.roughness,
                    "metallicRoughnessTexture": {"index": 1},
                },
                "normalTexture": {"index": 2, "scale": 0.65},
                "occlusionTexture": {"index": 4, "strength": 0.72},
                "emissiveFactor": list(material.emissive),
                "emissiveTexture": {"index": 3},
            }
        )

    document = {
        "asset": {"version": "2.0", "generator": "RNE deterministic mm_mobile_lift visual authoring"},
        "scene": 0,
        "scenes": [{"nodes": [0]}],
        "nodes": [{"name": link, "mesh": 0}],
        "meshes": [{"name": f"{link}.lod{lod}", "primitives": primitives}],
        "materials": materials,
        "textures": [{"source": index} for index in range(len(maps))],
        "images": [{"bufferView": view, "mimeType": "image/png"} for view in image_views],
        "buffers": [{"byteLength": len(binary)}],
        "bufferViews": buffer_views,
        "accessors": accessors,
    }
    json_bytes = json.dumps(document, sort_keys=True, separators=(",", ":"), ensure_ascii=True).encode("utf-8")
    while len(json_bytes) % 4:
        json_bytes += b" "
    while len(binary) % 4:
        binary.append(0)
    total_length = 12 + 8 + len(json_bytes) + 8 + len(binary)
    return b"".join(
        (
            struct.pack("<III", 0x46546C67, 2, total_length),
            struct.pack("<II", len(json_bytes), 0x4E4F534A),
            json_bytes,
            struct.pack("<II", len(binary), 0x004E4942),
            bytes(binary),
        )
    )


LINKS = (
    "base_link",
    "left_wheel",
    "right_wheel",
    "torso_link",
    "upper_arm_link",
    "forearm_link",
    "wrist_link",
    "gripper_base_link",
    "left_finger_link",
    "right_finger_link",
)


def generate(output_dir: Path = OUTPUT_DIR, *, check: bool = False) -> dict[str, tuple[int, str]]:
    if not check:
        output_dir.mkdir(parents=True, exist_ok=True)
    results: dict[str, tuple[int, str]] = {}
    for link in LINKS:
        for lod in (0, 1):
            filename = f"{link}.lod{lod}.glb"
            payload = make_glb(link, lod)
            path = output_dir / filename
            if check:
                if not path.is_file():
                    raise FileNotFoundError(f"generated visual is missing: {path}")
                committed = path.read_bytes()
                if committed != payload:
                    expected = hashlib.sha256(payload).hexdigest()
                    actual = hashlib.sha256(committed).hexdigest()
                    raise ValueError(
                        f"generated visual differs: {path} "
                        f"(expected sha256:{expected}, actual sha256:{actual})"
                    )
            else:
                path.write_bytes(payload)
            results[filename] = (len(payload), hashlib.sha256(payload).hexdigest())
    return results


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=OUTPUT_DIR, help="output mesh directory")
    parser.add_argument(
        "--check",
        action="store_true",
        help="verify existing GLBs are byte-identical without rewriting them",
    )
    args = parser.parse_args()
    output = args.output if args.output.is_absolute() else ROOT / args.output
    try:
        results = generate(output, check=args.check)
    except (FileNotFoundError, ValueError) as error:
        print(error, file=sys.stderr)
        return 1
    for filename in sorted(results):
        size, digest = results[filename]
        print(f"{filename}\t{size}\tsha256:{digest}")
    if args.check:
        print(f"verified {len(results)} deterministic GLBs")
    return 0


if __name__ == "__main__":
    sys.exit(main())
