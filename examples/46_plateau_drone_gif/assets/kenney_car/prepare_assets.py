"""Rebuild the checked-in Kenney sedan assets from the official CC0 archive."""

from __future__ import annotations

import argparse
import hashlib
import io
import zipfile
from pathlib import Path

from PIL import Image


ARCHIVE_SHA256 = "fac7dacac5c7874348cf19729af3ef205f3d366493edaf0a827d93f4fdf3d0c4"
BODY_OBJ_PATH = "Models/OBJ format/sedan.obj"
WHEEL_OBJ_PATH = "Models/OBJ format/wheel-default.obj"
COLORMAP_PATH = "Models/OBJ format/Textures/colormap.png"


def extract_group(source: str, group_name: str) -> str:
    """Retain shared vertex data and faces belonging to one OBJ group."""

    vertices = [
        tuple(map(float, line.split()[1:4]))
        for line in source.splitlines()
        if line.startswith("v ")
    ]
    output = ["# Derived from Kenney Car Kit 3.1 (CC0)", "mtllib vehicle.mtl"]
    active_group = ""
    for line in source.splitlines():
        if line.startswith("g "):
            active_group = line[2:].strip()
            if active_group == group_name:
                output.append(f"g {group_name}")
        elif line.startswith(("v ", "vt ", "vn ")):
            output.append(line)
        elif active_group == group_name and line.startswith(("usemtl ", "s ")):
            output.append(line)
        elif active_group == group_name and line.startswith("f "):
            indices = [int(ref.split("/")[0]) - 1 for ref in line.split()[1:]]
            points = [vertices[index] for index in indices]
            is_detached_rear_bumper = (
                max(point[2] for point in points) <= -1.24
                and max(abs(point[0]) for point in points) <= 0.18
                and max(point[1] for point in points) <= 0.33
            )
            if not is_detached_rear_bumper:
                output.append(line)
    output.append("")
    return "\n".join(output)


def rewrite_material_library(source: str) -> str:
    """Point a complete upstream OBJ at the shared checked-in material."""

    lines = [
        "mtllib vehicle.mtl" if line.startswith("mtllib ") else line
        for line in source.splitlines()
    ]
    return "\n".join(lines).rstrip() + "\n"


def recolor_body(source_png: bytes, target_rgb: tuple[int, int, int]) -> bytes:
    """Replace the warm body palette while preserving baked palette shading."""

    image = Image.open(io.BytesIO(source_png)).convert("RGBA")
    pixels = []
    for red, green, blue, alpha in image.get_flattened_data():
        if blue > 180 and green > 180 and blue > red + 8:
            shade = blue / 255.0
            pixels.append(
                (
                    round(42 * shade),
                    round(64 * shade),
                    round(78 * shade),
                    alpha,
                )
            )
        elif red > 150 and green < 170 and blue < 130:
            shade = red / 255.0
            pixels.append(
                (
                    round(target_rgb[0] * shade),
                    round(target_rgb[1] * shade),
                    round(target_rgb[2] * shade),
                    alpha,
                )
            )
        else:
            pixels.append((red, green, blue, alpha))
    image.putdata(pixels)
    encoded = io.BytesIO()
    image.save(encoded, format="PNG", optimize=False)
    return encoded.getvalue()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("archive", type=Path, help="official kenney_car-kit.zip")
    parser.add_argument(
        "--output",
        type=Path,
        default=Path(__file__).resolve().parent,
        help="asset output directory",
    )
    args = parser.parse_args()

    archive_bytes = args.archive.read_bytes()
    digest = hashlib.sha256(archive_bytes).hexdigest()
    if digest != ARCHIVE_SHA256:
        raise SystemExit(f"archive SHA-256 mismatch: {digest}")

    args.output.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(io.BytesIO(archive_bytes)) as archive:
        body_source = archive.read(BODY_OBJ_PATH).decode("utf-8")
        wheel_source = archive.read(WHEEL_OBJ_PATH).decode("utf-8")
        colormap = archive.read(COLORMAP_PATH)
        license_text = archive.read("License.txt")

    (args.output / "sedan-body.obj").write_text(
        extract_group(body_source, "body"), encoding="utf-8", newline="\n"
    )
    (args.output / "wheel.obj").write_text(
        rewrite_material_library(wheel_source), encoding="utf-8", newline="\n"
    )
    (args.output / "vehicle.mtl").write_text(
        "# Derived from Kenney Car Kit 3.1 (CC0)\n"
        "newmtl colormap\n"
        "Kd 1 1 1\n"
        "map_Kd colormap.png\n",
        encoding="utf-8",
        newline="\n",
    )
    (args.output / "colormap.png").write_bytes(colormap)
    (args.output / "colormap-red.png").write_bytes(
        recolor_body(colormap, (230, 42, 24))
    )
    (args.output / "colormap-blue.png").write_bytes(
        recolor_body(colormap, (34, 105, 235))
    )
    normalized_license = "\n".join(
        line.rstrip() for line in license_text.decode("utf-8").splitlines()
    ).strip()
    (args.output / "LICENSE.txt").write_text(
        normalized_license + "\n", encoding="utf-8", newline="\n"
    )


if __name__ == "__main__":
    main()
