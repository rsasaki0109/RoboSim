"""Render a mobile-manipulator rollout as a house-scene animated GIF.

The input is produced by:

    python examples/27_mobile_manipulator_rl/train.py --policy-in best_policy.json --eval-only --rollout-csv rollout.csv

The GIF writer is dependency-free and uses only the Python standard library.
Use ``--demo`` to generate a short synthetic house scene without rne_py.
Add ``--demo-rollout-csv`` to save that synthetic trajectory as canonical rollout CSV.
Use ``--verify-metadata`` to validate an existing GIF metadata JSON and its GIF.
"""

import argparse
import csv
import hashlib
import json
import math
import os
import sys

from rollout_schema import ROLLOUT_CSV_FIELDS


HOUSE_GIF_METADATA_VERSION = 1

PALETTE = (
    (248, 250, 252),  # 0 paper
    (226, 232, 240),  # 1 wall shadow
    (255, 255, 255),  # 2 wall
    (203, 213, 225),  # 3 border
    (148, 163, 184),  # 4 muted
    (71, 85, 105),  # 5 dark slate
    (15, 118, 110),  # 6 teal
    (20, 184, 166),  # 7 light teal
    (220, 38, 38),  # 8 red
    (251, 146, 60),  # 9 orange
    (250, 204, 21),  # 10 yellow
    (34, 197, 94),  # 11 green
    (59, 130, 246),  # 12 blue
    (125, 211, 252),  # 13 sky
    (120, 113, 108),  # 14 wood dark
    (214, 184, 137),  # 15 wood
    (245, 230, 203),  # 16 wood light
    (244, 114, 182),  # 17 pink
    (168, 85, 247),  # 18 violet
    (30, 41, 59),  # 19 near black
    (241, 245, 249),  # 20 pale floor
    (217, 249, 157),  # 21 plant
    (101, 163, 13),  # 22 plant dark
    (254, 226, 226),  # 23 target fill
)

PAPER = 0
WALL_SHADOW = 1
WALL = 2
BORDER = 3
MUTED = 4
DARK = 5
TEAL = 6
LIGHT_TEAL = 7
RED = 8
ORANGE = 9
YELLOW = 10
GREEN = 11
BLUE = 12
SKY = 13
WOOD_DARK = 14
WOOD = 15
WOOD_LIGHT = 16
PINK = 17
VIOLET = 18
BLACK = 19
FLOOR = 20
PLANT = 21
PLANT_DARK = 22
TARGET_FILL = 23


class Canvas:
    def __init__(self, width, height, fill=PAPER):
        self.width = width
        self.height = height
        self.pixels = [fill] * (width * height)

    def set(self, x, y, color):
        x = int(x)
        y = int(y)
        if 0 <= x < self.width and 0 <= y < self.height:
            self.pixels[y * self.width + x] = color

    def rect(self, x0, y0, x1, y1, color):
        x0 = max(0, int(round(x0)))
        y0 = max(0, int(round(y0)))
        x1 = min(self.width, int(round(x1)))
        y1 = min(self.height, int(round(y1)))
        for y in range(y0, y1):
            offset = y * self.width
            self.pixels[offset + x0 : offset + x1] = [color] * max(0, x1 - x0)

    def outline_rect(self, x0, y0, x1, y1, color, thickness=1):
        for i in range(thickness):
            self.line(x0 + i, y0 + i, x1 - i, y0 + i, color)
            self.line(x0 + i, y1 - i, x1 - i, y1 - i, color)
            self.line(x0 + i, y0 + i, x0 + i, y1 - i, color)
            self.line(x1 - i, y0 + i, x1 - i, y1 - i, color)

    def disk(self, cx, cy, radius, color):
        radius = int(round(radius))
        cx = int(round(cx))
        cy = int(round(cy))
        rr = radius * radius
        for y in range(cy - radius, cy + radius + 1):
            for x in range(cx - radius, cx + radius + 1):
                dx = x - cx
                dy = y - cy
                if dx * dx + dy * dy <= rr:
                    self.set(x, y, color)

    def circle(self, cx, cy, radius, color, thickness=1):
        for angle in range(0, 360, 2):
            rad = math.radians(angle)
            x = cx + math.cos(rad) * radius
            y = cy + math.sin(rad) * radius
            self.disk(x, y, max(1, thickness), color)

    def line(self, x0, y0, x1, y1, color, thickness=1):
        dx = x1 - x0
        dy = y1 - y0
        steps = max(1, int(max(abs(dx), abs(dy))))
        radius = max(0, (thickness - 1) / 2.0)
        for step in range(steps + 1):
            t = step / steps
            x = x0 + dx * t
            y = y0 + dy * t
            if thickness <= 1:
                self.set(x, y, color)
            else:
                self.disk(x, y, radius, color)


def _subblocks(data):
    chunks = bytearray()
    for offset in range(0, len(data), 255):
        chunk = data[offset : offset + 255]
        chunks.append(len(chunk))
        chunks.extend(chunk)
    chunks.append(0)
    return bytes(chunks)


def _bitpack_lzw_codes(codes):
    out = bytearray()
    buffer = 0
    bits = 0
    for code, code_size in codes:
        buffer |= code << bits
        bits += code_size
        while bits >= 8:
            out.append(buffer & 0xFF)
            buffer >>= 8
            bits -= 8
    if bits:
        out.append(buffer & 0xFF)
    return bytes(out)


def _lzw_codes(indices, min_code_size):
    clear = 1 << min_code_size
    end = clear + 1
    dictionary = {(index,): index for index in range(clear)}
    code_size = min_code_size + 1
    next_code = end + 1

    yield clear, code_size
    iterator = iter(indices)
    try:
        prefix = (next(iterator),)
    except StopIteration:
        yield end, code_size
        return

    for index in iterator:
        candidate = prefix + (index,)
        if candidate in dictionary:
            prefix = candidate
            continue

        yield dictionary[prefix], code_size
        if next_code < 4096:
            dictionary[candidate] = next_code
            next_code += 1
            if next_code > (1 << code_size) and code_size < 12:
                code_size += 1
        else:
            yield clear, code_size
            dictionary = {(value,): value for value in range(clear)}
            code_size = min_code_size + 1
            next_code = end + 1
        prefix = (index,)

    yield dictionary[prefix], code_size
    yield end, code_size


def _gif_image_data(indices, palette_size):
    min_code_size = max(2, (palette_size - 1).bit_length())
    for index in indices:
        if index < 0 or index >= palette_size:
            raise ValueError(f"palette index out of range: {index}")

    return bytes([min_code_size]) + _subblocks(
        _bitpack_lzw_codes(_lzw_codes(indices, min_code_size))
    )


def write_gif(path, frames, *, width, height, fps):
    palette_size = 1
    while palette_size < len(PALETTE):
        palette_size *= 2
    padded_palette = list(PALETTE) + [(0, 0, 0)] * (palette_size - len(PALETTE))
    color_table_size = (palette_size.bit_length() - 1) - 1
    delay_cs = max(1, int(round(100 / fps)))

    with open(path, "wb") as handle:
        handle.write(b"GIF89a")
        handle.write(width.to_bytes(2, "little"))
        handle.write(height.to_bytes(2, "little"))
        handle.write(bytes([0x80 | 0x70 | color_table_size, PAPER, 0]))
        for red, green, blue in padded_palette:
            handle.write(bytes([red, green, blue]))
        handle.write(b"\x21\xff\x0bNETSCAPE2.0\x03\x01\x00\x00\x00")
        for frame in frames:
            handle.write(b"\x21\xf9\x04")
            handle.write(bytes([0x08]))
            handle.write(delay_cs.to_bytes(2, "little"))
            handle.write(b"\x00\x00")
            handle.write(b"\x2c")
            handle.write((0).to_bytes(2, "little"))
            handle.write((0).to_bytes(2, "little"))
            handle.write(width.to_bytes(2, "little"))
            handle.write(height.to_bytes(2, "little"))
            handle.write(b"\x00")
            handle.write(_gif_image_data(frame, palette_size))
        handle.write(b";")


def _file_sha256(path):
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return f"sha256:{digest.hexdigest()}"


def write_metadata(
    path,
    *,
    gif_path,
    metadata_gif_path=None,
    source,
    sample_count,
    frame_count,
    max_frames,
    width,
    height,
    fps,
):
    payload = {
        "schema_version": HOUSE_GIF_METADATA_VERSION,
        "artifact": "rne_mobile_manipulator_house_gif",
        "gif_path": metadata_gif_path or gif_path,
        "source": source,
        "sample_count": sample_count,
        "frame_count": frame_count,
        "max_frames": max_frames,
        "width": width,
        "height": height,
        "fps": fps,
        "byte_size": os.path.getsize(gif_path),
        "sha256": _file_sha256(gif_path),
    }
    directory = os.path.dirname(os.path.abspath(path))
    if directory:
        os.makedirs(directory, exist_ok=True)
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2, sort_keys=True)
        handle.write("\n")
    return payload


def _skip_gif_subblocks(data, offset, *, context):
    while offset < len(data):
        size = data[offset]
        offset += 1
        if size == 0:
            return offset
        offset += size
        if offset > len(data):
            raise ValueError(f"{context}: GIF sub-block exceeds file length")
    raise ValueError(f"{context}: GIF sub-block terminator missing")


def inspect_gif(path):
    context = str(path)
    with open(path, "rb") as handle:
        data = handle.read()
    if len(data) < 14:
        raise ValueError(f"{context}: GIF artifact is too small")
    if data[:6] not in (b"GIF87a", b"GIF89a"):
        raise ValueError(f"{context}: GIF header mismatch")

    width = int.from_bytes(data[6:8], "little")
    height = int.from_bytes(data[8:10], "little")
    if width <= 0 or height <= 0:
        raise ValueError(f"{context}: GIF logical screen must be non-empty")

    offset = 13
    packed = data[10]
    if packed & 0x80:
        color_count = 1 << ((packed & 0x07) + 1)
        offset += color_count * 3
    if offset >= len(data):
        raise ValueError(f"{context}: GIF body missing")

    frame_count = 0
    while offset < len(data):
        marker = data[offset]
        offset += 1
        if marker == 0x3B:
            if offset != len(data):
                raise ValueError(f"{context}: GIF has trailing bytes after trailer")
            break
        if marker == 0x21:
            if offset >= len(data):
                raise ValueError(f"{context}: GIF extension label missing")
            offset += 1
            offset = _skip_gif_subblocks(data, offset, context=context)
            continue
        if marker != 0x2C:
            raise ValueError(f"{context}: GIF block marker mismatch: 0x{marker:02x}")

        if offset + 9 > len(data):
            raise ValueError(f"{context}: GIF image descriptor truncated")
        image_packed = data[offset + 8]
        offset += 9
        if image_packed & 0x80:
            color_count = 1 << ((image_packed & 0x07) + 1)
            offset += color_count * 3
            if offset > len(data):
                raise ValueError(f"{context}: GIF local color table truncated")
        if offset >= len(data):
            raise ValueError(f"{context}: GIF image data missing")
        offset += 1
        offset = _skip_gif_subblocks(data, offset, context=context)
        frame_count += 1
    else:
        raise ValueError(f"{context}: GIF trailer missing")

    if frame_count == 0:
        raise ValueError(f"{context}: GIF has no image frames")
    return {
        "width": width,
        "height": height,
        "frame_count": frame_count,
        "byte_size": len(data),
        "sha256": f"sha256:{hashlib.sha256(data).hexdigest()}",
    }


def _positive_int(value, field):
    if not isinstance(value, int) or isinstance(value, bool) or value <= 0:
        raise ValueError(f"metadata field {field!r} must be a positive integer")
    return value


def _positive_number(value, field):
    if not isinstance(value, (int, float)) or isinstance(value, bool) or value <= 0:
        raise ValueError(f"metadata field {field!r} must be a positive number")
    return float(value)


def _resolve_metadata_path(metadata_path, value, field):
    if not isinstance(value, str) or not value:
        raise ValueError(f"metadata field {field!r} must be a non-empty string")
    if os.path.isabs(value):
        return os.path.abspath(value)
    metadata_dir = os.path.dirname(os.path.abspath(metadata_path))
    return os.path.abspath(os.path.join(metadata_dir, value))


def verify_metadata(metadata_path):
    with open(metadata_path, "r", encoding="utf-8") as handle:
        metadata = json.load(handle)
    if not isinstance(metadata, dict):
        raise ValueError("metadata must be an object")
    if metadata.get("schema_version") != HOUSE_GIF_METADATA_VERSION:
        raise ValueError(
            f"metadata schema_version mismatch: {metadata.get('schema_version')!r}"
        )
    if metadata.get("artifact") != "rne_mobile_manipulator_house_gif":
        raise ValueError(f"metadata artifact mismatch: {metadata.get('artifact')!r}")
    if not isinstance(metadata.get("source"), dict):
        raise ValueError("metadata source must be an object")

    gif_path = _resolve_metadata_path(metadata_path, metadata.get("gif_path"), "gif_path")
    if not os.path.isfile(gif_path):
        raise ValueError(f"metadata gif_path does not exist: {gif_path}")
    gif_info = inspect_gif(gif_path)

    for field in ("width", "height", "frame_count", "byte_size"):
        expected = _positive_int(metadata.get(field), field)
        if gif_info[field] != expected:
            raise ValueError(
                f"metadata {field} mismatch: {gif_info[field]!r} != {expected!r}"
            )
    sample_count = _positive_int(metadata.get("sample_count"), "sample_count")
    max_frames = _positive_int(metadata.get("max_frames"), "max_frames")
    if gif_info["frame_count"] > max_frames:
        raise ValueError(
            f"metadata frame_count exceeds max_frames: "
            f"{gif_info['frame_count']!r} > {max_frames!r}"
        )
    _positive_number(metadata.get("fps"), "fps")
    if metadata.get("sha256") != gif_info["sha256"]:
        raise ValueError(
            f"metadata sha256 mismatch: {gif_info['sha256']!r} != {metadata.get('sha256')!r}"
        )
    return {
        **gif_info,
        "gif_path": gif_path,
        "sample_count": sample_count,
        "max_frames": max_frames,
        "fps": float(metadata["fps"]),
    }


def _float(row, key):
    try:
        return float(row[key])
    except KeyError as error:
        raise ValueError(f"rollout CSV missing column: {key}") from error


def load_samples(csv_path):
    with open(csv_path, newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames != list(ROLLOUT_CSV_FIELDS):
            raise ValueError(f"rollout CSV header mismatch: {reader.fieldnames!r}")
        rows = list(reader)
    if not rows:
        raise ValueError(f"rollout CSV has no rows: {csv_path}")

    samples = []
    for row in rows:
        sample = {
            "step": int(float(row["step"])),
            "base_x": _float(row, "base_x"),
            "base_y": _float(row, "base_y"),
            "base_yaw": _float(row, "base_yaw"),
            "ee_x": _float(row, "ee_x"),
            "ee_y": _float(row, "ee_y"),
            "ee_z": _float(row, "ee_z"),
            "target_x": _float(row, "ee_x") + _float(row, "target_dx"),
            "target_y": _float(row, "ee_y") + _float(row, "target_dy"),
            "target_z": _float(row, "ee_z") + _float(row, "target_dz"),
            "target_error": math.sqrt(
                _float(row, "target_dx") ** 2
                + _float(row, "target_dy") ** 2
                + _float(row, "target_dz") ** 2
            ),
            "shoulder_action": _float(row, "shoulder_action"),
            "elbow_action": _float(row, "elbow_action"),
            "reward": _float(row, "reward"),
            "total_reward": _float(row, "total_reward"),
            "done": row.get("done", "").strip().lower() == "true",
        }
        samples.append(sample)
    return samples


def _clamp01(value):
    return max(0.0, min(1.0, value))


def _lerp(a, b, t):
    return a + (b - a) * t


def _smoothstep(t):
    t = _clamp01(t)
    return t * t * (3.0 - 2.0 * t)


def demo_samples(count=90):
    samples = []
    total_reward = 0.0
    for step in range(count):
        phase = step / max(1, count - 1)
        pick_x = -0.08
        pick_z = 0.62
        place_x = 0.76
        place_z = 0.78
        gripper_closed = 0.0
        carrying = 0.0

        if phase < 0.28:
            local = _smoothstep(phase / 0.28)
            base_x = _lerp(-0.82, -0.32, local)
            ee_x = base_x + _lerp(0.18, 0.27, local)
            ee_z = _lerp(0.52, 0.62, local)
            target_x = pick_x
            target_z = pick_z
            object_x = pick_x
            object_z = pick_z
        elif phase < 0.46:
            local = _smoothstep((phase - 0.28) / 0.18)
            base_x = -0.32
            ee_x = _lerp(base_x + 0.26, pick_x, local)
            ee_z = _lerp(0.62, pick_z, local)
            target_x = pick_x
            target_z = pick_z
            object_x = pick_x
            object_z = pick_z
            gripper_closed = 1.0 if local > 0.62 else 0.0
            carrying = 1.0 if local > 0.72 else 0.0
        elif phase < 0.72:
            local = _smoothstep((phase - 0.46) / 0.26)
            base_x = _lerp(-0.32, 0.48, local)
            ee_x = base_x + 0.25
            ee_z = 0.74 + math.sin(local * math.pi) * 0.06
            target_x = place_x
            target_z = place_z
            object_x = ee_x
            object_z = ee_z
            gripper_closed = 1.0
            carrying = 1.0
        elif phase < 0.9:
            local = _smoothstep((phase - 0.72) / 0.18)
            base_x = 0.48
            ee_x = _lerp(base_x + 0.25, place_x, local)
            ee_z = _lerp(0.74, place_z, local)
            target_x = place_x
            target_z = place_z
            object_x = ee_x
            object_z = ee_z
            gripper_closed = 1.0 if local < 0.78 else 0.0
            carrying = 1.0 if local < 0.86 else 0.0
            if not carrying:
                object_x = place_x
                object_z = place_z
        else:
            local = _smoothstep((phase - 0.9) / 0.1)
            base_x = _lerp(0.48, 0.36, local)
            ee_x = _lerp(place_x, base_x + 0.16, local)
            ee_z = _lerp(place_z, 0.56, local)
            target_x = place_x
            target_z = place_z
            object_x = place_x
            object_z = place_z

        target_dx = target_x - ee_x
        target_dz = target_z - ee_z
        target_error = math.sqrt(target_dx * target_dx + target_dz * target_dz)
        reward = max(0.0, 0.35 - target_error * 0.12)
        total_reward += reward
        samples.append(
            {
                "step": step,
                "base_x": base_x,
                "base_y": 0.0,
                "base_yaw": 0.0,
                "ee_x": ee_x,
                "ee_y": 0.0,
                "ee_z": ee_z,
                "target_x": target_x,
                "target_y": 0.0,
                "target_z": target_z,
                "target_error": target_error,
                "object_x": object_x,
                "object_z": object_z,
                "gripper_closed": gripper_closed,
                "carrying": carrying,
                "shoulder_action": math.sin(phase * math.pi * 1.5) * 1.4,
                "elbow_action": math.cos(phase * math.pi * 2.0) * 1.2,
                "reward": reward,
                "total_reward": total_reward,
                "done": step == count - 1,
            }
        )
    return samples


def write_demo_rollout_csv(path, samples):
    directory = os.path.dirname(os.path.abspath(path))
    if directory:
        os.makedirs(directory, exist_ok=True)
    tmp_path = f"{path}.tmp"
    with open(tmp_path, "w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=ROLLOUT_CSV_FIELDS)
        writer.writeheader()
        for sample in samples:
            writer.writerow(
                {
                    "step": sample["step"],
                    "base_x": sample["base_x"],
                    "base_y": sample["base_y"],
                    "base_yaw": sample["base_yaw"],
                    "ee_x": sample["ee_x"],
                    "ee_y": sample["ee_y"],
                    "ee_z": sample["ee_z"],
                    "target_dx": sample["target_x"] - sample["ee_x"],
                    "target_dy": sample["target_y"] - sample["ee_y"],
                    "target_dz": sample["target_z"] - sample["ee_z"],
                    "shoulder_action": sample["shoulder_action"],
                    "elbow_action": sample["elbow_action"],
                    "reward": sample["reward"],
                    "total_reward": sample["total_reward"],
                    "done": "true" if sample["done"] else "false",
                }
            )
    os.replace(tmp_path, path)


def _select_frames(samples, max_frames):
    if len(samples) <= max_frames:
        return list(samples)
    indexes = []
    for i in range(max_frames):
        index = round(i * (len(samples) - 1) / (max_frames - 1))
        if not indexes or indexes[-1] != index:
            indexes.append(index)
    return [samples[index] for index in indexes]


def _scene_bounds(samples):
    xs = []
    zs = [0.0]
    for sample in samples:
        xs.extend([sample["base_x"], sample["ee_x"], sample["target_x"]])
        zs.extend([sample["ee_z"], sample["target_z"]])
        if "object_x" in sample:
            xs.append(sample["object_x"])
        if "object_z" in sample:
            zs.append(sample["object_z"])
    x0 = min(xs)
    x1 = max(xs)
    z0 = 0.0
    z1 = max(1.2, max(zs))
    x_pad = max(0.35, (x1 - x0) * 0.25)
    return x0 - x_pad, x1 + x_pad, z0, z1 + 0.25


def _mapper(width, height, bounds):
    x0, x1, z0, z1 = bounds
    left = int(width * 0.12)
    right = int(width * 0.9)
    floor_y = int(height * 0.82)
    ceiling_y = int(height * 0.12)

    def map_x(value):
        return left + (value - x0) / (x1 - x0) * (right - left)

    def map_z(value):
        return floor_y - (value - z0) / (z1 - z0) * (floor_y - ceiling_y)

    return map_x, map_z, floor_y


def _draw_house(canvas, floor_y):
    width = canvas.width
    height = canvas.height
    canvas.rect(0, 0, width, height, WALL)
    canvas.rect(0, floor_y, width, height, FLOOR)
    canvas.rect(0, floor_y - 5, width, floor_y, BORDER)

    canvas.rect(width * 0.08, height * 0.12, width * 0.32, height * 0.42, SKY)
    canvas.outline_rect(width * 0.08, height * 0.12, width * 0.32, height * 0.42, BORDER, 2)
    canvas.line(width * 0.2, height * 0.12, width * 0.2, height * 0.42, BORDER, 2)
    canvas.line(width * 0.08, height * 0.27, width * 0.32, height * 0.27, BORDER, 2)

    canvas.rect(width * 0.06, floor_y - 32, width * 0.32, floor_y - 18, WOOD_LIGHT)
    canvas.rect(width * 0.1, floor_y - 54, width * 0.26, floor_y - 30, PINK)
    canvas.disk(width * 0.13, floor_y - 58, 14, PINK)
    canvas.disk(width * 0.23, floor_y - 58, 14, PINK)

    canvas.rect(width * 0.63, floor_y - 78, width * 0.86, floor_y - 18, WOOD)
    canvas.outline_rect(width * 0.63, floor_y - 78, width * 0.86, floor_y - 18, WOOD_DARK, 2)
    for shelf in (0.68, 0.76):
        canvas.line(width * 0.63, floor_y - height * (shelf - 0.62), width * 0.86, floor_y - height * (shelf - 0.62), WOOD_DARK)
    canvas.rect(width * 0.69, floor_y - 98, width * 0.74, floor_y - 80, PLANT_DARK)
    canvas.disk(width * 0.68, floor_y - 105, 11, PLANT)
    canvas.disk(width * 0.72, floor_y - 116, 12, PLANT)
    canvas.disk(width * 0.76, floor_y - 104, 11, PLANT)

    canvas.rect(width * 0.36, floor_y - 15, width * 0.61, floor_y - 5, WOOD_LIGHT)
    canvas.outline_rect(width * 0.36, floor_y - 15, width * 0.61, floor_y - 5, WOOD, 1)
    canvas.rect(width * 0.38, floor_y - 70, width * 0.55, floor_y - 60, WOOD)
    canvas.rect(width * 0.39, floor_y - 60, width * 0.41, floor_y - 15, WOOD_DARK)
    canvas.rect(width * 0.52, floor_y - 60, width * 0.54, floor_y - 15, WOOD_DARK)


def _draw_robot(canvas, sample, map_x, map_z, floor_y):
    base_x = map_x(sample["base_x"])
    body_y = floor_y - 25
    canvas.rect(base_x - 25, body_y - 12, base_x + 25, body_y + 11, DARK)
    canvas.rect(base_x - 21, body_y - 20, base_x + 19, body_y - 11, TEAL)
    canvas.disk(base_x - 16, body_y + 13, 8, BLACK)
    canvas.disk(base_x + 16, body_y + 13, 8, BLACK)
    canvas.disk(base_x - 16, body_y + 13, 4, MUTED)
    canvas.disk(base_x + 16, body_y + 13, 4, MUTED)

    shoulder_x = map_x(sample["base_x"] + 0.08)
    shoulder_y = map_z(0.38)
    ee_x = map_x(sample["ee_x"])
    ee_y = map_z(sample["ee_z"])
    elbow_x = (shoulder_x + ee_x) / 2 + math.sin(sample["elbow_action"]) * 10
    elbow_y = min(shoulder_y, ee_y) - 22 - abs(math.sin(sample["shoulder_action"])) * 10

    canvas.line(shoulder_x, shoulder_y, elbow_x, elbow_y, TEAL, 6)
    canvas.line(elbow_x, elbow_y, ee_x, ee_y, LIGHT_TEAL, 6)
    canvas.disk(shoulder_x, shoulder_y, 8, DARK)
    canvas.disk(elbow_x, elbow_y, 7, DARK)
    canvas.disk(ee_x, ee_y, 7, ORANGE)
    jaw = 3 if sample.get("gripper_closed", 0.0) >= 0.5 else 9
    canvas.line(ee_x, ee_y, ee_x + 14, ee_y - jaw, DARK, 2)
    canvas.line(ee_x, ee_y, ee_x + 14, ee_y + jaw, DARK, 2)


def _inferred_phase(sample, final_step):
    return _clamp01(sample["step"] / max(1, final_step))


def _object_pose(sample, final_step):
    if "object_x" in sample and "object_z" in sample:
        return sample["object_x"], sample["object_z"]

    phase = _inferred_phase(sample, final_step)
    if 0.46 <= phase < 0.9:
        return sample["ee_x"], sample["ee_z"]
    return sample["target_x"], sample["target_z"]


def _draw_task_object(canvas, sample, map_x, map_z, final_step):
    object_x, object_z = _object_pose(sample, final_step)
    x = map_x(object_x)
    y = map_z(object_z)
    canvas.rect(x - 9, y - 9, x + 9, y + 9, YELLOW)
    canvas.outline_rect(x - 9, y - 9, x + 9, y + 9, ORANGE, 2)
    canvas.line(x - 7, y - 10, x + 7, y - 10, WOOD_DARK, 2)


def _draw_frame(sample, history, bounds, width, height, final_step):
    canvas = Canvas(width, height, WALL)
    map_x, map_z, floor_y = _mapper(width, height, bounds)
    _draw_house(canvas, floor_y)

    nav_y = floor_y - 31
    for previous, current in zip(history, history[1:]):
        canvas.line(
            map_x(previous["base_x"]),
            nav_y,
            map_x(current["base_x"]),
            nav_y,
            SKY,
            3,
        )
    if history:
        canvas.disk(map_x(history[0]["base_x"]), nav_y, 5, BLUE)
        canvas.disk(map_x(history[-1]["base_x"]), nav_y, 5, TEAL)

    for previous, current in zip(history, history[1:]):
        canvas.line(
            map_x(previous["ee_x"]),
            map_z(previous["ee_z"]),
            map_x(current["ee_x"]),
            map_z(current["ee_z"]),
            BLUE,
            2,
        )

    target_x = map_x(sample["target_x"])
    target_y = map_z(sample["target_z"])
    canvas.disk(target_x, target_y, 12, TARGET_FILL)
    canvas.circle(target_x, target_y, 12, RED, 2)
    canvas.line(target_x - 8, target_y, target_x + 8, target_y, RED, 2)
    canvas.line(target_x, target_y - 8, target_x, target_y + 8, RED, 2)

    if sample.get("carrying", 0.0) < 0.5:
        _draw_task_object(canvas, sample, map_x, map_z, final_step)
    _draw_robot(canvas, sample, map_x, map_z, floor_y)
    if sample.get("carrying", 0.0) >= 0.5:
        _draw_task_object(canvas, sample, map_x, map_z, final_step)

    progress_width = int((width - 40) * (sample["step"] / final_step))
    canvas.rect(20, height - 18, width - 20, height - 12, WALL_SHADOW)
    canvas.rect(20, height - 18, 20 + progress_width, height - 12, TEAL)
    return canvas.pixels


def render_house_frames(samples, *, width, height, max_frames):
    selected = _select_frames(samples, max_frames)
    bounds = _scene_bounds(selected)
    final_step = max(1, selected[-1]["step"])
    frames = []
    for index, sample in enumerate(selected):
        history = selected[: index + 1]
        frames.append(_draw_frame(sample, history, bounds, width, height, final_step))
    return frames


def default_output_path(input_path, demo=False):
    if demo:
        return "house_mobile_manipulator.gif"
    root, _ = os.path.splitext(input_path)
    return f"{root}_house.gif"


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv_path", nargs="?", help="rollout CSV produced by train.py")
    parser.add_argument("--demo", action="store_true", help="render a built-in synthetic house scene")
    parser.add_argument(
        "--demo-rollout-csv",
        help="write the built-in --demo trajectory as canonical rollout CSV",
    )
    parser.add_argument("--out", help="GIF output path")
    parser.add_argument("--metadata-out", help="write GIF metadata JSON to this path")
    parser.add_argument(
        "--verify-metadata",
        help="validate an existing GIF metadata JSON and its referenced GIF",
    )
    parser.add_argument("--fps", type=float, default=12.0, help="GIF playback frames per second")
    parser.add_argument("--max-frames", type=int, default=72, help="maximum frames to encode")
    parser.add_argument("--width", type=int, default=360, help="GIF width in pixels")
    parser.add_argument("--height", type=int, default=240, help="GIF height in pixels")
    args = parser.parse_args()
    if args.verify_metadata is not None:
        if any(
            value is not None
            for value in (args.csv_path, args.demo_rollout_csv, args.out, args.metadata_out)
        ) or args.demo:
            parser.error("--verify-metadata cannot be combined with rendering options")
        return args
    if not args.demo and args.csv_path is None:
        parser.error("csv_path is required unless --demo is used")
    if args.demo_rollout_csv is not None and not args.demo:
        parser.error("--demo-rollout-csv requires --demo")
    if args.fps <= 0.0:
        parser.error("--fps must be positive")
    if args.max_frames <= 1:
        parser.error("--max-frames must be greater than 1")
    if args.width <= 0 or args.height <= 0:
        parser.error("--width and --height must be positive")
    return args


def main():
    args = parse_args()
    if args.verify_metadata is not None:
        info = verify_metadata(args.verify_metadata)
        print(
            f"house gif metadata verified: {args.verify_metadata} "
            f"gif={info['gif_path']} frames={info['frame_count']} "
            f"size={info['width']}x{info['height']} sha256={info['sha256']}"
        )
        return

    samples = demo_samples() if args.demo else load_samples(args.csv_path)
    if args.demo_rollout_csv is not None:
        write_demo_rollout_csv(args.demo_rollout_csv, samples)
        print(
            f"demo rollout csv saved: {args.demo_rollout_csv} rows={len(samples)}"
        )
    output_path = args.out or default_output_path(args.csv_path, demo=args.demo)
    frames = render_house_frames(
        samples, width=args.width, height=args.height, max_frames=args.max_frames
    )
    write_gif(output_path, frames, width=args.width, height=args.height, fps=args.fps)
    print(
        f"house gif saved: {output_path} samples={len(samples)} "
        f"frames={len(frames)} size={args.width}x{args.height}"
    )
    if args.metadata_out is not None:
        if args.demo:
            source = {"kind": "demo", "task": "navigate_pick_place"}
            if args.demo_rollout_csv is not None:
                source["rollout_csv_path"] = args.demo_rollout_csv
        else:
            source = {"kind": "rollout_csv", "path": args.csv_path}
        metadata = write_metadata(
            args.metadata_out,
            gif_path=output_path,
            source=source,
            sample_count=len(samples),
            frame_count=len(frames),
            max_frames=args.max_frames,
            width=args.width,
            height=args.height,
            fps=args.fps,
        )
        print(
            f"house gif metadata saved: {args.metadata_out} "
            f"bytes={metadata['byte_size']} sha256={metadata['sha256']}"
        )


if __name__ == "__main__":
    try:
        main()
    except Exception as error:
        sys.exit(str(error))
