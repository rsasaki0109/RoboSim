#!/usr/bin/env python3
"""Prepare a deterministic, repository-sized derivative of Voxel51 Playroom 3DGS.

The upstream Graphdeco PLY is large. This tool keeps every Nth Gaussian and
copies its position, DC colour, opacity, scale, and rotation floats byte-for-byte.
Normals and view-dependent spherical harmonics are omitted; the RNE loader
defaults missing optional properties to zero. The tool also supports an offline
integrity check of the committed derivative.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import pathlib
import re
import struct
import urllib.request


SOURCE_URL = (
    "https://huggingface.co/datasets/Voxel51/gaussian_splatting/resolve/main/"
    "FO_dataset/playroom/point_cloud/iteration_30000/point_cloud.ply?download=true"
)
SOURCE_BYTES = 475_263_524
SOURCE_SHA256 = "c6fddedf6c7b412d078bbbaa1826e7a1b258f75f862c5190dc50a646243d7d9e"
STRIDE = 6
SOURCE_RECORD_BYTES = 248
OUTPUT_RECORD_BYTES = 56
OUTPUT_RECORDS = 319_397
OUTPUT_BYTES = 17_886_594
OUTPUT_SHA256 = "88f4ebffee1fdb1f558625b23fb93ad4c257a1d7dae5dc00443596c390717022"

OUTPUT_HEADER = b"""ply
format binary_little_endian 1.0
element vertex {vertex_count}
property float x
property float y
property float z
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
"""


def sha256(path: pathlib.Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def split_header(stream) -> tuple[bytes, int]:
    lines: list[bytes] = []
    vertex_count = -1
    while True:
        line = stream.readline()
        if not line:
            raise ValueError("PLY header has no end_header")
        lines.append(line)
        if line.startswith(b"element vertex "):
            vertex_count = int(line.split()[2])
        if line.rstrip(b"\r\n") == b"end_header":
            break
    if vertex_count < 0:
        raise ValueError("PLY header has no vertex count")
    return b"".join(lines), vertex_count


def prepare(
    source: pathlib.Path,
    output: pathlib.Path,
    stride: int = STRIDE,
    preserve_all_properties: bool = False,
) -> tuple[int, int]:
    if source.stat().st_size != SOURCE_BYTES:
        raise ValueError(f"unexpected source byte length: {source.stat().st_size}")
    actual_hash = sha256(source)
    if actual_hash != SOURCE_SHA256:
        raise ValueError(f"unexpected source SHA-256: {actual_hash}")

    with source.open("rb") as input_stream:
        header, source_count = split_header(input_stream)
        selected_count = (source_count + stride - 1) // stride
        output.parent.mkdir(parents=True, exist_ok=True)
        with output.open("wb") as output_stream:
            if preserve_all_properties:
                output_stream.write(
                    re.sub(
                        rb"element vertex \d+",
                        f"element vertex {selected_count}".encode(),
                        header,
                        count=1,
                    )
                )
            else:
                output_stream.write(
                    OUTPUT_HEADER.replace(b"{vertex_count}", str(selected_count).encode())
                )
            for index in range(source_count):
                record = input_stream.read(SOURCE_RECORD_BYTES)
                if len(record) != SOURCE_RECORD_BYTES:
                    raise ValueError(f"truncated Gaussian record {index}")
                if index % stride == 0:
                    if preserve_all_properties:
                        output_stream.write(record)
                        continue
                    # x/y/z, f_dc_0..2, opacity, scale_0..2, rot_0..3.
                    selected = (
                        record[0:12]
                        + record[24:36]
                        + record[216:248]
                    )
                    if len(selected) != OUTPUT_RECORD_BYTES:
                        raise AssertionError("unexpected prepared Gaussian size")
                    output_stream.write(selected)
            if input_stream.read(1):
                raise ValueError("unexpected bytes after final Gaussian record")
    return source_count, selected_count


def read_colmap_cameras(path: pathlib.Path) -> dict[int, dict[str, object]]:
    model_param_counts = {0: 3, 1: 4}
    cameras: dict[int, dict[str, object]] = {}
    with path.open("rb") as stream:
        count = struct.unpack("<Q", stream.read(8))[0]
        for _ in range(count):
            camera_id, model_id = struct.unpack("<ii", stream.read(8))
            width, height = struct.unpack("<QQ", stream.read(16))
            param_count = model_param_counts.get(model_id)
            if param_count is None:
                raise ValueError(f"unsupported COLMAP camera model id {model_id}")
            params = struct.unpack(f"<{param_count}d", stream.read(8 * param_count))
            cameras[camera_id] = {
                "model_id": model_id,
                "width": width,
                "height": height,
                "params": params,
            }
    return cameras


def qvec_to_rotation(qvec: tuple[float, ...]) -> list[list[float]]:
    w, x, y, z = qvec
    return [
        [1 - 2 * y * y - 2 * z * z, 2 * x * y - 2 * w * z, 2 * x * z + 2 * w * y],
        [2 * x * y + 2 * w * z, 1 - 2 * x * x - 2 * z * z, 2 * y * z - 2 * w * x],
        [2 * x * z - 2 * w * y, 2 * y * z + 2 * w * x, 1 - 2 * x * x - 2 * y * y],
    ]


def transpose(matrix: list[list[float]]) -> list[list[float]]:
    return [list(row) for row in zip(*matrix)]


def mat_vec(matrix: list[list[float]], vector: tuple[float, ...]) -> list[float]:
    return [sum(a * b for a, b in zip(row, vector)) for row in matrix]


def rotation_to_quaternion(matrix: list[list[float]]) -> list[float]:
    trace = matrix[0][0] + matrix[1][1] + matrix[2][2]
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (matrix[2][1] - matrix[1][2]) / scale
        y = (matrix[0][2] - matrix[2][0]) / scale
        z = (matrix[1][0] - matrix[0][1]) / scale
    elif matrix[0][0] > matrix[1][1] and matrix[0][0] > matrix[2][2]:
        scale = math.sqrt(1.0 + matrix[0][0] - matrix[1][1] - matrix[2][2]) * 2.0
        w = (matrix[2][1] - matrix[1][2]) / scale
        x = 0.25 * scale
        y = (matrix[0][1] + matrix[1][0]) / scale
        z = (matrix[0][2] + matrix[2][0]) / scale
    elif matrix[1][1] > matrix[2][2]:
        scale = math.sqrt(1.0 + matrix[1][1] - matrix[0][0] - matrix[2][2]) * 2.0
        w = (matrix[0][2] - matrix[2][0]) / scale
        x = (matrix[0][1] + matrix[1][0]) / scale
        y = 0.25 * scale
        z = (matrix[1][2] + matrix[2][1]) / scale
    else:
        scale = math.sqrt(1.0 + matrix[2][2] - matrix[0][0] - matrix[1][1]) * 2.0
        w = (matrix[1][0] - matrix[0][1]) / scale
        x = (matrix[0][2] + matrix[2][0]) / scale
        y = (matrix[1][2] + matrix[2][1]) / scale
        z = 0.25 * scale
    return [x, y, z, w]


def read_colmap_image(path: pathlib.Path, camera_name: str) -> dict[str, object]:
    with path.open("rb") as stream:
        count = struct.unpack("<Q", stream.read(8))[0]
        for _ in range(count):
            image_id = struct.unpack("<i", stream.read(4))[0]
            qvec = struct.unpack("<4d", stream.read(32))
            tvec = struct.unpack("<3d", stream.read(24))
            camera_id = struct.unpack("<i", stream.read(4))[0]
            name_bytes = bytearray()
            while (byte := stream.read(1)) != b"\0":
                if not byte:
                    raise ValueError("truncated COLMAP image name")
                name_bytes.extend(byte)
            name = name_bytes.decode("utf-8")
            point_count = struct.unpack("<Q", stream.read(8))[0]
            stream.seek(point_count * 24, 1)
            if name == camera_name:
                rotation_world_to_camera = qvec_to_rotation(qvec)
                rotation_camera_to_world = transpose(rotation_world_to_camera)
                center = [
                    -value
                    for value in mat_vec(rotation_camera_to_world, tvec)
                ]
                # RNE cameras use +X right, +Y up, and -Z forward. COLMAP uses
                # +X right, +Y down, and +Z forward.
                basis = [
                    [
                        rotation_camera_to_world[row][0],
                        -rotation_camera_to_world[row][1],
                        -rotation_camera_to_world[row][2],
                    ]
                    for row in range(3)
                ]
                return {
                    "image_id": image_id,
                    "camera_id": camera_id,
                    "name": name,
                    "center": center,
                    "rne_camera_basis_columns": basis,
                    "rne_camera_quaternion_xyzw": rotation_to_quaternion(basis),
                }
    raise ValueError(f"COLMAP camera not found: {camera_name}")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--source",
        type=pathlib.Path,
        default=pathlib.Path("target/rne-real-indoor-source/playroom_30000.ply"),
    )
    parser.add_argument(
        "--output",
        type=pathlib.Path,
        default=pathlib.Path(
            "assets/environments/voxel51_playroom_3dgs/playroom_dc_every6.ply"
        ),
    )
    parser.add_argument("--check", action="store_true")
    parser.add_argument("--stride", type=int, default=STRIDE)
    parser.add_argument("--preserve-all-properties", action="store_true")
    parser.add_argument("--colmap-cameras", type=pathlib.Path)
    parser.add_argument("--colmap-images", type=pathlib.Path)
    parser.add_argument("--camera-name", default="DSC05572.jpg")
    args = parser.parse_args()

    if args.colmap_cameras or args.colmap_images:
        if not args.colmap_cameras or not args.colmap_images:
            raise SystemExit("--colmap-cameras and --colmap-images must be used together")
        camera = read_colmap_image(args.colmap_images, args.camera_name)
        intrinsics = read_colmap_cameras(args.colmap_cameras)[camera["camera_id"]]
        params = intrinsics["params"]
        if intrinsics["model_id"] == 0:
            fx = fy = params[0]
            cx, cy = params[1:]
        else:
            fx, fy, cx, cy = params
        camera["intrinsics"] = {
            "width": intrinsics["width"],
            "height": intrinsics["height"],
            "fx": fx,
            "fy": fy,
            "cx": cx,
            "cy": cy,
            "fov_y_rad": 2.0 * math.atan(intrinsics["height"] / (2.0 * fy)),
        }
        print(json.dumps(camera, indent=2))
        return

    if args.check:
        if not args.output.is_file():
            raise SystemExit(f"missing prepared asset: {args.output}")
        actual_bytes = args.output.stat().st_size
        actual_hash = sha256(args.output)
        with args.output.open("rb") as stream:
            header, actual_records = split_header(stream)
        if actual_records != OUTPUT_RECORDS:
            raise SystemExit(f"unexpected prepared record count: {actual_records}")
        if header != OUTPUT_HEADER.replace(
            b"{vertex_count}", str(OUTPUT_RECORDS).encode()
        ):
            raise SystemExit("unexpected prepared PLY header")
        if actual_bytes != OUTPUT_BYTES:
            raise SystemExit(f"unexpected prepared byte length: {actual_bytes}")
        if actual_hash != OUTPUT_SHA256:
            raise SystemExit(f"unexpected prepared SHA-256: {actual_hash}")
        print(
            f"prepared={args.output} records={actual_records} "
            f"bytes={actual_bytes} sha256={actual_hash}"
        )
        return

    if not args.source.is_file():
        args.source.parent.mkdir(parents=True, exist_ok=True)
        print(f"downloading {SOURCE_URL}")
        urllib.request.urlretrieve(SOURCE_URL, args.source)
    if args.stride < 1:
        raise SystemExit("--stride must be positive")
    source_count, selected_count = prepare(
        args.source, args.output, args.stride, args.preserve_all_properties
    )
    print(
        f"source_records={source_count} selected_records={selected_count} "
        f"bytes={args.output.stat().st_size} sha256={sha256(args.output)}"
    )


if __name__ == "__main__":
    main()
