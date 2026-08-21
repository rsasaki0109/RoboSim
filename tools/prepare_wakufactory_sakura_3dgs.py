#!/usr/bin/env python3
"""Prepare the checked-in WakuFactory Sakura real-capture 3DGS asset.

The upstream PLY is a CC0 Scaniverse export.  To keep a normal clone small,
this script retains every fourth Gaussian record byte-for-byte and only updates
the PLY vertex count.  No generated colour, geometry, or synthetic splats are
introduced.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import pathlib
import tempfile
import urllib.request


SOURCE_URL = "https://www.wakufactory.jp/wxr/splats/data/sakura1.ply"
SOURCE_BYTES = 58_573_675
SOURCE_SHA256 = "9c508561fac30ca9f4a154b21efa3262cbe2cabcfc4c2c9cdb58ec26508ea016"
SOURCE_VERTICES = 236_178
STRIDE = 4
RECORD_BYTES = 248
OUTPUT_VERTICES = 59_045
OUTPUT_BYTES = 14_644_690
OUTPUT_SHA256 = "ac0cee7f06f2cebf9d912bf211bc87cd8f3229a0ebd59e0389daadf530389298"


def sha256(path: pathlib.Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def verify(path: pathlib.Path, expected_bytes: int, expected_sha256: str) -> None:
    actual_bytes = path.stat().st_size
    if actual_bytes != expected_bytes:
        raise ValueError(f"{path}: expected {expected_bytes} bytes, got {actual_bytes}")
    actual_sha256 = sha256(path)
    if actual_sha256 != expected_sha256:
        raise ValueError(f"{path}: expected sha256 {expected_sha256}, got {actual_sha256}")


def download_source(destination: pathlib.Path) -> None:
    request = urllib.request.Request(SOURCE_URL, headers={"User-Agent": "RNE asset preparer/1"})
    with urllib.request.urlopen(request) as response, destination.open("wb") as output:
        while block := response.read(1024 * 1024):
            output.write(block)


def prepare(source: pathlib.Path, output: pathlib.Path) -> None:
    with source.open("rb") as input_handle:
        header_lines: list[bytes] = []
        while True:
            line = input_handle.readline()
            if not line:
                raise ValueError("PLY ended before end_header")
            header_lines.append(line)
            if line == b"end_header\n":
                break
        header = b"".join(header_lines)
        expected_count = f"element vertex {SOURCE_VERTICES}".encode()
        if header.count(expected_count) != 1:
            raise ValueError("unexpected upstream PLY vertex declaration")
        header = header.replace(expected_count, f"element vertex {OUTPUT_VERTICES}".encode())

        output.parent.mkdir(parents=True, exist_ok=True)
        with output.open("wb") as output_handle:
            output_handle.write(header)
            selected = 0
            for index in range(SOURCE_VERTICES):
                record = input_handle.read(RECORD_BYTES)
                if len(record) != RECORD_BYTES:
                    raise ValueError(f"truncated Gaussian record {index}")
                if index % STRIDE == 0:
                    output_handle.write(record)
                    selected += 1
            if input_handle.read(1):
                raise ValueError("unexpected bytes after final Gaussian record")
    if selected != OUTPUT_VERTICES:
        raise ValueError(f"selected {selected} records, expected {OUTPUT_VERTICES}")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=pathlib.Path, help="use a previously downloaded PLY")
    parser.add_argument(
        "--output",
        type=pathlib.Path,
        default=pathlib.Path("assets/environments/wakufactory_sakura_3dgs/sakura1_every4.ply"),
    )
    parser.add_argument("--check", action="store_true", help="verify the committed derivative only")
    args = parser.parse_args()

    if args.check:
        verify(args.output, OUTPUT_BYTES, OUTPUT_SHA256)
        print(json.dumps({"output": str(args.output), "bytes": OUTPUT_BYTES, "sha256": OUTPUT_SHA256}))
        return

    if args.source:
        source = args.source
        verify(source, SOURCE_BYTES, SOURCE_SHA256)
        prepare(source, args.output)
    else:
        with tempfile.TemporaryDirectory(prefix="rne-wakufactory-sakura-") as temp_dir:
            source = pathlib.Path(temp_dir) / "sakura1.ply"
            download_source(source)
            verify(source, SOURCE_BYTES, SOURCE_SHA256)
            prepare(source, args.output)
    verify(args.output, OUTPUT_BYTES, OUTPUT_SHA256)
    print(json.dumps({"output": str(args.output), "bytes": OUTPUT_BYTES, "sha256": OUTPUT_SHA256}))


if __name__ == "__main__":
    main()
