#!/usr/bin/env python3
"""Create a byte-stable ZIP archive for one staged release bundle."""

from __future__ import annotations

import argparse
from pathlib import Path
from zipfile import ZIP_DEFLATED, ZipFile, ZipInfo

FIXED_ZIP_TIME = (1980, 1, 1, 0, 0, 0)


def archive(source: Path, output: Path) -> None:
    source = source.resolve(strict=True)
    if not source.is_dir():
        raise SystemExit(f"release bundle is not a directory: {source}")
    if output.exists():
        raise SystemExit(f"refusing to replace existing archive: {output}")

    members: list[Path] = []
    for member in source.rglob("*"):
        if member.is_symlink():
            raise SystemExit(f"release bundle contains a symbolic link: {member}")
        if member.is_file():
            members.append(member)
        elif not member.is_dir():
            raise SystemExit(f"release bundle contains an unsupported member: {member}")
    members.sort(key=lambda member: member.relative_to(source).as_posix())
    if not members:
        raise SystemExit("release bundle is empty")

    output.parent.mkdir(parents=True, exist_ok=True)
    with ZipFile(output, mode="x", compression=ZIP_DEFLATED, compresslevel=9) as zip_file:
        for member in members:
            relative = member.relative_to(source).as_posix()
            info = ZipInfo(f"{source.name}/{relative}", date_time=FIXED_ZIP_TIME)
            info.compress_type = ZIP_DEFLATED
            info.create_system = 3
            info.external_attr = (0o100644 & 0xFFFF) << 16
            zip_file.writestr(info, member.read_bytes(), compresslevel=9)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    options = parser.parse_args()
    archive(options.source, options.output)


if __name__ == "__main__":
    main()
