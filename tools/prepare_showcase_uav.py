#!/usr/bin/env python3
"""Prepare the README-sized PLATEAU UAV media from example 46 output."""

from __future__ import annotations

import argparse
import hashlib
import pathlib
import shutil
import subprocess


SOURCE_GIF = pathlib.Path("docs/media/plateau-uav.gif")
SOURCE_POSTER = pathlib.Path("docs/media/plateau-uav.png")
OUTPUT_GIF = pathlib.Path("docs/media/showcase-uav.gif")
OUTPUT_POSTER = pathlib.Path("docs/media/showcase-uav.png")
SOURCE_GIF_SHA256 = "46b7c3e54a073d92bc61fa12076b9a35b6a21229b9a1caaee9ccbc2c806648fd"
SOURCE_POSTER_SHA256 = "088125812625c29da69098bee67169471aa3760c5dfbddda05544fad2201f8cb"
OUTPUT_GIF_BYTES = 4_329_461
OUTPUT_POSTER_BYTES = 439_474
OUTPUT_GIF_SHA256 = "f78ebe4f843eb61998cd3ac0e70a2b5c127fe0d9f006c605d354a88f245e3a76"
OUTPUT_POSTER_SHA256 = "9260530e1806efc1f445eb7285a3c434edc0a18abda8f66b50aad719eaf8f282"


def sha256(path: pathlib.Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def require_file(path: pathlib.Path, expected_hash: str) -> None:
    if not path.is_file():
        raise SystemExit(f"missing media source: {path}")
    actual_hash = sha256(path)
    if actual_hash != expected_hash:
        raise SystemExit(f"unexpected SHA-256 for {path}: {actual_hash}")


def verify_output(path: pathlib.Path, expected_bytes: int, expected_hash: str) -> None:
    if not path.is_file():
        raise SystemExit(f"missing prepared media: {path}")
    actual_bytes = path.stat().st_size
    actual_hash = sha256(path)
    if actual_bytes != expected_bytes or actual_hash != expected_hash:
        raise SystemExit(
            f"prepared media drifted: {path} bytes={actual_bytes} sha256={actual_hash}"
        )


def prepare() -> None:
    ffmpeg = shutil.which("ffmpeg")
    if ffmpeg is None:
        raise SystemExit("ffmpeg is required to prepare showcase UAV media")
    require_file(SOURCE_GIF, SOURCE_GIF_SHA256)
    require_file(SOURCE_POSTER, SOURCE_POSTER_SHA256)
    subprocess.run(
        [
            ffmpeg,
            "-y",
            "-loglevel",
            "error",
            "-i",
            str(SOURCE_GIF),
            "-filter_complex",
            "[0:v]fps=6,scale=960:540:flags=lanczos,split[s0][s1];"
            "[s0]palettegen=max_colors=32:stats_mode=diff[p];"
            "[s1][p]paletteuse=dither=bayer:bayer_scale=3",
            "-loop",
            "0",
            str(OUTPUT_GIF),
        ],
        check=True,
    )
    subprocess.run(
        [
            ffmpeg,
            "-y",
            "-loglevel",
            "error",
            "-i",
            str(SOURCE_POSTER),
            "-vf",
            "scale=960:540:flags=lanczos",
            "-frames:v",
            "1",
            str(OUTPUT_POSTER),
        ],
        check=True,
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    if not args.check:
        prepare()
    verify_output(OUTPUT_GIF, OUTPUT_GIF_BYTES, OUTPUT_GIF_SHA256)
    verify_output(OUTPUT_POSTER, OUTPUT_POSTER_BYTES, OUTPUT_POSTER_SHA256)
    print(
        f"showcase_uav gif_bytes={OUTPUT_GIF_BYTES} gif_sha256={OUTPUT_GIF_SHA256} "
        f"poster_bytes={OUTPUT_POSTER_BYTES} poster_sha256={OUTPUT_POSTER_SHA256}"
    )


if __name__ == "__main__":
    main()
