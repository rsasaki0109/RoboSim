#!/usr/bin/env python3
"""Prepare the README-sized PLATEAU UAV media from example 46 output."""

from __future__ import annotations

import argparse
import hashlib
import pathlib
import shutil
import subprocess


SOURCE_FRAMES = pathlib.Path("target/plateau-sanjo-drive-demo/uav-frames")
SOURCE_FRAME_COUNT = 144
SOURCE_POSTER = pathlib.Path("docs/media/plateau-uav.png")
OUTPUT_GIF = pathlib.Path("docs/media/showcase-uav.gif")
OUTPUT_POSTER = pathlib.Path("docs/media/showcase-uav.png")
SOURCE_FRAMES_SHA256 = "961a5e15179deec8191ab2f4ce5b77786020d45524517712bfbd8ae6f463d620"
SOURCE_POSTER_SHA256 = "ba915ae0de5a43c0c6fc80ee2b277f004745b47789a1b9be58e4539ca92cb3d9"
OUTPUT_GIF_BYTES = 4_451_885
OUTPUT_POSTER_BYTES = 585_765
OUTPUT_GIF_SHA256 = "6aeca14762682fa67df76cf527f9383963018bbc7a99e2cf7bbdd6172f2fc627"
OUTPUT_POSTER_SHA256 = "b4e10f3cc5facbf9d742bbc5daeb049a568ba6b7d1e1f24e3fed632e7c4a578d"


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


def frames_sha256() -> str:
    frames = sorted(SOURCE_FRAMES.glob("frame-*.png"))
    if len(frames) != SOURCE_FRAME_COUNT:
        raise SystemExit(
            f"expected {SOURCE_FRAME_COUNT} frames in {SOURCE_FRAMES}, found {len(frames)}; "
            "run example 46 first"
        )
    digest = hashlib.sha256()
    for frame in frames:
        digest.update(frame.name.encode())
        digest.update(bytes.fromhex(sha256(frame)))
    return digest.hexdigest()


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
    actual_frames_hash = frames_sha256()
    if actual_frames_hash != SOURCE_FRAMES_SHA256:
        raise SystemExit(f"unexpected SHA-256 for {SOURCE_FRAMES}: {actual_frames_hash}")
    require_file(SOURCE_POSTER, SOURCE_POSTER_SHA256)
    subprocess.run(
        [
            ffmpeg,
            "-y",
            "-loglevel",
            "error",
            "-framerate",
            "12",
            "-i",
            str(SOURCE_FRAMES / "frame-%03d.png"),
            "-filter_complex",
            "[0:v]fps=5,scale=960:540:flags=lanczos,split[s0][s1];"
            "[s0]palettegen=max_colors=32:stats_mode=diff[p];"
            "[s1][p]paletteuse=dither=bayer:bayer_scale=5:diff_mode=rectangle",
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
