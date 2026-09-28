"""Encode a directory of PNG frames as a frame-differenced GIF.

ffmpeg's encoder re-sends the bounding box of every change, which for a fixed
camera watching several things move at once is most of the frame: the
warehouse relay came out at 7-13 MB whatever the palette or dither. Pillow's
optimizer sends only changed pixels against one shared palette, which is the
same animation at a fraction of the size.

    python3 tools/encode_gif.py FRAMES_DIR OUTPUT.gif [--fps 12] [--colors 128]
"""

from __future__ import annotations

import argparse
from pathlib import Path

from PIL import Image


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("frames", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--fps", type=float, default=12.0)
    parser.add_argument("--colors", type=int, default=128)
    args = parser.parse_args()

    paths = sorted(args.frames.glob("frame-*.png"))
    if not paths:
        raise SystemExit(f"no frame-*.png in {args.frames}")
    frames = [Image.open(path).convert("RGB") for path in paths]
    # One palette for the whole animation, built from frames spread across it
    # so a colour that only appears late still gets an entry.
    samples = [frames[i * (len(frames) - 1) // 5] for i in range(6)]
    width, height = samples[0].size
    montage = Image.new("RGB", (width, height * len(samples)))
    for index, sample in enumerate(samples):
        montage.paste(sample, (0, index * height))
    palette = montage.quantize(colors=args.colors, method=Image.Quantize.MEDIANCUT)
    indexed = [frame.quantize(palette=palette, dither=Image.Dither.NONE) for frame in frames]
    indexed[0].save(
        args.output,
        save_all=True,
        append_images=indexed[1:],
        duration=round(1000.0 / args.fps),
        loop=0,
        optimize=True,
        disposal=1,
    )
    print(f"wrote {args.output} ({args.output.stat().st_size} bytes, {len(frames)} frames)")


if __name__ == "__main__":
    main()
