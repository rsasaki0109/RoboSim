"""Create a dependency-free mobile-manipulator house GIF demo bundle.

This script is intentionally independent of ``rne_py``. It writes a synthetic
rollout CSV, an animated GIF, checksum metadata, and a small HTML preview:

    python examples/27_mobile_manipulator_rl/house_gif_demo.py
"""

import argparse
import html
import os
import sys
import tempfile

from render_house_gif import (
    demo_samples,
    render_house_frames,
    verify_metadata,
    write_demo_rollout_csv,
    write_gif,
    write_metadata,
)


DEFAULT_OUT_DIR = "house_mobile_manipulator_demo"
CSV_NAME = "house_mobile_manipulator.csv"
GIF_NAME = "house_mobile_manipulator.gif"
METADATA_NAME = "house_mobile_manipulator.json"
INDEX_NAME = "index.html"


def _write_text_atomic(path, text):
    directory = os.path.dirname(os.path.abspath(path))
    if directory:
        os.makedirs(directory, exist_ok=True)
    tmp_path = f"{path}.tmp"
    with open(tmp_path, "w", encoding="utf-8") as handle:
        handle.write(text)
    os.replace(tmp_path, path)


def _render_index(*, metadata, csv_name, gif_name, metadata_name):
    return f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>RNE mobile manipulator house GIF</title>
  <style>
    :root {{
      color-scheme: light;
      font-family: system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
      background: #f8fafc;
      color: #0f172a;
    }}
    body {{
      margin: 0;
      padding: 28px;
    }}
    main {{
      max-width: 820px;
      margin: 0 auto;
    }}
    h1 {{
      margin: 0 0 8px;
      font-size: 28px;
    }}
    p {{
      color: #475569;
    }}
    img {{
      display: block;
      width: 100%;
      height: auto;
      border: 1px solid #cbd5e1;
      border-radius: 8px;
      background: #fff;
    }}
    dl {{
      display: grid;
      grid-template-columns: max-content minmax(0, 1fr);
      gap: 8px 14px;
      margin: 20px 0;
    }}
    dt {{
      color: #64748b;
      font-weight: 700;
    }}
    dd {{
      margin: 0;
      font-variant-numeric: tabular-nums;
    }}
    nav {{
      display: flex;
      flex-wrap: wrap;
      gap: 12px;
    }}
    a {{
      color: #0f766e;
      font-weight: 700;
    }}
  </style>
</head>
<body>
  <main>
    <h1>RNE mobile manipulator house GIF</h1>
    <p>Dependency-free synthetic rollout preview.</p>
    <a href="{html.escape(gif_name)}"><img src="{html.escape(gif_name)}" alt="Mobile manipulator moving through a house scene"></a>
    <dl>
      <dt>frames</dt><dd>{metadata["frame_count"]}</dd>
      <dt>samples</dt><dd>{metadata["sample_count"]}</dd>
      <dt>size</dt><dd>{metadata["width"]}x{metadata["height"]}</dd>
      <dt>sha256</dt><dd><code>{html.escape(metadata["sha256"])}</code></dd>
    </dl>
    <nav>
      <a href="{html.escape(csv_name)}">rollout CSV</a>
      <a href="{html.escape(metadata_name)}">metadata JSON</a>
      <a href="{html.escape(gif_name)}">GIF</a>
    </nav>
  </main>
</body>
</html>
"""


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--out-dir",
        default=None,
        help=f"directory to write demo artifacts into (default: {DEFAULT_OUT_DIR})",
    )
    parser.add_argument(
        "--check",
        action="store_true",
        help="write the bundle in a temporary directory and verify it without keeping artifacts",
    )
    parser.add_argument("--width", type=int, default=360, help="GIF width in pixels")
    parser.add_argument("--height", type=int, default=240, help="GIF height in pixels")
    parser.add_argument("--max-frames", type=int, default=72, help="maximum frames to encode")
    parser.add_argument("--fps", type=float, default=12.0, help="GIF playback frames per second")
    args = parser.parse_args()
    if args.check and args.out_dir is not None:
        parser.error("--check cannot be combined with --out-dir")
    if args.width <= 0 or args.height <= 0:
        parser.error("--width and --height must be positive")
    if args.max_frames <= 1:
        parser.error("--max-frames must be greater than 1")
    if args.fps <= 0.0:
        parser.error("--fps must be positive")
    return args


def write_demo_bundle(*, out_dir, width, height, max_frames, fps):
    out_dir = os.path.abspath(out_dir)
    os.makedirs(out_dir, exist_ok=True)

    csv_path = os.path.join(out_dir, CSV_NAME)
    gif_path = os.path.join(out_dir, GIF_NAME)
    metadata_path = os.path.join(out_dir, METADATA_NAME)
    index_path = os.path.join(out_dir, INDEX_NAME)

    samples = demo_samples()
    write_demo_rollout_csv(csv_path, samples)
    frames = render_house_frames(
        samples, width=width, height=height, max_frames=max_frames
    )
    write_gif(gif_path, frames, width=width, height=height, fps=fps)
    metadata = write_metadata(
        metadata_path,
        gif_path=gif_path,
        metadata_gif_path=GIF_NAME,
        source={"kind": "demo", "rollout_csv_path": CSV_NAME},
        sample_count=len(samples),
        frame_count=len(frames),
        max_frames=max_frames,
        width=width,
        height=height,
        fps=fps,
    )
    verify_metadata(metadata_path)
    _write_text_atomic(
        index_path,
        _render_index(
            metadata=metadata,
            csv_name=CSV_NAME,
            gif_name=GIF_NAME,
            metadata_name=METADATA_NAME,
        ),
    )
    return {
        "out_dir": out_dir,
        "csv": csv_path,
        "gif": gif_path,
        "metadata": metadata_path,
        "index": index_path,
    }


def main():
    args = parse_args()
    if args.check:
        with tempfile.TemporaryDirectory(prefix="rne_house_gif_demo_check_") as temp:
            bundle = write_demo_bundle(
                out_dir=temp,
                width=args.width,
                height=args.height,
                max_frames=args.max_frames,
                fps=args.fps,
            )
            print(
                "house gif demo check ok: "
                f"frames={args.max_frames} size={args.width}x{args.height} "
                f"metadata={bundle['metadata']}"
            )
        return

    bundle = write_demo_bundle(
        out_dir=args.out_dir or DEFAULT_OUT_DIR,
        width=args.width,
        height=args.height,
        max_frames=args.max_frames,
        fps=args.fps,
    )

    print(f"house gif demo written: {bundle['out_dir']}")
    print(f"  html: {bundle['index']}")
    print(f"  gif: {bundle['gif']}")
    print(f"  metadata: {bundle['metadata']}")
    print(f"  csv: {bundle['csv']}")


if __name__ == "__main__":
    try:
        main()
    except Exception as error:
        sys.exit(str(error))
