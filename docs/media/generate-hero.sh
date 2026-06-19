#!/usr/bin/env bash
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
OUT_DIR="$(mktemp -d)"
trap 'rm -rf "$OUT_DIR"' EXIT

if ! command -v ffmpeg >/dev/null; then
  echo "ffmpeg is required to extract docs/media/rne-hero.png" >&2
  exit 1
fi

python "$ROOT/examples/27_mobile_manipulator_rl/house_gif_demo.py" \
  --out-dir "$OUT_DIR" \
  --width 960 \
  --height 540 \
  --max-frames 72 \
  --fps 12

cp "$OUT_DIR/house_mobile_manipulator.gif" "$ROOT/docs/media/rne-hero.gif"
ffmpeg -y -v error -i "$ROOT/docs/media/rne-hero.gif" -frames:v 1 "$ROOT/docs/media/rne-hero.png"
PYTHONPATH="$ROOT/examples/27_mobile_manipulator_rl" python - "$ROOT" <<'PY'
import os
import sys

from render_house_gif import write_metadata

root = sys.argv[1]
write_metadata(
    os.path.join(root, "docs/media/rne-hero.json"),
    gif_path=os.path.join(root, "docs/media/rne-hero.gif"),
    metadata_gif_path="rne-hero.gif",
    source={
        "kind": "demo",
        "generator": "examples/27_mobile_manipulator_rl/house_gif_demo.py",
    },
    sample_count=90,
    frame_count=72,
    max_frames=72,
    width=960,
    height=540,
    fps=12.0,
)
PY
PYTHONPATH="$ROOT/examples/27_mobile_manipulator_rl" python - "$ROOT/docs/media/rne-hero.json" <<'PY'
import sys

from render_house_gif import verify_metadata

verify_metadata(sys.argv[1])
PY

echo "updated $ROOT/docs/media/rne-hero.png, rne-hero.gif, and rne-hero.json"
