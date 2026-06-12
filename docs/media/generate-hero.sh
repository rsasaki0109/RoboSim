#!/usr/bin/env bash
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
MEDIA="$ROOT/docs/media"
POSTER="$MEDIA/rne-hero.png"
GIF="$MEDIA/rne-hero.gif"

if [[ "${RNE_SKIP_GPU:-}" == "1" ]]; then
  echo "RNE_SKIP_GPU set; keeping existing README media"
  exit 0
fi

if cargo run --manifest-path "$ROOT/examples/18_readme_hero/Cargo.toml" --example 18_readme_hero; then
  echo "rendered poster from mesh diff-drive scene"
else
  echo "wgpu render failed; keeping existing poster at $POSTER"
fi

if [[ ! -f "$POSTER" ]]; then
  echo "missing $POSTER; aborting GIF generation"
  exit 1
fi

python3 - "$POSTER" "$GIF" <<'PY'
import sys
from pathlib import Path

from PIL import Image

poster = Path(sys.argv[1])
gif = Path(sys.argv[2])
src = Image.open(poster).convert("RGB")
w, h = src.size
frames = []
for i in range(24):
    t = i / 23
    scale = 1.0 + 0.05 * t
    nw, nh = int(w / scale), int(h / scale)
    x = int((w - nw) * (0.38 + 0.08 * t))
    y = int((h - nh) * (0.30 + 0.06 * t))
    crop = src.crop((x, y, x + nw, y + nh)).resize((800, 450), Image.Resampling.LANCZOS)
    frames.append(crop.convert("P", palette=Image.Palette.ADAPTIVE, colors=128))

frames[0].save(
    gif,
    save_all=True,
    append_images=frames[1:],
    duration=120,
    loop=0,
    optimize=True,
    disposal=2,
)
print(f"updated {poster} and {gif} ({gif.stat().st_size} bytes)")
PY
