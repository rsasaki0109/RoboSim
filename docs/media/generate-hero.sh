#!/usr/bin/env bash
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

if [[ "${RNE_SKIP_GPU:-}" == "1" ]]; then
  echo "RNE_SKIP_GPU set; keeping existing README media"
  exit 0
fi

if ! command -v ffmpeg >/dev/null; then
  echo "ffmpeg not found; run 18_readme_hero anyway and install ffmpeg for GIF output" >&2
fi

cargo run --manifest-path "$ROOT/examples/18_readme_hero/Cargo.toml" --example 18_readme_hero

echo "updated $ROOT/docs/media/rne-hero.png and $ROOT/docs/media/rne-hero.gif"
