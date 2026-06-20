#!/usr/bin/env bash
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

if [[ "${RNE_SKIP_GPU:-}" == "1" ]]; then
  echo "RNE_SKIP_GPU set; keeping existing README hero media"
  exit 0
fi

if ! command -v ffmpeg >/dev/null; then
  echo "ffmpeg is required to encode docs/media/rne-hero.gif" >&2
  exit 1
fi

cargo run \
  --manifest-path "$ROOT/examples/32_lift_pick_place_hero/Cargo.toml" \
  --example 32_lift_pick_place_hero

PYTHONPATH="$ROOT/examples/27_mobile_manipulator_rl" python - "$ROOT" <<'PY'
import json
import os
import sys

from render_house_gif import inspect_gif

root = sys.argv[1]
metadata_path = os.path.join(root, "docs/media/rne-hero.json")
gif_path = os.path.join(root, "docs/media/rne-hero.gif")
gif = inspect_gif(gif_path)
payload = {
    "schema_version": 1,
    "artifact": "rne_3d_mobile_manipulator_navigation_reach_hero",
    "gif_path": "rne-hero.gif",
    "poster_path": "rne-hero.png",
    "source": {
        "kind": "wgpu_simulation",
        "generator": "examples/32_lift_pick_place_hero",
        "scene": "assets/scenes/mm_mobile.rne.scene.toml",
        "policy": "MobileReachHeroPolicy",
        "physics": "MobileManipulatorSim/Rapier",
    },
    "fps": 12.0,
    "frame_count": gif["frame_count"],
    "width": gif["width"],
    "height": gif["height"],
    "byte_size": gif["byte_size"],
    "sha256": gif["sha256"],
    "settle_steps": 120,
    "policy_steps": 520,
    "overlays": ["base_path", "reach_target"],
}
with open(metadata_path, "w", encoding="utf-8") as handle:
    json.dump(payload, handle, indent=2, sort_keys=True)
    handle.write("\n")
print(f"updated {metadata_path} sha256={gif['sha256']}")
PY

echo "updated $ROOT/docs/media/rne-hero.png, rne-hero.gif, and rne-hero.json"
