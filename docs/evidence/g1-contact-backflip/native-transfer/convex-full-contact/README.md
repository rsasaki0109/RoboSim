# Convex full-contact import: structural-contact mismatch exposed

The native scene now loads all 21 source body collision meshes as convex hulls,
retains two four-part sole colliders, enables self-collision and preserves
34.13385728 kg / 23 movable joints. These are diagnostic 500 µs runs; neither
qualifies a native full-body backflip.

Standing completes 5 s with continuous foot contact and final-second speed
below 0.001740 m/s, but upright cosine is only 0.975959 (threshold > 0.99).
The same candidate's maneuver fails at 1.366 s and rotates +1.935609 rad,
not a backward revolution. Recorded joint speed stays below rating in these
failed trials; this does not make the motion feasible.

The unfiltered contact audit exposes structural collisions from the first
physics step: fixed `logo_link` / `torso_link` reaches 150.059525 N·s, knee /
hip-yaw links reach about 16.25 N·s, and fixed head / torso reaches 8.84 N·s.
These contacts are reported throughout standing. Collisions between fixed
parts and joint-connected bodies must be modeled consistently before tuning
this plant. The next step is a topology-derived structural contact policy,
while retaining ground and nonadjacent self-contact; no arbitrary observed
failure pairs should be suppressed. This experiment retains the unfiltered
failure rather than declaring full-contact success or loosening upright gates.

`standing.json.gz` and `flip.json.gz` contain raw native recordings. Pair audits
include preparation, report counts, first time and peak normal impulse.
`observer-regression.json.gz` repeats the earlier foot-only held candidate:
all 600 frames and all shared numerical metrics match exactly. Only the scene
path spelling and added diagnostic metadata differ. Thus the contact observer
and ground-only support detection preserve the previous foot-only trajectory.

Regenerate with `scripts/g1_native_model.py --source-soles --independent-soles
--source-passive-loss --full-contact --output NEW_MODEL_DIR`. Run example 114:

```bash
$BINARY --native-probe --native-declared --native-scene "$SCENE" \
  --native-dt-us 500 --native-stop-on-fall --native-candidate candidate.json \
  --native-output "$NEW_OUTPUT"
```

Add `--native-stand` for standing. Use `--native-model-check` to inspect the
21 convex and two four-part colliders without advancing the simulation clock.
The full-contact profile changes body geometry and self-collision; it does
not assert source/native solver or structural-exclusion equivalence. The
source sole profile still uses its declared four 2 mm points per foot.

Validation: importer 38 tests, assets 65 tests, example 13 tests, Python
model/search 12 tests; targeted release Clippy and formatting pass. Full
workspace/xtask CI was not rerun to preserve the 30 GiB reserve.
