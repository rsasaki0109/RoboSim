# Independent native sole primitives — landing still fails

The comparison G1 uses **34.13385728 kg**, 23 movable joints, and two compound
foot colliders containing **four independent spheres each**. Their positions
and 2 mm radii match the source sole profile. Ray tests distinguish the empty
space between spheres from the old bounding box. Declared body mass/inertia
is retained; parts share the original foot body's material and entity identity.

The opt-in TOML extension `urdf.preserve_collision_parts` is read separately
from public asset structs. `attach_urdf_collision_parts` is an additive importer
API. Existing public asset/spawn structs and the physics error enum are retained.
The legacy option-off path continues to use its earlier bounding geometry.

## Recorded result

All runs use force-based velocity servos, 0.7 sliding friction, a 120 N m knee
ceiling, and the same source maneuver. No base state or base wrench is imposed.

| Step | Standing min upright, final second | Standing max base speed (m/s) | Continuous foot contact | Standing gate | Flip max speed/rating | Flip landing |
|---|---:|---:|---|---|---:|---|
| 0.5 ms | 0.99999218 | 0.09993762 | no | Fail | 2.31648 | Collapses |
| 0.125 ms | 0.99999879 | 0.04400468 | no | Fail | 2.18477 | Collapses |

Standing remains upright, but intermittent loss of positive foot contact
impulse fails the unchanged standing gate. The finer step reduces body speed;
it does not turn the result into a pass. Both flip runs collapse at maneuver
time 1.2 s. `qualified_backflip` remains false and no success GIF is generated.

The foot-box approximation is resolved in this profile. Body meshes and
self-collision, source joint armature/passive losses, and controller/contact
solver differences remain unqualified. The change does not establish full
plant equivalence or real-robot readiness.

## Reproducibility and compatibility

- The repeated 0.5 ms compound flip is byte-identical.
- With the option disabled, every recorded physical frame and peak joint speed
  matches the earlier box-model recording in `../model-alignment/`.
- Recordings retain source/binary hashes. The standing runs were produced at
  `da27b96`; the later API-compatible extension implementation is checked against
  the original flip recordings. Source versions are explicit in `sources.json`.
- Public asset/spawn struct declarations and the physics error enum were checked
  unchanged against the prior head `ed1b1c3`. A full SemVer baseline build was
  not run locally; it remains a separate CI gate.

```bash
python3 scripts/g1_native_model.py --source-soles --independent-soles \
  --output target/research/native-compound-model
cargo run --release -p g1_backflip_gif --example 114_g1_backflip_gif -- \
  --native-probe --native-declared --native-velocity-servo \
  --native-scene target/research/native-compound-model/scene.rne.scene.toml \
  --native-dt-us 500 --native-stop-on-fall \
  --native-candidate docs/evidence/g1-contact-backflip/native-transfer/compound-soles/candidate.json \
  --native-output target/research/compound-flip.json
```

Use `--native-stand` instead of `--native-stop-on-fall` for standing, and
`--native-dt-us 125` for the fine step. Output paths must be fresh. Generated
mesh paths are absolute, so regenerate after relocating the checkout.

Validation: 141 library tests across physics, Rapier, URDF import and assets;
four example tests; eight Python tests; targeted release Clippy with warnings
denied; Ruff and workspace formatting. The full workspace/xtask suite was not
rerun to preserve the 30 GiB disk reserve. See
[ADR 029](../../../../adr/029-compound-contact-primitives.md) for the contract.
