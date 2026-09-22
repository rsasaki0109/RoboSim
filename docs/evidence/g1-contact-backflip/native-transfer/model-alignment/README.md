# Native model alignment and automatic search — landing remains unsuccessful

The generated comparison model has **34.13385728 kg and 23 movable joints**,
verified both from URDF data and from the live native articulated model at
simulation time zero. Four empty fixed sensor frames are omitted so the native
importer's 1 kg fallback does not add 4 kg. All physical inertia is retained.
The source URDF and legacy importer defaults are unchanged.

| Native comparison | Standing gate | Final-second minimum upright | Maximum base speed (m/s) | Continuous foot contact | Backflip |
|---|---|---:|---:|---|---|
| Declared mass, original sole dimensions | Pass | 0.99999368 | 0.06914131 | yes | Collapses |
| Declared mass, source sole dimensions | Fail | 0.99999469 | 0.08101159 | no | Collapses |

Both use force-based velocity servos, a 120 N m knee ceiling, 0.7 sliding
friction, a 0.5 ms physics step, and the source candidate. Standing runs have
one second of preparation and five seconds of evaluation. The second model
remains upright but intermittently loses foot contact; its standing gate is
not relaxed. Both flip runs hit the collapse guard at maneuver time 1.2 s.

## Remaining physical differences

- The original URDF's four 5 mm-radius foot spheres become a single native box,
  with a bounding footprint of 0.18 m by 0.07 m.
- The source contact plant instead uses 2 mm-radius spheres at x = -0.05/0.09 m,
  y = +/-0.025 m, with their bottom at z = -0.035 m. The optional source-sole
  profile matches these sphere definitions, but native import still merges them
  into a box (bounding footprint 0.144 m by 0.054 m).
- Twenty-one URDF mesh collision elements are disabled. Native mesh import
  currently approximates them by boxes; merely enabling it does not reproduce
  the source convex contact model. Self-collision is disabled.
- The source adds joint armature 0.01, viscous damping 0.05, and Coulomb loss 0.2.
  These are not matched by this native probe. Contact solvers and motor models
  also differ.

Consequently every generated audit has `qualification_ready: false`. Matching
mass and sole dimensions is useful for diagnosis, not physical equivalence.
The next model work is independent sole contacts and motor/inertia comparison,
followed by complete body/contact qualification.

## Native search result

`search/` contains all 13 candidates and native rollouts from one bounded
coordinate-pattern round in the mass-only comparison. The six coordinates
cover crouch depth, launch hip, tuck hip/knee, opening angle and landing hip.
The optimizer evaluates a deterministic ordered batch with four workers and
uses measured speed/position excess, completion, rotation and posture in its
objective. It is non-RL and has no external Python dependencies.

All candidates fail landing. Minimum loss selects candidate 9, which reduces
the opening angle from 6.03570354 to 5.78570354 rad; its peak joint-speed/rating
is **1.59451** and it still collapses. The baseline's peak ratio is 3.07911.
A lower loss is never a pass or proof of optimality. This single local round
also does not establish that a successful trajectory is impossible.

## Reproduce

From a checkout containing source commit `7355284`, build example 114, then:

```bash
python3 scripts/g1_native_model.py --output target/research/native-mass-model
cargo run --release -p g1_backflip_gif --example 114_g1_backflip_gif -- \
  --native-probe --native-declared \
  --native-scene target/research/native-mass-model/scene.rne.scene.toml \
  --native-model-check

python3 scripts/g1_native_search.py \
  --binary target/release/examples/114_g1_backflip_gif \
  --scene target/research/native-mass-model/scene.rne.scene.toml \
  --candidate docs/evidence/g1-contact-backflip/native-transfer/model-alignment/candidate.json \
  --output target/research/native-search --rounds 1 --workers 4
```

Use `--source-soles` during preparation for the second model. For a single
native rollout, pass `--native-velocity-servo --native-dt-us 500`, the generated
`--native-scene`, `--native-candidate`, and a fresh `--native-output` filename;
add `--native-stand` or `--native-stop-on-fall` for the corresponding baseline.

Generation embeds absolute mesh paths; regenerate after relocating the
checkout. Audits preserve the source and generated URDF hashes for this run.
The search manifest stores the built binary hash; `sources.json` records source
and artifact hashes. Scene-path strings can differ across checkouts.

Validation: seven Python tests (physical-data preservation, sole dimensions,
refusal to remove physical/branch links, disk reserves, loss ranking and
serial/parallel selection), Ruff, four Rust example tests, targeted release
Clippy with `-D warnings`, workspace formatting, and the real model/rollouts
above. Full workspace tests/xtask were not rerun to preserve the 30 GiB reserve.
No successful native GIF is produced for these failed rollouts.
