# Native G1 backflip: refinement passed

The same non-RL candidate completes one backward revolution, feet-only landing,
and recovery through 15 seconds at **500, 125 and 62.5 µs**. Every run passes
the unchanged recorded physical gates, including measured actuator effort,
joint speed/position, continuous final-second foot support, no non-foot ground
contact and no penetrating nonadjacent self-contact.

| Step | Rotation (rad) | Peak speed / rating | Final-second maximum base speed (m/s) | Standing error |
|---|---:|---:|---:|---:|
| 500 µs | -6.27857736 | 1.04385118 | 0.00054883 | 0.00465757 |
| 125 µs | -6.28434162 | 1.04447365 | 0.00106624 | 0.00467077 |
| 62.5 µs | -6.28346599 | 1.03942852 | 0.00000000 | 0.00457211 |

`verification.json` records the combined result. `verify.py` recomputes all
physical audits, verifies recording/model/source hashes, checks that controller
parameters match the executed candidates, and requires the same producer,
model settings and torque ceilings across all three distinct timesteps.
Six verifier tests include corrupted bytes, duplicate timesteps, changed
controllers/producers and a speed violation with otherwise updated checksums.

```bash
python3 docs/evidence/g1-contact-backflip/native-transfer/selected005-long-validation/verify.py
python3 docs/evidence/g1-contact-backflip/native-transfer/selected005-long-validation/test_verify.py
```

This establishes the native simulated outcome under timestep refinement.
It does not establish formal trajectory convergence or hardware readiness.
Full repository CI is tracked separately in PR #311.

## Model and controller

The candidate is the +0.005 rad landing-knee refinement selected by the existing
five-second diagnostic loss in `../fine-knee-refinement/`. It uses direct joint
effort, 0.001 Nm command headroom within the original physical torque ceilings,
0.01 kg m² armature and a ten-second recovery. There is no imposed base path
or external root wrench. The 34.13385728 kg model enables 21 convex body
colliders, two four-sphere soles and all body-ground contact. Thirty-eight
structural exclusions derive from fixed clusters and directly connected links;
nonadjacent self-collision remains enabled. This is the declared source-sole
profile, not a claim of exact hardware or MuJoCo contact-plant equivalence.

Producer source: `0c84bfdec8bdbdd3a4d9724e5bf13469fc895243`.
Binary SHA-256: `dcc339bfa4d739a991150b1ec2e14559c7b7524993e54e35068a43b6e82aa2c8`.
The `model-*` files preserve the exact input scene, robot configuration, URDF
and preparation audit. `model-provenance.json` verifies all 30 source URDF/mesh
files against the producer commit and the generated URDF against its preparation
hash. The snapshot was taken after the coarse run and during the finer runs.
The preparation audit predates the candidate's armature override; the rollouts
record the applied 0.01 kg m². Absolute mesh paths require regeneration on
another checkout. Raw probe/renderer `qualification pending` fields remain
unchanged: these components do not run the combined refinement/contact audit;
`verification.json` supplies that independent verdict.

## Reproduce the physical runs

The commands below use the PR checkout, which retains the native implementation
with subsequent unrelated demo repairs. To rebuild the original producer commit,
retain this evidence directory separately and pass its absolute candidate path.
Keep at least 30 GiB free and use a fresh output directory. Source asset hashes
are checked by the verifier; new builds can differ in their executable hash.

```bash
python3 scripts/g1_native_model.py --source-soles --independent-soles \
  --source-passive-loss --full-contact --output target/research/g1-native-reproduce
cargo build --release --locked -p g1_backflip_gif --example 114_g1_backflip_gif
for dt in 500 125 62.5; do
  "${CARGO_TARGET_DIR:-target}/release/examples/114_g1_backflip_gif" \
    --native-probe --native-declared --native-structural-filter \
    --native-scene target/research/g1-native-reproduce/scene.rne.scene.toml \
    --native-candidate docs/evidence/g1-contact-backflip/native-transfer/selected005-long-validation/62p5-candidate-0000.json \
    --native-dt-us "$dt" --native-duration-s 15 --native-stop-on-fall \
    --native-output "target/research/g1-native-reproduce/rollout-$dt.json"
  python3 scripts/g1_native_audit.py "target/research/g1-native-reproduce/rollout-$dt.json"
done
```

## GIF provenance

[The native GIF](../../../../media/unitree-g1-robosim-native-backflip.gif) now
shows the **62.5 µs successful full-contact run**. All 1,600 native recorded poses
are validated; 204 frames (15.21 s) are encoded with the authored URDF materials.
Playback executes zero physics ticks and introduces no synthetic base motion.
`gif-provenance.json` pins the recording, visual URDF and output GIF hashes.
The prior foot-only diagnostic remains in git history at commit `021d40d`.
The coarse render preflight is also retained; it did not produce the final GIF.
