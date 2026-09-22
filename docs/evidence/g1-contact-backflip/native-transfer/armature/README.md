# Native generalized-armature comparison

These are rejected native Rapier probes. They are not successful backflips.
All use the 34.13385728 kg, 23-joint G1 comparison scene, four sphere contacts
per foot, friction 0.7, and the passive-loss profile (damping 0.05,
regularized Coulomb 0.2 Nm). Body mesh/self-collision remain disabled.
The controller retains 2 ms held commands and a 2 ms delay. No base state,
base velocity or external base wrench is imposed.

## Implementation and validation

`RevoluteJointArmature` adds a constant generalized diagonal of 0.01 kg·m²
to each of the 23 actuated coordinates. It does not increase robot mass or
replace link spatial inertia. A small, licensed Rapier 0.22 patch updates both
acceleration and constraint inertia and preserves coordinate remapping.
The experimental backend feature is enabled by example 114; normal backend
builds still compile with the unmodified upstream dependency. See
[ADR 030](../../../../adr/030-revolute-joint-armature.md) and
[the reviewable vendor patch](../../../../../third_party/rapier3d/RNE.patch).

Validation passed: 44 physics/backend tests, all 17 vendor unit tests,
5 example tests, targeted release Clippy with warnings denied, default-feature
compilation against the original registry source, and workspace formatting.
The unchanged vendored dependency emits 19 upstream compiler warnings.
Full workspace/xtask and the immutable SemVer build were not rerun to retain
30 GiB free. These targeted checks are not a claim of full CI completion.

The zero-armature rollout exactly reproduces **all prior fields and frames**
from `../contact-diagnostics/flip-loss-16.json`; the new coefficient field is
additive. Analytic tests check fixed and floating bases under direct torque
and torque-limited motor impulses, component removal and invalid values.

## Standing

At 0.5 ms with velocity motors and armature, the final-second maximum base
speed is 0.0265763 m/s, versus 0.1025667 without armature. Minimum upright is
0.999999852. Active ground pairs persist for all 2000 final-second samples.
There are 52 zero-impulse steps, longest interval 1.5 ms; the strict existing
positive-impulse standing gate therefore remains false. The lowest sphere's
y range is -0.04238 to +0.00012 mm. No gate has been changed.

## Backflips

| Case | Peak measured speed/rating | Final signed rotation (rad) | Last sampled base height (m) | Outcome |
|---|---:|---:|---:|---|
| Velocity, zero armature, 0.5 ms | 1.704054 | -5.323281 | 0.113961 | Collapsed |
| Velocity, armature 0.01, 0.5 ms | 1.540063 | -5.400714 | -0.008575 | Collapsed |
| Direct torque, armature 0.01, 0.125 ms | 1.730680 | -5.263576 | 0.095620 | Collapsed |
| Direct torque, opening 5.5 rad, 0.125 ms | 1.105067 | -4.910096 | 0.103836 | Collapsed |
| Direct torque, opening 5.0 rad, 0.125 ms | 1.029548 | -5.363584 | -0.025895 | Collapsed |

Every flip terminates on collapse at maneuver time 1.2 s. Direct torque
with the original opening threshold reaches approximately one rotation in
flight, but the base is already only 0.217 m high at 1.060125 s and landing
fails. Earlier opening at 5.0 rad lowers the peak speed ratio to 1.02955,
but the robot under-rotates and falls; speed compliance alone is not success.
These two timing probes change only parameter 11, retaining the same armature,
plant, controller mode and step. No native-success GIF was generated.

Remaining differences include source constraint friction versus smooth
friction, contact/limit solvers, full body/self-contact, and Rapier's free-root
numerical angular damping. Matching armature does not establish complete plant
parity or hardware readiness. Launch/tuck/opening must be optimized for this
native plant, and any resulting landing requires full validation.

## Reproduction

From the repository root, use fresh output paths and retain at least 30 GiB
free. The scene audit describes the URDF; armature is added at runtime by the
candidate and does not change that generated URDF's hash.

```bash
python3 scripts/g1_native_model.py --source-soles --independent-soles \
  --source-passive-loss --output target/research/g1-armature-model
cargo run --release -p g1_backflip_gif --example 114_g1_backflip_gif -- \
  --native-probe --native-declared --native-velocity-servo \
  --native-scene target/research/g1-armature-model/scene.rne.scene.toml \
  --native-dt-us 500 --native-stop-on-fall \
  --native-candidate docs/evidence/g1-contact-backflip/native-transfer/armature/candidate.json \
  --native-output target/research/g1-armature-flip.json
```

For standing, replace `--native-stop-on-fall` with `--native-stand`.
For direct torque, omit `--native-velocity-servo` and use step 125.
The two earlier-opening cases use the corresponding `open-*-candidate.json`.
For zero armature, use `../compound-soles/candidate.json` at step 500 with
velocity motors. Use different output paths for every run.
`sources.json` pins source, binary, candidate/rollout hashes and the scene audit.
