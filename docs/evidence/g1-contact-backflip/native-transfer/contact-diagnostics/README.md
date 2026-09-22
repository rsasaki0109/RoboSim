# Native contact and passive-loss comparison

These are rejected native Rapier probes, not successful backflips. All runs
use independent four-sphere soles, mass 34.13385728 kg, 23 movable joints,
friction 0.7, bounded velocity motors, and the unchanged source candidate.
The outer command interval and delay remain 2 ms. No base state is imposed.

## Standing, 0.5 ms steps

Counters cover every physics step in the final second. An active ground pair
is distinct from a positive normal impulse. Sphere bottom extrema use the
lowest of all eight foot spheres, measured after integration against y=0.
These heights are not the contact solver's pre-integration manifold distances.

| Case | Zero impulse | No active ground pair | Longest zero impulse (ms) | Max base speed (m/s) | Lowest sphere y range (mm) | Standing passed |
|---|---:|---:|---:|---:|---:|---|
| stand-16 | 24/2000 | 0 | 2.0 | 0.099938 | -1.333776 to -0.147079 | false |
| stand-64 | 8/2000 | 0 | 0.5 | 0.044249 | -0.223935 to -0.005006 | false |
| stand-loss-16 | 14/2000 | 0 | 1.5 | 0.102567 | -1.335264 to -0.136610 | false |

The baseline has 24 zero-impulse steps, despite retaining an active ground pair
at every step. The minimum sphere height remains below the ground plane,
including unloaded samples. Thus the former positive-contact failure does not
show loss of all foot-ground contact. Solver iterations 16→64 reduce the
longest unloaded interval and geometric penetration, but the original strict
standing gate still fails. No criterion has been changed.

`stand-loss-16` replaces Rapier's default angular damping 0.1 with 0.05 and
adds `0.2 * tanh(velocity / 0.1)` Nm Coulomb loss on all 23 movable joints.
This also fails the 0.1 m/s tail-speed gate. The regularized friction differs
from source MuJoCo constraint friction; source armature 0.01 is still absent.

## Backflip with passive loss

| Step | Peak measured speed/rating | Position limit excess | Outcome |
|---|---:|---:|---|
| 0.5 ms | 1.704054 | 0 rad | Collapsed at maneuver time 1.2 s |
| 0.125 ms | 1.734277 | 0 rad | Collapsed at maneuver time 1.2 s |

Corresponding previous no-override peaks were 2.316476 and 2.184768.
Reduced overspeed does not establish a landing. `qualified_backflip` remains
false. Full body/self-contact and joint armature remain unresolved, alongside
controller/contact-solver differences. No native-success GIF was produced.

## Reproduction

Run from the repository root, using fresh output paths. The preparation tool
requires at least 30 GiB free. These comparisons reused the existing release
target and installed dependencies; no bulk downloads or environment install.

```bash
python3 scripts/g1_native_model.py --source-soles --independent-soles \
  --output target/research/contact-baseline
python3 scripts/g1_native_model.py --source-soles --independent-soles \
  --source-passive-loss --output target/research/contact-passive
cargo run --release -p g1_backflip_gif --example 114_g1_backflip_gif -- \
  --native-probe --native-declared --native-velocity-servo --native-stand \
  --native-scene target/research/contact-baseline/scene.rne.scene.toml \
  --native-dt-us 500 --native-solver-iterations 16 \
  --native-candidate docs/evidence/g1-contact-backflip/native-transfer/compound-soles/candidate.json \
  --native-output target/research/stand-contact-16.json
```

For `stand-64`, change only solver iterations to 64 and the output filename.
For `stand-loss-16`, use the passive scene and 16 iterations. For either flip,
use the passive scene, remove `--native-stand`, add `--native-stop-on-fall`,
and select step 500 or 125 with distinct output filenames.

All previous baseline fields except the differently spelled scene path,
including all 600 recorded physical frames, exactly match
[`../compound-soles/stand.json`](../compound-soles/stand.json).
Five Rust example tests, nine Python tests, targeted release Clippy with
warnings denied, Ruff, and workspace formatting passed. Full workspace/xtask
and SemVer builds were not rerun to retain the 30 GiB reserve. `sources.json`
pins the code, binary, candidate and artifacts; `passive-model-audit.json`
records model differences and generated URDF hash.
