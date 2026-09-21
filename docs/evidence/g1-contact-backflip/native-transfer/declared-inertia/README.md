# Native inertia and joint-frame follow-up

The legacy preparation failures are retained in the parent directory. These
new runs use the separately authored `unitree_g1_backflip_probe` scene and the
corrected generalized-effort axis. No external base force, pose or velocity
command is applied during simulation.

All runs use a 38.13385728 kg model: declared URDF inertial properties plus the
importer's 1 kg fallback for four links with no declared mass. Fixed children
are welded and joint-origin rotations are enabled. Mesh collisions and
self-collision are disabled, so these are diagnostic results, **not qualified
physical backflips**. `failure: null` only means numerical execution completed;
it does not mean landing or standing passed.

Use example 114 with `--native-probe --native-declared --native-dt-us STEP`.
Add `--native-stand` for standing-only cases and `--native-implicit` for native
force-based position motors. Without the implicit flag, direct PD effort uses
the motor-speed taper. Preparation/standing ankle feedback is sampled every
2 ms, with one additional command-period delay. The implicit diagnostic has
no speed taper. Standing runs simulate one preparation second plus five
evaluation seconds. See `docs/G1_CONTACT_BACKFLIP.md` for the standing gate.

| Recording | Standing gate passed | Final-second minimum upright | Final-second maximum base speed (m/s) | Execution |
|---|---|---|---|---|
| [g1-native-probe-125us-declared-flip-effort.json](g1-native-probe-125us-declared-flip-effort.json) | False | -0.0340204686151111 | 0.0014756711416462646 | completed; see posture/qualification |
| [g1-native-probe-125us-declared-stand-effort.json](g1-native-probe-125us-declared-stand-effort.json) | False | 0.9991376261404101 | 0.46416810437633704 | completed; see posture/qualification |
| [g1-native-probe-500us-declared-flip-effort.json](g1-native-probe-500us-declared-flip-effort.json) | False | None | None | Joint state diverged at maneuver time 1.077500 s |
| [g1-native-probe-500us-declared-flip-implicit.json](g1-native-probe-500us-declared-flip-implicit.json) | False | -0.04664577399947234 | 0.010760787174863912 | completed; see posture/qualification |
| [g1-native-probe-500us-declared-stand-effort.json](g1-native-probe-500us-declared-stand-effort.json) | False | 0.999560260687209 | 0.45442759074247274 | completed; see posture/qualification |
| [g1-native-probe-500us-declared-stand-implicit.json](g1-native-probe-500us-declared-stand-implicit.json) | True | 0.9999951505813768 | 0.05280047376627155 | completed; see posture/qualification |

Only the 0.5 ms implicit standing run passes the standing gate. Direct-effort
runs remain oscillatory at both 0.5 ms and 0.125 ms. The maneuver can rotate
through a backward revolution, but does not land upright: all backflip
qualification flags remain false. Next use the stable native motor baseline
for landing optimization and reconcile collision geometry/missing masses.

Validation: all 25 Rapier backend tests pass, including a rotated-origin
revolute/prismatic effort regression that failed before the fix; targeted
release Clippy and the recorded-state coordinate test pass. All 500 external
recording frames still pass headless projection. Workspace formatting passes.
Full workspace test/xtask builds were not repeated to preserve the 30 GiB disk
reserve. Source fingerprints are in `sources.json`.
