# Native velocity-servo diagnostics: no successful landing

The committed native controller at `fb5f634` uses bounded velocity commands,
force-based implicit motors, and per-step measurement of actual joint speed.
This directory retains 20 rejected launch/tuck/opening candidates at 0.5 ms.
All use the 38.13385728 kg declared-inertia probe, primitive contacts,
self-collision disabled, 0.5 foot/ground friction and a 120 N m knee ceiling.
No base pose, base velocity, or external base wrench is prescribed.

Every candidate collapsed; none is a native backflip success. Some complete
one backward revolution before collapse. `qualified_backflip` remains false.
Velocity commands are capped at 90% of the URDF rating, but contact impulses
can still push measured speeds beyond the 1.05 screening threshold.

| Candidate | Maximum speed / rating | Peak-speed joint | Peak time (s) |
|---|---:|---|---:|
| velocity-ankle65 | 2.4325 | right_ankle_roll_link | 1.1590 |
| velocity-b10-open45 | 1.3720 | left_ankle_roll_link | 1.0195 |
| velocity-b10-open5 | 2.8410 | right_ankle_roll_link | 0.9985 |
| velocity-b20-open45 | 0.9137 | right_ankle_pitch_link | 0.6095 |
| velocity-b20-open5 | 1.1162 | right_knee_link | 1.0290 |
| velocity-bias10 | 2.8397 | left_ankle_roll_link | 1.0565 |
| velocity-bias20 | 2.7254 | right_ankle_roll_link | 1.0855 |
| velocity-bias27 | 2.6251 | right_ankle_roll_link | 1.1090 |
| velocity-bias45 | 2.6885 | right_ankle_roll_link | 1.1635 |
| velocity-crouch22 | 3.5575 | right_ankle_roll_link | 1.1795 |
| velocity-hip07 | 0.9074 | left_knee_link | 0.6305 |
| velocity-hip10 | 2.7039 | right_ankle_roll_link | 1.1205 |
| velocity-k20-b10 | 4.9230 | right_ankle_roll_link | 1.1525 |
| velocity-k20-b20 | 3.0770 | left_ankle_roll_link | 1.1735 |
| velocity-k22-b10 | 3.3534 | right_ankle_roll_link | 1.1025 |
| velocity-k22-b20 | 3.3579 | right_ankle_roll_link | 1.1490 |
| velocity-tuck15-23 | 1.9672 | left_ankle_pitch_link | 1.1625 |
| velocity-tuck15-26 | 1.5799 | left_ankle_roll_link | 1.1550 |
| velocity-tuck20-23 | 2.0402 | right_ankle_roll_link | 1.1105 |
| velocity-tuck20-26 | 2.1429 | right_ankle_roll_link | 1.1070 |

`rejected-sweep.json` records every candidate's parameters and outcome. Two
complete recordings are retained for inspection; `sources.json` identifies
the exact source commit and hashes for both sources and evidence. The saved
traces end at the early collapse guard, so no final-second standing metrics
are available. The guard is a diagnostic stop, not a success condition.

Reproduce either retained candidate from this repository root (use a fresh
output filename):

```bash
cargo run --release -p g1_backflip_gif --example 114_g1_backflip_gif -- \
  --native-probe --native-declared --native-velocity-servo \
  --native-dt-us 500 --native-stop-on-fall \
  --native-candidate docs/evidence/g1-contact-backflip/native-transfer/velocity-servo/velocity-bias27-candidate.json \
  --native-output target/native-velocity-bias27.json
```

Later source versions add explicit `contact_friction` metadata; use the recorded
source commit for byte-for-byte evidence reproduction. The underlying default
coefficient remains 0.5.

Validation: four example unit tests (frame conversion, COM momentum, bounded
roll feedback, bounded velocity commands), release Clippy with `-D warnings`,
workspace formatting, and malformed-input rejection checks. Full workspace
build/test/xtask commands were not rerun: approximately 32 GiB remained free
and the task retains a 30 GiB disk reserve. These diagnostics do not establish
complete collision qualification or hardware readiness.
