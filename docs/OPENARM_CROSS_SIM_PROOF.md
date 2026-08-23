# OpenArm Rapier / Gazebo proof

RNE compiles one content-addressed OpenArm controller into a 1,400-step action
trace and supplies those exact bytes to both RNE/Rapier and Gazebo Harmonic
8.15. The retained proof covers deterministic success replay and an intentional
9-to-8-element controller-output truncation.

- [Browser inspector](evidence/openarm-cross-sim/evidence/replay-inspector.html)
- [Cross-simulator report](evidence/openarm-cross-sim/evidence/cross-sim-report.json)
- [Failure Capsule manifest](evidence/openarm-cross-sim/capsule.json)
- [Controller failure replay](evidence/openarm-cross-sim/replay/controller-failure.rne-replay)

The successful traces both contain 1,400 fixed steps. Rapier's final maximum
tracking error is approximately 0.000003 rad, Gazebo's is approximately
0.001954 rad, and their final maximum joint-position delta is approximately
0.001951 rad. All pass the named 0.01 rad tolerances. Transient divergence is
retained as a non-gating diagnostic for the control-dynamics track.

The intentional truncation first violates the action-width contract at step
307 (5,116,666,769 ns) on both paths. Gazebo rejects the malformed action before
state advance; accepting the corrected action afterward reproduces the clean
step-307 observation exactly.

Verify all 12 content-addressed artifacts without Gazebo:

```bash
cargo run --locked -p xtask -- failure-capsule verify \
  docs/evidence/openarm-cross-sim
```

Rebuilding the full proof requires Ubuntu 22.04, Gazebo Harmonic 8.15, and the
commands in
[`adapters/simulator/rne_gazebo_harmonic/README.md`](../adapters/simulator/rne_gazebo_harmonic/README.md).

This repository-authored adapter is real external-simulator execution evidence,
but it does not satisfy the separate independent third-party-adapter gate.
