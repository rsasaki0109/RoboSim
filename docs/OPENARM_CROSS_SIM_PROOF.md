# OpenArm Rapier / Gazebo proof

RNE compiles one content-addressed OpenArm controller into a 1,800-step action
trace and supplies those exact bytes to both RNE/Rapier and Gazebo Harmonic
8.15. The retained proof covers deterministic success replay and an intentional
9-to-8-element controller-output truncation.

- [Browser inspector](evidence/openarm-cross-sim/evidence/replay-inspector.html)
- [Cross-simulator report](evidence/openarm-cross-sim/evidence/cross-sim-report.json)
- [Control-dynamics report](evidence/openarm-cross-sim/evidence/control-dynamics-report.html)
- [Failure Capsule manifest](evidence/openarm-cross-sim/capsule.json)
- [Controller failure replay](evidence/openarm-cross-sim/replay/controller-failure.rne-replay)

The successful traces both contain 1,800 fixed steps. Rapier's final maximum
tracking error is approximately 0.001472 rad, Gazebo's is approximately
0.001954 rad, and their final maximum joint-position delta is approximately
0.000482 rad. All pass the named 0.01 rad final-pose tolerances.

That final-pose pass is deliberately not the whole dynamics verdict. The
browser-readable control report evaluates every fixed step with RMSE, IAE, ISE,
terminal bias, peak velocity, and measured position range. It binds the explicit
RNE force-based servo configuration, Gazebo runtime/configuration, URDF limits,
TaskSpec, controller, action trace, and both backend traces. The current honest
result is `needs_tuning`: Rapier joint 5 has approximately 0.259 rad tracking
RMSE against the registered 0.10 rad bound, while Gazebo passes and neither
backend crosses a URDF position or velocity hard limit. This retained weakness
is the next controller/plant-identification target; the tolerance is not widened.

The follow-up identification controller first excites joint 5 alone and then
moves the remaining arm joints while holding the joint-5 target at -0.3 rad. In
the measured baseline, Rapier passes isolated tracking at approximately 0.0156
rad RMSE but reaches approximately 1.280 rad RMSE in the coupled window and
first crosses the 1.5708 rad URDF upper limit at step 1642. Gazebo passes both
windows. The generated report fits an ARX(2,2) model on the isolated window and
retains its coupled-window residual, localizing the next fix to articulation
coupling/constraint enforcement rather than the portable reference trajectory.

The intentional truncation first violates the action-width contract at step
307 (5,116,666,769 ns) on both paths. Gazebo rejects the malformed action before
state advance; accepting the corrected action afterward reproduces the clean
step-307 observation exactly.

Verify all content-addressed artifacts without Gazebo:

```bash
cargo run --locked -p xtask -- failure-capsule verify \
  docs/evidence/openarm-cross-sim
```

Rebuilding the full proof requires Ubuntu 22.04, Gazebo Harmonic 8.15, and the
commands in
[`adapters/simulator/rne_gazebo_harmonic/README.md`](../adapters/simulator/rne_gazebo_harmonic/README.md).

This repository-authored adapter is real external-simulator execution evidence,
but it does not satisfy the separate independent third-party-adapter gate.
