# OpenArm Rapier / native MuJoCo / Gazebo proof

RNE compiles one content-addressed OpenArm controller into a 1,800-step action
trace and supplies those exact bytes to RNE/Rapier, native MuJoCo 3.9.0, and
Gazebo Harmonic 8.15. Each backend executes the same typed-feedback PID artifact
against its own delayed joint observations. The retained proof covers
deterministic success replay and an intentional 9-to-8-element
controller-output truncation.

- [Browser inspector](evidence/openarm-cross-sim/evidence/replay-inspector.html)
- [Cross-simulator report](evidence/openarm-cross-sim/evidence/cross-sim-report.json)
- [Control-dynamics report](evidence/openarm-cross-sim/evidence/control-dynamics-report.html)
- [Joint-feedback sensor report](evidence/openarm-cross-sim/evidence/sensor-validation-report.html)
- [Failure Capsule manifest](evidence/openarm-cross-sim/capsule.json)
- [Controller failure replay](evidence/openarm-cross-sim/replay/controller-failure.rne-replay)
- [OpenArm plant lab](evidence/openarm-plant-lab/evidence/openarm-plant-lab-report.html)
- [Plant settling Failure Capsule](evidence/openarm-plant-lab/capsule.json)
- [PID/state-space comparison](evidence/openarm-controller-lab/evidence/openarm-controller-comparison-report.html)
- [PID settling Failure Capsule](evidence/openarm-controller-lab/capsule.json)

The successful traces each contain 1,800 fixed steps. The final maximum tracking
errors are approximately 0.004235 rad for Rapier, 0.004991 rad for native
MuJoCo, and 0.003525 rad for Gazebo. The maximum pairwise final joint-position
delta is approximately 0.006015 rad. All pass the unchanged named 0.01 rad
final-pose tolerances.

That final-pose pass is deliberately not the whole dynamics verdict. The
browser-readable control report evaluates every fixed step with RMSE, IAE, ISE,
terminal bias, peak velocity, and measured position range. It binds the explicit
backend-neutral force-based servo configuration, native MuJoCo source/runtime
evidence, Gazebo runtime/configuration, URDF limits, TaskSpec, controller,
action trace, and all three backend traces. The result is `passed`: joint 5 has
approximately 0.0142 rad tracking RMSE on Rapier, 0.0107 rad on native MuJoCo,
and 0.0078 rad on Gazebo against the registered 0.10 rad bound. No backend
crosses a URDF position or velocity hard limit. The correction
enables the URDF-declared mass, centre of mass, and complete inertia tensor
instead of substituting one-kilogram link defaults. It also keeps reset
references 0.01 rad inside one-sided mechanical stops. The robot asset
configuration hash is bound independently from the URDF and actuation hashes;
no effort ceiling or tolerance was widened.

The PID controller consumes only typed `JointFeedback` frames available through
the DataBus after the declared one-period latency. Each runner records 2
bootstrap frames and 1,798 feedback decisions. The report independently
recomputes the reference, integral state, bounded correction, and final target;
timing mismatches are zero and the largest numerical round-trip/reproduction
delta is below 8.9e-16 rad. Per-joint integrator clamps are also checked as an
anti-windup contract. MuJoCo compiles the configured velocity damping as native
implicit joint damping while preserving the same bounded total-effort equation,
avoiding unstable explicit damping without changing the controller artifact.
Rapier and MuJoCo each produce 1,800 distinct portable articulated-state
digests under the versioned `rne_physics_state_v2_fnv1a_1e-6_si` contract, and
their exact replay final digests match. Those same-runtime digests are not
claimed to match between different solvers.

The follow-up identification controller first excites joint 5 alone and then
moves the remaining arm joints while holding the joint-5 target at -0.3 rad. In
the historical baseline, Rapier passed isolated tracking at approximately
0.0156 rad RMSE but reached approximately 1.280 rad RMSE in the coupled window
and first crossed the 1.5708 rad URDF upper limit at step 1642. With exact URDF
inertial properties, Rapier now measures approximately 0.016 rad isolated RMSE
and 0.014 rad coupled RMSE with no limit crossing; Gazebo passes both windows.
The generated report fits an ARX(2,2) model on the isolated window and evaluates
the coupled-window residual without refitting, so the corrected plant result is
not inferred from final pose alone.

The intentional truncation first violates the action-width contract at step
307 (5,116,666,769 ns) on all three paths. Native MuJoCo and Gazebo reject the
malformed action before state advance; accepting the corrected action afterward
reproduces the clean step-307 observation exactly.

Verify all content-addressed artifacts without running MuJoCo or Gazebo:

```bash
cargo run --locked -p xtask -- failure-capsule verify \
  docs/evidence/openarm-cross-sim
```

Rebuilding the full proof requires Ubuntu 22.04, MuJoCo 3.9.0, Gazebo Harmonic
8.15, and the commands in
[`adapters/simulator/rne_gazebo_harmonic/README.md`](../adapters/simulator/rne_gazebo_harmonic/README.md).

This repository-authored adapter is real external-simulator execution evidence,
but it does not satisfy the separate independent third-party-adapter gate.

## OpenArm control-engineering plant lab

The retained plant lab replaces final-pose-only judgment with one compiled,
3,600-step open-loop experiment on Rapier 0.22, native MuJoCo 3.9.0, and Gazebo
Harmonic 8.15. A versioned manifest conditions the nine-joint arm to a safe
operating point, isolates right joint 5 with a step doublet and triangular ramp,
then supplies a bounded linear chirp, an independent multisine validation
window, and frequency-separated multi-axis excitation. All runners consume the
same controller and action bytes.

The [self-contained HTML report](evidence/openarm-plant-lab/evidence/openarm-plant-lab-report.html)
contains complete time response, empirical frequency response, cross-axis
coupling matrices, saturation evidence, ARX(2,2) training/validation results,
hard-limit checks, and the fixed requirements registry. Its JSON report SHA-256
is `7cbe59fa79e5e5971ef79730bb3a9741bfcc72bc37a44e3581853c944f7711d4`,
and an exact rerun reproduces the same bytes.

The result is intentionally `needs_tuning`, not tolerance-inflated to green.
MuJoCo and Gazebo settle in approximately 0.583 s and 0.450 s. Rapier rises in
approximately 0.367 s but remains 0.007633 rad from the 0.12 rad target at the
3.5 s deadline, outside the fixed +/-0.0024 rad band. The first failed
`closed_loop_performance` requirement is therefore localized at step 571.
There is no URDF position/velocity violation; ramp RMSE is approximately
0.01050, 0.00850, and 0.00624 rad respectively, and the maximum cross-backend
joint-5 tracking-RMSE delta is 0.00517 rad against the fixed 0.04 rad limit.

The plant model is fitted only on the chirp window and evaluated without
refitting on the independent multisine window. Validation RMSE is approximately
0.000337 rad for Rapier, 0.00000942 rad for MuJoCo, and 0.0000108 rad for
Gazebo. The Failure Capsule retains the experiment/compiler, requirements,
controller/actions, model/configuration, three complete traces, datasets,
report source, HTML/JSON, and a 572-frame behavior replay of the Rapier settling
failure. Verify its 34 content-addressed artifacts with:

```bash
cargo run --locked -p xtask -- failure-capsule verify \
  docs/evidence/openarm-plant-lab
```

## PID versus state-space controller proof

The controller successor uses the retained Rapier ARX(2,2) model as its nominal
plant without refitting. Its compiler forms a discrete three-state realization,
adds integrated tracking error, proves the augmented controllability determinant
is nonzero (`1.3369e-5`), and places the declared poles at 0.50, 0.65, 0.75, and
0.85. The two unknown output-history states are observable; the third plant
state is previous input supplied by controller-owned history rather than
pretended to be inferred from the sensor. Because controller-visible joint
feedback is one sample old, the runtime
first predicts the current position with the retained ARX coefficients; it does
not treat delayed measurement as current state.

PID and state-space controllers use identical 3,600-step references, joint-5-only
authority, one-period latency, target bounds, actuator model, +/-0.04 rad total
feedback correction, and +/-0.015 rad integral correction. The state-space
controller reduces Rapier settling from approximately 4.983 s to 0.567 s while
MuJoCo moves from 3.017 s to 0.550 s and Gazebo from 1.283 s to 0.467 s. All
state-space runs pass the fixed 3.5 s, 25% overshoot, and 0.08 rad ramp-RMSE
requirements. The maximum decision reproduction delta across all 21,600
decisions is below `9.8e-17 rad`.

The same six runs now include a declared actuator-realization disturbance:
steps 3241 through 3300 add `+0.03 rad` to joint 5 after controller limits. It
is not folded into the reference or exposed directly to the controller. The
state-space controller holds peak tracking error to `0.0111-0.0125 rad`,
recovers into the fixed `0.005 rad` band in `0.20-0.233 s`, and records
`0.00448-0.00576 rad*s` IAE across Rapier, MuJoCo, and Gazebo. Each retained
frame separates reference, controller target, disturbance, and applied plant
target, and the report finds no disturbance-realization mismatch.

The PID baseline is retained as a real, non-gating failure. It is briefly inside
the +/-0.0024 rad band at the step-571 deadline, then first exits at step 577;
the dedicated behavior replay therefore ends at step 577 rather than claiming
the deadline itself was the first deviation.

The retained [comparison report](evidence/openarm-controller-lab/evidence/openarm-controller-comparison-report.html)
and [Failure Capsule](evidence/openarm-controller-lab/capsule.json) contain 43
content-addressed artifacts, including both controllers, both action traces, all
six successful backend traces, all six intentional action-width failures, the
model/requirements/configuration inputs, and the 578-frame PID replay. Verify
them without loading a simulator. The capsule is bound to producer commit
`6ed02b94595144b9497350a170cc18256eee1531`:

```bash
cargo run --locked -p xtask -- failure-capsule verify \
  docs/evidence/openarm-controller-lab
```

## State-feedback robustness boundary

The actuator-bias sweep changes one declared plant-input parameter while
holding the TaskSpec, controller gains, reference, latency, actuator limits,
and requirements fixed. Rapier runs the complete
`[0.00, 0.03, 0.06, 0.09, 0.12] rad` grid. The boundary predecessor and first
failure are then replayed on native MuJoCo and Gazebo.

All three backends pass at `0.03 rad` and first fail the same
`controller.state.maximum_disturbance_iae_rad_s` requirement at `0.06 rad`.
Peak tracking error and recovery time remain within their limits, preventing a
terminal-pose or single-peak metric from hiding the accumulated tracking loss.
Rapier first crosses the fixed `0.02 rad*s` IAE limit at step 3292 with
`0.020305 rad*s`; the dedicated replay stops at that measurement.

The retained [robustness report](evidence/openarm-robustness-lab/evidence/openarm-robustness-report.html)
and [minimum-failure Capsule](evidence/openarm-robustness-lab/capsule.json) can
be verified without loading a simulator. Its 49 artifacts are bound to
producer commit `4860baca1cffb91c5872f6aecfaaa73dcd2fa55a` and contain the
complete Rapier grid plus both portable boundary cases:

```bash
cargo run --locked -p xtask -- failure-capsule verify \
  docs/evidence/openarm-robustness-lab
```
