# Gazebo Harmonic OpenArm adapter

This process adapter executes the official OpenArm v2 right-arm model in
Gazebo Harmonic and translates the RNE external-simulator JSONL contract into
fixed-step Gazebo commands. It is an adapter, not an RNE physics backend: no
Gazebo, ROS 2, DDS, or simulator-specific type enters a core crate.

The example contract exposes nine joint positions and velocities as its
observation and accepts nine joint-position targets as its action. A bounded
proportional servo becomes Gazebo joint-velocity commands during PreUpdate;
positions and velocities are sampled during PostUpdate. Every accepted action
advances exactly one 16,666,667 ns contract step.

## Requirements

- Ubuntu 22.04 amd64
- Gazebo Harmonic (`gz-sim8`)
- `python3-gz-sim8`
- a release build containing `rne-simulator-conformance`

The checked runtime manifest records the exact locally validated Gazebo
version. Regenerate it when deliberately qualifying another patch version.

## Run conformance

From the repository root on Ubuntu:

```bash
cargo build --locked -p rne_hardware_gateway --bin rne-simulator-conformance
target/debug/rne-simulator-conformance \
  --adapter /usr/bin/python3 \
  --subject adapters/simulator/rne_gazebo_harmonic/rne_gazebo_harmonic_adapter.py \
  --runtime-manifest adapters/simulator/rne_gazebo_harmonic/runtime.json \
  --task adapters/simulator/rne_gazebo_harmonic/openarm_right_joint_tracking.task.json \
  --timeout-ms 15000 \
  --output artifacts/gazebo-openarm-conformance/report.json \
  --adapter-arg adapters/simulator/rne_gazebo_harmonic/rne_gazebo_harmonic_adapter.py \
  --adapter-arg --runtime-manifest \
  --adapter-arg adapters/simulator/rne_gazebo_harmonic/runtime.json \
  --adapter-arg --task \
  --adapter-arg adapters/simulator/rne_gazebo_harmonic/openarm_right_joint_tracking.task.json
```

Gazebo Harmonic's DART backend currently reports that it cannot create the
URDF gripper's mimic constraint. The adapter therefore commands and observes
both finger joints explicitly. The upstream bimanual URDF also uses negative
mesh scales for its mirrored left arm, which DART rejects; this first fixture
intentionally qualifies the official positive-scale right-arm URDF only.

This in-repository adapter proves real external-simulator execution but does
not count as independent third-party adapter evidence.

## Run the same OpenArm controller on Rapier, MuJoCo, and Gazebo

The portable pose-cycle controller owns one content-addressed reference
trajectory, typed joint-feedback timing contract, and bounded joint-space PID
correction law with per-joint integral anti-windup limits. RNE/Rapier, native
MuJoCo 3.9.0, and Gazebo execute that same controller artifact against their own
observations; no backend owns a private trajectory or gain set. The generated
`controller-actions.json` is therefore the shared reference input, while each
backend trace retains the observation sequence, age, proportional/derivative
plus integral correction, and emitted target for every decision.

The Rapier command also writes `sensor-validation-report.json` and a
self-contained `sensor-validation-report.html`, plus the complete
`sensor-dropout-trace.json` and `sensor-stuck-trace.json` golden streams. That
gate reruns the real OpenArm plant for nominal replay, sequence-307 dropout, and
sequence-307 stuck-value cases; it verifies exact one-period DataBus latency,
zero sampling phase error, backend calibration, explicit unavailable effort
measurements, observable actuator saturation, two declared bootstrap frames,
and 1,798 sensor-feedback decisions. The same evidence proves fail-closed
controller behavior at action step 309: sequence-307 dropout exceeds the
one-period age contract, while sequence-307 stuck feedback violates the
required nominal status. A failed check is written to both reports before the
command exits non-zero.

```bash
cargo run --locked -p showcase_captures --bin rne-openarm-rapier-trace -- \
  --output artifacts/openarm-cross-sim
MUJOCO_DYNAMIC_LINK_DIR=/path/to/mujoco-3.9.0/lib \
LD_LIBRARY_PATH=/path/to/mujoco-3.9.0/lib \
cargo run --locked -p showcase_captures --features mujoco \
  --bin rne-openarm-mujoco-trace -- \
  --actions artifacts/openarm-cross-sim/controller-actions.json \
  --output artifacts/openarm-cross-sim
python3 adapters/simulator/rne_gazebo_harmonic/run_openarm_trace.py \
  --actions artifacts/openarm-cross-sim/controller-actions.json \
  --output artifacts/openarm-cross-sim
python3 adapters/simulator/rne_gazebo_harmonic/build_openarm_cross_sim_report.py \
  --output artifacts/openarm-cross-sim
python3 adapters/simulator/rne_gazebo_harmonic/build_openarm_control_dynamics_report.py \
  --trace-root artifacts/openarm-cross-sim \
  --output artifacts/openarm-cross-sim
```

The successful comparison independently recomputes all 1,800 controller
decisions from the artifact and each backend's retained typed observation. It
requires zero timing mismatches and at most `1e-12 rad` reproduction error,
then gates all three final reference-tracking errors and the maximum pairwise
final joint delta with named radian tolerances. Maximum transient divergence is
retained as a non-gating dynamics diagnostic. The intentional controller fault
truncates the nine-element action at step 307; all three paths identify that
exact first violation, and native MuJoCo plus Gazebo prove that rejection did
not advance state before accepting the corrected action.

Rapier and native MuJoCo also use the portable
`rne_physics_state_v2_fnv1a_1e-6_si` replay digest. It covers articulated joint
coordinates and velocities as well as non-fixed rigid-body pose and velocity;
the report fails if the moving arm produces a constant digest or if the exact
replay final digest changes. Solver-private digests are never compared across
backends.

The control-dynamics report evaluates the complete trajectory rather than only
the final pose. It binds the backend-neutral force-based actuation
configuration, native MuJoCo source/runtime evidence, and Gazebo
runtime/configuration hashes, then records per-joint RMSE, IAE, ISE, terminal
bias, position range, peak velocity, all three pairwise backend deltas, and the
first URDF position/velocity-limit violation. MuJoCo compiles the declared
velocity damping as native implicit joint damping, while the backend adds it
back when forming the bounded effort so the resulting total effort remains the
same typed actuator law. `needs_tuning` is a valid diagnostic result and must
not be converted to `passed` by widening a tolerance.

## Identify the OpenArm joint-5 coupled response

The identification controller uses the same TaskSpec, robot model, actuator
configuration, and trace runners. It first excites joint 5 in isolation, then
holds its target while moving the rest of the arm. Generate both traces and the
self-contained report with:

```bash
cargo run --locked -p showcase_captures --bin rne-openarm-rapier-trace -- \
  --controller adapters/simulator/rne_gazebo_harmonic/openarm_joint5_identification.controller.json \
  --output artifacts/openarm-joint5-identification
python3 adapters/simulator/rne_gazebo_harmonic/run_openarm_trace.py \
  --controller adapters/simulator/rne_gazebo_harmonic/openarm_joint5_identification.controller.json \
  --actions artifacts/openarm-joint5-identification/controller-actions.json \
  --output artifacts/openarm-joint5-identification
python3 adapters/simulator/rne_gazebo_harmonic/build_openarm_joint5_identification_report.py \
  --trace-root artifacts/openarm-joint5-identification \
  --output artifacts/openarm-joint5-identification
```

The report fits a SISO ARX(2,2) model only on the isolated window, validates it
on the coupled window, and records the first URDF position-limit violation. Its
corrected status is `coupled_response_passed`: the Rapier path consumes the
URDF-declared mass, centre of mass, and complete inertia tensor, while the robot
asset configuration, actuation configuration, and model remain independently
content-addressed. The coupled and hard-limit checks turned green without
changing their registered tolerances or URDF effort limits.

## Run the OpenArm plant and control-engineering lab

Compile the versioned experiment manifest once, then supply the exact controller
and generated action trace to all three backends:

```bash
python3 adapters/simulator/rne_gazebo_harmonic/build_openarm_plant_controller.py \
  --output artifacts/openarm-plant-lab/controller.json
cargo run --locked -p showcase_captures --bin rne-openarm-rapier-trace -- \
  --controller artifacts/openarm-plant-lab/controller.json \
  --output artifacts/openarm-plant-lab
MUJOCO_DYNAMIC_LINK_DIR=/path/to/mujoco-3.9.0/lib \
LD_LIBRARY_PATH=/path/to/mujoco-3.9.0/lib \
cargo run --locked -p showcase_captures --features mujoco \
  --bin rne-openarm-mujoco-trace -- \
  --controller artifacts/openarm-plant-lab/controller.json \
  --actions artifacts/openarm-plant-lab/controller-actions.json \
  --output artifacts/openarm-plant-lab
python3 adapters/simulator/rne_gazebo_harmonic/run_openarm_trace.py \
  --controller artifacts/openarm-plant-lab/controller.json \
  --actions artifacts/openarm-plant-lab/controller-actions.json \
  --output artifacts/openarm-plant-lab
python3 adapters/simulator/rne_gazebo_harmonic/build_openarm_plant_report.py \
  --trace-root artifacts/openarm-plant-lab \
  --controller artifacts/openarm-plant-lab/controller.json \
  --output artifacts/openarm-plant-lab
cargo run --locked -p showcase_captures \
  --bin rne-openarm-plant-failure-replay -- \
  --report artifacts/openarm-plant-lab/openarm-plant-lab-report.json \
  --trace artifacts/openarm-plant-lab/rapier-success-trace.json \
  --output artifacts/openarm-plant-lab/plant-settling-failure.rne-replay
```

The report independently recompiles the manifest and rejects controller/action
drift. It checks time response, saturation, empirical frequency response,
frequency-separated coupling, disjoint ARX training/validation windows, exact
same-runtime replay, URDF limits, and cross-backend differences against fixed
SI-unit requirements. `needs_tuning` is a valid diagnostic: the retained
baseline localizes Rapier's joint-5 settling-time failure at step 571 instead
of widening the 3.5 s requirement or the +/-0.0024 rad settling band.

The committed 34-artifact proof, including complete traces and the derived
failure replay, verifies without loading any simulator:

```bash
cargo run --locked -p xtask -- failure-capsule verify \
  docs/evidence/openarm-plant-lab
```

## Compare PID and state-space control

The controller lab consumes the retained Rapier ARX model without refitting,
forms the discrete three-state plant plus an integrated tracking-error state,
checks controllability, and places four declared stable poles. A one-sample ARX
predictor compensates the exact typed-observation latency. PID and state-space
artifacts control only right joint 5 and share the same reference, sample time,
latency, +/-0.04 rad feedback-correction bound, +/-0.015 rad integral-correction
bound, target limits, actuator configuration, intentional failure, and a
declared one-second `+0.03 rad` actuator-target bias pulse. The pulse is applied
after controller limits and is invisible to the controller except through the
same delayed typed joint feedback.

```bash
python3 adapters/simulator/rne_gazebo_harmonic/build_openarm_controller_suite.py \
  --output artifacts/openarm-controller-lab

# Run this command once for each generated controller, changing ROLE and CONTROLLER.
cargo run --locked -p showcase_captures --bin rne-openarm-rapier-trace -- \
  --controller artifacts/openarm-controller-lab/CONTROLLER \
  --output artifacts/openarm-controller-lab/ROLE
# Then run the MuJoCo and Gazebo commands from the plant lab with the same
# controller and ROLE/controller-actions.json paths.

python3 adapters/simulator/rne_gazebo_harmonic/build_openarm_controller_report.py \
  --suite-root artifacts/openarm-controller-lab \
  --output artifacts/openarm-controller-lab/report
cargo run --locked -p showcase_captures \
  --bin rne-openarm-controller-failure-replay -- \
  --report artifacts/openarm-controller-lab/report/openarm-controller-comparison-report.json \
  --trace artifacts/openarm-controller-lab/pid/rapier-success-trace.json \
  --output artifacts/openarm-controller-lab/report/pid-settling-failure.rne-replay
```

The report independently reproduces all 21,600 controller decisions across two
controllers and three backends. The fixed PID baseline settles in approximately
4.983 s on Rapier, 3.017 s on MuJoCo, and 1.283 s on Gazebo; only the Rapier
baseline misses the unchanged 3.5 s requirement. State-space control settles in
approximately 0.567 s, 0.550 s, and 0.467 s respectively. Its largest
cross-backend settling delta is approximately 0.10 s, and every declared pole
lies inside the unit circle. The PID replay distinguishes the 3.5 s deadline at
step 571 from the first subsequent band exit at step 577.

Under the shared actuator-realization disturbance, state-space control limits
joint-5 peak error to approximately `0.0111-0.0125 rad`, recovers into the fixed
`0.005 rad` band in `0.20-0.233 s`, and records `0.00448-0.00576 rad*s` IAE
across Rapier, MuJoCo, and Gazebo. Every trace separately records reference,
controller output, injected offset, and actual backend target; the report
reproduces all four values rather than treating the pulse as a reference move.

The committed 43-artifact controller proof verifies without loading Rapier,
MuJoCo, or Gazebo:

```bash
cargo run --locked -p xtask -- failure-capsule verify \
  docs/evidence/openarm-controller-lab
```

## Sweep the OpenArm actuator-bias robustness boundary

The first robustness dimension holds the TaskSpec, state-feedback design,
typed-observation latency, actuator limits, reference, and fixed requirements
constant while sweeping an unobserved joint-5 actuator-target bias over
`[0.00, 0.03, 0.06, 0.09, 0.12] rad`. Rapier executes the complete grid; the
last passing and first failing cases are then run unchanged on MuJoCo and
Gazebo:

```bash
python3 adapters/simulator/rne_gazebo_harmonic/build_openarm_robustness_suite.py \
  --output artifacts/openarm-robustness-lab

# Run every generated controller on Rapier. Then run the 0.03 rad and 0.06 rad
# cases on MuJoCo and Gazebo using the same controller-actions.json artifact.
python3 adapters/simulator/rne_gazebo_harmonic/build_openarm_robustness_report.py \
  --suite-root artifacts/openarm-robustness-lab \
  --output artifacts/openarm-robustness-lab/report
cargo run --locked -p showcase_captures \
  --bin rne-openarm-robustness-failure-replay -- \
  --report artifacts/openarm-robustness-lab/report/openarm-robustness-report.json \
  --trace artifacts/openarm-robustness-lab/bias-060mrad/rne_rapier/rapier-success-trace.json \
  --output artifacts/openarm-robustness-lab/report/minimum-bias-failure.rne-replay
```

The fixed grid brackets the boundary at `0.03 rad` passing and `0.06 rad`
failing. At `0.06 rad`, all three backends first fail the unchanged
`0.02 rad*s` IAE requirement while still passing peak-error and recovery-time
requirements. Rapier localizes the first cumulative IAE crossing to step 3292
at `0.020305 rad*s`; the replay ends at that first violation.

The retained 49-artifact capsule includes all five Rapier sweep traces, both
boundary cases on MuJoCo and Gazebo, the exact controllers/actions, report,
model/configuration inputs, and the 3,293-frame minimum-failure replay:

```bash
cargo run --locked -p xtask -- failure-capsule verify \
  docs/evidence/openarm-robustness-lab
```

The same manifest also defines a controller-visible joint-position bias sweep:

```bash
python3 adapters/simulator/rne_gazebo_harmonic/build_openarm_robustness_suite.py \
  --dimension joint_position_measurement_bias \
  --output artifacts/openarm-sensor-robustness-lab
```

This path preserves the raw typed feedback, then separately records the
nominal-status bias and the exact delayed position consumed by the controller.
The fixed `[0.00, 0.01, 0.02, 0.04, 0.06] rad` grid brackets the boundary at
`0.01 rad` passing and `0.02 rad` failing. All three backends first fail the
same `0.02 rad*s` IAE requirement; Rapier crosses at step 3303 and MuJoCo plus
Gazebo at step 3304. The bias is active for exactly 60 controller decisions on
every run, with at most `3.47e-18 rad` realization error.

The retained 43-artifact capsule is bound to producer commit `a730cce` and
contains both portable boundary cases, all five controller variants, the exact
raw/visible observation report, and the 3,304-frame first-failure replay.

```bash
cargo run --locked -p xtask -- failure-capsule verify \
  docs/evidence/openarm-sensor-robustness-lab
```

The third dimension sweeps controller-ingress publication dropout while keeping
the backend measurement intact:

```bash
python3 adapters/simulator/rne_gazebo_harmonic/build_openarm_robustness_suite.py \
  --dimension joint_feedback_publication_dropout \
  --output artifacts/openarm-sensor-dropout-robustness-lab
```

The observation contract permits a maximum age of three control periods. It
therefore passes bursts of zero, one, and two dropped frames. Three frames is
the first failure: Rapier, MuJoCo, and Gazebo all observe the same
`66,666,668`-tick age, reject exactly one decision at step 3244, hold the last
accepted target with zero realization delta while freezing controller state,
and resume on fresh nominal sequence 3243 at step 3245. The first contract
deviation remains the third missing publication at capture sequence 3242, so
the minimum replay stops there rather than hiding it behind the later rejection.

The browser report and retained capsule preserve all five Rapier cases and the
two/three-frame boundary on all backends:

```bash
cargo run --locked -p xtask -- failure-capsule verify \
  docs/evidence/openarm-sensor-dropout-robustness-lab
```
