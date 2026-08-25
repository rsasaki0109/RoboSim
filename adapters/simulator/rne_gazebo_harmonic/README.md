# Gazebo Harmonic OpenArm adapter

This process adapter executes the official OpenArm v2 right-arm model in
Gazebo Harmonic and translates the RNE external-simulator JSONL contract into
fixed-step Gazebo commands. It is an adapter, not an RNE physics backend: no
Gazebo, ROS 2, DDS, or simulator-specific type enters a core crate.

The example contract exposes nine joint positions and velocities as its
observation and accepts nine joint-position targets as its action. The default
runtime uses a bounded velocity servo. A runtime may instead declare the tested
effort-PD realization, explicit per-joint effort limits, failure behavior, and
multiple physics substeps per 16,666,667 ns control step. Positions and
velocities are sampled during PostUpdate. Trace collection also writes a
deterministic actuation sidecar for each replay. It records raw and applied
commands, command kind and units, initial/final position error, and saturation
count across every physics substep while leaving the strict JSONL wire v1
payload unchanged.

## Requirements

- Ubuntu 22.04 amd64
- Gazebo Harmonic (`gz-sim8`)
- `python3-gz-sim8`
- a release build containing `rne-simulator-conformance`

The checked runtime manifest records the exact locally validated Gazebo
version. Regenerate it when deliberately qualifying another patch version.

Generate deterministic physical payload fixtures with:

```bash
python adapters/simulator/rne_gazebo_harmonic/build_openarm_payload_suite.py \
  --output artifacts/openarm-payload/fixtures
```

The nonzero cases lump the payload into the hand inertial using the
parallel-axis theorem and retain explicit mass, center-of-mass, inertia, model,
scene, and runtime hashes. Their Gazebo world fixes the robot base and selects
bounded effort-PD actuation over ten physics substeps, so payload mass affects
the measured trajectory. Gain scaling is declared per joint so shoulder and
wrist authority are not conflated. The derivative path declares a deterministic
first-order low-pass time constant and records both measured and filtered
velocity; the URDF effort limits remain unchanged. Invalid actuator declarations
fail before startup.

After collecting the three backend traces, independently recompute the model
and control requirements and write the browser report with:

```bash
python adapters/simulator/rne_gazebo_harmonic/build_openarm_payload_report.py \
  --fixture-root artifacts/openarm-payload/fixtures \
  --trace-root artifacts/openarm-payload \
  --output artifacts/openarm-payload/report
```

Compile the joint-5 actuator-authority envelope from the zero-payload physical
fixture with:

```bash
python adapters/simulator/rne_gazebo_harmonic/build_openarm_authority_suite.py \
  --baseline-fixture artifacts/openarm-payload/fixtures/payload-0000g \
  --output artifacts/openarm-authority/fixtures
```

The suite scales only the controlled joint's `max_effort_nm`, writes matching
native and Gazebo actuation artifacts, and binds every model/config/runtime
hash. After collecting the three backend traces, build the portable report:

```bash
python adapters/simulator/rne_gazebo_harmonic/build_openarm_authority_report.py \
  --fixture-root artifacts/openarm-authority/fixtures \
  --trace-root artifacts/openarm-authority \
  --output artifacts/openarm-authority/report
```

Compile the joint-5 plant viscous-damping envelope independently of actuator
servo damping with:

```bash
python adapters/simulator/rne_gazebo_harmonic/build_openarm_joint_loss_suite.py \
  --baseline-fixture artifacts/openarm-payload/fixtures/payload-0000g \
  --output artifacts/openarm-joint-loss/fixtures
python adapters/simulator/rne_gazebo_harmonic/build_openarm_joint_loss_controller_tuning.py \
  --output artifacts/openarm-joint-loss/controller-tuning
```

The fixed `[0, 2.5, 5, 10, 20] N*m*s/rad` grid holds Coulomb friction at zero
and leaves the TaskSpec, controller, actuator gains/limits, scene, world, and
Gazebo adapter config unchanged. Each fixture binds the requested value, the
independently parsed URDF realization, and every model/config/runtime hash. The
zero case is byte-identical to the source model. Nonzero cases add exactly one
joint-5 `<dynamics>` declaration. Plant damping and the existing joint-5 servo
damping remain separate unit-bearing report fields.
This viscous-only grid still holds Coulomb friction at zero so its existing
boundary evidence remains one-dimensional. The portable Coulomb successor uses
`-magnitude*tanh(velocity/transition_velocity)` rather than backend-native
static-friction settings. Rapier and MuJoCo consume the typed plant component;
the Gazebo effort adapter consumes explicit per-joint
`plant_coulomb_friction_nm` and
`plant_coulomb_transition_velocity_rad_s` arrays, adds passive loss after
actuator saturation, and records passive effort plus the total force sent on
every physics substep. This model is smooth kinetic loss and makes no stiction
or breakaway claim.

The controller tuning manifest freezes the `[0.04, 0.05, 0.06, 0.08] rad`
state-feedback correction-limit candidates before execution. Run only the
declared MuJoCo 10 N*m*s/rad tuning case for each candidate, then select it with:

```bash
python adapters/simulator/rne_gazebo_harmonic/build_openarm_joint_loss_controller_tuning_report.py \
  --candidate-root artifacts/openarm-joint-loss/controller-tuning \
  --trace-root artifacts/openarm-joint-loss/controller-tuning-results \
  --output artifacts/openarm-joint-loss/controller-tuning-report
```

The report selects the passing candidate with minimum RMSE, breaking a tie in
favor of the smaller correction limit, and copies the exact selected controller
with its SHA-256. The remaining damping/backend combinations are validation
data, not tuning data. After running the selected controller unchanged beneath
`rapier/CASE`, `mujoco/CASE`, and `gazebo/CASE`, build the browser report and
minimum aggregate boundary-failure replay:

```bash
python adapters/simulator/rne_gazebo_harmonic/build_openarm_joint_loss_report.py \
  --fixture-root artifacts/openarm-joint-loss/fixtures \
  --trace-root artifacts/openarm-joint-loss \
  --controller artifacts/openarm-joint-loss/controller-tuning-report/openarm-joint-loss-selected.controller.json \
  --output artifacts/openarm-joint-loss/report
cargo run --locked -p showcase_captures \
  --bin rne-openarm-joint-loss-failure-replay -- \
  --report artifacts/openarm-joint-loss/report/openarm-joint-loss-report.json \
  --trace artifacts/openarm-joint-loss/rapier/joint5-damping-20000mnms-per-rad/rapier-success-trace.json \
  --output artifacts/openarm-joint-loss/report/minimum-joint-loss-failure.rne-replay
```

The initially declared 10 N*m*s/rad cross-backend contract and `0.02 rad` RMSE
limit are not relaxed after measurement. The predeclared selection chooses the
bounded `0.08 rad` correction candidate. Its content-addressed controller passes
10 on Rapier, MuJoCo, and Gazebo at `0.013450`, `0.017185`, and `0.009439 rad`
RMSE respectively. At the first out-of-envelope point, 20, all three fail the
same unchanged RMSE contract at `0.021962`, `0.024173`, and `0.021684 rad`.
The final report is `passed`: supported cases pass, model/hash/replay checks
remain exact, and the 20-point rows are retained as
`expected_boundary_failure`, not hidden or converted into successes.

Build the separate regularized-Coulomb envelope with:

```bash
python adapters/simulator/rne_gazebo_harmonic/build_openarm_coulomb_friction_suite.py \
  --baseline-fixture artifacts/openarm-payload/fixtures/payload-0000g \
  --output artifacts/openarm-coulomb/fixtures
python adapters/simulator/rne_gazebo_harmonic/build_openarm_coulomb_friction_report.py \
  --fixture-root artifacts/openarm-coulomb/fixtures \
  --trace-root artifacts/openarm-coulomb \
  --controller artifacts/openarm-joint-loss/controller-tuning-report/openarm-joint-loss-selected.controller.json \
  --output artifacts/openarm-coulomb/report
cargo run --locked -p showcase_captures \
  --bin rne-openarm-joint-loss-failure-replay -- \
  --report artifacts/openarm-coulomb/report/openarm-coulomb-friction-report.json \
  --trace artifacts/openarm-coulomb/rapier/joint5-coulomb-0250mn/rapier-success-trace.json \
  --output artifacts/openarm-coulomb/report/openarm-coulomb-friction-first-failure.rne-replay
```

Use `--case-id joint5-coulomb-0500mn` for a focused three-backend rerun. The
compiled Gazebo fixture copies the portable actuation artifact's exact PD gains,
effort and velocity limits, full effort-controlled joint set, and physics
substep count. Its effort-speed envelope is applied after effort clamping, just
as in the native backends; Gazebo evidence remains adapter diagnostics rather
than a backend effort measurement.

The frozen `[0, 0.25, 0.5, 1, 2] N*m` grid keeps plant viscous damping at
10 N*m*s/rad and the transition velocity at 0.01 rad/s. All 15 real runs have
exact same-runtime replay and exact independently checked parameter
realization. The report deliberately remains `needs_tuning`: the first
supported failure is Rapier at 0.25 N*m, where RMSE is 0.038961 rad against the
unchanged 0.02 rad limit. MuJoCo and Gazebo pass through the declared 0.5 N*m
point. A diagnostic Rapier controller-correction sweep at 0.5 N*m did not
recover the limit, so transition-width/integration sensitivity is the next
predeclared tuning dimension. The supported envelope and tolerance are not
changed after observing this result.

URDF cannot encode that transition width. Portable fixtures therefore bind it
in the hashed robot asset rather than a runner-only argument:

```toml
[[urdf.joint_passive_dynamics]]
joint = "openarm_right_joint5"
coulomb_transition_velocity_rad_s = 0.01
```

The asset loader requires articulation, a unique known joint, exactly one
revolute or prismatic unit field, and a finite positive value. It preserves the
URDF damping and Coulomb magnitude and overrides only the unrepresentable
transition velocity. The predeclared Rapier tuning grid
`[0.01, 0.02, 0.04, 0.05] rad/s` also requires at least 95% of the requested
kinetic loss at 0.1 rad/s. All four candidates retain exact replay and exact
realization but fail the fixed RMSE limit at `0.036139`, `0.041792`, `0.044637`,
and `0.038900 rad`. No transition is selected; the browser report remains
`needs_tuning`, and physics-substep sensitivity is the next predeclared
experiment.

The substep experiment partitions the exact `16,666,667 tick` control period
without drift and freezes `[1, 2, 5, 10]` physics steps before execution. It
also stays red: RMSE is `0.036139`, `0.047853`, `0.324674`, and `0.084528 rad`.
The 5- and 10-substep cases materially worsen the force-based servo response,
so no substep count is selected and the 1-step production behavior remains
unchanged. Transition smoothing and numerical subdivision are therefore ruled
out as fixes for this contract; controller/plant-model retuning at the fixed
0.5 N*m case is next.

The first controller retuning experiment freezes that same plant, TaskSpec,
trajectory, observation/disturbance contracts, correction limits, and
single-substep execution while replacing only the four desired closed-loop
poles. Build its predeclared candidates and browser report with:

```bash
python adapters/simulator/rne_gazebo_harmonic/build_openarm_coulomb_controller_pole_tuning.py \
  --base-controller artifacts/openarm-controller-lab/openarm-plant-state-feedback.controller.json \
  --output artifacts/openarm-coulomb-controller-poles/candidates
# Run each candidate on the unchanged 0.5 N*m Rapier fixture, then:
python adapters/simulator/rne_gazebo_harmonic/build_openarm_coulomb_controller_pole_tuning_report.py \
  --candidate-root artifacts/openarm-coulomb-controller-poles/candidates \
  --trace-root artifacts/openarm-coulomb-controller-poles/results \
  --output artifacts/openarm-coulomb-controller-poles/report
```

The `fast`, `baseline`, `medium`, and `slow` candidates produce respectively
`0.055565`, `0.036139`, `0.034136`, and `0.035176 rad` RMSE. All retain exact
replay, exact controller identity, and exact plant realization, but none passes
the unchanged `0.02 rad` gate, so the report remains `needs_tuning` and selects
no controller. The baseline spends 1,020 of 3,600 samples at the joint-5 effort
command limit: its saturated-sample RMSE is `0.065922 rad`, versus `0.010210
rad` while not saturated. The otherwise identical zero-Coulomb run reaches
that command-model limit for only three samples and passes at `0.013454 rad`.
The browser report deliberately distinguishes commands from measurements. At
0.5 N*m, Rapier and MuJoCo retain command-model saturation fractions of
`28.333%` and `0.278%`, while Gazebo's adapter-owned backend diagnostic reports
`6.314%`; none of the three traces claims measured joint effort. This localizes
the next experiment to portable actuator-realization evidence before bounded
model-based Coulomb compensation is accepted. The supported friction envelope
and acceptance limit remain unchanged.

The portable measurement boundary is `rne_physics::JointEffortMeasurement`.
It is optional completed-step evidence, so a missing backend measurement stays
`Unavailable` instead of being replaced by the reconstructed PD command. The
joint-feedback sensor preserves its capture timestamp and declared latency,
adds no effort noise, and rejects non-finite or revolute/prismatic mismatches.
MuJoCo now publishes its native actuator-space force through this path. A real
0.5 N*m rerun retains all 3,600 measurements, exact replay, and the unchanged
`0.017886 rad` RMSE, while exposing an `18.148104 N*m` actuator-space peak
against the `7 N*m` bounded command. That peak comes from MuJoCo's implicit
damping compensation being realized outside the command clamp; it is retained
as a conformance failure lead, not accepted as equivalent actuator semantics.
Rapier continues to report effort unavailable until a qualifying solver or
hardware measurement exists.

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

## Sweep the OpenArm actuator command-transport delay

The fourth robustness dimension delays only right joint 5 after controller
limits and before backend actuation. The controller continues to receive only
the typed, one-period-latent joint feedback; it does not read the delay history
or backend state:

```bash
python3 adapters/simulator/rne_gazebo_harmonic/build_openarm_robustness_suite.py \
  --dimension actuator_command_delay \
  --output artifacts/openarm-command-delay-robustness-lab
```

The fixed `[0, 1, 2, 3, 4]` control-period grid passes the declared supported
maximum of two periods and first leaves the supported envelope at three. The
same two/three-period boundary is executed on Rapier, native MuJoCo, and
Gazebo. For all six boundary traces, the report independently recomputes the
source step and proves with zero realization delta that the applied joint-5
target at step `k` equals the retained controller target at `k-delay_steps`;
the other eight targets remain current. The first failing case is localized to
step 3241, where three periods selects source step 3238. Tracking peak, IAE,
and recovery requirements remain green and are reported separately from the
declared transport envelope.

```bash
python3 adapters/simulator/rne_gazebo_harmonic/build_openarm_robustness_report.py \
  --suite-root artifacts/openarm-command-delay-robustness-lab \
  --output artifacts/openarm-command-delay-robustness-lab/report
cargo run --locked -p showcase_captures \
  --bin rne-openarm-robustness-failure-replay -- \
  --report artifacts/openarm-command-delay-robustness-lab/report/openarm-command-delay-robustness-report.json \
  --trace artifacts/openarm-command-delay-robustness-lab/delay-003steps/rne_rapier/rapier-success-trace.json \
  --output artifacts/openarm-command-delay-robustness-lab/report/minimum-command-delay-failure.rne-replay
```

## Sweep the OpenArm actuator command slew-rate limit

The fifth robustness dimension applies a physical command slew-rate limit to
right joint 5 after controller limits and before backend actuation. Each
applied target is clamped against the previous applied target using the fixed
control period, so the contract is expressed in `rad/s` rather than an
abstract severity value:

```bash
python3 adapters/simulator/rne_gazebo_harmonic/build_openarm_robustness_suite.py \
  --dimension actuator_command_rate_limit \
  --output artifacts/openarm-command-rate-limit-robustness-lab
```

The descending `[0.40, 0.25, 0.15, 0.10, 0.05] rad/s` grid exercises steps
1298 through 1357, where all three backends issue changing joint-5 commands.
`0.15 rad/s` is the last supported case: Rapier, native MuJoCo, and Gazebo
perform 43, 42, and 38 limited applications respectively while passing peak,
IAE, and recovery gates. `0.10 rad/s` is the first unsupported case and limits
60, 59, and 57 applications. Every backend localizes the first contract
deviation to step 1298 against the fixed `0.15 rad/s` minimum. The browser
report independently reconstructs the recursive previous-applied-target
relationship for both boundary cases and obtains zero realization delta on all
six traces; the later closed-loop IAE effect remains a separate check.

```bash
python3 adapters/simulator/rne_gazebo_harmonic/build_openarm_robustness_report.py \
  --suite-root artifacts/openarm-command-rate-limit-robustness-lab \
  --output artifacts/openarm-command-rate-limit-robustness-lab/report
cargo run --locked -p showcase_captures \
  --bin rne-openarm-robustness-failure-replay -- \
  --report artifacts/openarm-command-rate-limit-robustness-lab/report/openarm-command-rate-limit-robustness-report.json \
  --trace artifacts/openarm-command-rate-limit-robustness-lab/rate-100mrad-s/rne_rapier/rapier-success-trace.json \
  --output artifacts/openarm-command-rate-limit-robustness-lab/report/minimum-command-rate-limit-failure.rne-replay
```

## Sweep the OpenArm actuator command deadband

The sixth robustness dimension applies a physical command deadband to right
joint 5 after controller limits and before backend actuation. During the pulse,
the backend-facing target holds its previous applied value whenever the new
controller command differs by no more than the declared deadband. The
controller observes the result only through typed, one-period-latent joint
feedback:

```bash
python3 adapters/simulator/rne_gazebo_harmonic/build_openarm_robustness_suite.py \
  --dimension actuator_command_deadband \
  --output artifacts/openarm-command-deadband-robustness-lab
```

The fixed `[0, 0.00025, 0.0005, 0.001, 0.002] rad` grid exercises steps 882
through 941. `0.001 rad` is the last supported case: Rapier, native MuJoCo,
and Gazebo hold 28, 31, and 29 changing commands while passing every control
performance gate. `0.002 rad` is the first unsupported case and produces 38,
40, and 40 holds. The largest independently recomputed held gaps are
`0.982-0.999 mrad` and `1.787-1.962 mrad` at the two boundary values, with
zero realization delta on all six traces. Every backend therefore fails only
the fixed `0.001 rad` actuator requirement at step 882; the browser report
keeps peak error, IAE, and recovery as separate passing checks.

```bash
python3 adapters/simulator/rne_gazebo_harmonic/build_openarm_robustness_report.py \
  --suite-root artifacts/openarm-command-deadband-robustness-lab \
  --output artifacts/openarm-command-deadband-robustness-lab/report
cargo run --locked -p showcase_captures \
  --bin rne-openarm-robustness-failure-replay -- \
  --report artifacts/openarm-command-deadband-robustness-lab/report/openarm-command-deadband-robustness-report.json \
  --trace artifacts/openarm-command-deadband-robustness-lab/deadband-2000urad/rne_rapier/rapier-success-trace.json \
  --output artifacts/openarm-command-deadband-robustness-lab/report/minimum-command-deadband-failure.rne-replay
```
