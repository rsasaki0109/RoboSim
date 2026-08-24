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

## Run the same OpenArm controller on Rapier and Gazebo

The portable pose-cycle controller is compiled once into an exact action trace.
RNE/Rapier and Gazebo then consume those identical bytes; neither backend owns
a private copy of the trajectory.

```bash
cargo run --locked -p showcase_captures --bin rne-openarm-rapier-trace -- \
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

The successful comparison gates both final tracking errors and the final
cross-backend joint delta with named radian tolerances. Maximum transient
divergence is retained as a non-gating dynamics diagnostic. The intentional
controller fault truncates the nine-element action at step 307; both RNE and
Gazebo identify that exact first violation, and Gazebo proves that rejection
did not advance state before accepting the corrected action.

The control-dynamics report evaluates the complete trajectory rather than only
the final pose. It binds the RNE force-based actuation configuration and Gazebo
runtime/configuration hashes, then records per-joint RMSE, IAE, ISE, terminal
bias, position range, peak velocity, and the first URDF position/velocity-limit
violation. `needs_tuning` is a valid diagnostic result and must not be converted
to `passed` by widening a tolerance.

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
expected diagnostic status is `expected_coupling_failure_reproduced` until the
Rapier coupled response is corrected; a future fix must turn the coupled and
hard-limit checks green rather than changing the registered tolerances.
