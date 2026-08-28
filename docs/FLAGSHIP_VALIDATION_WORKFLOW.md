# Flagship validation workflow

The v0.2.0 release flagship is one fixed-step workflow, not a launcher for
disconnected demos. A lift-capable mobile manipulator loads the committed
scene, robot manifest, and URDF, validates wrist RGB-D observations, yields at
a red signal while a traffic actor clears a shared aisle, then navigates,
friction-grasps, lifts, transports, and places the payload.

Run the complete clean-checkout gate on Windows or Linux:

```bash
cargo run --locked -p xtask -- flagship
```

The command regenerates `artifacts/flagship-validation`, proves the successful
run, injects a deterministic RGB-D blackout for seed 7, reproduces the
fail-closed `perception_stream_alive` violation, minimizes three active
dimensions to the one required blackout dimension, creates a Failure Capsule
from the minimized replay, and verifies every capsule digest and replay
invariant.

The source-tree command above is the native Rapier development gate. The
release archive additionally ships a pinned MuJoCo runtime and the
`rne-flagship-proof` executable. From the extracted archive, an external user
can run the complete installed proof without Cargo, ROS 2, a separate MuJoCo
installation, or network access:

```bash
./bin/rne-flagship-proof flagship-proof --cross-backend \
  --verify-installed-bundle . --measure-on "lab-machine-a"
```

On Windows use `bin\rne-flagship-proof.exe` with the same arguments. The
explicit, non-placeholder machine label writes `time-to-proof-report.json` and
measures bundle verification through the verified proof against the 15-minute
target. See [external flagship reproduction](EXTERNAL_FLAGSHIP_REPRODUCTION.md)
for extraction, checksum, submission, and maintainer-verification steps.

Large generated evidence can live outside the source checkout. Set
`RNE_ARTIFACTS_DIR` to an absolute path; `flagship`, `ci-headless`, and `ci`
then write flagship, parity, fuzz, behavior, Python API, physics-conformance,
and scenario-scale evidence under that real directory. Flagship replacement
retains the same symlink and bounded-deletion checks:

```powershell
$env:RNE_ARTIFACTS_DIR = "E:\RoboSim-artifacts"
cargo run --locked -p xtask -- ci
```

The configured directory itself must be a real directory, not a symlink or
junction. Paths containing spaces are passed directly to Cargo without shell
interpolation.

## One coordinated simulation

Example 74 advances the robot episode and backend-neutral traffic runtime with
the same `SimDuration`. Its coordinator has three gates:

1. Three valid wrist RGB-D observations complete inspection.
2. A deterministic step-60 event changes the shared-aisle signal to green.
3. Robot motion remains inhibited until the traffic actor has cleared the
   shared aisle.

Only then does the typed `IkMobileLiftPickPlacePolicy` control navigation and
manipulation. The traffic interlock, perception health, traffic collision
count, inspection deadline, aisle-clear deadline, and final pick/place outcome
are evaluated as `BehaviorContract`s. Core crates do not depend on one another
to fit this example: coordination stays at the application boundary, and both
subsystems retain their independent tests.

The fault plan contains a boolean perception blackout plus seeded traffic
departure and speed variations. The minimizer proves that the traffic
variations are irrelevant to the selected failure and retains only the
blackout. The failing step emits a zero-motion action and ends the workflow;
the simulated sensor payload is never treated as healthy after the injected
loss.

## Generated evidence

The output directory contains:

| File | Purpose |
|---|---|
| `workflow-report.json` | Versioned summary, imported-asset digest, event list, success verdict, and minimization facts |
| `flagship.task.json` | Portable TaskSpec v1 for the shared observation, action, reward, reset, randomization, and termination contract |
| `success.behavior-report.json` | Seven typed contracts from the successful 5,921-step run |
| `failure.behavior-report.json` | Expected failure verdict and artifact references |
| `failure.rne-replay` | Original three-dimension failure replay |
| `failure-minimized.rne-replay` | Deterministically verified one-dimension replay |
| `failure-minimized.behavior-case.json` | Small standalone failure input |
| `replay-inspector.html` | Self-contained success/failure timeline and top-down browser view |
| `failure-capsule/` | Portable capsule containing the minimized replay, reports, and browser inspector |
| `cross-backend-report.json` | With `--cross-backend`: same TaskSpec and controller on `rapier_native` and `mujoco_native`, with exact outcomes and SI-unit tolerances |
| `mujoco-failure.rne-replay` | With `--cross-backend`: MuJoCo reproduction of the same intentional first violation |
| `recorded-shadow-proof.json` | With `--cross-backend`: bound recorded playback, non-actuating shadow, and expected-disconnect cases |
| `installed-proof-report.json` | Installed producer, bundle verification, execution paths, artifact hashes, and aggregate verdict |
| `time-to-proof-report.json` | With `--measure-on`: named-machine elapsed measurement and 15-minute verdict |

Open `replay-inspector.html` directly in a browser; it embeds its data and does
not require a server or network access. The run selector switches between the
complete successful trace and the minimized failure, and the frame slider
shows robot, payload, traffic, signal, perception, interlock, grasp, and task
state.

The installed cross-backend proof runs the same
`rne.flagship.mobile_lift_shared_aisle.v2` TaskSpec and
`rne.ai.portable_ik_mobile_lift_pick_place_controller.v2` controller
configuration on Rapier and native MuJoCo. Both backends must pass the nominal
task, reproduce the intentional failure at the exact first step and simulation timestamp,
and pass the registered tolerances with explicit units. The Failure Capsule
binds the task, model, configuration, reports, replay, browser inspector,
producer, and installed-bundle verification by size and SHA-256.

The backend-neutral source of that exact contract is
`rne_ai::flagship_mobile_lift_task_spec_v2`; the stable TaskSpec/controller IDs
are exported beside it. Release, recorded/shadow, simulator-adapter, and
bounded hardware paths must call that generator rather than copy its tensor schema.

The recorded proof exercises the same typed contract as bounded playback and
non-actuating shadow traffic, plus an expected process-disconnect case. It is
not evidence of physical actuation. Gazebo qualification remains a separate
process-isolated external simulator-adapter submission, and bounded physical
execution remains a separate hardware-evidence gate; neither is relabelled as
having passed by this release-archive proof.
