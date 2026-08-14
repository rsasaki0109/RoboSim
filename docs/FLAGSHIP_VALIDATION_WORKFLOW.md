# Flagship validation workflow

The v0.7 flagship is one fixed-step workflow, not a launcher for disconnected
demos. A lift-capable mobile manipulator loads the committed scene, robot
manifest, and URDF, validates wrist RGB-D observations, yields at a red signal
while a traffic actor clears a shared aisle, then navigates, friction-grasps,
lifts, transports, and places the payload.

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

Open `replay-inspector.html` directly in a browser; it embeds its data and does
not require a server or network access. The run selector switches between the
complete successful trace and the minimized failure, and the frame slider
shows robot, payload, traffic, signal, perception, interlock, grasp, and task
state.

This is the native Rapier execution path for the v0.7 workflow. The remaining
v0.7 portability gate is to run the same versioned flagship task through a
second production physics path and register cross-backend outcome tolerances;
the report names only `rapier_native` until that evidence exists.
