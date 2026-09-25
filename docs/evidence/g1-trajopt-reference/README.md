# G1 external optimization measurements — 2026-09-21

Generated using `scripts/g1_trajopt_reference.py` and the pinned external
checkout/environment described in [the experiment guide](../../G1_TRAJOPT_REFERENCE.md).
The default **RNE missing-inertia mass policy** is used: 38.13385728 kg.

| Case | dt (s) | Push / flight / landing intervals | IPOPT iterations | Max NLP bound/constraint violation | CoM rise (m) | Peak torque (N m) | Relative flight linear-momentum error | Overall gate |
|---|---|---|---|---|---|---|---|---|
| Stand | 0.03 | 2 / 2 / 2 (all contacts active) | 6 | 5.71e-9 | ~0 | 6.27 | 0 | Pass |
| Jump | 0.03 | 15 / 10 / 15 | 36 | 5.26e-10 | 0.1316 | 46.23 | 0.350% | Pass |
| Backflip | 0.05 | 12 / 12 / 12 | 77 | 4.26e-5 | 0.4555 | 135.19 | **48.07%** | **Fail** |

All three IPOPT solves succeeded. The backflip's signed rotation is -2 pi and
its discrete transcription passes, but the world linear-momentum discrepancy
over flight is **107.90 N s**. It cannot be promoted to a physically valid
backflip. Even the passing jump is only a transcription/conservation candidate;
none of these are physics-backend replays or successful landing-hold tests.

`backflip-world/` records the experimental world-velocity integration run,
warm-started from the saved backflip. It hit the 240 s wall-time budget after 46
iterations with a constraint violation of 120.67 and failed all promotion gates.
Its jump/rotation metrics are not evidence of a feasible maneuver. The
single-body ballistic regression passes, but this bounded G1 run does not yet
establish that the alternative integration can solve the full contact problem.

The `summary.json` files apply the final combined gate to the recorded solver
metrics: `passed = transcription_passed && flight_momentum_screen_passed`.
This classification was added after the initial solves; the solver metrics and
trajectory bytes are unchanged. The environment's timing and solver stopping
point are not portable success criteria. The trajectory SHA-256 excludes timing.

The three baseline directories contain:

- `summary.json`: model checksum, joint order, limits, contact geometry, phase
  durations, solver outcome, residuals, conservation errors, and limitations.
- `trajectory.json`: poses, body twists and their derivatives, joint torques,
  world contact forces, CoM positions, and centroidal momentum at each node.
- `native-check.json`: independently recomputed `rne_dynamics` results. The
  maximum joint-torque disagreement with Pinocchio is below 5e-13 N m for these
  trajectories, and total masses agree within 2e-14 kg. This checks the model
  and inverse dynamics, not integration, contact simulation, or tracking.

Recheck the saved trajectory without installing any external Python solver:

```bash
cargo run --release -p g1_backflip --example 113_g1_backflip -- --check-reference docs/evidence/g1-trajopt-reference/backflip
```

The native check should pass while the backflip's overall gate remains false.
To regenerate a case from scratch, use the corresponding counts above with
the commands in the guide. `environment-linux-64.txt` records the full resolved
environment; the portable dependency choices are in
`scripts/g1-trajopt-environment.yml`. The recorded environment includes a
host-specific x86-64 microarchitecture selection.

Initial runs imported NumPy 2.4.2 and SciPy 1.17.0 from the user site despite
newer versions being installed in the conda environment. The exported environment
has since been aligned to those actually used versions. The guide uses Python
isolation (`-I`); all 10 tests pass in isolation, and an isolated standing solve
reproduces the exact trajectory hash below. Jump/backflip trajectories were not
regenerated solely for this environment correction.

The independent standing solves, including the isolated run, produced the same trajectory hash:
`77f44b069ad2b17cdef5bd320523ec512081f8d3df4b86c769cbd33880abaadf`.
This is same-environment optimizer reproducibility, not a world-state replay
hash from a physics plant.

## Momentum integration and free-flight follow-up

The momentum formulation replaces base translation/velocity integration with
CoM position and centroidal impulse balance. The saved 50 ms result passes the
discrete gate; it is not a contact-plant backflip demonstration.

| Directory | Configuration | Iterations | Max constraint violation | Overall gate |
|---|---|---|---|---|
| `backflip-momentum-seed` | 50 ms, posture/acceleration cost, 180 s budget | 41 | 3.129 | Fail |
| `backflip-momentum-50ms` | Same mesh, feasibility-only, warm-started from that seed | 7 | 6.33e-10 | Pass |
| `backflip-momentum-30ms` | 20/20/20 intervals, interpolated passing seed, 180 s | 17 | 0.771 | Fail |
| `backflip-momentum-30ms-projected` | Same refinement, accelerations recomputed from torque/forces, 180 s | 14 | 180.174 | Fail |
| `jump-world` | World-velocity scheme, original jump seed, 120 s | 56 | 0.0176 | Fail |

The passing backflip has a -2 pi rotation, 0.4457 m CoM rise, and peak joint torque
138.11 N m. Relative linear/angular flight-momentum errors are 2.69e-14 and
4.46e-14. Its `native-check.json` passes with joint-torque disagreement below
2.6e-13 N m. Flight conservation is now imposed by the transcription, so these
small numbers alone do not independently validate continuous dynamics.

The subsequent free-flight checker uses ABA and explicit midpoint integration,
starting from the saved takeoff state and ending at the first landing state.
It applies held joint torques with no contacts; it does not replay the saved
accelerations. Optional joint PD feedback is torque-clipped and uses linearly
interpolated reference joint positions/velocities. It does not simulate takeoff
or landing. The high-rate simulation/control step is a numerical experiment,
not a claim about an implementable hardware sample rate.

| Control | Integration/control step | Final max joint position error (rad) | Final root rotation error (rad) | Max joint position-limit violation (rad) |
|---|---|---|---|---|
| Open loop | 1 ms | 2.0718 | 0.26380 | 0.56457 |
| Open loop | 0.5 ms | 2.0716 | 0.26390 | 0.56451 |
| PD, kp=80 N m/rad, kd=4 N m s/rad | 0.01 ms | 0.054279 | 0.065206 | 0 |
| Same PD | 0.005 ms | 0.054278 | 0.065206 | 0 |

Both fine PD runs have zero torque saturation, peak torque about 39.19 N m,
root position error about 6.56 mm, and final max joint velocity error about
0.681 rad/s. The 0.005 ms run has ballistic CoM error 1.84e-9 m. The coarse
1 ms PD pilot is numerically unstable; the 0.1 ms pilot still saturates heavily.
Their measurements are retained as negative numerical-step evidence and must
not be treated as trustworthy tracking results. The open-loop error persists
as the step is halved, while the fine PD results agree. This supports the need
for feedback, but does not establish mesh convergence of the trajectory solve.

Each optimization directory preserves the original summary/trajectory bytes;
source trajectory hashes record the warm-start chain. The timed-out seed used
for the passing solve is included so its exact stopping point need not be
reproduced. See the guide for the corresponding command. Earlier pilot flight
reports omit the later-added `numerical_momentum_screen_passed` field; their raw
metrics are unchanged.

Validation for this Python-only follow-up: 15 isolated tests, Ruff, and the native
saved-trajectory check pass. No Rust source changed in this follow-up, and no
large Cargo builds were repeated after the disk-space reminder. Total committed
evidence is approximately 1.3 MiB; disk free space remained approximately 43 GiB.
Full workspace/CI status remains as recorded in the guide, not newly green.
