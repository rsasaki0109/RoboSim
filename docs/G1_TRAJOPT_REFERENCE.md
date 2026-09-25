# G1 non-RL trajectory-optimization reference

This campaign targets offline whole-body trajectory optimization followed by
feedback tracking. No RL training is involved. The first deliverable is a
reproducible external direct-transcription comparison, not a simulated backflip.

The [recorded measurements and trajectories](evidence/g1-trajopt-reference/README.md)
include a passing jump candidate, an initial full-rotation candidate that fails
the flight-momentum screen, and a new 50 ms momentum-based backflip candidate
that passes transcription, flight conservation, and native inverse dynamics.
Independent fine-step free-flight tracking is measured separately; contact
landing and mesh convergence are still unvalidated.

## Run the external comparison

The optional Python environment and upstream checkout live under `target/`.
Neither Pinocchio nor IPOPT is a dependency of any RNE crate. From the repository
root, with micromamba available:

```bash
git clone https://github.com/upatras-lar/se3_trajopt target/research/se3_trajopt
git -C target/research/se3_trajopt checkout 1bbadc9573b2989a0f414888d4fa4af137d57db9
micromamba create -y -p "$PWD/target/research/trajopt-env" -f scripts/g1-trajopt-environment.yml
target/research/trajopt-env/bin/python -I -m unittest discover -s scripts -p test_g1_trajopt_reference.py
target/research/trajopt-env/bin/python -I scripts/g1_trajopt_reference.py --task stand --push-steps 2 --flight-steps 2 --landing-steps 2 --output target/research/g1-stand
target/research/trajopt-env/bin/python -I scripts/g1_trajopt_reference.py --task jump --output target/research/g1-jump
target/research/trajopt-env/bin/python -I scripts/g1_trajopt_reference.py --task backflip --dt-s .05 --push-steps 12 --flight-steps 12 --landing-steps 12 --output target/research/g1-backflip
cargo run --release -p g1_backflip --example 113_g1_backflip -- --check-reference target/research/g1-backflip
```

Use Python isolation (`-I`) so user-installed packages and `PYTHONPATH` do not
override the pinned environment. The script refuses a different revision or modified tracked upstream files.
`--max-iterations` and `--max-wall-time-s` bound a solve. `--check-jacobian`
enables IPOPT's derivative checker; the unit suite also checks a seeded
directional finite difference against the assembled sparse Jacobian.

Each run writes `summary.json` and `trajectory.json`, including failures.
Exit 0 requires solver success, finite values, constraint and variable bounds
within tolerance, independently recomputed dynamics/integration residuals, the
requested maneuver, and the flight-momentum screen. Exit 2 means that gate
failed. Timing uses a monotonic
clock only to measure optimization execution, never to advance simulated state.
`transcription_passed` separates a converged, nondegenerate discrete trajectory
from `flight_momentum_screen_passed`. The latter can reject a converged
transcription; `passed` requires both. `plant_validated` is always false at this
stage. In particular, the measured coarse backflip exits 2 despite IPOPT success.

`--warm-start PREVIOUS_OUTPUT_DIRECTORY` interpolates a saved candidate to a
different mesh with the same phase durations. Model, mass policy, contacts and
joint order must match. For example, a 12/12/12-node schedule at 0.05 s becomes
20/20/20 at 0.03 s. The source trajectory's SHA-256 is recorded. An unsuccessful
candidate can still be used as a warm start, but is never relabelled successful.
The transition regression checks that interpolating onto the same grid retains
the last push force even though the following flight node has no force entry.
`--project-warm-start-dynamics` optionally recomputes acceleration using forward
dynamics with interpolated torques and contact forces. This makes each seed node
dynamically consistent, but need not improve integration residuals or convergence;
the recorded 30 ms experiment was worse with this projection.

`--integration body-euler` is the upstream baseline. The experimental
`--integration world` updates base linear/angular velocity in world coordinates,
using classical acceleration `R (a_linear + omega x v_linear)`, and a
constant-acceleration root position/rotation update. Joint integration retains
the semi-implicit rule. The single-body regression verifies ballistic motion
through a large rotation when the center of mass is at the root. This does not
establish conservation for a general articulated robot; the same flight screen
still applies. Its cheap kinematic constraint uses central-difference Jacobians;
inverse-dynamics derivatives remain analytic. Both integration Jacobians have
directional-difference tests.

```bash
target/research/trajopt-env/bin/python -I scripts/g1_trajopt_reference.py --task backflip --integration world --dt-s .05 --push-steps 12 --flight-steps 12 --landing-steps 12 --warm-start docs/evidence/g1-trajopt-reference/backflip --max-wall-time-s 240 --output target/research/g1-backflip-world
```

`--integration momentum` replaces the three root translation rows with a
constant-force CoM position update and the six base velocity rows with centroidal
impulse balance. World orientation and joint integration retain the `world`
scheme. The contact wrench is evaluated at the start of each interval about the
current CoM; gravity acts at the CoM. Consequently, a feasible flight interval
has ballistic CoM translation, the gravity-induced linear momentum change, and
constant centroidal angular momentum. This is still a finite-step stance model,
not an independent physics replay or a proof of mesh convergence.

Its integration residual uses m, rad, joint rad/s, N s, and N m s; the audit
reports the impulse errors separately from velocity errors. The NLP and residual
audit share the momentum-update helper, while the trajectory-level flight screen
checks exported momentum independently. Tests verify the full sparse Jacobian
and construct an articulated rotating flight state from its centroidal map:
ballistic motion passes, but an unforced horizontal velocity kick fails.

```bash
target/research/trajopt-env/bin/python -I scripts/g1_trajopt_reference.py --task backflip --integration momentum --dt-s .05 --push-steps 12 --flight-steps 12 --landing-steps 12 --warm-start docs/evidence/g1-trajopt-reference/backflip --max-wall-time-s 180 --output target/research/g1-backflip-momentum
```

That bounded run supplies a seed, not a feasible trajectory. To reproduce the
passing 50 ms candidate from the saved seed, add `--feasibility-only`. This removes
posture/acceleration costs but retains every constraint and promotion check:

```bash
target/research/trajopt-env/bin/python -I scripts/g1_trajopt_reference.py --task backflip --integration momentum --feasibility-only --dt-s .05 --push-steps 12 --flight-steps 12 --landing-steps 12 --warm-start docs/evidence/g1-trajopt-reference/backflip-momentum-seed --max-wall-time-s 180 --output target/research/g1-backflip-momentum-feasible
target/research/trajopt-env/bin/python -I scripts/g1_trajopt_flight_check.py docs/evidence/g1-trajopt-reference/backflip-momentum-50ms --dt-s .000005 --kp-nm-rad 80 --kd-nm-s-rad 4
```

The flight checker starts at the saved takeoff state, applies saved joint torques
with optional clipped joint PD feedback, and integrates independent ABA forward
dynamics using explicit midpoint. It never applies the saved accelerations or
contact forces. Feedback uses interpolated joint positions/velocities; base gains
are absent. It reports endpoint errors and limit violations without declaring
the maneuver successful. The high PD damping needs a small integration/control
step for this model: 1 ms is numerically unstable, whereas 0.01 and 0.005 ms give
matching tracking results. These are offline numerical checks, not demonstrated
hardware control rates. Takeoff/contact and landing are excluded.

## Comparison contract

- Load RNE's `assets/robots/g1_description/g1_23dof.urdf`, recording its SHA-256.
  It has **23 actuated joints plus 6 floating-base velocities**, not 29 actuated
  joints. Names accompany every torque/state vector.
- Use the declared URDF inertias **and the selected missing-inertia policy**.
  Default `--mass-policy rne` reproduces example 113's importer fallback:
  the four inertial-less fixed frames (`imu_in_torso`, `imu_in_pelvis`,
  `d435_link`, `mid360_link`) each contribute a 1 kg point mass, giving
  **38.13385728 kg**. `--mass-policy declared` instead retains Pinocchio's
  zero-mass treatment, giving **34.13385728 kg**. The native importer is unchanged.
  This 4 kg difference was discovered by the native dynamics check: with the
  wrong mass policy even standing left about 39 N of unbalanced force.
  Constant torque ceilings match example 113;
  joint position and velocity bounds come from the URDF. These are benchmark
  bounds, not a motor torque-speed envelope.
- Pinocchio uses Z-up, position plus `xyzw` quaternion, and local body-twist
  base velocity. World vectors transform to RNE as `(x, y, z) -> (x, z, -y)`.
  Do not rotate the local body-twist components as though they were world vectors.
- Default contacts are four points per sole at local x `[-0.05, 0.09]` m,
  y `[-0.025, 0.025]` m, z `-0.035` m. These are a declared benchmark support
  polygon, not an identified Rapier collision manifold. `--contacts toes`
  selects example 113's two points at `(0.09, 0, -0.035)` m.
- Initial leg roll offsets are zero so the soles are parallel to the floor.
  Root height is computed from forward kinematics so the contact points start
  on the ground; it is not forced to example 113's 0.82 m. The manifest records
  this difference. Thus this is a **same-URDF comparison**, not an identical
  reproduction of the historical example 113 warm start.
- The default timestep is 0.03 s, with 15 push, 10 flight, and 15 landing
  intervals (41 states). The standing task keeps contacts throughout. Landing
  returns to the initial pose with zero velocity and terminal acceleration.
- Forces are optimization variables in local contact coordinates; exports
  convert them to world coordinates. Floating-base RNEA rows are constrained to
  zero. Actuated RNEA rows recover torques and are bounded. Configuration,
  velocity, and acceleration are simultaneous decision variables.

Two small compatibility corrections are implemented in the wrapper, with the
upstream checkout unchanged: duplicate total-time Jacobian entries are removed,
and terminal friction evaluation visits all contacts rather than returning
after the first. The latter has a regression with a downward force at the last
terminal contact.

## What the evidence means

The independent audit reconstructs `M a + h - J^T f` and the selected
configuration/velocity integration using Pinocchio directly. It reports force
in N, torque in N m, translational errors in m or m/s, and angular errors in rad
or rad/s. These are checked separately against the declared numeric tolerance;
the raw mixed-unit maximum is not a physical distance.

The native `--check-reference` mode verifies the URDF and optional trajectory
checksums, maps joints by name, reconstructs the floating-base chart, and
recomputes inverse dynamics and `J^T f` with **rne_dynamics**. It compares
base force/torque residuals and recovered joint torques, and checks total mass.
It does not run the native contact solver, integrate the controls, or establish
that the reference can be tracked.

The additional flight audit checks world linear momentum against gravity's
impulse and world angular momentum against its takeoff value. It includes the
first landing state because this transcription has no impact reset. Relative
linear error is divided by the larger of initial momentum magnitude, gravity
impulse over flight, and 1 N s; angular error is divided by the larger of initial
angular momentum magnitude and 1 N m s. A declared 2% screen distinguishes
discretization artifacts from small NLP defects. This is an engineering screen,
not a claim of hardware accuracy. The regression suite explicitly injects an
impossible horizontal impulse and requires the screen to fail.

A jump additionally requires at least 0.05 m CoM rise and 0.02 m clearance of
the lowest declared foot point at some flight node. A backflip additionally
requires a signed sagittal rotation within 0.2 rad of -2 pi. The rotation is
unwrapped from the sagittal projection; this diagnostic is intended for the
sagittal maneuver, not general 3D acrobatics. A low-defect no-jump/no-flip
trajectory must fail the maneuver gate.

`plant_validated` remains false: this is finite-step transcription with smooth
landing, no impulsive reset, and contact constraints at nodes. There are no
self-collision/non-foot collision constraints, identified motor envelopes,
independent physics-backend replay, or post-landing hold test yet. Even a passing
transcription is only a candidate for the next stage. No-slip here means constant
contact positions at successive nodes; it does not impose instantaneous `J v = 0`.

## Native evaluation repair

`rne_oc::max_defect` now returns infinity on failed dynamics, malformed
predictions/trajectories, and non-finite values. Previously a failed interval
could disappear from the maximum, and NaNs could be hidden by `f64::max`.
Regression tests include a solver run that must not report convergence.
Example 113 also counts non-finite/malformed predictions as failed nodes.

## Next experiments

1. Standing and vertical jump transcription evidence with the same manifest
   and native inverse-dynamics agreement is recorded; plant replay remains open.
2. Refine the passing 50 ms momentum-based backflip candidate and establish
   continuous-dynamics and mesh convergence. A coarse body-twist Euler
   transcription can satisfy every node equation yet invent horizontal momentum
   during a full rotation. Compare a midpoint/implicit or momentum-preserving
   integration scheme before claiming physical feasibility. The experimental
   world-velocity integration run did not converge within its 240 s budget.
   Add collision,
   actuation, and landing-impact constraints.
3. Extend the independent free-flight ABA/PD check to a high-rate RNE plant
   including takeoff and landing contacts. Joint names and state conventions
   already have a native inverse-dynamics comparison.
4. Add non-RL tracking (joint PD/WBC, then MPC if needed), with an actual
   liftoff, one rotation, foot landing, several seconds of stable standing, and
   deterministic world-state replay as the final gate.
5. Port the successful formulation to `rne_oc`, using lifted acceleration and
   contact-force variables and a solve that couples the entire horizon.

The historical defect floor is evidence about particular references and solver
settings, not a proof of G1 physical infeasibility. Keep the historical 0.18 and
the committed example 113 reference's larger defect as distinct experiments.

## Implementation verification (2026-09-21)

- Formatting, workspace Clippy (`-D warnings`), and dependency boundaries pass.
- Six multiple-shooting tests, the native coordinate-conversion test, and all
  10 isolated Python tests pass. All three baseline native dynamics checks pass.
- `cargo run -p xtask -- ci-headless` passes, including the flagship workflow.
- The first workspace build exhausted disk space. After build-artifact cleanup,
  the workspace test run failed three existing TCP/RGB-D control tests. A serial
  rerun passed those three but failed another quit-response check; a subsequent
  normal CI run passed that control suite. This suggests intermittent teardown
  behavior, not a confirmed cause.
- Full CI was stopped during the mobility benchmark tests to preserve disk
  capacity after the user's disk-space reminder. It did not complete; the full
  workspace/CI must not be described as green. Only debug information in newly
  generated executables and this task's package download cache were cleaned;
  source and recorded trajectories were retained. Free disk space recovered to
  approximately 43 GiB. Check capacity before further large builds and preserve
  at least 30 GiB of free space for this campaign.

## References

- [se3_trajopt](https://github.com/upatras-lar/se3_trajopt), BSD-2-Clause;
  [floating-base parameterization study](https://arxiv.org/abs/2508.11520),
  Humanoids 2025. This wrapper calls its API rather than vendoring it.
- [robotoc](https://github.com/mayataka/robotoc), BSD-3-Clause;
  [lifted contact dynamics](https://arxiv.org/abs/2108.01781) and
  [switching-time optimization](https://arxiv.org/abs/2112.07232).
- [Aligator](https://github.com/Simple-Robotics/aligator), for a further
  constrained trajectory-optimization solver comparison.

OmniXtreme is background evidence only; it is not part of this non-RL route.
Its [Table III](https://arxiv.org/html/2602.23843v1) reports 96.36% for seven
motions in the Flip category over 55 trials, not for backflips alone.
