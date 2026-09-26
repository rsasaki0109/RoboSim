# Arm position control and the servo stability boundary

A light manipulator has to do two things before any of its controllers are
worth measuring: start where it was authored, and stay there when a servo is
asked to hold it. The shipped SO-101 scene did neither, and the second failure
was mistaken for a control-tuning problem for as long as the first one was
hiding it.

## The authored pose was not the simulated pose

`assets/robots/so101.rne.robot.toml` declared `articulation = true` and nothing
else. The SO-101 URDF comes from OnShape and every joint carries a non-identity
origin `rpy` — `shoulder_lift` is `rpy="-1.5708 -1.5708 0"`, `wrist_roll` is
`rpy="1.5708 0.0486795 3.14159"`. Without `use_joint_origin_rpy` the wired joint
frames are axis-aligned, they disagree with the authored link poses, and the
solver resolves that disagreement on the first step as if it were a constraint
violation.

Worst link displacement after exactly one physics step, measured from the
authored pose:

| Asset configuration | After 1 step | After 300 steps |
| --- | ---: | ---: |
| `articulation` only (as shipped) | **11701.2600 m** | 0.8100 m |
| `+ use_joint_origin_rpy` | **0.0089 m** | 0.2368 m |
| `+ multibody` | 0.0492 m | 0.4214 m |
| `+ weld_fixed_children` | 0.0913 m | 0.3901 m |

The arm was launched 11.7 km and then dragged back by the impulse joints over
the following steps, which is why the failure did not look like an explosion
from the outside: by the time anything sampled the arm it was merely in the
wrong pose. `assets/robots/mm_mobile_so101.rne.robot.toml` has carried
`use_joint_origin_rpy = true` with a comment explaining exactly this since it
was written; the standalone asset simply never got the flag.

The fix is that one line. The 0.2368 m residual over 300 steps is the
unactuated arm falling under gravity, which is correct behaviour for an arm
with no servo.

## 60 Hz is below this arm's servo stability boundary

With the pose fixed, a position servo asked to hold the authored angles still
drifts, and **the drift grows with stiffness**. Settled worst-link drift after
600 control steps, damping fixed at `0.1 * stiffness`, effort ceiling 10 N·m:

| Physics rate | k=5 N·m/rad | k=20 | k=80 |
| --- | ---: | ---: | ---: |
| **60 Hz (default)** | — | 0.3242 m | **0.4406 m** |
| 120 Hz | 0.5618 m | 0.1322 m | 0.0801 m |
| 240 Hz | 0.0965 m | 0.0978 m | 0.0846 m |
| 480 Hz | 0.0847 m | 0.0898 m | 0.0885 m |
| 960 Hz | 0.0823 m | 0.0852 m | 0.0854 m |

At 60 Hz the error rises with gain, and the transient is worse than the settled
value: k=80 peaks at 2.0693 m on the way, and the legacy `JointMotor` path at
k=20000 peaks at 41.3855 m. From 120 Hz the ordering inverts and behaves like a
servo — stiffer holds better. From 240 Hz the sweep is flat, because the servo
is no longer what limits the result.

This is a timestep bound, not a solver-convergence bound. The SO-101 links
weigh about 0.1 kg with inertias near `1e-4 kg m^2`, and an explicit position
spring is stable only while roughly `k < 2 I / dt^2`; at 60 Hz that ceiling is
under 1 N·m/rad, which is below any gain that would hold the arm at all. Adding
solver iterations does not help because the instability is between steps, not
within one.

**Any servo on this arm needs at least 240 Hz.** Use
`step_joint_position_actuation_targets_substeps` to subdivide the control period
rather than raising the control rate itself.

## Use the unit-explicit actuation path

There are two position-control paths and they do not mean the same thing.

- `configure_named_revolute_position_actuation` inserts
  `JointActuation::RevolutePosition` together with
  `JointMotorGainModel::ForceBased`. Stiffness is N·m/rad, damping N·m·s/rad,
  effort N·m.
- `configure_position_motors` writes the legacy `JointMotor` and leaves the gain
  model at its default `AccelerationBased`, where gains are normalized by body
  inertia. On a link with `1e-4 kg m^2` a "stiffness" of 2000 is about
  0.2 N·m/rad of real authority, so the numbers a caller reasons about and the
  torques the joint receives are four orders of magnitude apart.

Manipulator work should use the first. The second remains for the existing
callers that were tuned against it.

## Open: a gain- and rate-independent residual

From 240 Hz upward a residual of about 0.085 m remains, and it responds to
neither stiffness nor physics rate. That pattern is not a control-loop
limitation: a servo that is merely too soft improves when stiffened, and one
that is unstable improves when the step shrinks. A constant offset that ignores
both is the signature of the servo holding a different angle from the one being
measured.

The suspected mechanism is the initial target.
`configure_named_revolute_position_actuation` reads the current angle from
`multibody_joint_position`, which returns `None` for an impulse-joint scene such
as this one, and falls back to the `Joint` component's `position`. If that
fallback reads zero while the joint frame carries the URDF's origin `rpy`, the
servo holds a pose the arm was never in.

**This is a hypothesis and has not been demonstrated.** What is established is
that the residual exists, that it is about 0.085 m, and that it is independent
of both gain and timestep over the ranges measured above.

A related usability problem is established: the shipped SO-101 scene cannot
report its own joint angles at all. `named_joint_position` reads `JointState`,
which impulse joints do not carry, so every joint reads exactly 0.000 rad while
the links are visibly moving. An earlier revision of
`docs/MULTI_FLOOR_NAVIGATION.md` reported that reading as evidence the motors
were dead; it was an artifact of the accessor.

## Reproducing

```bash
cargo test --locked -p rne_ai --test so101_arm_control
```

`the_authored_pose_survives_the_first_step` fails if the joint frames and the
authored pose diverge again. `a_stiffer_servo_holds_the_pose_at_least_as_well`
fails if the servo returns to the unstable side of the boundary.
