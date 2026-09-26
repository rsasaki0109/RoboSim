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

Raising the rate removes the instability. It does not make the servo work — see
the next section. Use `step_joint_position_actuation_targets_substeps` to
subdivide the control period rather than raising the control rate itself.

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

## Open: the position servo has no authority on this arm

From 240 Hz upward the held-pose error stops responding to stiffness. It is not
that the servo is too soft — **it is indistinguishable from no servo at all.**
Worst-link displacement after 1200 control steps at 240 Hz, effort ceiling
10 N·m, damping `0.1 * k`:

| | shoulder | upper_arm | lower_arm | wrist | gripper | gripper_frame |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| no servo configured | 0.0002 | 0.0177 | 0.0353 | 0.0733 | 0.0889 | 0.1643 |
| k = 20 N·m/rad | 0.0005 | 0.0181 | 0.0393 | 0.0759 | 0.0879 | 0.1222 |
| k = 200 | 0.0003 | 0.0173 | 0.0420 | 0.0801 | 0.0956 | 0.1327 |
| k = 2000 | 0.0002 | 0.0168 | 0.0421 | 0.0803 | 0.0937 | 0.1281 |

A gain of 2000 N·m/rad against an arm whose gravity torques are near 0.1 N·m
changes nothing. Read on a multibody build of the same arm, where joint angles
are legible, the joints configured to hold 0 rad settle at:

| joint | target | settled |
| --- | ---: | ---: |
| shoulder_link | 0.000 | 0.1460 |
| upper_arm_link | 0.000 | −0.0639 |
| lower_arm_link | 0.000 | 0.2508 |
| wrist_link | 0.000 | **−1.3145** |
| gripper_link | 0.000 | **1.5045** |

The target is correct and the joint is nowhere near it. The shoulder's
0.1460 rad is the same number an earlier investigation recorded as "0.147 rad"
and attributed to weak tracking; it is the arm hanging where gravity leaves it.

The error accumulating monotonically down the chain is a consequence of each
joint being free, not a per-joint servo droop: link displacement sums the
angular error of every joint above it.

**What is established:** the unit-explicit position actuation path produces no
usable torque on this scene, over gains spanning two orders of magnitude and
rates spanning sixteen. **What is not established:** why. An earlier revision of
this document proposed that the servo was holding a different angle from the one
being measured; the table above refutes that — the configured target is 0 and
the servo neither reaches nor defends it.

A related usability problem is established: the shipped SO-101 scene cannot
report its own joint angles at all. `named_joint_position` reads `JointState`,
which impulse joints do not carry, so every joint reads exactly 0.000 rad while
the links are visibly moving. An earlier revision of
`docs/MULTI_FLOOR_NAVIGATION.md` reported that reading as evidence the motors
were dead; it was an artifact of the accessor, but the motors are in fact
ineffective for the different reason above.

## Reproducing

```bash
cargo test --locked -p rne_ai --test so101_arm_control
```

`the_authored_pose_survives_the_first_step` fails if the joint frames and the
authored pose diverge again. `a_stiffer_servo_holds_the_pose_at_least_as_well`
fails if the servo returns to the unstable side of the boundary.
