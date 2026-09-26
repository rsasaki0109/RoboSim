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

### It is specific to this arm, and not to the rate

`mm_minimal` is the control: a two-joint arm whose URDF joint origins are all
identity. Same API, same gains, same rates. Worst link displacement after 1200
control steps:

| | no servo | k=20 | k=200 | k=2000 |
| --- | ---: | ---: | ---: | ---: |
| **so101** @ 240 Hz | 0.08888 | 0.08794 | 0.09560 | 0.09367 |
| **so101** @ 960 Hz | 0.08173 | 0.08439 | 0.08512 | 0.08420 |
| **so101** @ 1920 Hz | 0.08231 | 0.08218 | 0.08185 | 0.08221 |
| mm_minimal @ 240 Hz | 0.23367 | **0.00041** | 0.00033 | 0.00032 |
| mm_minimal @ 960 Hz | 0.99436 | **0.00033** | 0.00005 | 0.00002 |

On `mm_minimal` the servo improves the held pose by three orders of magnitude.
On SO-101 no cell differs from the no-servo column at any rate up to 1920 Hz.
So this is neither a gain problem, nor a timestep problem, nor a problem with
the actuation path in general.

### Hypotheses tried and refuted

The first was that the servo held a different angle from the one measured. The
settled-angle table above refutes it: the configured target is 0 rad and the
joint is as far as 1.5 rad from it.

The second was the obvious code-level suspect. Joint wiring composes the
authored joint-origin rotation into the parent frame only:

```rust
joint.data.local_frame1.rotation =
    quat_to_rapier(desc.relative_rotation) * joint.data.local_frame1.rotation;
```

SO-101 is the robot with non-identity origins and `mm_minimal` is not, so this
looked like the difference. It is not: `position_servo_follows_rotated_joint_origin`
in `rne_physics_rapier` drives a single revolute joint to 0.4 rad with a
90-degree joint origin and reaches it, exactly as it does with an identity
origin. That test is new — the path had coverage for direct effort under a
rotated origin but not for a position servo — and it passes.

### The authority is marginal rather than absent

"Indistinguishable from no servo" is measured for *holding* the authored pose.
Commanding is slightly different: driving all five joints to a target does bias
the outcome, just nowhere near enough to place the arm. Gripper displacement
after 1200 steps at 240 Hz, k = 200 N·m/rad:

| all joints commanded to | gripper moved |
| ---: | ---: |
| 0.00 rad (hold) | 0.09560 m |
| +0.50 rad | 0.09650 m |
| +1.00 rad | 0.10692 m |
| −1.00 rad | 0.19794 m |

Five joints turning a radian should move the gripper by tens of centimetres.

Direct effort is no better, which is the finding that rules out the servo
itself: with `configure_named_revolute_effort_actuation` at a 25 N·m ceiling,
gripper displacement is 0.09050 m at 0 N·m, 0.09817 m at 2 N·m and 0.11553 m at
20 N·m. Twenty newton-metres on a 0.1 kg arm moves it by 2.6 cm. Whatever the
fault is, it is upstream of the choice between position and effort control.

Ground contact is ruled out: with the base lifted 1 m and the ground plane
disabled, the gain sweep is 0.06456 / 0.06452 / 0.06363 m for no servo, k=20 and
k=200.

### A separate defect, now fixed: every link weighed 1 kg

The URDF declares 0.079 to 0.104 kg per link. The simulation used exactly
1.0000 kg for all of them — the `RigidBody` default, not a geometry-derived
value — so the arm massed about 5 kg instead of 0.5 kg.

`use_declared_inertial_masses = true` is the fix and could not be applied: the
import failed with `invalid inertial properties for link gripper_frame_link`.
That link declares 1e-9 kg and an all-zero inertia tensor, which CAD exporters
emit wherever a model needs a named coordinate frame. The importer now reads a
degenerate tensor on a link of a milligram or less as a frame marker and treats
its inertial as absent rather than refusing the whole robot; a link carrying
real mass with a malformed tensor still fails, because that is data the caller
meant and got wrong. The asset opts in, and every link now simulates at its
declared mass.

**It did not fix the servo.** With the arm at its true mass, a servo is *worse*
than no servo at every gain and rate measured:

| | no servo | k=2 | k=20 | k=200 |
| --- | ---: | ---: | ---: | ---: |
| 240 Hz | 0.10548 | 0.18154 | 0.18970 | 0.22174 |
| 960 Hz | 0.12387 | 0.17938 | 0.17636 | 0.19092 |

Raising the effort ceiling does not help either. Before the mass fix, with the
arm ten times too heavy:

| effort ceiling | hold drift | commanded −1 rad moved |
| ---: | ---: | ---: |
| 10 N·m | 0.09560 m | 0.19794 m |
| 50 N·m | 0.11918 m | 0.26496 m |
| 200 N·m | 0.19684 m | 0.39218 m |
| 1000 N·m | 0.22442 m | 0.28195 m |

More authority, more motion, worse holding — the same signature at every stage
of this investigation.

**What is established:** joint actuation of any kind — position or effort —
produces only marginal motion on the SO-101 scene, over gains spanning two
orders of magnitude, rates spanning thirty-two, with and without ground
contact, and at both the wrong mass and the right one, while an identical call
on a control arm improves the held pose by three orders of magnitude.

**What is not established:** why. Six mechanisms have suggested themselves and
all six have been measured and ruled out: a frame mismatch between the servo's
target and the reading; the parent-frame-only joint-origin composition; ground
contact; a saturating effort ceiling; the ten-times-too-heavy links; and the
arm's own collision geometry. The mass was a real defect and is fixed, but it
was not this one.

### The arm has been resting on itself

The sixth attempt found the control arm's one clear structural difference:
SO-101 carries 34 mesh elements and `mm_minimal` carries none, so SO-101's
collision geometry is 34 AABBs approximating meshes while the control's is
primitives. Overlapping boxes on adjacent links would give the solver a
permanent penetration to chew on, which could plausibly swamp a joint torque.

Turning self-collision off makes the drift **worse**, not better, and leaves
the servo exactly as ineffective:

| | no servo | k=2 | k=20 |
| --- | ---: | ---: | ---: |
| self-collision on | 0.08888 | 0.09309 | 0.08794 |
| self-collision off | 0.32985 | 0.32918 | 0.35325 |

That refutes the mechanism and establishes something more useful about every
other measurement in this document: **the 0.089 m figure is partly the arm
jamming on its own colliders.** With them removed it falls three times as far.
The SO-101 arm has not been holding a pose, resting in a gravity equilibrium,
or being held by a servo — it has been propped up by its own approximated
collision geometry.

Note that removing the colliders entirely is not a usable comparison: every
SO-101 collision element is a mesh, so `mesh_collisions = false` takes the arm
out of physics altogether and it reads 0.00000 m of drift with no servo at all.

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
