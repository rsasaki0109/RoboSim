# Disturbance injection

Evaluating a balance or locomotion controller requires shoving the robot, and shoving
an articulated robot is less obvious than it sounds. This documents the disturbance
primitives RNE provides, and — because they were expensive to learn — the mechanisms
that measurably do **not** work on a Rapier multibody.

## What does not work, measured

Three obvious mechanisms were implemented and rejected against the dynamic Unitree
Go2 (probes live in `unitree_go2_episode.rs`):

| mechanism | result | why |
| --- | --- | --- |
| ECS velocity write on a link | no effect | multibody link velocities derive from the articulation's generalized joint state; the next solve recomputes them |
| root **translation** | no disturbance | forward kinematics moves the whole tree — feet included — so the contact configuration is unchanged and nothing has to recover |

Each failure mode produced bit-identical tilt across disturbance magnitudes, which is
how they were caught: a disturbance whose size does not matter is not being applied.

### Retracted (2026-09-25): "body-level forces do not reach multibody links"

This table previously also listed *body-level impulse / one-step force* as having
no effect, explained as "Rapier's multibody solver does not consume body-level
external forces on articulated links". **That explanation is wrong and the row is
withdrawn.** Rapier's multibody solver does consume them: it projects each link's
body force and torque onto the generalized coordinates through the body Jacobian
(`third_party/rapier3d/.../multibody.rs`, where `rb.forces` enters
`accelerations.gemv_tr(.., body_jacobians[i], ..)`).

The measurement that replaces it is
`external_wrench_drives_reduced_coordinate_multibody_links`: a one-link pendulum
under a 10 N lateral wrench swings 0.157 m as a reduced-coordinate multibody and
0.181 m as an impulse-joint chain, against exactly 0.000 m for both with no
force, and doubling the force increases the response. Whatever produced the
original negative result, it was not the solver discarding the force.

## What works: rotation

`UrdfSceneSim::tilt_named_body_rad(name, axis_angle)` rotates a root body about a
world-frame axis, preserving velocities. Unlike a translation, a rotation changes the
**contact configuration**: feet on one side press into the ground, the other side
lifts, and gravity plus the contact solver produce genuine tipping dynamics the
controller must catch. A 0.4 rad tilt registers in full on the attitude observation.

`UnitreeGo2Push { step, roll_tilt_rad, duration_steps }` schedules one such tilt
at an explicit episode step, about the body X axis. With `duration_steps > 1` the
total tilt spreads evenly across consecutive steps — a *sustained* lean rather
than a slap. Measured caveat: a sustained injection below roughly the plant's
own recovery rate simply never accumulates (0.9 rad spread over 90 steps at
60 Hz produced *less* peak tilt than undisturbed trotting); a push that is meant
to matter must beat the recovery rate, e.g. the same 0.9 rad within 15 steps.

Two velocity-preserving primitives from the same investigation remain available for
plain (non-articulated) dynamic bodies: `RapierBackend::apply_velocity_impulse`
(a one-step force of `mass * delta_v / dt`, auto-cleared after the step) and
`UrdfSceneSim::displace_named_body_m`.

## What also works: a real external wrench

`UrdfSceneSim::apply_named_link_wrench(link, point_world_m, force_world_n,
torque_world_nm)` queues a world-frame wrench on any dynamic link, including a
reduced-coordinate articulation link, for exactly the next step. Unlike a
rotation, nothing is re-posed: the force enters the solver and is opposed by
contact friction, inertia and actuation, and a push applied away from the link's
center of mass induces the matching moment. A sustained shove is queued once per
step, and queued wrenches are applied in request order so a replay reproduces
them.

This is the preferred primitive when the disturbance should be a *force* with a
magnitude in newtons. The rotation primitive above remains useful when the
intent is to set a contact configuration directly rather than to apply a load.

Example 118 sweeps it on the pinned Go2 standing pose, with a 12-step lateral
trunk push and 4 s of recovery:

| push | peak tilt | residual tilt | lateral displacement | recovers |
| --- | --- | --- | --- | --- |
| 0 N | 0.009 rad | 0.009 rad | +0.001 m | yes |
| 40 N | 0.016 rad | 0.009 rad | +0.002 m | yes |
| 80 N | 0.073 rad | 0.052 rad | +0.018 m | yes |
| 120 N | 0.463 rad | 0.081 rad | +0.005 m | yes |
| 160 N | 3.089 rad | 3.076 rad | +0.593 m | no |
| 200 N | 2.746 rad | 2.722 rad | +0.640 m | no |

The pinned pose recovers up to 120 N and is flipped by 160 N. Peak tilt is
ordered in the push force only while the robot stays on its feet; past the
tipping point the body rotates through large angles and settles wherever it
lands, so tilt there is a fall indicator rather than a measure of push strength.
The 40 N case replays bit-identically. This measures the *existing* pinned pose,
not a push-recovery controller: no controller was added or retuned.

## The measured Go2 actuation map

Feedback pairing was fixed empirically rather than from axis conventions
(`axis_derivation_probe` pins it in CI):

| input | rel_roll | rel_pitch |
| --- | --- | --- |
| body-X tilt (the push) | ~0 | **+0.138** per 0.15 applied |
| body-Z tilt | **+0.149** per 0.15 | ~0 |
| uniform hip abduction +0.2 | ~0 | **−0.145** |
| uniform thigh offset +0.2 | ~0 | ~0 |
| front/back differential thigh ±0.15 | ~0 | ~0 |

So the push axis and the hip-abduction axis coincide (the observation labels that
axis *pitch*; physically it is the lateral lean), a positive hip command drives the
reading negative — hence a positive feedback gain opposes the lean — and no leg
pattern actuates the orthogonal axis at all. Mirrored hip signs merely widen the
stance, which stabilizes nothing; the correction therefore applies the **same** angle
to all four hips.

## What the acceptance tests pin

`pushed_trot_registers_recovers_and_feedback_reduces_peak_lean` (stiff motors):

- a 0.4 rad shove registers in full on the attitude observation;
- the scripted open-loop trot **recovers by itself** — stiff position motors and a
  wide stance make it passively stable against instantaneous tilts, which is an
  honest property of this plant, not a staged save;
- correctly signed hip feedback strictly reduces the peak lean, and the inverted
  sign strictly increases it;
- runs are bit-identical on repeat.

`sustained_push_topples_weak_motor_trot_and_two_channel_feedback_saves_it` (the
fall-versus-save scenario, rendered by `examples/52_go2_fall_vs_save`):

- on 8 N·m torque-limited motors, a 1.8 rad flank push spread over 20 steps
  topples the open-loop trot — it crosses the 1.2 rad termination tilt and ends
  flat on its side (final tilt > 1.3 rad);
- two-channel lean feedback — hip abduction (`1.6·lean + 6·lean_rate`) plus
  differential leg-length extension (`−(2.5·lean + 5·lean_rate)` on the calves)
  — holds the peak lean under a third of the open-loop excursion and ends
  **standing at full height**;
- the inverted feedback sign does not save the robot;
- runs are bit-identical on repeat.

Why two channels: the correction-versus-lean equilibrium map of the hip channel
is monotonic but saturates — hip correction 0.34 props the body at 1.14 rad,
0.5 at 0.97, and the full ±0.8 rad clamp at 0.67 — so hip abduction alone can
only *brace* a toppling push in a deep propped lean, and pure damping injection
saves nothing at all. The measured way out is a second, independent authority
on the same lean axis: left/right differential calf extension ("push up with
the downhill legs", ±0.4 rad differential ≈ 0.25 rad of lean, pinned by
`axis_derivation_probe`). Stacked, the two channels cap the peak lean inside
the passive capture region and the trot walks itself back upright.

## Motor compliance: what the stiffness knob actually does

Making the gait compliant enough to be toppleable exposed a Rapier property that
cost a full measurement campaign: joint motors default to
`MotorModel::AccelerationBased`, whose effective position gain is
`1 / (dt + damping/stiffness)`. Absolute stiffness cancels out — stiffness
180/damping 18 and stiffness 0.5/damping 0.05 are **bit-identical**, and a
"compliance scan" that keeps the ratio fixed is a no-op (the tell, once again:
identical results across the whole scan). The two real knobs are the
`damping/stiffness` ratio and `max_force`; of the two, only the torque limit
produces honest weak-actuator dynamics, which is why
`UnitreeGo2EpisodeConfig::motor_max_force_n` is the compliance lever the
fall-versus-save setup uses.

## Where this goes next

The measured saturation boundary makes the next step concrete: a recovery
*step* — swing a leg toward the fall to move the support polygon under the
body — is the only strategy that can turn the brace into an upright recovery.
[GO2_LOCOMOTION.md](GO2_LOCOMOTION.md) measures the flip side: a *walking*
trot already does this implicitly (it shrugs off the push that topples the
slow trot with no controller at all), and the same campaign shows scripted
joint-space gaits cannot steer this 3-DoF-per-leg platform — both point at
learned or model-based gaits. Ground-plane perturbation remains the other open
disturbance channel.
