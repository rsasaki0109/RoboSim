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
| body-level impulse / one-step force | no effect | Rapier's multibody solver does not consume body-level external forces on articulated links |
| root **translation** | no disturbance | forward kinematics moves the whole tree — feet included — so the contact configuration is unchanged and nothing has to recover |

Each failure mode produced bit-identical tilt across disturbance magnitudes, which is
how they were caught: a disturbance whose size does not matter is not being applied.

## What works: rotation

`UrdfSceneSim::tilt_named_body_rad(name, axis_angle)` rotates a root body about a
world-frame axis, preserving velocities. Unlike a translation, a rotation changes the
**contact configuration**: feet on one side press into the ground, the other side
lifts, and gravity plus the contact solver produce genuine tipping dynamics the
controller must catch. A 0.4 rad tilt registers in full on the attitude observation.

`UnitreeGo2Push { step, roll_tilt_rad }` schedules one such tilt at an explicit
episode step, about the body X axis.

Two velocity-preserving primitives from the same investigation remain available for
plain (non-articulated) dynamic bodies: `RapierBackend::apply_velocity_impulse`
(a one-step force of `mass * delta_v / dt`, auto-cleared after the step) and
`UrdfSceneSim::displace_named_body_m`.

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

## What the acceptance test pins

`pushed_trot_registers_recovers_and_feedback_reduces_peak_lean`:

- a 0.4 rad shove registers in full on the attitude observation;
- the scripted open-loop trot **recovers by itself** — stiff position motors and a
  wide stance make it passively stable against instantaneous tilts, which is an
  honest property of this plant, not a staged save;
- correctly signed hip feedback strictly reduces the peak lean, and the inverted
  sign strictly increases it;
- runs are bit-identical on repeat.

## Where this goes next

Instantaneous tilts cannot topple this plant. A controller-versus-fall demonstration
needs disturbances that outlast the passive recovery — sustained pushes across
several gait cycles, ground-plane perturbation, or a compliant (lower-stiffness)
gait whose stability genuinely depends on feedback. That is the planned entry point
for the legged-locomotion arc.
