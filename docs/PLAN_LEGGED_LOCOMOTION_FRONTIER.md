# Plan: legged locomotion frontier

Status: measured boundary, campaign next

This plan records where the official Unitree Go2 and G1 locomotion actually
stop, why parameter search does not move the wall, and the concrete campaign
that could. It is written after a measurement pass, so every "blocked" claim
below is an observed result, not an assumption. Detailed evidence lives in
[GO2_LOCOMOTION.md](GO2_LOCOMOTION.md) and [G1_LOCOMOTION.md](G1_LOCOMOTION.md).

## What is done

- **Go2 learning boundary.** The learned turn, sprint, and schedule searches,
  the torque pathway, and the aerial-duty test are all complete; the walkable
  schedule plateau is pinned by tests and examples 52–65.
- **G1 long-horizon stability (v0.3).** The validated heading candidate walks
  3000 ticks (50 s) without falling, upright, with the correct mean yaw-rate
  sign and a bounded integrated yaw. Pinned by
  `v03_sustained_envelope_walks_50s_without_falling`; hero-captured by example
  92 (`docs/media/unitree-g1-sustained-walk.gif`).

## What is blocked, and the measured reason

| Goal | Wall | Measurement |
|---|---|---|
| Go2 parkour | Foot clearance is kinematic | Swing foot tops out at 2.1 cm for every tested stride/lift/overlay; a 4 cm step topples the walk |
| Go2 steering | Contact schedule / morphology | Three search spaces and a 5× torque scan plateau at ~0.02 rad/s |
| G1 sustained turn | Contact schedule | 8-dim schedule CEM (18×40, both directions, 25 s): **no upright candidate**; the best turn 267–811° and fall |
| G1 higher speed | 60 Hz solver stability | Any forward command above the pinned 0.0276 m/s blows the solver up into NaNs |
| G1 long-horizon disturbance | 60 Hz solver stability | A pelvis disturbance over a long horizon also blows up |

The common thread: the walls are **contact schedules and solver stability
margin**, not gains. More CEM over the same knobs will keep returning the same
plateau.

## Campaign

One theme is active at a time; each advances only on its machine-readable exit
evidence.

### Theme A — 60 Hz solver stability margin — PARTIALLY TESTED, OPEN

Hypothesis: the NaN onset above the pinned `0.0276 m/s` forward command is a
60 Hz fixed-step artifact.

Result so far: **not the simple version.** At the fixed 60 Hz step the fastest
stable command is `~0.030 m/s` (within 9% of the pin); `0.034/0.040` reach
NaNs. A `0.03 rad` disturbance falls at the pinned command. `0.5×`/`0.25×`
proximal torque-PD stiffness does not stabilize and only diverges.

A naive integer-tick substep of the legacy acceleration-based `JointMotor`
gains destabilizes even the nominal walk (NaNs from two substeps up), so it is
not a valid test of a higher-rate plant. The right test is a dedicated
fixed-delta plant (e.g. `from_scene_path_with_solver_iterations_and_fixed_delta`
at 240 Hz) with its gains re-derived for that rate; that remains open and is
not the current priority. Evidence: `G1_LOCOMOTION.md`, "The solver-margin
hypothesis".

### Theme B — contact-schedule redesign above the joint targets (active)

Introduce a foot-placement layer (swing timing and trajectory planning) above
the current joint-target gait, so the schedule is a free variable rather than a
fixed function of phase. This is the lever Theme A ruled out in favour of.

- Define a backend-neutral `LeggedFootPlacement` schedule (per-leg contact
  timing, swing apex height, touchdown offset), mirroring the Go2
  `UnitreeGo2GaitSchedule` structure but for a biped.
- Search swing apex and touchdown offset against the anti-cheat
  late-window integrated-yaw objective, with the upright gate.
- Port the same layer to the Go2 to attack foot clearance for parkour
  (Theme C).

Exit: at least one upright sustained turn (both directions, ≥ 0.02 rad/s mean
for 20 s, bounded height/tilt) or a documented, searched negative; for the Go2,
a gait that clears a 6 cm step.

### Theme C — Go2 foot-clearance gait

Once swing timing is a free variable, search a high-clearance trot: swing apex
and knee/hip trajectory plus schedule duty, scored on step clearance with an
upright, straightness, and torque-ceiling gate. A parkour hero capture follows
only after the headless gate passes.

Exit: a headless 6 cm step crossing with no fall, exact replay, and a
reproducible `--train` search; then the wgpu hero.

## Cross-cutting rules

- Simulation decisions use `SimClock`, explicit seeds, and deterministic
  ordering; replay tests compare stable hashes.
- Every campaign candidate is scored by the median of ULP-perturbed replays
  (the chaos-floor discipline the Go2 campaign established).
- A failed candidate may panic the solver; the search must catch-unwind and
  score it at the floor.
- No physical-accuracy claim is made from a renderer-only demo; the headless
  gate is authoritative.

## Immediate next step

Theme B. Concretely: define a backend-neutral `LeggedFootPlacement` schedule
(per-leg contact timing, swing apex height, touchdown offset) and the function
that maps it plus phase to the existing joint targets, then reproduce the v0.3
walk through the new layer before giving the schedule to a search. The
reproduction gate is: the v0.3 candidate's 50 s metrics are unchanged when the
foot-placement layer is set to the current schedule. Only then does the swing
apex become a searched variable.

### Theme D — joint-space RL locomotion (started)

The root cause of the G1 shuffle is the action space: the episode exposed three
gait parameters on a scripted stepper, so no policy could step. Theme D
replaces it with the OSS joint-space contract.

- **Done:** `rne.unitree_g1.joint_locomotion.v1` (12 leg-joint residual action,
  46-dim OSS-style observation, velocity-tracking/air-time reward), the pyo3
  binding, and a Gymnasium + Stable-Baselines3 PPO example
  (`examples/93_g1_joint_locomotion_rl`).
- **Boundary:** from-scratch PPO is throughput-bound at ~750 steps/s on the
  Python `SubprocVecEnv` path (single-thread reset reloads the scene); the
  nominal gait trade-off and the balance-controller plateau are recorded in
  `G1_LOCOMOTION.md`.
- **Done:** `VectorizedUnitreeG1JointLocomotionEnv`, a `std::thread`-parallel
  batch that steps every environment in one Rust call with optional action
  repeat, exposed as `rne_py.UnitreeG1JointBatch` and wrapped as an SB3
  `VecEnv` (`examples/93_g1_joint_locomotion_rl/native_vec_env.py`).
- **Boundary:** the native batch reaches ~2700 physics ticks/s in a
  microbenchmark, but end-to-end SB3 PPO is `~262` env-steps/s (`~1050`
  ticks/s) because the **CPU policy update dominates**. A 20M-step run is
  ~20 hours on this 8-core host.
- **Next:** a GPU PPO path (or a Rust-native optimizer over a low-dimensional
  phase-conditioned policy) is required before a 100M-step campaign is
  practical. This is the critical path to genuine walking.

### Theme E — open-loop dynamic push-off probe (measured negative)

`examples/106_go2_jump` tests whether a scripted crouch plus a saturated
knee-extension torque can make the Go2 leave the ground. It is a headless
measurement with `--scan` (torque sweep) and `--trace` (per-step telemetry).

- **Result:** with the knees held and the hips/thighs free, a +23.7 N·m
  knee-extension burst extends the stance by ~0.21 m while the lowest foot stays
  planted at ~0.02 m; `airborne_steps` is zero for every scanned combination.
  Adding thigh torque reaches a higher apex (up to +0.45 m) but as a pitch-up
  rear on the planted feet, not a hop, and the body still returns upright.
- **Conclusion:** a true liftoff needs a closed-loop, contact-force-driven
  push-off (and a flight-phase controller), not an open-loop torque profile.
  Consistent with the base contact-schedule/closing wall, this keeps Theme B
  (foot placement) and a whole-body jump trajectory as the prerequisites for
  any parkour capture.
