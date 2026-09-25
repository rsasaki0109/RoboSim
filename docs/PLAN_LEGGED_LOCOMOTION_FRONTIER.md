# Plan: legged locomotion frontier

**Native backflip refinement passed (2026-09-22):** the same non-RL G1 controller completes a full native RoboSim/Rapier backflip, feet-only landing and 15-second recovery at 0.125 and 0.0625 ms (0.5 ms also passes), with unchanged physical gates. The [successful native GIF and verification bundle](evidence/g1-contact-backflip/native-transfer/selected005-long-validation/README.md) are saved. Hardware remains unvalidated; full CI is still under review.

**Contact-plant milestone (2026-09-21):** the optional
[G1 non-RL optimization benchmark](G1_CONTACT_BACKFLIP.md) now completes a
physics-simulated backflip in MuJoCo, with feet-only landing and stable standing,
at both 0.125 ms and 0.0625 ms. The checked 38.13 kg model uses bounded joint
motors and no base wrench. A recorded GIF and replay-hash regression accompany
the evidence. This is separate from the native solver/transcription results
below; RNE/Rapier and hardware transfer remain unvalidated. The benchmark
contract documents motor assumptions, soft-stop tolerances and self-collision.


Status: measured boundary, campaign next

**Current backflip direction: non-RL trajectory optimization.** The active
next step is the [external G1 direct-transcription comparison](G1_TRAJOPT_REFERENCE.md):
standing, jump/landing, then backflip, using the repository's 23-actuated-joint
URDF and explicit comparison manifests. External Pinocchio/IPOPT dependencies
stay in an optional offline Python environment. Historical solver plateaus below
are not a proof of physical infeasibility; references and contact conditions
must match before their defect values can be compared. Example 113's `nv = 29`
means 23 actuated joints plus 6 base velocities.

**First non-RL comparison result (2026-09-21).** The external lifted
direct-transcription solve produces a 0.132 m jump candidate and a full-rotation
backflip candidate with a 4.26e-5 constraint residual. A new native
inverse-dynamics check exposed a 4 kg missing-inertia-policy mismatch (four
sensor frames each acquire 1 kg in RNE); after matching that policy, native and
external joint torques agree below 5e-13 N m on the saved trajectories. The
coarse backflip still fails the new flight-conservation screen: about 48%
relative linear-momentum error despite the small NLP residual. Thus a
momentum-consistent integration/mesh-refinement comparison is the next lever,
before plant tracking or any physical-success claim. See the
[evidence](evidence/g1-trajopt-reference/README.md); historical probes below
retain their original conditions and are not directly comparable scores.

**Momentum-based follow-up (2026-09-21).** A centroidal impulse/CoM integration
and feasibility-only solve now yield a 50 ms backflip transcription passing
all existing gates (6.33e-10 constraint violation, flight momentum error near
roundoff, native inverse-dynamics agreement). Independent ABA free-flight
replay exposes a 2.07 rad open-loop joint error. Torque-limited joint PD reduces
that endpoint error to about 0.054 rad at sufficiently small numerical steps.
The two 30 ms refinement attempts did not converge within 180 s; mesh convergence,
contact replay, and stable landing remain open. This is not yet a simulated
backflip success. See the same evidence directory for positive and negative runs.

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
- **Scripted-locomotion CLI and CI gates.** Examples 110 (`go2_walk`) and 111
  (`g1_walk`) expose the two scripted walks as runnable, headless gates. The Go2
  forward-trot regression
  (`official_unitree_go2_dynamic_trot_walks_forward_without_falling`) is now a
  600-step forward/straightness/determinism contract rather than a short
  upright-only smoke, and both examples join the headless smoke set.

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
fixed-delta plant (e.g., `from_scene_path_with_solver_iterations_and_fixed_delta`
at 240 Hz) with its gains re-derived for that rate; that remains open and is
not the current priority. Evidence: `G1_LOCOMOTION.md`, "The solver-margin
hypothesis".

**First result on the 240 Hz plant (example 112).** `go2_wbc_stance` loads a
declared-inertial-mass Go2 scene (so the WBC model and the plant share the same
masses) and drives it with `rne_wbc` torques. Findings:

- At 60 Hz every configuration collapses within 5 s, including a pure gravity
  hold, even though the same robot is stable under the position motors.
- `rne_dynamics` is verified consistent for the body-twist base velocity:
  `floating_base_link_motions_match_frame_jacobian` checks `frame_jacobian * qd`
  and `com_jacobian * qd` against `link_motions`, and
  `free_floating_body_matches_newton_euler` checks `forward_dynamics` against the
  analytic free-body equations.
- **Bug found and fixed in `rne_dynamics`.** A new central-difference test,
  `floating_base_point_bias_acceleration_matches_finite_difference`, showed that
  `link_motions` was missing the frame-rotation term `omega_body x v_body` when
  converting the spatial link acceleration to the classical world acceleration.
  This term is zero only when the link is not moving, so it had been invisible
  in the static WBC tests and in the `qd` base-zero stance. It directly feeds
  `LinkMotion::point_bias_acceleration_m_s2`, and therefore the WBC contact and
  CoM bias terms.
- **Impact.** Before the fix, a posture-only 240 Hz WBC appeared to hold an
  upright stance; that stability was an artifact of the missing term. With the
  corrected bias the same solve diverges within the run, faster and regardless
  of posture gains or solver regularization. The earlier "240 Hz stable stance"
  result is retracted: the WBC was tuned against buggy dynamics.

- **The WBC core is validated on the corrected model.** A new `rne_wbc` test,
  `solution_matches_constrained_forward_dynamics_with_base_velocity`, shows that
  with no task competing the weighted-least-squares solve reproduces
  `rne_dynamics::constrained_forward_dynamics` acceleration exactly, including a
  nonzero base twist. The formulation is self-consistent, so the remaining
  divergence is a model-vs-plant gap.
- **Remaining gap: the contact model.** The WBC assumes rigid point contacts at
  `foot_link + (0, 0, -0.02)`, while the plant is Rapier's soft/penalty solver on
  a radius-0.022 foot sphere. Under motion those are not the same contact, so the
  planned wrenches are not realised.
- **Opt-in contact compliance helps but does not close it.** `WholeBodyConfig`
  gained `contact_compliance` (`J qdd + bias = compliance * f`), pinned by
  `contact_compliance_relaxes_the_no_slip_constraint`. On the Go2 stance it cuts
  the worst tilt from `2.88` to `1.27` rad at `compliance ~1e-4` (posture-only),
  which is the first mechanism so far that moves the needle, but it only delays
  the fall: the stance is still down by 5 s. Adding a CoM or attitude task, or
  more joint-velocity damping, makes it worse again. Bandwidth knobs (posture
  gain `1..100`, torque filter `0..0.95`, `contact_weight 1e6..1`) do not help.
- The next step is to feed the plant's measured contact wrenches back into the
  WBC (which needs a directional contact-force API; the current
  `link_contact_impulse_ns` is scalar) or to fit the compliance and damping to
  Rapier's contact model, before retrying CoM/attitude tasks and the contact
  schedule.

### Theme A.1 — WBC stance on the corrected model (active)

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

**Ground prerequisite delivered (2026-09-25).** Theme C previously had no
ground to cross: every scene stood on a flat `ColliderShape::Plane` or a large
cuboid, so a "6 cm step" could only be a separately spawned box.
`rne_physics::HeightfieldCollider` ([ADR 033](adr/033-sampled-heightfield-terrain.md))
adds a sampled terrain patch behind the backend-neutral contract, with the
Rapier implementation, the pinned row-along-X axis convention, and example 116
checking reported contact heights against the analytic surface they were
sampled from.

**First measurement on terrain (2026-09-25), and a retraction.** Example 117
sweeps the pinned flat-ground Go2 standing pose (the gains and targets of
`official_unitree_go2_dynamic_multibody_stands_on_four_feet`, unchanged) across
heightfield slopes. An earlier revision of this section published a six-row
table claiming the stance held to 5 deg and lost foot contact at 10 deg. **That
table is withdrawn**: it was produced at the scene's default 60 Hz, where the
Go2's foot links tunnel through the open terrain surface and fall past -4 m, so
every row from 10 deg up measured a lost body rather than a stance.

Re-measured at 240 Hz with an explicit tunnelling check, only two rows are valid:

| slope | base clearance | deviation from terrain-parallel | stability margin | loaded feet | stands |
|---|---|---|---|---|---|
| 0 deg | 0.228 m | 0.002 rad | +0.158 m | 4 | yes |
| 5 deg | 0.229 m | 0.086 rad | +0.299 m | 2 | no |
| 10-25 deg | — | — | — | — | invalid: a body tunnelled through the surface |

So the honest result is narrower than first reported: the pinned pose already
loses half its feet at 5 deg, and slopes at or above 10 deg **cannot currently
be measured at all** with this robot on this terrain representation.

The stability margin is the signed distance from the ground-projected center of
mass to the boundary of the measured support polygon
(`rne_legged::SupportPolygon`, built from the solver's own contact points). It
stays positive in both valid rows, so the 5 deg row is a robot standing on two
feet and its calves rather than a robot about to tip.

**The blocker for terrain locomotion is the terrain representation, not the
controller.** A heightfield is an open surface with no volume: a penetration is
never resolved, so a body that tunnels falls forever. A finer step bounds
per-step penetration but only delays the event over a long run, and Parry's
`FIX_INTERNAL_EDGES` was tried and did not prevent it. Before any legged
controller is evaluated on terrain, the ground needs a representation a foot
cannot fall out of — a solid swept volume beneath the sampled surface, thicker
foot collision geometry, or continuous collision detection, none of which the
physics contract currently exposes. The 2.1 cm swing-height plateau therefore
remains unmeasured against sloped or rough ground.

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
- **Follow-up scan:** shallower crouches (thigh 0.95 / calf -1.75) reach a
  +0.27 m apex with only 0.21 rad tilt, and a front/rear differential thigh
  torque on the measured pitch holds tilt down to 0.16 rad, but the lowest
  foot still stays planted in every combination: the stance simply extends to
  full leg length, so there is no surplus upward momentum. The next attempt
  must command a *force* (centroidal) profile rather than a joint torque and
  map it through the leg Jacobian with an attitude task.
- **Conclusion:** a true liftoff needs a closed-loop, contact-force-driven
  push-off (and a flight-phase controller), not an open-loop torque profile.
  Consistent with the base contact-schedule/closing wall, this keeps Theme B
  (foot placement) and a whole-body jump trajectory as the prerequisites for
  any parkour capture.

### Theme F — 3D acrobatics and the floating-base chart (measured)

Requested target: a Unitree G1 backflip. Literature review: Unitree's
**OmniXtreme** (flow-matching pretraining plus actuation-aware
post-training) reports 96.36% over seven G1 Flip-category motions and 55 trials
([Table III](https://arxiv.org/html/2602.23843v1)), not backflips alone. Its public
release includes checkpoints and sim-to-sim evaluation, not the full training
pipeline. It is background evidence, not the selected non-RL approach. Trajectory
optimization work (Konishi et al., IROS 2025; `se3_trajopt`; Crocoddyl-based
`humanoid-trajopt-playground`) solves whole-body takeoff/flight/landing. Both
rely on a singularity-free floating-base representation and a flight-phase
controller.

- **A sagittal backflip maps to a regular Euler coordinate here.** The floating
  base composes as `base_world = R_euler * R_x(-90)`, so a rotation about the
  world lateral axis (a backflip) is a change of the base **yaw**, where the
  Euler-rate map is regular. (An earlier note claiming a backflip hits the
  `pitch = ±pi/2` singularity was wrong: it assumed the root on the other side
  of the composition.)
- **The OC floating-base chart was inconsistent (fixed).** `rne_oc` integrated
  the body twist as Euler rates (`q += qd * dt`); `rne_dynamics::integrate_configuration`
  now maps the twist through `base_velocity_map` first, and both integrators use
  it. This is the prerequisite for any large-rotation plan.
- **Whole-body FDDP is closer but still does not converge.** The analytic
  dynamics derivatives are now implemented and verified in `rne_dynamics`:
  `link_pose_derivatives`, `link_pose_body_twist_derivatives`,
  `xup_derivatives`, `mass_matrix_gradient` (`dM/dq`),
  `non_linear_effects_gradient` (`dh/dq`, `dh/dqd`), and
  `forward_dynamics_gradient` (`d(qdd)/d(q, qd, tau)`). `rne_oc` uses them
  through the `analytic_derivatives` hook instead of central differences.
  - **Second-order extension.** `xup_hessian` (`d²(xup)/dq²`) and
    `mass_matrix_hessian` (`d²M/dq²`) are exact and verified against central
    differences on a fixed and a floating chain. The velocity Hessian
    `d²h/dqd²` is *not* the Christoffel combination
    `dM_aj/dq_i + dM_ai/dq_j - dM_ij/dq_a`: that identity holds for holonomic
    (chart) velocities, but the floating base uses the body twist, a
    quasi-velocity, so the base block needs the SE(3) structure-constant terms.
    A Christoffel-only version matched a fixed chain but returned 0 where the
    floating chain has `-4`, so it was reverted. `non_linear_effects_hessian`
    therefore differentiates the verified analytic gradient with Richardson
    extrapolation, and `forward_dynamics_hessian` assembles `d²qdd/dx²` from it
    with the `M^-1` product rule. Both are FD-verified. The Hessian is now wired
    into the multiple-shooting local step through
    `MultipleShootingConfig::use_exact_hessian` and the
    `analytic_hessian` hook, and `ArticulatedDynamics` implements it; a pendulum
    warm start converges on the exact-Hessian path. Two gaps remain before it
    can be measured on the backflip: (1) the articulated Hessian differentiates
    the analytic first derivatives, so it costs `O(nx + nu)` gradient
    evaluations per node — a single-pass analytic Hessian is needed for the
    29-DoF G1; and (2) `ContactSequenceDynamics` and
    `ContactImplicitArticulatedDynamics` do not implement `analytic_hessian`
    yet, so the contact problems still fall back to Gauss-Newton.
  - **Compliant contact derivatives are now cheap.** The compliant model
    differentiates its smooth dynamics analytically and its contact force by
    central differences of the force alone, which removes the whole-step finite
    differences and makes a 600-sweep compliant G1 solve take about five minutes
    (was far slower). The remaining compliant plateau is unchanged: at
    `k=1000, c=0, n=2` the worst defect is 0.29.
  - **Freeing the first control does not help.** The forward-only sweep leaves
    `u[0]` frozen, so the node-0 defect is fixed by the warm start and was the
    worst defect at 0.29. Adding a dedicated initial-control step (state fixed,
    control optimized against the first defect) moved the worst defect to node 6
    but regressed the trajectory badly — cost 10988 and no jump, against 467 and
    a 0.22 m jump — because the node-0 equation alone can drive `u[0]` into a
    degenerate push. It was reverted. Coupling it to a whole-trajectory
    augmented-Lagrangian merit acceptance (with failed roll-outs scored at
    infinity, fixing the earlier under-count) also diverged — gap 9e10, cost
    6e13 — so both were reverted again. The     defect-based acceptance and the
    frozen first control stay as they are; a robust first-control update remains
    open.
  - **The solver plateau is not a local-step deficiency.** Every variant tried
    against the compliant G1 warm start (at `k=1000, c=0, n=2`, baseline gap
    0.293, cost 467, 0.22 m jump at 150 sweeps) was worse: a whole-trajectory
    augmented-Lagrangian merit acceptance (gap 0.257 but cost 17790 and no
    jump), a trust region of 1.0/0.3/0.1 (gap 0.35-0.57), a symmetric sweep
    (gap 0.257 at a lower cost), and a freed first control (cost 10988, no
    jump). The baseline Gauss-Newton local step with the max-defect acceptance
    and the frozen first control is the best across all of them. This points at
    genuine infeasibility of the rigid/compliant contact model at the
    push-to-flight transition rather than a step-size problem, so the next
    credible route is a different contact formulation (complementarity at a
    small or variable step), not more acceptance tuning.
  - **Hard complementarity contact does not move the plateau either.**
    `ComplementarityContactDynamics` (velocity-level sequential impulse, no
    penetration) run on the same G1 warm start reaches gap 0.295 at 100 sweeps
    against 0.293 for the compliant penalty at 150, but halves the deepest
    penetration (0.066 m vs 0.123 m). Two very different contact laws landing on
    the same defect says the plateau is the planning/solver structure at the
    push-to-flight transition, not the contact stiffness. The next lever is the
    transition treatment itself (e.g. a contact-consistent reference or a
    dedicated transition phase), not the contact model.
  - **Reference shaping at the transition does not lower the plateau.** The
    compliant G1 warm start zeroed the base vertical velocity at every node,
    discarding the push launch velocity. Deriving the flight vertical velocity
    from the height profile lowers the raw gap to 0.269 but only by trading away
    the jump (cost 467 -> 915, jump 0.222 -> 0.151); adding a push-phase base
    height ramp back to the standing height restores a proper jump (cost 471,
    jump 0.227) and the gap returns to 0.294. So the 0.269 was a degenerate
    low-jump solution, not progress. The physically consistent reference is kept,
    but the plateau is unchanged at ~0.29 and is bit-identical across sweep
    counts. Combined with the complementarity result, this says the residual
    defect is a structural property of the single-shooting reference and rigid
    contact at 30 ms, and needs either a variable/smaller step or a
    contact-consistent trajectory (e.g. optimize the contact schedule), not more
    reference tuning.
  - **Sub-stepping the hard contact does not help the plateau.** The
    complementarity model gained sub-stepping (smaller internal impulse steps).
    At 60 sweeps, one sub-step reaches gap 0.525 while three sub-steps reach
    1.196, so the stiffer sub-stepped model is harder for the
    finite-difference-derivative shooting solver, not easier. The small-step
    hypothesis needs the derivative quality to improve with it; as it stands,
    sub-stepping the contact and trusting finite differences makes the
    optimization worse. This closes out the "smaller step" route for the current
    solver and leaves contact-schedule optimization as the open direction.
  With the FDDP warm start restored, example 113 now runs its full iteration
  budget and reaches cost 1494 (was 74096), a 0.138 m jump, a 6.15 rad yaw span,
  and realistic 45 Nm torques. The dynamics gap is still ~3.9 and concentrates
  at the last push node (node 13 of 36), on a base or leg joint velocity. That
  node is the push-to-flight boundary: a rigid double-contact push cannot
  produce the stored launch velocity, so the phase transition, not the
  derivatives, is now the blocker. A per-component diagnostic shows the residual
  gap is the left ankle-pitch velocity at the last double-contact push node: a
  fully constrained double-foot push determines the ankle velocity through the
  contact, so a kinematic warm start cannot make it continuous. Ramping the spin
  from zero, blending the legs into the tuck with a smooth ease-in/ease-out, and
  giving the push an eased extension cut the gap from ~8 to ~3.5 and the cost
  from 74096 to 1185. Redesigning the push to a single toe contact and keeping
  that contact across crouch and push removed the contact-set change at the
  boundary, dropping the gap to ~1.8 and moving it to the terminal node on a
  wrist joint. An earlier attempt that switched from sole to toe at crouch
  introduced a spurious `impulse_velocity` reset and moved the gap to node 0,
  so the contact set must stay constant until takeoff. The residual gap sits at
  the terminal transition on a wrist velocity and is insensitive to the terminal
  weight. The FDDP path already projects the warm start onto a feasible rollout
  and polishes with standard DDP, and a doubled polish budget changes nothing,
  and the FDDP feasibility projection is the real failure: the forward rollout
  from the initial state diverges at the first or second flight node, so the
  solver never applies the feasible polish and returns the open-gap trajectory.
  A `DdpConfig::gap_weight` was added to penalize open gaps during the line
  search; it lowers the raw gap (2.05 to 1.31 on a 60-iteration run) but does
  not stabilize the projection. The projection is now damped
  (`project_feasible_rollout` scales a control down until the step stays finite)
  and instrumentation shows the replayed open-loop controls explode to
  `max|state| ~ 1e157` by node 14, before the state can recover, because the
  controls are tuned to the FDDP's infeasible states. The Boston Dynamics-style
  reading is that the aerial phase has uncontrolled angular momentum, which
  makes the problem ill-conditioned; the next step is a flight cost on
  `rne_dynamics::centroidal_momentum` and a feasibility QP or flight-phase
  controller; `rne_wbc::WholeBodyController` now accepts an empty contact set (flight phase), so only the flight-phase task and controller design remain. Landing-impact resets for
  contact additions work through `ContactSequenceDynamics`'s
  `impulse_velocity`.
  `rne_dynamics::centroidal_momentum` now drives an optional flight cost in
  example 113 (`G1_MOMENTUM_WEIGHT`), a Boston Dynamics-style centroidal
  angular-momentum tracker. At weight 50 the optimizer keeps the flip and cuts
  the peak joint torque from 68 Nm to under 9 Nm while increasing the jump to
  0.21 m, which is the efficient, momentum-driven motion the reference uses. The
  dynamics gap grows to ~3.2 because the cost trades feasibility for momentum
  tracking, so the flight cost and the feasibility projection must be solved
  together, not one after the other.
- **Landing impact needs impulse-aware DDP.** Adding a landing phase after
  re-contact (legs extend, then bend to absorb) improves the objective a lot:
  cost 140, jump 0.23 m, and a full 6.29 rad flip. But the contact-addition
  `impulse_velocity` reset at the flight-to-landing boundary produces a state
  discontinuity the warm start does not satisfy, and the dynamics gap explodes
  to ~57 on an ankle joint. The `ContactSequenceDynamics` reset is already
  seen by the finite-difference Jacobian, so the failure is the warm start, not
  the derivative. Seeding the landing nodes with the post-impact velocity from
  `rne_dynamics::impulse_velocity` cuts the gap from ~57 to ~15, but the impact
  map is a non-smooth velocity projection and the solver still does not close
  it. The next step is a landing impulse treated explicitly in the DDP
  (impulse-aware backward pass or a non-smooth transition model), not a
  kinematic warm start. With the landing phase in the horizon the FDDP solve
  itself diverges (the state reaches ~5e4 by node 11) and the feasibility
  projection, now bounded by `MAX_PROJECTED_STATE_MAGNITUDE`, correctly refuses
  to return it. Rolling out the FDDP feedback policy instead of the controls
  does not help either: the reference trajectory is itself infeasible, so the
  linearization around it makes the policy amplify the deviation (the rollout
  reaches ~3e4 by node 6). A hard impact on a 46-step 29-DoF horizon needs a
  dedicated non-smooth or multiple-shooting solver.
- **Smooth landing via `ContactSequenceDynamics::new_without_impact`.** Dropping
  the impulsive reset and letting the regularized constrained dynamics absorb
  the contact makes the objective far better: with a landing phase the cost
  falls to ~65, the jump is 0.23 m, the flip is a full 6.28 rad, and the peak
  torque is ~1.1 Nm. The transition gap falls from ~57 to ~3.9 once the incoming
  spin is carried into the landing and the joint blend is a smoothstep, but it
  does not close and the feasibility projection still diverges, so a
  multiple-shooting or otherwise non-smooth-aware solver is still required.
  A strong `gap_weight` on the smooth model reaches a near-feasible trajectory
  (gap ~1.4e-3), and dropping the control bounds or refining the step to 0.01 s
  both help (the gap falls to ~0.1 and ~0.7 respectively), but the feasibility
  rollout still diverges. The replay is exponentially unstable: a 0.1 defect
  grows to ~1e4 within a few nodes, so the stored open-gap trajectory cannot be
  reproduced by any open-loop or feedback rollout. The remaining work is a
  multiple-shooting solver with an implicit or much finer contact integration,
  not more parameter tuning.
- **Sub-stepping makes the projection feasible, but the solution is poor.**
  `ContactSequenceDynamics::new_with_substeps` integrates each planner step with
  several inner steps, and on the no-landing backflip the FDDP feasibility
  projection now succeeds and returns a trajectory with a **zero dynamics gap**
  for the first time. The trajectory is dynamically consistent but degenerate:
  it crouches and then free-falls instead of launching, at a high cost. So
  sub-stepping removes the replay instability that blocked feasibility, and the
  remaining problem is the trajectory quality and the landing, which still needs
  a multiple-shooting or otherwise better-conditioned formulation.
  Initializing the controls by inverse dynamics of the warm-start trajectory
  (`rnea`) gives a much more natural trajectory (cost 1848, jump 0.23 m, a full
  6.28 rad flip, peak torque ~21 Nm instead of ~73) but the floating-base rows
  are dropped, so the actual base motion does not follow the reference and the
  gap is ~5. The base reaction must be part of the solve, which again points to
  a multiple-shooting or centroidal formulation.
- **Multiple shooting repairs the warm start.** `rne_oc::solve_multiple_shooting`
  makes the whole state trajectory a decision variable with the dynamics as a
  defect constraint `x_{k+1} - f(x_k, u_k) = 0`, corrected by Gauss-Seidel
  sweeps of local Gauss-Newton steps. On the G1 backflip it repairs the
  computed-torque warm start to a full 6.28 rad flip with a dynamics gap of
  ~0.12, an order of magnitude below the FDDP single-shooting gap (1.76), which
  validates the direction. Bounds are handled by backtracking on the local
  penalized objective with the control clamped at each trial, and the defect
  penalty now carries an augmented-Lagrangian multiplier so the cost keeps
  shaping the trajectory. With torque bounds the G1 backflip returns a natural
  trajectory: a full 6.33 rad flip, a 0.19 m jump, and a 31 Nm peak torque, at a
  dynamics gap of ~0.33. More iterations help (1000 sweeps reach a gap of ~0.18
  with cost 577, a 6.10 rad flip, a 0.15 m jump, and a 98 Nm peak torque), but
  a stronger penalty makes the gap worse because the sweeps roll back. The
  remaining work is closing the gap to the 1e-4 level: the Gauss-Seidel sweep
  is the bottleneck, and 2500 sweeps give the same gap as 1000, so the
  augmented-Lagrangian sweep has plateaued at ~0.18. The next step is a banded
  SQP or a Riccati-based multiple-shooting solve that exploits the
  block-tridiagonal structure and converges in a handful of Newton steps. A Riccati SQP step (backward pass with
  the defect as an affine forcing, then a projected forward pass) was implemented
  and passes the unit tests, but on the stiff 29-DoF G1 contact dynamics the
  linearization is too poor for the merit-function line search to accept a step,
  so the Gauss-Seidel sweep remains the better engine there. The Riccati step
  needs a trust region or an exact-Hessian term before it helps on this problem.
  A trust-region version was then implemented (step scaled into a shrinking
  radius, accepted only on a merit decrease) and still does not beat the
  Gauss-Seidel sweep on the G1 contact dynamics: it drives the cost down to ~565
  but leaves a defect of ~11, because the contact linearization is too poor for
  the Riccati step to reduce defects and the merit line search accepts
  cost-improving steps that barely move them. A numerical exact-Hessian term or
  an implicit contact linearization is the prerequisite for a Riccati SQP here.
  A compliant (penalty) contact model with the multiple-shooting solver was also
  tried, hoping the smooth force law would help; its high stiffness makes the
  Gauss-Seidel sweep worse (gap ~24), so the hard-contact sequence with the
  Gauss-Seidel multiple shooting (gap ~0.18) remains the best result. The G1
  backflip is therefore at a research boundary: below ~0.2 requires an implicit
  contact formulation or an exact-Hessian/FD second derivative that this solver
  does not yet have.
- **A compliant penalty contact was tried as the smooth contact model and does
  not resolve it at the 30 ms step.** Example 115 (a compliant contact-implicit
  model with the multiple-shooting and DDP solvers) shows the penalty law needs
  `p = sqrt(F/k)` of penetration to carry the few-hundred-newton ground force of
  a backflip push. A step-stable `k` (~1e3) implies 0.15-0.25 m of penetration,
  which is unphysical, and the physically intended `k` (1e5-1e6) is unstable at
  30 ms. With the contact chart fixed, the multiple-shooting gap settles near 0.4
  and the single-shooting rollout diverges once the feet sink; the hard-contact
  multiple shooting (~0.18 gap, no penetration) remains the best result. The
  probe did find a real bug: `ContactImplicitArticulatedDynamics::step_state`
  integrated the base with the raw body twist instead of through
  `integrate_configuration`, so a rotated base drifted its world position; this
  is fixed with a regression test. The remaining unlock is a genuine
  complementarity contact formulation at a small enough step (or a variable-step
  integrator), not a softer penalty.
- **Two multiple-shooting solver tweaks did not break the G1 plateau.** A
  symmetric Gauss-Seidel sweep (forward then backward, so the terminal condition
  also propagates) reaches gap 0.257 at 1000 sweeps against 0.18 for the
  forward-only sweep, though at a lower true cost (313 vs 577); the extra
  backward pass only moves the residual to the flight yaw position. Replacing
  the "defect grew by 50%" sweep-acceptance with an augmented-Lagrangian merit
  test diverged (it under-counts a failed roll-out step, so a blowing-up sweep
  looks like a merit decrease); it needs the failed step scored at infinity to
  be usable. Both were reverted, so the forward-only sweep remains the best
  measured solver for this problem.
- **Sub-stepping makes a stiff penalty stable, but the optimizer cannot afford
  it yet.** `ContactImplicitArticulatedDynamics::with_substeps` splits the outer
  step so a `k = 1e6` law settles within a few millimetres of the surface, where
  a single 10 ms step diverges to `1e38` (unit test
  `sub_steps_keep_a_stiff_contact_stable`). In the G1 optimizer each sub-step
  multiplies the finite-difference derivative cost (`4 (nx + nu)` extra roll-outs
  per node), so the affordable sub-step count is far too small to hold the
  backflip. Exploiting the stiffness therefore needs analytic derivatives of the
  compliant contact dynamics (the contact Jacobian derivative, i.e. a
  third-order kinematic quantity), which is the next research step. A zero-control
  warm-start roll-out is not a valid stability probe here because the joint
  free-fall, not the contact, is what diverges.
- **Deliverable:** example 114 renders a clearly labeled forward-kinematics
  backflip reference (`--smoke` gate, `--gif` capture) without claiming physical
  accuracy. A physically simulated backflip requires, in order: (1) analytic
  dynamics derivatives (the central-difference solver does not converge), (2) a
  flight-phase controller and a landing catch (`rne_wbc` now supports the empty-contact flight phase),
  and (3) an articulated centroidal-momentum term to shape the aerial rotation
  (`centroidal_momentum` is now available).

### Open research task: the G1 backflip defect floor

**Problem.** Every planner in `rne_oc` leaves a dynamics defect of about 0.18
(hard contact) to 0.29 (compliant or complementarity contact) at the
push-to-flight transition of the 29-DoF G1 backflip, at a 30 ms planning step.
The defect is bit-identical across 150 and 400+ Gauss-Seidel sweeps, so it is a
fixed point of the solver, not a budget issue.

**Evidence that it is not a tuning problem.** These were all measured on the
same warm start and none lowered the floor: merit-based acceptance, a trust
region, a symmetric sweep, freeing the frozen first control, reference shaping
(ballistic flight velocity and a push base-height ramp), and sub-stepping the
contact. Changing the contact law from a compliant penalty to a hard
velocity-level complementarity solve left the defect unchanged while halving the
penetration. So the residual is a structural property of the single-shooting
reference plus rigid contact at 30 ms, not of the contact stiffness, the
acceptance rule, or the step size.

**Concrete next steps, in increasing cost.**
1. *Contact-schedule optimization.* Make the contact activation per node a
   decision variable (the phase lengths and the active set) and optimize the
   schedule in an outer loop over the inner shooting solve. This is the most
   likely fix because the defect concentrates exactly where the active set
   changes. Example 113's phase lengths are now CLI-tunable (`--crouch`,
   `--push`, `--flight`) and a first sweep shows the schedule matters: the FDDP
   gap is 1.76 at flight 22 but 0.92 at flight 18, with push and crouch
   variations worse (1.8-4.5). But the space is strongly non-convex — flight 16
   and 20 are both far worse than 18 — and the low-gap schedules are degenerate
   (jump 0.06 m against 0.12 m), so a naive local search finds low-defect
   no-jump motions. A schedule optimizer must therefore score both the defect
   and the task (jump/rotation), not the defect alone. The FDDP probe also shows
   the dominant 113 defect is not the push-to-flight transition but the terminal
   node: at `(8,6,22)` the worst component is the right wrist-roll velocity at
   the last node, where the terminal stop fights the low wrist torque limit.
   Lowering the apex target (1.05 -> 0.90) does not reduce the defect
   (1.43-2.18) and removes the jump, so the residual is a boundary-condition
   feasibility limit, not a contact-schedule or jump-height limit.
   Example 113 now has a task-aware grid search (`--search`) that scores
   `gap + w * max(0, jump_target - jump)`. With a weak jump penalty it returns
   the degenerate `(8,6,18)` (gap 0.92, jump 0.06); with a strong one the
   baseline `(8,6,22)` wins (gap 1.76, jump 0.12). No schedule reaches a small
   gap and a real jump at once, which is the same trade-off seen everywhere
   else and confirms the maneuver is near the platform limit at this reference
   and step size.
2. *Exact contact derivatives.* `frame_jacobian_gradient` and
   `constrained_forward_dynamics_gradient` are now implemented and
   finite-difference verified, and `ContactSequenceDynamics::analytic_derivatives`
   uses them instead of differencing the whole step. Example 113 drops from about
   two minutes to 47 seconds and reaches a larger jump (0.229 m against 0.118 m)
   with a different defect (3.37 against 1.76), so the analytic Jacobians change
   the solved trajectory as well as the cost. The remaining second-order work is
   the contact Hessian (differentiate the KKT solution a second time) and a
   Riccati/SQP step on top of it. The chain is now complete and
   finite-difference verified: `frame_jacobian_gradient`,
   `constrained_forward_dynamics_gradient`,
   `constrained_forward_dynamics_hessian`, and
   `ContactSequenceDynamics::{analytic_derivatives, analytic_hessian}`. Example
   113 gained a `G1_SOLVER=ms` / `G1_EXACT_HESSIAN=1` mode. The exact-Hessian
   hard-contact multiple shooting did not finish in 60 minutes because the
   contact Hessian is itself a finite difference of the analytic gradient
   (`(nx + nu)` gradient evaluations per node per sweep), so a hand-derived
   contact Hessian is required before the exact step is affordable at 29 DoF.
   The plain multiple shooting on this warm start reaches gap 4.12, well above
   the historical 0.18, because the committed 113 reference differs from the
   probe that produced 0.18.
  - **The exact Hessian does not change the contact convergence.** On a small
    contact problem (a floating body against the ground, no actuation) the
    multiple shooting reaches defect 0.0604 with Gauss-Newton and 0.0610 with the
    exact constrained Hessian, both at the 120-iteration cap. The second-order
    term therefore does not move the contact defect floor, which is consistent
    with the plateau being structural (a hard-contact fixed point of the
    augmented-Lagrangian sweep) rather than a Hessian-approximation issue. This
    removes the main motivation for hand-deriving the fourth-order contact
    Hessian.
3. *A contact-consistent trajectory.* Instead of a kinematic reference, obtain
   the warm start by solving a short optimal-control problem over the transition
   with the contact schedule fixed but the torque and timing free, so the
   reference itself satisfies the dynamics.

Example 113 (hard contact, FDDP) and example 115 (compliant/complementarity
contact, multiple shooting) are the probes. The forward-kinematics reference in
example 114 is the visual baseline.


### G1 hardware-oriented screening follow-up

The optional contact backflip benchmark now supports self-collision, separate
physics/command periods, a held target-and-gain command with transport delay,
and explicit 90/120 N m knee ceilings. Replaying the saved trajectory at 500 Hz
alone passes, but knee-limited and self-collision-enabled variants fail. Combined
G1/G1 EDU profiles are partial specification screens, not hardware validation.
See [measured results and remaining model gaps](G1_CONTACT_BACKFLIP.md).
The next motion search must solve collision-free launch and landing under these
conditions before claiming progress toward a real-machine backflip.


### 2026-09-22: EDU contact backflip under stricter screening

A 16-parameter non-RL controller now passes five-second MuJoCo rollouts at both
0.125 ms and 0.0625 ms using the URDF-declared 34.13 kg mass, self-collision,
120 N m knee caps, 500 Hz held commands and one assumed command-tick delay.
It completes about 360.022 degrees, lands feet-only, and remains standing for
the final second, with peak speed ratio about 1.033 and no joint-position excess.
The optional optimizer checkpoints ordered parallel batches; regression tests
cover parallel determinism and both legacy/EDU recorded-state hashes.
[Evidence and GIF](G1_CONTACT_BACKFLIP.md#edu-partial-specification-result) remain
model-specific. The same candidate fails at 90 N m; native RNE/Rapier and real
hardware transfer, power/current limits and uncertainty testing remain open.

### RoboSim-space playback and native transfer (2026-09-22)

- Example 114 accepts `--recording DIR` to validate and project all 500 physical
  states into the RoboSim G1 world; `--gif` renders them using wgpu. Full base
  quaternion, joint-name mapping, recording/model hashes, and a zero native
  simulation clock are checked. This is explicitly labeled MuJoCo replay.
- `--native-probe` exercises bounded joint effort in the live Rapier scene,
  without writing the base pose or velocity. Standing preparation is unstable
  under direct PD effort; an implicit-position-motor diagnostic stands, but
  becomes unstable after takeoff. Native success remains open.
- Next: reconcile collision/mass/inertia and actuator integration differences,
  establish stable native motor-only standing, then reoptimize and apply the
  full landing/contact qualification gate. See `G1_CONTACT_BACKFLIP.md`.

### Native joint-effort and inertial-model correction (2026-09-22)

- Generalized joint effort and Coulomb friction now use the authored
  joint-origin rotation, matching the constraint frame. The new revolute and
  prismatic regression fails before the correction and passes afterward;
  all 25 Rapier backend tests pass.
- A separate G1 probe scene enables declared inertia, authored joint frames,
  and fixed-child welding; the legacy walking asset is unchanged. It weighs
  38.13385728 kg because four links still use the importer's 1 kg mass default.
- Sampled ankle pitch/rate feedback plus native force-based position motors
  passes a six-second standing test at 0.5 ms: final-second upright cosine
  >= 0.99999515, base speed <= 0.05280 m/s, with continuous foot contact.
- Direct explicit PD effort remains oscillatory. The native maneuver can
  rotate through a full revolution, but does not land successfully. Mesh
  collisions/self-collision and a full physical qualification gate remain
  open; next use the stable native motor baseline for landing optimization.


### Native backflip actuator screening follow-up

The native probe now measures joint-speed/rating and position-limit excess at
every physics step, uses whole-robot COM velocity in landing feedback, and can
apply bounded implicit velocity commands with torque ceilings. A previous
near-upright position-motor candidate exceeded knee speed by at least 1.928x;
it is rejected. Twenty recorded velocity-servo candidates and four explicit
friction comparisons still fail landing. Native success and a native-success
GIF remain open; the existing RoboSim GIF is labeled MuJoCo state replay.
See [native velocity diagnostics](evidence/g1-contact-backflip/native-transfer/velocity-servo/README.md).


### Native model alignment and first automatic search

A separate generated G1 model now removes four empty fixed sensor frames,
matching the declared 34.13385728 kg while preserving all physical inertia and
23 movable joints. Native standing passes with original sole dimensions.
An optional source-sole profile matches the source contact points/radii but
still imports as a box; its standing contact-continuity gate fails. Both
baseline flips and all 13 candidates in the first native coordinate-pattern
search fail landing. See [model alignment evidence](evidence/g1-contact-backflip/native-transfer/model-alignment/README.md).
Independent sole contacts, body/self-collision and source joint armature/loss
remain unmatched; mass alignment is not complete plant equivalence.


### Independent native sole primitives

An opt-in `urdf.preserve_collision_parts` TOML extension now preserves each
sole sphere in a `CompoundCollider` supported by Rapier. The existing public
asset/spawn structs and physics error enum remain unchanged. The generated
34.13385728 kg model reports two four-part feet; gap raycasts and declared-mass
tests distinguish it from the legacy box. The option-off recording matches
prior physical frames, and repeated compound flips are byte-identical.
Standing still fails positive-contact continuity at 0.5 and 0.125 ms, and both
flip runs collapse (peak speed/rating 2.31648 and 2.18477). Full-body collision,
source armature/passive losses and successful native landing remain open.
See [compound sole evidence](evidence/g1-contact-backflip/native-transfer/compound-soles/README.md)
and [ADR 029](adr/029-compound-contact-primitives.md).

### Native unloading and passive-loss comparison

Every-step diagnostics distinguish positive foot impulse from an active ground
contact pair. Baseline standing has 24 unloaded steps per final second (maximum
2 ms) with no missing ground pair; sole geometry remains below the ground plane.
Increasing solver iterations 16→64 reduces this to 8 steps (maximum 0.5 ms),
but the unchanged standing gate still fails. A separate generated model applies
damping 0.05 and regularized Coulomb loss 0.2 Nm to all 23 movable joints.
Both its 0.5 ms and 0.125 ms backflips collapse, with peak speed/rating 1.70405
and 1.73428. Joint armature 0.01 remains unmatched; Rapier 0.22 exposes no
public generalized-inertia setter. Adding link spatial inertia would not be
an equivalent implementation. Native landing and its success GIF remain open.
See [contact and passive-loss evidence](evidence/g1-contact-backflip/native-transfer/contact-diagnostics/README.md).

### Native generalized motor armature

A repository-local Rapier 0.22 patch now supports constant revolute generalized
armature without changing spatial link mass/inertia. Example 114 enables the
experimental backend feature and accepts `joint_armature_kg_m2`; default
backend builds still compile against unmodified upstream Rapier. Analytic
fixed/floating-base acceleration, torque-limited motor, remapping and invalid
input tests pass; zero-armature native output exactly reproduces prior fields
and frames. See [ADR 030](adr/030-revolute-joint-armature.md).

At 0.01 kg·m² per movable joint, standing tail speed improves from 0.10257 to
0.02658 m/s, but the unchanged positive-impulse continuity gate remains false.
Velocity-motor backflip overspeed decreases to 1.54006x. Direct torque at
0.125 ms becomes runnable and reaches one rotation in flight, yet landing
collapses. Earlier opening at 5.0 rad lowers overspeed to 1.02955x but
under-rotates and falls. Native success/GIF remain open; launch, tuck and
opening need optimization on the native plant. Full-body/self-contact,
constraint friction, contact/limit solvers and free-root numerical damping
remain different. See [armature comparison evidence](evidence/g1-contact-backflip/native-transfer/armature/README.md).

### Native free-root COM integration and landing recovery

A torque-excited G1 regression exposed f32-to-f64 quaternion norm error in the
Rapier/world boundary; promoted rotations now normalize before hierarchical
inverse transforms. A separate zero-gravity welded-pair regression then
reproduced a free-root COM integration defect: root linear velocity describes
the COM while translation coordinates previously described the body origin.
The vendored Rapier patch now integrates free roots at their COM and retains
body pose across COM and fixed/dynamic changes. See [ADR 031](adr/031-multibody-free-root-com.md).

Independent source-URDF FK of native recordings reduces the same candidate's
maximum airborne ballistic position residual from 0.152132 m to 0.0001713 m;
external simulation time stays zero during this audit. The corrected source
candidate rotates once and touches down, then rebounds and falls at 1.991125 s.
Earlier-opening and slower-recovery probes also fail. Seven corrected-plant
recordings are retained in [free-root COM evidence](evidence/g1-contact-backflip/native-transfer/free-root-com/README.md).
Earlier search results apply to the pre-correction plant and retain separate
hashes. Native stable landing/GIF remain open. Contact-phase stiffness and
damping can now be varied independently of the flight servo to investigate
rebound without changing launch or opening gains.

Post-touchdown gain/landing-hip probes (nine trials) and a corrected-plant
launch/opening search (17 trials plus three crouch follow-ups) also fail.
The coarse-step best extends time to collapse from 1.8595 to 2.743 s but reaches
1.190973x rated joint speed; it is not a qualified candidate. See
[landing probes](evidence/g1-contact-backflip/native-transfer/landing-search/README.md)
and [corrected launch search](evidence/g1-contact-backflip/native-transfer/corrected-launch-search/README.md).
Remaining work is sustained landing balance and actuator/contact qualification;
no native-success GIF has been generated.


### Native held landing and fine-step rejection

Native early ankle pitch-rate feedback (1 s gain) and a 10 s recovery hold a
backflip through 15 s at 500 µs. Final-second root speed is at most 1.789 mm/s,
upright cosine exceeds 0.999989, and all final-second steps have positive foot
impulse. A native-dynamics GIF now records this coarse-step result with its
backend, timestep and failed actuator-limit status displayed.

The identical candidate **falls at 2.575125 s at 125 µs**. Peak measured speed
is 1.190973x at 500 µs and 1.171478x at 125 µs. Full-body/self-collision remains
disabled and `qualified_backflip` remains false. Native robust landing,
actuator compliance and full-contact qualification are still open; the GIF
must not be interpreted as fine-step or hardware validation. Optional paired
capture feedback alone did not solve balance. See
[held recovery](evidence/g1-contact-backflip/native-transfer/held-recovery/README.md)
and [support-feedback probes](evidence/g1-contact-backflip/native-transfer/support-recovery/README.md).

### Full-body contact prerequisite: convex geometry

Added backend-neutral `ConvexCollider` without changing the existing public
shape enum or collider struct. Rapier builds a convex hull from local mesh
vertices and retains declared inertia and collision groups. A tetrahedron
regression verifies the actual sloped surface and empty AABB corner, and
invalid/degenerate hulls are rejected without panicking. See
[ADR 032](adr/032-convex-contact-geometry.md).

This supplies the geometry prerequisite only: wire source collision meshes
through the importer, enable self-contact and measure all body-ground pairs
before the next full-body native optimization campaign. The existing 500 µs
held result and rejected 125 µs result remain unchanged and unqualified.

### Convex full-contact model and structural interference

G1 now imports 21 convex body meshes with self-collision enabled and retains
the two four-part feet. Native full-contact standing stays up but fails the
upright threshold (0.975959); the maneuver fails at 1.366 s. The contact audit
shows fixed logo/torso interference up to 150.06 N·s, plus persistent joint-
connected link contacts. Address topology-derived structural exclusions
before optimizing this plant, retaining nonadjacent body/self and ground
contacts. The foot-only observer regression matches all 600 prior frames
exactly. See [full-contact evidence](evidence/g1-contact-backflip/native-transfer/convex-full-contact/README.md).

### Structural filters and self-contact-free tuck

The native probe can now exclude fixed-cluster/internal and directly connected
cluster contacts using the authored joint graph. Ground and nonadjacent self
contacts remain active. Fixed logo/torso interference disappears, but standing
still fails speed/positive-impulse continuity. The baseline flip hits knee/torso
in flight and falls at 2.5165 s. Tuck hip targets 2.10, 2.20 and 2.25 rad remove
positive self-contact impulses but still fall and exceed rated speed. Optimize
launch and opening jointly around these clear tuck poses; no qualification
gate has been relaxed. See [structural contact evidence](evidence/g1-contact-backflip/native-transfer/structural-contact/README.md).

### Full-contact optimization candidate within measured speed threshold

A 17-evaluation coordinate search finds a full-contact 500 µs candidate that
survives through 5 s with peak speed/rating 1.049782 and continuous foot impulse.
It is still recovering (maximum tail root speed 0.161819 m/s), so standing is
not qualified. Long 500/125 µs validation and signed-separation auditing are
in progress; no fine-step or full-body success has been established. The
signed-distance query distinguishes overlap from predictive gaps even when
normal impulse is zero. Exact 62.5 µs timing is available for later refinement.
See [full-contact optimization evidence](evidence/g1-contact-backflip/native-transfer/full-launch-open-search/README.md).

The selected candidate now completes 15 s at 500 µs with peak speed/rating
1.049782, continuous final-second foot impulse, upright cosine above 0.999989
and root speed below 0.000757 m/s. Signed solver-manifold evidence has no
negative nonadjacent self-contact separation. The 125 µs and 62.5 µs runs are
still in progress, so full qualification remains open. A +0.02 rad landing-
knee variant lowers coarse peak speed to 1.031872 but has only been checked
for 5 s. See [long validation evidence](evidence/g1-contact-backflip/native-transfer/full-long-validation/README.md).

- Native full-contact refinement: candidate 14 completed 15 s at 125 µs with
  continuous final-second foot support and no nonadjacent self overlap, but
  peak joint speed 1.05026598x exceeds the unchanged strict 1.05x gate. Rejected;
  62.5 µs was explicitly cancelled for that failure (partial log preserved).
  Advance the +0.02 rad landing-knee margin candidate. Added per-step realized
  actuator effort aggregates, minimum tail height, and source standing-error
  telemetry plus a fail-closed recorded-metrics audit. No final qualification.

- +0.02 rad landing-knee candidate holds 15 s at 500 µs: speed 1.031872x,
  standing error 0.004656, tail base speed 0.002044 m/s, full-contact checks
  pass. The new realized-effort telemetry reveals 120.00001526 Nm at a 120 Nm
  ceiling; retain strict rejection and add 0.001 Nm command headroom. Cancelled
  its unfinished finer-step runs for this measured failure; archive preserves
  the completed coarse result and cancellation reasons. CI run 35685000529 is
  live on eed9b41; its assets semver job flags the pre-existing main field
  `UrdfRobotAsset.weld_fixed_children` (introduced by #291), not a new convex
  extension field. Other CI stages remain under observation.

- Command-headroom producer 0c84bfd passes every recorded gate at 500 µs over
  15 s: speed 1.03187251x, knee effort 119.99901581 Nm (<120 Nm), standing
  error 0.00465579, continuous support and full-contact audit. Same-candidate
  125/62.5 µs runs remain live. Archive: `full-headroom-validation`.
- CI investigation confirms `UrdfRobotAsset.weld_fixed_children` also failed
  the frozen-API baseline on the PR base 407acba (run 35480027356). Its smoke
  stages passed then, so current smoke31/60 failures are regressions. Also
  reproduced and fixed a CI changed-crate filter bug: `echo | grep -q` under
  `pipefail` can return SIGPIPE on large file lists and skip changed packages;
  a here-string preserves the intended check and unchanged-package skip.

- Headroom candidate +0.02 rad fails at 62.5 µs: landing collapses at 2.98925 s,
  so its coarse pass is not final qualification. Preserve the finer failed
  rollout; begin three finer-step probes at +0.002/+0.005/+0.010 rad relative
  to candidate 14, retaining 0.001 Nm headroom and all existing gates.

- The +0.02 rad headroom candidate also passes every recorded gate at 125 µs
  over 15 s (speed 1.03142250x); 62.5 µs still rejects it. Three smaller-knee
  finer-step probes remain active. Shared-engine repairs pass example 31
  (1.13 m carry/release), example 60 (+0.288 rad turn, 2.86 m displacement),
  five mm_lift tests, six pick/place tests, the unchanged robust Go2 turn test,
  and rne_ai all-target release Clippy. Full rne_ai testing exposes additional
  existing-path regressions (366 pass, 10 fail, 12 ignored), including the
  procedural diff-drive rolling direction and other gait probes. Do not claim
  full CI success; continue the regression audit without weakening assertions.

## Native G1 backflip refinement result

The selected +0.005 rad knee refinement completes all three 15-second native
runs. Peak speed ratios at 500/125/62.5 µs are 1.04385118/1.04447365/1.03942852,
all below the unchanged strict 1.05 gate. All measured torque, position,
standing, foot support and full-contact gates pass. The final native GIF and
source/model/controller/recording hashes are archived in the selected-candidate
bundle. Six negative-control/verifier tests pass. Local AI tests now pass
376/376 with 12 existing ignored tests; whole-workspace CI is running on
`021d40d`. Native simulation success does not establish hardware readiness.
