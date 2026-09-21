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
open-sourced **OmniXtreme** (flow-matching pretraining plus actuation-aware
post-training) reports a 96.36% G1 backflip success rate, and trajectory
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
