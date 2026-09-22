# Plan: legged locomotion frontier

Status: measured boundary, campaign next

This plan records where the official Unitree Go2 and G1 locomotion actually
stop, why parameter search does not move the wall, and the concrete campaign
that could. It is written after a measurement pass, so every "blocked" claim
below is an observed result, not an assumption. Detailed evidence lives in
[GO2_LOCOMOTION.md](GO2_LOCOMOTION.md) and [G1_LOCOMOTION.md](G1_LOCOMOTION.md).

## Non-RL G1 backflip contact benchmark

The [optional optimization benchmark](G1_CONTACT_BACKFLIP.md) now demonstrates
a full G1 backflip in MuJoCo using the repository's URDF, bounded joint motors,
actual takeoff/landing contacts, and no RL. The pinned controller passes at
0.125 ms and 0.0625 ms, including a final second of stable standing. A recorded
physics GIF and replay-hash regression accompany the measurements. This is an
external contact-plant milestone; native RNE/Rapier and hardware validation
remain separate work. See the benchmark for mass policy, joint-stop tolerances,
motor assumptions, and disabled self-collision.

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
