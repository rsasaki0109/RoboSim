# Go2 locomotion: speed, stability, and the steering boundary

Measurements of what the official Go2's scripted diagonal-pair trot can and
cannot do, on the dynamic multibody under RNE physics. Everything here was
measured on the plant; the numbers that matter are pinned by tests in
`unitree_go2_episode.rs`.

## Speed

Forward speed follows stride amplitude and cadence
(`UnitreeGo2GaitCommand::{stride_rad, cycle_steps}`, 60 Hz steps):

| stride (rad) | cycle (steps) | speed (m/s) |
| --- | --- | --- |
| 0.06 | 90 | 0.020 |
| 0.12 | 90 | 0.053 |
| 0.24 | 90 | 0.096 |
| 0.24 | 45 | 0.166 |

Doubling the cadence at the widest stride roughly triples the default-gait
speed with no loss of height or straightness (yaw drift under 0.001 rad/s).

## Motion is stability

The same sustained flank push (1.8 rad over 20 steps) on the same torque-limited
8 N·m motors:

- the **slow trot** (cycle 90, stride 0.12) capsizes and ends flat on its side —
  this is the fall half of the fall-versus-save scenario in
  [DISTURBANCE_INJECTION.md](DISTURBANCE_INJECTION.md);
- the **walking trot** (cycle 45, stride 0.24, ~0.17 m/s) leans to ~0.9 rad and
  recovers **with no controller at all**, then keeps walking.

Cyclic foot replanting is itself a stabilizer: every half-cycle the swing pair
re-plants under the displaced body, doing implicitly what a capture-step
controller does explicitly. The push that needs a two-channel feedback save at
standstill is shrugged off by the open-loop walk
(`walking_trot_shrugs_off_the_push_that_topples_the_slow_trot`, rendered by
`examples/53_go2_walk_vs_stand_push`).

The robustness has a ceiling — 2.2 rad topples even the walk — and the
posture-feedback save from the standing scenario does not transfer: its
saturated corrections distort the legs enough to stall the gait, and a washout
filter that relaxes the correction re-falls under a sustained push. Keeping a
*walking* gait upright past its open-loop ceiling genuinely requires stepping
control, not posture control.

## The steering boundary, measured

Six joint-space steering mechanisms were measured on the walking trot, and none
produces usable yaw. The Go2 has no hip-yaw joint — each leg is abduction,
thigh pitch, calf pitch — and the abduction axis rolls the body rather than
yawing it, so every hip-based pattern turns into a lean or a bounded twist:

| mechanism | result |
| --- | --- |
| left/right stride asymmetry (up to 4:1) | ~0.01 rad yaw per 8 s — diagonal support pairs cancel the couple |
| front/rear constant hip offset | kills forward motion; unsigned yaw |
| diagonal-pair constant hip offset | clean **signed** but **bounded elastic twist** (~0.55 rad of body twist per rad of offset); springs back on release, no accumulation |
| stance-ramp hip sweep (trot waveform) | cancels — with 0.7 duty the two pairs overlap 40 % of the cycle in opposite sweep phases |
| exclusive-stance-window hip sweep | destroys forward speed, unsigned residual yaw |
| pulsed diagonal twist (ratchet attempt) | elastic: yaw returns to zero on every release, path direction shifts once (~13°) and stops accumulating |

The one honest conclusion: a 3-DoF-per-leg quadruped steers by coordinating
foot placement with body dynamics — exactly what scripted joint-space gaits
cannot express. The learned-gait experiment below quantifies how far a learned
*overlay* can push that boundary, and where it too stops.

## Learning against the boundary

`examples/54_go2_learned_turn` runs a deterministic, resumable, parallel
cross-entropy search over `UnitreeGo2GaitOverlay` — Fourier joint offsets on the
walking trot, including half-frequency terms that deliberately break the trot's
half-cycle symmetry. Three findings, each pinned or reproducible with
`--train` (seed 42):

1. **Objectives get gamed by twist physics.** Maximizing total yaw rediscovered
   the bounded elastic twist (0.32 rad in 8 s, zero thereafter). Maximizing yaw
   in a single late window found a *slow oscillation* whose reversal hid beyond
   the rollout horizon. The objective that survives both: score the **minimum**
   yaw over two disjoint late windows of a 24-second rollout — a twist scores
   zero there and an oscillation goes negative.
2. **The honest optimum is real but small.** The best anti-cheat-scored overlay
   (`UnitreeGo2GaitOverlay::LEARNED_TURN`) turns at a genuinely sustained
   ~0.025 rad/s — positive through every measurement window, upright, walking —
   the first sustained yaw this platform has produced under any control in
   these measurements. `learned_overlay_turns_the_walking_trot` pins it.
3. **The boundary stands.** 0.025 rad/s is an order of magnitude short of
   practical steering. The obvious next hypothesis — that the trot's fixed
   contact schedule is the binding constraint — was then tested directly.

## Testing the contact-schedule hypothesis

`UnitreeGo2GaitSchedule` generalizes the gait generator itself: per-leg phase
offsets, duty factors (0.55–0.85), stride scales, and hip placement/sweep —
contact re-sequencing, the freedom the overlay lacks by construction
(`trot_schedule_reproduces_the_scripted_trot` pins that the trot is the
identity point of this space). `examples/55_go2_stepped_turn` searches its
20 dimensions with the same anti-cheat objective. The result **refutes the
hypothesis** within walkable schedules: the best schedule sustains ~0.015 rad/s
— *below* the overlay's 0.025 — pinned by
`learned_schedule_turn_is_sustained_but_does_not_beat_the_overlay`.

A torque scan closes the remaining obvious explanation: running the overlay's
turn on 23.7, 60, and 120 N·m actuators leaves the yaw rate unchanged
(`yaw_plateau_is_not_torque_limited`). Three search spaces and a five-fold
torque range all plateau at ~0.02 rad/s, which localizes the constraint in the
contact mechanics and morphology: four point feet dragging under hard position
servos shed their tangential impulse in slip, and no joint-space pattern
changes that. What remains genuinely untested: torque-level control (shaping
contact forces instead of positions), aerial-duty gaits below 0.55, and foot
geometry — the concrete openings for the full learned-locomotion arc.

## Torque-level control

The pathway the plateau points at now exists.
`UrdfSceneSim::step_joint_torques` drives any subset of joints with feed-forward
torques inside the real actuator envelope (23.7 N·m, 30.1 rad/s speed ceiling)
while the rest stay position-held, and `named_joint_position` /
`named_joint_velocity` read the reduced-coordinate joint state back in the same
convention the position targets use — everything a closed-loop torque controller
needs. Under the hood a force-capped velocity motor whose target sits at the
actuator's speed ceiling *is* a torque source: the backend applies exactly the
commanded magnitude below the ceiling and brakes with it above, with no new
physics-backend machinery.

Four pinned measurements establish that the servo constraint is genuinely gone
(`unitree_go2_episode.rs`):

- the readback agrees with the position-target convention on every standing
  joint (`joint_state_readback_matches_the_position_convention`);
- a ±8 N·m feed-forward on one calf moves it with the commanded sign while the
  other eleven joints hold (`feed_forward_torque_moves_a_calf_with_the_commanded_sign`);
- zero torque on all twelve joints collapses the stand — torque mode really
  turns the servos off (`zero_torque_frees_every_joint_and_the_stand_collapses`);
- a joint-space PD computed entirely in torque space (kp 25, kd 0.5) holds the
  stand quietly — 0.212 m height, peak tilt 0.023 rad — and replays bit-exactly
  (`torque_pd_holds_the_stand_and_replays_exactly`).

The tuning boundary is itself a pinned result: at the 60 Hz control rate the
explicit velocity feedback destabilizes once kd exceeds roughly `2·I/dt` for the
light distal links — kd 1.0 turns the same quiet stand into a 0.56 rad thrash
(kp 60–200 with kd 2–10, the classic position-servo-like gains, thrash harder
still). Low-rate explicit torque control demands low-bandwidth gains; the
implicit speed-ceiling brake is what keeps the light links bounded.

## Torque-level walking, and the steering nulls repeat at force level

The same low-bandwidth PD **walks**: kp 40 / kd 0.5 tracks the cycle-45 walking
trot at position-servo speed (2.1 m per 12 s) while staying up, and kp 80
crosses the discrete stability bound exactly as the stand did — the gait
thrashes and falls (`torque_pd_tracks_the_walking_trot`). A softer kp 25 walks
faster still (3.3 m) but rides visibly lower. Dynamic locomotion under pure
feed-forward torque commands is real on this platform.

Steering, however, repeats its position-space history at force level.
Three hand-designed mechanisms that position control cannot express at all —
contact-gated diagonal hip twist torque (±4 N·m on stance hips), contact-gated
left/right differential stance thrust (±4/8 N·m on stance thighs, the
tank-steer couple), and yaw-rate feedback through the thrust channel (gains
10/25 toward 0.3 rad/s) — all fail the two-window sustained-turn bar in both
directions, and the feed-forward thrust asymmetry stalls forward progress
(4.7 m → 0.3 m) instead of steering: the gait's own propulsion cycle absorbs
the couple (`contact_gated_hand_torques_do_not_steer_the_torque_walk` pins all
four runs). One more honest boundary: the torque-PD walk's open-loop yaw noise
(~±0.1 rad per 8 s window) is itself an order of magnitude above the position
walk's drift, so any force-level steering signal must clear a noisier plant.

Nine hand mechanisms across two actuation regimes now agree: this morphology
does not steer through any single joint-space or torque-space channel. What
remains is coordination — torque patterns coupled across legs and phase, which
is a search problem, not a design problem.

## Breaking the plateau in torque space

`examples/56_go2_torque_turn` runs that search: the same deterministic,
resumable, parallel CEM harness, now over `UnitreeGo2TorqueOverlay` — per-joint
contact-gated Fourier feed-forward torques (72 coefficients, ±8 N·m) added to
the torque-PD walk, scored by the same anti-cheat two-window objective. The
stance gate couples each term to the leg's *measured* foot contact, a coupling
no position overlay can express.

The search breaks the plateau. The seed-42 winner
(`UnitreeGo2TorqueOverlay::LEARNED_TURN`) sustains **~0.034 rad/s** — windows
+0.304/+0.274 rad per 8 s — while the robot keeps walking (2.9 m per 24 s,
upright throughout). Every position-space result sat at or below 0.025 rad/s,
and the five-fold torque scan proved raw actuator force wasn't the lever;
shaping *when and how* stance torques act is. Force-level coordination
genuinely moves the boundary that joint-space control could not
(`learned_torque_overlay_out_turns_the_position_plateau` pins the comparison).

Two honest caveats, both measured: the margin is 37 % over the old plateau,
not an order of magnitude — practical steering (>0.2 rad/s) remains open — and
the contact-gated rollout is chaotic enough that the pinned coefficients must
carry the search state's full 12-decimal precision; rounding to 6 decimals
lands on a different, non-turning trajectory. The remaining openings are
unchanged: aerial-duty gaits, foot geometry/friction, and richer torque
policies (state feedback instead of phase-indexed feed-forward).
