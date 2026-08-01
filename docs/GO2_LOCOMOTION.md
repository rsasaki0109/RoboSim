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
contact mechanics and morphology. (The working hypothesis at this point —
point feet shedding tangential impulse in slip — was later tested directly
and refuted; see "The feet are not slipping" below. The localization stands,
the mechanism turned out to be geometric.) What remained untested here:
torque-level control (shaping contact forces instead of positions),
aerial-duty gaits below 0.55, and foot geometry/friction — the concrete
openings the following sections work through.

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

The search breaks the plateau — within a boundary the measurement itself
revealed. The seed-42 winner (`UnitreeGo2TorqueOverlay::LEARNED_TURN`) turns
at **0.038 rad/s** through the first measurement window (+0.304 rad over
steps 480–960) while the robot keeps walking (2.9 m per 24 s, upright
throughout). That window is *robust*: identical across coefficient
perturbations at the 1e-9 scale and across operating systems, and a bounded
elastic twist saturates before it opens, so the number is twist-proof. Every
position-space result sat at or below 0.025 rad/s in the same window; shaping
*when and how* stance torques act does what raw actuator force (the five-fold
scan) could not (`learned_torque_overlay_out_turns_the_position_plateau` pins
the comparison).

The boundary: the compliant torque walk has a **chaos horizon of roughly
16 s**. Beyond it, single-ulp differences — a 1e-9 coefficient nudge, or the
libm rounding differences between operating systems — swing the second
window's yaw by ±0.3 rad (measured ensemble: +0.27, +0.20, +0.11, −0.07
across perturbations and platforms; `--ensemble` reproduces the spread). This
is the third objective-gaming lesson of the campaign: *chaos games
single-trajectory objectives*. A score read from one trajectory past the
horizon is noise, so the pinned test asserts the robust window and pins the
horizon itself (no catastrophic reversal), and the pinned coefficients carry
the search state's full 12-decimal precision — 6-decimal rounding is already
a different trajectory.

The honest margins: 52 % over the old plateau in the robust window, not an
order of magnitude, and provably sustained for 16 s rather than indefinitely.

## Buying robustness with the objective

If chaos games single-trajectory objectives, make the objective sample the
chaos: `--train-robust` re-runs the CEM scoring each candidate by the
**median of three replays** whose coefficients differ by one part in 10⁹,
warm-started from the fragile winner. The search pays 3× per candidate and
buys exactly what it measures. Its winner
(`UnitreeGo2TorqueOverlay::LEARNED_ROBUST_TURN`) turns at **~0.031 rad/s
with both windows positive** (+0.250/+0.274 rad per 8 s) — genuinely
sustained past the horizon that broke the fragile winner — and its perturbed
replays land on the *same* windows to three decimals instead of scattering:
the trajectory is locally contracting, an attractor rather than a knife edge
(`robust_torque_turn_survives_perturbation` pins both the turn and the
contraction).

The limits of what was bought, also measured. Contraction holds at the ulp
scale the objective sampled, but a 10⁻⁶ coefficient change is still a
different trajectory (`--ensemble` prints the spread for both winners), so
the 12-decimal pinning discipline stays. And a *persistent* per-step
perturbation — running the identical binary against a different OS libm —
settles the walk onto a nearby orbit rather than the same one: Linux CI
measures +0.146/+0.121 where Windows measures +0.250/+0.274. Both windows
stay positive on both platforms (the fragile winner categorically fails
that bar — its second window reverses), so the cross-platform guarantee is
the sustained turn itself, while its exact rate is platform-local.
Parameter-scale robustness — a turn that survives *gait-level* variation —
is a different objective for a later search.

## Closing the loop: a state-feedback torque policy

Every controller above is a clock — phase-indexed offsets or torques
replayed open-loop. `examples/57_go2_torque_policy` closes the loop: a
linear [`UnitreeGo2TorquePolicy`] maps the measured body state (yaw-invariant
up-vector lean components, body-frame lean rates, world yaw rate, two-cycle
phase, bias — eight features, 96 weights) to per-joint feed-forward torques
on the torque-PD walk, searched by the same ensemble-median CEM.

The result refines rather than breaks the rate boundary: the policy's turn
(+0.226/+0.346 rad per 8 s) matches the feed-forward overlay's rate instead
of beating it — three different controller families now deliver
0.03-0.04 rad/s, which is starting to look like the platform's honest
turn-in-place capability under this gait. What the closed loop uniquely
buys is the *operating point*: the policy turns **while walking** — 3.8 m
per 24 s of forward progress against the overlay's 1.4 m — the first
controller here that steers without largely stalling the gait, and its
ulp-perturbed replays land on identical windows (the contraction the
ensemble objective selects for).

Linux CI then supplied the arc's sharpest finding: on the other OS libm the
same policy walks even further (5.6 m) and still sustains a coherent turn —
**in the opposite direction** (−0.17/−0.38 rad per window versus Windows'
+0.23/+0.35). Closing the loop feeds the chaotic body state back into the
control, so the orbit difference selects *which* turning attractor the walk
settles into. A linear policy with no reference input shapes the dynamics;
it does not encode a turn *command*. `torque_policy_turns_while_walking`
therefore pins the direction-free claims — a coherent sustained turn, the
preserved walk, same-platform contraction, and a bit-exact replay — and the
next rung is explicit: a commanded yaw-rate *reference* in the feedback (an
error term, not raw state), which is what turns dynamics-shaping into
steering.

## Commanding the turn

`examples/58_go2_steered_turn` climbs the rung the direction finding named:
the yaw-rate feature becomes a *tracking error* against a commanded
reference, and the CEM scores every candidate by the **worse of its two
commanded directions** (+0.25 and −0.25 rad/s) — a chaos-selected one-way
orbit scores its bad direction, so only genuine command obedience survives,
and the two commands double as a diverse ensemble against
single-trajectory objective gaming.

The winner (`UnitreeGo2TorquePolicy::LEARNED_COMMANDED_TURN`) responds to
the command: one set of weights turns +0.094/+0.117 rad per window when
told +0.25 rad/s and −0.095/−0.109 when told −0.25 — four obedient windows
on the training platform.

Linux CI then priced that claim against the chaos floor. The achieved
windows (~0.1 rad) sit *inside* the ±0.3 rad spread that cross-OS libm
orbit differences produce, so on the other platform the per-window signs
scatter (+ref measured −0.02/+0.03 there). What stays above the floor on
both platforms is the **differential response**: commanding + versus −
shifts the total yaw in the commanded direction by 0.23 rad (Linux) to
0.42 rad (Windows) over the measurement horizon.
`commanded_yaw_reference_steers_both_ways` pins that separation, both
runs' uprightness, and a bit-exact replay; absolute four-window obedience
stays a same-platform observation. The general lesson now appears twice:
**a learned behavior is only as portable as its margin above the chaos
floor** — and this behavior's margin is differential, not absolute.

## Authority is not the lever

The obvious next hypothesis — raise the feed-forward clamp (±8 → ±12 N·m)
and give the policy an integral of the yaw-rate error (a learned PI
structure), and absolute obedience will clear the floor — was tested by
`examples/59_go2_authority_turn` and **refuted**. The higher-authority
search's best commanded score reached 0.094, *below* the ±8 winner's
0.188, and the pinned winner's sign-corrected windows stay far below the
±0.3 rad floor (`authority_and_integral_do_not_lift_the_commanded_turn`
pins the refutation; the winner's obedience did not even survive its own
12-decimal state round-trip, more evidence of sub-floor fragility).

This closes the torque-policy chapter with a clean localization. The
commanded amplitude is bounded by the platform's turn capability
(~0.03–0.04 rad/s, established across three controller families), and
that capability is itself the size of the cross-platform chaos floor —
so absolute commanded obedience cannot clear the floor *within this gait
and morphology*, no matter the controller. The campaign's original
boundary diagnosis stands at the end as at the beginning: the remaining
levers are morphological — aerial-duty gaits, foot geometry/friction —
plus richer policies for other objectives on the now-proven closed-loop
pathway.

## The feet are not slipping

The last morphological lever — foot friction — dies at its root, and takes
the campaign's oldest explanation with it. The earlier sections attribute
the yaw plateau to point feet "shedding tangential impulse in slip". Tested
directly (`UrdfSceneSim::set_named_collider_friction` reaches the live
collider, verified by its own test): an **eight-fold foot-friction range
(μ 0.25 → 2.0) produces bit-identical turning trajectories**. The friction
cones stay interior through the entire walking turn — the feet never slip.
Even on near-ice (feet *and* ground at μ 0.02, where the cones finally
bind) the trot keeps walking and keeps turning (+0.155/+0.233 rad windows).
`the_feet_are_not_slipping` pins all of it.

The corrected mechanism: a sphere contact transmits no torsion about the
vertical, so yaw torque can only come from force *couples* between
separated point contacts — and those couples sit comfortably inside the
friction cones at every μ measured. The plateau is **geometric**: stance
width sets the couple's lever arms and the morphology has no hip-yaw axis
to reorient the stance. Friction was never the constraint; slip was never
the loss. With actuation (5× torque), gait shape (three search spaces),
control regime (position, torque, closed-loop), duty (flight phases), and
now friction all measured out, the ~0.02–0.04 rad/s plateau is as
localized as this platform can make it: it is what these legs, this
stance, and this contact geometry can do.

## Learning to outwalk the trot

The steering campaign's tooling — the torque pathway, the deterministic
resumable CEM, the ensemble-median objective, the anti-cheat window
structure — pointed at *transport* instead of yaw for the first time
(`examples/61_go2_learned_sprint`, seed 42). The objective is the minimum
**straight-line displacement** over the two disjoint late windows (lateral
shimmy scores nothing, a dive scores its bad window; per the campaign's
lessons the score is the ensemble median of three ulp-perturbed replays),
with the usual fall/crouch penalties plus a straightness penalty.

The learned-locomotion chapter opens with a decisive result
(`UnitreeGo2TorqueOverlay::LEARNED_SPRINT`, pinned by
`learned_torques_out_walk_the_scripted_trot`): on the same torque-PD walk,
the zero overlay covers 4.65 m per 24 s (0.19 m/s) and the learned overlay
covers **11.79 m (0.49 m/s)** — 2.5× the torque baseline and 3× the
position-servo scripted trot — while staying straight (|yaw| ≈ 0.1 rad),
upright (tilt ≤ 0.37), at height, with ulp-perturbed replays landing on
identical windows. Where every steering search collided with a geometric
ceiling, the transport objective found headroom the hand-scripted gait
never used: learning beats the hand gait at the hand gait's own job. The
same torque pathway also ports to the G1 humanoid — gains scale with the
plant, ankles stay servo — see [G1_LOCOMOTION.md](G1_LOCOMOTION.md).

## The search declines to fly

The first morphological lever, tested: the schedule duty range opens from
0.5 down to **0.30** — below 0.5 the diagonal pairs no longer cover the
cycle and the gait acquires flight phases
(`examples/55_go2_stepped_turn -- --train-aerial`, same anti-cheat
objective, seed 42). The result is a double negative, pinned by
`aerial_duty_freedom_is_declined_by_the_search`: given the freedom to fly,
**the search declines it** — every winning leg settles at duty ≥ 0.52 —
and the winner's turn (~0.014 rad/s, windows +0.126/+0.111) lands exactly
on the walkable-schedule plateau rather than beating the overlay. Under
position servos, flight phases cost stability and buy no steering; the
foot-geometry/friction lever is the one still standing.
