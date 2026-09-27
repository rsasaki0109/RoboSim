# Go2 locomotion: speed, stability, and steering

Measurements of what the official Go2's scripted diagonal-pair trot can and
cannot do, on the dynamic multibody under RNE physics. Everything here was
measured on the plant; the numbers that matter are pinned by tests in
`unitree_go2_episode.rs`.

> **The robot measured here has no feet ([#346](https://github.com/rsasaki0109/RoboSim/issues/346)).**
> `unitree_go2_dynamic.rne.robot.toml` does not set `weld_fixed_children`, so
> the importer leaves the fixed-joint children -- the four `*_foot` links and
> `Head_upper`/`Head_lower` -- as loose bodies at their spawn poses on the
> floor. The robot walks on its bare calf ends. Everything on this page was
> measured on that robot, and two things follow directly:
>
> * `link_contact_impulse_ns("*_foot")` reads the loose foot, a constant
>   ~0.2 N·s, so every "contact-gated" torque term below is applied on every
>   step, not only in stance.
> * The foot-friction result in "The feet are not slipping" changed the
>   friction of the loose feet, which cannot affect the robot.
>
> On this asset the scripted trot also walks tail first (it faces +x and
> travels -x). The same model with its fixed children welded
> (`unitree_go2_jump`) does not walk or steer with the current stack. Until
> the walking asset is fixed and re-measured, treat the results here as
> results for this footless robot.

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
speed with no loss of height or straightness (heading drift about
0.003 rad/s).

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

## Correction: the steering chapters measured the wrong axis

Until 2026-09-27 `UrdfSceneObservation::base_relative_yaw_rad` was computed
from `reference⁻¹ · current`, the rotation expressed in the robot's own
spawn frame. The Go2 URDF is z-up and is spawned rotated into the y-up world,
so that frame's "yaw" axis was horizontal: a heading change came out as
relative *roll*, and relative *yaw* was a mix of tilt terms. Every turn in the
steering chapters below was scored and pinned on that signal. The observation
now takes the rotation in the world frame (`current · reference⁻¹`), and was
checked against the heading of the body's forward axis on the torque walk.

Re-measured on Linux (`unitree_go2_episode.rs`), rad per 8 s window over the
two late windows of a 24 s rollout; positive is counter-clockwise seen from
above:

| gait | what the old chapters said | measured about the world vertical |
| --- | --- | --- |
| position trot, no overlay | straight | +0.023 / −0.023 |
| position `UnitreeGo2GaitOverlay::LEARNED_TURN` | +0.025 rad/s sustained left turn | −0.125 / −0.211, sustained right turn (~0.02 rad/s) |
| same at 120 N·m | unchanged | −0.128 / −0.213, unchanged |
| position `UnitreeGo2GaitSchedule::LEARNED_TURN` | slower than the overlay | −0.186 / −0.215, about 1.5× the overlay in the worse window |
| position `LEARNED_AERIAL_TURN` | ~0.014 rad/s | −0.192 / −0.167 |
| torque-PD walk, no feed-forward | ±0.1 rad of noise | −0.791 / +0.719 — it wanders |
| hand thrust ±4 N·m (contact-gated, stance thighs) | stalls, does not steer | −4.30 / −3.72 (+4) and +3.39 / +4.29 (−4): turns the way the sign says |
| hand thrust ±8 N·m | not measured | −9.94 / −10.06 and +9.96 / +9.09, ~1.2 rad/s, upright |
| hand diagonal hip twist ±4 N·m | bounded, no turn | +0.96 / +2.20 (+4) and +0.67 / +0.43 (−4): turns, but not by sign |
| torque `LEARNED_TURN` | +0.304 in window A, ~0.038 rad/s | +1.97 / +2.37, positive under 1e-9, 1e-6 and 1e-3 nudges |
| torque `LEARNED_ROBUST_TURN` | +0.169 / +0.118 | +1.83 / +0.71, contracting under a 3e-9 nudge |
| `UnitreeGo2TorquePolicy::LEARNED_TURN` | coherent turn while walking | +0.049 / +2.965, not a sustained turn; walks 5.59 m |
| `LEARNED_COMMANDED_TURN`, told ±0.25 rad/s | obeys the commanded sign | +1.63 / +1.34 and +1.09 / +0.82: both left; the command shifts the rate only |
| `LEARNED_AUTHORITY_TURN`, told ±0.25 rad/s | stays below the chaos floor | +1.87 / +1.66 and +0.62 / +1.70: both left |
| torque `LEARNED_SPRINT` | straight (\|yaw\| ≈ 0.1) | drifts −1.23 rad over 24 s |

What this overturns:

- **The Go2 steers under torque control.** A contact-gated left/right torque
  on the stance thighs — the tank-steer couple — turns the walk in the
  direction its sign commands, faster as it grows, upright
  (`contact_gated_differential_thrust_steers_the_torque_walk`). At ±4 N·m and
  above it spins nearly in place; at ±1 N·m it still walks 2–4 m in 24 s.
  The "steering boundary", the "~0.02–0.04 rad/s plateau", and "nine hand
  mechanisms do not steer" were artifacts of the axis.
- **The torque overlays turn about ten times faster than reported**, while
  the position-space results keep their size (~0.02 rad/s) but turn the other
  way, and the schedule search edges past the overlay instead of losing to it.
- **Nothing learned yet obeys a turn command.** The commanded and authority
  policies turn left whichever way they are told. The feedback policy's turn
  is not sustained.
- **The searches optimised the wrong signal.** Examples 54–59 and 61 scored
  candidates on the old yaw; `--train` now scores the corrected one and will
  not reproduce the pinned coefficients. The pinned constants are kept as
  what they are and re-described by what they measurably do.

The six position-space hand mechanisms in the first table below were measured
once with the old signal and have no test; they are not re-measured. The
chapters below are kept as the campaign log, with their turn numbers marked
where the table above replaces them.

## The steering boundary, as first measured (superseded)

Six joint-space steering mechanisms were measured on the walking trot with the
faulty yaw signal above; none was re-measured. The table is kept as a log, not
as a result:

| mechanism | result (old signal) |
| --- | --- |
| left/right stride asymmetry (up to 4:1) | ~0.01 rad yaw per 8 s |
| front/rear constant hip offset | kills forward motion; unsigned yaw |
| diagonal-pair constant hip offset | signed but bounded elastic twist |
| stance-ramp hip sweep (trot waveform) | cancels |
| exclusive-stance-window hip sweep | destroys forward speed, unsigned residual yaw |
| pulsed diagonal twist (ratchet attempt) | elastic, path direction shifts once (~13°) |

## Learning a position-space turn

`examples/54_go2_learned_turn` runs a deterministic, resumable, parallel
cross-entropy search over `UnitreeGo2GaitOverlay` — Fourier joint offsets on the
walking trot, including half-frequency terms that deliberately break the trot's
half-cycle symmetry. Its objective lessons stand independently of the axis:
maximizing total yaw rewards a bounded twist, a single late window rewards a
slow oscillation, and the minimum over two disjoint late windows of a 24 s
rollout survives both.

The winner (`UnitreeGo2GaitOverlay::LEARNED_TURN`) does turn the position
trot at a sustained rate, but clockwise: −0.125/−0.211 rad per window
(`learned_overlay_turns_the_walking_trot`).

## Testing the contact-schedule hypothesis

`UnitreeGo2GaitSchedule` generalizes the gait generator itself: per-leg phase
offsets, duty factors (0.55–0.85), stride scales, and hip placement/sweep —
contact re-sequencing, the freedom the overlay lacks by construction
(`trot_schedule_reproduces_the_scripted_trot` pins that the trot is the
identity point of this space). `examples/55_go2_stepped_turn` searches its
20 dimensions with the same two-window objective. Its winner turns clockwise
at −0.186/−0.215 rad per window, about 1.5× the overlay in the worse window
(`learned_schedule_turn_is_sustained_and_edges_past_the_overlay`): contact
re-sequencing buys a modest margin under position servos.

Running the overlay's turn on 23.7 and 120 N·m actuators leaves it unchanged
(`yaw_plateau_is_not_torque_limited`): under position servos the turn rate is
not set by actuator force.

## Torque-level control

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

## Torque-level walking

The same low-bandwidth PD **walks**: kp 40 / kd 0.5 tracks the cycle-45 walking
trot at position-servo speed (2.1 m per 12 s) while staying up, and kp 80
crosses the discrete stability bound exactly as the stand did — the gait
thrashes and falls (`torque_pd_tracks_the_walking_trot`). A softer kp 25 walks
faster still (3.3 m) but rides visibly lower. Dynamic locomotion under pure
feed-forward torque commands is real on this platform.

Its heading is not quiet: with no feed-forward at all it wanders by up to
~0.8 rad per 8 s window, and a 1e-9 nudge to any coefficient changes the
wander. Any steering claim on this walk has to clear that.

Three hand-designed force channels were tried. Contact-gated left/right
differential stance thrust (±4 and ±8 N·m on the stance thighs) **steers**:
it turns the way its sign commands in both windows, several times the wander,
upright (`contact_gated_differential_thrust_steers_the_torque_walk`).
Contact-gated diagonal hip twist turns the walk but not by its sign (+4 and −4
both turn left on Linux). Yaw-rate feedback 25 through the thrust channel
falls.

## Searching torque space

`examples/56_go2_torque_turn` runs the same deterministic, resumable, parallel
CEM harness over `UnitreeGo2TorqueOverlay` — per-joint contact-gated Fourier
feed-forward torques (72 coefficients, ±8 N·m) added to the torque-PD walk.
The stance gate couples each term to the leg's *measured* foot contact, a
coupling no position overlay can express.

The seed-42 winner (`UnitreeGo2TorqueOverlay::LEARNED_TURN`) turns left at
+1.97/+2.37 rad per window, and stays positive in every window under 1e-9,
1e-6 and 1e-3 coefficient nudges
(`learned_torque_overlay_out_turns_the_position_plateau`). The pinned
coefficients carry the search state's full 12-decimal precision: rounding
is a different trajectory.

## Buying robustness with the objective

`--train-robust` re-runs the CEM scoring each candidate by the **median of
three replays** whose coefficients differ by one part in 10⁹. Its winner
(`UnitreeGo2TorqueOverlay::LEARNED_ROBUST_TURN`) is locally contracting: a
3e-9 nudge lands on the same windows, +1.83/+0.71 rad
(`robust_torque_turn_survives_perturbation`). Parameter-scale nudges (1e-6,
1e-3) change the trajectory but keep the turn left.

The README comparison in `examples/60_go2_turn_gif` replays that same
`LEARNED_ROBUST_TURN` overlay beside the zero-feed-forward torque-PD walk. Its
capture begins after the 480-step transient and measures the full 960 rendered
steps. The media gate requires the baseline to hold heading, the overlay to
exceed 0.45 rad of turn while transporting the body by more than 1.0 m, and
both official dynamic Go2 models to remain upright; the smoke measures
−0.07 rad for the baseline and +2.55 rad for the overlay over that interval.
`-- --smoke` runs the identical interval without initializing wgpu.

## Closing the loop: a state-feedback torque policy

`examples/57_go2_torque_policy` closes the loop: a linear
[`UnitreeGo2TorquePolicy`] maps the measured body state (yaw-invariant
up-vector lean components, body-frame lean rates, world yaw rate, two-cycle
phase, bias — eight features, 96 weights) to per-joint feed-forward torques
on the torque-PD walk, searched by the same ensemble-median CEM.

It keeps walking — 5.59 m in 24 s, upright, and ulp-perturbed replays land on
the same windows — but its turn is not sustained: +0.049 rad in the first
window, +2.965 in the second (`torque_policy_keeps_walking_under_feedback`).
A linear policy with no reference input shapes the dynamics; it does not
encode a turn.

## Commanding the turn

`examples/58_go2_steered_turn` makes the yaw-rate feature a *tracking error*
against a commanded reference and scores every candidate by the worse of its
two commanded directions (+0.25 and −0.25 rad/s). The winner
(`UnitreeGo2TorquePolicy::LEARNED_COMMANDED_TURN`) does not obey the
direction: told +0.25 it turns +1.63/+1.34 rad per window, told −0.25 it turns
+1.09/+0.82 — left both times. The command shifts the rate the commanded way
(about +1.06 rad over both windows), and that separation is all
`commanded_yaw_reference_shifts_the_turn_rate` pins.

## Authority is not the lever

`examples/59_go2_authority_turn` raised the feed-forward clamp (±8 → ±12 N·m)
and gave the policy an integral of the yaw-rate error. Its winner
(`LEARNED_AUTHORITY_TURN`) turns left under both commands too, +1.87/+1.66
and +0.62/+1.70 (`authority_and_integral_do_not_lift_the_commanded_turn`).

The learned policies all turn one way; the hand-set differential thrust
already turns both ways on command. A commanded-steering search that starts
from that channel, and scores the corrected heading, is the obvious next step.

## The feet are not slipping

> **Invalid ([#346](https://github.com/rsasaki0109/RoboSim/issues/346)).** The
> `*_foot` colliders this section changes are loose bodies lying at the spawn
> point, not part of the walking robot, so identical trajectories under
> different foot friction say nothing about slip. Kept as the log of what was
> run.

`UrdfSceneSim::set_named_collider_friction` reaches the live collider
(verified by its own test). An **eight-fold foot-friction range (μ 0.25 →
2.0) produces bit-identical turning trajectories** for the position-space
overlay turn: the friction cones stay interior and the feet never slip. Even
on near-ice (feet *and* ground at μ 0.02, where the cones finally bind) the
trot keeps walking and keeps turning the same way (−0.091/−0.158 rad windows).
`the_feet_are_not_slipping` pins all of it. A sphere contact transmits no
torsion about the vertical, so yaw torque comes from force couples between
separated point contacts, and those couples sit inside the cones at every μ
measured.

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
position-servo scripted trot — upright (tilt ≤ 0.37), at height, with
ulp-perturbed replays landing on identical windows. It does not hold its
heading: it drifts about −1.2 rad over the 24 s, so its path curves (the
search's straightness penalty read the faulty yaw signal). The transport
objective found headroom the hand-scripted gait never used: learning beats
the hand gait at the hand gait's own job. The
same torque pathway also ports to the G1 humanoid — gains scale with the
plant, ankles stay servo — see [G1_LOCOMOTION.md](G1_LOCOMOTION.md).

## Removing the scripted trot

The torque overlay still used the scripted trot as a hidden position reference:
the overlay shaped force, but a position target and PD term supplied the main
walking action. `UnitreeGo2PureTorquePolicy` removes that dependency. After a
repeatable startup stand, `examples/64_go2_pure_torque` reads the twelve joint
states, evaluates a phase-conditioned action table plus local joint feedback,
and sends all twelve `UrdfJointTorqueTarget` values directly. No locomotion
step calls `unitree_go2_trot_targets`, and no example-side position-PD torque is
assembled.

The pinned Windows replay covers **1.235/1.182 m** in the two late 8 s windows,
travels 3.418 m total, stays above **0.187 m**, and keeps the measured true tilt
below **0.424 rad**. Exact rates are platform-local under Rapier/libm, so the
headless contract is the two-window transport, height margin, and actuator
limit rather than a cross-platform speed claim. The policy's phase table is a
compact teacher-seeded baseline for this structural arc; the next section adds
explicit velocity commands and terrain observations around that same actuator
path.

The startup stand is not part of the locomotion controller: it uses the
existing position motors only to put both replay and policy in the same initial
configuration. Once the 240-step settle finishes, the example sends every
joint through `step_joint_torques`; the pure policy test therefore exercises the
actual all-joint torque path rather than a hybrid fallback.

## Commanding speed over terrain

The pure-torque baseline now accepts a [`UnitreeGo2VelocityPolicyInput`]: a
forward-speed command, measured body velocity, joint state, and a compact
contact-derived terrain observation. `UnitreeGo2PureTorquePolicy` uses the
velocity error to scale its phase action around a nominal 0.14 m/s walk, stops
the phase action for a zero command, and reverses phase progression for a
negative command. This is speed control around the learned walk, not a new
position-servo path.

The terrain observation contains front/rear contact elevations, their
front-to-rear slope, and the elevation span across contacting feet. The policy
uses slope and span to increase swing-calf torque, giving the foot clearance
authority a terrain-conditioned input without placing a physics-backend type
in `rne_ai`. `assets/scenes/unitree_go2_terrain.rne.scene.toml` supplies a
fixed, low-angle ramp; `examples/65_go2_velocity_terrain` runs the same
headless controller on the flat and ramp scenes and checks speed, height,
terrain observation, and the 23.7 N·m actuator limit.

```bash
cargo run --release -p go2_velocity_terrain --example 65_go2_velocity_terrain
cargo run -p go2_velocity_terrain --example 65_go2_velocity_terrain -- --smoke
```

On the pinned Windows release replay, the 0.14 m/s command averages about
0.18 m/s on the flat scene and 0.16 m/s across the ramp rollout. Exact rates
remain platform-local under Rapier/libm; the portable contract is positive
transport, bounded torque, upright height, and a non-zero contact terrain
signal.

## The search declines to fly

The first morphological lever, tested: the schedule duty range opens from
0.5 down to **0.30** — below 0.5 the diagonal pairs no longer cover the
cycle and the gait acquires flight phases
(`examples/55_go2_stepped_turn -- --train-aerial`, same anti-cheat
objective, seed 42). The result is a double negative, pinned by
`aerial_duty_freedom_is_declined_by_the_search`: given the freedom to fly,
**the search declines it** — every winning leg settles at duty ≥ 0.52 —
and the winner's turn (−0.192/−0.167 rad per window) stays in the same
position-space regime as the walkable schedule. Under position servos,
flight phases cost stability and buy no extra turn.

## Foot clearance and the parkour boundary

> **Almost certainly an artifact ([#346](https://github.com/rsasaki0109/RoboSim/issues/346)).**
> The loose `*_foot` bodies rest on the floor at y = 0.021 m, which is exactly
> the "fixed" 2.1 cm clearance below. The measurement was not pinned in code,
> so which link it read cannot be checked, but a swing-foot height that no
> stride, lift or overlay changes is what a foot left on the floor reports.

The obvious next crowd-pleaser — a Go2 parkour course — was measured before it
was built, and the plant says no. The swing foot's maximum world height is
**2.1 cm** and is essentially fixed: the learned pure-torque walk, the scripted
trot at its maximum `foot_lift_rad = 0.4`, and a hand-augmented calf/thigh
overlay all plateau at the same 2.1 cm clearance. The plateau is kinematic —
the position targets the gait can express do not lift the toe further at any
tested stride, lift, or overlay.

On a 4 cm step the walk does not climb; it topples (tilt ≈ 1.85 rad). A real
parkour goal therefore needs a **foot-clearance-trained gait**, not a larger
torque budget or a different overlay: the search must be allowed to retime
contact and reshape the foot trajectory, which the current
`UnitreeGo2TorqueOverlay` / `UnitreeGo2GaitSchedule` spaces were not asked to
do for height. This is a campaign, not a parameter tweak; the boundary itself
is now measured rather than assumed.

### Normalized rotation calibration

After promoting Rapier rotations to unit f64 quaternions, the historical robust
feed-forward overlay reversed during its first measured turn window (on the
old yaw signal). A deterministic scalar sweep retuned the pinned
`LEARNED_ROBUST_TURN` coefficients to 0.85 times the previous values, the
earlier torque overlay to 0.95, and the reference-free feedback policy's two
body-lean feature columns to 0.8. The corrected measurements of all three are
in the table at the top of this page.
