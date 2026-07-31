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
   practical steering. Sixty dimensions of joint-space freedom cannot
   re-sequence the contacts; the trot's fixed stance/swing schedule is the
   binding constraint. Steering this platform requires controlling the contact
   schedule itself — stepping timing and placement — which is precisely the
   province of full learned or model-based locomotion.
