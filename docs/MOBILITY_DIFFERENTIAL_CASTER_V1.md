# Differential-drive trailing-caster benchmark v1

Status: implemented additive M3-C dynamics subgate

This fixture replaces an equivalent support point with an explicit three-point
multibody robot: two rigid driven-wheel stations and one passive trailing caster. The
same `mobility_diff_drive_trailing_caster_v1` TaskSpec, 1 ms clock, open-loop voltage
sequence, motor/transmission/tire laws, contact conditioning, and scoring code run through
Rapier and MuJoCo.

## Why the caster is explicit

Wu et al., [“Steering-angle computation for the multibody modelling of
differential-driving mobile robots with a caster”](https://doi.org/10.1177/1729881418820166),
show that a platform-only kinematic model can be adequate for low-level navigation, but
the caster steering angle and multibody terms matter for dynamic, high-speed, or
heavy-load operation. The v1 contract therefore records rather than hides:

- mount location and mechanical trail;
- bracket and wheel masses;
- swivel and axle inertias;
- swivel and rolling damping;
- caster rolling and combined-slip tire state.

The maneuver settles under gravity, enters a ramped differential arc, straightens, then
ramps through zero into reverse. Evidence includes all three support loads, contact
participation, chassis pose and velocity, wrapped and unwrapped caster swivel, swivel
rate, caster roll rate, lateral force, friction utilization, and backend identity.

## Contact and backend boundary

RNE consumes solved contact points and normal loads and applies the next-step tire wrench
through backend-neutral `ContactPointSample` and `ExternalBodyWrench` contracts. This is
consistent with MuJoCo's documented contact-force/Jacobian formulation and Rapier's
contact graph and solver-force reporting:

- [MuJoCo computation: contacts and constraint forces](https://mujoco.readthedocs.io/en/latest/computation/)
- [Rapier advanced collision detection and contact graph](https://rapier.rs/docs/user_guides/rust/advanced_collision_detection/)

Two MuJoCo compilation invariants are required by this fixture. RNE `CollisionGroups`
are compiled to deterministic pairwise `<exclude>` entries because MuJoCo's native
`contype`/`conaffinity` acceptance rule is not identical to RNE's bilateral mask rule.
Zero-friction colliders compile with `condim="1"`; retaining tangential constraint rows
with zero friction capacity produced an energy-injecting degeneracy under tire wrenches.

## Acceptance evidence

Each trace carries unit-bearing min/max verdicts and a mutation-detecting digest. Both
backends must pass:

- caster and both drive wheels remain in solved contact for at least half the driven run;
- acceleration unloads and reverse braking reloads the caster;
- horizontal displacement, caster lateral force, swivel angle, and swivel rate are
  non-trivial and bounded;
- the cross-backend gaps in load recovery, displacement, swivel, and lateral force remain
  inside declared SI-unit tolerances.

The tests also require same-runtime Rapier byte-for-byte repeatability, validate the
MuJoCo/Rapier comparison, reject trace mutation, verify collision-group compilation, and
verify normal-only compilation for frictionless contact.

The verified comparison artifact is generated outside the repository at
`E:\RNE-build\m3c-sensor\diff-caster-comparison-v1.json`:

| metric | Rapier | MuJoCo | cross-backend gap |
| --- | ---: | ---: | ---: |
| caster contact fraction | 1.000 | 1.000 | — |
| caster load recovery | 124.669 N | 125.335 N | 0.666 N |
| final horizontal displacement | 0.9541 m | 0.9595 m | 0.0055 m |
| maximum caster lateral force | 8.406 N | 8.632 N | 0.225 N |
| maximum caster swivel | 0.5646 rad | 0.5602 rad | 0.0044 rad |
| maximum caster swivel rate | 0.9709 rad/s | 0.9565 rad/s | — |
| minimum drive-wheel contact fraction | 1.000 | 1.000 | — |

Run and emit evidence with:

```powershell
cargo run -p rne_mobility_benchmark --features mujoco -- `
  --backend diff-caster-rapier --output diff-caster-rapier-v1.json
cargo run -p rne_mobility_benchmark --features mujoco -- `
  --backend diff-caster-mujoco --output diff-caster-mujoco-v1.json
cargo run -p rne_mobility_benchmark --features mujoco -- `
  --backend diff-caster-compare --output diff-caster-comparison-v1.json
```

## Explicit limits

The additive `diff_caster_control` module prepares the sensor-only controller
boundary: forward +x, left +y, positive counterclockwise yaw (world x/z and
negative world-Y yaw for this fixture). It accepts only odometry estimates and
task velocity references. The oldest source capture bounds command holding;
duplicates do not renew it or integrate PI state. Expiry clears voltage and
integrators, and malformed or saturated inputs fail closed. Zero terminal voltage
is not an instantaneous physical stop. Gains are provisional, not identified.
Controller-only tests cover signs, deadlines, duplicates, initialization,
saturation and invalid input. `diff_caster_observed` now connects this controller
to the shared plant through actual 2048-count encoders, seeded IMU and motor-current
frontends (10 ms capture period, 2 ms latency). The 10-second run settles, tracks
an arc, straightens and reverses. It records every 1 ms decision, including held
and expired commands. An optional 50 ms IMU capture outage tests expiry/recovery.
The estimator uses world x/z as planar x/y; sensor +Z is mounted along body -Y.
Truth is read only for evidence/scoring, never passed into the controller.

The first Rapier run reproduced exactly and passed the preliminary limits, but
tracking remains coarse: on the predeclared 4–5 second arc window, speed RMSE is
0.09707 m/s against a 0.25 m/s target and yaw-rate RMSE is 0.05147 rad/s against
0.15 rad/s. Final reverse speed is only -0.03460 m/s against -0.15 m/s. Do not
interpret these as tuned control performance. MuJoCo also completed the same
10-second TaskSpec: arc speed RMSE 0.09680 m/s, yaw-rate RMSE 0.05129 rad/s and
final reverse speed -0.03276 m/s. Its final planar position differs from Rapier
by about 0.01053 m. These first runs agree closely but share the tracking deficit.
MuJoCo repeatability/outage regression tests passed, including identical TaskSpec,
source sequence/capture timelines, full-run speed/yaw gap envelopes of 0.02 m/s
and 0.02 rad/s, and final planar gap below 0.05 m. These are synthetic regression
envelopes, not physical validation tolerances. All 12 focused tests and MuJoCo-feature
all-target Clippy passed. Stronger trace validation, Failure Capsules and controller
improvement remain open.

Initial closed-loop artifacts (uncommitted development builds):

- `E:\RNE-build\m3c-sensor\caster-observed-rapier-v1.json`, SHA-256
  `31dc8f1f448f67d88f81c1665bb95be8e9f15054102a28e218a84b43be787304`.
- `E:\RNE-build\m3c-sensor\caster-observed-mujoco-v1.json`, SHA-256
  `879df828e207835d76609aef0ad2ac02c1db2b523c53cd6514c4375998e4e324`.

The next controller candidate is model-based velocity feedforward plus the same
PI feedback. [WPILib's official feedforward documentation](https://docs.wpilib.org/en/latest/docs/software/advanced-controls/controllers/feedforward.html)
describes separating predictable voltage demand from feedback correction, with
explicit voltage/velocity units. For the nominal fixture only,
`k_v = (K_e + R*b/K_t)*gear_ratio/wheel_radius = 14.375 V s/m` and the
differential yaw coefficient is `k_v*track_width/2 = 4.3125 V s/rad`.
These follow the existing motor model's no-load steady-state equations; they are
not empirically identified coefficients. Do not read sampled/randomized privileged
plant parameters into the actor. Static friction and transient acceleration
compensation are not inferred from this calculation. The candidate is now
implemented as `nominal_caster_feedforward_spec`, selected explicitly through
`run_caster_observed_with_control_spec` or the exporter's `--nominal-feedforward`
flag. PI-only remains the default comparison baseline. The same task, references,
sensor seeds, delays, feedback gains and voltage limit are retained; no parameter
search is used. Before evaluation, the comparison requires both arc RMSEs to
halve and final reverse speed to exceed 0.10 m/s in magnitude in the correct
direction. Both predeclared checks passed without changing feedback gains or the
test window:

| Backend | PI speed RMSE (m/s) | FF+PI speed RMSE (m/s) | PI yaw RMSE (rad/s) | FF+PI yaw RMSE (rad/s) | FF+PI final reverse (m/s) |
| --- | ---: | ---: | ---: | ---: | ---: |
| Rapier | 0.097071 | 0.004231 | 0.051468 | 0.001762 | -0.142213 |
| MuJoCo | 0.096799 | 0.003993 | 0.051288 | 0.000947 | -0.143420 |

All 15 focused tests and feature-enabled Clippy passed for that comparison.
Candidate-specific repeatability and outage tests also passed on both backends:
each candidate reproduces exactly, the 50 ms IMU capture outage expires the entire
feedforward-plus-feedback voltage, and fresh measurements restore tracking.
The result demonstrates control improvement on this fixed synthetic model, not
generalization to unknown tire friction, payload, motor temperature or real hardware.

FF+PI artifacts (10000 decisions each, external storage):

- `E:\RNE-build\m3c-sensor\caster-observed-rapier-feedforward-v1.json`, SHA-256
  `8309ac879b75c3a4675c1e61db7374d779e987d1dea00e7f1b7677d521d904c5`.
- `E:\RNE-build\m3c-sensor\caster-observed-mujoco-feedforward-v1.json`, SHA-256
  `9a8efc9d1e548508f84cea160bc9bf9b01369cd048a0e3bc5377d34b831e75fd`.

### Versioned controller evidence

The new trace schema is version 2 (`rne_diff_caster_sensor_trace`) and the actor
TaskSpec is `mobility_diff_caster_sensor_twist_v2`. It explicitly declares whether
a new estimate is available and its health code. Unavailable estimate tensors must
not be interpreted as fresh zeros; the controller receives no estimate and follows
its hold/expiry contract. The earlier files above are unversioned preview evidence:
retain them as baselines, but do not synthesize headers to upgrade them to v2.

Each v2 decision records the voltage supplied to the current motor/tire update as
well as the newly selected next-interval voltage. Accepted estimates retain oldest
capture time and health; each motor measurement retains its original DataBus
stream/entity/sequence/capture/availability header and complete electrical payload.

`CasterObservedRun::validate` checks the complete fixed-step timeline, declared
capture schedule/outage, estimator field presence, finite values, motor source
continuity/latency, voltage bounds and prior-decision-to-applied-command linkage.
It then replays the controller from recorded estimates/references and requires exact
command/status agreement. A SHA-256 content digest detects accidental modifications;
it is not a signature or an attestation that the data came from real hardware.
Even a recomputed digest does not waive timing or controller checks. This is not
raw-sensor estimator replay or full-physics replay; those remain separate evidence
requirements. Validation/round-trip and rehashed-mutation tests passed, along with
all 16 focused caster tests and MuJoCo-feature all-target Clippy. Full workspace
CI for frozen commit `bd4afbfc677ff59d876fd81171ebd8090d5139f5` completed
with exit 0 on 2026-09-08, including workspace checks, RL smokes, headless/OSS
parity, 361 fuzz cases and Behavior CI 10/10. The log is
`E:\RNE-build\m3c-sensor\caster-observed-v2-ci.log`, SHA-256
`6ea895a5c2a009a389599f1d19cb40a73347c9e95bd7c9652c3cace01765c6f8`.
The original open-loop contract is retained. Before/after Rapier trace files are
byte-identical after the shared-plant refactor (SHA-256
`45c87f423f5a744f0eb12fbcbad004957bb0250589280fe6224fa23f1fba7a30`;
`E:\RNE-build\m3c-sensor\diff-caster-refactor-before.json` and
`diff-caster-refactor-after.json`). All ten focused caster/controller tests and
benchmark all-target Clippy passed at the earlier refactor stage. The complete
closed-loop slice is covered by the later `bd4afbf` full CI above, not the prior
`98c7c40` run.

Export preliminary evidence to a new external filename with:

```powershell
cargo run -p rne_mobility_benchmark --example diff_caster_sensor_loop -- rapier E:\RNE-build\m3c-sensor\caster-observed-v2.json
```

Add `--imu-blackout` after the output path to exercise the outage. With
`--features mujoco`, replace `rapier` by `mujoco`. The exporter refuses overwrite.

This is a dynamics fixture, not a full real-robot validation claim. The drive and caster
contacts use sphere proxies on a flat rigid plane; suspension compliance, wheel profile,
roughness, curb impact, split friction, lift/recontact, qualified closed-loop control,
and real-log parameter identification remain later M3-C/M5 gates. Three support points
avoid the over-constrained uneven-ground problem; four-or-more-wheel vehicles still need
identified suspension before comparable load-transfer claims are justified.
