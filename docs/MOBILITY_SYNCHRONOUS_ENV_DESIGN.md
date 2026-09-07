# Mobility synchronous environment: implementation contract

Status: persistent single-world and CPU batched stepping are implemented; the full
training/batch contract below is **not yet complete**.
The existing episode-parallel runner and policy callback are documented in
[sensor episode batch v1](MOBILITY_SENSOR_EPISODE_BATCH_V1.md).

## Implemented primitive

`observed::fixed_evaluation` additionally executes complete 330-command histories
on fresh worlds and reports integrated absolute tracking error (m), physical
horizon speed (m/s), and final state hashes. Cross-backend evaluation uses the
same explicit reset and voltage history, with separate gap and final-speed
verdicts. The gap budgets are 0.10 m accumulated error and 0.10 m/s final speed;
these are engineering regression budgets, not measured real-vehicle accuracy.
Final speed retains the existing 1.0 +/- 0.1 m/s gate. Matching failures remain
failed final-speed results. Reports are diagnostic values, not authenticated
evidence; comparison executes both worlds rather than trusting imported verdicts.
This open-loop check does not replace closed-loop policy evaluation, real-log
calibration, complete-task acceptance, or common Capsule verification.

The three targeted tests and MuJoCo-enabled Clippy passed on 2026-09-07.
For reset seeds 42/43/44, a 0 V settling interval followed by 3 V produced maximum
cross-backend gaps of 0.007016 m accumulated error and 0.002509 m/s final speed
(rounded upward). Both backends failed the final-speed gate for all three cases;
these runs are retained, not described as successful tracking. Evidence:
`E:/RNE-build/m3c-sensor/fixed-si-evaluation-tests.log` and `fixed-si-clippy.log`.
Workspace-wide validation for this evaluator addition also passed: full `xtask ci`
exited with code zero, including headless, OSS parity, fuzz smoke (361 cases), and
Behavior CI (10/10 seeds). The MuJoCo-enabled mobility suite passed 86 library tests
and one CLI test. Full logs: `E:/RNE-build/m3c-sensor/fixed-si-ci.log` and
`E:/RNE-build/m3c-sensor/fixed-si-mujoco-tests.log`.

`observed::SensorFixedEnvironment` owns a persistent physical world, drive state,
pending wrench, frontends, bus and estimator. `step(voltage_v)` advances ten 1 ms
ticks, returning an exact 10 ms boundary with optional sensor-only observation,
original decision/capture timestamps, input age at the boundary and a freshness
flag. The first reset observation is explicitly missing. After 330 steps the
result is truncated; further stepping requires explicit reset, not autoreset.

`reset(backend, manifest, episode_seed)` constructs replacement state before
swapping it in. The caller supplies the backend and explicit reset seed. Invalid
voltage leaves the current environment unchanged; errors after execution starts
make it unusable until reset. The separately named privileged hash accessor is
for evaluation and is not included in the actor result.

The fixed actor/action/reward TaskSpec and bounded reset/action replay are now
implemented. A training adapter remains pending; this is not yet a complete training
integration.

Validation (2026-09-07): full `cargo run -p xtask -- ci` exited successfully,
including workspace format/Clippy/tests, headless and parity checks, fuzz smoke
and Behavior CI (10/10 seeds). The feature-enabled MuJoCo mobility suite passed
83 library tests and one CLI test, with feature-enabled Clippy also passing.
Logs are retained on the external build drive as
`E:/RNE-build/m3c-sensor/fixed-runtime-ci.log` and
`E:/RNE-build/m3c-sensor/fixed-runtime-mujoco-tests.log`.
These checks establish implementation/replay behavior, not physical calibration
or cross-backend fixed-task acceptance.
No old event-driven trace is relabeled as a fixed-period trace. The fixed runner
and legacy controller/replay share the same extracted 1 ms runtime.

Tests cover exact boundary alignment across different sensor reset seeds, stale
and missing observations, horizon rejection, reset equivalence over full 330-step
trajectories in Rapier and MuJoCo, invalid-action non-mutation and reset recovery
after a deliberately injected post-step plant error. The injected error exercises
the fail-closed wrapper; it is not evidence of every possible native solver fault.

## Fixed TaskSpec and reward

`sensor_fixed_task_spec()` declares `mobility_longitudinal_fixed_sensor_v1`, distinct
from the event-driven trace contract. `SensorFixedObservation::actor_tensors()`
returns ordered F64 tensors with matching shapes/units: validity/freshness, current
and sensor timestamps, age, estimated pose/twist/uncertainty/health, measured motor
voltage/current and the current scheduled speed target. Missing sensor inputs are
zero placeholders with validity zero, never physical truth. The target is recomputed
from current simulation time, even when the retained sensor estimate is older.

Every step scores ten completed 1 ms physical updates, regardless of how many sensor
updates arrived. Its raw diagnostic term is the sum of absolute true speed error
times 0.001 s (meters), using the target at each integration interval's start.
The target is 0 m/s for the first 300 ms, then 1 m/s. Reward is minus this integral
normalized by 1 m, minus 0.001 per control step. Both the diagnostic term and scalar
reward are outside actor tensors. There is no synthetic success termination;
the 330-step horizon truncates and final physical acceptance is a separate gate.

Generic TaskSpec validation now permits an empty termination-condition list only
when a positive step budget is supplied. Empty unbounded tasks and zero horizons
remain invalid. Existing serialized task schemas are unchanged; older validators
that require a condition may reject this newly admitted truncation-only contract.

The generic `randomization` field is not populated with a partial approximation:
the joint reset distribution still lives in `SensorObservedContract`. Binding that
reset contract, backend identity and full reset/action history into fixed-rollout
evidence is handled by the replay proof below. Do not claim standalone TaskSpec
captures all reset physics.

## Bounded persistent operation replay

`record_fixed_batch_replay` executes an explicit list of step/reset operations on
fresh persistent worlds. Its v1 artifact binds the exact backend manifest and fixed
TaskSpec, root seed, fixed WorldRandom noise seed (0), width, initial state projection
and one evidence SHA-256 per operation. The projection covers actual joint-reset
contracts, lane/episode identities, actor tensors, transition rewards/truncation and
completed physical-state hashes. It also binds per-lane execution errors; poisoned
lanes have an error marker instead of a successful physical-state hash.

`verify_fixed_batch_replay` constructs fresh worlds, executes every operation and
requires exact equality of the resulting proof. Worker topology is absent from the
artifact. This tests scheduling independence within one backend, not exact equality
between Rapier and MuJoCo. `validate_metadata` and `decode_fixed_batch_replay` check
structure/integrity only; they cannot establish physical consistency.

Limits are 16 MiB decoded JSON, 1–64 lanes and 1–1024 operations, with existing worker
bounds. Each operation's action width/range and ordered reset mask is checked before
constructing worlds. Terminal/poison preflight and constructor errors abort recording;
lane execution errors can be followed by a reset. This proof is compact: it retains
hashes, not raw projection traces, signed build provenance, backend snapshots or a
common Failure Capsule. Those richer integration paths are not claimed here.

Tests replay 330-step two-lane histories with an intervening partial reset on both
backends and different worker counts. Tests also change commands, reset episode IDs
and result digests, recompute the outer content hash, then require physical replay
to reject them. Bounded decoding rejects oversized/unknown-field/schema inputs.

## CLI reproduction on the external SSD

With the existing external target/TEMP and MuJoCo environment configured:

```powershell
cargo run -p rne_mobility_benchmark --features mujoco -- --backend fixed-record-rapier --input tests/mobility_benchmark/fixtures/fixed-batch-operations.json --seed 42 --num-envs 2 --workers 1 --output E:\RNE-build\m3c-sensor\fixed-replay-rapier-workers-1.json
cargo run -p rne_mobility_benchmark --features mujoco -- --backend fixed-verify-rapier --input E:\RNE-build\m3c-sensor\fixed-replay-rapier-workers-1.json --workers 2
```

Use `fixed-record-mujoco` / `fixed-verify-mujoco` for MuJoCo. Re-record with two
workers and a different output filename to compare byte hashes. Both modes require
`--input`; record additionally requires an explicit seed, width and new output
filename. Verify forbids replacement seed/width/output flags. File input is bounded
to 16 MiB. Output uses create-new semantics and never overwrites existing evidence;
an IO failure can leave a partial new file, which must not be treated as a proof.

The committed fixture runs 30 two-lane steps, resets only lane 1 to episode 7, then
runs 300 more steps. Lane 0 reaches the 3.3 s horizon while lane 1 ends at 3.0 s of
its new episode. Voltages are constant 3 V / 2 V; this is deterministic interface
evidence, not a trained controller or successful task-acceptance claim.

Verified CLI file SHA-256 values (each identical for workers 1 and 2):

- Rapier: `cd933d113664a9267b6e3c278d9f77f67fbbb2cd1a920e89f89ebbd23e14cfbd`.
- MuJoCo: `c20ded02811e42882bb8b7c99d92392ae18591563990fe4948e3b678cd994a7d`.

Both recorded proofs also passed full physical re-execution with two workers.
Files are retained externally at `E:\RNE-build\m3c-sensor\fixed-replay-{backend}-workers-{1,2}.json`.

## Persistent CPU batch

`observed_fixed_batch::SensorFixedBatch` constructs 1–64 independent fixed worlds
at episode index zero, using `derive_episode_seed(root_seed, lane_id, episode_index)`.
`step(&voltages_v)` advances each world by 10 ms using 1–16 scoped CPU workers.
All input widths, voltages, poisoned states and terminal states are checked before
any lane advances. Results remain in lane-ID order regardless of worker count.

Execution is not an atomic physical transaction: each result retains either a
completed transition or an error, so a failed lane cannot hide another lane's
successful transition. Lane execution panics are caught after the environment has
marked itself poisoned; only a fresh reset can recover it. Infrastructure panics
outside lane execution return a batch error and do not promise partial evidence.

`reset_lanes(&[(lane_id, episode_index)])` requires nonempty, increasing unique IDs
in range. All replacement worlds are constructed before existing lanes are swapped,
so factory failure preserves existing worlds. Factory side effects outside those
worlds are caller-owned and are not rolled back. Unselected worlds retain their
sensor queues, episode clocks and pending wrench. Each lane remains at exact 10 ms
boundaries, but episode-local times may differ after partial reset; no global episode
clock or automatic reset is implied.

Five tests verify complete Rapier serial/parallel/single-world transitions over the
330-step horizon, MuJoCo worker independence including final physical hashes, partial
reset isolation, invalid action/mask rejection, atomic replacement construction and
per-lane failure retention. The fault-injection backend delegates to real Rapier
stepping then deliberately returns an error or panics, demonstrating recovery after
actual physical progress without claiming that Rapier naturally emitted that error.

## Evidence driving the design

The current observed loop advances physics by 1 ms, samples sensor frontends,
then invokes its policy only when a newly available encoder pair can update the
estimator. It holds the previous voltage otherwise. Its TaskSpec's nominal
control period is 10 ms; that does not make callback intervals fixed.

In the external `episode-rapier-workers-1.json` evidence (root 42, episode 0),
each of three lanes has 329 decisions. Lane 0 intervals range 8–19 ms, lane 1
8–20 ms, lane 2 8–22 ms. First decisions occur at 2, 1 and 3 ms respectively;
the last decisions precede the 3.3 s physical horizon. Therefore a callback
counter is neither a common physical-time barrier nor the terminal timestamp.

[Farama's autoreset specification](https://farama.org/Vector-Autoreset-Mode)
distinguishes next-step, same-step and disabled reset modes; replay handling
depends on which mode is used. We select explicit reset initially, preserving
terminal observations without introducing ignored actions during autoreset.
[MuJoCo's rollout documentation](https://mujoco.readthedocs.io/en/stable/python.html#rollout)
describes parallel trajectory execution with worker-local data. This informs
ownership, but trajectory batching alone does not supply our feedback step API.

## Required implementation

1. Extract a backend-neutral persistent single-world runtime from `observed.rs`.
   Own the backend, physics world ID, ECS, sensor bus, estimator, drive state,
   pending wrench and simulation counter. Do not rebuild/replay a prefix for
   each action and do not use wall-clock timing in the state machine.
2. Preserve the existing event-driven reference/replay execution path exactly.
   Existing voltage traces and hashes are regression evidence, not something to
   silently rewrite to accommodate a new control schedule.
3. Give the fixed-period environment a distinct TaskSpec/trace contract. A call
   advances exactly ten 1 ms ticks, holds one validated voltage throughout,
   and returns at a common physical-time boundary. Wrench staging remains
   explicit. The horizon is 330 control steps / 3.3 s, including settling.
4. Sample/deliver frontends and update the estimator at physics ticks. Returning
   the latest available estimate at a control boundary must not fabricate a new
   sensor capture. Expose validity, capture/availability provenance, age and
   whether an estimate is new. At reset, an unavailable delayed measurement is
   explicitly invalid, not silently obtained from truth or future frames.
5. Return fallible transitions. Non-finite or out-of-bounds actions are rejected
   before any state mutation. Backend/sensor faults are errors, never task
   success. A solver error after mutation poisons the lane until explicit reset;
   do not claim rollback. Horizon truncation is separate from task acceptance.
6. Reset reconstructs all owned state, including queues, estimator history and
   pending force. Explicit `(root_seed, lane_id, episode_index)` derives the
   reset seed. Record the fixed WorldRandom noise seed honestly until a separate
   noise-stream contract is introduced. No hidden radius enters the estimator.
7. Build batched stepping over these persistent runtimes. Validate the entire
   action batch and lane mask before advancing any lane. Keep stable lane IDs;
   no auto-reset in v1. Retain per-lane failure/terminal status when another
   lane fails. Do not promise an atomic rollback of physical solver operations.
8. Separate actor results from privileged evaluator results. Truth-based reward
   calculation and final physics hash belong to the evaluator; runtime/world
   access must not be embedded in actor observations. Report transition duration
   and define reward accumulation over the same interval before training use.

The existing `rne_ai::Episode` and portable batch APIs assume infallible stepping.
Do not convert solver errors into panic or invented terminal success to fit them;
establish a fallible adapter boundary before reusing checkpoint infrastructure.

## Acceptance tests before claiming completion

- Existing Rapier and MuJoCo event-driven full traces remain exactly unchanged.
- Fixed-step timestamps are common across lanes despite different jitter/dropout.
- No observation references a capture/delivery later than the returned time.
- Invalid input leaves a replay-equivalent lane unchanged; solver failure marks
  the lane unusable until reset. A mixed batch reports which lanes advanced.
- Repeating an explicit reset tuple clears delayed frames and stale actions and
  reproduces the full trace, including after a prior failed episode.
- Terminal observation, duration, final physics hash and truncation occur at the
  horizon even when the final sensor capture precedes it. Post-terminal step
  rejects input until reset, and reset is not counted as an action transition.
- Lane width, worker count and partial reset of other lanes do not alter a
  continuing lane. Replay includes reset operations and per-lane episode seeds.
- Run both backends, within-backend exact replay and cross-backend SI-unit
  tolerances; retain unsuccessful tasks. Run workspace CI on the final code.

This design does not close the broader goal's per-wheel/Ackermann randomization,
real-log calibration, independent noise resets, HIL or throughput gates.
