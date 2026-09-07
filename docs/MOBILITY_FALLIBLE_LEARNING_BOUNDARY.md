# Mobility fallible learning boundary

Status: fallible transition projection and a reference online policy-update loop
implemented; broader learner integration and full validation remain incomplete.
This follows the fixed sensor environment and independent noise reset work. It does
not replace real-log calibration, hardware evidence or measured throughput gates.

## Observed mismatch

`rne_ai::Episode::step/reset` return `EpisodeStep` directly, without a failure
result. `EpisodeStep::termination`, generic vectorized `success_count` and portable
batch `success_count` interpret termination as success. Existing portable and
generic batches consume that infallible episode interface. `rne_py` depends on
`rne_ai`, not on the mobility benchmark package or its backend implementations.

In contrast, `SensorFixedEnvironment` can fail after partial physical progress and
require reset. `SensorFixedBatch` retains outcomes for every lane; action preflight
is atomic but physical execution is not. Its fixed task ends by horizon truncation,
not success. These semantics cannot be faithfully encoded by catching an error and
returning a synthetic successful terminal transition.

## Required implementation boundary

Add an opt-in fallible learning adapter; do not change the semantics of existing
infallible episodes or force physics backends to implement them. Keep benchmark
and backend types out of `rne_ai` public contracts and do not make `rne_py` depend
on a benchmark/test package just to expose this environment.

- Provide TaskSpec-ordered, shape-checked actor tensors and validity/freshness
  masks. No world reference, reset parameters, true velocity or physical hash may
  enter policy inference. Reset roots are explicit evaluator configuration.
- Keep successful transition rewards and horizon truncation separate from actor
  observation tensors. A tracking-task horizon must not be labeled success.
- Distinguish batch preflight rejection from per-lane execution failure. Preserve
  which lanes advanced and which require reset; never insert zero reward, fake
  observations or synthetic terminal transitions for a failed execution.
- A failed lane is excluded from learning updates until explicitly reset. Healthy
  lane results remain available, with original lane/episode identities and clocks.
  Do not discard all healthy results because a neighboring lane failed.
- Partial reset must not advance other lanes or enter the rollout as an action.
  Preserve the existing explicit episode index and independent noise-root policy.
- Bind the actual actions, resets and outcomes into replay evidence. A learner
  checkpoint additionally needs policy/optimizer and learner-RNG state; the current
  action replay is not already a resumable training checkpoint.

## Demonstration and acceptance

First connect a real policy update loop to persistent sensor-only batch stepping,
not pre-generated actions or a batch of completed reference-controller episodes.
Demonstrate training on declared train seeds and evaluate once on disjoint held-out
reset/noise seeds, without changing parameters after examining held-out results.
Retain unsuccessful episodes. An improved training reward is not real calibration.

Tests must inject an invalid action and a post-physics failure, verify the exact
lane/mutation/reset behavior, and prove that failed steps never become optimizer
samples. Compare worker counts and partial resets using recorded actions. Execute
both Rapier and MuJoCo; keep within-backend replay separate from SI-unit cross-
backend tolerances. Measure actual batch stepping and policy-update throughput
with rendering disabled and external SSD storage. Python exposure, if needed,
must preserve these fallible contracts rather than silently coercing them into
the current infallible API. Full-workspace CI is required after implementation.

## Implemented transition projection

`SensorFixedBatch::step_learning` steps the persistent batch and returns separate
`transitions` and `failures` collections. A `LearningTransition` binds before/after
TaskSpec-ordered sensor-only tensors, applied voltage, evaluator reward, lane and
episode identity, exact interval timestamps and horizon truncation. It has no
success flag, physical parameters, world handle or physical-state hash. Actor
tensor shapes and finite values are checked against the fixed TaskSpec.

Preflight rejection advances no physics. After stepping, a failed lane has only a
`LearningLaneFailure` diagnostic, never a zero-reward or fake terminal transition.
Healthy lane transitions are retained. Invalid post-step output also invalidates
that lane until explicit reset. Existing `reset_lanes` does not create a learning
transition; the next transition retains its actual lane-local clock and episode.

The adapter does not reinterpret `rne_ai::Episode`, automatically reset, call an
optimizer, or expose the benchmark through `rne_py`. Failure exclusion from this
transition collection alone is not proof of correct learner integration.
Tests cover invalid actions, a complete 330-step horizon,
post-physics errors/panics, healthy-lane retention and recovery after partial reset.
Both initial targeted tests and crate Clippy passed before the reference learner
was added. Updated backend and regression checks are tracked below.

## Reference learner (integration diagnostic)

`SensorTableLearner::train_step` now chooses an action from current sensor tensors,
executes the persistent batch, and updates Q values only for valid transitions.
The bounded voltage grid is [0, 3, 12, 18, 24] V. The table keys contain the
10 ms episode index, estimate-valid mask and estimated speed quantized at 0.25 m/s
and clipped to [-2, 2] m/s. Missing estimates have a dedicated key. No physical
parameters, reference speed measurement or privileged velocity enters the actor.
The fixed task's target schedule is implicit in episode time; this is not a
general-purpose observation encoder. Exploration uses an independent explicit
seed and lane/global-decision coordinates (the caller must not reuse a decision
index across resets). Greedy evaluation neither explores nor updates.

The update follows [Watkins and Dayan's Q-learning rule](https://doi.org/10.1007/BF00992698)
with alpha=0.1, discount=0.99 and epsilon=0.2. The objective here is explicitly the
finite 330-action return: horizon continuation is zero, without labeling it
success. This is not the bootstrap policy for an arbitrary continuing task.
Delayed/quantized observations are not assumed Markov, constant alpha does not
meet the usual diminishing-step assumptions, and no convergence claim is made.
Update validation is transactional; physical stepping is not. Execution failure
diagnostics never enter the learner. This small CPU reference is an integration
diagnostic, not evidence of improved control, scalability or sim-to-real transfer.
Held-out policy-quality evaluation, measured throughput, checkpointing and full
regression validation remain required by the acceptance plan above.

Current focused checks passed: four reference-learner tests with the `mujoco`
feature, both fallible-projection tests, and crate Clippy including MuJoCo. The
online tests execute 330 steps in two actual worlds for each backend, repeat with
one versus two workers, compare the resulting tables exactly within each backend,
and verify 660 updates with nonzero learned values. A separate numerical test
checks the Bellman update, zero finite-horizon continuation, changed greedy action
and read-only evaluation. Invalid sample updates are atomic. Injected post-step
errors and panics in lane 1 produce exactly two updates from healthy lanes; an
explicit partial reset then permits three further updates. These tests measure
integration correctness, not task success or generalization. Full CI completed
without retries (exit 0), including workspace Clippy/tests, example and RL smokes,
headless validation, OSS parity, 361 fuzz cases and Behavior CI 10/10. Its log is
on the external SSD at `E:\RNE-build\m3c-sensor\reference-learner-ci.log`.
The standalone evaluation example was added after that CI's lint/test phase;
it separately passed MuJoCo-feature all-target Clippy and an actual example build.
The experiment protocol and implementation were frozen in commit `07e7cbd`
before any held-out evaluation; see `MOBILITY_REFERENCE_LEARNING_EXPERIMENT.md`.

## Learner checkpoint boundary

`SensorTableLearner::checkpoint(next_decision_index)` encodes schema v1; the
bounded `from_checkpoint` constructor returns a replacement learner and the
saved coordinate without mutating any existing learner or world. The payload
binds the fixed algorithm/version, exact TaskSpec, exploration root, update count,
next global decision coordinate and sorted Q table. Fixed learning hyperparameters
have no momentum or other hidden optimizer state. The stateless exploration RNG
needs its root and next coordinate, not an unrecorded stream cursor.

Q values are stored as integer IEEE-754 bit patterns, preserving negative zero
and small values exactly. The reader accepts at most 1 MiB and at most 5,940 state
entries. It rejects nonfinite decoded values, invalid time/mask/speed bins,
duplicate/unsorted states, inconsistent entry/update counts, unknown fields
(including nested fields), unsupported schema/algorithm/TaskSpec and content hash
mismatch. The format expects its serializer's numeric representations. SHA-256
provides content integrity, not authenticity against someone who can rehash a
modified checkpoint. It does not prove that Q values arose from a real training
history, or that a caller supplied the correct next decision coordinate.

This is deliberately **not a complete training-session snapshot**. Physics,
sensors and lane episode clocks must be reconstructed from matching action/reset
history, and their evidence verified before further learning. Tests serialize
after 50 two-lane learning decisions including a partial reset; only then create
fresh backends with a different worker count, replay and compare every prior
transition, and continue another 50 decisions. Both the resumed transitions and
the final 200-update learner must match uninterrupted execution exactly within
each backend. These tests do not imply equality between Rapier and MuJoCo.

A self-contained session artifact binding this checkpoint to world history is
implemented below. Automatic recovery after unexpected learner/evidence failures
and common Failure Capsule packaging remain pending. The held-out v1 experiment
was not rerun or retuned for these additions.

Checkpoint-focused validation: all three tests passed with `--features mujoco`
(both backend resumes and corruption/bounds rejection), followed by MuJoCo-feature
all-target Clippy with warnings denied. Full CI for this checkpoint slice completed
with exit 0 and no retries, including 361 fuzz cases and Behavior CI 10/10,
separately logged at `E:\RNE-build\m3c-sensor\learner-checkpoint-ci.log`. Its early
lint/test phases preceded the later session implementation and v2 expansion.

## Replay-verified training sessions

`SensorLearningSession` owns the physical batch, learner, next exploration
coordinate and bounded operation history. Callers can step, reset selected lanes,
inspect a read-only learner, encode a checkpoint or reconstruct from one. They
cannot mutate the learner or worlds behind the recorded history. The versioned
session file includes backend identity, physical/noise/exploration roots, lane
count, initial-world evidence, ordered training/reset operations and the final
learner checkpoint. Worker count is intentionally not part of replay identity.

Each train event binds the requested actions (including failed lanes), actual
learning transitions/failures, resulting learner checkpoint digest, lane reset
contracts, sensor observations and physical-state hashes. Actions are recomputed
by the learner during replay rather than supplied by an unchecked caller. Reset
events do not consume a decision coordinate or update. Decoding enforces a
schema-specific byte/operation cap, rejects invalid reset masks, decision/history
mismatch, unknown fields and corrupt digests before replay. The inner learner
checkpoint retains its separate 1 MiB bound and schema/TaskSpec validation.

Restoration creates fresh worlds and re-executes every operation, including
online learning updates. Every event digest and the final learner bytes must
match; a rehashed, structurally valid but incorrect learner or event is not enough
to resume. The constructor returns no partially restored session on a mismatch.
This is bounded replay recovery, not constant-time solver-state restoration or a
signature proving the identity of a hardware capture.

Per-lane solver errors/panics remain failures and permit only explicit reset of
the affected lanes before the next step. Healthy transitions still update the
learner. A failure can be recorded, but restoration requires the factory to
reproduce it; nondeterministic/external faults must not be silently ignored.
Preflight rejections (including the horizon and operation cap) do not advance
physics or learning. Unexpected errors after progress but before a complete
event invalidate the session and prevent incomplete checkpoints. Automatic
recovery of those errors and their common Failure Capsule representation are
not implemented yet.

All four session-focused tests passed with the MuJoCo feature, followed by
MuJoCo-feature all-target Clippy. Tests cover both backend resumes after partial
reset and different worker counts, rehashed incorrect event/learner rejection,
post-physics error and panic recovery, unchanged state after invalid reset/horizon
or operation-cap rejection, and oversized/unknown-field rejection before backend
construction. Full regression validation for the session remains pending. The
completed `learner-checkpoint-ci.log` run began before this code was added, so
its early lint/test phases cannot establish coverage for this later addition.
Per-step checkpoint/evidence work adds overhead that was not included in the
earlier raw-learner v1 throughput measurement; that number must not be reused as
session-checkpoint throughput.

New sessions use schema v2: at most 16,384 operations and 32 MiB. This capacity
covers the prior experiment's 10,560 training decisions plus 31 resets. Existing
schema v1 files remain byte-stable on restoration and keep their original 1,024
operations / 8 MiB limits; restoration does not silently upgrade them. Both
versions retain the same event/learner evidence semantics and exact replay checks.
The reader rejects unsupported versions. The fixed smoke example now uses the
exported `MAX_LEARNING_SESSION_BYTES` input bound rather than a divergent constant.
After rebuilding that example with v2 support, both previously saved v1 files
were resumed in separate processes. Their 120-update continuation digests matched
the values below exactly, and SHA-256 checks confirmed neither file changed.

Four short session tests and MuJoCo-feature all-target Clippy passed after the v2
change, including v1 round-trip preservation. Two long tests are deliberately
excluded from routine tests and must be run explicitly with
`cargo test -p rne_mobility_benchmark --features mujoco training_session_full_training_job -- --ignored --nocapture --test-threads=1`.
Both passed explicitly (2 passed, 0 failed, exit 0), logged to
`E:\RNE-build\m3c-sensor\learning-session-v2-long.log`. Each backend trains 32
episodes on four lanes (42,240 updates), checkpoints all 10,591 operations,
reconstructs with a different worker count, checks byte identity, then resets and
continues both copies. Recovery-test roots 5000/5001/5002 are not the prior
held-out performance seeds; this does not repeat or retune that evaluation.
The full-job checkpoints were 1,624,973 bytes for MuJoCo and 1,624,104 bytes for
Rapier. Both restored byte-identically and continued identically after a reset.
The combined long tests took 978.34 s, including both backends, learning, full
history verification and continuation checks, with other validation work also
running on the host. This is not a standalone throughput benchmark. Per-step
checkpoint/evidence overhead warrants profiling; the earlier raw learner's
updates-per-second figure does not characterize this journaled session.
Arbitrary unbounded training and constant-time snapshots are still unsupported;
history must not be discarded to claim equivalent recovery.

### Separate-process evidence

The `learning_session_checkpoint` example takes `<rapier|mujoco> <record|resume>
<path>`. Record saves after 30 two-lane learning decisions and one partial reset,
then continues for 30 more. Resume reads the bounded file in a new process,
verifies history with one worker instead of two, and performs the same
continuation. It checks for 120 final updates and prints the continued-session
digest. This is a fixed smoke fixture, not a general long-job launcher.

Both backend record/resume process pairs exited 0 and produced identical final
digests within each backend on 2026-09-08:

- Rapier: `b9b59657a66a35ad5a012c17c00e3ae8191b4f4361f8c9f2b6b6c0d5d4a288b0`.
  Saved artifact `E:\RNE-build\m3c-sensor\learning-session-rapier-v1.json`,
  10,450 bytes, SHA-256
  `7c0bffb3fb0295d19fadcfd466de94491c62a0d3f91898ca07226932e1e5bc0e`.
- MuJoCo: `713d0af1e8f42904bb33635d1dca67f571f499ff23eccab09034fec6a0dc3f5c`.
  Saved artifact `E:\RNE-build\m3c-sensor\learning-session-mujoco-v1.json`,
  10,449 bytes, SHA-256
  `7aac999107c1281e15cdcd54f1e2c91e453994d1896b460e63ba8fb22c429c95`.

Attempting to record over the Rapier file failed with an already-exists error,
and its SHA-256 remained unchanged. Attempting to resume that file with MuJoCo
failed with an initial-world/backend mismatch. The example passed feature-enabled
all-target Clippy and build checks. Full session regression CI remains pending;
these process tests are focused evidence, not a substitute for the full goal.
