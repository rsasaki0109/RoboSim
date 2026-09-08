# Execution-failure evidence: observed gap and next contract

Status: progress instrumentation, opt-in attempt capture and a shared metadata
schema, exact-byte artifact intake and initial-capture execution replay are implemented.
Broader variants and full regression qualification remain open.

Focused qualification on 2026-09-08: 39 `rne_log` library tests passed, including
legacy Capsule golden compatibility; the MuJoCo-enabled mobility library passed
156 tests with zero failures and two long training jobs ignored by the normal test
selection. MuJoCo-feature all-target Clippy passed with warnings denied. These
results include no-directory-on-invalid-bindings, actual failure replay, healthy
non-reproduction, wrong-build rejection and rehashed-attempt mismatch tests. They
do not replace full workspace CI or rerun the two ignored long training jobs.
Learning-session v2 full-job replay and full CI have passed; the subsequent
caster sensor-loop full CI also passed at `bd4afbf`. This execution-failure
integration is not covered or implied by those earlier passing runs.

The first instrumentation slice adds common `rne_log::ExecutionStage` and
`ExecutionProgress` diagnostic types. The fixed environment exposes progress even
after poisoning: successfully returned control intervals and their local time are
separate from completed physics/ECS/drive ticks. Stages are recorded before entering
fallible work, including physics, synchronization, sensor sampling and estimation.
A backend error after internal physical progress can therefore report the Physics
stage with zero recorded drive ticks; zero does not claim the backend stayed still.
Reset clears these episode-local counters. These diagnostics do not yet capture
actions, lane identity, hashes or replay outcomes and are not a complete Capsule.
The batch's read-only `execution_progress()` additionally binds each diagnostic to
its stable lane ID, episode index and reset seed, without querying a poisoned
backend. Returned diagnostics remain evaluator-only and preserve divergent local
clocks after partial resets. They are not serialized into the legacy replay schema.
The learning session exposes the same diagnostics. Learning projection preserves
an earlier poisoned lane's failure stage; only rejection of an otherwise completed
interval is labeled `LearningProjection`. Tests exercise the real session path for
both post-physics errors and panics, including healthy-lane learner updates, rejected
retries, partial reset and fresh-factory replay with a different worker count.

`SensorLearningSession::capture_step()` is an opt-in in-memory attempt capture.
It first obtains a bounded valid checkpoint; failure there executes no step. It
retains requested voltages only after action selection succeeds, completed batch
outputs before learner/evidence work, actual before/after learner update counts,
and lane diagnostics. Session boundaries distinguish action selection, the batch
call (which also includes preflight/projection), learning and evidence generation.
A caught attempt panic invalidates the session rather than claiming rollback.
Normal `step()` does not copy the checkpoint/history. Allocation aborts and failures
in diagnostic capture itself are not claimed recoverable. Captures can be packaged
with `write_new`; writing alone never produces a verified replay verdict.
The writer separates contract, prior history, attempt and Capsule into four files,
binds exact bytes with SHA-256, refuses existing directories and syncs each file.
It rejects wholly successful outcomes and declares physical post-state unavailable
and replay not attempted. Supplied build provenance is not authenticated. Writer
round-trip, overwrite refusal, success rejection and tamper tests pass. Injected
partial-write acceptance tests remain pending.
Tests injecting errors and panics at the learning/evidence call boundaries
pass and retain physical results and actual update counts; these are test-local
faults, not failures observed in hardware or inside a production learner.

`rne_log::execution_capsule` now defines a separate v1 metadata envelope with
mandatory contract, prior-history and attempted-operation references. Post-attempt
state is explicitly captured or unavailable with a reason. Replay status is named
a producer claim: decoding neither authenticates it nor executes a replay. The
1 MiB bounded decoder validates roles, canonical references, unique paths and text
bounds without reading files. Legacy Capsule serialization is unchanged. Domain
adapters must still verify exact referenced bytes and schemas, bind backend/TaskSpec/
build, package the live attempt and actually re-execute before claiming reproduction.

The benchmark's `observed_execution_capsule::read_execution_artifact` provides
bounded exact-byte intake (at most 32 MiB per reference). Callers supply trusted
role/kind/version and a per-artifact cap; normalized/canonical paths must remain
inside the evidence root, and SHA-256 covers all bytes including whitespace.
It does not validate payload semantics or authenticate a producer. Its filesystem
assumption is a stable local directory, not an adversary racing path replacement.
`SensorLearningSession::replay_failure_capsule` verifies references and expected
backend/TaskSpec/build, reconstructs history using a fresh factory, and executes
the captured operation. It returns Reproduced only when actions, lane progress,
outputs, learner update counts and failure diagnostics match. It does not update
the stored claim or verify unavailable physical post-state. Current support is
initial captures with replay NotAttempted and post-state Unavailable; captured
post-state/report variants are rejected explicitly. Contract and attempt JSON
must match the writer's compact encoding, rejecting duplicate keys and alternate
representations. Post-physics error and panic fixtures pass reproduction with
different worker counts; a healthy factory returns NotReproduced. Wrong build
identity rejects before construction, and rehashed changed update counts do not
pass actual re-execution. These are synthetic fault tests, not hardware evidence.

## Observed incompatibility

`rne_log::FailureMetadata` in `crates/rne_log/src/capsule.rs` requires a first
failing simulation step, an exact simulation timestamp and a state digest recorded
at that step. Capsule validation also requires the failure step to be within the
run's step count. This fits the existing end-of-task voltage acceptance capsule.

It does not honestly describe all `SensorLearningSession` execution errors:

- A backend may advance internally and then return an error or panic before ECS
  synchronization. An exact post-failure world hash may be unavailable.
- One lane may fail while other lanes finish their 10 ms interval and update the
  learner. Partially reset lanes have different clocks; the global decision index
  is not a lane's physical timestamp.
- An unexpected learning/evidence error can follow physical progress. The session
  intentionally refuses to emit a falsely complete checkpoint in that state.
- A failure on the first attempted interval can have zero completed intervals.
  Legacy run metadata has one recorded-step count, not separate attempted and
  completed counts. A new capture must not silently label an attempt as completed.

Do not supply zero hashes, estimated crash times, or the last successful hash
labeled as the failing state. Do not package a successfully restored *history*
as proof that a new, unreproduced runtime fault was reproduced.

## Required shared representation

Extend the common `rne_log` artifact boundary, not a benchmark-only lookalike.
Keep legacy capsule parsing/encoding and golden files unchanged. A versioned
execution-failure variant must distinguish:

1. Known last-successful evidence: lane/episode identity, actual local clock,
   available physical hash, learner checkpoint and verified history reference.
2. Attempted operation: global operation/decision identity, requested action or
   reset, interval start time when known, and failure stage (preflight, physics,
   learning, or evidence generation). Start time is not an inferred crash time.
3. Post-attempt evidence: explicitly captured or explicitly unavailable, with a
   reason. Unavailable is not an all-zero value. Attempted and completed counts
   are distinct, including per-lane progress where the batch is mixed.
4. Replay outcome: not attempted, attempted but not reproduced, or reproduced
   with checked evidence. State the verified fields: reproducing an error does
   not establish equality of unavailable post-failure state. These are not
   task-success flags.

References must bind exact bytes, schema, backend/TaskSpec and build provenance.
Ordinary checksums remain integrity checks, not producer authentication. Reading
metadata does not execute the attempt and must not upgrade its replay status.

## Integration and acceptance work still required

Reset capture now has an opt-in in-memory path retaining pre-attempt history,
requested lane/episode pairs, pre/post lane progress and observations returned
before evidence recording. Construction/validation and post-application evidence
failure are distinct boundaries. Caught panics invalidate the session without
claiming rollback. An in-memory reset replay adapter restores history into fresh
worlds and re-executes the reset, comparing its outcome, observations, progress,
stage and learner counters. It does not verify unavailable solver state or build
authenticity. A reset-specific writer now packages bounded prior history, contract
and attempt artifacts into a new directory with the Capsule metadata written last.
Replay is explicitly not attempted by the writer. The replay reader accepts the
reset-specific artifact kind, checks bounded/hash-bound inputs and the expected
backend/TaskSpec/build contract, then restores history and executes the requested
reset. It compares the complete reset attempt value rather than a message alone.

Reset regression evidence currently covers a later replacement factory returning
an error or panicking, unchanged existing lane progress on construction failure,
successful recovery and history replay after an ordinary error, and evidence
error/panic after replacement. Fresh-world replay reproduces injected construction
faults; healthy factories and altered update/drive-tick diagnostics do not reproduce
them. The focused reset tests and MuJoCo-enabled all-target Clippy pass. Earlier
batch regression coverage passed 30 tests with two long training jobs ignored;
this is not a full CI result for the reset changes.

The reset file format preserves reset-observation missingness and integer
timestamps, not just zero-filled actor tensors. It binds prior history, requested
lane/episode pairs, decision/update counters and execution stage to the
backend/TaskSpec/build contract. Tests cover saved construction error/panic replay,
healthy-factory non-reproduction and rehashed update-count tampering. Invalid
initial observation fields are rejected before a directory is created. Declared
metadata or a matching failure message alone never upgrades a replay claim.
Injected partial filesystem-write failures and physical hardware resets remain
outside this evidence; no unavailable post-reset solver state is certified.

The complete MuJoCo-enabled mobility benchmark library run after these changes
passed 159 tests with zero failures; two explicitly ignored long training jobs
were not rerun (257.07 s). All-target Clippy with warnings denied passed after
the final reset-observation check. Full workspace CI for this reset slice is
still pending; the earlier `c48c85e` CI checkpoint covers suspension hardening,
not these subsequent reset changes.

Validation checkpoint: commit `99d9196` completed the full `cargo run -p xtask -- ci`
with exit code 0, including workspace lint/tests, smoke and RL checks, headless,
OSS parity, 361 fuzz cases, and Behavior CI 10/10 seeds. The external log is
`E:\RNE-build\m3c-sensor\execution-capsule-v1-ci.log`, SHA-256
`abbf6015fd342edb2ca054880a10f7675f1f5e7c48f4f4b25715ef2d96f02c29`.
This is software regression evidence for that revision, not physical calibration
or hardware/HIL validation.

Capture the last valid session and attempted inputs before crossing a fallible
boundary. Retain healthy-lane outputs and actual learner update counts. When the
post-step recorder itself fails, return an explicit incomplete-evidence outcome;
do not recursively assume that failure recording is infallible. Preserve partial
files without overwriting old artifacts, and bound reads and reference counts.

Test first-step failure with zero completions, post-physics error/panic, partial
reset clock divergence, learner failure after healthy physics, evidence-capture
failure, non-reproducing faults, and unchanged v1 golden bytes. Execute actual
factory-backed replays and reject rehashed but mismatched evidence. A deterministic
fault fixture is not hardware/HIL evidence. The existing real-log calibration and
cross-backend SI gates remain separate requirements of the full goal.
