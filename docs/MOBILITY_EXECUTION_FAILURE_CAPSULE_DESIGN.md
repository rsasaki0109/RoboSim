# Execution-failure evidence: observed gap and next contract

Status: design based on current code; no new capsule schema implemented yet.
The learning-session v2 implementation is frozen while its long validation runs.

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
