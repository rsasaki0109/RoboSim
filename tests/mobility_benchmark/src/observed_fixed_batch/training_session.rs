//! Replay-verified training continuation, not opaque solver-state serialization.

use super::*;
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};

mod reset_capsule;

/// Maximum input size of a v2 learning session (32 MiB); v1 retains its 8 MiB cap.
pub const MAX_LEARNING_SESSION_BYTES: usize = 32 * 1024 * 1024;
const MAX_BYTES: usize = MAX_LEARNING_SESSION_BYTES;
const MAX_EVENTS: usize = 16 * 1024;

/// Outcome of actual fresh-factory execution, not a decoded producer claim.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LearningAttemptReplayOutcome {
    /// History, requested operation, progress, outputs, update counts and failure
    /// diagnostics match. Unavailable physical post-state is not verified.
    Reproduced,
    /// History restored but the new attempt did not match the recorded failure.
    NotReproduced,
}

/// In-memory diagnostic capture of one attempted training operation.
/// Not a serialized Capsule or proof that an external failure reproduces.
#[derive(Debug)]
pub struct CapturedLearningAttempt {
    /// Bounded, valid session checkpoint captured before attempting execution.
    pub before_checkpoint: Vec<u8>,
    /// Exploration coordinate requested, not a lane-local clock.
    pub decision_index: u64,
    /// Lane identities and last known progress before the attempt.
    pub before: Vec<SensorFixedLaneProgress>,
    /// Requested voltages, or None if action selection did not complete.
    pub requested_actions_v: Option<Vec<f64>>,
    /// Completed batch output, retained even if subsequent learning/evidence fails.
    pub batch_output: Option<LearningBatchStep>,
    /// Entered session boundary; lane diagnostics give finer physical stages.
    pub stage: Option<rne_log::ExecutionStage>,
    /// Actual learner updates before the attempt.
    pub updates_before: u64,
    /// Actual learner updates after the attempt; not inferred from requested actions.
    pub updates_after: u64,
    /// Post-attempt diagnostics, not post-failure physical-state hashes.
    pub after: Vec<SensorFixedLaneProgress>,
    /// Normal step result, explicit error, or caught panic diagnostic.
    pub outcome: std::result::Result<LearningBatchStep, String>,
}

fn limits(version: u32) -> Result<(usize, usize)> {
    match version {
        1 => Ok((1024, 8 * 1024 * 1024)),
        2 => Ok((MAX_EVENTS, MAX_BYTES)),
        _ => anyhow::bail!("unsupported session schema"),
    }
}

/// Entered reset boundary, not proof of an unavailable physical post-state.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum ResetAttemptStage {
    /// Reset validation and replacement-world construction, before batch return.
    Construct,
    /// Replacement worlds were applied; session evidence is being recorded.
    Evidence,
}

/// Opt-in reset diagnostics; packaging does not establish failure reproduction.
#[derive(Debug)]
pub struct CapturedResetAttempt {
    /// Bounded valid history retained before executing the reset.
    pub before_checkpoint: Vec<u8>,
    /// Exploration coordinate before reset; reset must not consume a decision.
    pub decision_index: u64,
    /// Actual learner update count before attempting reset.
    pub updates_before: u64,
    /// Actual learner update count after attempting reset.
    pub updates_after: u64,
    /// Requested lane IDs and episode indices, in caller order.
    pub requested_lanes: Vec<(usize, u64)>,
    /// Lane identities and progress before attempting reset.
    pub before: Vec<SensorFixedLaneProgress>,
    /// Lane identities and progress after the attempt, including failures.
    pub after: Vec<SensorFixedLaneProgress>,
    /// Returned reset observations retained before evidence recording.
    pub observations: Option<Vec<SensorFixedObservation>>,
    /// Last entered boundary, or None on success.
    pub stage: Option<ResetAttemptStage>,
    /// Reset success, explicit error, or caught panic diagnostic.
    pub outcome: std::result::Result<Vec<SensorFixedObservation>, String>,
}

impl CapturedLearningAttempt {
    fn attempt_value(&self) -> serde_json::Value {
        serde_json::json!({
            "kind": "rne_learning_execution_attempt", "schema_version": 1,
            "operation": "train", "decision_index": self.decision_index,
            "before": self.before, "after": self.after,
            "requested_actions_v": self.requested_actions_v,
            "batch_output": self.batch_output, "stage": self.stage,
            "updates_before": self.updates_before, "updates_after": self.updates_after,
            "outcome": self.outcome,
        })
    }

    /// Packages a failed attempt into a new directory; never overwrites evidence.
    /// This binds exact bytes but does not verify execution or attest supplied build
    /// provenance. Partial files remain on write failure. No post-failure physical
    /// hash is fabricated, and replay is explicitly not attempted by this method.
    #[allow(clippy::too_many_lines)] // TODO(cleanup): split (151/150 lines); see PR body
    pub fn write_new(
        &self,
        directory: &std::path::Path,
        build: &rne_log::BuildMetadata,
    ) -> Result<rne_log::execution_capsule::ExecutionFailureCapsule> {
        use rne_log::execution_capsule::{
            ExecutionFailureCapsule, ExecutionReplayClaim, PostAttemptEvidence,
        };
        use std::io::Write;
        ensure!(
            self.before_checkpoint.len() <= MAX_BYTES,
            "prior checkpoint exceeds bound"
        );
        let mut prior: Snapshot = serde_json::from_slice(&self.before_checkpoint)?;
        ensure!(
            self.before_checkpoint.len() <= limits(prior.schema_version)?.1,
            "prior schema byte limit"
        );
        ensure!(
            (1..=MAX_SENSOR_BATCH_LANES).contains(&prior.num_envs)
                && prior.events.len() <= limits(prior.schema_version)?.0,
            "prior history bounds"
        );
        prior.backend.validate()?;
        let expected_digest = std::mem::take(&mut prior.content_sha256);
        ensure!(
            digest(&prior)? == expected_digest,
            "prior history digest mismatch"
        );
        prior.content_sha256 = expected_digest;
        let (learner, next) =
            SensorTableLearner::from_checkpoint(prior.learner_checkpoint.as_bytes())?;
        ensure!(
            next == self.decision_index
                && next
                    == prior
                        .events
                        .iter()
                        .filter(|event| event.operation == Operation::Train)
                        .count() as u64
                && learner.updates() == self.updates_before,
            "prior learner/attempt coordinate mismatch"
        );
        ensure!(
            self.before.len() == prior.num_envs && self.after.len() == prior.num_envs,
            "attempt lane count mismatch"
        );
        for (index, (before, after)) in self.before.iter().zip(&self.after).enumerate() {
            ensure!(
                before.lane_id == index
                    && after.lane_id == index
                    && before.episode_index == after.episode_index
                    && before.episode_seed == after.episode_seed
                    && before.episode_seed
                        == derive_episode_seed(
                            prior.physical_root,
                            index as u64,
                            before.episode_index
                        ),
                "attempt lane identity mismatch"
            );
            ensure!(
                after
                    .progress
                    .completed_intervals
                    .checked_sub(before.progress.completed_intervals)
                    .is_some_and(|delta| delta <= 1)
                    && after
                        .progress
                        .completed_drive_ticks
                        .checked_sub(before.progress.completed_drive_ticks)
                        .is_some_and(|delta| delta <= 10)
                    && after.progress.last_successful_time_ticks
                        >= before.progress.last_successful_time_ticks,
                "invalid attempt progress"
            );
        }
        if let Some(actions) = &self.requested_actions_v {
            ensure!(
                actions.len() == prior.num_envs && actions.iter().all(|value| value.is_finite()),
                "invalid attempt actions"
            );
        }
        ensure!(
            self.updates_after
                .checked_sub(self.updates_before)
                .is_some_and(|delta| delta <= prior.num_envs as u64),
            "invalid learner update delta"
        );
        if let Ok(output) = &self.outcome {
            ensure!(
                self.batch_output.as_ref() == Some(output) && self.stage.is_none(),
                "completed outcome mismatch"
            );
        }
        let message = match &self.outcome {
            Err(error) => error.clone(),
            Ok(output) => {
                ensure!(
                    !output.failures.is_empty(),
                    "successful attempt is not a failure capsule"
                );
                "one or more execution lanes failed".to_string()
            }
        };
        let contract = serde_json::to_vec(&serde_json::json!({
            "kind": "rne_learning_execution_contract", "schema_version": 1,
            "backend": prior.backend, "task_spec": crate::observed::sensor_fixed_task_spec(), "build": build,
        }))?;
        let attempt = serde_json::to_vec(&self.attempt_value())?;
        for bytes in [&contract, &attempt] {
            ensure!(
                bytes.len() <= 1024 * 1024,
                "execution metadata artifact exceeds bound"
            );
        }
        let reference = |role: &str, kind: &str, version, path: &str, bytes: &[u8]| {
            rne_log::ArtifactRef::new(
                role,
                kind,
                version,
                path,
                format!("{:x}", Sha256::digest(bytes)),
            )
        };
        let capsule = ExecutionFailureCapsule {
            kind: "rne_execution_failure_capsule".into(), schema_version: 1,
            failure_code: if self.outcome.is_err() { "learning_attempt_error" } else { "lane_execution_error" }.into(),
            message,
            contract: reference("execution_contract", "rne_learning_execution_contract", 1, "contract.json", &contract)?,
            prior_history: reference("prior_history", "rne_learning_session", prior.schema_version, "prior-history.json", &self.before_checkpoint)?,
            attempt: reference("execution_attempt", "rne_learning_execution_attempt", 1, "attempt.json", &attempt)?,
            post_attempt: PostAttemptEvidence::Unavailable { reason: "capture records progress only; no post-attempt physical-state snapshot was collected".into() },
            replay: ExecutionReplayClaim::NotAttempted,
        };
        capsule.validate()?;
        let metadata = serde_json::to_vec(&capsule)?;
        ensure!(
            metadata.len() <= rne_log::execution_capsule::MAX_EXECUTION_CAPSULE_BYTES,
            "capsule byte limit"
        );
        std::fs::create_dir(directory).context("create new execution capsule directory")?;
        for (name, bytes) in [
            ("contract.json", contract.as_slice()),
            ("prior-history.json", self.before_checkpoint.as_slice()),
            ("attempt.json", attempt.as_slice()),
            ("capsule.json", metadata.as_slice()),
        ] {
            let mut file = std::fs::OpenOptions::new()
                .write(true)
                .create_new(true)
                .open(directory.join(name))?;
            file.write_all(bytes)?;
            file.sync_all()?;
        }
        Ok(capsule)
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "operation", rename_all = "snake_case", deny_unknown_fields)]
enum Operation {
    Train,
    Reset { lanes: Vec<(usize, u64)> },
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct Event {
    operation: Operation,
    evidence_sha256: String,
}

#[derive(Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct Snapshot {
    schema_version: u32,
    backend: PhysicsBackendManifest,
    physical_root: u64,
    noise_root: u64,
    exploration_root: u64,
    num_envs: usize,
    initial_sha256: String,
    events: Vec<Event>,
    learner_checkpoint: String,
    content_sha256: String,
}

fn digest(value: &impl Serialize) -> Result<String> {
    Ok(format!(
        "sha256:{:x}",
        Sha256::digest(serde_json::to_vec(value)?)
    ))
}

/// Bounded online-learning session whose checkpoint binds learner and world history.
///
/// Restoration re-executes at most 16,384 operations from fresh worlds, validates
/// each physical/sensor/output/learner digest and the final learner checkpoint,
/// then returns live worlds. It does not trust a hash as proof of execution.
/// Worker count may change; backend identity may not. No solver handles or hidden
/// physical parameters enter the learner. This is not an O(1) solver snapshot.
pub struct SensorLearningSession<B: PhysicsBackend, F> {
    format_version: u32,
    batch: SensorFixedBatch<B, F>,
    learner: SensorTableLearner,
    exploration_root: u64,
    next_decision: u64,
    initial_sha256: String,
    events: Vec<Event>,
    usable: bool,
    #[cfg(test)]
    attempt_fault: Option<(rne_log::ExecutionStage, bool)>,
}

impl<B: PhysicsBackend, F> std::fmt::Debug for SensorLearningSession<B, F> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("SensorLearningSession")
            .field("format_version", &self.format_version)
            .field("batch", &self.batch)
            .field("learner", &self.learner)
            .field("next_decision", &self.next_decision)
            .field("events", &self.events.len())
            .field("usable", &self.usable)
            .finish_non_exhaustive()
    }
}

impl<B, F> SensorLearningSession<B, F>
where
    B: PhysicsBackend,
    F: Fn() -> Result<(B, PhysicsBackendManifest)>,
{
    /// Creates fresh worlds and a zero-initialized learner with three explicit roots.
    pub fn new(
        factory: F,
        physical_root: u64,
        noise_root: u64,
        exploration_root: u64,
        num_envs: usize,
        workers: usize,
    ) -> Result<Self> {
        let batch = SensorFixedBatch::new_with_noise_root(
            factory,
            physical_root,
            noise_root,
            num_envs,
            workers,
        )?;
        let initial_sha256 = replay::evidence_digest(&batch, serde_json::Value::Null)?;
        Ok(Self {
            format_version: 2,
            batch,
            learner: SensorTableLearner::new(exploration_root),
            exploration_root,
            next_decision: 0,
            initial_sha256,
            events: Vec::new(),
            usable: true,
            #[cfg(test)]
            attempt_fault: None,
        })
    }

    /// Read-only learned policy; callers cannot mutate session learning state.
    ///
    /// See also [`Self::replay_failure_capsule`] for independent failure verification.
    pub fn learner(&self) -> &SensorTableLearner {
        &self.learner
    }

    /// Re-executes an in-memory failed reset after restoring its history into fresh
    /// worlds. Compares diagnostics and returned observations, not unavailable
    /// solver state. This does not authenticate a build or read serialized Capsules.
    pub fn replay_captured_reset(
        factory: F,
        workers: usize,
        expected: &CapturedResetAttempt,
    ) -> Result<LearningAttemptReplayOutcome> {
        ensure!(expected.outcome.is_err(), "reset capture is not a failure");
        ensure!(
            expected.before_checkpoint.len() <= MAX_BYTES,
            "reset history exceeds bound"
        );
        ensure!(
            expected.requested_lanes.len() <= MAX_SENSOR_BATCH_LANES,
            "reset mask exceeds bound"
        );
        let mut restored = Self::from_checkpoint(factory, workers, &expected.before_checkpoint)?;
        let actual = restored.capture_reset(&expected.requested_lanes)?;
        let matches = actual.outcome.is_err()
            && actual.outcome == expected.outcome
            && actual.before_checkpoint == expected.before_checkpoint
            && actual.decision_index == expected.decision_index
            && actual.updates_before == expected.updates_before
            && actual.updates_after == expected.updates_after
            && actual.before == expected.before
            && actual.after == expected.after
            && actual.observations == expected.observations
            && actual.stage == expected.stage;
        Ok(if matches {
            LearningAttemptReplayOutcome::Reproduced
        } else {
            LearningAttemptReplayOutcome::NotReproduced
        })
    }

    /// Reads bounded artifacts, restores history into fresh worlds and executes the
    /// attempted training step or reset with the supplied factory. No existing world is modified.
    /// Accepts initial captures with unavailable post-state and no replay claim;
    /// other envelope variants are unsupported rather than silently trusted.
    /// This does not verify unavailable physical state or authenticate build metadata.
    pub fn replay_failure_capsule(
        factory: F,
        workers: usize,
        directory: &std::path::Path,
        expected_build: &rne_log::BuildMetadata,
    ) -> Result<LearningAttemptReplayOutcome> {
        use crate::observed_execution_capsule::{
            decode_canonical_execution_json, read_execution_artifact,
        };
        use rne_log::execution_capsule::{
            ExecutionFailureCapsule, ExecutionReplayClaim, PostAttemptEvidence,
            MAX_EXECUTION_CAPSULE_BYTES,
        };
        use std::io::Read;
        let mut bytes = Vec::new();
        std::fs::File::open(directory.join("capsule.json"))?
            .take(MAX_EXECUTION_CAPSULE_BYTES as u64 + 1)
            .read_to_end(&mut bytes)?;
        let capsule = ExecutionFailureCapsule::decode(&bytes)?;
        ensure!(
            capsule.replay == ExecutionReplayClaim::NotAttempted
                && matches!(
                    capsule.post_attempt,
                    PostAttemptEvidence::Unavailable { .. }
                ),
            "unsupported learning execution envelope variant"
        );
        let contract = read_execution_artifact(
            directory,
            &capsule.contract,
            "execution_contract",
            "rne_learning_execution_contract",
            1,
            1024 * 1024,
        )?;
        let prior_version = capsule.prior_history.schema_version;
        let prior_bytes = read_execution_artifact(
            directory,
            &capsule.prior_history,
            "prior_history",
            "rne_learning_session",
            prior_version,
            limits(prior_version)?.1,
        )?;
        let is_reset = capsule.attempt.kind == "rne_reset_execution_attempt";
        let attempt_kind = if is_reset {
            "rne_reset_execution_attempt"
        } else {
            "rne_learning_execution_attempt"
        };
        let attempt = read_execution_artifact(
            directory,
            &capsule.attempt,
            "execution_attempt",
            attempt_kind,
            1,
            1024 * 1024,
        )?;
        let prior: Snapshot = serde_json::from_slice(&prior_bytes)?;
        ensure!(
            prior.schema_version == prior_version,
            "prior history reference version mismatch"
        );
        let contract = decode_canonical_execution_json(&contract)?;
        ensure!(
            contract
                == serde_json::json!({
                    "kind": "rne_learning_execution_contract", "schema_version": 1,
                    "backend": prior.backend, "task_spec": crate::observed::sensor_fixed_task_spec(), "build": expected_build,
                }),
            "execution contract/backend/TaskSpec/build mismatch"
        );
        let attempt = decode_canonical_execution_json(&attempt)?;
        if is_reset {
            ensure!(
                attempt["kind"] == attempt_kind
                    && attempt["schema_version"] == 1
                    && attempt["operation"] == "reset",
                "unsupported reset attempt"
            );
            let lanes_value = attempt["requested_lanes"]
                .as_array()
                .context("reset mask is not an array")?;
            ensure!(
                lanes_value.len() <= MAX_SENSOR_BATCH_LANES,
                "reset mask exceeds bound"
            );
            let lanes: Vec<(usize, u64)> =
                serde_json::from_value(attempt["requested_lanes"].clone())?;
            let mut restored = Self::from_checkpoint(factory, workers, &prior_bytes)?;
            let actual = restored.capture_reset(&lanes)?;
            let reproduced = actual.outcome.as_ref().err().is_some_and(|error| {
                capsule.failure_code == "reset_attempt_error" && capsule.message == *error
            }) && actual
                .reset_attempt_value()
                .is_ok_and(|value| value == attempt);
            return Ok(if reproduced {
                LearningAttemptReplayOutcome::Reproduced
            } else {
                LearningAttemptReplayOutcome::NotReproduced
            });
        }
        ensure!(
            attempt["kind"] == "rne_learning_execution_attempt"
                && attempt["schema_version"] == 1
                && attempt["operation"] == "train",
            "unsupported learning attempt"
        );
        let mut restored = Self::from_checkpoint(factory, workers, &prior_bytes)?;
        let actual = restored.capture_step()?;
        let actual_failure = match &actual.outcome {
            Err(error) => Some(("learning_attempt_error", error.as_str())),
            Ok(output) if !output.failures.is_empty() => {
                Some(("lane_execution_error", "one or more execution lanes failed"))
            }
            _ => None,
        };
        Ok(
            if actual_failure == Some((capsule.failure_code.as_str(), capsule.message.as_str()))
                && attempt == actual.attempt_value()
            {
                LearningAttemptReplayOutcome::Reproduced
            } else {
                LearningAttemptReplayOutcome::NotReproduced
            },
        )
    }

    /// Evaluator-only lane progress, available even after an unrecorded failure.
    /// This does not checkpoint the session, certify learner updates or resume it.
    pub fn execution_progress(&self) -> Vec<SensorFixedLaneProgress> {
        self.batch.execution_progress()
    }

    fn preflight(&self) -> Result<()> {
        ensure!(
            self.usable,
            "session has an unrecorded execution/update failure"
        );
        ensure!(
            self.events.len() < limits(self.format_version)?.0,
            "session operation limit reached"
        );
        Ok(())
    }

    /// Chooses sensor-only actions, executes one interval and records actual updates.
    ///
    /// Ordinary per-lane solver failures are retained in the returned output and
    /// history; reset failed lanes explicitly. Invalid input/horizon preflight
    /// leaves the session unchanged. An unexpected error after physics starts or
    /// during evidence generation invalidates the session rather than producing
    /// an incomplete checkpoint. Such errors require external diagnosis.
    pub fn step(&mut self) -> Result<LearningBatchStep> {
        self.step_inner(None)
    }

    /// Captures a valid pre-attempt checkpoint, then executes one training step.
    /// Pre-capture errors return without execution. A caught panic invalidates the
    /// session; this does not roll back physics or claim a recoverable solver state.
    /// History copying is opt-in and retains the existing schema-specific byte cap.
    pub fn capture_step(&mut self) -> Result<CapturedLearningAttempt> {
        self.preflight()?;
        let mut capture = CapturedLearningAttempt {
            before_checkpoint: self.checkpoint()?,
            decision_index: self.next_decision,
            before: self.execution_progress(),
            requested_actions_v: None,
            batch_output: None,
            stage: None,
            updates_before: self.learner.updates(),
            updates_after: self.learner.updates(),
            after: Vec::new(),
            outcome: Err("attempt not executed".to_string()),
        };
        capture.outcome = match std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            self.step_inner(Some(&mut capture))
        })) {
            Ok(result) => result.map_err(|error| format!("{error:#}")),
            Err(_) => {
                self.usable = false;
                Err("training attempt panicked; session invalidated".to_string())
            }
        };
        capture.updates_after = self.learner.updates();
        capture.after = self.execution_progress();
        Ok(capture)
    }

    fn step_inner(
        &mut self,
        mut capture: Option<&mut CapturedLearningAttempt>,
    ) -> Result<LearningBatchStep> {
        self.preflight()?;
        let next = self
            .next_decision
            .checked_add(1)
            .context("decision counter overflow")?;
        if let Some(capture) = capture.as_deref_mut() {
            capture.stage = Some(rne_log::ExecutionStage::ActionSelection);
        }
        let actions = self
            .batch
            .observations()
            .into_iter()
            .enumerate()
            .map(|(lane, observation)| {
                self.learner.action(
                    &observation?.actor_tensors(),
                    lane,
                    self.next_decision,
                    true,
                )
            })
            .collect::<Result<Vec<_>>>()?;
        if let Some(capture) = capture.as_deref_mut() {
            capture.requested_actions_v = Some(actions.clone());
            capture.stage = Some(rne_log::ExecutionStage::BatchExecution);
        }
        self.usable = false;
        let output = self.batch.step_learning(&actions)?;
        if let Some(capture) = capture.as_deref_mut() {
            capture.batch_output = Some(output.clone());
            capture.stage = Some(rne_log::ExecutionStage::Learning);
        }
        #[cfg(test)]
        self.inject_attempt_fault(rne_log::ExecutionStage::Learning)?;
        self.learner.learn(&output.transitions)?;
        if let Some(capture) = capture.as_deref_mut() {
            capture.stage = Some(rne_log::ExecutionStage::Evidence);
        }
        #[cfg(test)]
        self.inject_attempt_fault(rne_log::ExecutionStage::Evidence)?;
        let learner_sha256 = digest(&self.learner.checkpoint(next)?)?;
        let evidence_sha256 = replay::evidence_digest(
            &self.batch,
            serde_json::json!({
                "requested_actions_v": actions, "learning": output, "learner_sha256": learner_sha256,
            }),
        )?;
        self.events.push(Event {
            operation: Operation::Train,
            evidence_sha256,
        });
        self.next_decision = next;
        self.usable = true;
        if let Some(capture) = capture {
            capture.stage = None;
        }
        Ok(output)
    }

    #[cfg(test)]
    fn inject_attempt_fault(&self, stage: rne_log::ExecutionStage) -> Result<()> {
        if let Some((selected, panic)) = self.attempt_fault {
            if selected == stage {
                assert!(!panic, "injected attempt-boundary panic");
                anyhow::bail!("injected attempt-boundary error");
            }
        }
        Ok(())
    }

    /// Resets selected lanes without consuming an exploration coordinate or update.
    /// Batch reset validation/construction is atomic. A post-reset evidence error
    /// invalidates the session, because physical reset has already occurred.
    pub fn reset_lanes(&mut self, lanes: &[(usize, u64)]) -> Result<Vec<SensorFixedObservation>> {
        self.reset_lanes_inner(lanes, None)
    }

    /// Retains pre-reset history and post-attempt diagnostics. A caught panic
    /// invalidates the session without claiming rollback. Requests larger than the
    /// batch width are rejected before history copying. No filesystem is touched.
    pub fn capture_reset(&mut self, lanes: &[(usize, u64)]) -> Result<CapturedResetAttempt> {
        self.preflight()?;
        ensure!(
            lanes.len() <= self.batch.lanes.len(),
            "reset capture mask exceeds batch width"
        );
        let mut capture = CapturedResetAttempt {
            before_checkpoint: self.checkpoint()?,
            decision_index: self.next_decision,
            updates_before: self.learner.updates(),
            updates_after: self.learner.updates(),
            requested_lanes: lanes.to_vec(),
            before: self.execution_progress(),
            after: Vec::new(),
            observations: None,
            stage: Some(ResetAttemptStage::Construct),
            outcome: Err("reset not attempted".to_string()),
        };
        capture.outcome = match std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            self.reset_lanes_inner(lanes, Some(&mut capture))
        })) {
            Ok(result) => result.map_err(|error| format!("{error:#}")),
            Err(_) => {
                self.usable = false;
                Err("reset attempt panicked; session invalidated".to_string())
            }
        };
        capture.after = self.execution_progress();
        capture.updates_after = self.learner.updates();
        Ok(capture)
    }

    fn reset_lanes_inner(
        &mut self,
        lanes: &[(usize, u64)],
        mut capture: Option<&mut CapturedResetAttempt>,
    ) -> Result<Vec<SensorFixedObservation>> {
        self.preflight()?;
        let observations = self.batch.reset_lanes(lanes)?;
        self.usable = false;
        if let Some(capture) = capture.as_deref_mut() {
            capture.observations = Some(observations.clone());
            capture.stage = Some(ResetAttemptStage::Evidence);
        }
        #[cfg(test)]
        self.inject_attempt_fault(rne_log::ExecutionStage::Evidence)?;
        let output = observations
            .iter()
            .map(SensorFixedObservation::actor_tensors)
            .collect::<Vec<_>>();
        let evidence_sha256 =
            replay::evidence_digest(&self.batch, serde_json::json!({"reset": output}))?;
        self.events.push(Event {
            operation: Operation::Reset {
                lanes: lanes.to_vec(),
            },
            evidence_sha256,
        });
        self.usable = true;
        if let Some(capture) = capture {
            capture.stage = None;
        }
        Ok(observations)
    }

    /// Encodes at most 32 MiB and 16,384 operations in v2. No filesystem is touched.
    /// Restored v1 sessions keep their original 8 MiB / 1,024-operation bounds and
    /// serialization; no implicit upgrade occurs. History cannot be discarded. A failed
    /// lane can be saved, but restoration requires that failure to reproduce.
    /// Digests are corruption checks, not signatures or proof of hardware behavior.
    pub fn checkpoint(&self) -> Result<Vec<u8>> {
        ensure!(self.usable, "cannot checkpoint unrecorded partial progress");
        let mut snapshot = Snapshot {
            schema_version: self.format_version,
            backend: self.batch.manifest.clone(),
            physical_root: self.batch.root_seed,
            noise_root: self
                .batch
                .noise_root_seed
                .context("session requires explicit noise root")?,
            exploration_root: self.exploration_root,
            num_envs: self.batch.lanes.len(),
            initial_sha256: self.initial_sha256.clone(),
            events: self.events.clone(),
            learner_checkpoint: String::from_utf8(self.learner.checkpoint(self.next_decision)?)?,
            content_sha256: String::new(),
        };
        snapshot.content_sha256 = digest(&snapshot)?;
        let bytes = serde_json::to_vec(&snapshot)?;
        ensure!(
            bytes.len() <= limits(self.format_version)?.1,
            "session exceeds schema byte limit"
        );
        Ok(bytes)
    }

    /// Reconstructs and verifies the entire training history before returning it.
    ///
    /// Parsing is bounded and does not change an existing session. A digest-valid
    /// but execution-invalid artifact is rejected. A replay mismatch never yields
    /// a partially restored session. Old worlds are not mutated or reused.
    pub fn from_checkpoint(factory: F, workers: usize, bytes: &[u8]) -> Result<Self> {
        ensure!(bytes.len() <= MAX_BYTES, "session exceeds 32 MiB");
        let mut snapshot: Snapshot = serde_json::from_slice(bytes)?;
        let (event_limit, byte_limit) = limits(snapshot.schema_version)?;
        ensure!(
            bytes.len() <= byte_limit,
            "session exceeds schema byte limit"
        );
        ensure!(
            serde_json::from_slice::<serde_json::Value>(bytes)? == serde_json::to_value(&snapshot)?,
            "unsupported session fields or representations"
        );
        ensure!(
            (1..=MAX_SENSOR_BATCH_LANES).contains(&snapshot.num_envs)
                && snapshot.events.len() <= event_limit,
            "invalid session schema/size"
        );
        snapshot.backend.validate()?;
        let (_, next) =
            SensorTableLearner::from_checkpoint(snapshot.learner_checkpoint.as_bytes())?;
        let decisions = snapshot
            .events
            .iter()
            .filter(|event| event.operation == Operation::Train)
            .count();
        ensure!(
            next == decisions as u64,
            "session decision/history mismatch"
        );
        for event in &snapshot.events {
            if let Operation::Reset { lanes } = &event.operation {
                ensure!(
                    !lanes.is_empty()
                        && lanes.len() <= snapshot.num_envs
                        && lanes.iter().all(|(lane, _)| *lane < snapshot.num_envs)
                        && lanes.windows(2).all(|pair| pair[0].0 < pair[1].0),
                    "invalid session reset"
                );
            }
        }
        let expected = std::mem::take(&mut snapshot.content_sha256);
        ensure!(
            expected == digest(&snapshot)?,
            "session content digest mismatch"
        );
        let mut restored = Self::new(
            factory,
            snapshot.physical_root,
            snapshot.noise_root,
            snapshot.exploration_root,
            snapshot.num_envs,
            workers,
        )?;
        ensure!(
            restored.batch.manifest == snapshot.backend
                && restored.initial_sha256 == snapshot.initial_sha256,
            "session initial world/backend mismatch"
        );
        restored.format_version = snapshot.schema_version;
        for expected in snapshot.events {
            match &expected.operation {
                Operation::Train => {
                    restored.step()?;
                }
                Operation::Reset { lanes } => {
                    restored.reset_lanes(lanes)?;
                }
            }
            ensure!(
                restored.events.last() == Some(&expected),
                "session replay evidence mismatch"
            );
        }
        ensure!(
            restored.learner.checkpoint(restored.next_decision)?
                == snapshot.learner_checkpoint.as_bytes(),
            "replayed learner differs from checkpoint"
        );
        Ok(restored)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn captured_reset_factory_failure_preserves_all_existing_lanes() {
        for panic in [false, true] {
            let calls = std::cell::Cell::new(0);
            let fail = std::cell::Cell::new(false);
            let factory = || {
                calls.set(calls.get() + 1);
                if fail.get() && calls.get() == 2 {
                    if panic {
                        panic!("injected replacement construction panic");
                    }
                    anyhow::bail!("injected replacement construction error");
                }
                Ok((
                    rne_physics_rapier::RapierBackend::new(),
                    rne_physics_rapier::RapierBackend::manifest(),
                ))
            };
            let mut session = SensorLearningSession::new(factory, 620, 621, 622, 2, 1).unwrap();
            session.step().unwrap();
            let before = session.checkpoint().unwrap();
            calls.set(0);
            fail.set(true);
            let mut captured = session.capture_reset(&[(0, 3), (1, 4)]).unwrap();
            assert_eq!(calls.get(), 2);
            assert!(captured.outcome.is_err());
            assert_eq!(captured.stage, Some(ResetAttemptStage::Construct));
            assert_eq!(captured.before, captured.after);
            assert!(captured.observations.is_none());
            assert_eq!(captured.before_checkpoint, before);
            calls.set(0);
            // Fresh history constructs two worlds, then reset constructs two more.
            let replay_calls = std::cell::Cell::new(0);
            let replay_factory = || {
                replay_calls.set(replay_calls.get() + 1);
                if replay_calls.get() == 4 {
                    if panic {
                        panic!("injected replacement construction panic");
                    }
                    anyhow::bail!("injected replacement construction error");
                }
                Ok((
                    rne_physics_rapier::RapierBackend::new(),
                    rne_physics_rapier::RapierBackend::manifest(),
                ))
            };
            assert_eq!(
                SensorLearningSession::replay_captured_reset(replay_factory, 2, &captured).unwrap(),
                LearningAttemptReplayOutcome::Reproduced
            );
            // Reproducing the same error is insufficient when recorded progress
            // or learner coordinates have been altered.
            let root = tempfile::tempdir().unwrap();
            let directory = root.path().join("reset-replay");
            let build = rne_log::BuildMetadata::new(
                "test",
                "synthetic-test-build",
                "test",
                "test-target",
                "test-compiler",
                "0".repeat(64),
            );
            captured.write_new(&directory, &build).unwrap();
            replay_calls.set(0);
            assert_eq!(
                SensorLearningSession::replay_failure_capsule(
                    replay_factory,
                    2,
                    &directory,
                    &build
                )
                .unwrap(),
                LearningAttemptReplayOutcome::Reproduced
            );
            captured.updates_after += 1;
            replay_calls.set(0);
            assert_eq!(
                SensorLearningSession::replay_captured_reset(replay_factory, 2, &captured).unwrap(),
                LearningAttemptReplayOutcome::NotReproduced
            );
            captured.updates_after -= 1;
            captured.after[0].progress.completed_drive_ticks += 1;
            replay_calls.set(0);
            assert_eq!(
                SensorLearningSession::replay_captured_reset(replay_factory, 2, &captured).unwrap(),
                LearningAttemptReplayOutcome::NotReproduced
            );
            captured.after[0].progress.completed_drive_ticks -= 1;
            fail.set(false);
            assert_eq!(
                SensorLearningSession::replay_failure_capsule(factory, 2, &directory, &build)
                    .unwrap(),
                LearningAttemptReplayOutcome::NotReproduced
            );
            assert_eq!(
                SensorLearningSession::replay_captured_reset(factory, 2, &captured).unwrap(),
                LearningAttemptReplayOutcome::NotReproduced
            );
            let mut metadata = rne_log::execution_capsule::ExecutionFailureCapsule::decode(
                &std::fs::read(directory.join("capsule.json")).unwrap(),
            )
            .unwrap();
            let mut attempt: serde_json::Value =
                serde_json::from_slice(&std::fs::read(directory.join("attempt.json")).unwrap())
                    .unwrap();
            attempt["updates_after"] = serde_json::json!(captured.updates_after + 1);
            let altered = serde_json::to_vec(&attempt).unwrap();
            metadata.attempt.sha256 = format!("{:x}", Sha256::digest(&altered));
            std::fs::write(directory.join("attempt.json"), altered).unwrap();
            std::fs::write(
                directory.join("capsule.json"),
                serde_json::to_vec(&metadata).unwrap(),
            )
            .unwrap();
            replay_calls.set(0);
            assert_eq!(
                SensorLearningSession::replay_failure_capsule(
                    replay_factory,
                    2,
                    &directory,
                    &build
                )
                .unwrap(),
                LearningAttemptReplayOutcome::NotReproduced
            );
            if panic {
                assert!(session.checkpoint().is_err());
            } else {
                assert_eq!(session.checkpoint().unwrap(), before);
                fail.set(false);
                let reset = session.capture_reset(&[(0, 3)]).unwrap();
                assert!(reset.outcome.is_ok());
                assert_eq!(reset.stage, None);
                assert_eq!(reset.before[1], reset.after[1]);
                let restored = SensorLearningSession::from_checkpoint(
                    factory,
                    2,
                    &session.checkpoint().unwrap(),
                )
                .unwrap();
                assert_eq!(
                    session.checkpoint().unwrap(),
                    restored.checkpoint().unwrap()
                );
            }
        }
    }

    #[test]
    fn captured_reset_distinguishes_rejection_from_applied_evidence_failure() {
        let factory = || {
            Ok((
                rne_physics_rapier::RapierBackend::new(),
                rne_physics_rapier::RapierBackend::manifest(),
            ))
        };
        for panic in [false, true] {
            let mut session = SensorLearningSession::new(factory, 620, 621, 622, 2, 1).unwrap();
            session.step().unwrap();
            let before = session.checkpoint().unwrap();
            assert!(session.capture_reset(&[(0, 3), (0, 3), (0, 3)]).is_err());
            assert_eq!(session.checkpoint().unwrap(), before);
            let rejected = session.capture_reset(&[(2, 3)]).unwrap();
            assert!(rejected.outcome.is_err());
            assert_eq!(rejected.stage, Some(ResetAttemptStage::Construct));
            assert_eq!(rejected.before, rejected.after);
            assert!(rejected.observations.is_none());
            assert_eq!(session.checkpoint().unwrap(), before);

            let updates = session.learner.updates();
            let decision = session.next_decision;
            session.attempt_fault = Some((rne_log::ExecutionStage::Evidence, panic));
            let mut captured = session.capture_reset(&[(0, 3)]).unwrap();
            assert_eq!(captured.before_checkpoint, before);
            assert!(captured.outcome.is_err());
            assert_eq!(captured.stage, Some(ResetAttemptStage::Evidence));
            assert_eq!(captured.decision_index, decision);
            assert_eq!(captured.updates_before, updates);
            assert_eq!(captured.updates_after, updates);
            let root = tempfile::tempdir().unwrap();
            let directory = root.path().join("reset-capsule");
            let build = rne_log::BuildMetadata::new(
                "test",
                "synthetic-test-build",
                "test",
                "test-target",
                "test-compiler",
                "0".repeat(64),
            );
            let capsule = captured.write_new(&directory, &build).unwrap();
            let decoded = rne_log::execution_capsule::ExecutionFailureCapsule::decode(
                &std::fs::read(directory.join("capsule.json")).unwrap(),
            )
            .unwrap();
            assert_eq!(capsule, decoded);
            assert_eq!(
                capsule.replay,
                rne_log::execution_capsule::ExecutionReplayClaim::NotAttempted
            );
            let bytes = crate::observed_execution_capsule::read_execution_artifact(
                &directory,
                &capsule.attempt,
                "execution_attempt",
                "rne_reset_execution_attempt",
                1,
                1024 * 1024,
            )
            .unwrap();
            let attempt: serde_json::Value = serde_json::from_slice(&bytes).unwrap();
            assert_eq!(
                attempt["observations"][0]["latest"],
                serde_json::Value::Null
            );
            assert_eq!(attempt["observations"][0]["time_ticks"], 0);
            assert!(captured.write_new(&directory, &build).is_err());
            assert_eq!(
                std::fs::read(directory.join("attempt.json")).unwrap(),
                bytes
            );
            assert_eq!(captured.observations.as_ref().unwrap().len(), 1);
            captured.observations.as_mut().unwrap()[0].capture_ticks = Some(0);
            let invalid_directory = root.path().join("invalid-reset");
            assert!(captured.write_new(&invalid_directory, &build).is_err());
            assert!(!invalid_directory.exists());
            captured.observations.as_mut().unwrap()[0].capture_ticks = None;
            assert_eq!(captured.after[0].episode_index, 3);
            assert_eq!(captured.after[0].progress.completed_intervals, 0);
            assert_eq!(captured.before[1], captured.after[1]);
            assert_eq!(session.learner.updates(), updates);
            assert_eq!(session.next_decision, decision);
            assert!(session.checkpoint().is_err());
            assert!(session.capture_reset(&[(0, 4)]).is_err());
        }
    }

    #[test]
    fn captured_attempt_preserves_progress_after_learning_and_evidence_boundary_faults() {
        let factory = || {
            Ok((
                rne_physics_rapier::RapierBackend::new(),
                rne_physics_rapier::RapierBackend::manifest(),
            ))
        };
        for stage in [
            rne_log::ExecutionStage::Learning,
            rne_log::ExecutionStage::Evidence,
        ] {
            for panic in [false, true] {
                let mut session = SensorLearningSession::new(factory, 620, 621, 622, 2, 1).unwrap();
                let before = session.checkpoint().unwrap();
                session.attempt_fault = Some((stage, panic));
                let captured = session.capture_step().unwrap();
                assert_eq!(captured.before_checkpoint, before);
                assert!(captured.outcome.is_err());
                assert_eq!(captured.stage, Some(stage));
                assert_eq!(captured.updates_before, 0);
                assert_eq!(
                    captured.updates_after,
                    if stage == rne_log::ExecutionStage::Learning {
                        0
                    } else {
                        2
                    }
                );
                assert_eq!(captured.batch_output.as_ref().unwrap().transitions.len(), 2);
                assert!(captured.batch_output.as_ref().unwrap().failures.is_empty());
                assert!(captured
                    .after
                    .iter()
                    .all(|lane| lane.progress.completed_intervals == 1
                        && lane.progress.last_successful_time_ticks == 10_000_000));
                assert_eq!(session.next_decision, 0);
                assert!(session.events.is_empty());
                assert!(session.checkpoint().is_err());
                assert!(session.capture_step().is_err());
                let root = tempfile::tempdir().unwrap();
                let directory = root.path().join("failure");
                let build = rne_log::BuildMetadata::new(
                    "test",
                    "synthetic-test-build",
                    "test",
                    "test-target",
                    "test-compiler",
                    "0".repeat(64),
                );
                let capsule = captured.write_new(&directory, &build).unwrap();
                let metadata = std::fs::read(directory.join("capsule.json")).unwrap();
                assert_eq!(
                    rne_log::execution_capsule::ExecutionFailureCapsule::decode(&metadata).unwrap(),
                    capsule
                );
                assert_eq!(
                    capsule.replay,
                    rne_log::execution_capsule::ExecutionReplayClaim::NotAttempted
                );
                assert!(matches!(
                    capsule.post_attempt,
                    rne_log::execution_capsule::PostAttemptEvidence::Unavailable { .. }
                ));
                use crate::observed_execution_capsule::read_execution_artifact;
                let history = read_execution_artifact(
                    &directory,
                    &capsule.prior_history,
                    "prior_history",
                    "rne_learning_session",
                    2,
                    MAX_BYTES,
                )
                .unwrap();
                assert_eq!(history, before);
                let contract = read_execution_artifact(
                    &directory,
                    &capsule.contract,
                    "execution_contract",
                    "rne_learning_execution_contract",
                    1,
                    1024 * 1024,
                )
                .unwrap();
                let contract: serde_json::Value = serde_json::from_slice(&contract).unwrap();
                assert_eq!(contract["build"], serde_json::to_value(&build).unwrap());
                assert_eq!(
                    contract["task_spec"],
                    serde_json::to_value(crate::observed::sensor_fixed_task_spec()).unwrap()
                );
                let attempt = read_execution_artifact(
                    &directory,
                    &capsule.attempt,
                    "execution_attempt",
                    "rne_learning_execution_attempt",
                    1,
                    1024 * 1024,
                )
                .unwrap();
                let decoded: serde_json::Value = serde_json::from_slice(&attempt).unwrap();
                assert_eq!(decoded["stage"], serde_json::to_value(stage).unwrap());
                assert_eq!(decoded["updates_after"], captured.updates_after);
                assert!(captured.write_new(&directory, &build).is_err());
                assert_eq!(
                    std::fs::read(directory.join("capsule.json")).unwrap(),
                    metadata
                );
                assert_eq!(
                    std::fs::read(directory.join("attempt.json")).unwrap(),
                    attempt
                );
                std::fs::write(directory.join("attempt.json"), b"{}").unwrap();
                assert!(read_execution_artifact(
                    &directory,
                    &capsule.attempt,
                    "execution_attempt",
                    "rne_learning_execution_attempt",
                    1,
                    1024 * 1024
                )
                .is_err());
                // Restoring valid history does not reproduce this newly injected fault.
                let mut restored =
                    SensorLearningSession::from_checkpoint(factory, 2, &before).unwrap();
                let success = restored.capture_step().unwrap();
                assert!(success.outcome.is_ok());
                let successful_directory = root.path().join("not-a-failure");
                assert!(success.write_new(&successful_directory, &build).is_err());
                assert!(!successful_directory.exists());
            }
        }
    }

    #[test]
    fn captured_attempt_rejects_unusable_session_before_execution() {
        let mut session = SensorLearningSession::new(
            || {
                Ok((
                    rne_physics_rapier::RapierBackend::new(),
                    rne_physics_rapier::RapierBackend::manifest(),
                ))
            },
            620,
            621,
            622,
            2,
            1,
        )
        .unwrap();
        let before = session.execution_progress();
        session.usable = false;
        assert!(session.capture_step().is_err());
        assert_eq!(session.execution_progress(), before);
        assert_eq!(session.learner().updates(), 0);
        assert_eq!(session.next_decision, 0);
    }

    #[test]
    fn captured_attempt_invalid_bindings_create_no_directory() {
        let build = rne_log::BuildMetadata::new(
            "test",
            "synthetic-test-build",
            "test",
            "test-target",
            "test-compiler",
            "0".repeat(64),
        );
        for mutation in 0..5 {
            let mut session = SensorLearningSession::new(
                || {
                    Ok((
                        rne_physics_rapier::RapierBackend::new(),
                        rne_physics_rapier::RapierBackend::manifest(),
                    ))
                },
                620,
                621,
                622,
                2,
                1,
            )
            .unwrap();
            session.attempt_fault = Some((rne_log::ExecutionStage::Evidence, false));
            let mut capture = session.capture_step().unwrap();
            match mutation {
                0 => {
                    let mut prior: serde_json::Value =
                        serde_json::from_slice(&capture.before_checkpoint).unwrap();
                    prior["physical_root"] = serde_json::json!(999);
                    capture.before_checkpoint = serde_json::to_vec(&prior).unwrap();
                }
                1 => capture.decision_index += 1,
                2 => capture.updates_before += 1,
                3 => capture.after[0].episode_seed += 1,
                4 => capture.requested_actions_v.as_mut().unwrap()[0] = f64::NAN,
                _ => unreachable!(),
            }
            let root = tempfile::tempdir().unwrap();
            let directory = root.path().join("invalid");
            assert!(
                capture.write_new(&directory, &build).is_err(),
                "mutation {mutation}"
            );
            assert!(!directory.exists());
        }
    }

    fn resume<B, F>(factory: F)
    where
        B: PhysicsBackend,
        F: Fn() -> Result<(B, PhysicsBackendManifest)> + Copy,
    {
        let mut live = SensorLearningSession::new(factory, 620, 621, 622, 2, 2).unwrap();
        for index in 0..30 {
            if index == 15 {
                live.reset_lanes(&[(1, 3)]).unwrap();
            }
            assert!(live.step().unwrap().failures.is_empty());
        }
        let bytes = live.checkpoint().unwrap();
        let mut restored = SensorLearningSession::from_checkpoint(factory, 1, &bytes).unwrap();
        assert_eq!(bytes, restored.checkpoint().unwrap());
        let mut legacy: Snapshot = serde_json::from_slice(&bytes).unwrap();
        legacy.schema_version = 1;
        legacy.content_sha256.clear();
        legacy.content_sha256 = digest(&legacy).unwrap();
        let legacy_bytes = serde_json::to_vec(&legacy).unwrap();
        let legacy_session =
            SensorLearningSession::from_checkpoint(factory, 1, &legacy_bytes).unwrap();
        assert_eq!(legacy_session.format_version, 1);
        assert_eq!(legacy_bytes, legacy_session.checkpoint().unwrap());
        for _ in 0..30 {
            assert_eq!(live.step().unwrap(), restored.step().unwrap());
        }
        assert_eq!(live.checkpoint().unwrap(), restored.checkpoint().unwrap());
        assert_eq!(live.learner().updates(), 120);
        let mut wire: Snapshot = serde_json::from_slice(&bytes).unwrap();
        wire.events[0].evidence_sha256 = "sha256:wrong".into();
        wire.content_sha256.clear();
        wire.content_sha256 = digest(&wire).unwrap();
        assert!(SensorLearningSession::from_checkpoint(
            factory,
            1,
            &serde_json::to_vec(&wire).unwrap()
        )
        .is_err());
        let saved = live.checkpoint().unwrap();
        assert!(live.reset_lanes(&[(2, 1)]).is_err());
        assert_eq!(saved, live.checkpoint().unwrap());
        let mut wire: Snapshot = serde_json::from_slice(&bytes).unwrap();
        wire.learner_checkpoint =
            String::from_utf8(SensorTableLearner::new(622).checkpoint(30).unwrap()).unwrap();
        wire.content_sha256.clear();
        wire.content_sha256 = digest(&wire).unwrap();
        assert!(SensorLearningSession::from_checkpoint(
            factory,
            1,
            &serde_json::to_vec(&wire).unwrap()
        )
        .is_err());
    }

    #[test]
    fn training_session_bounds_and_horizon_reject_without_progress() {
        let factory = || {
            Ok((
                rne_physics_rapier::RapierBackend::new(),
                rne_physics_rapier::RapierBackend::manifest(),
            ))
        };
        let mut session = SensorLearningSession::new(factory, 620, 621, 622, 1, 1).unwrap();
        for _ in 0..330 {
            session.step().unwrap();
        }
        let saved = session.checkpoint().unwrap();
        assert!(session.step().is_err());
        assert_eq!(saved, session.checkpoint().unwrap());
        session.reset_lanes(&[(0, 1)]).unwrap();
        session.step().unwrap();
        // Synthetic full history exercises the preflight bound, not provenance.
        session.events.resize(MAX_EVENTS, session.events[0].clone());
        let observations = session.batch.observations();
        let ticks = observations[0].as_ref().unwrap().time_ticks;
        let updates = session.learner().updates();
        assert!(session.step().is_err());
        assert!(session.reset_lanes(&[(0, 2)]).is_err());
        assert_eq!(updates, session.learner().updates());
        assert_eq!(
            ticks,
            session.batch.observations()[0].as_ref().unwrap().time_ticks
        );
        let no_factory =
            || -> Result<(rne_physics_rapier::RapierBackend, PhysicsBackendManifest)> {
                panic!("invalid metadata must not construct a backend")
            };
        assert!(
            SensorLearningSession::from_checkpoint(no_factory, 1, &vec![b' '; MAX_BYTES + 1])
                .is_err()
        );
        let mut wire: serde_json::Value = serde_json::from_slice(&saved).unwrap();
        wire["unknown"] = true.into();
        assert!(SensorLearningSession::from_checkpoint(
            no_factory,
            1,
            &serde_json::to_vec(&wire).unwrap()
        )
        .is_err());
    }

    #[test]
    fn training_session_rapier_replays_learning_and_resumes() {
        resume(|| {
            Ok((
                rne_physics_rapier::RapierBackend::new(),
                rne_physics_rapier::RapierBackend::manifest(),
            ))
        });
    }

    fn full_training_job<B, F>(factory: F)
    where
        B: PhysicsBackend,
        F: Fn() -> Result<(B, PhysicsBackendManifest)> + Copy,
    {
        // Recovery qualification only: these are not held-out performance seeds.
        let mut session = SensorLearningSession::new(factory, 5000, 5001, 5002, 4, 2).unwrap();
        for episode in 0..32 {
            if episode != 0 {
                session
                    .reset_lanes(&(0..4).map(|lane| (lane, episode)).collect::<Vec<_>>())
                    .unwrap();
            }
            for _ in 0..330 {
                assert!(session.step().unwrap().failures.is_empty());
            }
        }
        assert_eq!(session.events.len(), 10_591);
        assert_eq!(session.learner().updates(), 42_240);
        let bytes = session.checkpoint().unwrap();
        assert!(bytes.len() <= MAX_BYTES);
        let mut restored = SensorLearningSession::from_checkpoint(factory, 1, &bytes).unwrap();
        assert_eq!(bytes, restored.checkpoint().unwrap());
        let reset = (0..4).map(|lane| (lane, 32)).collect::<Vec<_>>();
        session.reset_lanes(&reset).unwrap();
        restored.reset_lanes(&reset).unwrap();
        assert_eq!(session.step().unwrap(), restored.step().unwrap());
        assert_eq!(
            session.checkpoint().unwrap(),
            restored.checkpoint().unwrap()
        );
        eprintln!(
            "full training job verified: operations=10591 updates=42240 checkpoint_bytes={}",
            bytes.len()
        );
    }

    #[test]
    #[ignore = "long physical training/replay qualification; run explicitly"]
    fn training_session_full_training_job_rapier() {
        full_training_job(|| {
            Ok((
                rne_physics_rapier::RapierBackend::new(),
                rne_physics_rapier::RapierBackend::manifest(),
            ))
        });
    }

    #[cfg(feature = "mujoco")]
    #[test]
    #[ignore = "long physical training/replay qualification; run explicitly"]
    fn training_session_full_training_job_mujoco() {
        full_training_job(|| {
            Ok((
                rne_physics_mujoco::MuJoCoBackend::new(rne_core::SimDuration::from_ticks(
                    1_000_000,
                ))?,
                rne_physics_mujoco::MuJoCoBackend::manifest(),
            ))
        });
    }

    #[cfg(feature = "mujoco")]
    #[test]
    fn training_session_mujoco_replays_learning_and_resumes() {
        resume(|| {
            Ok((
                rne_physics_mujoco::MuJoCoBackend::new(rne_core::SimDuration::from_ticks(
                    1_000_000,
                ))?,
                rne_physics_mujoco::MuJoCoBackend::manifest(),
            ))
        });
    }
}
