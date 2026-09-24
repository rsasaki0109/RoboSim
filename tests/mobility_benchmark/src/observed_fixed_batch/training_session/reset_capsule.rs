//! Reset-specific execution artifacts, with explicit unavailable physical state.

use super::*;
use rne_log::execution_capsule::{
    ExecutionFailureCapsule, ExecutionReplayClaim, PostAttemptEvidence,
};
use std::io::Write;

impl CapturedResetAttempt {
    pub(super) fn reset_attempt_value(&self) -> Result<serde_json::Value> {
        let error = self
            .outcome
            .as_ref()
            .err()
            .context("reset was successful")?;
        let observations = self.observations.as_ref().map(|values| {
            values.iter().map(|value| {
                ensure!(value.latest.is_none() && value.time_ticks == 0
                    && value.capture_ticks.is_none() && value.input_age_ticks.is_none()
                    && !value.fresh_estimate, "reset observation is not an initial missing estimate");
                Ok(serde_json::json!({
                    "time_ticks": value.time_ticks, "latest": null,
                    "capture_ticks": value.capture_ticks, "input_age_ticks": value.input_age_ticks,
                    "fresh_estimate": value.fresh_estimate,
                }))
            }).collect::<Result<Vec<_>>>()
        }).transpose()?;
        Ok(serde_json::json!({
            "kind": "rne_reset_execution_attempt", "schema_version": 1, "operation": "reset",
            "requested_lanes": self.requested_lanes, "decision_index": self.decision_index,
            "updates_before": self.updates_before, "updates_after": self.updates_after,
            "before": self.before, "after": self.after, "observations": observations,
            "stage": self.stage, "error": error,
        }))
    }

    /// Writes a failed reset into a new directory, with metadata written last.
    /// References bind exact bytes, not producer authenticity or physical replay.
    /// Existing directories are never overwritten; partial files remain on error.
    #[allow(clippy::too_many_lines)] // TODO(cleanup): split (167/150 lines); see PR body
    pub fn write_new(
        &self,
        directory: &std::path::Path,
        build: &rne_log::BuildMetadata,
    ) -> Result<ExecutionFailureCapsule> {
        ensure!(
            self.before_checkpoint.len() <= MAX_BYTES,
            "reset history byte limit"
        );
        let mut prior: Snapshot = serde_json::from_slice(&self.before_checkpoint)?;
        let (event_limit, byte_limit) = limits(prior.schema_version)?;
        ensure!(
            self.before_checkpoint.len() <= byte_limit && prior.events.len() <= event_limit,
            "reset history bounds"
        );
        ensure!(
            (1..=MAX_SENSOR_BATCH_LANES).contains(&prior.num_envs),
            "reset history width"
        );
        prior.backend.validate()?;
        let expected = std::mem::take(&mut prior.content_sha256);
        ensure!(digest(&prior)? == expected, "reset history digest mismatch");
        let (learner, next) =
            SensorTableLearner::from_checkpoint(prior.learner_checkpoint.as_bytes())?;
        ensure!(
            next == self.decision_index
                && next
                    == prior
                        .events
                        .iter()
                        .filter(|event| event.operation == Operation::Train)
                        .count() as u64,
            "reset decision mismatch"
        );
        ensure!(
            learner.updates() == self.updates_before && self.updates_after == self.updates_before,
            "reset learner update mismatch"
        );
        ensure!(
            self.before.len() == prior.num_envs
                && self.after.len() == prior.num_envs
                && self.requested_lanes.len() <= prior.num_envs,
            "reset capture width"
        );
        ensure!(self.stage.is_some(), "failed reset has no stage");
        for (index, (before, after)) in self.before.iter().zip(&self.after).enumerate() {
            for lane in [before, after] {
                ensure!(
                    lane.lane_id == index
                        && lane.episode_seed
                            == derive_episode_seed(
                                prior.physical_root,
                                index as u64,
                                lane.episode_index
                            ),
                    "reset lane identity mismatch"
                );
            }
        }
        if self.stage == Some(ResetAttemptStage::Construct) {
            ensure!(
                self.before == self.after && self.observations.is_none(),
                "construction failure changed lanes"
            );
        } else {
            ensure!(
                !self.requested_lanes.is_empty()
                    && self
                        .requested_lanes
                        .windows(2)
                        .all(|pair| pair[0].0 < pair[1].0),
                "invalid applied reset mask"
            );
            let observations = self
                .observations
                .as_ref()
                .context("applied reset lacks observations")?;
            ensure!(
                observations.len() == self.requested_lanes.len(),
                "reset observation width"
            );
            for &(id, episode) in &self.requested_lanes {
                let lane = self
                    .after
                    .get(id)
                    .context("applied reset lane out of range")?;
                ensure!(
                    lane.episode_index == episode
                        && lane.progress == rne_log::ExecutionProgress::default(),
                    "applied reset progress mismatch"
                );
            }
            for (before, after) in self.before.iter().zip(&self.after) {
                if !self
                    .requested_lanes
                    .iter()
                    .any(|(id, _)| *id == before.lane_id)
                {
                    ensure!(before == after, "unselected reset lane changed");
                }
            }
        }
        let attempt = serde_json::to_vec(&self.reset_attempt_value()?)?;
        let contract = serde_json::to_vec(&serde_json::json!({
            "kind": "rne_learning_execution_contract", "schema_version": 1,
            "backend": prior.backend, "task_spec": crate::observed::sensor_fixed_task_spec(), "build": build,
        }))?;
        ensure!(
            attempt.len() <= 1024 * 1024 && contract.len() <= 1024 * 1024,
            "reset metadata byte limit"
        );
        let reference = |role, kind, version, path, bytes: &[u8]| {
            rne_log::ArtifactRef::new(
                role,
                kind,
                version,
                path,
                format!("{:x}", Sha256::digest(bytes)),
            )
        };
        let capsule = ExecutionFailureCapsule {
            kind: "rne_execution_failure_capsule".into(),
            schema_version: 1,
            failure_code: "reset_attempt_error".into(),
            message: self.outcome.as_ref().unwrap_err().clone(),
            contract: reference(
                "execution_contract",
                "rne_learning_execution_contract",
                1,
                "contract.json",
                &contract,
            )?,
            prior_history: reference(
                "prior_history",
                "rne_learning_session",
                prior.schema_version,
                "prior-history.json",
                &self.before_checkpoint,
            )?,
            attempt: reference(
                "execution_attempt",
                "rne_reset_execution_attempt",
                1,
                "attempt.json",
                &attempt,
            )?,
            post_attempt: PostAttemptEvidence::Unavailable {
                reason: "reset capture contains diagnostics, not a physical solver snapshot".into(),
            },
            replay: ExecutionReplayClaim::NotAttempted,
        };
        capsule.validate()?;
        let metadata = serde_json::to_vec(&capsule)?;
        ensure!(
            metadata.len() <= rne_log::execution_capsule::MAX_EXECUTION_CAPSULE_BYTES,
            "reset capsule byte limit"
        );
        std::fs::create_dir(directory)?;
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
