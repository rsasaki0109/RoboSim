//! Replay-verified training continuation, not opaque solver-state serialization.

use super::*;
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};

const MAX_BYTES: usize = 8 * 1024 * 1024;
const MAX_EVENTS: usize = 1024;

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
/// Restoration re-executes at most 1,024 operations from fresh worlds, validates
/// each physical/sensor/output/learner digest and the final learner checkpoint,
/// then returns live worlds. It does not trust a hash as proof of execution.
/// Worker count may change; backend identity may not. No solver handles or hidden
/// physical parameters enter the learner. This is not an O(1) solver snapshot.
pub struct SensorLearningSession<B: PhysicsBackend, F> {
    batch: SensorFixedBatch<B, F>,
    learner: SensorTableLearner,
    exploration_root: u64,
    next_decision: u64,
    initial_sha256: String,
    events: Vec<Event>,
    usable: bool,
}

impl<B: PhysicsBackend, F> std::fmt::Debug for SensorLearningSession<B, F> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("SensorLearningSession")
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
            batch,
            learner: SensorTableLearner::new(exploration_root),
            exploration_root,
            next_decision: 0,
            initial_sha256,
            events: Vec::new(),
            usable: true,
        })
    }

    /// Read-only learned policy; callers cannot mutate session learning state.
    pub fn learner(&self) -> &SensorTableLearner {
        &self.learner
    }

    fn preflight(&self) -> Result<()> {
        ensure!(
            self.usable,
            "session has an unrecorded execution/update failure"
        );
        ensure!(
            self.events.len() < MAX_EVENTS,
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
        self.preflight()?;
        let next = self
            .next_decision
            .checked_add(1)
            .context("decision counter overflow")?;
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
        self.usable = false;
        let output = self.batch.step_learning(&actions)?;
        self.learner.learn(&output.transitions)?;
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
        Ok(output)
    }

    /// Resets selected lanes without consuming an exploration coordinate or update.
    /// Batch reset validation/construction is atomic. A post-reset evidence error
    /// invalidates the session, because physical reset has already occurred.
    pub fn reset_lanes(&mut self, lanes: &[(usize, u64)]) -> Result<Vec<SensorFixedObservation>> {
        self.preflight()?;
        let observations = self.batch.reset_lanes(lanes)?;
        self.usable = false;
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
        Ok(observations)
    }

    /// Encodes at most 8 MiB of history and learner state. No filesystem is touched.
    /// History cannot be discarded to bypass the 1,024-operation bound. A failed
    /// lane can be saved, but restoration requires that failure to reproduce.
    /// Digests are corruption checks, not signatures or proof of hardware behavior.
    pub fn checkpoint(&self) -> Result<Vec<u8>> {
        ensure!(self.usable, "cannot checkpoint unrecorded partial progress");
        let mut snapshot = Snapshot {
            schema_version: 1,
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
        ensure!(bytes.len() <= MAX_BYTES, "session exceeds 8 MiB");
        Ok(bytes)
    }

    /// Reconstructs and verifies the entire training history before returning it.
    ///
    /// Parsing is bounded and does not change an existing session. A digest-valid
    /// but execution-invalid artifact is rejected. A replay mismatch never yields
    /// a partially restored session. Old worlds are not mutated or reused.
    pub fn from_checkpoint(factory: F, workers: usize, bytes: &[u8]) -> Result<Self> {
        ensure!(bytes.len() <= MAX_BYTES, "session exceeds 8 MiB");
        let mut snapshot: Snapshot = serde_json::from_slice(bytes)?;
        ensure!(
            serde_json::from_slice::<serde_json::Value>(bytes)? == serde_json::to_value(&snapshot)?,
            "unsupported session fields or representations"
        );
        ensure!(
            snapshot.schema_version == 1
                && (1..=MAX_SENSOR_BATCH_LANES).contains(&snapshot.num_envs)
                && snapshot.events.len() <= MAX_EVENTS,
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
