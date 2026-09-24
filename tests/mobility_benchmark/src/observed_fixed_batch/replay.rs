//! Bounded action/reset replay with exact per-operation evidence hashes.

use super::*;
use crate::observed::sensor_fixed_task_spec;
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};

/// Maximum serialized replay accepted by the bounded decoder (16 MiB).
pub const MAX_FIXED_REPLAY_BYTES: usize = 16 * 1024 * 1024;
const MAX_OPERATIONS: usize = 1024;

/// One explicit operation; initial reset is implicit at episode index zero.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "operation", rename_all = "snake_case", deny_unknown_fields)]
pub enum FixedBatchOperation {
    /// One voltage per lane, held over the next 10 ms interval.
    Step {
        /// Stable lane-ordered voltages, within [-24, 24] V.
        actions_v: Vec<f64>,
    },
    /// Partial reset without advancing unselected lanes.
    Reset {
        /// Strictly increasing lane IDs paired with explicit episode indices.
        lanes: Vec<(usize, u64)>,
    },
}

/// An operation and the SHA-256 of its complete evaluator result/state projection.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct FixedBatchReplayEvent {
    /// Operation executed on persistent physical worlds.
    pub operation: FixedBatchOperation,
    /// Digest binds observations, rewards, timing/termination, lane identities,
    /// exact reset contracts and completed physical-state hashes (or lane errors).
    pub evidence_sha256: String,
}

/// Compact fixed-period replay proof. Hashes bind results but are not signatures
/// or full raw traces; verification must execute fresh physics, not just parse JSON.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct FixedBatchReplay {
    /// Stable artifact discriminator.
    pub kind: String,
    /// Replay schema version.
    pub schema_version: u32,
    /// Exact common backend manifest.
    pub backend: PhysicsBackendManifest,
    /// Exact fixed actor/action/reward/horizon contract.
    pub task_spec: rne_ai::TaskSpec,
    /// Explicit root seed for lane/episode reset derivation.
    pub root_seed: u64,
    /// Legacy v1 fixed `WorldRandom` seed. Must remain zero; in v2 the optional
    /// noise root below replaces this field's seed-selection behavior.
    pub world_noise_seed: u64,
    /// Independent lane/episode noise root in schema v2. Absent in legacy v1,
    /// which uses `world_noise_seed` zero. V2 retains that legacy field as zero.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub noise_root_seed: Option<u64>,
    /// Number of persistent worlds.
    pub num_envs: usize,
    /// Initial reset projection, including complete joint-reset contracts.
    pub initial_evidence_sha256: String,
    /// Ordered operation/result evidence. Worker topology is intentionally absent.
    pub events: Vec<FixedBatchReplayEvent>,
    /// SHA-256 of compact JSON with this field empty.
    pub content_sha256: String,
}

impl FixedBatchReplay {
    /// Validates bounded shape, operations, backend/TaskSpec and content integrity.
    /// Does not prove that recorded outcomes follow from the commands.
    pub fn validate_metadata(&self) -> Result<()> {
        ensure!(
            self.kind == "rne_fixed_sensor_batch_replay"
                && self.schema_version == if self.noise_root_seed.is_some() { 2 } else { 1 },
            "fixed replay kind/schema mismatch"
        );
        ensure!(
            self.task_spec == sensor_fixed_task_spec(),
            "fixed replay TaskSpec mismatch"
        );
        self.backend.validate()?;
        ensure!(
            self.world_noise_seed == crate::observed::WORLD_SEED,
            "fixed replay noise seed mismatch"
        );
        validate_operations(
            self.num_envs,
            self.events.iter().map(|event| &event.operation),
        )?;
        ensure!(
            is_digest(&self.initial_evidence_sha256)
                && self
                    .events
                    .iter()
                    .all(|event| is_digest(&event.evidence_sha256)),
            "invalid evidence digest"
        );
        ensure!(
            self.content_sha256 == content_digest(self)?,
            "fixed replay content digest mismatch"
        );
        Ok(())
    }
}

/// Decodes at most 16 MiB, rejects unknown fields/schema, and validates metadata.
pub fn decode_fixed_batch_replay(bytes: &[u8]) -> Result<FixedBatchReplay> {
    ensure!(
        bytes.len() <= MAX_FIXED_REPLAY_BYTES,
        "fixed replay exceeds byte limit"
    );
    let replay: FixedBatchReplay = serde_json::from_slice(bytes).context("decode fixed replay")?;
    replay.validate_metadata()?;
    Ok(replay)
}

/// Runs bounded operations on fresh persistent worlds and records evidence digests.
/// Reset contracts are hashed from the actual environments, not inferred solely
/// from the generic `TaskSpec`. Preflight/constructor errors abort; lane execution
/// errors remain bound in the event and can be followed by an explicit reset.
pub fn record_fixed_batch_replay<B, F>(
    factory: F,
    root_seed: u64,
    num_envs: usize,
    workers: usize,
    operations: &[FixedBatchOperation],
) -> Result<FixedBatchReplay>
where
    B: PhysicsBackend,
    F: Fn() -> Result<(B, PhysicsBackendManifest)>,
{
    record_with_noise_policy(factory, root_seed, None, num_envs, workers, operations)
}

/// Records schema-v2 evidence with independently controlled lane/episode noise.
/// Reset operations derive new noise seeds only for selected lanes.
pub fn record_fixed_batch_replay_with_noise_root<B, F>(
    factory: F,
    root_seed: u64,
    noise_root_seed: u64,
    num_envs: usize,
    workers: usize,
    operations: &[FixedBatchOperation],
) -> Result<FixedBatchReplay>
where
    B: PhysicsBackend,
    F: Fn() -> Result<(B, PhysicsBackendManifest)>,
{
    record_with_noise_policy(
        factory,
        root_seed,
        Some(noise_root_seed),
        num_envs,
        workers,
        operations,
    )
}

fn record_with_noise_policy<B, F>(
    factory: F,
    root_seed: u64,
    noise_root_seed: Option<u64>,
    num_envs: usize,
    workers: usize,
    operations: &[FixedBatchOperation],
) -> Result<FixedBatchReplay>
where
    B: PhysicsBackend,
    F: Fn() -> Result<(B, PhysicsBackendManifest)>,
{
    validate_operations(num_envs, operations.iter())?;
    let mut batch = SensorFixedBatch::with_noise_policy(
        factory,
        root_seed,
        noise_root_seed,
        num_envs,
        workers,
    )?;
    let initial_evidence_sha256 = evidence_digest(&batch, serde_json::Value::Null)?;
    let mut events = Vec::with_capacity(operations.len());
    for operation in operations {
        let output = match operation {
            FixedBatchOperation::Step { actions_v } => {
                let outcomes = batch.step(actions_v)?;
                serde_json::Value::Array(outcomes.into_iter().map(|lane| {
                    match lane.outcome {
                        Ok(step) => serde_json::json!({"lane_id": lane.lane_id, "episode_index": lane.episode_index,
                            "episode_seed": lane.episode_seed, "actor_tensors": step.observation.actor_tensors(),
                            "time_ticks": step.observation.time_ticks, "reward": step.reward,
                            "tracking_error_integral_m": step.privileged_tracking_error_integral_m, "truncated": step.truncated}),
                        Err(error) => serde_json::json!({"lane_id": lane.lane_id, "episode_index": lane.episode_index,
                            "episode_seed": lane.episode_seed, "error": error}),
                    }
                }).collect())
            }
            FixedBatchOperation::Reset { lanes } => {
                let reset = batch.reset_lanes(lanes)?;
                serde_json::json!({"reset_actor_tensors": reset.iter().map(SensorFixedObservation::actor_tensors).collect::<Vec<_>>()})
            }
        };
        events.push(FixedBatchReplayEvent {
            operation: operation.clone(),
            evidence_sha256: evidence_digest(&batch, output)?,
        });
    }
    let mut replay = FixedBatchReplay {
        kind: "rne_fixed_sensor_batch_replay".into(),
        schema_version: if noise_root_seed.is_some() { 2 } else { 1 },
        backend: batch.manifest.clone(),
        task_spec: sensor_fixed_task_spec(),
        root_seed,
        world_noise_seed: crate::observed::WORLD_SEED,
        noise_root_seed,
        num_envs,
        initial_evidence_sha256,
        events,
        content_sha256: String::new(),
    };
    replay.content_sha256 = content_digest(&replay)?;
    replay.validate_metadata()?;
    ensure!(
        serde_json::to_vec(&replay)?.len() <= MAX_FIXED_REPLAY_BYTES,
        "generated replay exceeds byte limit"
    );
    Ok(replay)
}

/// Re-executes operations from time zero and compares every result/state digest.
/// A different worker count must reproduce identical evidence. This is not a
/// portable solver snapshot or a cross-backend equality test.
pub fn verify_fixed_batch_replay<B, F>(
    factory: F,
    workers: usize,
    source: &FixedBatchReplay,
) -> Result<()>
where
    B: PhysicsBackend,
    F: Fn() -> Result<(B, PhysicsBackendManifest)>,
{
    source.validate_metadata()?;
    let operations: Vec<_> = source
        .events
        .iter()
        .map(|event| event.operation.clone())
        .collect();
    let checked_factory = || {
        let (backend, manifest) = factory()?;
        ensure!(
            manifest == source.backend,
            "fixed replay backend identity mismatch"
        );
        Ok((backend, manifest))
    };
    let actual = record_with_noise_policy(
        checked_factory,
        source.root_seed,
        source.noise_root_seed,
        source.num_envs,
        workers,
        &operations,
    )?;
    ensure!(
        actual == *source,
        "fixed physical replay differs from recorded evidence"
    );
    Ok(())
}

fn validate_operations<'a>(
    width: usize,
    operations: impl Iterator<Item = &'a FixedBatchOperation>,
) -> Result<()> {
    ensure!(
        (1..=MAX_SENSOR_BATCH_LANES).contains(&width),
        "invalid replay width"
    );
    let mut count = 0;
    for operation in operations {
        count += 1;
        ensure!(
            count <= MAX_OPERATIONS,
            "fixed replay exceeds operation limit"
        );
        match operation {
            FixedBatchOperation::Step { actions_v } => {
                ensure!(
                    actions_v.len() == width
                        && actions_v
                            .iter()
                            .all(|v| v.is_finite() && (-24.0..=24.0).contains(v)),
                    "invalid replay action batch"
                );
            }
            FixedBatchOperation::Reset { lanes } => {
                ensure!(
                    !lanes.is_empty()
                        && lanes.len() <= width
                        && lanes.iter().all(|(id, _)| *id < width)
                        && lanes.windows(2).all(|pair| pair[0].0 < pair[1].0),
                    "invalid replay reset mask"
                );
            }
        }
    }
    ensure!(count > 0, "empty fixed replay");
    Ok(())
}

pub(super) fn evidence_digest<B, F>(
    batch: &SensorFixedBatch<B, F>,
    output: serde_json::Value,
) -> Result<String>
where
    B: PhysicsBackend,
    F: Fn() -> Result<(B, PhysicsBackendManifest)>,
{
    let lanes: Vec<_> = batch.lanes.iter().enumerate().map(|(id, lane)| {
        serde_json::json!({"lane_id": id, "episode_index": lane.episode_index, "episode_seed": lane.episode_seed,
            "reset_contract": lane.environment.reset_contract(),
            "actor_tensors": lane.environment.observation().map(|value| value.actor_tensors()).map_err(|error| error.to_string()),
            "physics_hash_v2": lane.environment.privileged_physics_hash_v2().map_err(|error| error.to_string())})
    }).collect();
    hash(&serde_json::json!({"output": output, "lanes": lanes}))
}

fn hash(value: &impl Serialize) -> Result<String> {
    Ok(format!(
        "sha256:{:x}",
        Sha256::digest(serde_json::to_vec(value)?)
    ))
}
fn content_digest(source: &FixedBatchReplay) -> Result<String> {
    let mut normalized = source.clone();
    normalized.content_sha256.clear();
    hash(&normalized)
}
fn is_digest(value: &str) -> bool {
    value.strip_prefix("sha256:").is_some_and(|hex| {
        hex.len() == 64
            && hex
                .bytes()
                .all(|b| b.is_ascii_digit() || (b'a'..=b'f').contains(&b))
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    #[cfg(feature = "mujoco")]
    #[test]
    fn independent_noise_mujoco_replays_across_workers_and_reset() {
        let factory = || -> Result<_> {
            Ok((
                rne_physics_mujoco::MuJoCoBackend::new(rne_core::SimDuration::from_ticks(
                    1_000_000,
                ))?,
                rne_physics_mujoco::MuJoCoBackend::manifest(),
            ))
        };
        let mut operations = vec![
            FixedBatchOperation::Step {
                actions_v: vec![3.0, 3.0]
            };
            6
        ];
        operations.push(FixedBatchOperation::Reset {
            lanes: vec![(0, 4)],
        });
        operations.extend(vec![
            FixedBatchOperation::Step {
                actions_v: vec![3.0, 3.0]
            };
            6
        ]);
        let source =
            record_fixed_batch_replay_with_noise_root(factory, 42, 99, 2, 1, &operations).unwrap();
        verify_fixed_batch_replay(factory, 2, &source).unwrap();
    }

    #[test]
    fn independent_noise_replay_is_worker_invariant_and_binds_root() {
        let mut operations = vec![
            FixedBatchOperation::Step {
                actions_v: vec![3.0, 3.0]
            };
            6
        ];
        operations.push(FixedBatchOperation::Reset {
            lanes: vec![(0, 4)],
        });
        operations.extend(vec![
            FixedBatchOperation::Step {
                actions_v: vec![3.0, 3.0]
            };
            6
        ]);
        let source =
            record_fixed_batch_replay_with_noise_root(factory, 42, 99, 2, 1, &operations).unwrap();
        assert_eq!(source.schema_version, 2);
        verify_fixed_batch_replay(factory, 2, &source).unwrap();
        let changed =
            record_fixed_batch_replay_with_noise_root(factory, 42, 100, 2, 2, &operations).unwrap();
        assert_ne!(
            source.initial_evidence_sha256,
            changed.initial_evidence_sha256
        );
        assert_ne!(source.events, changed.events);
        let mut forged = source.clone();
        forged.noise_root_seed = Some(100);
        forged.content_sha256 = content_digest(&forged).unwrap();
        forged.validate_metadata().unwrap();
        assert!(verify_fixed_batch_replay(factory, 1, &forged).is_err());
        forged.schema_version = 1;
        forged.content_sha256 = content_digest(&forged).unwrap();
        assert!(forged.validate_metadata().is_err());
        assert_eq!(
            decode_fixed_batch_replay(&serde_json::to_vec(&source).unwrap()).unwrap(),
            source
        );
    }

    use rne_physics_rapier::RapierBackend;

    fn factory() -> Result<(RapierBackend, PhysicsBackendManifest)> {
        Ok((RapierBackend::new(), RapierBackend::manifest()))
    }

    fn operations() -> Vec<FixedBatchOperation> {
        let mut operations = vec![
            FixedBatchOperation::Step {
                actions_v: vec![3.0, 2.0]
            };
            30
        ];
        operations.push(FixedBatchOperation::Reset {
            lanes: vec![(1, 7)],
        });
        operations.extend(vec![
            FixedBatchOperation::Step {
                actions_v: vec![3.0, 2.0]
            };
            300
        ]);
        operations
    }

    #[test]
    fn persistent_history_replays_resets_rewards_and_physics_across_workers() {
        let source = record_fixed_batch_replay(factory, 42, 2, 1, &operations()).unwrap();
        let parallel = record_fixed_batch_replay(factory, 42, 2, 2, &operations()).unwrap();
        assert_eq!(source, parallel);
        let bytes = serde_json::to_vec(&source).unwrap();
        assert_eq!(source, decode_fixed_batch_replay(&bytes).unwrap());
        verify_fixed_batch_replay(factory, 2, &source).unwrap();
    }

    #[test]
    fn valid_metadata_cannot_substitute_for_physical_reexecution() {
        let source = record_fixed_batch_replay(factory, 42, 2, 2, &operations()[..32]).unwrap();
        for mutation in 0..3 {
            let mut forged = source.clone();
            match mutation {
                0 => {
                    forged.events[0].operation = FixedBatchOperation::Step {
                        actions_v: vec![0.0, 2.0],
                    }
                }
                1 => {
                    forged.events[30].operation = FixedBatchOperation::Reset {
                        lanes: vec![(1, 8)],
                    }
                }
                _ => forged.events[31].evidence_sha256 = format!("sha256:{}", "0".repeat(64)),
            }
            forged.content_sha256 = content_digest(&forged).unwrap();
            forged.validate_metadata().unwrap();
            assert!(verify_fixed_batch_replay(factory, 1, &forged).is_err());
        }
    }

    #[test]
    fn bounded_decoder_and_operation_limits_reject_invalid_evidence() {
        assert!(decode_fixed_batch_replay(&vec![b' '; MAX_FIXED_REPLAY_BYTES + 1]).is_err());
        assert!(record_fixed_batch_replay(factory, 42, 2, 1, &[]).is_err());
        assert!(record_fixed_batch_replay(
            factory,
            42,
            2,
            1,
            &vec![
                FixedBatchOperation::Step {
                    actions_v: vec![0.0; 2]
                };
                MAX_OPERATIONS + 1
            ]
        )
        .is_err());
        assert!(record_fixed_batch_replay(
            factory,
            42,
            2,
            1,
            &[FixedBatchOperation::Reset {
                lanes: vec![(1, 0), (0, 0)]
            }]
        )
        .is_err());
        let source = record_fixed_batch_replay(factory, 42, 2, 1, &operations()[..1]).unwrap();
        let mut json = serde_json::to_value(&source).unwrap();
        json["unknown"] = serde_json::json!(true);
        assert!(decode_fixed_batch_replay(&serde_json::to_vec(&json).unwrap()).is_err());
        let mut wrong_schema = source;
        wrong_schema.schema_version = 2;
        assert!(wrong_schema.validate_metadata().is_err());
    }

    #[cfg(feature = "mujoco")]
    #[test]
    fn mujoco_history_replays_across_worker_counts() {
        use rne_physics_mujoco::MuJoCoBackend;
        let factory = || {
            Ok((
                MuJoCoBackend::new(rne_core::SimDuration::from_ticks(1_000_000))?,
                MuJoCoBackend::manifest(),
            ))
        };
        let source = record_fixed_batch_replay(factory, 42, 2, 1, &operations()).unwrap();
        verify_fixed_batch_replay(factory, 2, &source).unwrap();
    }
}
