//! Bounded CPU-parallel physical episodes with scheduling-independent evidence.

use crate::observed::{run_randomized_mobility_sensor_trace, SensorObservedTrace};
use anyhow::{ensure, Context, Result};
use rne_ai::derive_episode_seed;
use rne_physics::{PhysicsBackend, PhysicsBackendManifest};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};

/// Maximum retained physical episodes in one report.
pub const MAX_SENSOR_BATCH_LANES: usize = 64;
/// Maximum worker threads created by the episode runner.
pub const MAX_SENSOR_BATCH_WORKERS: usize = 16;

/// One independently seeded physical/sensor reset and its complete result.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SensorEpisodeLane {
    /// Stable lane identity, independent of worker assignment.
    pub lane_id: u64,
    /// Seed derived from root seed, lane identity and episode index.
    pub episode_seed: u64,
    /// Sensor-only closed-loop execution, including failures and physical truth.
    pub trace: SensorObservedTrace,
}

/// Canonically ordered episode evidence; execution topology is intentionally excluded.
/// This is an episode-parallel CPU runner, not a lockstep vectorized policy interface.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SensorEpisodeBatch {
    /// Stable artifact kind.
    pub kind: String,
    /// Batch schema version (nested traces carry their own schema).
    pub schema_version: u32,
    /// Explicit root seed used only to derive lane-local reset seeds.
    pub root_seed: u64,
    /// Episode index shared by this batch.
    pub episode_index: u64,
    /// All requested lanes, in increasing identity order.
    pub lanes: Vec<SensorEpisodeLane>,
    /// Number of failed tasks, retained rather than filtered from the batch.
    pub failed_lanes: usize,
    /// True only if every task passed its acceptance thresholds.
    pub passed: bool,
    /// SHA-256 of compact JSON with this field empty.
    pub content_digest: String,
}

impl SensorEpisodeBatch {
    /// Verifies identities, reset seeds, nested evidence, failure accounting and digest.
    /// This does not independently re-execute the physical episodes.
    pub fn validate(&self) -> Result<()> {
        ensure!(
            self.kind == "rne_mobility_sensor_episode_batch" && self.schema_version == 1,
            "batch kind/schema mismatch"
        );
        ensure!(
            (1..=MAX_SENSOR_BATCH_LANES).contains(&self.lanes.len()),
            "invalid batch width"
        );
        for (index, lane) in self.lanes.iter().enumerate() {
            ensure!(lane.lane_id == index as u64, "lane order mismatch");
            ensure!(
                lane.episode_seed
                    == derive_episode_seed(self.root_seed, lane.lane_id, self.episode_index),
                "lane seed mismatch"
            );
            lane.trace.validate()?;
            ensure!(
                lane.trace.contract.physical_profile.is_some(),
                "joint physical reset missing"
            );
            ensure!(
                lane.trace
                    .contract
                    .randomization
                    .as_ref()
                    .map(|sample| sample.episode_seed)
                    == Some(lane.episode_seed),
                "trace reset seed mismatch"
            );
            ensure!(
                lane.trace.backend == self.lanes[0].trace.backend
                    && lane.trace.task_spec == self.lanes[0].trace.task_spec,
                "lane backend/task mismatch"
            );
        }
        let failures = self.lanes.iter().filter(|lane| !lane.trace.passed).count();
        ensure!(
            self.failed_lanes == failures && self.passed == (failures == 0),
            "batch task verdict mismatch"
        );
        ensure!(
            self.content_digest == digest(self)?,
            "batch digest mismatch"
        );
        Ok(())
    }
}

/// Executes independent physical worlds on a bounded set of scoped CPU threads.
///
/// The factory must create a fresh backend per call. No ECS, sensor bus or controller
/// is shared between episodes. Changing worker count changes scheduling only; lane
/// seeds and serialized results do not depend on worker identity or completion order.
/// Factories and solver errors fail the call, while ordinary task failures stay in it.
pub fn run_sensor_episode_batch<B, F>(
    factory: F,
    root_seed: u64,
    episode_index: u64,
    num_envs: usize,
    num_workers: usize,
) -> Result<SensorEpisodeBatch>
where
    B: PhysicsBackend,
    F: Fn() -> Result<(B, PhysicsBackendManifest)> + Sync,
{
    ensure!(
        (1..=MAX_SENSOR_BATCH_LANES).contains(&num_envs),
        "batch width must be 1..=64"
    );
    ensure!(
        (1..=MAX_SENSOR_BATCH_WORKERS).contains(&num_workers),
        "worker count must be 1..=16"
    );
    let workers = num_workers.min(num_envs);
    let mut lanes = std::thread::scope(|scope| -> Result<Vec<SensorEpisodeLane>> {
        let factory = &factory;
        let handles: Vec<_> = (0..workers)
            .map(|worker| {
                scope.spawn(move || -> Result<Vec<SensorEpisodeLane>> {
                    (worker..num_envs)
                        .step_by(workers)
                        .map(|index| {
                            let lane_id = index as u64;
                            let episode_seed =
                                derive_episode_seed(root_seed, lane_id, episode_index);
                            let (backend, manifest) =
                                factory().context("create independent lane backend")?;
                            let trace = run_randomized_mobility_sensor_trace(
                                backend,
                                manifest,
                                episode_seed,
                            )?;
                            Ok(SensorEpisodeLane {
                                lane_id,
                                episode_seed,
                                trace,
                            })
                        })
                        .collect()
                })
            })
            .collect();
        let mut results = Vec::with_capacity(num_envs);
        let mut first_error = None;
        for handle in handles {
            match handle
                .join()
                .map_err(|_| anyhow::anyhow!("episode worker panicked"))
                .and_then(|result| result)
            {
                Ok(lanes) => results.extend(lanes),
                Err(error) => {
                    if first_error.is_none() {
                        first_error = Some(error);
                    }
                }
            }
        }
        if let Some(error) = first_error {
            return Err(error);
        }
        Ok(results)
    })?;
    lanes.sort_by_key(|lane| lane.lane_id);
    let failed_lanes = lanes.iter().filter(|lane| !lane.trace.passed).count();
    let mut report = SensorEpisodeBatch {
        kind: "rne_mobility_sensor_episode_batch".into(),
        schema_version: 1,
        root_seed,
        episode_index,
        lanes,
        failed_lanes,
        passed: failed_lanes == 0,
        content_digest: String::new(),
    };
    report.content_digest = digest(&report)?;
    report.validate()?;
    Ok(report)
}

fn digest(report: &SensorEpisodeBatch) -> Result<String> {
    let mut normalized = report.clone();
    normalized.content_digest.clear();
    Ok(format!(
        "sha256:{:x}",
        Sha256::digest(serde_json::to_vec(&normalized)?)
    ))
}

#[cfg(test)]
mod tests {
    use super::*;
    use rne_physics_rapier::RapierBackend;

    fn factory() -> Result<(RapierBackend, PhysicsBackendManifest)> {
        Ok((RapierBackend::new(), RapierBackend::manifest()))
    }

    #[test]
    fn physical_batch_is_worker_and_width_independent() {
        let serial = run_sensor_episode_batch(factory, 42, 0, 3, 1).unwrap();
        let parallel = run_sensor_episode_batch(factory, 42, 0, 3, 2).unwrap();
        assert_eq!(serial, parallel);
        assert_eq!(
            serial.lanes[0],
            run_sensor_episode_batch(factory, 42, 0, 1, 1)
                .unwrap()
                .lanes[0]
        );
        assert_ne!(
            serial.lanes[0].trace.contract,
            serial.lanes[1].trace.contract
        );
        assert_eq!(
            serial.failed_lanes,
            serial
                .lanes
                .iter()
                .filter(|lane| !lane.trace.passed)
                .count()
        );
        let mut forged = serial;
        forged.lanes.swap(0, 1);
        assert!(forged.validate().is_err());
    }

    #[test]
    fn invalid_width_and_factory_errors_are_not_empty_successes() {
        assert!(run_sensor_episode_batch(factory, 42, 0, 0, 1).is_err());
        assert!(run_sensor_episode_batch(factory, 42, 0, 1, 0).is_err());
        assert!(run_sensor_episode_batch(factory, 42, 0, 65, 1).is_err());
        assert!(run_sensor_episode_batch(factory, 42, 0, 1, 17).is_err());
        assert!(run_sensor_episode_batch(
            || -> Result<(RapierBackend, PhysicsBackendManifest)> {
                anyhow::bail!("intentional factory failure")
            },
            42,
            0,
            2,
            2
        )
        .is_err());
    }

    #[cfg(feature = "mujoco")]
    #[test]
    fn mujoco_episode_batch_is_worker_independent() {
        use rne_physics_mujoco::MuJoCoBackend;
        let factory = || {
            Ok((
                MuJoCoBackend::new(rne_core::SimDuration::from_ticks(
                    crate::observed::SENSOR_OBSERVED_FIXED_DELTA_TICKS,
                ))?,
                MuJoCoBackend::manifest(),
            ))
        };
        assert_eq!(
            run_sensor_episode_batch(factory, 42, 0, 2, 1).unwrap(),
            run_sensor_episode_batch(factory, 42, 0, 2, 2).unwrap()
        );
    }

    #[test]
    fn all_panicking_workers_are_joined_and_reported_as_errors() {
        let result = run_sensor_episode_batch(
            || -> Result<(RapierBackend, PhysicsBackendManifest)> {
                panic!("intentional worker panic")
            },
            42,
            0,
            2,
            2,
        );
        assert!(result.unwrap_err().to_string().contains("worker panicked"));
    }
}
