//! Persistent, fallible CPU batched stepping with explicit partial resets.

use crate::observed::{SensorFixedEnvironment, SensorFixedObservation, SensorFixedStep};
use crate::observed_batch::{MAX_SENSOR_BATCH_LANES, MAX_SENSOR_BATCH_WORKERS};
use anyhow::{ensure, Context, Result};
use rne_ai::derive_episode_seed;
use rne_physics::{PhysicsBackend, PhysicsBackendManifest};

mod learning;
mod reference_learner;
mod replay;
pub use learning::{LearningBatchStep, LearningLaneFailure, LearningTransition};
pub use reference_learner::SensorTableLearner;
pub use replay::{
    decode_fixed_batch_replay, record_fixed_batch_replay,
    record_fixed_batch_replay_with_noise_root, verify_fixed_batch_replay, FixedBatchOperation,
    FixedBatchReplay, FixedBatchReplayEvent, MAX_FIXED_REPLAY_BYTES,
};

/// One stable lane's completed transition or execution failure.
#[derive(Clone, Debug, PartialEq)]
pub struct SensorFixedLaneStep {
    /// Zero-based stable environment identity.
    pub lane_id: usize,
    /// Explicit episode index most recently reset into this lane.
    pub episode_index: u64,
    /// Derived reset seed, evaluator metadata rather than an actor tensor.
    pub episode_seed: u64,
    /// Successful 10 ms transition or error. An error may follow partial physical
    /// progress; that lane requires reset. Other lane outcomes are retained.
    pub outcome: std::result::Result<SensorFixedStep, String>,
}

struct Lane<B: PhysicsBackend> {
    environment: SensorFixedEnvironment<B>,
    episode_index: u64,
    episode_seed: u64,
}

/// Bounded CPU-parallel persistent worlds, with no automatic reset.
///
/// Input validation is atomic; physical stepping is not transactional. After an
/// execution error, callers must inspect every outcome and reset failed lanes.
/// Worker count affects scheduling only. Each transition includes the fixed-task
/// evaluator reward separately from its sensor-only actor snapshot.
pub struct SensorFixedBatch<B: PhysicsBackend, F> {
    lanes: Vec<Lane<B>>,
    factory: F,
    manifest: PhysicsBackendManifest,
    root_seed: u64,
    noise_root_seed: Option<u64>,
    workers: usize,
}

impl<B: PhysicsBackend, F> std::fmt::Debug for SensorFixedBatch<B, F> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("SensorFixedBatch")
            .field("num_envs", &self.lanes.len())
            .field("root_seed", &self.root_seed)
            .field("noise_root_seed", &self.noise_root_seed)
            .field("workers", &self.workers)
            .finish_non_exhaustive()
    }
}

impl<B, F> SensorFixedBatch<B, F>
where
    B: PhysicsBackend,
    F: Fn() -> Result<(B, PhysicsBackendManifest)>,
{
    /// Constructs 1..=64 fresh worlds at episode index zero with 1..=16 workers.
    /// The factory must return independent backends with identical manifests.
    pub fn new(factory: F, root_seed: u64, num_envs: usize, workers: usize) -> Result<Self> {
        Self::with_noise_policy(factory, root_seed, None, num_envs, workers)
    }

    /// Constructs worlds with a separate noise root. Noise seeds derive from
    /// `(noise_root_seed, lane_id, episode_index)`, independently of reset parameters.
    pub fn new_with_noise_root(
        factory: F,
        root_seed: u64,
        noise_root_seed: u64,
        num_envs: usize,
        workers: usize,
    ) -> Result<Self> {
        Self::with_noise_policy(factory, root_seed, Some(noise_root_seed), num_envs, workers)
    }

    fn with_noise_policy(
        factory: F,
        root_seed: u64,
        noise_root_seed: Option<u64>,
        num_envs: usize,
        workers: usize,
    ) -> Result<Self> {
        ensure!(
            (1..=MAX_SENSOR_BATCH_LANES).contains(&num_envs),
            "invalid fixed batch width"
        );
        ensure!(
            (1..=MAX_SENSOR_BATCH_WORKERS).contains(&workers),
            "invalid fixed batch worker count"
        );
        let mut lanes = Vec::with_capacity(num_envs);
        let mut expected_manifest = None;
        for lane_id in 0..num_envs {
            let (backend, manifest) = factory().context("create fixed batch backend")?;
            if let Some(expected) = &expected_manifest {
                ensure!(expected == &manifest, "fixed batch backend identity drift");
            } else {
                expected_manifest = Some(manifest.clone());
            }
            let episode_seed = derive_episode_seed(root_seed, lane_id as u64, 0);
            lanes.push(Lane {
                environment: SensorFixedEnvironment::new_with_noise_seed(
                    backend,
                    manifest,
                    episode_seed,
                    noise_root_seed.map_or(crate::observed::WORLD_SEED, |root| {
                        derive_episode_seed(root, lane_id as u64, 0)
                    }),
                )?,
                episode_index: 0,
                episode_seed,
            });
        }
        Ok(Self {
            lanes,
            factory,
            manifest: expected_manifest.context("empty batch")?,
            root_seed,
            noise_root_seed,
            workers,
        })
    }

    /// Returns sensor-only snapshots in lane order, without advancing worlds.
    /// Poisoned lanes are errors, never stale successful observations.
    pub fn observations(&self) -> Vec<Result<SensorFixedObservation>> {
        self.lanes
            .iter()
            .map(|lane| lane.environment.observation())
            .collect()
    }

    /// Advances every lane by 10 ms after validating the entire voltage batch.
    /// Width, action, terminal and poisoned-state errors reject before mutation.
    /// Execution errors are returned per lane; all workers are joined. Panicking
    /// lane execution is caught, reported and cannot be resumed without reset.
    pub fn step(&mut self, actions_v: &[f64]) -> Result<Vec<SensorFixedLaneStep>> {
        ensure!(
            actions_v.len() == self.lanes.len(),
            "fixed action batch width mismatch"
        );
        for (lane_id, (lane, action)) in self.lanes.iter().zip(actions_v).enumerate() {
            lane.environment
                .validate_action(*action)
                .with_context(|| format!("fixed lane {lane_id} preflight"))?;
        }
        let chunk_size = self.lanes.len().div_ceil(self.workers);
        std::thread::scope(|scope| {
            let handles: Vec<_> = self
                .lanes
                .chunks_mut(chunk_size)
                .enumerate()
                .map(|(chunk_index, chunk)| {
                    let offset = chunk_index * chunk_size;
                    let actions = &actions_v[offset..offset + chunk.len()];
                    scope.spawn(move || {
                        chunk
                            .iter_mut()
                            .zip(actions)
                            .enumerate()
                            .map(|(index, (lane, voltage))| {
                                // A panic unwinds through step after it marks the world poisoned.
                                // We never resume mutated solver state: only explicit reset recovers it.
                                let outcome =
                                    std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                                        lane.environment.step(*voltage)
                                    }))
                                    .map_err(|_| {
                                        "fixed lane execution panicked; reset required".to_string()
                                    })
                                    .and_then(|result| {
                                        result.map_err(|error| format!("{error:#}"))
                                    });
                                SensorFixedLaneStep {
                                    lane_id: offset + index,
                                    episode_index: lane.episode_index,
                                    episode_seed: lane.episode_seed,
                                    outcome,
                                }
                            })
                            .collect::<Vec<_>>()
                    })
                })
                .collect();
            let mut results = Vec::with_capacity(actions_v.len());
            let mut worker_error = None;
            for handle in handles {
                match handle.join() {
                    Ok(outcomes) => results.extend(outcomes),
                    Err(_) => {
                        worker_error = Some(anyhow::anyhow!("fixed worker infrastructure panicked"))
                    }
                }
            }
            if let Some(error) = worker_error {
                return Err(error);
            }
            Ok(results)
        })
    }

    /// Reconstructs only selected lanes at explicit episode indices. IDs must be
    /// nonempty, strictly increasing and in range. Every replacement is constructed
    /// before any existing lane changes; failed construction preserves the batch.
    /// Unselected lane state, clocks and pending actions remain untouched.
    pub fn reset_lanes(&mut self, resets: &[(usize, u64)]) -> Result<Vec<SensorFixedObservation>> {
        ensure!(!resets.is_empty(), "empty fixed reset mask");
        ensure!(
            resets.windows(2).all(|pair| pair[0].0 < pair[1].0),
            "fixed reset IDs must be strictly increasing"
        );
        ensure!(
            resets.iter().all(|(id, _)| *id < self.lanes.len()),
            "fixed reset ID out of range"
        );
        let mut replacements = Vec::with_capacity(resets.len());
        for &(lane_id, episode_index) in resets {
            let (backend, manifest) = (self.factory)().context("construct fixed reset backend")?;
            ensure!(
                manifest == self.manifest,
                "fixed reset backend identity drift"
            );
            let episode_seed = derive_episode_seed(self.root_seed, lane_id as u64, episode_index);
            let noise_seed = self
                .noise_root_seed
                .map_or(crate::observed::WORLD_SEED, |root| {
                    derive_episode_seed(root, lane_id as u64, episode_index)
                });
            let environment = SensorFixedEnvironment::new_with_noise_seed(
                backend,
                manifest,
                episode_seed,
                noise_seed,
            )?;
            let observation = environment.observation()?;
            replacements.push((
                lane_id,
                Lane {
                    environment,
                    episode_index,
                    episode_seed,
                },
                observation,
            ));
        }
        let mut observations = Vec::with_capacity(replacements.len());
        for (id, lane, observation) in replacements {
            self.lanes[id] = lane;
            observations.push(observation);
        }
        Ok(observations)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rne_physics_rapier::RapierBackend;

    #[test]
    fn independent_noise_partial_reset_preserves_other_lane_and_width() {
        let mut narrow = SensorFixedBatch::new_with_noise_root(factory, 42, 99, 2, 1).unwrap();
        let mut wide = SensorFixedBatch::new_with_noise_root(factory, 42, 99, 3, 2).unwrap();
        for _ in 0..6 {
            assert_eq!(
                narrow.step(&[3.0, 3.0]).unwrap(),
                wide.step(&[3.0, 3.0, 3.0]).unwrap()[..2]
            );
        }
        let continuing = narrow.lanes[1].environment.observation().unwrap();
        let old_seed = narrow.lanes[0]
            .environment
            .reset_contract()
            .sensor_noise_seed;
        narrow.reset_lanes(&[(0, 4)]).unwrap();
        wide.reset_lanes(&[(0, 4)]).unwrap();
        assert_eq!(
            narrow.lanes[1].environment.observation().unwrap(),
            continuing
        );
        let new_seed = narrow.lanes[0]
            .environment
            .reset_contract()
            .sensor_noise_seed;
        assert_eq!(new_seed, derive_episode_seed(99, 0, 4));
        assert_ne!(old_seed, new_seed);
        for _ in 0..6 {
            assert_eq!(
                narrow.step(&[3.0, 3.0]).unwrap(),
                wide.step(&[3.0, 3.0, 3.0]).unwrap()[..2]
            );
        }
    }

    fn factory() -> Result<(RapierBackend, PhysicsBackendManifest)> {
        Ok((RapierBackend::new(), RapierBackend::manifest()))
    }

    #[test]
    fn learning_projection_preserves_clock_preflight_and_truncation() {
        let mut batch = SensorFixedBatch::new_with_noise_root(factory, 42, 99, 2, 2).unwrap();
        assert!(batch.step_learning(&[3.0, f64::NAN]).is_err());
        assert_eq!(batch.observations()[0].as_ref().unwrap().time_ticks, 0);
        for index in 0..330 {
            let output = batch.step_learning(&[3.0, 3.0]).unwrap();
            assert!(output.failures.is_empty());
            assert_eq!(output.transitions.len(), 2);
            for transition in output.transitions {
                assert_eq!(transition.start_ticks, index * 10_000_000);
                assert_eq!(transition.end_ticks, (index + 1) * 10_000_000);
                assert_eq!(transition.truncated, index == 329);
                assert_eq!(
                    transition.observation.len(),
                    crate::observed::sensor_fixed_task_spec()
                        .observation
                        .tensors
                        .len()
                );
            }
        }
        assert!(batch.step_learning(&[3.0, 3.0]).is_err());
    }

    #[test]
    fn learning_projection_excludes_failed_steps_and_retains_healthy_lanes() {
        use std::sync::{
            atomic::{AtomicU8, AtomicUsize, Ordering},
            Arc,
        };
        for mode in [1, 2] {
            let calls = AtomicUsize::new(0);
            let failing = Arc::new(AtomicU8::new(0));
            let factory = || {
                let id = calls.fetch_add(1, Ordering::Relaxed);
                Ok((
                    FaultBackend {
                        inner: RapierBackend::new(),
                        fault: if id == 1 {
                            failing.clone()
                        } else {
                            Arc::new(AtomicU8::new(0))
                        },
                    },
                    RapierBackend::manifest(),
                ))
            };
            let mut batch = SensorFixedBatch::new_with_noise_root(factory, 42, 99, 3, 2).unwrap();
            let mut learner = SensorTableLearner::new(71);
            failing.store(mode, Ordering::Relaxed);
            let result = learner.train_step(&mut batch, 0).unwrap();
            assert_eq!(learner.updates(), 2);
            assert_eq!(
                result
                    .transitions
                    .iter()
                    .map(|t| t.lane_id)
                    .collect::<Vec<_>>(),
                vec![0, 2]
            );
            assert_eq!(result.failures.len(), 1);
            assert_eq!(result.failures[0].lane_id, 1);
            assert!(batch.step_learning(&[3.0; 3]).is_err());
            assert_eq!(
                batch.observations()[0].as_ref().unwrap().time_ticks,
                10_000_000
            );
            batch.reset_lanes(&[(1, 4)]).unwrap();
            let resumed = learner.train_step(&mut batch, 1).unwrap();
            assert_eq!(learner.updates(), 5);
            assert!(resumed.failures.is_empty());
            assert_eq!(resumed.transitions[0].start_ticks, 10_000_000);
            assert_eq!(resumed.transitions[1].start_ticks, 0);
            assert_eq!(resumed.transitions[1].episode_index, 4);
        }
    }

    struct FaultBackend {
        inner: RapierBackend,
        fault: std::sync::Arc<std::sync::atomic::AtomicU8>,
    }

    impl PhysicsBackend for FaultBackend {
        type BodyHandle = <RapierBackend as PhysicsBackend>::BodyHandle;
        type ColliderHandle = <RapierBackend as PhysicsBackend>::ColliderHandle;
        fn create_world(
            &mut self,
            desc: rne_physics::PhysicsWorldDesc,
        ) -> std::result::Result<rne_physics::PhysicsWorldId, rne_physics::PhysicsError> {
            self.inner.create_world(desc)
        }
        fn sync_from_ecs(
            &mut self,
            world: &mut rne_ecs::World,
            id: rne_physics::PhysicsWorldId,
        ) -> std::result::Result<(), rne_physics::PhysicsError> {
            self.inner.sync_from_ecs(world, id)
        }
        fn sync_to_ecs(
            &mut self,
            world: &mut rne_ecs::World,
            id: rne_physics::PhysicsWorldId,
        ) -> std::result::Result<(), rne_physics::PhysicsError> {
            self.inner.sync_to_ecs(world, id)
        }
        fn step(
            &mut self,
            id: rne_physics::PhysicsWorldId,
            dt: rne_core::SimDuration,
        ) -> std::result::Result<(), rne_physics::PhysicsError> {
            self.inner.step(id, dt)?;
            match self.fault.load(std::sync::atomic::Ordering::Relaxed) {
                1 => Err(rne_physics::PhysicsError::WorldNotFound),
                2 => panic!("intentional post-physics panic"),
                _ => Ok(()),
            }
        }
        fn raycast(
            &self,
            id: rne_physics::PhysicsWorldId,
            query: rne_physics::RaycastQuery,
        ) -> std::result::Result<Vec<rne_physics::RaycastHit>, rne_physics::PhysicsError> {
            self.inner.raycast(id, query)
        }
        fn contacts(
            &self,
            id: rne_physics::PhysicsWorldId,
        ) -> std::result::Result<&[rne_physics::ContactEvent], rne_physics::PhysicsError> {
            self.inner.contacts(id)
        }
        fn contact_points(
            &self,
            id: rne_physics::PhysicsWorldId,
        ) -> std::result::Result<&[rne_physics::ContactPointSample], rne_physics::PhysicsError>
        {
            self.inner.contact_points(id)
        }
        fn apply_external_body_wrench(
            &mut self,
            id: rne_physics::PhysicsWorldId,
            wrench: rne_physics::ExternalBodyWrench,
        ) -> std::result::Result<(), rne_physics::PhysicsError> {
            self.inner.apply_external_body_wrench(id, wrench)
        }
        fn capabilities(&self) -> &[rne_physics::PhysicsCapability] {
            self.inner.capabilities()
        }
    }

    #[test]
    fn post_physics_error_and_panic_retain_other_lanes_and_require_reset() {
        use std::sync::{
            atomic::{AtomicU8, AtomicUsize, Ordering},
            Arc,
        };
        for mode in [1, 2] {
            let calls = AtomicUsize::new(0);
            let failing = Arc::new(AtomicU8::new(0));
            let factory = || {
                let id = calls.fetch_add(1, Ordering::Relaxed);
                Ok((
                    FaultBackend {
                        inner: RapierBackend::new(),
                        fault: if id == 1 {
                            failing.clone()
                        } else {
                            Arc::new(AtomicU8::new(0))
                        },
                    },
                    RapierBackend::manifest(),
                ))
            };
            let mut batch = SensorFixedBatch::new(factory, 42, 3, 2).unwrap();
            failing.store(mode, Ordering::Relaxed);
            let outcomes = batch.step(&[3.0; 3]).unwrap();
            assert!(outcomes[0].outcome.is_ok());
            assert!(outcomes[1].outcome.is_err());
            assert!(outcomes[2].outcome.is_ok());
            assert!(batch.observations()[1].is_err());
            assert!(batch.step(&[3.0; 3]).is_err());
            assert_eq!(
                batch.observations()[0].as_ref().unwrap().time_ticks,
                10_000_000
            );
            batch.reset_lanes(&[(1, 1)]).unwrap();
            let recovered = batch.step(&[3.0; 3]).unwrap();
            assert!(recovered.iter().all(|lane| lane.outcome.is_ok()));
            assert_eq!(
                recovered[0]
                    .outcome
                    .as_ref()
                    .unwrap()
                    .observation
                    .time_ticks,
                20_000_000
            );
            assert_eq!(
                recovered[1]
                    .outcome
                    .as_ref()
                    .unwrap()
                    .observation
                    .time_ticks,
                10_000_000
            );
        }
    }

    #[test]
    fn fixed_batch_matches_single_world_and_worker_counts() {
        let mut serial = SensorFixedBatch::new(factory, 42, 3, 1).unwrap();
        let mut parallel = SensorFixedBatch::new(factory, 42, 3, 2).unwrap();
        let mut single = SensorFixedEnvironment::new(
            RapierBackend::new(),
            RapierBackend::manifest(),
            derive_episode_seed(42, 0, 0),
        )
        .unwrap();
        for step in 1..=330 {
            let serial_result = serial.step(&[3.0, 2.0, 4.0]).unwrap();
            assert_eq!(serial_result, parallel.step(&[3.0, 2.0, 4.0]).unwrap());
            assert_eq!(
                serial_result[0].outcome.as_ref().unwrap(),
                &single.step(3.0).unwrap()
            );
            assert!(serial_result.iter().all(|lane| lane
                .outcome
                .as_ref()
                .unwrap()
                .observation
                .time_ticks
                == step * 10_000_000));
        }
        assert_eq!(
            serial.lanes[0]
                .environment
                .privileged_physics_hash_v2()
                .unwrap(),
            single.privileged_physics_hash_v2().unwrap()
        );
        assert!(serial.step(&[3.0, 2.0, 4.0]).is_err());
    }

    #[test]
    fn partial_reset_and_invalid_input_preserve_other_lanes() {
        let mut batch = SensorFixedBatch::new(factory, 42, 2, 2).unwrap();
        let mut reference = SensorFixedBatch::new(factory, 42, 2, 1).unwrap();
        for _ in 0..20 {
            assert_eq!(
                batch.step(&[3.0, 2.0]).unwrap(),
                reference.step(&[3.0, 2.0]).unwrap()
            );
        }
        let before: Vec<_> = batch
            .observations()
            .into_iter()
            .map(Result::unwrap)
            .collect();
        assert!(batch.step(&[3.0, f64::NAN]).is_err());
        assert!(batch.step(&[3.0]).is_err());
        assert!(batch.reset_lanes(&[(1, 1), (0, 1)]).is_err());
        assert!(batch.reset_lanes(&[(1, 1), (1, 2)]).is_err());
        assert!(batch.reset_lanes(&[(2, 1)]).is_err());
        assert_eq!(
            before,
            batch
                .observations()
                .into_iter()
                .map(Result::unwrap)
                .collect::<Vec<_>>()
        );
        assert!(batch.reset_lanes(&[(1, 7)]).unwrap()[0].latest.is_none());
        let mut reset_reference = SensorFixedEnvironment::new(
            RapierBackend::new(),
            RapierBackend::manifest(),
            derive_episode_seed(42, 1, 7),
        )
        .unwrap();
        for _ in 0..25 {
            let result = batch.step(&[3.0, 2.0]).unwrap();
            assert_eq!(result[0], reference.step(&[3.0, 2.0]).unwrap()[0]);
            assert_eq!(result[1].episode_index, 7);
            assert_eq!(
                result[1].outcome.as_ref().unwrap(),
                &reset_reference.step(2.0).unwrap()
            );
        }
    }

    #[test]
    fn failed_reset_construction_keeps_all_existing_worlds() {
        use std::sync::atomic::{AtomicUsize, Ordering};
        let calls = AtomicUsize::new(0);
        let factory = || {
            // Initial two worlds and first replacement succeed; second replacement fails.
            ensure!(
                calls.fetch_add(1, Ordering::Relaxed) != 3,
                "intentional reset factory failure"
            );
            Ok((RapierBackend::new(), RapierBackend::manifest()))
        };
        let mut batch = SensorFixedBatch::new(factory, 42, 2, 2).unwrap();
        batch.step(&[3.0, 2.0]).unwrap();
        let before: Vec<_> = batch
            .observations()
            .into_iter()
            .map(Result::unwrap)
            .collect();
        assert!(batch.reset_lanes(&[(0, 1), (1, 1)]).is_err());
        assert_eq!(
            before,
            batch
                .observations()
                .into_iter()
                .map(Result::unwrap)
                .collect::<Vec<_>>()
        );
        assert!(batch.lanes.iter().all(|lane| lane.episode_index == 0));
    }

    #[cfg(feature = "mujoco")]
    #[test]
    fn fixed_mujoco_batch_is_worker_independent() {
        use rne_physics_mujoco::MuJoCoBackend;
        let factory = || {
            Ok((
                MuJoCoBackend::new(rne_core::SimDuration::from_ticks(1_000_000))?,
                MuJoCoBackend::manifest(),
            ))
        };
        let mut serial = SensorFixedBatch::new(factory, 42, 2, 1).unwrap();
        let mut parallel = SensorFixedBatch::new(factory, 42, 2, 2).unwrap();
        for _ in 0..330 {
            assert_eq!(
                serial.step(&[3.0, 2.0]).unwrap(),
                parallel.step(&[3.0, 2.0]).unwrap()
            );
        }
        for index in 0..2 {
            assert_eq!(
                serial.lanes[index]
                    .environment
                    .privileged_physics_hash_v2()
                    .unwrap(),
                parallel.lanes[index]
                    .environment
                    .privileged_physics_hash_v2()
                    .unwrap()
            );
        }
    }
}
