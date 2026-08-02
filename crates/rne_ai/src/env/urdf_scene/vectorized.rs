//! Vectorized Unitree Go2 and G1 locomotion episodes.

use super::{
    UnitreeG1GaitAction, UnitreeG1GaitEpisode, UnitreeG1GaitEpisodeConfig,
    UnitreeG1GaitObservation, UnitreeGo2Action, UnitreeGo2Episode, UnitreeGo2EpisodeConfig,
    UnitreeGo2Observation,
};
use crate::{
    VectorizedEpisode, VectorizedEpisodeCheckpoint, VectorizedEpisodeConfig, VectorizedEpisodeStep,
};
use rne_assets::AssetError;

/// Configuration for a vectorized Unitree Go2 gait batch.
#[derive(Clone, Debug, PartialEq)]
pub struct VectorizedUnitreeGo2GaitConfig {
    /// Per-environment Go2 episode configuration.
    pub episode: UnitreeGo2EpisodeConfig,
    /// Number of parallel Go2 environments.
    pub num_envs: usize,
    /// Root seed; environment `i` receives `seed + i` with wrapping arithmetic.
    pub seed: u64,
    /// Whether an ended Go2 episode is reset during the same batch step.
    pub auto_reset: bool,
}

impl Default for VectorizedUnitreeGo2GaitConfig {
    fn default() -> Self {
        Self {
            episode: UnitreeGo2EpisodeConfig::default(),
            num_envs: 1,
            seed: 1,
            auto_reset: true,
        }
    }
}

/// Configuration for a vectorized Unitree G1 gait batch.
#[derive(Clone, Debug, PartialEq)]
pub struct VectorizedUnitreeG1GaitConfig {
    /// Per-environment G1 episode configuration.
    pub episode: UnitreeG1GaitEpisodeConfig,
    /// Number of parallel G1 environments.
    pub num_envs: usize,
    /// Root seed; environment `i` receives `seed + i` with wrapping arithmetic.
    pub seed: u64,
    /// Whether an ended G1 episode is reset during the same batch step.
    pub auto_reset: bool,
}

impl Default for VectorizedUnitreeG1GaitConfig {
    fn default() -> Self {
        Self {
            episode: UnitreeG1GaitEpisodeConfig::default(),
            num_envs: 1,
            seed: 1,
            auto_reset: true,
        }
    }
}

/// Batch step returned by [`VectorizedUnitreeGo2GaitEnv`].
pub type VectorizedUnitreeGo2GaitStep = VectorizedEpisodeStep<UnitreeGo2Observation>;

/// Replay checkpoint accepted by [`VectorizedUnitreeGo2GaitEnv`].
pub type VectorizedUnitreeGo2GaitCheckpoint = VectorizedEpisodeCheckpoint<UnitreeGo2Action>;

/// Batch step returned by [`VectorizedUnitreeG1GaitEnv`].
pub type VectorizedUnitreeG1GaitStep = VectorizedEpisodeStep<UnitreeG1GaitObservation>;

/// Replay checkpoint accepted by [`VectorizedUnitreeG1GaitEnv`].
pub type VectorizedUnitreeG1GaitCheckpoint = VectorizedEpisodeCheckpoint<UnitreeG1GaitAction>;

/// Seeded vectorized wrapper around the official Go2 gait episode.
pub struct VectorizedUnitreeGo2GaitEnv {
    inner: VectorizedEpisode<UnitreeGo2Episode>,
}

impl VectorizedUnitreeGo2GaitEnv {
    /// Creates and seeds every Go2 episode in stable environment-index order.
    ///
    /// # Errors
    ///
    /// Returns the asset error from the first environment that fails to load.
    ///
    /// # Panics
    ///
    /// Panics when `config.num_envs` is zero.
    pub fn new(config: VectorizedUnitreeGo2GaitConfig) -> Result<Self, AssetError> {
        assert!(config.num_envs > 0, "num_envs must be positive");
        let episodes = (0..config.num_envs)
            .map(|index| {
                UnitreeGo2Episode::new_with_seed(
                    config.episode.clone(),
                    config.seed.wrapping_add(index as u64),
                )
            })
            .collect::<Result<Vec<_>, _>>()?;
        Ok(Self {
            inner: VectorizedEpisode::from_episodes(
                episodes,
                VectorizedEpisodeConfig {
                    num_envs: config.num_envs,
                    seed: config.seed,
                    auto_reset: config.auto_reset,
                },
            ),
        })
    }

    /// Returns the number of parallel Go2 environments.
    pub fn num_envs(&self) -> usize {
        self.inner.num_envs()
    }

    /// Resets every Go2 environment.
    pub fn reset(&mut self) -> VectorizedUnitreeGo2GaitStep {
        self.inner.reset()
    }

    /// Applies one gait action per Go2 environment.
    pub fn step(&mut self, actions: &[UnitreeGo2Action]) -> VectorizedUnitreeGo2GaitStep {
        self.inner.step(actions)
    }

    /// Returns the deterministic digest of the reset and replay history.
    pub fn replay_digest(&self) -> u64 {
        self.inner.replay_digest()
    }

    /// Returns a replay checkpoint for the current batch.
    pub fn checkpoint(
        &self,
    ) -> Result<VectorizedUnitreeGo2GaitCheckpoint, crate::VectorizedEpisodeCheckpointError> {
        self.inner.checkpoint()
    }

    /// Restores a Go2 batch by resetting and replaying its checkpoint actions.
    pub fn restore_checkpoint(
        &mut self,
        checkpoint: &VectorizedUnitreeGo2GaitCheckpoint,
    ) -> Result<(), crate::VectorizedEpisodeCheckpointError> {
        self.inner.restore_checkpoint(checkpoint)
    }

    /// Returns read access to one underlying Go2 episode.
    pub fn episode(&self, index: usize) -> &UnitreeGo2Episode {
        self.inner.episode(index)
    }
}

/// Seeded vectorized wrapper around the official G1 gait episode.
pub struct VectorizedUnitreeG1GaitEnv {
    inner: VectorizedEpisode<UnitreeG1GaitEpisode>,
}

impl VectorizedUnitreeG1GaitEnv {
    /// Creates and seeds every G1 episode in stable environment-index order.
    ///
    /// # Errors
    ///
    /// Returns the asset error from the first environment that fails to load.
    ///
    /// # Panics
    ///
    /// Panics when `config.num_envs` is zero.
    pub fn new(config: VectorizedUnitreeG1GaitConfig) -> Result<Self, AssetError> {
        assert!(config.num_envs > 0, "num_envs must be positive");
        let episodes = (0..config.num_envs)
            .map(|index| {
                UnitreeG1GaitEpisode::new_with_seed(
                    config.episode.clone(),
                    config.seed.wrapping_add(index as u64),
                )
            })
            .collect::<Result<Vec<_>, _>>()?;
        Ok(Self {
            inner: VectorizedEpisode::from_episodes(
                episodes,
                VectorizedEpisodeConfig {
                    num_envs: config.num_envs,
                    seed: config.seed,
                    auto_reset: config.auto_reset,
                },
            ),
        })
    }

    /// Returns the number of parallel G1 environments.
    pub fn num_envs(&self) -> usize {
        self.inner.num_envs()
    }

    /// Resets every G1 environment.
    pub fn reset(&mut self) -> VectorizedUnitreeG1GaitStep {
        self.inner.reset()
    }

    /// Applies one gait action per G1 environment.
    pub fn step(&mut self, actions: &[UnitreeG1GaitAction]) -> VectorizedUnitreeG1GaitStep {
        self.inner.step(actions)
    }

    /// Returns the deterministic digest of the reset and replay history.
    pub fn replay_digest(&self) -> u64 {
        self.inner.replay_digest()
    }

    /// Returns a replay checkpoint for the current batch.
    pub fn checkpoint(
        &self,
    ) -> Result<VectorizedUnitreeG1GaitCheckpoint, crate::VectorizedEpisodeCheckpointError> {
        self.inner.checkpoint()
    }

    /// Restores a G1 batch by resetting and replaying its checkpoint actions.
    pub fn restore_checkpoint(
        &mut self,
        checkpoint: &VectorizedUnitreeG1GaitCheckpoint,
    ) -> Result<(), crate::VectorizedEpisodeCheckpointError> {
        self.inner.restore_checkpoint(checkpoint)
    }

    /// Returns read access to one underlying G1 episode.
    pub fn episode(&self, index: usize) -> &UnitreeG1GaitEpisode {
        self.inner.episode(index)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn go2_vectorized_replay_restores_digest() {
        let mut env = VectorizedUnitreeGo2GaitEnv::new(VectorizedUnitreeGo2GaitConfig {
            episode: UnitreeGo2EpisodeConfig {
                max_steps: 4,
                cycle_steps: 45,
                ..UnitreeGo2EpisodeConfig::default()
            },
            num_envs: 2,
            seed: 301,
            auto_reset: false,
        })
        .expect("Go2 vectorized environment");
        assert_eq!(env.episode(0).sim().world_seed(), 301);
        assert_eq!(env.episode(1).sim().world_seed(), 302);
        env.reset();
        env.step(&[UnitreeGo2Action::default(); 2]);
        let checkpoint = env.checkpoint().expect("Go2 checkpoint");
        let digest = env.replay_digest();
        env.step(&[UnitreeGo2Action::default(); 2]);
        env.restore_checkpoint(&checkpoint).expect("Go2 replay");
        assert_eq!(env.replay_digest(), digest);
    }

    #[test]
    fn g1_vectorized_batch_preserves_environment_order() {
        let mut env = VectorizedUnitreeG1GaitEnv::new(VectorizedUnitreeG1GaitConfig {
            episode: UnitreeG1GaitEpisodeConfig {
                max_steps: 2,
                ..UnitreeG1GaitEpisodeConfig::default()
            },
            num_envs: 2,
            seed: 302,
            auto_reset: false,
        })
        .expect("G1 vectorized environment");
        assert_eq!(env.episode(0).sim().world_seed(), 302);
        assert_eq!(env.episode(1).sim().world_seed(), 303);
        let reset = env.reset();
        assert_eq!(reset.observations.len(), 2);
        let step = env.step(&[UnitreeG1GaitAction::default(); 2]);
        assert_eq!(step.observations.len(), 2);
        assert!(step
            .observations
            .iter()
            .all(|observation| observation.progress > 0.0));
    }
}
