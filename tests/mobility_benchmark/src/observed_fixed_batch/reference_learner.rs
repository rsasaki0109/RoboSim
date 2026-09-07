//! Bounded diagnostic learner, not a claim of policy quality or convergence.

use super::*;
use rne_core::KeyedRandom;
use std::collections::BTreeMap;

const ACTIONS_V: [f64; 5] = [0.0, 3.0, 12.0, 18.0, 24.0];
type State = [u16; 3];

mod checkpoint;

/// Small sensor-only tabular Q learner for exercising the fallible boundary.
///
/// Fixed alpha=0.1, discount=0.99 and epsilon=0.2; ties choose the lowest voltage.
/// The objective is explicitly the finite 330-action episode return: the horizon
/// has zero continuation value, but does not mean success. Quantized, delayed
/// sensor observations are not assumed Markov; no convergence guarantee applies.
#[derive(Clone, Debug, PartialEq)]
pub struct SensorTableLearner {
    values: BTreeMap<State, [f64; 5]>,
    exploration: KeyedRandom,
    updates: u64,
}

fn state(values: &[Vec<f64>]) -> Result<State> {
    let task = crate::observed::sensor_fixed_task_spec();
    ensure!(
        values.len() == task.observation.tensors.len(),
        "invalid actor count"
    );
    for (value, spec) in values.iter().zip(&task.observation.tensors) {
        ensure!(
            value.len() == spec.shape.iter().product::<usize>()
                && value.iter().all(|v| v.is_finite()),
            "invalid actor tensor"
        );
    }
    ensure!(
        values[0][0] == 0.0 || values[0][0] == 1.0,
        "invalid estimate mask"
    );
    let tick = values[2][0] * 100.0;
    ensure!(
        (0.0..=330.0).contains(&tick.round()) && (tick - tick.round()).abs() < 1e-9,
        "actor time is outside the fixed task grid"
    );
    // Invalid estimates have one dedicated state, never a fabricated speed.
    let speed = if values[0][0] == 0.0 {
        0
    } else {
        (((values[8][0].clamp(-2.0, 2.0) + 2.0) * 4.0).round() as u16) + 1
    };
    Ok([tick.round() as u16, values[0][0] as u16, speed])
}

fn greedy(values: &[f64; 5]) -> usize {
    let mut best = 0;
    for index in 1..values.len() {
        if values[index] > values[best] {
            best = index;
        }
    }
    best
}

impl SensorTableLearner {
    /// Creates zero action values and an independent, explicit exploration seed.
    pub fn new(exploration_seed: u64) -> Self {
        Self {
            values: BTreeMap::new(),
            exploration: KeyedRandom::new(exploration_seed, 0x514c4541524e),
            updates: 0,
        }
    }

    /// Number of valid transitions actually applied to the value table.
    pub fn updates(&self) -> u64 {
        self.updates
    }

    /// Chooses from sensor tensors only. Coordinates must be unique per decision
    /// during training, including across resets; they are RNG keys, not features.
    /// Evaluation disables exploration and never changes the learner.
    pub fn action(
        &self,
        observation: &[Vec<f64>],
        lane_id: usize,
        decision_index: u64,
        explore: bool,
    ) -> Result<f64> {
        let key = state(observation)?;
        ensure!(key[0] < 330, "episode requires reset");
        let values = self.values.get(&key).copied().unwrap_or([0.0; 5]);
        let index = if explore
            && self
                .exploration
                .sample_unit_f64(lane_id as u64, decision_index, 0)
                < 0.2
        {
            (self
                .exploration
                .sample_unit_f64(lane_id as u64, decision_index, 1)
                * 5.0) as usize
        } else {
            greedy(&values)
        };
        Ok(ACTIONS_V[index])
    }

    /// Applies successful transitions in their supplied stable order. Diagnostics
    /// cannot enter this API. Invalid samples reject the entire update atomically.
    pub fn learn(&mut self, transitions: &[LearningTransition]) -> Result<()> {
        let mut candidate = self.clone();
        for transition in transitions {
            let before = state(&transition.observation)?;
            let after = state(&transition.next_observation)?;
            ensure!(
                before[0] < 330
                    && after[0] == before[0] + 1
                    && transition.start_ticks == u64::from(before[0]) * 10_000_000
                    && transition.end_ticks == u64::from(after[0]) * 10_000_000
                    && transition.truncated == (after[0] == 330)
                    && transition.reward.is_finite(),
                "invalid learning interval"
            );
            let action = ACTIONS_V
                .iter()
                .position(|v| *v == transition.action_v)
                .context("action outside reference learner grid")?;
            let next = candidate.values.get(&after).copied().unwrap_or([0.0; 5]);
            let continuation = if transition.truncated {
                0.0
            } else {
                next[greedy(&next)]
            };
            let values = candidate.values.entry(before).or_insert([0.0; 5]);
            let updated =
                values[action] + 0.1 * (transition.reward + 0.99 * continuation - values[action]);
            ensure!(updated.is_finite(), "nonfinite Q update");
            values[action] = updated;
            candidate.updates = candidate
                .updates
                .checked_add(1)
                .context("update counter overflow")?;
        }
        *self = candidate;
        Ok(())
    }

    /// Selects actions from current sensors, advances physics, then updates from
    /// valid transitions only. Returns all execution failures for explicit reset.
    /// A learner validation error may occur after physical progress; no rollback
    /// of worlds is implied and the caller must stop and inspect that error.
    pub fn train_step<B, F>(
        &mut self,
        batch: &mut SensorFixedBatch<B, F>,
        decision_index: u64,
    ) -> Result<LearningBatchStep>
    where
        B: PhysicsBackend,
        F: Fn() -> Result<(B, PhysicsBackendManifest)>,
    {
        let actions = batch
            .observations()
            .into_iter()
            .enumerate()
            .map(|(lane, observation)| {
                self.action(&observation?.actor_tensors(), lane, decision_index, true)
            })
            .collect::<Result<Vec<_>>>()?;
        let output = batch.step_learning(&actions)?;
        self.learn(&output.transitions)?;
        Ok(output)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rne_physics_rapier::RapierBackend;

    #[cfg(feature = "mujoco")]
    #[test]
    fn reference_learner_mujoco_updates_across_workers() {
        let run = |workers| {
            let factory = || {
                Ok((
                    rne_physics_mujoco::MuJoCoBackend::new(rne_core::SimDuration::from_ticks(
                        1_000_000,
                    ))?,
                    rne_physics_mujoco::MuJoCoBackend::manifest(),
                ))
            };
            let mut batch =
                SensorFixedBatch::new_with_noise_root(factory, 42, 99, 2, workers).unwrap();
            let mut learner = SensorTableLearner::new(71);
            for index in 0..330 {
                assert!(learner
                    .train_step(&mut batch, index)
                    .unwrap()
                    .failures
                    .is_empty());
            }
            assert_eq!(learner.updates(), 660);
            assert!(learner.values.values().flatten().any(|v| *v != 0.0));
            learner
        };
        assert_eq!(run(1), run(2));
    }

    #[test]
    fn reference_learner_bellman_update_and_finite_horizon_are_explicit() {
        let factory = || Ok((RapierBackend::new(), RapierBackend::manifest()));
        let mut batch = SensorFixedBatch::new(factory, 42, 1, 1).unwrap();
        let mut sample = batch.step_learning(&[0.0]).unwrap().transitions.remove(0);
        sample.reward = -1.0;
        let mut learner = SensorTableLearner::new(71);
        learner
            .values
            .insert(state(&sample.next_observation).unwrap(), [2.0; 5]);
        learner.learn(&[sample.clone()]).unwrap();
        let q = learner.values[&state(&sample.observation).unwrap()][0];
        assert!((q - 0.098).abs() < 1e-12);
        sample.start_ticks = 3_290_000_000;
        sample.end_ticks = 3_300_000_000;
        sample.observation[2][0] = 3.29;
        sample.next_observation[2][0] = 3.3;
        sample.truncated = true;
        learner
            .values
            .insert(state(&sample.next_observation).unwrap(), [100.0; 5]);
        learner.learn(&[sample.clone()]).unwrap();
        assert_eq!(
            learner.values[&state(&sample.observation).unwrap()][0],
            -0.1
        );
        let saved = learner.clone();
        assert_eq!(
            learner.action(&sample.observation, 0, 0, false).unwrap(),
            3.0
        );
        assert_eq!(learner, saved);
    }

    #[test]
    fn reference_learner_updates_online_and_repeats_across_workers() {
        let run = |workers| {
            let factory = || Ok((RapierBackend::new(), RapierBackend::manifest()));
            let mut batch =
                SensorFixedBatch::new_with_noise_root(factory, 42, 99, 2, workers).unwrap();
            let mut learner = SensorTableLearner::new(71);
            for index in 0..330 {
                assert!(learner
                    .train_step(&mut batch, index)
                    .unwrap()
                    .failures
                    .is_empty());
            }
            assert_eq!(learner.updates(), 660);
            assert!(learner.values.values().flatten().any(|v| *v != 0.0));
            assert!(learner.train_step(&mut batch, 330).is_err());
            learner
        };
        assert_eq!(run(1), run(2));
    }

    #[test]
    fn reference_learner_rejects_invalid_update_atomically() {
        let factory = || Ok((RapierBackend::new(), RapierBackend::manifest()));
        let mut batch = SensorFixedBatch::new(factory, 42, 2, 1).unwrap();
        let mut samples = batch.step_learning(&[3.0; 2]).unwrap().transitions;
        samples[1].reward = f64::NAN;
        let mut learner = SensorTableLearner::new(71);
        let original = learner.clone();
        assert!(learner.learn(&samples).is_err());
        assert_eq!(learner, original);
        samples[1].reward = -1.0;
        learner.learn(&samples).unwrap();
        assert_eq!(learner.updates(), 2);
    }
}
