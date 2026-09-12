//! Strict, bounded learner snapshots; physical worlds are restored separately.

use super::*;
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};

const MAX_BYTES: usize = 1024 * 1024;
const MAX_ENTRIES: usize = 330 * 18;
const ALGORITHM: &str = "rne_sensor_table_q_v1_alpha_0.1_gamma_0.99_epsilon_0.2";

#[derive(Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct Entry {
    state: State,
    // Integer IEEE-754 encodings preserve every bit across JSON implementations.
    value_bits: [u64; 5],
}

#[derive(Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct Checkpoint {
    schema_version: u32,
    algorithm: String,
    task_spec: rne_ai::TaskSpec,
    exploration_seed: u64,
    next_decision_index: u64,
    updates: u64,
    entries: Vec<Entry>,
    content_sha256: String,
}

impl Checkpoint {
    fn digest(&self) -> Result<String> {
        // Callers clear the digest before computing it.
        Ok(format!("{:x}", Sha256::digest(serde_json::to_vec(self)?)))
    }
}

impl SensorTableLearner {
    /// Encodes a bounded, versioned learner checkpoint with bit-exact Q values.
    ///
    /// Includes the independent exploration seed, update counter and caller's
    /// next global decision coordinate. The caller must supply that coordinate
    /// correctly across resets. This is not a physical-world snapshot: restore
    /// matching worlds through verified action/reset replay before continuing.
    /// The digest detects corruption, not an adversary who can recompute it.
    pub fn checkpoint(&self, next_decision_index: u64) -> Result<Vec<u8>> {
        let mut checkpoint = Checkpoint {
            schema_version: 1,
            algorithm: ALGORITHM.to_owned(),
            task_spec: crate::observed::sensor_fixed_task_spec(),
            exploration_seed: self.exploration.root_seed(),
            next_decision_index,
            updates: self.updates,
            entries: self
                .values
                .iter()
                .map(|(state, values)| Entry {
                    state: *state,
                    value_bits: values.map(f64::to_bits),
                })
                .collect(),
            content_sha256: String::new(),
        };
        checkpoint.content_sha256 = checkpoint.digest()?;
        let bytes = serde_json::to_vec(&checkpoint)?;
        // Apply the same bounds and invariants on both producer and consumer.
        Self::from_checkpoint(&bytes)?;
        Ok(bytes)
    }

    /// Decodes at most 1 MiB without mutating an existing learner or any worlds.
    /// Rejects unknown fields, unsupported algorithms/TaskSpecs, duplicate or
    /// unordered states, invalid bins, nonfinite Q values and digest mismatch.
    /// Returns the learner and saved next global decision index, not a restored
    /// training session. Fixed hyperparameters have no additional optimizer state.
    pub fn from_checkpoint(bytes: &[u8]) -> Result<(Self, u64)> {
        ensure!(bytes.len() <= MAX_BYTES, "learner checkpoint exceeds 1 MiB");
        let mut checkpoint: Checkpoint = serde_json::from_slice(bytes)?;
        ensure!(
            serde_json::from_slice::<serde_json::Value>(bytes)?
                == serde_json::to_value(&checkpoint)?,
            "checkpoint contains unsupported nested fields or numeric representations"
        );
        ensure!(
            checkpoint.schema_version == 1 && checkpoint.algorithm == ALGORITHM,
            "unsupported learner checkpoint algorithm/schema"
        );
        ensure!(
            checkpoint.task_spec == crate::observed::sensor_fixed_task_spec(),
            "learner checkpoint TaskSpec mismatch"
        );
        ensure!(
            checkpoint.entries.len() <= MAX_ENTRIES
                && checkpoint.updates >= checkpoint.entries.len() as u64,
            "invalid checkpoint table/update count"
        );
        let expected = std::mem::take(&mut checkpoint.content_sha256);
        ensure!(
            expected == checkpoint.digest()?,
            "learner checkpoint digest mismatch"
        );
        let mut learner = Self::new(checkpoint.exploration_seed);
        learner.updates = checkpoint.updates;
        let mut previous = None;
        for entry in checkpoint.entries {
            let key = entry.state;
            ensure!(
                key[0] < 330
                    && ((key[1] == 0 && key[2] == 0)
                        || (key[1] == 1 && (1..=17).contains(&key[2]))),
                "invalid checkpoint state bin"
            );
            ensure!(
                previous.is_none_or(|value| value < key),
                "checkpoint states must be strictly sorted"
            );
            let values = entry.value_bits.map(f64::from_bits);
            ensure!(
                values.iter().all(|value| value.is_finite()),
                "nonfinite checkpoint Q value"
            );
            learner.values.insert(key, values);
            previous = Some(key);
        }
        Ok((learner, checkpoint.next_decision_index))
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
        let mut original = SensorFixedBatch::new_with_noise_root(factory, 530, 531, 2, 2).unwrap();
        let mut learner = SensorTableLearner::new(532);
        let mut history = Vec::new();
        for decision in 0..50 {
            if decision == 20 {
                original.reset_lanes(&[(1, 2)]).unwrap();
            }
            let output = learner.train_step(&mut original, decision).unwrap();
            assert!(output.failures.is_empty());
            let actions = output
                .transitions
                .iter()
                .map(|t| t.action_v)
                .collect::<Vec<_>>();
            history.push((actions, output));
        }
        let bytes = learner.checkpoint(50).unwrap();
        let (mut restored, next) = SensorTableLearner::from_checkpoint(&bytes).unwrap();
        assert_eq!(next, 50);
        assert_eq!(restored, learner);
        assert_eq!(bytes, restored.checkpoint(next).unwrap());
        // Reconstruct only after deserialization, without retaining backend state
        // or updating the restored learner during history replay.
        let mut replayed = SensorFixedBatch::new_with_noise_root(factory, 530, 531, 2, 1).unwrap();
        for (decision, (actions, output)) in history.into_iter().enumerate() {
            if decision == 20 {
                replayed.reset_lanes(&[(1, 2)]).unwrap();
            }
            assert_eq!(output, replayed.step_learning(&actions).unwrap());
        }
        for decision in next..100 {
            assert_eq!(
                learner.train_step(&mut original, decision).unwrap(),
                restored.train_step(&mut replayed, decision).unwrap()
            );
        }
        assert_eq!(learner, restored);
        assert_eq!(learner.updates(), 200);
    }

    #[test]
    fn learner_checkpoint_rapier_resumes_after_action_and_partial_reset_replay() {
        resume(|| {
            Ok((
                rne_physics_rapier::RapierBackend::new(),
                rne_physics_rapier::RapierBackend::manifest(),
            ))
        });
    }

    #[cfg(feature = "mujoco")]
    #[test]
    fn learner_checkpoint_mujoco_resumes_after_action_and_partial_reset_replay() {
        resume(|| {
            Ok((
                rne_physics_mujoco::MuJoCoBackend::new(rne_core::SimDuration::from_ticks(
                    1_000_000,
                ))?,
                rne_physics_mujoco::MuJoCoBackend::manifest(),
            ))
        });
    }

    #[test]
    fn learner_checkpoint_rejects_corruption_and_rehashed_invalid_tables() {
        let mut learner = SensorTableLearner::new(532);
        learner
            .values
            .insert([0, 0, 0], [-0.0, -0.1, 1e-300, 0.3, 0.0]);
        learner.updates = 1;
        let bytes = learner.checkpoint(1).unwrap();
        assert_eq!(
            bytes,
            SensorTableLearner::from_checkpoint(&bytes)
                .unwrap()
                .0
                .checkpoint(1)
                .unwrap()
        );
        for mutation in 0..7 {
            let mut wire: Checkpoint = serde_json::from_slice(&bytes).unwrap();
            match mutation {
                0 => wire.entries[0].value_bits[0] = f64::NAN.to_bits(),
                1 => wire.entries[0].state = [330, 0, 0],
                2 => {
                    wire.updates = 2;
                    wire.entries.push(Entry {
                        state: [0, 0, 0],
                        value_bits: [0; 5],
                    });
                }
                3 => wire.entries[0].state = [0, 0, 1],
                4 => wire.updates = 0,
                5 => wire.algorithm.push('x'),
                _ => wire.schema_version = 2,
            }
            wire.content_sha256.clear();
            wire.content_sha256 = wire.digest().unwrap();
            assert!(
                SensorTableLearner::from_checkpoint(&serde_json::to_vec(&wire).unwrap()).is_err()
            );
        }
        let mut value: serde_json::Value = serde_json::from_slice(&bytes).unwrap();
        value["task_spec"]["unknown"] = true.into();
        assert!(SensorTableLearner::from_checkpoint(&serde_json::to_vec(&value).unwrap()).is_err());
        let mut value: serde_json::Value = serde_json::from_slice(&bytes).unwrap();
        value["exploration_seed"] = 999.into();
        assert!(SensorTableLearner::from_checkpoint(&serde_json::to_vec(&value).unwrap()).is_err());
        value["unknown"] = true.into();
        assert!(SensorTableLearner::from_checkpoint(&serde_json::to_vec(&value).unwrap()).is_err());
        assert!(SensorTableLearner::from_checkpoint(&vec![b' '; MAX_BYTES + 1]).is_err());
    }
}
