//! Deterministic finite-candidate PI selection, not general policy learning.

use super::*;
use sha2::{Digest, Sha256};

/// Policy gains; physical and estimator calibration parameters remain untouched.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PiGains {
    /// Proportional voltage gain per speed error, in V s/m.
    pub kp_v_s_m: f64,
    /// Integral voltage gain per integrated speed error, in V/m.
    pub ki_v_m: f64,
}

impl PiGains {
    /// Rejects non-finite or out-of-domain gains before constructing a world.
    pub fn validate(self) -> Result<()> {
        ensure!(
            [self.kp_v_s_m, self.ki_v_m]
                .iter()
                .all(|gain| gain.is_finite() && (0.0..=100.0).contains(gain)),
            "invalid fixed PI gain"
        );
        Ok(())
    }
}

/// One completed rollout's compact evaluator evidence (failures are stored separately).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PiEpisodeScore {
    /// Whole-horizon integrated absolute true speed error, in meters.
    pub tracking_error_integral_m: f64,
    /// Final true speed in m/s, including unsuccessful tasks.
    pub final_velocity_m_s: f64,
    /// Original final-speed acceptance; never used to filter rollouts.
    pub final_velocity_accepted: bool,
    /// SHA-256 of the full evaluation JSON, binding actions, reset, diagnostics and physics.
    /// This digest is replay evidence, not an authenticated provenance signature.
    pub evaluation_sha256: String,
}

/// A seed and its completed result or execution error, in requested seed order.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PiSeedOutcome {
    /// Explicit episode reset seed; independent sensor noise resets are not introduced here.
    pub episode_seed: u64,
    /// Backend/factory/policy failures are preserved, not converted to successful scores.
    pub outcome: std::result::Result<PiEpisodeScore, String>,
}

/// Training results for one candidate, without held-out data.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PiCandidateScore {
    /// Exact policy gains evaluated.
    pub gains: PiGains,
    /// All training seeds, including execution failures.
    pub training: Vec<PiSeedOutcome>,
    /// Mean training integral; absent if any training execution failed.
    pub mean_training_error_m: Option<f64>,
}

/// Diagnostic selection report; no claims of real-world generalization.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PiSelectionReport {
    /// Common fixed-period actor/action/reward/horizon contract.
    pub task_spec: TaskSpec,
    /// Candidates in caller order, which also resolves exact score ties.
    pub candidates: Vec<PiCandidateScore>,
    /// Minimum mean-training-error candidate, or none if every candidate had an error.
    pub selected_index: Option<usize>,
    /// Declared held-out seeds even if selection failed and none could be run.
    pub held_out_seeds: Vec<u64>,
    /// Only the selected policy's held-out results; never consulted during selection.
    pub held_out: Vec<PiSeedOutcome>,
}

/// Selects from 1..=32 unique PI candidates using 1..=64 training seeds, then runs
/// the winner on 1..=64 disjoint held-out seeds. Each seed list must be strictly
/// increasing. The factory constructs independent backends with a stable manifest.
/// Any candidate with an execution error is ineligible; failed speed gates remain
/// eligible and scored. Selection minimizes mean integral error, equivalently
/// maximizes this fixed-horizon TaskSpec's summed reward. Ties use caller order.
/// This function never retunes based on held-out results. Panics are not caught.
pub fn select_fixed_pi_policy<B: PhysicsBackend>(
    factory: impl Fn() -> Result<(B, PhysicsBackendManifest)>,
    candidates: &[PiGains],
    training_seeds: &[u64],
    held_out_seeds: &[u64],
) -> Result<PiSelectionReport> {
    ensure!(
        (1..=32).contains(&candidates.len()),
        "invalid PI candidate count"
    );
    for (index, gains) in candidates.iter().enumerate() {
        gains.validate()?;
        ensure!(
            !candidates[..index].contains(gains),
            "duplicate PI candidate"
        );
    }
    for seeds in [training_seeds, held_out_seeds] {
        ensure!(
            (1..=64).contains(&seeds.len()) && seeds.windows(2).all(|pair| pair[0] < pair[1]),
            "invalid PI seed list"
        );
    }
    ensure!(
        !training_seeds
            .iter()
            .any(|seed| held_out_seeds.binary_search(seed).is_ok()),
        "training/held-out seed overlap"
    );
    let mut expected_manifest = None;
    let mut run = |gains: PiGains, seed| {
        let result = (|| -> Result<PiEpisodeScore> {
            let (backend, manifest) = factory()?;
            if let Some(expected) = &expected_manifest {
                ensure!(expected == &manifest, "PI backend manifest changed");
            } else {
                expected_manifest = Some(manifest.clone());
            }
            let report =
                evaluate_fixed_pi_policy(backend, manifest, seed, gains.kp_v_s_m, gains.ki_v_m)?;
            Ok(PiEpisodeScore {
                tracking_error_integral_m: report.tracking_error_integral_m,
                final_velocity_m_s: report.final_velocity_m_s,
                final_velocity_accepted: report.final_velocity_accepted,
                evaluation_sha256: format!(
                    "sha256:{:x}",
                    Sha256::digest(serde_json::to_vec(&report)?)
                ),
            })
        })();
        PiSeedOutcome {
            episode_seed: seed,
            outcome: result.map_err(|error| format!("{error:#}")),
        }
    };
    let mut scores = Vec::with_capacity(candidates.len());
    let mut selected_index = None;
    let mut best_error_m = f64::INFINITY;
    for gains in candidates {
        let training: Vec<_> = training_seeds
            .iter()
            .map(|seed| run(*gains, *seed))
            .collect();
        let mean = training.iter().try_fold(0.0, |sum, episode| {
            episode
                .outcome
                .as_ref()
                .ok()
                .map(|score| sum + score.tracking_error_integral_m / training.len() as f64)
        });
        if let Some(mean) = mean {
            if mean < best_error_m {
                best_error_m = mean;
                selected_index = Some(scores.len());
            }
        }
        scores.push(PiCandidateScore {
            gains: *gains,
            training,
            mean_training_error_m: mean,
        });
    }
    let held_out = selected_index.map_or_else(Vec::new, |index| {
        held_out_seeds
            .iter()
            .map(|seed| run(candidates[index], *seed))
            .collect()
    });
    Ok(PiSelectionReport {
        task_spec: sensor_fixed_task_spec(),
        candidates: scores,
        selected_index,
        held_out_seeds: held_out_seeds.to_vec(),
        held_out,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use rne_physics_rapier::RapierBackend;
    use std::cell::Cell;

    fn factory() -> Result<(RapierBackend, PhysicsBackendManifest)> {
        Ok((RapierBackend::new(), RapierBackend::manifest()))
    }

    fn candidates() -> [PiGains; 4] {
        [
            PiGains {
                kp_v_s_m: 15.0,
                ki_v_m: 20.0,
            },
            PiGains {
                kp_v_s_m: 30.0,
                ki_v_m: 20.0,
            },
            PiGains {
                kp_v_s_m: 15.0,
                ki_v_m: 40.0,
            },
            PiGains {
                kp_v_s_m: 30.0,
                ki_v_m: 40.0,
            },
        ]
    }

    #[test]
    fn pi_selection_is_deterministic_and_held_out_cannot_change_training_choice() {
        let calls = Cell::new(0);
        let report = select_fixed_pi_policy(
            || {
                calls.set(calls.get() + 1);
                factory()
            },
            &candidates(),
            &[101, 102, 103, 104],
            &[1001, 1002, 1003, 1004],
        )
        .unwrap();
        assert_eq!(calls.get(), 20);
        assert_eq!(
            report,
            select_fixed_pi_policy(
                factory,
                &candidates(),
                &[101, 102, 103, 104],
                &[1001, 1002, 1003, 1004]
            )
            .unwrap()
        );
        let different =
            select_fixed_pi_policy(factory, &candidates(), &[101, 102, 103, 104], &[2001]).unwrap();
        assert_eq!(report.candidates, different.candidates);
        assert_eq!(report.selected_index, different.selected_index);
        let selected = report.candidates[report.selected_index.unwrap()].gains;
        for episode in &report.held_out {
            let (backend, manifest) = factory().unwrap();
            let replay = evaluate_fixed_pi_policy(
                backend,
                manifest,
                episode.episode_seed,
                selected.kp_v_s_m,
                selected.ki_v_m,
            )
            .unwrap();
            assert_eq!(
                episode.outcome.as_ref().unwrap().evaluation_sha256,
                format!(
                    "sha256:{:x}",
                    Sha256::digest(serde_json::to_vec(&replay).unwrap())
                )
            );
        }
        println!("PI selection {}", serde_json::to_string(&report).unwrap());
    }

    #[test]
    fn pi_selection_rejects_overlap_and_invalid_inputs_before_factory_calls() {
        let calls = Cell::new(0);
        let create = || {
            calls.set(calls.get() + 1);
            factory()
        };
        for (train, test) in [
            (vec![1], vec![1]),
            (vec![2, 1], vec![3]),
            (vec![1, 1], vec![3]),
            (vec![], vec![3]),
            (vec![1], vec![]),
        ] {
            assert!(select_fixed_pi_policy(create, &candidates(), &train, &test).is_err());
        }
        assert!(select_fixed_pi_policy(create, &[], &[1], &[2]).is_err());
        assert!(select_fixed_pi_policy(create, &[candidates()[0]; 2], &[1], &[2]).is_err());
        for invalid in [f64::NAN, f64::INFINITY, -1.0, 100.1] {
            assert!(select_fixed_pi_policy(
                create,
                &[PiGains {
                    kp_v_s_m: invalid,
                    ki_v_m: 20.0
                }],
                &[1],
                &[2]
            )
            .is_err());
        }
        assert_eq!(calls.get(), 0);
    }

    #[test]
    fn pi_selection_preserves_execution_errors_and_unsuccessful_tasks() {
        let calls = Cell::new(0);
        let failed = select_fixed_pi_policy::<RapierBackend>(
            || {
                calls.set(calls.get() + 1);
                anyhow::bail!("factory unavailable")
            },
            &candidates(),
            &[101, 102],
            &[1001],
        )
        .unwrap();
        assert_eq!(calls.get(), 8);
        assert_eq!(failed.selected_index, None);
        assert!(failed.held_out.is_empty());
        assert_eq!(failed.held_out_seeds, vec![1001]);
        assert!(failed.candidates.iter().all(|candidate| candidate
            .mean_training_error_m
            .is_none()
            && candidate
                .training
                .iter()
                .all(|episode| episode.outcome.is_err())));
        let zero = select_fixed_pi_policy(
            factory,
            &[PiGains {
                kp_v_s_m: 0.0,
                ki_v_m: 0.0,
            }],
            &[101],
            &[1001],
        )
        .unwrap();
        assert_eq!(zero.selected_index, Some(0));
        assert!(
            !zero.held_out[0]
                .outcome
                .as_ref()
                .unwrap()
                .final_velocity_accepted
        );
    }

    #[test]
    fn pi_selection_keeps_mixed_failures_and_does_not_reselect_after_held_out_errors() {
        let calls = Cell::new(0);
        let mixed = select_fixed_pi_policy(
            || {
                calls.set(calls.get() + 1);
                if calls.get() == 1 {
                    anyhow::bail!("first training failure");
                }
                if calls.get() > 8 {
                    anyhow::bail!("held-out failure");
                }
                factory()
            },
            &candidates(),
            &[101, 102],
            &[1001, 1002],
        )
        .unwrap();
        assert_eq!(calls.get(), 10);
        assert!(mixed.candidates[0].training[0].outcome.is_err());
        assert!(mixed.candidates[0].training[1].outcome.is_ok());
        assert!(mixed.candidates[0].mean_training_error_m.is_none());
        assert_ne!(mixed.selected_index, Some(0));
        assert!(mixed.selected_index.is_some());
        assert_eq!(mixed.held_out.len(), 2);
        assert!(mixed
            .held_out
            .iter()
            .all(|episode| episode.outcome.is_err()));
        let without_errors =
            select_fixed_pi_policy(factory, &candidates()[1..], &[101, 102], &[1001, 1002])
                .unwrap();
        assert_eq!(
            mixed.selected_index,
            without_errors.selected_index.map(|index| index + 1)
        );
    }

    #[cfg(feature = "mujoco")]
    #[test]
    fn pi_selected_on_rapier_transfers_to_mujoco_without_retuning() {
        use rne_physics_mujoco::MuJoCoBackend;
        let selection = select_fixed_pi_policy(
            factory,
            &candidates(),
            &[101, 102, 103, 104],
            &[1001, 1002, 1003, 1004],
        )
        .unwrap();
        let gains = selection.candidates[selection.selected_index.unwrap()].gains;
        for episode in &selection.held_out {
            let backend =
                MuJoCoBackend::new(SimDuration::from_ticks(SENSOR_OBSERVED_FIXED_DELTA_TICKS))
                    .unwrap();
            let second = evaluate_fixed_pi_policy(
                backend,
                MuJoCoBackend::manifest(),
                episode.episode_seed,
                gains.kp_v_s_m,
                gains.ki_v_m,
            )
            .unwrap();
            let first = episode.outcome.as_ref().unwrap();
            assert!((first.final_velocity_m_s - second.final_velocity_m_s).abs() <= 0.10);
            assert!(
                (first.tracking_error_integral_m - second.tracking_error_integral_m).abs() <= 0.10
            );
            println!("PI transfer seed={} kp={} ki={} rapier_speed={} mujoco_speed={} rapier_accepted={} mujoco_accepted={}", episode.episode_seed, gains.kp_v_s_m, gains.ki_v_m, first.final_velocity_m_s, second.final_velocity_m_s, first.final_velocity_accepted, second.final_velocity_accepted);
        }
    }
}
