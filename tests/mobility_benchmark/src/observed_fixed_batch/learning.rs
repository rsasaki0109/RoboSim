//! Fallible learning projection; execution errors never become training samples.

use super::*;
use crate::observed::sensor_fixed_task_spec;

/// One valid action transition. Reward is evaluator output, not an actor tensor.
#[derive(Clone, Debug, PartialEq, serde::Serialize)]
pub struct LearningTransition {
    /// Stable lane identity.
    pub lane_id: usize,
    /// Explicit episode index; resets are not action transitions.
    pub episode_index: u64,
    /// TaskSpec-ordered sensor-only tensors before the action.
    pub observation: Vec<Vec<f64>>,
    /// Applied terminal voltage, held for this interval.
    pub action_v: f64,
    /// TaskSpec-ordered sensor-only tensors after the action.
    pub next_observation: Vec<Vec<f64>>,
    /// Evaluator reward from the completed interval.
    pub reward: f64,
    /// Start of the completed interval in simulation nanoseconds.
    pub start_ticks: u64,
    /// End of the completed interval in simulation nanoseconds.
    pub end_ticks: u64,
    /// Horizon truncation, never a success flag.
    pub truncated: bool,
}

/// A failed lane has no fabricated observation, reward or terminal transition.
#[derive(Clone, Debug, PartialEq, Eq, serde::Serialize)]
pub struct LearningLaneFailure {
    /// Stable lane identity requiring explicit reset before further stepping.
    pub lane_id: usize,
    /// Episode in which execution failed.
    pub episode_index: u64,
    /// Diagnostic error; not an optimizer input.
    pub error: String,
}

/// Successful samples and failed lanes are separate collections in stable order.
#[derive(Clone, Debug, PartialEq, serde::Serialize)]
pub struct LearningBatchStep {
    /// Only valid completed transitions may be offered to a learner.
    pub transitions: Vec<LearningTransition>,
    /// Execution failures requiring reset; healthy transitions remain available.
    pub failures: Vec<LearningLaneFailure>,
}

fn actor_values(observation: &SensorFixedObservation) -> Result<Vec<Vec<f64>>> {
    let values = observation.actor_tensors();
    let task = sensor_fixed_task_spec();
    ensure!(
        values.len() == task.observation.tensors.len(),
        "actor tensor count drift"
    );
    for (value, spec) in values.iter().zip(&task.observation.tensors) {
        ensure!(
            value.len() == spec.shape.iter().product::<usize>()
                && value.iter().all(|v| v.is_finite()),
            "actor tensor shape/value drift"
        );
    }
    Ok(values)
}

impl<B, F> SensorFixedBatch<B, F>
where
    B: PhysicsBackend,
    F: Fn() -> Result<(B, PhysicsBackendManifest)>,
{
    /// Steps persistent worlds and projects only valid results into learning samples.
    ///
    /// Preflight errors return before physics advances. Per-lane execution errors
    /// retain other lanes' transitions and require explicit reset. No autoreset,
    /// success fabrication, policy update or optimizer checkpoint is performed.
    pub fn step_learning(&mut self, actions_v: &[f64]) -> Result<LearningBatchStep> {
        let before: Vec<_> = self
            .observations()
            .into_iter()
            .map(|result| {
                let observation = result?;
                Ok((observation.time_ticks, actor_values(&observation)?))
            })
            .collect::<Result<_>>()?;
        let outcomes = self.step(actions_v)?;
        let mut result = LearningBatchStep {
            transitions: Vec::new(),
            failures: Vec::new(),
        };
        for lane in outcomes {
            let transition = lane.outcome.map_err(anyhow::Error::msg).and_then(|step| {
                let (start_ticks, observation) = &before[lane.lane_id];
                ensure!(
                    step.observation.time_ticks.checked_sub(*start_ticks) == Some(10_000_000)
                        && step.reward.is_finite(),
                    "invalid completed learning interval"
                );
                Ok(LearningTransition {
                    lane_id: lane.lane_id,
                    episode_index: lane.episode_index,
                    observation: observation.clone(),
                    action_v: actions_v[lane.lane_id],
                    next_observation: actor_values(&step.observation)?,
                    reward: step.reward,
                    start_ticks: *start_ticks,
                    end_ticks: step.observation.time_ticks,
                    truncated: step.truncated,
                })
            });
            match transition {
                Ok(transition) => result.transitions.push(transition),
                Err(error) => {
                    self.lanes[lane.lane_id]
                        .environment
                        .reject_learning_output();
                    result.failures.push(LearningLaneFailure {
                        lane_id: lane.lane_id,
                        episode_index: lane.episode_index,
                        error: format!("{error:#}"),
                    });
                }
            }
        }
        Ok(result)
    }
}
