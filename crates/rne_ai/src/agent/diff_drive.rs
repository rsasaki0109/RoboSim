//! Diff-drive agent runner with attachable policies.

use super::components::AttachedPolicy;
use crate::action::DiffDriveAction;
use crate::env::DiffDriveEpisode;
use crate::episode::{Episode, EpisodeStep};
use crate::observation::DiffDriveObservation;
use crate::policy::Policy;
use crate::DiffDriveEpisodeConfig;
use bevy_ecs::prelude::Component;
use rne_ecs::{Entity, World};

/// Type-erased diff-drive policy for storage on agent entities.
pub trait DiffDrivePolicySource: Send + Sync {
    /// Chooses the next action from the latest observation.
    fn act(&mut self, observation: &DiffDriveObservation) -> DiffDriveAction;
}

impl<P> DiffDrivePolicySource for P
where
    P: Policy<DiffDriveEpisode> + Send + Sync,
{
    fn act(&mut self, observation: &DiffDriveObservation) -> DiffDriveAction {
        Policy::act(self, observation)
    }
}

/// Runs a diff-drive episode with an attachable policy.
#[derive(Component)]
pub struct DiffDriveAgentState {
    episode: DiffDriveEpisode,
    policy: Option<Box<dyn DiffDrivePolicySource>>,
    last_step: EpisodeStep<DiffDriveObservation>,
}

impl DiffDriveAgentState {
    /// Creates agent state without an attached policy.
    pub fn new(config: DiffDriveEpisodeConfig) -> Self {
        Self {
            episode: DiffDriveEpisode::new(config),
            policy: None,
            last_step: EpisodeStep {
                observation: DiffDriveObservation::default(),
                reward: 0.0,
                terminated: false,
                truncated: false,
            },
        }
    }

    /// Attaches a policy that will be used on the next reset/step.
    pub fn attach_policy<P>(&mut self, policy: P)
    where
        P: Policy<DiffDriveEpisode> + Send + Sync + 'static,
    {
        self.policy = Some(Box::new(policy));
    }

    /// Returns true when a policy has been attached.
    pub fn has_policy(&self) -> bool {
        self.policy.is_some()
    }

    /// Resets the underlying episode and returns the initial step.
    pub fn reset(&mut self) -> EpisodeStep<DiffDriveObservation> {
        self.last_step = self.episode.reset();
        self.last_step
    }

    /// Applies the attached policy and advances the episode by one tick.
    pub fn step(&mut self) -> EpisodeStep<DiffDriveObservation> {
        let Some(policy) = self.policy.as_mut() else {
            panic!("diff-drive agent stepped without an attached policy");
        };

        if self.last_step.is_done() {
            return self.last_step;
        }

        let action = policy.act(&self.last_step.observation);
        self.last_step = self.episode.step(action);
        self.last_step
    }

    /// Returns the latest episode step.
    pub fn last_step(&self) -> EpisodeStep<DiffDriveObservation> {
        self.last_step
    }

    /// Returns read access to the underlying episode.
    pub fn episode(&self) -> &DiffDriveEpisode {
        &self.episode
    }
}

/// Attaches a policy to a diff-drive agent entity.
pub fn attach_diff_drive_policy<P>(world: &mut World, agent: Entity, policy: P)
where
    P: Policy<DiffDriveEpisode> + Send + Sync + 'static,
{
    let mut entity = world
        .get_entity_mut(agent)
        .expect("diff-drive agent entity must exist");
    let mut state = entity
        .get_mut::<DiffDriveAgentState>()
        .expect("entity must have DiffDriveAgentState");
    state.attach_policy(policy);
    entity.insert(AttachedPolicy);
}
