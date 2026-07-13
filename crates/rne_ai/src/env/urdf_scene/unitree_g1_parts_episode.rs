use super::{
    unitree_g1_inspection_targets, unitree_g1_parts_pick_place_scene_path, UrdfJointPositionTarget,
    UrdfSceneSim,
};
use crate::{Episode, EpisodeStep};
use rne_assets::{AssetError, SceneAsset};
use std::path::PathBuf;

const SETTLE_STEPS: u64 = 120;
const CARRY_STEPS: u64 = 90;
const HOLD_BEFORE_RELEASE_STEPS: u64 = 60;
const PLACE_SETTLE_STEPS: u64 = 90;
const HAND_PROXY_SIZE_M: [f64; 3] = [0.14, 0.14, 0.14];
const MIN_LIFT_HEIGHT_M: f64 = 0.98;
const MIN_PLACED_HEIGHT_M: f64 = 0.84;
const MAX_PLACED_SPEED_M_S: f64 = 0.05;

/// Script phase reported by [`UnitreeG1PartsEpisode`].
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum UnitreeG1PartsPhase {
    /// Contact-gated attachment and arm carry motion.
    #[default]
    PickAndCarry,
    /// Hold the target pose until the payload velocity settles.
    Stabilize,
    /// Release and wait for the part to land in the tray.
    Place,
    /// The part landed in the semantic place zone.
    Complete,
}

/// Configuration for the fixed-base G1 parts task.
#[derive(Clone, Debug, PartialEq)]
pub struct UnitreeG1PartsEpisodeConfig {
    /// Scene containing the fixed-base G1, part, tray, and place marker.
    pub scene_path: PathBuf,
    /// Right-hand link name used as the grasp parent.
    pub hand_name: String,
    /// Dynamic workpiece entity name.
    pub part_name: String,
    /// Semantic place-zone marker name.
    pub place_marker_name: String,
    /// Maximum controlled steps before truncation.
    pub max_steps: u64,
}

impl Default for UnitreeG1PartsEpisodeConfig {
    fn default() -> Self {
        Self {
            scene_path: unitree_g1_parts_pick_place_scene_path(),
            hand_name: "right_wrist_roll_rubber_hand".into(),
            part_name: "g1_inspection_part".into(),
            place_marker_name: "parts_place_zone".into(),
            max_steps: 260,
        }
    }
}

/// Action controlling whether the scripted parts task advances.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct UnitreeG1PartsAction {
    /// When true, advance by one simulation step.
    pub advance: bool,
}

/// Observation emitted by [`UnitreeG1PartsEpisode`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct UnitreeG1PartsObservation {
    /// Current task phase.
    pub phase: UnitreeG1PartsPhase,
    /// Workpiece world position in meters.
    pub part_position_m: [f64; 3],
    /// Current workpiece height in meters.
    pub part_height_m: f64,
    /// Maximum workpiece height reached this episode in meters.
    pub max_part_height_m: f64,
    /// Workpiece linear speed in meters per second.
    pub part_speed_m_s: f64,
    /// Horizontal distance from the workpiece to the place marker in meters.
    pub place_distance_m: f64,
    /// Place-marker interaction radius in meters.
    pub place_radius_m: f64,
    /// Whether the hand and part touched during the latest physics step.
    pub hand_contact: bool,
    /// Whether the part currently carries the grasp weld.
    pub grasped: bool,
    /// Whether a contact-gated grasp occurred during this episode.
    pub was_grasped: bool,
    /// Whether the payload reached the required lift height.
    pub lifted: bool,
    /// Whether the released part is inside the semantic place zone.
    pub placed: bool,
}

/// Deterministic fixed-base G1 contact-grasp pick-and-place episode.
pub struct UnitreeG1PartsEpisode {
    config: UnitreeG1PartsEpisodeConfig,
    sim: UrdfSceneSim,
    episode_index: u32,
    step_in_episode: u64,
    was_grasped: bool,
    max_part_height_m: f64,
}

impl UnitreeG1PartsEpisode {
    /// Loads, configures, and settles the fixed-base G1 parts scene.
    pub fn new(config: UnitreeG1PartsEpisodeConfig) -> Result<Self, AssetError> {
        validate_scene_names(&config)?;
        let mut sim = configured_sim(&config)?;
        settle(&mut sim);
        let initial_height_m = sim
            .named_translation_m(&config.part_name)
            .expect("validated part")
            .1;
        Ok(Self {
            config,
            sim,
            episode_index: 0,
            step_in_episode: 0,
            was_grasped: false,
            max_part_height_m: initial_height_m,
        })
    }

    /// Returns the underlying simulation for read-only rendering and diagnostics.
    pub fn simulation(&self) -> &UrdfSceneSim {
        &self.sim
    }

    fn phase(&self) -> UnitreeG1PartsPhase {
        if self.success() {
            UnitreeG1PartsPhase::Complete
        } else if self.step_in_episode < CARRY_STEPS {
            UnitreeG1PartsPhase::PickAndCarry
        } else if self.step_in_episode < CARRY_STEPS + HOLD_BEFORE_RELEASE_STEPS {
            UnitreeG1PartsPhase::Stabilize
        } else {
            UnitreeG1PartsPhase::Place
        }
    }

    fn observation(&self) -> UnitreeG1PartsObservation {
        let part = self
            .sim
            .named_translation_m(&self.config.part_name)
            .expect("validated part");
        let marker = self
            .sim
            .task_marker(&self.config.place_marker_name)
            .expect("validated place marker");
        let place_distance_m = (part.0 - marker.0).hypot(part.2 - marker.2);
        let part_speed_m_s = self
            .sim
            .named_linear_speed_m_s(&self.config.part_name)
            .expect("validated dynamic part");
        let grasped = self.sim.named_child_is_welded(&self.config.part_name);
        let placed = self.was_grasped
            && !grasped
            && place_distance_m <= marker.3
            && part.1 >= MIN_PLACED_HEIGHT_M
            && part_speed_m_s <= MAX_PLACED_SPEED_M_S;
        UnitreeG1PartsObservation {
            phase: self.phase(),
            part_position_m: [part.0, part.1, part.2],
            part_height_m: part.1,
            max_part_height_m: self.max_part_height_m,
            part_speed_m_s,
            place_distance_m,
            place_radius_m: marker.3,
            hand_contact: self
                .sim
                .named_entities_in_contact(&self.config.hand_name, &self.config.part_name),
            grasped,
            was_grasped: self.was_grasped,
            lifted: self.max_part_height_m >= MIN_LIFT_HEIGHT_M,
            placed,
        }
    }

    fn success(&self) -> bool {
        if self.step_in_episode < CARRY_STEPS + HOLD_BEFORE_RELEASE_STEPS + PLACE_SETTLE_STEPS {
            return false;
        }
        let part = self
            .sim
            .named_translation_m(&self.config.part_name)
            .expect("validated part");
        let marker = self
            .sim
            .task_marker(&self.config.place_marker_name)
            .expect("validated place marker");
        self.was_grasped
            && self.max_part_height_m >= MIN_LIFT_HEIGHT_M
            && !self.sim.named_child_is_welded(&self.config.part_name)
            && (part.0 - marker.0).hypot(part.2 - marker.2) <= marker.3
            && part.1 >= MIN_PLACED_HEIGHT_M
            && self
                .sim
                .named_linear_speed_m_s(&self.config.part_name)
                .is_some_and(|speed_m_s| speed_m_s <= MAX_PLACED_SPEED_M_S)
    }
}

impl Episode for UnitreeG1PartsEpisode {
    type Observation = UnitreeG1PartsObservation;
    type Action = UnitreeG1PartsAction;

    fn reset(&mut self) -> EpisodeStep<Self::Observation> {
        self.sim = configured_sim(&self.config).expect("reload fixed-base G1 parts scene");
        settle(&mut self.sim);
        self.episode_index = self.episode_index.wrapping_add(1);
        self.step_in_episode = 0;
        self.was_grasped = false;
        self.max_part_height_m = self
            .sim
            .named_translation_m(&self.config.part_name)
            .expect("validated part")
            .1;
        EpisodeStep {
            observation: self.observation(),
            reward: 0.0,
            terminated: false,
            truncated: false,
        }
    }

    fn step(&mut self, action: Self::Action) -> EpisodeStep<Self::Observation> {
        let before = self.observation();
        if action.advance {
            if self.step_in_episode == 0 {
                self.was_grasped = self
                    .sim
                    .weld_named_child_on_contact(&self.config.hand_name, &self.config.part_name);
            }
            let hold = unitree_g1_inspection_targets(CARRY_STEPS - 1);
            if self.step_in_episode < CARRY_STEPS {
                self.sim
                    .step_joint_position_targets(&unitree_g1_inspection_targets(
                        self.step_in_episode,
                    ));
            } else {
                if self.step_in_episode == CARRY_STEPS + HOLD_BEFORE_RELEASE_STEPS {
                    self.sim.release_named_child(&self.config.part_name);
                }
                self.sim.step_joint_position_targets(&hold);
            }
            self.step_in_episode += 1;
            let height_m = self
                .sim
                .named_translation_m(&self.config.part_name)
                .expect("validated part")
                .1;
            self.max_part_height_m = self.max_part_height_m.max(height_m);
        }
        let observation = self.observation();
        let success = self.success();
        let lift_progress_m = (observation.part_height_m - before.part_height_m).max(0.0);
        let place_progress_m = if observation.grasped {
            0.0
        } else {
            before.place_distance_m - observation.place_distance_m
        };
        let reward = 4.0 * lift_progress_m
            + 3.0 * place_progress_m
            + if self.was_grasped && !before.was_grasped {
                2.0
            } else {
                0.0
            }
            + if success { 10.0 } else { 0.0 };
        EpisodeStep {
            observation,
            reward,
            terminated: success,
            truncated: self.step_in_episode >= self.config.max_steps && !success,
        }
    }

    fn episode_index(&self) -> u32 {
        self.episode_index
    }

    fn step_in_episode(&self) -> u64 {
        self.step_in_episode
    }
}

fn configured_sim(config: &UnitreeG1PartsEpisodeConfig) -> Result<UrdfSceneSim, AssetError> {
    let mut sim = UrdfSceneSim::from_scene_path(&config.scene_path)?;
    sim.configure_position_motors(220.0, 24.0, 88.0);
    if !sim.add_named_box_contact_proxy(&config.hand_name, HAND_PROXY_SIZE_M) {
        return Err(AssetError::Invalid {
            path: config.scene_path.display().to_string(),
            message: format!("could not add contact proxy to `{}`", config.hand_name),
        });
    }
    Ok(sim)
}

fn validate_scene_names(config: &UnitreeG1PartsEpisodeConfig) -> Result<(), AssetError> {
    let scene: SceneAsset = rne_assets::load_scene_asset(&config.scene_path)?;
    for name in [&config.part_name, &config.place_marker_name] {
        let exists = scene.objects.iter().any(|object| object.name == *name)
            || scene.task_markers.iter().any(|marker| marker.name == *name);
        if !exists {
            return Err(AssetError::Invalid {
                path: config.scene_path.display().to_string(),
                message: format!("missing named task entity `{name}`"),
            });
        }
    }
    Ok(())
}

fn settle(sim: &mut UrdfSceneSim) {
    for _ in 0..SETTLE_STEPS {
        sim.step_joint_position_targets(&standing_targets());
    }
}

fn standing_targets() -> [UrdfJointPositionTarget<'static>; 6] {
    [
        target("left_hip_pitch_link", -0.18),
        target("left_knee_link", 0.36),
        target("left_ankle_pitch_link", -0.18),
        target("right_hip_pitch_link", -0.18),
        target("right_knee_link", 0.36),
        target("right_ankle_pitch_link", -0.18),
    ]
}

fn target(link_name: &'static str, position: f64) -> UrdfJointPositionTarget<'static> {
    UrdfJointPositionTarget {
        link_name,
        position,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn replay_after_reset_is_exactly_deterministic() {
        let mut episode = UnitreeG1PartsEpisode::new(Default::default()).expect("parts episode");
        let mut first = None;
        for _ in 0..240 {
            first = Some(episode.step(UnitreeG1PartsAction { advance: true }));
        }
        episode.reset();
        let mut second = None;
        for _ in 0..240 {
            second = Some(episode.step(UnitreeG1PartsAction { advance: true }));
        }
        let final_step = first.as_ref().expect("final step");
        assert!(final_step.terminated);
        assert!(final_step.observation.was_grasped);
        assert!(final_step.observation.lifted);
        assert!(final_step.observation.placed);
        assert_eq!(final_step.observation.phase, UnitreeG1PartsPhase::Complete);
        assert!(final_step.reward > 9.0);
        assert_eq!(first, second);
    }

    #[test]
    fn weld_gate_rejects_non_contacting_entities() {
        let config = UnitreeG1PartsEpisodeConfig::default();
        let mut sim = configured_sim(&config).expect("configured scene");
        assert!(!sim.weld_named_child_on_contact(&config.hand_name, "inspection_parts_tray"));
        assert!(!sim.named_child_is_welded("inspection_parts_tray"));
    }
}
