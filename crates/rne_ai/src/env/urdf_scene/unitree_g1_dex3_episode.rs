use super::{
    unitree_g1_dex3_pick_targets, unitree_g1_dex3_scene_path, UnitreeG1Dex3HandCommand,
    UrdfSceneSim,
};
use crate::{Episode, EpisodeStep};
use rne_assets::{AssetError, SceneAsset};
use std::path::PathBuf;

const SETTLE_STEPS: u64 = 4;
const APPROACH_STEPS: u64 = 12;
const CLOSE_STEPS: u64 = 36;
const PINCH_SETTLE_STEPS: u64 = 44;
const LIFT_STEPS: u64 = 72;
const HOLD_STEPS: u64 = 8;
const OPEN_STEPS: u64 = 8;
const PLACE_SETTLE_STEPS: u64 = 60;
const THUMB_SENSOR_NAME: &str = "right_dex3_thumb_contact_sensor";
const INDEX_SENSOR_NAME: &str = "right_dex3_index_contact_sensor";
const GRASP_FRAME_NAME: &str = "right_dex3_grasp_frame";
const THUMB_SENSOR_SIZE_M: [f64; 3] = [0.026, 0.050, 0.026];
const THUMB_SENSOR_OFFSET_M: [f64; 3] = [0.0, 0.026, 0.0];
const INDEX_SENSOR_SIZE_M: [f64; 3] = [0.050, 0.026, 0.026];
const INDEX_SENSOR_OFFSET_M: [f64; 3] = [0.026, 0.0, 0.0];
const LIFT_START_STEP: u64 = APPROACH_STEPS + CLOSE_STEPS + PINCH_SETTLE_STEPS;
const RELEASE_STEP: u64 = LIFT_START_STEP + LIFT_STEPS + HOLD_STEPS;
const SUCCESS_STEP: u64 = RELEASE_STEP + PLACE_SETTLE_STEPS;
const GRASP_ANCHOR_M: [f64; 3] = [0.084, 0.068, 0.014];
const MIN_LIFT_HEIGHT_M: f64 = 0.98;
const MIN_PLACED_HEIGHT_M: f64 = 0.82;
const MAX_PLACED_SPEED_M_S: f64 = 0.05;

/// Script phase reported by [`UnitreeG1Dex3Episode`].
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum UnitreeG1Dex3Phase {
    /// Move the open hand around the part.
    #[default]
    Approach,
    /// Close the articulated thumb and fingers around the part.
    Close,
    /// Raise and carry a two-sided grasp.
    Lift,
    /// Stabilize the arm before release.
    Hold,
    /// Open the hand and let the part settle in the tray.
    Place,
    /// The released part is settled inside the place zone.
    Complete,
}

/// Configuration for the fixed-base G1 29-DoF + Dex3 task.
#[derive(Clone, Debug, PartialEq)]
pub struct UnitreeG1Dex3EpisodeConfig {
    /// Scene containing the official G1, Dex3 hand, part, and tray.
    pub scene_path: PathBuf,
    /// Palm link used as the parent of the grasp-follow frame.
    pub palm_name: String,
    /// Thumb-tip link used for the first side of the pinch gate.
    pub thumb_name: String,
    /// Index-tip link used for the second side of the pinch gate.
    pub index_name: String,
    /// Dynamic workpiece entity name.
    pub part_name: String,
    /// Semantic place-zone marker name.
    pub place_marker_name: String,
    /// Maximum controlled steps before truncation.
    pub max_steps: u64,
}

impl Default for UnitreeG1Dex3EpisodeConfig {
    fn default() -> Self {
        Self {
            scene_path: unitree_g1_dex3_scene_path(),
            palm_name: "right_hand_palm_link".into(),
            thumb_name: "right_hand_thumb_2_link".into(),
            index_name: "right_hand_index_1_link".into(),
            part_name: "dex3_inspection_part".into(),
            place_marker_name: "dex3_place_zone".into(),
            max_steps: SUCCESS_STEP + 8,
        }
    }
}

/// Action controlling whether the scripted Dex3 task advances.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct UnitreeG1Dex3Action {
    /// When true, advance by one simulation step.
    pub advance: bool,
}

/// Observation emitted by [`UnitreeG1Dex3Episode`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct UnitreeG1Dex3Observation {
    /// Current task phase.
    pub phase: UnitreeG1Dex3Phase,
    /// Workpiece world position in meters.
    pub part_position_m: [f64; 3],
    /// Maximum workpiece height reached in meters.
    pub max_part_height_m: f64,
    /// Workpiece linear speed in meters per second.
    pub part_speed_m_s: f64,
    /// Horizontal distance from the workpiece to the place marker in meters.
    pub place_distance_m: f64,
    /// Whether the thumb tip contacted the part in the latest physics step.
    pub thumb_contact: bool,
    /// Whether the index tip contacted the part in the latest physics step.
    pub index_contact: bool,
    /// Whether both sides contacted simultaneously in the latest physics step.
    pub dual_contact: bool,
    /// Distance between the thumb-tip and index-tip link origins in meters.
    pub pinch_gap_m: f64,
    /// Whether the part currently follows the contact-gated grasp frame.
    pub grasped: bool,
    /// Whether a two-sided contact-gated grasp occurred this episode.
    pub was_grasped: bool,
    /// Whether the payload reached the required lift height.
    pub lifted: bool,
    /// Whether the released payload is settled inside the place zone.
    pub placed: bool,
}

/// Deterministic fixed-base G1 29-DoF task with an articulated Dex3 pinch.
pub struct UnitreeG1Dex3Episode {
    config: UnitreeG1Dex3EpisodeConfig,
    sim: UrdfSceneSim,
    episode_index: u32,
    step_in_episode: u64,
    was_grasped: bool,
    grasped: bool,
    max_part_height_m: f64,
}

impl UnitreeG1Dex3Episode {
    /// Loads and configures the official G1 29-DoF + Dex3 scene.
    pub fn new(config: UnitreeG1Dex3EpisodeConfig) -> Result<Self, AssetError> {
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
            grasped: false,
            max_part_height_m: initial_height_m,
        })
    }

    /// Returns the underlying simulation for rendering and diagnostics.
    pub fn simulation(&self) -> &UrdfSceneSim {
        &self.sim
    }

    fn phase(&self) -> UnitreeG1Dex3Phase {
        if self.success() {
            UnitreeG1Dex3Phase::Complete
        } else if self.step_in_episode < APPROACH_STEPS {
            UnitreeG1Dex3Phase::Approach
        } else if self.step_in_episode < LIFT_START_STEP {
            UnitreeG1Dex3Phase::Close
        } else if self.step_in_episode < LIFT_START_STEP + LIFT_STEPS {
            UnitreeG1Dex3Phase::Lift
        } else if self.step_in_episode < RELEASE_STEP {
            UnitreeG1Dex3Phase::Hold
        } else {
            UnitreeG1Dex3Phase::Place
        }
    }

    fn observation(&self) -> UnitreeG1Dex3Observation {
        let part = self
            .sim
            .named_translation_m(&self.config.part_name)
            .expect("validated part");
        let marker = self
            .sim
            .task_marker(&self.config.place_marker_name)
            .expect("validated marker");
        let place_distance_m = (part.0 - marker.0).hypot(part.2 - marker.2);
        let part_speed_m_s = self
            .sim
            .named_linear_speed_m_s(&self.config.part_name)
            .expect("dynamic part");
        let thumb_contact = self
            .sim
            .named_entities_in_contact(THUMB_SENSOR_NAME, &self.config.part_name);
        let index_contact = self
            .sim
            .named_entities_in_contact(INDEX_SENSOR_NAME, &self.config.part_name);
        let grasped = self.grasped;
        let pinch_gap_m = pinch_gap_m(&self.sim, &self.config);
        let placed = self.was_grasped
            && !grasped
            && place_distance_m <= marker.3
            && part.1 >= MIN_PLACED_HEIGHT_M
            && part_speed_m_s <= MAX_PLACED_SPEED_M_S;
        UnitreeG1Dex3Observation {
            phase: self.phase(),
            part_position_m: [part.0, part.1, part.2],
            max_part_height_m: self.max_part_height_m,
            part_speed_m_s,
            place_distance_m,
            thumb_contact,
            index_contact,
            dual_contact: thumb_contact && index_contact,
            pinch_gap_m,
            grasped,
            was_grasped: self.was_grasped,
            lifted: self.max_part_height_m >= MIN_LIFT_HEIGHT_M,
            placed,
        }
    }

    fn success(&self) -> bool {
        self.step_in_episode >= SUCCESS_STEP && self.observation_without_phase().placed
    }

    fn observation_without_phase(&self) -> UnitreeG1Dex3Observation {
        let part = self
            .sim
            .named_translation_m(&self.config.part_name)
            .expect("validated part");
        let marker = self
            .sim
            .task_marker(&self.config.place_marker_name)
            .expect("validated marker");
        let place_distance_m = (part.0 - marker.0).hypot(part.2 - marker.2);
        let part_speed_m_s = self
            .sim
            .named_linear_speed_m_s(&self.config.part_name)
            .expect("dynamic part");
        let thumb_contact = self
            .sim
            .named_entities_in_contact(THUMB_SENSOR_NAME, &self.config.part_name);
        let index_contact = self
            .sim
            .named_entities_in_contact(INDEX_SENSOR_NAME, &self.config.part_name);
        let grasped = self.grasped;
        let pinch_gap_m = pinch_gap_m(&self.sim, &self.config);
        UnitreeG1Dex3Observation {
            phase: UnitreeG1Dex3Phase::Place,
            part_position_m: [part.0, part.1, part.2],
            max_part_height_m: self.max_part_height_m,
            part_speed_m_s,
            place_distance_m,
            thumb_contact,
            index_contact,
            dual_contact: thumb_contact && index_contact,
            pinch_gap_m,
            grasped,
            was_grasped: self.was_grasped,
            lifted: self.max_part_height_m >= MIN_LIFT_HEIGHT_M,
            placed: self.was_grasped
                && !grasped
                && place_distance_m <= marker.3
                && part.1 >= MIN_PLACED_HEIGHT_M
                && part_speed_m_s <= MAX_PLACED_SPEED_M_S,
        }
    }
}

impl Episode for UnitreeG1Dex3Episode {
    type Observation = UnitreeG1Dex3Observation;
    type Action = UnitreeG1Dex3Action;

    fn reset(&mut self) -> EpisodeStep<Self::Observation> {
        self.sim = configured_sim(&self.config).expect("reload G1 Dex3 scene");
        settle(&mut self.sim);
        self.episode_index = self.episode_index.wrapping_add(1);
        self.step_in_episode = 0;
        self.was_grasped = false;
        self.grasped = false;
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
            let step = self.step_in_episode;
            if step == RELEASE_STEP {
                self.grasped = false;
            }
            let (approach, lift, closure) = command_at_step(step);
            self.sim
                .step_joint_position_targets(&unitree_g1_dex3_pick_targets(
                    approach,
                    lift,
                    UnitreeG1Dex3HandCommand { closure },
                ));
            if !self.was_grasped
                && (APPROACH_STEPS..RELEASE_STEP).contains(&step)
                && self.sim.named_child_has_distinct_dual_contact(
                    THUMB_SENSOR_NAME,
                    INDEX_SENSOR_NAME,
                    &self.config.part_name,
                )
            {
                self.was_grasped = true;
                self.grasped = true;
            }
            if self.grasped
                && !self
                    .sim
                    .follow_named_body_to_frame(&self.config.part_name, GRASP_FRAME_NAME)
            {
                panic!("validated grasp follower entities");
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
        let lift_progress_m = (observation.part_position_m[1] - before.part_position_m[1]).max(0.0);
        let reward = 4.0 * lift_progress_m
            + if observation.dual_contact && !before.dual_contact {
                1.0
            } else {
                0.0
            }
            + if self.was_grasped && !before.was_grasped {
                3.0
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

fn command_at_step(step: u64) -> (f64, f64, f64) {
    if step < APPROACH_STEPS {
        ((step + 1) as f64 / APPROACH_STEPS as f64, 0.0, 0.0)
    } else if step < APPROACH_STEPS + CLOSE_STEPS {
        (
            1.0,
            0.0,
            (step - APPROACH_STEPS + 1) as f64 / CLOSE_STEPS as f64,
        )
    } else if step < LIFT_START_STEP {
        (1.0, 0.0, 1.0)
    } else if step < LIFT_START_STEP + LIFT_STEPS {
        (
            1.0,
            (step - LIFT_START_STEP + 1) as f64 / LIFT_STEPS as f64,
            1.0,
        )
    } else if step < RELEASE_STEP {
        (1.0, 1.0, 1.0)
    } else {
        (
            1.0,
            1.0,
            1.0 - ((step - RELEASE_STEP + 1) as f64 / OPEN_STEPS as f64).clamp(0.0, 1.0),
        )
    }
}

fn configured_sim(config: &UnitreeG1Dex3EpisodeConfig) -> Result<UrdfSceneSim, AssetError> {
    let mut sim = UrdfSceneSim::from_scene_path(&config.scene_path)?;
    sim.configure_position_motors(220.0, 24.0, 88.0);
    for (name, max_force_nm) in [
        ("right_hand_thumb_0_link", 2.45),
        ("right_hand_thumb_1_link", 1.4),
        ("right_hand_thumb_2_link", 1.4),
        ("right_hand_middle_0_link", 1.4),
        ("right_hand_middle_1_link", 1.4),
        ("right_hand_index_0_link", 1.4),
        ("right_hand_index_1_link", 1.4),
    ] {
        if !sim.configure_named_position_motor(name, 40.0, 4.0, max_force_nm) {
            return Err(invalid(config, format!("missing Dex3 motor `{name}`")));
        }
    }
    if !sim.add_named_child_frame(&config.palm_name, GRASP_FRAME_NAME, GRASP_ANCHOR_M) {
        return Err(invalid(config, "could not add Dex3 grasp frame"));
    }
    if !sim.add_named_child_box_sensor(
        &config.thumb_name,
        THUMB_SENSOR_NAME,
        THUMB_SENSOR_SIZE_M,
        THUMB_SENSOR_OFFSET_M,
    ) || !sim.add_named_child_box_sensor(
        &config.index_name,
        INDEX_SENSOR_NAME,
        INDEX_SENSOR_SIZE_M,
        INDEX_SENSOR_OFFSET_M,
    ) {
        return Err(invalid(config, "could not add Dex3 fingertip sensors"));
    }
    Ok(sim)
}

fn settle(sim: &mut UrdfSceneSim) {
    for _ in 0..SETTLE_STEPS {
        sim.step_joint_position_targets(&unitree_g1_dex3_pick_targets(
            0.0,
            0.0,
            UnitreeG1Dex3HandCommand { closure: 0.0 },
        ));
    }
}

fn pinch_gap_m(sim: &UrdfSceneSim, config: &UnitreeG1Dex3EpisodeConfig) -> f64 {
    let thumb = sim
        .named_transform(&config.thumb_name)
        .expect("validated thumb")
        .translation;
    let index = sim
        .named_transform(&config.index_name)
        .expect("validated index")
        .translation;
    thumb.distance(index)
}

fn validate_scene_names(config: &UnitreeG1Dex3EpisodeConfig) -> Result<(), AssetError> {
    let scene: SceneAsset = rne_assets::load_scene_asset(&config.scene_path)?;
    for name in [&config.part_name, &config.place_marker_name] {
        let exists = scene.objects.iter().any(|object| object.name == *name)
            || scene.task_markers.iter().any(|marker| marker.name == *name);
        if !exists {
            return Err(invalid(config, format!("missing task entity `{name}`")));
        }
    }
    Ok(())
}

fn invalid(config: &UnitreeG1Dex3EpisodeConfig, message: impl Into<String>) -> AssetError {
    AssetError::Invalid {
        path: config.scene_path.display().to_string(),
        message: message.into(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn dual_contact_gate_rejects_missing_or_duplicate_fingers() {
        let config = UnitreeG1Dex3EpisodeConfig::default();
        let sim = configured_sim(&config).expect("configured Dex3 scene");
        assert!(!sim.named_child_has_distinct_dual_contact(
            THUMB_SENSOR_NAME,
            THUMB_SENSOR_NAME,
            &config.part_name,
        ));
    }

    #[test]
    fn command_sequence_has_distinct_approach_close_lift_and_open_phases() {
        assert_eq!(command_at_step(0), (1.0 / APPROACH_STEPS as f64, 0.0, 0.0));
        assert_eq!(command_at_step(APPROACH_STEPS - 1), (1.0, 0.0, 0.0));
        assert_eq!(
            command_at_step(APPROACH_STEPS + CLOSE_STEPS - 1),
            (1.0, 0.0, 1.0)
        );
        assert_eq!(command_at_step(LIFT_START_STEP - 1), (1.0, 0.0, 1.0));
        assert_eq!(
            command_at_step(LIFT_START_STEP + LIFT_STEPS - 1),
            (1.0, 1.0, 1.0)
        );
        assert_eq!(command_at_step(RELEASE_STEP + OPEN_STEPS), (1.0, 1.0, 0.0));
    }

    #[test]
    fn dex3_episode_requires_two_contacts_and_replays_exactly() {
        let mut first = UnitreeG1Dex3Episode::new(Default::default()).expect("first episode");
        let mut second = UnitreeG1Dex3Episode::new(Default::default()).expect("second episode");
        let initial_gap_m = first.observation().pinch_gap_m;
        let mut saw_single_contact_before_grasp = false;
        let mut saw_grasp_transition = false;

        loop {
            let first_step = first.step(UnitreeG1Dex3Action { advance: true });
            let second_step = second.step(UnitreeG1Dex3Action { advance: true });
            assert_eq!(
                first_step, second_step,
                "identical episodes must replay exactly"
            );

            let observation = first_step.observation;
            if !observation.was_grasped && observation.thumb_contact ^ observation.index_contact {
                saw_single_contact_before_grasp = true;
                assert!(!observation.grasped, "one-sided contact must not grasp");
            }
            if observation.grasped && !saw_grasp_transition {
                saw_grasp_transition = true;
                assert!(observation.dual_contact, "grasp must start on dual contact");
                assert!(observation.pinch_gap_m < initial_gap_m);
            }

            if first_step.is_done() {
                assert!(
                    first_step.terminated,
                    "episode should complete, not truncate"
                );
                assert_eq!(first.step_in_episode(), SUCCESS_STEP);
                assert!(observation.was_grasped);
                assert!(observation.lifted);
                assert!(observation.placed);
                assert!(!observation.grasped);
                break;
            }
        }

        assert!(saw_single_contact_before_grasp);
        assert!(saw_grasp_transition);
    }
}
