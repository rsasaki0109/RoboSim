use super::{
    unitree_g1_factory_scene_path, unitree_g1_inspection_targets, UrdfJointPositionTarget,
    UrdfSceneSim,
};
use crate::{Episode, EpisodeStep};
use rne_assets::AssetError;
use std::path::PathBuf;

const SETTLE_STEPS: u64 = 120;
const INSPECTION_COMPLETE_STEP: u64 = 90;

/// Configuration for the G1 factory inspection episode.
#[derive(Clone, Debug, PartialEq)]
pub struct UnitreeG1InspectionEpisodeConfig {
    /// Factory scene containing G1 and the named inspection marker.
    pub scene_path: PathBuf,
    /// Ordered task marker entity names forming the inspection route.
    pub marker_names: Vec<String>,
    /// Maximum controlled steps before truncation.
    pub max_steps: u64,
}

impl Default for UnitreeG1InspectionEpisodeConfig {
    fn default() -> Self {
        Self {
            scene_path: unitree_g1_factory_scene_path(),
            marker_names: vec![
                "inspection_parts_check".into(),
                "inspection_safety_check".into(),
                "inspection_panel_check".into(),
            ],
            max_steps: 300,
        }
    }
}

/// Action controlling whether the scripted inspection sequence advances.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct UnitreeG1InspectionAction {
    /// When true, advance by one simulation step.
    pub advance: bool,
}

/// Observation emitted by [`UnitreeG1InspectionEpisode`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct UnitreeG1InspectionObservation {
    /// G1 pelvis world position in meters.
    pub base_position_m: [f64; 3],
    /// Horizontal distance to the inspection marker in meters.
    pub marker_distance_m: f64,
    /// Marker interaction radius in meters.
    pub marker_radius_m: f64,
    /// Point-and-confirm gesture progress in `[0, 1]`.
    pub gesture_progress: f64,
    /// Whether G1 is inside the marker interaction zone.
    pub inside_marker: bool,
    /// Zero-based index of the current marker in the ordered route.
    pub current_marker_index: usize,
    /// Number of markers already confirmed.
    pub completed_markers: usize,
    /// Total number of markers in the route.
    pub marker_count: usize,
}

/// Deterministic walk-to-marker and point-and-confirm factory task.
pub struct UnitreeG1InspectionEpisode {
    config: UnitreeG1InspectionEpisodeConfig,
    sim: UrdfSceneSim,
    episode_index: u32,
    step_in_episode: u64,
    step_at_marker: u64,
    current_marker_index: usize,
}

impl UnitreeG1InspectionEpisode {
    /// Loads and settles the factory G1 scene.
    pub fn new(config: UnitreeG1InspectionEpisodeConfig) -> Result<Self, AssetError> {
        let mut sim = UrdfSceneSim::from_scene_path(&config.scene_path)?;
        settle(&mut sim);
        if config.marker_names.is_empty() {
            return Err(AssetError::Invalid {
                path: config.scene_path.display().to_string(),
                message: "inspection route must contain at least one task marker".into(),
            });
        }
        for marker_name in &config.marker_names {
            if sim.task_marker(marker_name).is_none() {
                return Err(AssetError::Invalid {
                    path: config.scene_path.display().to_string(),
                    message: format!("missing task marker `{marker_name}`"),
                });
            }
        }
        Ok(Self {
            config,
            sim,
            episode_index: 0,
            step_in_episode: 0,
            step_at_marker: 0,
            current_marker_index: 0,
        })
    }

    fn observation(&self) -> UnitreeG1InspectionObservation {
        let base = self.sim.observe();
        let (marker_x_m, _, marker_z_m, marker_radius_m) = self
            .sim
            .task_marker(&self.config.marker_names[self.current_marker_index])
            .expect("validated inspection marker");
        let marker_distance_m = (base.base_x_m - marker_x_m).hypot(base.base_z_m - marker_z_m);
        UnitreeG1InspectionObservation {
            base_position_m: [base.base_x_m, base.base_y_m, base.base_z_m],
            marker_distance_m,
            marker_radius_m,
            gesture_progress: ((self.step_at_marker.saturating_sub(60)) as f64 / 30.0)
                .clamp(0.0, 1.0),
            inside_marker: marker_distance_m <= marker_radius_m,
            current_marker_index: self.current_marker_index,
            completed_markers: self.current_marker_index,
            marker_count: self.config.marker_names.len(),
        }
    }
}

impl Episode for UnitreeG1InspectionEpisode {
    type Observation = UnitreeG1InspectionObservation;
    type Action = UnitreeG1InspectionAction;

    fn reset(&mut self) -> EpisodeStep<Self::Observation> {
        self.sim = UrdfSceneSim::from_scene_path(&self.config.scene_path)
            .expect("reload Unitree G1 factory scene");
        settle(&mut self.sim);
        self.episode_index = self.episode_index.wrapping_add(1);
        self.step_in_episode = 0;
        self.step_at_marker = 0;
        self.current_marker_index = 0;
        EpisodeStep {
            observation: self.observation(),
            reward: 0.0,
            terminated: false,
            truncated: false,
        }
    }

    fn step(&mut self, action: Self::Action) -> EpisodeStep<Self::Observation> {
        let before_distance_m = self.observation().marker_distance_m;
        if action.advance {
            self.sim
                .step_joint_position_targets(&unitree_g1_inspection_targets(self.step_at_marker));
            self.step_in_episode += 1;
            self.step_at_marker += 1;
        }
        let mut observation = self.observation();
        let distance_progress_m = before_distance_m - observation.marker_distance_m;
        let marker_complete = observation.inside_marker
            && observation.gesture_progress >= 1.0
            && self.step_at_marker >= INSPECTION_COMPLETE_STEP;
        let mut success = false;
        if marker_complete {
            if self.current_marker_index + 1 == self.config.marker_names.len() {
                success = true;
                observation.completed_markers = self.config.marker_names.len();
            } else {
                self.current_marker_index += 1;
                self.step_at_marker = 0;
                observation = self.observation();
            }
        }
        let reward = 5.0 * distance_progress_m
            + 0.02 * observation.gesture_progress
            + if marker_complete { 3.0 } else { 0.0 }
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

fn settle(sim: &mut UrdfSceneSim) {
    sim.configure_position_motors(220.0, 24.0, 88.0);
    let targets = [
        target("left_hip_pitch_link", -0.18),
        target("left_knee_link", 0.36),
        target("left_ankle_pitch_link", -0.18),
        target("right_hip_pitch_link", -0.18),
        target("right_knee_link", 0.36),
        target("right_ankle_pitch_link", -0.18),
    ];
    for _ in 0..SETTLE_STEPS {
        sim.step_joint_position_targets(&targets);
    }
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
    fn factory_inspection_completes_inside_named_marker() {
        let mut episode = UnitreeG1InspectionEpisode::new(Default::default())
            .expect("factory inspection episode");
        let mut last = None;
        for _ in 0..INSPECTION_COMPLETE_STEP * 3 {
            last = Some(episode.step(UnitreeG1InspectionAction { advance: true }));
        }
        let last = last.expect("inspection step");
        assert!(last.observation.inside_marker);
        assert_eq!(last.observation.gesture_progress, 1.0);
        assert_eq!(last.observation.completed_markers, 3);
        assert!(last.terminated);
        assert!(last.reward > 9.0);
    }
}
