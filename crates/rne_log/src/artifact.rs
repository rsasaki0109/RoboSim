//! Versioned, inspectable replay artifacts for fixed-step simulation runs.

use serde::{Deserialize, Serialize};
use std::fs;
use std::io;
use std::path::Path;
use thiserror::Error;

/// Current `.rne-replay` artifact schema version.
pub const REPLAY_ARTIFACT_VERSION: u32 = 1;

/// Replay artifact I/O, serialization, or schema validation failure.
#[derive(Debug, Error)]
pub enum ReplayArtifactError {
    /// The artifact could not be read or written.
    #[error(transparent)]
    Io(#[from] io::Error),
    /// The artifact could not be serialized or deserialized.
    #[error(transparent)]
    Json(#[from] serde_json::Error),
    /// The artifact uses a schema version unsupported by this engine.
    #[error("unsupported replay artifact version: expected {expected}, got {actual}")]
    UnsupportedVersion {
        /// Schema version supported by this engine.
        expected: u32,
        /// Schema version found in the artifact.
        actual: u32,
    },
    /// The artifact contains a value that cannot be replayed safely.
    #[error("invalid replay artifact: {0}")]
    Invalid(String),
}

/// Fixed-step clock metadata stored in a replay artifact.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ReplayClock {
    /// Number of recorded fixed simulation steps.
    pub steps: u64,
    /// Fixed simulation rate in hertz.
    pub hz: f64,
}

impl ReplayClock {
    /// Creates fixed-step replay clock metadata.
    pub const fn new(steps: u64, hz: f64) -> Self {
        Self { steps, hz }
    }
}

/// Controller boundary used while producing a replay artifact.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ReplayControllerKind {
    /// No non-zero actuator command was configured.
    #[default]
    None,
    /// Differential-drive wheel commands were recorded.
    DifferentialDrive,
}

/// One action sample recorded for a replay step.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ReplayAction {
    /// Wheel velocity command in radians per second.
    pub wheel_velocity_rad_s: f64,
}

impl ReplayAction {
    /// Creates a differential-drive wheel action.
    pub const fn differential_drive(wheel_velocity_rad_s: f64) -> Self {
        Self {
            wheel_velocity_rad_s,
        }
    }
}

/// Selected state observation recorded after one replay step.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ReplayObservation {
    /// First differential-drive base translation in metres, when present.
    pub base_translation_m: Option<[f64; 3]>,
}

impl ReplayObservation {
    /// Creates a selected observation for a replay frame.
    pub const fn new(base_translation_m: Option<[f64; 3]>) -> Self {
        Self { base_translation_m }
    }
}

/// One fixed-step action, observation, and deterministic state digest.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ReplayFrame {
    /// Zero-based fixed-step index.
    pub step: u64,
    /// Simulation time after this step in nanosecond ticks.
    pub sim_ticks: u64,
    /// Action applied during this step.
    pub action: ReplayAction,
    /// Selected state observed after this step.
    pub observation: ReplayObservation,
    /// Stable physics-world hash after this step.
    pub physics_hash: u64,
}

impl ReplayFrame {
    /// Creates one fixed-step replay frame.
    pub const fn new(
        step: u64,
        sim_ticks: u64,
        action: ReplayAction,
        observation: ReplayObservation,
        physics_hash: u64,
    ) -> Self {
        Self {
            step,
            sim_ticks,
            action,
            observation,
            physics_hash,
        }
    }
}

/// Final report captured alongside a replay artifact.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ReplayFinalReport {
    /// Number of fixed simulation steps.
    pub steps: u64,
    /// Final simulation time in seconds.
    pub sim_time_s: f64,
    /// World seed used by the run.
    pub seed: u64,
    /// Number of spawned robots.
    pub robot_count: usize,
    /// Number of spawned differential-drive robots.
    pub differential_drive_count: usize,
    /// Final physics-world hash.
    pub physics_hash: u64,
    /// First differential-drive base translation in metres, when present.
    pub first_base_translation_m: Option<[f64; 3]>,
}

impl ReplayFinalReport {
    /// Creates a final replay report.
    #[allow(clippy::too_many_arguments)]
    pub const fn new(
        steps: u64,
        sim_time_s: f64,
        seed: u64,
        robot_count: usize,
        differential_drive_count: usize,
        physics_hash: u64,
        first_base_translation_m: Option<[f64; 3]>,
    ) -> Self {
        Self {
            steps,
            sim_time_s,
            seed,
            robot_count,
            differential_drive_count,
            physics_hash,
            first_base_translation_m,
        }
    }
}

/// A self-contained fixed-step replay recording.
///
/// The artifact intentionally stores actions and selected observations rather
/// than a complete ECS snapshot. Replaying it reruns the scene and compares
/// every recorded frame hash and observation, while keeping the file small and
/// readable by tools such as the browser viewer.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ReplayArtifact {
    /// Artifact schema version.
    pub version: u32,
    /// Scene path used by the producing runner.
    pub scene: String,
    /// Root world seed used by the run.
    pub seed: u64,
    /// Fixed-step clock used by the run.
    pub clock: ReplayClock,
    /// Controller boundary used by the run.
    pub controller: ReplayControllerKind,
    /// Per-step actions, observations, and state hashes.
    pub frames: Vec<ReplayFrame>,
    /// Final report captured after the last frame.
    pub final_report: ReplayFinalReport,
}

impl ReplayArtifact {
    /// Creates a replay artifact from one completed fixed-step run.
    pub fn new(
        scene: impl Into<String>,
        seed: u64,
        clock: ReplayClock,
        controller: ReplayControllerKind,
        frames: Vec<ReplayFrame>,
        final_report: ReplayFinalReport,
    ) -> Self {
        Self {
            version: REPLAY_ARTIFACT_VERSION,
            scene: scene.into(),
            seed,
            clock,
            controller,
            frames,
            final_report,
        }
    }

    /// Validates the schema and deterministic invariants of this artifact.
    pub fn validate(&self) -> Result<(), ReplayArtifactError> {
        if self.version != REPLAY_ARTIFACT_VERSION {
            return Err(ReplayArtifactError::UnsupportedVersion {
                expected: REPLAY_ARTIFACT_VERSION,
                actual: self.version,
            });
        }
        if self.scene.trim().is_empty() {
            return Err(ReplayArtifactError::Invalid(
                "scene path must not be empty".to_string(),
            ));
        }
        if !self.clock.hz.is_finite() || self.clock.hz <= 0.0 {
            return Err(ReplayArtifactError::Invalid(
                "clock.hz must be finite and positive".to_string(),
            ));
        }
        if self.clock.steps != self.frames.len() as u64 {
            return Err(ReplayArtifactError::Invalid(format!(
                "clock.steps={} but frames contains {} entries",
                self.clock.steps,
                self.frames.len()
            )));
        }
        if self.final_report.steps != self.clock.steps {
            return Err(ReplayArtifactError::Invalid(format!(
                "final_report.steps={} but clock.steps={}",
                self.final_report.steps, self.clock.steps
            )));
        }
        if self.final_report.seed != self.seed {
            return Err(ReplayArtifactError::Invalid(format!(
                "final_report.seed={} but seed={}",
                self.final_report.seed, self.seed
            )));
        }
        if !self.final_report.sim_time_s.is_finite() || self.final_report.sim_time_s < 0.0 {
            return Err(ReplayArtifactError::Invalid(
                "final_report.sim_time_s must be finite and non-negative".to_string(),
            ));
        }
        validate_translation(
            self.final_report.first_base_translation_m,
            "final_report.first_base_translation_m",
        )?;
        if matches!(self.controller, ReplayControllerKind::None)
            && self
                .frames
                .iter()
                .any(|frame| frame.action.wheel_velocity_rad_s != 0.0)
        {
            return Err(ReplayArtifactError::Invalid(
                "controller=none requires zero wheel actions".to_string(),
            ));
        }

        let mut previous_sim_ticks = None;
        for (expected_step, frame) in self.frames.iter().enumerate() {
            let expected_step = expected_step as u64;
            if frame.step != expected_step {
                return Err(ReplayArtifactError::Invalid(format!(
                    "frame index {expected_step} has step {}",
                    frame.step
                )));
            }
            if let Some(previous_sim_ticks) = previous_sim_ticks {
                if frame.sim_ticks <= previous_sim_ticks {
                    return Err(ReplayArtifactError::Invalid(format!(
                        "frame {} has non-increasing sim_ticks {} after {}",
                        frame.step, frame.sim_ticks, previous_sim_ticks
                    )));
                }
            }
            previous_sim_ticks = Some(frame.sim_ticks);
            if !frame.action.wheel_velocity_rad_s.is_finite() {
                return Err(ReplayArtifactError::Invalid(format!(
                    "frame {} wheel_velocity_rad_s must be finite",
                    frame.step
                )));
            }
            validate_translation(
                frame.observation.base_translation_m,
                &format!("frame {} observation.base_translation_m", frame.step),
            )?;
        }
        Ok(())
    }

    /// Serializes a validated replay artifact as pretty JSON.
    pub fn to_json(&self) -> Result<String, ReplayArtifactError> {
        self.validate()?;
        Ok(serde_json::to_string_pretty(self)?)
    }

    /// Parses and validates a replay artifact from JSON text.
    pub fn from_json(text: &str) -> Result<Self, ReplayArtifactError> {
        let artifact: Self = serde_json::from_str(text)?;
        artifact.validate()?;
        Ok(artifact)
    }

    /// Writes a validated replay artifact to a JSON file.
    pub fn write_json(&self, path: impl AsRef<Path>) -> Result<(), ReplayArtifactError> {
        let path = path.as_ref();
        let text = self.to_json()?;
        if let Some(parent) = path.parent() {
            if !parent.as_os_str().is_empty() {
                fs::create_dir_all(parent)?;
            }
        }
        fs::write(path, text)?;
        Ok(())
    }

    /// Loads and validates a replay artifact from a JSON file.
    pub fn read_json(path: impl AsRef<Path>) -> Result<Self, ReplayArtifactError> {
        let text = fs::read_to_string(path)?;
        Self::from_json(&text)
    }
}

fn validate_translation(
    translation: Option<[f64; 3]>,
    field: &str,
) -> Result<(), ReplayArtifactError> {
    if let Some(translation) = translation {
        if translation.iter().any(|value| !value.is_finite()) {
            return Err(ReplayArtifactError::Invalid(format!(
                "{field} must contain only finite values"
            )));
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use tempfile::NamedTempFile;

    fn sample_artifact() -> ReplayArtifact {
        let frames = vec![
            ReplayFrame::new(
                0,
                16_666_666,
                ReplayAction::differential_drive(6.0),
                ReplayObservation::new(Some([0.1, 0.0, 0.0])),
                0x11,
            ),
            ReplayFrame::new(
                1,
                33_333_332,
                ReplayAction::differential_drive(6.0),
                ReplayObservation::new(Some([0.2, 0.0, 0.0])),
                0x22,
            ),
        ];
        ReplayArtifact::new(
            "assets/scenes/example.rne.scene.toml",
            42,
            ReplayClock::new(2, 60.0),
            ReplayControllerKind::DifferentialDrive,
            frames,
            ReplayFinalReport::new(2, 1.0 / 30.0, 42, 1, 1, 0x22, Some([0.2, 0.0, 0.0])),
        )
    }

    #[test]
    fn replay_artifact_roundtrips_json() {
        let artifact = sample_artifact();
        let file = NamedTempFile::new().unwrap();

        artifact.write_json(file.path()).unwrap();
        let loaded = ReplayArtifact::read_json(file.path()).unwrap();

        assert_eq!(loaded, artifact);
        assert!(loaded.to_json().unwrap().contains("\"version\": 1"));
    }

    #[test]
    fn replay_artifact_rejects_non_sequential_frames() {
        let mut artifact = sample_artifact();
        artifact.frames[1].step = 3;

        let error = artifact.validate().unwrap_err();

        assert!(error.to_string().contains("frame index 1 has step 3"));
    }

    #[test]
    fn replay_artifact_rejects_unknown_version() {
        let mut artifact = sample_artifact();
        artifact.version = REPLAY_ARTIFACT_VERSION + 1;

        let error = artifact.validate().unwrap_err();

        assert!(matches!(
            error,
            ReplayArtifactError::UnsupportedVersion {
                expected: REPLAY_ARTIFACT_VERSION,
                actual: 2
            }
        ));
    }

    #[test]
    fn replay_artifact_rejects_non_zero_none_controller_action() {
        let mut artifact = sample_artifact();
        artifact.controller = ReplayControllerKind::None;

        let error = artifact.validate().unwrap_err();

        assert!(error
            .to_string()
            .contains("controller=none requires zero wheel actions"));
    }
}
