//! Versioned, inspectable replay artifacts for fixed-step simulation runs.

use serde::{Deserialize, Deserializer, Serialize};
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
    /// Named joint velocity commands were recorded.
    JointVelocity,
    /// Named joint effort commands were recorded.
    JointEffort,
}

/// One action sample recorded for a replay step.
#[derive(Clone, Debug, PartialEq, Serialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum ReplayAction {
    /// Differential-drive wheel velocity command in radians per second.
    DifferentialDrive {
        /// Wheel velocity command in radians per second.
        wheel_velocity_rad_s: f64,
    },
    /// Named joint velocity command.
    JointVelocity {
        /// URDF / ECS joint name.
        joint: String,
        /// Target velocity in radians per second.
        velocity_rad_s: f64,
    },
    /// Named joint effort command.
    JointEffort {
        /// URDF / ECS joint name.
        joint: String,
        /// Target effort in newton-meters.
        effort_nm: f64,
    },
}

impl<'de> Deserialize<'de> for ReplayAction {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        #[derive(Deserialize)]
        #[serde(tag = "kind", rename_all = "snake_case")]
        enum TaggedAction {
            DifferentialDrive { wheel_velocity_rad_s: f64 },
            JointVelocity { joint: String, velocity_rad_s: f64 },
            JointEffort { joint: String, effort_nm: f64 },
        }

        #[derive(Deserialize)]
        #[serde(deny_unknown_fields)]
        struct LegacyAction {
            wheel_velocity_rad_s: f64,
        }

        #[derive(Deserialize)]
        #[serde(untagged)]
        enum WireAction {
            Tagged(TaggedAction),
            Legacy(LegacyAction),
        }

        match WireAction::deserialize(deserializer)? {
            WireAction::Tagged(action) => Ok(match action {
                TaggedAction::DifferentialDrive {
                    wheel_velocity_rad_s,
                } => Self::DifferentialDrive {
                    wheel_velocity_rad_s,
                },
                TaggedAction::JointVelocity {
                    joint,
                    velocity_rad_s,
                } => Self::JointVelocity {
                    joint,
                    velocity_rad_s,
                },
                TaggedAction::JointEffort { joint, effort_nm } => {
                    Self::JointEffort { joint, effort_nm }
                }
            }),
            WireAction::Legacy(action) => Ok(Self::DifferentialDrive {
                wheel_velocity_rad_s: action.wheel_velocity_rad_s,
            }),
        }
    }
}

impl ReplayAction {
    /// Creates a differential-drive wheel action.
    pub const fn differential_drive(wheel_velocity_rad_s: f64) -> Self {
        Self::DifferentialDrive {
            wheel_velocity_rad_s,
        }
    }

    /// Creates a named joint velocity action.
    pub fn joint_velocity(joint: impl Into<String>, velocity_rad_s: f64) -> Self {
        Self::JointVelocity {
            joint: joint.into(),
            velocity_rad_s,
        }
    }

    /// Creates a named joint effort action.
    pub fn joint_effort(joint: impl Into<String>, effort_nm: f64) -> Self {
        Self::JointEffort {
            joint: joint.into(),
            effort_nm,
        }
    }

    /// Returns the controller boundary represented by this action.
    pub const fn controller_kind(&self) -> ReplayControllerKind {
        match self {
            Self::DifferentialDrive { .. } => ReplayControllerKind::DifferentialDrive,
            Self::JointVelocity { .. } => ReplayControllerKind::JointVelocity,
            Self::JointEffort { .. } => ReplayControllerKind::JointEffort,
        }
    }

    /// Returns whether this action carries no actuator input.
    pub fn is_zero(&self) -> bool {
        match self {
            Self::DifferentialDrive {
                wheel_velocity_rad_s,
            } => *wheel_velocity_rad_s == 0.0,
            Self::JointVelocity {
                joint,
                velocity_rad_s,
            } => joint.trim().is_empty() || *velocity_rad_s == 0.0,
            Self::JointEffort { joint, effort_nm } => joint.trim().is_empty() || *effort_nm == 0.0,
        }
    }
}

/// Named joint state captured in a replay observation.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ReplayJointState {
    /// Joint names matching the position and velocity arrays.
    pub names: Vec<String>,
    /// Joint positions in radians, in name order.
    pub positions_rad: Vec<f64>,
    /// Joint velocities in radians per second, in name order.
    pub velocities_rad_s: Vec<f64>,
}

/// Summary of one typed sensor stream captured in a replay observation.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ReplaySensorStream {
    /// DataBus stream identifier.
    pub stream_id: u64,
    /// Stable sensor kind label.
    pub kind: String,
    /// Number of samples emitted by this sensor so far.
    pub frame_count: u64,
    /// Last emitted sequence number, or zero when no sample exists.
    pub last_sequence: u64,
    /// Stable digest of the latest typed payload, or zero when no sample exists.
    pub payload_hash: u64,
}

/// Full typed payload captured for a subscribed sensor stream.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum ReplaySensorPayloadData {
    /// Inertial measurement unit sample.
    Imu(rne_data::ImuSample),
    /// LiDAR point cloud.
    Lidar(rne_data::PointCloud),
    /// RGB camera frame with its paired depth image.
    Camera {
        /// RGB frame payload.
        rgb: rne_data::ImageRgb8,
        /// Depth frame payload from the paired depth stream.
        depth: rne_data::ImageDepth,
    },
    /// Wheel encoder sample.
    WheelEncoder(rne_data::WheelEncoderSample),
}

/// One full sensor payload recorded in a replay observation.
///
/// Payloads are only recorded for streams selected by the producing run's
/// sensor subscriptions, so the artifact stays compact unless payload capture
/// is requested.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ReplaySensorPayload {
    /// DataBus stream identifier.
    pub stream_id: u64,
    /// Stable sensor kind label.
    pub kind: String,
    /// Sequence number of the captured sample, or zero when none exists.
    pub sequence: u64,
    /// Typed payload data.
    pub data: ReplaySensorPayloadData,
}

/// Selected state observation recorded after one replay step.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ReplayObservation {
    /// First differential-drive base translation in metres, when present.
    pub base_translation_m: Option<[f64; 3]>,
    /// Named joint state when the scene contains articulated joints.
    #[serde(default)]
    pub joint_state: Option<ReplayJointState>,
    /// Per-sensor DataBus stream summaries captured after this step.
    #[serde(default)]
    pub sensor_streams: Vec<ReplaySensorStream>,
    /// Full typed payloads for manifest-subscribed sensor streams.
    #[serde(default)]
    pub sensor_payloads: Vec<ReplaySensorPayload>,
}

impl ReplayObservation {
    /// Creates a selected observation for a replay frame.
    pub const fn new(base_translation_m: Option<[f64; 3]>) -> Self {
        Self {
            base_translation_m,
            joint_state: None,
            sensor_streams: Vec::new(),
            sensor_payloads: Vec::new(),
        }
    }

    /// Adds the named joint state captured for this observation.
    pub fn with_joint_state(mut self, joint_state: Option<ReplayJointState>) -> Self {
        self.joint_state = joint_state;
        self
    }

    /// Adds typed sensor stream summaries captured for this observation.
    pub fn with_sensor_streams(mut self, sensor_streams: Vec<ReplaySensorStream>) -> Self {
        self.sensor_streams = sensor_streams;
        self
    }

    /// Adds full typed sensor payloads captured for this observation.
    pub fn with_sensor_payloads(mut self, sensor_payloads: Vec<ReplaySensorPayload>) -> Self {
        self.sensor_payloads = sensor_payloads;
        self
    }
}

/// One fixed-step action, observation, and deterministic state digest.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
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
    /// Streams whose full typed payloads were captured, sorted and unique.
    #[serde(default)]
    pub sensor_payload_streams: Vec<u64>,
    /// Per-step actions, observations, and state hashes.
    pub frames: Vec<ReplayFrame>,
    /// Final report captured after the last frame.
    pub final_report: ReplayFinalReport,
}

impl ReplayArtifact {
    /// Creates a replay artifact from one completed fixed-step run.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        scene: impl Into<String>,
        seed: u64,
        clock: ReplayClock,
        controller: ReplayControllerKind,
        sensor_payload_streams: Vec<u64>,
        frames: Vec<ReplayFrame>,
        final_report: ReplayFinalReport,
    ) -> Self {
        Self {
            version: REPLAY_ARTIFACT_VERSION,
            scene: scene.into(),
            seed,
            clock,
            controller,
            sensor_payload_streams,
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
        validate_sensor_payload_streams(&self.sensor_payload_streams)?;
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
            validate_action(&frame.action, self.controller, frame.step)?;
            validate_joint_state(frame.observation.joint_state.as_ref(), frame.step)?;
            validate_translation(
                frame.observation.base_translation_m,
                &format!("frame {} observation.base_translation_m", frame.step),
            )?;
            validate_sensor_streams(&frame.observation.sensor_streams, frame.step)?;
            validate_sensor_payloads(
                &frame.observation.sensor_payloads,
                &frame.observation.sensor_streams,
                &self.sensor_payload_streams,
                frame.step,
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

fn validate_action(
    action: &ReplayAction,
    controller: ReplayControllerKind,
    step: u64,
) -> Result<(), ReplayArtifactError> {
    let action_kind = action.controller_kind();
    let valid_for_controller = match controller {
        ReplayControllerKind::None => action.is_zero(),
        expected => action_kind == expected,
    };
    if !valid_for_controller {
        return Err(ReplayArtifactError::Invalid(format!(
            "frame {step} action kind {:?} does not match controller {:?}",
            action_kind, controller
        )));
    }

    match action {
        ReplayAction::DifferentialDrive {
            wheel_velocity_rad_s,
        } => {
            if !wheel_velocity_rad_s.is_finite() {
                return Err(ReplayArtifactError::Invalid(format!(
                    "frame {step} wheel_velocity_rad_s must be finite"
                )));
            }
        }
        ReplayAction::JointVelocity {
            joint,
            velocity_rad_s,
        } => {
            if joint.trim().is_empty() {
                return Err(ReplayArtifactError::Invalid(format!(
                    "frame {step} joint action name must not be empty"
                )));
            }
            if !velocity_rad_s.is_finite() {
                return Err(ReplayArtifactError::Invalid(format!(
                    "frame {step} velocity_rad_s must be finite"
                )));
            }
        }
        ReplayAction::JointEffort { joint, effort_nm } => {
            if joint.trim().is_empty() {
                return Err(ReplayArtifactError::Invalid(format!(
                    "frame {step} joint action name must not be empty"
                )));
            }
            if !effort_nm.is_finite() {
                return Err(ReplayArtifactError::Invalid(format!(
                    "frame {step} effort_nm must be finite"
                )));
            }
        }
    }
    Ok(())
}

fn validate_joint_state(
    joint_state: Option<&ReplayJointState>,
    step: u64,
) -> Result<(), ReplayArtifactError> {
    let Some(joint_state) = joint_state else {
        return Ok(());
    };
    if joint_state.names.len() != joint_state.positions_rad.len()
        || joint_state.names.len() != joint_state.velocities_rad_s.len()
    {
        return Err(ReplayArtifactError::Invalid(format!(
            "frame {step} joint state arrays must have equal lengths"
        )));
    }
    let mut names = joint_state
        .names
        .iter()
        .map(String::as_str)
        .collect::<Vec<_>>();
    names.sort_unstable();
    if names.windows(2).any(|window| window[0] == window[1]) {
        return Err(ReplayArtifactError::Invalid(format!(
            "frame {step} joint state names must be unique"
        )));
    }
    if joint_state.names.iter().any(|name| name.trim().is_empty()) {
        return Err(ReplayArtifactError::Invalid(format!(
            "frame {step} joint state names must not be empty"
        )));
    }
    if joint_state
        .positions_rad
        .iter()
        .chain(joint_state.velocities_rad_s.iter())
        .any(|value| !value.is_finite())
    {
        return Err(ReplayArtifactError::Invalid(format!(
            "frame {step} joint state values must be finite"
        )));
    }
    Ok(())
}

fn validate_sensor_streams(
    streams: &[ReplaySensorStream],
    step: u64,
) -> Result<(), ReplayArtifactError> {
    for window in streams.windows(2) {
        if window[0].stream_id >= window[1].stream_id {
            return Err(ReplayArtifactError::Invalid(format!(
                "frame {step} sensor streams must be sorted by unique stream_id"
            )));
        }
    }
    if streams.iter().any(|stream| stream.kind.trim().is_empty()) {
        return Err(ReplayArtifactError::Invalid(format!(
            "frame {step} sensor stream kind must not be empty"
        )));
    }
    Ok(())
}

fn validate_sensor_payload_streams(streams: &[u64]) -> Result<(), ReplayArtifactError> {
    for window in streams.windows(2) {
        if window[0] >= window[1] {
            return Err(ReplayArtifactError::Invalid(
                "sensor_payload_streams must be sorted by unique stream_id".to_string(),
            ));
        }
    }
    Ok(())
}

fn validate_sensor_payloads(
    payloads: &[ReplaySensorPayload],
    streams: &[ReplaySensorStream],
    payload_streams: &[u64],
    step: u64,
) -> Result<(), ReplayArtifactError> {
    for window in payloads.windows(2) {
        if window[0].stream_id >= window[1].stream_id {
            return Err(ReplayArtifactError::Invalid(format!(
                "frame {step} sensor payloads must be sorted by unique stream_id"
            )));
        }
    }
    for payload in payloads {
        if !payload_streams.binary_search(&payload.stream_id).is_ok() {
            return Err(ReplayArtifactError::Invalid(format!(
                "frame {step} sensor payload stream {} is not listed in sensor_payload_streams",
                payload.stream_id
            )));
        }
        if payload.kind.trim().is_empty() {
            return Err(ReplayArtifactError::Invalid(format!(
                "frame {step} sensor payload kind must not be empty"
            )));
        }
        let summary = streams
            .iter()
            .find(|stream| stream.stream_id == payload.stream_id)
            .ok_or_else(|| {
                ReplayArtifactError::Invalid(format!(
                    "frame {step} sensor payload stream {} has no matching stream summary",
                    payload.stream_id
                ))
            })?;
        if summary.kind != payload.kind {
            return Err(ReplayArtifactError::Invalid(format!(
                "frame {step} sensor payload kind {} does not match stream summary kind {}",
                payload.kind, summary.kind
            )));
        }
        match &payload.data {
            ReplaySensorPayloadData::Imu(sample) => {
                validate_vec3(
                    sample.angular_velocity_rad_s,
                    "angular_velocity_rad_s",
                    step,
                )?;
                validate_vec3(
                    sample.linear_acceleration_m_s2,
                    "linear_acceleration_m_s2",
                    step,
                )?;
            }
            ReplaySensorPayloadData::Lidar(cloud) => {
                if !cloud.attributes_are_aligned() {
                    return Err(ReplayArtifactError::Invalid(format!(
                        "frame {step} lidar payload attributes must be aligned"
                    )));
                }
                if cloud.points_m.iter().any(|point| !point.is_finite()) {
                    return Err(ReplayArtifactError::Invalid(format!(
                        "frame {step} lidar payload points must be finite"
                    )));
                }
            }
            ReplaySensorPayloadData::Camera { rgb, depth } => {
                if rgb.width == 0 || rgb.height == 0 {
                    return Err(ReplayArtifactError::Invalid(format!(
                        "frame {step} camera rgb dimensions must be non-zero"
                    )));
                }
                if rgb.rgba8.len() != (rgb.width as usize) * (rgb.height as usize) * 4 {
                    return Err(ReplayArtifactError::Invalid(format!(
                        "frame {step} camera rgb buffer length does not match dimensions"
                    )));
                }
                if depth.width != rgb.width || depth.height != rgb.height {
                    return Err(ReplayArtifactError::Invalid(format!(
                        "frame {step} camera depth dimensions do not match rgb dimensions"
                    )));
                }
                if depth.depth_m.len() != (depth.width as usize) * (depth.height as usize) {
                    return Err(ReplayArtifactError::Invalid(format!(
                        "frame {step} camera depth buffer length does not match dimensions"
                    )));
                }
            }
            ReplaySensorPayloadData::WheelEncoder(sample) => {
                if !sample.position_rad.is_finite() || !sample.velocity_rad_s.is_finite() {
                    return Err(ReplayArtifactError::Invalid(format!(
                        "frame {step} wheel encoder sample must be finite"
                    )));
                }
            }
        }
    }
    Ok(())
}

fn validate_vec3(value: rne_math::Vec3, field: &str, step: u64) -> Result<(), ReplayArtifactError> {
    if !value.is_finite() {
        return Err(ReplayArtifactError::Invalid(format!(
            "frame {step} imu {field} must be finite"
        )));
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
            Vec::new(),
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

        assert!(error.to_string().contains("does not match controller"));
    }

    #[test]
    fn joint_action_uses_tagged_json_and_legacy_wheel_json_is_readable() {
        let action = ReplayAction::joint_velocity("left_hip_pitch_joint", 0.25);
        let tagged = serde_json::to_string(&action).unwrap();
        assert!(tagged.contains(r#""kind":"joint_velocity""#));
        assert_eq!(
            serde_json::from_str::<ReplayAction>(&tagged).unwrap(),
            action
        );

        let legacy: ReplayAction = serde_json::from_str(r#"{"wheel_velocity_rad_s":6.0}"#).unwrap();
        assert_eq!(legacy, ReplayAction::differential_drive(6.0));
    }

    #[test]
    fn sensor_payloads_roundtrip_and_validate() {
        let mut artifact = sample_artifact();
        artifact.sensor_payload_streams = vec![7];
        artifact.frames[0].observation.sensor_streams = vec![ReplaySensorStream {
            stream_id: 7,
            kind: "imu".to_string(),
            frame_count: 1,
            last_sequence: 1,
            payload_hash: 0xabc,
        }];
        artifact.frames[0].observation.sensor_payloads = vec![ReplaySensorPayload {
            stream_id: 7,
            kind: "imu".to_string(),
            sequence: 1,
            data: ReplaySensorPayloadData::Imu(rne_data::ImuSample::default()),
        }];

        artifact.validate().expect("payloads are valid");
        let json = artifact.to_json().unwrap();
        let loaded = ReplayArtifact::from_json(&json).unwrap();
        assert_eq!(loaded, artifact);
        assert!(json.contains(r#""kind": "imu""#));
    }

    #[test]
    fn legacy_artifact_without_payloads_still_parses() {
        let legacy = r#"{
            "version": 1,
            "scene": "assets/scenes/example.rne.scene.toml",
            "seed": 42,
            "clock": {"steps": 1, "hz": 60.0},
            "controller": "differential_drive",
            "frames": [{
                "step": 0,
                "sim_ticks": 16666666,
                "action": {"wheel_velocity_rad_s": 6.0},
                "observation": {"base_translation_m": [0.1, 0.0, 0.0]},
                "physics_hash": 17
            }],
            "final_report": {
                "steps": 1,
                "sim_time_s": 0.016666666,
                "seed": 42,
                "robot_count": 1,
                "differential_drive_count": 1,
                "physics_hash": 17,
                "first_base_translation_m": [0.1, 0.0, 0.0]
            }
        }"#;
        let artifact = ReplayArtifact::from_json(legacy).expect("legacy artifact parses");
        assert!(artifact.sensor_payload_streams.is_empty());
        assert!(artifact.frames[0].observation.sensor_payloads.is_empty());
    }

    #[test]
    fn sensor_payload_must_match_a_stream_summary() {
        let mut artifact = sample_artifact();
        artifact.sensor_payload_streams = vec![9];
        artifact.frames[0].observation.sensor_payloads = vec![ReplaySensorPayload {
            stream_id: 9,
            kind: "imu".to_string(),
            sequence: 1,
            data: ReplaySensorPayloadData::Imu(rne_data::ImuSample::default()),
        }];

        let error = artifact.validate().unwrap_err();
        assert!(error.to_string().contains("no matching stream summary"));
    }

    #[test]
    fn sensor_payload_must_be_listed_in_payload_streams() {
        let mut artifact = sample_artifact();
        artifact.sensor_payload_streams = Vec::new();
        artifact.frames[0].observation.sensor_streams = vec![ReplaySensorStream {
            stream_id: 9,
            kind: "imu".to_string(),
            frame_count: 1,
            last_sequence: 1,
            payload_hash: 0,
        }];
        artifact.frames[0].observation.sensor_payloads = vec![ReplaySensorPayload {
            stream_id: 9,
            kind: "imu".to_string(),
            sequence: 1,
            data: ReplaySensorPayloadData::Imu(rne_data::ImuSample::default()),
        }];

        let error = artifact.validate().unwrap_err();
        assert!(error
            .to_string()
            .contains("not listed in sensor_payload_streams"));
    }

    #[test]
    fn camera_payload_dimensions_must_match() {
        let mut artifact = sample_artifact();
        artifact.sensor_payload_streams = vec![7];
        artifact.frames[0].observation.sensor_streams = vec![ReplaySensorStream {
            stream_id: 7,
            kind: "camera".to_string(),
            frame_count: 1,
            last_sequence: 1,
            payload_hash: 0,
        }];
        artifact.frames[0].observation.sensor_payloads = vec![ReplaySensorPayload {
            stream_id: 7,
            kind: "camera".to_string(),
            sequence: 1,
            data: ReplaySensorPayloadData::Camera {
                rgb: rne_data::ImageRgb8::from_rgba8(2, 2, vec![0; 16]),
                depth: rne_data::ImageDepth::new(3, 2, vec![0.0; 6]),
            },
        }];

        let error = artifact.validate().unwrap_err();
        assert!(error.to_string().contains("depth dimensions do not match"));
    }
}
