//! LeKiwi + SO-101 reference-hardware contract for Robot Native Engine.
//!
//! This brand-specific crate remains outside RNE core. It pins the upstream
//! LeRobot interface used by the reference device, defines a conservative
//! base-only TaskSpec, maps vendor units into that contract, and preserves
//! fail-closed base-stop behavior. The SO-101 arm is observed and held at its
//! latest measured position; v1 deliberately does not grant it live actuation.

#![deny(missing_docs)]

pub mod flagship_projection;
pub mod flagship_rate;
pub mod physical_evidence;
pub mod session;

use rne_ai::{
    ActionSpec, ObservationSpec, ResetSpec, RewardSpec, RewardTermSpec, TaskSpec, TensorBounds,
    TensorDType, TensorSpec, TerminationConditionSpec, TerminationKind, TerminationSpec,
};
use rne_hardware_gateway::wire::HARDWARE_WIRE_SCHEMA_VERSION;
use rne_hardware_gateway::{ActuationFrame, HardwareMode, SafetyReason};
use serde::{Deserialize, Serialize};
use std::f64::consts::PI;

/// Schema version for the built-in LeKiwi reference profile.
pub const LEKIWI_REFERENCE_PROFILE_SCHEMA_VERSION: u32 = 1;

/// Schema version implemented by the companion Python device bridge.
pub const LEKIWI_DEVICE_BRIDGE_SCHEMA_VERSION: u32 = 1;

/// Device identity emitted by the dependency-free companion mock bridge.
pub const LEKIWI_MOCK_DEVICE_ID: &str = "rne.lekiwi_so101.mock.v1";

/// Required prefix for a companion bridge connected to physical hardware.
pub const LEKIWI_PHYSICAL_DEVICE_ID_PREFIX: &str = "rne.lekiwi_so101.physical.v1:";

/// Stable discriminator for LeKiwiReferenceProfile.
pub const LEKIWI_REFERENCE_PROFILE_KIND: &str = "rne_hardware_reference_profile";

/// Stable identity of the conservative LeKiwi + SO-101 reference profile.
pub const LEKIWI_REFERENCE_PROFILE_ID: &str = "rne.lekiwi_so101.base.v1";

/// Task identity shared by simulation, shadow, HIL, and live base control.
pub const LEKIWI_BASE_TASK_ID: &str = "rne.lekiwi_so101.base_shadow.v1";

/// Upstream LeRobot repository used by this profile.
pub const LEKIWI_UPSTREAM_REPOSITORY: &str = "https://github.com/huggingface/lerobot";

/// Upstream LeRobot release used by this profile.
pub const LEKIWI_UPSTREAM_VERSION: &str = "v0.6.0";

/// Content-addressed upstream revision for the selected LeRobot release.
pub const LEKIWI_UPSTREAM_REVISION: &str = "30da8e687a6dfc617fcd94afc367ac7071c376ce";

/// Upstream device watchdog interval.
pub const LEKIWI_UPSTREAM_WATCHDOG_TIMEOUT_MS: u64 = 500;

/// Conservative linear speed used for initial physical validation.
pub const LEKIWI_MAX_LINEAR_SPEED_M_S: f64 = 0.1;

/// Conservative angular speed used for initial physical validation.
pub const LEKIWI_MAX_ANGULAR_SPEED_RAD_S: f64 = PI / 6.0;

const RAD_PER_DEG: f64 = PI / 180.0;
const DEG_PER_RAD: f64 = 180.0 / PI;

const ARM_KEYS: [&str; 6] = [
    "arm_shoulder_pan.pos",
    "arm_shoulder_lift.pos",
    "arm_elbow_flex.pos",
    "arm_wrist_flex.pos",
    "arm_wrist_roll.pos",
    "arm_gripper.pos",
];

/// One pinned upstream implementation dependency.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct UpstreamReference {
    /// Human-readable upstream project name.
    pub project: String,
    /// Canonical source repository.
    pub repository: String,
    /// Selected release tag.
    pub version: String,
    /// Full content-addressed source revision.
    pub revision: String,
}

/// Mapping from one upstream scalar to a TaskSpec observation element.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ObservationChannelBinding {
    /// LeRobot observation dictionary key.
    pub vendor_key: String,
    /// Unit produced by the pinned upstream interface.
    pub vendor_unit: String,
    /// TaskSpec observation tensor name.
    pub tensor_name: String,
    /// Row-major element within the TaskSpec tensor.
    pub tensor_element: usize,
    /// Unit declared by the TaskSpec tensor.
    pub task_unit: String,
    /// Multiplier in task = vendor * scale.
    pub vendor_to_task_scale: f64,
}

/// Mapping from one TaskSpec action element to an upstream scalar.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ActionChannelBinding {
    /// TaskSpec action tensor name.
    pub tensor_name: String,
    /// Row-major element within the TaskSpec tensor.
    pub tensor_element: usize,
    /// Unit declared by the TaskSpec tensor.
    pub task_unit: String,
    /// LeRobot action dictionary key.
    pub vendor_key: String,
    /// Unit accepted by the pinned upstream interface.
    pub vendor_unit: String,
    /// Multiplier in vendor = task * scale.
    pub task_to_vendor_scale: f64,
}

/// One camera configured by the pinned LeRobot reference implementation.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ReferenceCameraStream {
    /// Stable RNE stream name.
    pub stream_name: String,
    /// LeRobot observation dictionary key.
    pub vendor_key: String,
    /// Configured sensor width before any declared rotation.
    pub configured_width_px: u32,
    /// Configured sensor height before any declared rotation.
    pub configured_height_px: u32,
    /// Clockwise presentation rotation configured upstream.
    pub rotation_deg: u16,
    /// Camera payloads travel through the dataset path, not the numeric wire.
    pub out_of_band_dataset_stream: bool,
}

/// Safety limits required by the reference profile.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct LeKiwiSafetyContract {
    /// RNE gateway maximum accepted observation age.
    pub max_observation_age_ms: u64,
    /// Maximum observation-to-command delay.
    pub command_deadline_ms: u64,
    /// Maximum age of a queued or active command.
    pub max_command_age_ms: u64,
    /// Independent device-process watchdog interval.
    pub device_watchdog_timeout_ms: u64,
    /// Inclusive planar linear speed limit per axis.
    pub max_linear_speed_m_s: f64,
    /// Inclusive yaw-rate limit.
    pub max_angular_speed_rad_s: f64,
    /// Whether disconnect disables servo torque after stopping the base.
    pub disconnect_disables_torque: bool,
    /// Whether the v1 profile grants controller authority over the arm.
    pub arm_actuation_enabled: bool,
    /// Whether the physical validation setup must provide power isolation.
    pub physical_emergency_stop_required: bool,
}

/// Versioned selection and mapping contract for the reference robot.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct LeKiwiReferenceProfile {
    /// Stable artifact discriminator.
    pub kind: String,
    /// Profile schema version.
    pub schema_version: u32,
    /// Stable profile identity.
    pub profile_id: String,
    /// RNE hardware wire version used by the bridge.
    pub wire_schema_version: u32,
    /// Selected upstream implementation pin.
    pub upstream: UpstreamReference,
    /// Authority modes supported by the physical adapter.
    pub supported_modes: Vec<HardwareMode>,
    /// Complete portable task contract.
    pub task: TaskSpec,
    /// Vendor-to-TaskSpec observation mapping in flattened TaskSpec order.
    pub observation_bindings: Vec<ObservationChannelBinding>,
    /// TaskSpec-to-vendor action mapping in flattened TaskSpec order.
    pub action_bindings: Vec<ActionChannelBinding>,
    /// Camera streams intentionally kept outside the numeric process wire.
    pub camera_streams: Vec<ReferenceCameraStream>,
    /// Fail-closed timing and authority contract.
    pub safety: LeKiwiSafetyContract,
}

impl LeKiwiReferenceProfile {
    /// Validates an untrusted profile against the exact built-in v1 contract.
    ///
    /// This is intentionally strict: a changed upstream pin, unit conversion,
    /// action limit, or arm-authority decision is a new reference profile.
    pub fn validate(&self) -> Result<(), LeKiwiProfileError> {
        self.task
            .validate()
            .map_err(|error| LeKiwiProfileError::Task(error.to_string()))?;
        let expected = lekiwi_reference_profile_v1();
        if self.kind != expected.kind {
            return Err(LeKiwiProfileError::Mismatch("kind"));
        }
        if self.schema_version != expected.schema_version {
            return Err(LeKiwiProfileError::Mismatch("schema_version"));
        }
        if self.profile_id != expected.profile_id {
            return Err(LeKiwiProfileError::Mismatch("profile_id"));
        }
        if self.wire_schema_version != expected.wire_schema_version {
            return Err(LeKiwiProfileError::Mismatch("wire_schema_version"));
        }
        if self.upstream != expected.upstream {
            return Err(LeKiwiProfileError::Mismatch("upstream"));
        }
        if self.supported_modes != expected.supported_modes {
            return Err(LeKiwiProfileError::Mismatch("supported_modes"));
        }
        if self.task != expected.task {
            return Err(LeKiwiProfileError::Mismatch("task"));
        }
        if self.observation_bindings != expected.observation_bindings {
            return Err(LeKiwiProfileError::Mismatch("observation_bindings"));
        }
        if self.action_bindings != expected.action_bindings {
            return Err(LeKiwiProfileError::Mismatch("action_bindings"));
        }
        if self.camera_streams != expected.camera_streams {
            return Err(LeKiwiProfileError::Mismatch("camera_streams"));
        }
        if self.safety != expected.safety {
            return Err(LeKiwiProfileError::Mismatch("safety"));
        }
        Ok(())
    }
}

/// Failure validating a selected reference profile.
#[derive(Clone, Debug, PartialEq, Eq, thiserror::Error)]
pub enum LeKiwiProfileError {
    /// Embedded TaskSpec validation failed.
    #[error("invalid reference TaskSpec: {0}")]
    Task(String),
    /// A field differs from the exact versioned built-in contract.
    #[error("reference profile field {0} does not match LeKiwi v1")]
    Mismatch(&'static str),
}

/// Raw numeric state returned by LeRobot v0.6.0 with use_degrees enabled.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LeKiwiVendorObservation {
    /// Five revolute arm joint positions in upstream degrees.
    pub arm_joint_position_deg: [f64; 5],
    /// Gripper position in the upstream normalized 0..100 range.
    pub gripper_position_pct: f64,
    /// Body-frame planar velocity in meters per second.
    pub base_linear_velocity_m_s: [f64; 2],
    /// Body-frame yaw rate in upstream degrees per second.
    pub base_angular_velocity_deg_s: f64,
}

impl LeKiwiVendorObservation {
    fn validate(self) -> Result<(), LeKiwiAdapterError> {
        for (index, value) in self
            .arm_joint_position_deg
            .into_iter()
            .chain([self.gripper_position_pct])
            .chain(self.base_linear_velocity_m_s)
            .chain([self.base_angular_velocity_deg_s])
            .enumerate()
        {
            if !value.is_finite() {
                return Err(LeKiwiAdapterError::NonFiniteObservation { index });
            }
        }
        if !(0.0..=100.0).contains(&self.gripper_position_pct) {
            return Err(LeKiwiAdapterError::GripperRange {
                value: self.gripper_position_pct,
            });
        }
        Ok(())
    }
}

/// Device-side command derived from an RNE actuation frame.
#[derive(Clone, Debug, PartialEq)]
pub enum LeKiwiDeviceCommand {
    /// Hold the last measured arm pose while applying a bounded base velocity.
    HoldArmAndDrive {
        /// Six upstream arm/gripper positions, in degrees then normalized percent.
        arm_position_vendor: [f64; 6],
        /// Body-frame x/y command in meters per second.
        base_linear_velocity_m_s: [f64; 2],
        /// Body-frame yaw command in upstream degrees per second.
        base_angular_velocity_deg_s: f64,
    },
    /// Call the device's independent base-stop operation.
    StopBase {
        /// Gateway reason carried by the fail-closed frame.
        reason: SafetyReason,
    },
}

/// Stateful defense-in-depth mapper used by a LeKiwi device process.
#[derive(Clone, Debug, Default)]
pub struct LeKiwiAdapter {
    last_arm_position_vendor: Option<[f64; 6]>,
}

impl LeKiwiAdapter {
    /// Creates an adapter without a cached physical observation.
    pub fn new() -> Self {
        Self::default()
    }

    /// Converts a vendor observation into flattened TaskSpec order.
    ///
    /// A successful conversion also captures the six arm/gripper values used
    /// to hold the physical arm during later base commands.
    pub fn ingest_observation(
        &mut self,
        observation: LeKiwiVendorObservation,
    ) -> Result<Vec<f64>, LeKiwiAdapterError> {
        observation.validate()?;
        let mut arm_position_vendor = [0.0; 6];
        arm_position_vendor[..5].copy_from_slice(&observation.arm_joint_position_deg);
        arm_position_vendor[5] = observation.gripper_position_pct;
        self.last_arm_position_vendor = Some(arm_position_vendor);

        Ok(observation
            .arm_joint_position_deg
            .into_iter()
            .map(|value| value * RAD_PER_DEG)
            .chain([observation.gripper_position_pct])
            .chain(observation.base_linear_velocity_m_s)
            .chain([observation.base_angular_velocity_deg_s * RAD_PER_DEG])
            .collect())
    }

    /// Converts one gateway-validated action into a vendor-side command.
    ///
    /// Safety frames are checked again and become a direct stop_base request.
    /// A normal base action is rejected until a fresh arm pose has been observed.
    pub fn command(
        &self,
        frame: &ActuationFrame,
    ) -> Result<LeKiwiDeviceCommand, LeKiwiAdapterError> {
        if frame.safety_stop {
            if frame.action_sequence.is_some()
                || frame.reason.is_none()
                || frame.values.len() != 3
                || frame.values.iter().any(|value| *value != 0.0)
            {
                return Err(LeKiwiAdapterError::InvalidSafetyStop);
            }
            return Ok(LeKiwiDeviceCommand::StopBase {
                reason: frame.reason.expect("checked above"),
            });
        }
        if frame.action_sequence.is_none() || frame.reason.is_some() {
            return Err(LeKiwiAdapterError::InvalidActuationEnvelope);
        }
        if frame.values.len() != 3 {
            return Err(LeKiwiAdapterError::ActionWidth {
                actual: frame.values.len(),
            });
        }
        for (index, value) in frame.values.iter().enumerate() {
            if !value.is_finite() {
                return Err(LeKiwiAdapterError::NonFiniteAction { index });
            }
        }
        for (index, value) in frame.values[..2].iter().copied().enumerate() {
            if value.abs() > LEKIWI_MAX_LINEAR_SPEED_M_S {
                return Err(LeKiwiAdapterError::LinearSpeedLimit { index, value });
            }
        }
        if frame.values[2].abs() > LEKIWI_MAX_ANGULAR_SPEED_RAD_S {
            return Err(LeKiwiAdapterError::AngularSpeedLimit {
                value: frame.values[2],
            });
        }
        let arm_position_vendor = self
            .last_arm_position_vendor
            .ok_or(LeKiwiAdapterError::NoArmHoldObservation)?;
        Ok(LeKiwiDeviceCommand::HoldArmAndDrive {
            arm_position_vendor,
            base_linear_velocity_m_s: [frame.values[0], frame.values[1]],
            base_angular_velocity_deg_s: frame.values[2] * DEG_PER_RAD,
        })
    }
}

/// Failure mapping a vendor observation or an RNE actuation.
#[derive(Clone, Debug, PartialEq, thiserror::Error)]
pub enum LeKiwiAdapterError {
    /// One vendor observation value was NaN or infinite.
    #[error("LeKiwi observation value {index} must be finite")]
    NonFiniteObservation {
        /// Flattened vendor observation index.
        index: usize,
    },
    /// The upstream normalized gripper value left its declared range.
    #[error("LeKiwi gripper position {value} is outside [0, 100]")]
    GripperRange {
        /// Rejected value.
        value: f64,
    },
    /// A safety frame did not request an exact zero base stop.
    #[error("invalid LeKiwi safety-stop frame")]
    InvalidSafetyStop,
    /// A normal frame had inconsistent action sequence or stop-reason fields.
    #[error("invalid LeKiwi actuation envelope")]
    InvalidActuationEnvelope,
    /// The action width differed from the base-only TaskSpec.
    #[error("LeKiwi base action must contain 3 values, got {actual}")]
    ActionWidth {
        /// Rejected width.
        actual: usize,
    },
    /// One action value was NaN or infinite.
    #[error("LeKiwi action value {index} must be finite")]
    NonFiniteAction {
        /// Flattened action index.
        index: usize,
    },
    /// One planar command exceeded the conservative reference limit.
    #[error("LeKiwi linear action {index}={value} exceeds the reference limit")]
    LinearSpeedLimit {
        /// Planar x/y element.
        index: usize,
        /// Rejected value.
        value: f64,
    },
    /// One yaw command exceeded the conservative reference limit.
    #[error("LeKiwi angular action {value} exceeds the reference limit")]
    AngularSpeedLimit {
        /// Rejected radians-per-second value.
        value: f64,
    },
    /// Base control was requested before an arm hold position was observed.
    #[error("LeKiwi base actuation requires a prior arm observation")]
    NoArmHoldObservation,
}

/// Returns the portable base-only TaskSpec used by the reference profile.
pub fn lekiwi_base_task_spec() -> TaskSpec {
    TaskSpec::new(
        LEKIWI_BASE_TASK_ID,
        1.0 / 30.0,
        ObservationSpec::new(vec![
            TensorSpec::new("arm_joint_position_rad", TensorDType::F64, vec![5], "rad"),
            TensorSpec::new("gripper_position_pct", TensorDType::F64, vec![], "pct")
                .with_bounds(TensorBounds::broadcast(0.0, 100.0)),
            TensorSpec::new("base_linear_velocity_m_s", TensorDType::F64, vec![2], "m/s"),
            TensorSpec::new(
                "base_angular_velocity_rad_s",
                TensorDType::F64,
                vec![],
                "rad/s",
            ),
        ]),
        ActionSpec::new(vec![
            TensorSpec::new("base_linear_velocity_m_s", TensorDType::F64, vec![2], "m/s")
                .with_bounds(TensorBounds::broadcast(
                    -LEKIWI_MAX_LINEAR_SPEED_M_S,
                    LEKIWI_MAX_LINEAR_SPEED_M_S,
                )),
            TensorSpec::new(
                "base_angular_velocity_rad_s",
                TensorDType::F64,
                vec![],
                "rad/s",
            )
            .with_bounds(TensorBounds::broadcast(
                -LEKIWI_MAX_ANGULAR_SPEED_RAD_S,
                LEKIWI_MAX_ANGULAR_SPEED_RAD_S,
            )),
        ]),
        RewardSpec::weighted_sum(vec![RewardTermSpec::new(
            "shadow_tracking_error",
            -1.0,
            "1",
        )]),
        TerminationSpec::new(
            vec![
                TerminationConditionSpec::new("operator_complete", TerminationKind::Success),
                TerminationConditionSpec::new("safety_trip", TerminationKind::Failure),
            ],
            Some(1_800),
        ),
        ResetSpec::splitmix64(false),
    )
}

/// Returns the exact LeKiwi + SO-101 v1 reference profile.
pub fn lekiwi_reference_profile_v1() -> LeKiwiReferenceProfile {
    let mut observation_bindings = Vec::with_capacity(9);
    for (tensor_element, vendor_key) in ARM_KEYS[..5].iter().enumerate() {
        observation_bindings.push(ObservationChannelBinding {
            vendor_key: (*vendor_key).to_string(),
            vendor_unit: "deg".to_string(),
            tensor_name: "arm_joint_position_rad".to_string(),
            tensor_element,
            task_unit: "rad".to_string(),
            vendor_to_task_scale: RAD_PER_DEG,
        });
    }
    observation_bindings.extend([
        ObservationChannelBinding {
            vendor_key: ARM_KEYS[5].to_string(),
            vendor_unit: "pct".to_string(),
            tensor_name: "gripper_position_pct".to_string(),
            tensor_element: 0,
            task_unit: "pct".to_string(),
            vendor_to_task_scale: 1.0,
        },
        ObservationChannelBinding {
            vendor_key: "x.vel".to_string(),
            vendor_unit: "m/s".to_string(),
            tensor_name: "base_linear_velocity_m_s".to_string(),
            tensor_element: 0,
            task_unit: "m/s".to_string(),
            vendor_to_task_scale: 1.0,
        },
        ObservationChannelBinding {
            vendor_key: "y.vel".to_string(),
            vendor_unit: "m/s".to_string(),
            tensor_name: "base_linear_velocity_m_s".to_string(),
            tensor_element: 1,
            task_unit: "m/s".to_string(),
            vendor_to_task_scale: 1.0,
        },
        ObservationChannelBinding {
            vendor_key: "theta.vel".to_string(),
            vendor_unit: "deg/s".to_string(),
            tensor_name: "base_angular_velocity_rad_s".to_string(),
            tensor_element: 0,
            task_unit: "rad/s".to_string(),
            vendor_to_task_scale: RAD_PER_DEG,
        },
    ]);

    LeKiwiReferenceProfile {
        kind: LEKIWI_REFERENCE_PROFILE_KIND.to_string(),
        schema_version: LEKIWI_REFERENCE_PROFILE_SCHEMA_VERSION,
        profile_id: LEKIWI_REFERENCE_PROFILE_ID.to_string(),
        wire_schema_version: HARDWARE_WIRE_SCHEMA_VERSION,
        upstream: UpstreamReference {
            project: "Hugging Face LeRobot LeKiwi".to_string(),
            repository: LEKIWI_UPSTREAM_REPOSITORY.to_string(),
            version: LEKIWI_UPSTREAM_VERSION.to_string(),
            revision: LEKIWI_UPSTREAM_REVISION.to_string(),
        },
        supported_modes: vec![HardwareMode::Shadow, HardwareMode::Hil, HardwareMode::Live],
        task: lekiwi_base_task_spec(),
        observation_bindings,
        action_bindings: vec![
            ActionChannelBinding {
                tensor_name: "base_linear_velocity_m_s".to_string(),
                tensor_element: 0,
                task_unit: "m/s".to_string(),
                vendor_key: "x.vel".to_string(),
                vendor_unit: "m/s".to_string(),
                task_to_vendor_scale: 1.0,
            },
            ActionChannelBinding {
                tensor_name: "base_linear_velocity_m_s".to_string(),
                tensor_element: 1,
                task_unit: "m/s".to_string(),
                vendor_key: "y.vel".to_string(),
                vendor_unit: "m/s".to_string(),
                task_to_vendor_scale: 1.0,
            },
            ActionChannelBinding {
                tensor_name: "base_angular_velocity_rad_s".to_string(),
                tensor_element: 0,
                task_unit: "rad/s".to_string(),
                vendor_key: "theta.vel".to_string(),
                vendor_unit: "deg/s".to_string(),
                task_to_vendor_scale: DEG_PER_RAD,
            },
        ],
        camera_streams: vec![
            ReferenceCameraStream {
                stream_name: "lekiwi.front.color".to_string(),
                vendor_key: "front".to_string(),
                configured_width_px: 640,
                configured_height_px: 480,
                rotation_deg: 180,
                out_of_band_dataset_stream: true,
            },
            ReferenceCameraStream {
                stream_name: "lekiwi.wrist.color".to_string(),
                vendor_key: "wrist".to_string(),
                configured_width_px: 480,
                configured_height_px: 640,
                rotation_deg: 90,
                out_of_band_dataset_stream: true,
            },
        ],
        safety: LeKiwiSafetyContract {
            max_observation_age_ms: 100,
            command_deadline_ms: 75,
            max_command_age_ms: 100,
            device_watchdog_timeout_ms: LEKIWI_UPSTREAM_WATCHDOG_TIMEOUT_MS,
            max_linear_speed_m_s: LEKIWI_MAX_LINEAR_SPEED_M_S,
            max_angular_speed_rad_s: LEKIWI_MAX_ANGULAR_SPEED_RAD_S,
            disconnect_disables_torque: true,
            arm_actuation_enabled: false,
            physical_emergency_stop_required: true,
        },
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn action(values: Vec<f64>) -> ActuationFrame {
        ActuationFrame {
            action_sequence: Some(1),
            queued_at_ms: 10,
            values,
            safety_stop: false,
            reason: None,
        }
    }

    #[test]
    fn built_in_profile_is_strict_and_valid() {
        let profile = lekiwi_reference_profile_v1();
        profile.validate().unwrap();
        assert_eq!(profile.task.action.tensors.len(), 2);
        assert_eq!(profile.observation_bindings.len(), 9);
        assert!(!profile.safety.arm_actuation_enabled);

        let mut altered = profile;
        altered.safety.device_watchdog_timeout_ms += 1;
        assert_eq!(
            altered.validate().unwrap_err(),
            LeKiwiProfileError::Mismatch("safety")
        );
    }

    #[test]
    fn observation_mapping_converts_degrees_and_caches_arm_hold() {
        let mut adapter = LeKiwiAdapter::new();
        let values = adapter
            .ingest_observation(LeKiwiVendorObservation {
                arm_joint_position_deg: [180.0, -90.0, 45.0, 0.0, 360.0],
                gripper_position_pct: 25.0,
                base_linear_velocity_m_s: [0.05, -0.025],
                base_angular_velocity_deg_s: 30.0,
            })
            .unwrap();
        assert_eq!(values.len(), 9);
        assert!((values[0] - PI).abs() < 1.0e-12);
        assert!((values[1] + PI / 2.0).abs() < 1.0e-12);
        assert_eq!(&values[5..8], &[25.0, 0.05, -0.025]);
        assert!((values[8] - PI / 6.0).abs() < 1.0e-12);

        let command = adapter.command(&action(vec![0.1, -0.1, PI / 6.0])).unwrap();
        let LeKiwiDeviceCommand::HoldArmAndDrive {
            arm_position_vendor,
            base_linear_velocity_m_s,
            base_angular_velocity_deg_s,
        } = command
        else {
            panic!("expected drive command");
        };
        assert_eq!(arm_position_vendor, [180.0, -90.0, 45.0, 0.0, 360.0, 25.0]);
        assert_eq!(base_linear_velocity_m_s, [0.1, -0.1]);
        assert!((base_angular_velocity_deg_s - 30.0).abs() < 1.0e-12);
    }

    #[test]
    fn base_action_requires_observation_and_rechecks_limits() {
        let adapter = LeKiwiAdapter::new();
        assert_eq!(
            adapter.command(&action(vec![0.0, 0.0, 0.0])).unwrap_err(),
            LeKiwiAdapterError::NoArmHoldObservation
        );

        let mut adapter = adapter;
        adapter
            .ingest_observation(LeKiwiVendorObservation {
                arm_joint_position_deg: [0.0; 5],
                gripper_position_pct: 0.0,
                base_linear_velocity_m_s: [0.0; 2],
                base_angular_velocity_deg_s: 0.0,
            })
            .unwrap();
        assert!(matches!(
            adapter.command(&action(vec![0.100_001, 0.0, 0.0])),
            Err(LeKiwiAdapterError::LinearSpeedLimit { .. })
        ));
        assert!(matches!(
            adapter.command(&action(vec![0.0, 0.0, PI / 6.0 + 1.0e-6])),
            Err(LeKiwiAdapterError::AngularSpeedLimit { .. })
        ));
    }

    #[test]
    fn safety_stop_never_becomes_a_position_command() {
        let adapter = LeKiwiAdapter::new();
        let frame = ActuationFrame {
            action_sequence: None,
            queued_at_ms: 42,
            values: vec![0.0; 3],
            safety_stop: true,
            reason: Some(SafetyReason::Disconnected),
        };
        assert_eq!(
            adapter.command(&frame).unwrap(),
            LeKiwiDeviceCommand::StopBase {
                reason: SafetyReason::Disconnected
            }
        );

        let mut unsafe_frame = frame;
        unsafe_frame.values[0] = 0.01;
        assert_eq!(
            adapter.command(&unsafe_frame).unwrap_err(),
            LeKiwiAdapterError::InvalidSafetyStop
        );
    }

    #[test]
    fn invalid_vendor_values_fail_before_state_is_cached() {
        let mut adapter = LeKiwiAdapter::new();
        assert!(matches!(
            adapter.ingest_observation(LeKiwiVendorObservation {
                arm_joint_position_deg: [f64::NAN, 0.0, 0.0, 0.0, 0.0],
                gripper_position_pct: 0.0,
                base_linear_velocity_m_s: [0.0; 2],
                base_angular_velocity_deg_s: 0.0,
            }),
            Err(LeKiwiAdapterError::NonFiniteObservation { index: 0 })
        ));
        assert_eq!(
            adapter.command(&action(vec![0.0; 3])).unwrap_err(),
            LeKiwiAdapterError::NoArmHoldObservation
        );
    }
}
