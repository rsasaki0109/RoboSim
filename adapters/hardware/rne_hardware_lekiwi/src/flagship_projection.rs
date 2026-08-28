//! Fail-closed action projection from the release flagship to LeKiwi.
//!
//! This first physical-path slice deliberately implements only the action
//! boundary. Observation fusion and 60-to-30 Hz scheduling remain separate
//! contracts, so a passing projection is not a claim that live flagship
//! execution is ready.

use crate::{lekiwi_base_task_spec, LEKIWI_BASE_TASK_ID, LEKIWI_REFERENCE_PROFILE_ID};
use rne_ai::{
    flagship_mobile_lift_task_spec, flagship_mobile_lift_task_spec_v2,
    mm_mobile_wheel_velocities_to_twist, TaskSpec, TensorBounds, TensorSpec,
    FLAGSHIP_MOBILE_LIFT_CONTROLLER_ID, FLAGSHIP_MOBILE_LIFT_CONTROLLER_ID_V2,
    FLAGSHIP_MOBILE_LIFT_CONTROL_PERIOD_TICKS, FLAGSHIP_MOBILE_LIFT_TASK_ID,
    FLAGSHIP_MOBILE_LIFT_TASK_ID_V2,
};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};

/// Schema version for the flagship-to-LeKiwi action projection evidence.
pub const FLAGSHIP_LEKIWI_ACTION_PROJECTION_SCHEMA_VERSION: u32 = 1;

/// Current projection schema version for the portable v2 parent contract.
pub const FLAGSHIP_LEKIWI_ACTION_PROJECTION_CURRENT_SCHEMA_VERSION: u32 = 2;

/// Stable discriminator for [`FlagshipLeKiwiActionProjection`].
pub const FLAGSHIP_LEKIWI_ACTION_PROJECTION_KIND: &str = "rne_flagship_lekiwi_action_projection";

/// Exact diff-drive geometry and coordinate convention used by the projection.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct FlagshipLeKiwiActionTransform {
    /// Parent wheel radius in metres.
    pub parent_wheel_radius_m: f64,
    /// Parent left-to-right wheel track in metres.
    pub parent_track_width_m: f64,
    /// Parent wheel tensor unit.
    pub parent_wheel_unit: String,
    /// Physical planar linear tensor unit.
    pub physical_linear_unit: String,
    /// Physical yaw tensor unit.
    pub physical_angular_unit: String,
    /// Body-axis convention retained by the evidence.
    pub body_axis_convention: String,
    /// Exact mathematical transform identity.
    pub transform_id: String,
    /// Policy at the physical safety envelope; v1 always fails closed.
    pub envelope_policy: String,
}

/// One parent action element deliberately denied physical authority.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SuppressedFlagshipAction {
    /// Parent action tensor name.
    pub tensor_name: String,
    /// Row-major element within the parent tensor.
    pub tensor_element: usize,
    /// Unit declared by the parent TaskSpec.
    pub unit: String,
    /// Exact controller value suppressed at this boundary.
    pub value: f64,
}

/// Content-addressed result of one bounded parent-controller action projection.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct FlagshipLeKiwiActionProjection {
    /// Stable artifact discriminator.
    pub kind: String,
    /// Projection schema version.
    pub schema_version: u32,
    /// Parent release TaskSpec identity.
    pub parent_task_id: String,
    /// Parent release controller identity.
    pub parent_controller_id: String,
    /// Physical adapter TaskSpec identity.
    pub physical_task_id: String,
    /// Exact physical reference profile identity.
    pub physical_profile_id: String,
    /// SHA-256 of the length-prefixed little-endian parent action values.
    pub parent_action_sha256: String,
    /// Exact transform configuration.
    pub transform: FlagshipLeKiwiActionTransform,
    /// LeKiwi action in TaskSpec order: body x, body y, and yaw rate.
    pub physical_action_values: [f64; 3],
    /// Parent elements intentionally denied physical authority.
    pub suppressed_actions: Vec<SuppressedFlagshipAction>,
    /// Stable success verdict for this action boundary only.
    pub status: String,
}

/// Projects one complete flagship controller action into the bounded LeKiwi base action.
pub fn project_flagship_action_to_lekiwi(
    parent_action: &[f64],
) -> Result<FlagshipLeKiwiActionProjection, FlagshipLeKiwiProjectionError> {
    let parent_task = flagship_mobile_lift_task_spec(FLAGSHIP_MOBILE_LIFT_CONTROL_PERIOD_TICKS);
    project_action(
        parent_action,
        &parent_task,
        FLAGSHIP_MOBILE_LIFT_TASK_ID,
        FLAGSHIP_MOBILE_LIFT_CONTROLLER_ID,
        FLAGSHIP_LEKIWI_ACTION_PROJECTION_SCHEMA_VERSION,
    )
}

/// Projects one complete portable v2 controller action into bounded LeKiwi base action.
pub fn project_flagship_action_to_lekiwi_v2(
    parent_action: &[f64],
) -> Result<FlagshipLeKiwiActionProjection, FlagshipLeKiwiProjectionError> {
    let parent_task = flagship_mobile_lift_task_spec_v2(FLAGSHIP_MOBILE_LIFT_CONTROL_PERIOD_TICKS);
    project_action(
        parent_action,
        &parent_task,
        FLAGSHIP_MOBILE_LIFT_TASK_ID_V2,
        FLAGSHIP_MOBILE_LIFT_CONTROLLER_ID_V2,
        FLAGSHIP_LEKIWI_ACTION_PROJECTION_CURRENT_SCHEMA_VERSION,
    )
}

fn project_action(
    parent_action: &[f64],
    parent_task: &TaskSpec,
    parent_task_id: &str,
    parent_controller_id: &str,
    schema_version: u32,
) -> Result<FlagshipLeKiwiActionProjection, FlagshipLeKiwiProjectionError> {
    validate_action(parent_task, parent_action, true)?;

    let (linear_x_m_s, angular_z_rad_s) =
        mm_mobile_wheel_velocities_to_twist(parent_action[0], parent_action[1]);
    let physical_action = [linear_x_m_s, 0.0, angular_z_rad_s];
    let physical_task = lekiwi_base_task_spec();
    validate_action(&physical_task, &physical_action, false)?;

    let suppressed_actions = [
        ("arm_joint_target_rad", 0, "rad", parent_action[2]),
        ("arm_joint_target_rad", 1, "rad", parent_action[3]),
        ("arm_joint_target_rad", 2, "rad", parent_action[4]),
        ("lift_target_m", 0, "m", parent_action[5]),
        ("gripper_velocity_m_s", 0, "m/s", parent_action[6]),
    ]
    .into_iter()
    .map(
        |(tensor_name, tensor_element, unit, value)| SuppressedFlagshipAction {
            tensor_name: tensor_name.to_string(),
            tensor_element,
            unit: unit.to_string(),
            value,
        },
    )
    .collect();

    Ok(FlagshipLeKiwiActionProjection {
        kind: FLAGSHIP_LEKIWI_ACTION_PROJECTION_KIND.to_string(),
        schema_version,
        parent_task_id: parent_task_id.to_string(),
        parent_controller_id: parent_controller_id.to_string(),
        physical_task_id: LEKIWI_BASE_TASK_ID.to_string(),
        physical_profile_id: LEKIWI_REFERENCE_PROFILE_ID.to_string(),
        parent_action_sha256: action_sha256(parent_action),
        transform: FlagshipLeKiwiActionTransform {
            parent_wheel_radius_m: rne_ai::env::MM_MOBILE_WHEEL_RADIUS_M,
            parent_track_width_m: rne_ai::env::MM_MOBILE_TRACK_WIDTH_M,
            parent_wheel_unit: "rad/s".to_string(),
            physical_linear_unit: "m/s".to_string(),
            physical_angular_unit: "rad/s".to_string(),
            body_axis_convention: "+x_forward,+y_left,+yaw_ccw".to_string(),
            transform_id: "diff_drive_wheels_to_planar_twist.v1".to_string(),
            envelope_policy: "fail_closed_without_clamping".to_string(),
        },
        physical_action_values: physical_action,
        suppressed_actions,
        status: "passed".to_string(),
    })
}

fn validate_action(
    task: &TaskSpec,
    values: &[f64],
    parent: bool,
) -> Result<(), FlagshipLeKiwiProjectionError> {
    let expected = task
        .action
        .tensors
        .iter()
        .map(tensor_elements)
        .sum::<usize>();
    if values.len() != expected {
        return Err(FlagshipLeKiwiProjectionError::ActionWidth {
            boundary: if parent { "parent" } else { "physical" },
            expected,
            actual: values.len(),
        });
    }

    let mut flat_index = 0_usize;
    for tensor in &task.action.tensors {
        let count = tensor_elements(tensor);
        let bounds =
            tensor
                .bounds
                .as_ref()
                .ok_or(FlagshipLeKiwiProjectionError::MissingBounds {
                    boundary: if parent { "parent" } else { "physical" },
                    tensor: tensor.name.clone(),
                })?;
        for element in 0..count {
            let value = values[flat_index];
            if !value.is_finite() {
                return Err(FlagshipLeKiwiProjectionError::NonFiniteAction {
                    boundary: if parent { "parent" } else { "physical" },
                    tensor: tensor.name.clone(),
                    element,
                });
            }
            let lower = bound_at(bounds, element, true);
            let upper = bound_at(bounds, element, false);
            if value < lower || value > upper {
                return Err(FlagshipLeKiwiProjectionError::ActionLimit {
                    boundary: if parent { "parent" } else { "physical" },
                    tensor: tensor.name.clone(),
                    element,
                    value,
                    lower,
                    upper,
                });
            }
            flat_index += 1;
        }
    }
    Ok(())
}

fn tensor_elements(tensor: &TensorSpec) -> usize {
    tensor.shape.iter().copied().product::<usize>().max(1)
}

fn bound_at(bounds: &TensorBounds, element: usize, lower: bool) -> f64 {
    let side = if lower { &bounds.lower } else { &bounds.upper };
    side[if side.len() == 1 { 0 } else { element }]
}

fn action_sha256(values: &[f64]) -> String {
    let mut hasher = Sha256::new();
    hasher.update((values.len() as u64).to_le_bytes());
    for value in values {
        hasher.update(value.to_bits().to_le_bytes());
    }
    format!("{:x}", hasher.finalize())
}

/// Failure validating or projecting one flagship controller action.
#[derive(Clone, Debug, PartialEq, thiserror::Error)]
pub enum FlagshipLeKiwiProjectionError {
    /// A flattened action did not match its bound TaskSpec width.
    #[error("{boundary} action width must be {expected}, got {actual}")]
    ActionWidth {
        /// Parent or physical boundary.
        boundary: &'static str,
        /// Required flattened width.
        expected: usize,
        /// Supplied flattened width.
        actual: usize,
    },
    /// A TaskSpec action tensor omitted mandatory hardware limits.
    #[error("{boundary} action tensor {tensor:?} has no bounds")]
    MissingBounds {
        /// Parent or physical boundary.
        boundary: &'static str,
        /// Tensor without bounds.
        tensor: String,
    },
    /// A controller or projected action contained NaN or infinity.
    #[error("{boundary} action {tensor}[{element}] must be finite")]
    NonFiniteAction {
        /// Parent or physical boundary.
        boundary: &'static str,
        /// Tensor containing the value.
        tensor: String,
        /// Row-major tensor element.
        element: usize,
    },
    /// An action exceeded the exact TaskSpec envelope.
    #[error("{boundary} action {tensor}[{element}]={value} is outside [{lower}, {upper}]")]
    ActionLimit {
        /// Parent or physical boundary.
        boundary: &'static str,
        /// Tensor containing the value.
        tensor: String,
        /// Row-major tensor element.
        element: usize,
        /// Rejected value.
        value: f64,
        /// Inclusive lower bound.
        lower: f64,
        /// Inclusive upper bound.
        upper: f64,
    },
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{LEKIWI_MAX_ANGULAR_SPEED_RAD_S, LEKIWI_MAX_LINEAR_SPEED_M_S};

    fn action(left_rad_s: f64, right_rad_s: f64) -> Vec<f64> {
        vec![left_rad_s, right_rad_s, 0.1, -0.2, 0.3, 0.04, -0.01]
    }

    #[test]
    fn straight_action_projects_without_clamping_and_retains_suppression() {
        let report = project_flagship_action_to_lekiwi(&action(1.0, 1.0)).unwrap();
        assert_eq!(report.parent_task_id, FLAGSHIP_MOBILE_LIFT_TASK_ID);
        assert_eq!(report.physical_task_id, LEKIWI_BASE_TASK_ID);
        assert_eq!(report.physical_action_values, [0.1, 0.0, 0.0]);
        assert_eq!(report.suppressed_actions.len(), 5);
        assert_eq!(report.suppressed_actions[0].value, 0.1);
        assert_eq!(report.parent_action_sha256.len(), 64);
        assert_eq!(report.status, "passed");
    }

    #[test]
    fn v2_projection_binds_portable_task_and_controller() {
        let report = project_flagship_action_to_lekiwi_v2(&action(1.0, 1.0)).unwrap();
        assert_eq!(report.schema_version, 2);
        assert_eq!(report.parent_task_id, FLAGSHIP_MOBILE_LIFT_TASK_ID_V2);
        assert_eq!(
            report.parent_controller_id,
            FLAGSHIP_MOBILE_LIFT_CONTROLLER_ID_V2
        );
        assert_eq!(report.physical_action_values, [0.1, 0.0, 0.0]);
    }

    #[test]
    fn turn_uses_parent_geometry_and_is_deterministic() {
        let first = project_flagship_action_to_lekiwi(&action(-1.0, 1.0)).unwrap();
        let second = project_flagship_action_to_lekiwi(&action(-1.0, 1.0)).unwrap();
        assert_eq!(first, second);
        assert_eq!(first.physical_action_values[0], 0.0);
        assert!((first.physical_action_values[2] - (0.2 / 0.45)).abs() < 1.0e-12);
        assert_eq!(
            first.transform.envelope_policy,
            "fail_closed_without_clamping"
        );
    }

    #[test]
    fn physical_envelope_parent_limit_and_nonfinite_input_fail_closed() {
        assert!(matches!(
            project_flagship_action_to_lekiwi(&action(2.0, 2.0)),
            Err(FlagshipLeKiwiProjectionError::ActionLimit {
                boundary: "physical",
                ..
            })
        ));
        assert!(matches!(
            project_flagship_action_to_lekiwi(&action(10.1, 0.0)),
            Err(FlagshipLeKiwiProjectionError::ActionLimit {
                boundary: "parent",
                ..
            })
        ));
        let mut invalid = action(0.0, 0.0);
        invalid[3] = f64::NAN;
        assert!(matches!(
            project_flagship_action_to_lekiwi(&invalid),
            Err(FlagshipLeKiwiProjectionError::NonFiniteAction {
                boundary: "parent",
                ..
            })
        ));
    }

    #[test]
    fn exported_limits_match_the_physical_task_contract() {
        let task = lekiwi_base_task_spec();
        assert_eq!(
            task.action.tensors[0].bounds.as_ref().unwrap().upper[0],
            LEKIWI_MAX_LINEAR_SPEED_M_S
        );
        assert_eq!(
            task.action.tensors[1].bounds.as_ref().unwrap().upper[0],
            LEKIWI_MAX_ANGULAR_SPEED_RAD_S
        );
    }
}
