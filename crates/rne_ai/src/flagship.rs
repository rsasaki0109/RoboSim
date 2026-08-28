//! Portable contract shared by the installed mobile-manipulation flagship.

use crate::{
    ActionSpec, ObservationSpec, RandomDistributionSpec, RandomizationParameterSpec,
    RandomizationSpec, ResetSpec, RewardSpec, RewardTermSpec, TaskSpec, TensorBounds, TensorDType,
    TensorSpec, TerminationConditionSpec, TerminationKind, TerminationSpec,
};

/// Stable TaskSpec identity used by the installed mobile-lift flagship.
pub const FLAGSHIP_MOBILE_LIFT_TASK_ID: &str = "rne.flagship.mobile_lift_shared_aisle.v1";

/// Stable controller identity used by the installed mobile-lift flagship.
pub const FLAGSHIP_MOBILE_LIFT_CONTROLLER_ID: &str = "rne.ai.ik_mobile_lift_pick_place_policy.v1";

/// Maximum controller decisions in one flagship episode.
pub const FLAGSHIP_MOBILE_LIFT_MAX_WORKFLOW_STEPS: u64 = 8_000;

/// Domain-randomization identity for traffic departure delay.
pub const FLAGSHIP_TRAFFIC_DEPARTURE_DIMENSION: &str = "traffic_departure_delay_s";

/// Domain-randomization identity for traffic speed delta.
pub const FLAGSHIP_TRAFFIC_SPEED_DIMENSION: &str = "traffic_speed_delta_m_s";

/// Builds the exact portable TaskSpec used by installed flagship proofs.
///
/// `fixed_delta_ticks` is the integer number of nanoseconds in one controller
/// decision. Keeping that integer at the API boundary prevents a hardware,
/// simulator, or recorded-stream adapter from silently choosing a different
/// rate through floating-point rounding.
pub fn flagship_mobile_lift_task_spec(fixed_delta_ticks: u64) -> TaskSpec {
    TaskSpec::new(
        FLAGSHIP_MOBILE_LIFT_TASK_ID,
        fixed_delta_ticks as f64 / 1_000_000_000.0,
        ObservationSpec::new(vec![
            TensorSpec::new("base_position_m", TensorDType::F64, vec![2], "m"),
            TensorSpec::new("arm_joint_position_rad", TensorDType::F64, vec![3], "rad"),
            TensorSpec::new("lift_position_m", TensorDType::F64, vec![], "m"),
            TensorSpec::new("gripper_position_m", TensorDType::F64, vec![], "m"),
            TensorSpec::new("payload_position_m", TensorDType::F64, vec![3], "m"),
            TensorSpec::new("wrist_camera_pixel_count", TensorDType::I64, vec![], "1")
                .with_bounds(TensorBounds::broadcast(0.0, i64::MAX as f64)),
            TensorSpec::new("wrist_depth_min_m", TensorDType::F64, vec![], "m")
                .with_bounds(TensorBounds::broadcast(0.0, f64::MAX)),
            TensorSpec::new("traffic_actor_position_m", TensorDType::F64, vec![3], "m"),
            TensorSpec::new("traffic_signal_green", TensorDType::Bool, vec![], "1"),
            TensorSpec::new("traffic_clear", TensorDType::Bool, vec![], "1"),
            TensorSpec::new("grasped", TensorDType::Bool, vec![], "1"),
            TensorSpec::new("policy_phase", TensorDType::I32, vec![], "1")
                .with_bounds(TensorBounds::broadcast(0.0, 9.0)),
        ]),
        ActionSpec::new(vec![
            TensorSpec::new("wheel_velocity_rad_s", TensorDType::F64, vec![2], "rad/s")
                .with_bounds(TensorBounds::broadcast(-10.0, 10.0)),
            TensorSpec::new("arm_joint_target_rad", TensorDType::F64, vec![3], "rad").with_bounds(
                TensorBounds::broadcast(-std::f64::consts::PI, std::f64::consts::PI),
            ),
            TensorSpec::new("lift_target_m", TensorDType::F64, vec![], "m")
                .with_bounds(TensorBounds::broadcast(-0.5, 0.5)),
            TensorSpec::new("gripper_velocity_m_s", TensorDType::F64, vec![], "m/s")
                .with_bounds(TensorBounds::broadcast(-0.1, 0.1)),
        ]),
        RewardSpec::weighted_sum(vec![
            RewardTermSpec::new("task_progress_m", 1.0, "m"),
            RewardTermSpec::new("step", -0.001, "1"),
            RewardTermSpec::new("task_completed", 10.0, "1"),
        ]),
        TerminationSpec::new(
            vec![
                TerminationConditionSpec::new(
                    "inspection_pick_place_completed",
                    TerminationKind::Success,
                ),
                TerminationConditionSpec::new("perception_stream_lost", TerminationKind::Failure),
            ],
            Some(FLAGSHIP_MOBILE_LIFT_MAX_WORKFLOW_STEPS),
        ),
        ResetSpec::splitmix64(false),
    )
    .with_randomization(RandomizationSpec::new(vec![
        RandomizationParameterSpec::new(
            FLAGSHIP_TRAFFIC_DEPARTURE_DIMENSION,
            "s",
            RandomDistributionSpec::Uniform {
                minimum: 0.0,
                maximum: 0.25,
            },
        ),
        RandomizationParameterSpec::new(
            FLAGSHIP_TRAFFIC_SPEED_DIMENSION,
            "m/s",
            RandomDistributionSpec::Uniform {
                minimum: 0.0,
                maximum: 0.25,
            },
        ),
    ]))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn flagship_contract_is_valid_ordered_and_tick_exact() {
        let task = flagship_mobile_lift_task_spec(16_666_667);
        task.validate().unwrap();

        assert_eq!(task.task_id, FLAGSHIP_MOBILE_LIFT_TASK_ID);
        assert_eq!(task.control_step_s, 0.016_666_667);
        assert_eq!(task.termination.max_episode_steps, Some(8_000));
        assert_eq!(task.observation.tensors.len(), 12);
        assert_eq!(task.action.tensors.len(), 4);
        assert_eq!(task.action.tensors[0].name, "wheel_velocity_rad_s");
        assert_eq!(task.action.tensors[1].name, "arm_joint_target_rad");
        assert_eq!(
            task.randomization.as_ref().map(|contract| {
                contract
                    .parameters
                    .iter()
                    .map(|parameter| parameter.name.as_str())
                    .collect::<Vec<_>>()
            }),
            Some(vec![
                FLAGSHIP_TRAFFIC_DEPARTURE_DIMENSION,
                FLAGSHIP_TRAFFIC_SPEED_DIMENSION,
            ])
        );
    }
}
