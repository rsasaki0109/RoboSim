use super::{
    step_unitree_g1_hybrid_joint_targets, unitree_g1_gait_targets_with_arm_pose, UnitreeG1ArmPose,
    UnitreeG1GaitCommand, UrdfJointPositionTarget, UrdfSceneSim,
};

/// Arm pose used by the factory inspection task's walk-and-point sequence.
///
/// Opted into [`UnitreeG1ArmPose::Hanging`] rather than the shared default:
/// this task's own gates (`factory_inspection_completes_inside_named_marker`,
/// `factory_inspection_stays_upright_throughout`,
/// `inspection_sequence_walks_then_points_and_repeats`) were re-validated
/// against it, unlike the wider gait-generator consumers that keep the
/// original pinned pose. See [`UnitreeG1ArmPose`] for why this needs to be
/// explicit instead of a codebase-wide default.
const INSPECTION_ARM_POSE: UnitreeG1ArmPose = UnitreeG1ArmPose::Hanging;

/// Generates a deterministic G1 walk-and-inspect task pose.
///
/// The first half of the sequence approaches the inspection station with a
/// short gait. The second half settles into a standing pose and raises the
/// right arm for a visible point-and-confirm gesture. `step` repeats every 120
/// simulation steps.
pub fn unitree_g1_inspection_targets(step: u64) -> [UrdfJointPositionTarget<'static>; 23] {
    const APPROACH_STEPS: u64 = 60;
    const TASK_STEPS: u64 = 120;
    let task_step = step % TASK_STEPS;
    if task_step < APPROACH_STEPS {
        // NOTE: the scripted hybrid gait is a near-stationary stepper across
        // its entire stable envelope (see docs/G1_LOCOMOTION.md): measured
        // base displacement here is only a few centimeters regardless of
        // stride amplitude, and this operating point is right at the
        // hybrid tick's discrete-stability edge — even a modest
        // `foot_lift_rad` increase (0.06 -> 0.08) was enough to blow the
        // solver up to NaN in testing, and a larger stride carried the
        // pelvis outside a marker's radius. So the pinned v0.1 stride/lift
        // is kept exactly as validated. A visibly longer walk between
        // markers needs the learned transport torque overlay
        // (`UnitreeG1TorqueOverlay::LEARNED_STRIDE`, as used by example 92)
        // ported into this approach phase under the same chaos-tested
        // discipline, which also needs a step-budget change that touches
        // the G1 workbench mission's walk timeout; left as follow-up.
        return unitree_g1_gait_targets_with_arm_pose(
            task_step,
            UnitreeG1GaitCommand {
                stride_rad: 0.06,
                foot_lift_rad: 0.06,
                cycle_steps: 60,
            },
            INSPECTION_ARM_POSE,
        );
    }

    let mut targets = unitree_g1_gait_targets_with_arm_pose(
        0,
        UnitreeG1GaitCommand {
            stride_rad: 0.0,
            foot_lift_rad: 0.0,
            cycle_steps: 60,
        },
        INSPECTION_ARM_POSE,
    );
    // Blend from the relaxed hanging-arm rest pose (see
    // `UnitreeG1ArmPose::Hanging` in `unitree_g1_gait`) to a deliberate
    // raised point-and-confirm gesture, rather than snapping from a
    // forward-reaching idle into another forward reach, which used to read
    // as an aimless jab.
    let blend = smoothstep((task_step - APPROACH_STEPS) as f64 / 30.0);
    set_target(
        &mut targets,
        "right_shoulder_pitch_link",
        INSPECTION_ARM_POSE.pitch_bias_rad() - 1.15 * blend,
    );
    set_target(
        &mut targets,
        "right_shoulder_roll_link",
        -INSPECTION_ARM_POSE.roll_rad() - 0.18 * blend,
    );
    set_target(&mut targets, "right_shoulder_yaw_link", -0.30 * blend);
    // The rest elbow bend (`UnitreeG1ArmPose::elbow_rad`) hangs the forearm
    // down; the point-and-confirm gesture straightens it back out toward
    // 0.15 rad as the raised shoulder brings the arm up and forward, so the
    // pointing hand reads as reaching rather than staying folded against
    // the hip.
    set_target(
        &mut targets,
        "right_elbow_link",
        INSPECTION_ARM_POSE.elbow_rad() - (INSPECTION_ARM_POSE.elbow_rad() - 0.15) * blend,
    );
    set_target(&mut targets, "right_wrist_roll_rubber_hand", 0.35 * blend);
    targets
}

/// Advances one factory-inspection tick on the validated hybrid G1 pathway.
///
/// Point-and-confirm segments use the same hybrid stepper as the locomotion
/// harness; approach segments reuse the pinned stride/lift/cycle pose envelope
/// without learned transport torques so marker radii stay satisfied.
pub fn step_unitree_g1_inspection(sim: &mut UrdfSceneSim, step: u64) {
    let targets = unitree_g1_inspection_targets(step);
    step_unitree_g1_hybrid_joint_targets(sim, &targets, [0.0; 8]);
}

fn smoothstep(value: f64) -> f64 {
    let value = value.clamp(0.0, 1.0);
    value * value * (3.0 - 2.0 * value)
}

fn set_target(
    targets: &mut [UrdfJointPositionTarget<'static>],
    link_name: &'static str,
    position: f64,
) {
    let target = targets
        .iter_mut()
        .find(|target| target.link_name == link_name)
        .expect("G1 inspection target link must exist in the gait pose");
    target.position = position;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn inspection_sequence_walks_then_points_and_repeats() {
        let walking = unitree_g1_inspection_targets(15);
        let pointing = unitree_g1_inspection_targets(100);
        assert_ne!(walking, pointing);
        assert_eq!(
            pointing
                .iter()
                .find(|target| target.link_name == "right_shoulder_pitch_link")
                .expect("right shoulder target")
                .position,
            -1.15
        );
        assert_eq!(
            unitree_g1_inspection_targets(0),
            unitree_g1_inspection_targets(120)
        );
        assert!(pointing.iter().all(|target| target.position.is_finite()));
    }
}
