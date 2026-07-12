use super::{unitree_g1_gait_targets, UnitreeG1GaitCommand, UrdfJointPositionTarget};

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
        return unitree_g1_gait_targets(
            task_step,
            UnitreeG1GaitCommand {
                stride_rad: 0.06,
                foot_lift_rad: 0.06,
                cycle_steps: 60,
            },
        );
    }

    let mut targets = unitree_g1_gait_targets(
        0,
        UnitreeG1GaitCommand {
            stride_rad: 0.0,
            foot_lift_rad: 0.0,
            cycle_steps: 60,
        },
    );
    let blend = smoothstep((task_step - APPROACH_STEPS) as f64 / 30.0);
    set_target(&mut targets, "right_shoulder_pitch_link", -1.15 * blend);
    set_target(
        &mut targets,
        "right_shoulder_roll_link",
        -0.20 - 0.18 * blend,
    );
    set_target(&mut targets, "right_shoulder_yaw_link", -0.30 * blend);
    set_target(&mut targets, "right_elbow_link", 0.42 - 0.24 * blend);
    set_target(&mut targets, "right_wrist_roll_rubber_hand", 0.35 * blend);
    targets
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
