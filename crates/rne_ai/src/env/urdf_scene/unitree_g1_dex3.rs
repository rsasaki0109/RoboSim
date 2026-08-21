use super::UrdfJointPositionTarget;

/// Normalized command for the right Unitree Dex3-1 hand.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct UnitreeG1Dex3HandCommand {
    /// Finger closure in the inclusive range 0 (open) to 1 (pinched).
    pub closure: f64,
}

/// Generates a fixed-base G1 29-DoF arm pose and an articulated Dex3 pinch.
///
/// `approach_blend` transitions from a slightly retracted pose to the low work
/// pose, while `lift_blend` transitions that work pose to the raised carry pose.
/// The hand closure is clamped to `[0, 1]` and drives the thumb, index, and
/// middle finger within their official limits. The inactive left arm and hand
/// receive explicit neutral targets so contact cannot make them drift.
///
/// For the horizontal carry sweep, use
/// [`unitree_g1_dex3_pick_targets_with_carry`].
pub fn unitree_g1_dex3_pick_targets(
    approach_blend: f64,
    lift_blend: f64,
    hand: UnitreeG1Dex3HandCommand,
) -> Vec<UrdfJointPositionTarget<'static>> {
    unitree_g1_dex3_pick_targets_with_carry(approach_blend, lift_blend, 0.0, hand)
}

/// Like [`unitree_g1_dex3_pick_targets`], with an explicit `carry_blend` sweep
/// toward the place tray after lift.
pub fn unitree_g1_dex3_pick_targets_with_carry(
    approach_blend: f64,
    lift_blend: f64,
    carry_blend: f64,
    hand: UnitreeG1Dex3HandCommand,
) -> Vec<UrdfJointPositionTarget<'static>> {
    let approach = smoothstep(approach_blend.clamp(0.0, 1.0));
    let lift = smoothstep(lift_blend.clamp(0.0, 1.0));
    let carry = smoothstep(carry_blend.clamp(0.0, 1.0));
    let closure = smoothstep(hand.closure.clamp(0.0, 1.0));
    vec![
        target("left_hip_pitch_link", -0.18),
        target("left_knee_link", 0.36),
        target("left_ankle_pitch_link", -0.18),
        target("right_hip_pitch_link", -0.18),
        target("right_knee_link", 0.36),
        target("right_ankle_pitch_link", -0.18),
        target("waist_yaw_link", 0.28 * carry),
        target("waist_roll_link", 0.0),
        target("torso_link", 0.0),
        target("left_shoulder_pitch_link", 0.0),
        target("left_shoulder_roll_link", 0.0),
        target("left_shoulder_yaw_link", 0.0),
        target("left_elbow_link", 0.0),
        target("left_wrist_roll_link", 0.0),
        target("left_wrist_pitch_link", 0.0),
        target("left_wrist_yaw_link", 0.0),
        target("left_hand_thumb_0_link", 0.0),
        target("left_hand_thumb_1_link", 0.0),
        target("left_hand_thumb_2_link", 0.0),
        target("left_hand_middle_0_link", 0.0),
        target("left_hand_middle_1_link", 0.0),
        target("left_hand_index_0_link", 0.0),
        target("left_hand_index_1_link", 0.0),
        target(
            "right_shoulder_pitch_link",
            0.18 * (1.0 - approach) * (1.0 - lift) - 0.58 * lift - 0.08 * carry,
        ),
        target(
            "right_shoulder_roll_link",
            -0.20 - 0.09 * lift - 0.06 * carry,
        ),
        target("right_shoulder_yaw_link", -0.15 * lift - 0.40 * carry),
        target(
            "right_elbow_link",
            (0.55 - 0.13 * approach) * (1.0 - lift) + 0.30 * lift + 0.05 * carry,
        ),
        target("right_wrist_roll_link", 0.18 * lift),
        target("right_wrist_pitch_link", 0.0),
        target("right_wrist_yaw_link", 0.10 * carry),
        target("right_hand_thumb_0_link", 0.0),
        target("right_hand_thumb_1_link", -0.75 * closure),
        target("right_hand_thumb_2_link", -1.15 * closure),
        target("right_hand_middle_0_link", 1.35 * closure),
        target("right_hand_middle_1_link", 1.35 * closure),
        target("right_hand_index_0_link", 1.40 * closure),
        target("right_hand_index_1_link", 1.35 * closure),
    ]
}

fn smoothstep(value: f64) -> f64 {
    value * value * (3.0 - 2.0 * value)
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
    fn dex3_targets_clamp_and_close_all_three_fingers() {
        let open = unitree_g1_dex3_pick_targets(-1.0, -1.0, UnitreeG1Dex3HandCommand::default());
        let closed =
            unitree_g1_dex3_pick_targets(2.0, 2.0, UnitreeG1Dex3HandCommand { closure: 2.0 });
        assert_eq!(open.len(), closed.len());
        assert!(open.iter().all(|target| target.position.is_finite()));
        for link_name in [
            "right_hand_thumb_2_link",
            "right_hand_middle_1_link",
            "right_hand_index_1_link",
        ] {
            let open_position = open
                .iter()
                .find(|target| target.link_name == link_name)
                .expect("open finger target")
                .position;
            let closed_position = closed
                .iter()
                .find(|target| target.link_name == link_name)
                .expect("closed finger target")
                .position;
            assert_ne!(open_position, closed_position);
        }
    }

    #[test]
    fn inactive_left_arm_stays_in_the_same_neutral_pose() {
        let approach =
            unitree_g1_dex3_pick_targets(0.0, 0.0, UnitreeG1Dex3HandCommand { closure: 0.0 });
        let carry = unitree_g1_dex3_pick_targets_with_carry(
            1.0,
            1.0,
            1.0,
            UnitreeG1Dex3HandCommand { closure: 1.0 },
        );
        for link_name in [
            "left_shoulder_pitch_link",
            "left_shoulder_roll_link",
            "left_shoulder_yaw_link",
            "left_elbow_link",
            "left_wrist_roll_link",
            "left_wrist_pitch_link",
            "left_wrist_yaw_link",
            "left_hand_thumb_0_link",
            "left_hand_thumb_1_link",
            "left_hand_thumb_2_link",
            "left_hand_middle_0_link",
            "left_hand_middle_1_link",
            "left_hand_index_0_link",
            "left_hand_index_1_link",
        ] {
            let position = |targets: &[UrdfJointPositionTarget<'_>]| {
                targets
                    .iter()
                    .find(|target| target.link_name == link_name)
                    .expect("stowed left-side target")
                    .position
            };
            assert_eq!(position(&approach), position(&carry), "{link_name}");
        }
    }

    #[test]
    fn carry_blend_sweeps_waist_and_right_arm_toward_the_place_zone() {
        let lifted = unitree_g1_dex3_pick_targets_with_carry(
            1.0,
            1.0,
            0.0,
            UnitreeG1Dex3HandCommand { closure: 1.0 },
        );
        let carried = unitree_g1_dex3_pick_targets_with_carry(
            1.0,
            1.0,
            1.0,
            UnitreeG1Dex3HandCommand { closure: 1.0 },
        );
        let position = |targets: &[UrdfJointPositionTarget<'_>], link_name: &str| {
            targets
                .iter()
                .find(|target| target.link_name == link_name)
                .expect(link_name)
                .position
        };
        assert!(position(&carried, "waist_yaw_link") > position(&lifted, "waist_yaw_link"));
        assert!(
            position(&carried, "right_shoulder_yaw_link")
                < position(&lifted, "right_shoulder_yaw_link")
        );
    }
}
