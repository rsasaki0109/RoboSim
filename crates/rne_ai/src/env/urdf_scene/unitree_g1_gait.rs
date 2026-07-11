use super::UrdfJointPositionTarget;

/// Command for the deterministic Unitree G1 walking gait generator.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct UnitreeG1GaitCommand {
    /// Hip-pitch stride amplitude in radians, clamped to `[0, 0.35]`.
    pub stride_rad: f64,
    /// Additional swing-leg knee bend in radians, clamped to `[0, 0.45]`.
    pub foot_lift_rad: f64,
    /// Number of simulation steps in one gait cycle, clamped to `[40, 180]`.
    pub cycle_steps: u64,
}

impl Default for UnitreeG1GaitCommand {
    fn default() -> Self {
        Self {
            stride_rad: 0.05,
            foot_lift_rad: 0.05,
            cycle_steps: 120,
        }
    }
}

/// Generates one deterministic 23-DoF G1 walking pose.
///
/// The returned targets use child-link names, matching
/// [`super::UrdfSceneSim::step_joint_position_targets`]. Left and right legs
/// run half a cycle apart while the arms counter-swing.
pub fn unitree_g1_gait_targets(
    step: u64,
    command: UnitreeG1GaitCommand,
) -> [UrdfJointPositionTarget<'static>; 23] {
    let stride = command.stride_rad.clamp(0.0, 0.35);
    let lift = command.foot_lift_rad.clamp(0.0, 0.45);
    let cycle = command.cycle_steps.clamp(40, 180);
    let phase = (step % cycle) as f64 / cycle as f64;
    let (left, left_lift) = gait_wave(phase);
    let (right, right_lift) = gait_wave((phase + 0.5) % 1.0);
    let leg = |side: &'static str, wave: f64, swing_lift: f64| {
        let (hip_pitch, hip_roll, hip_yaw, knee, ankle_pitch, ankle_roll) = if side == "left" {
            (
                "left_hip_pitch_link",
                "left_hip_roll_link",
                "left_hip_yaw_link",
                "left_knee_link",
                "left_ankle_pitch_link",
                "left_ankle_roll_link",
            )
        } else {
            (
                "right_hip_pitch_link",
                "right_hip_roll_link",
                "right_hip_yaw_link",
                "right_knee_link",
                "right_ankle_pitch_link",
                "right_ankle_roll_link",
            )
        };
        [
            target(hip_pitch, -0.18 + stride * wave),
            target(hip_roll, if side == "left" { 0.05 } else { -0.05 }),
            target(hip_yaw, 0.0),
            target(knee, 0.36 + lift * swing_lift),
            target(
                ankle_pitch,
                -0.18 - 0.45 * stride * wave - 0.5 * lift * swing_lift,
            ),
            target(ankle_roll, if side == "left" { -0.03 } else { 0.03 }),
        ]
    };
    let l = leg("left", left, left_lift);
    let r = leg("right", right, right_lift);
    [
        l[0],
        l[1],
        l[2],
        l[3],
        l[4],
        l[5],
        r[0],
        r[1],
        r[2],
        r[3],
        r[4],
        r[5],
        target("torso_link", 0.0),
        target("left_shoulder_pitch_link", -0.7 * stride * left),
        target("left_shoulder_roll_link", 0.20),
        target("left_shoulder_yaw_link", 0.0),
        target("left_elbow_link", 0.42),
        target("left_wrist_roll_rubber_hand", 0.0),
        target("right_shoulder_pitch_link", -0.7 * stride * right),
        target("right_shoulder_roll_link", -0.20),
        target("right_shoulder_yaw_link", 0.0),
        target("right_elbow_link", 0.42),
        target("right_wrist_roll_rubber_hand", 0.0),
    ]
}

fn gait_wave(phase: f64) -> (f64, f64) {
    const STANCE_FRACTION: f64 = 0.62;
    if phase < STANCE_FRACTION {
        (1.0 - 2.0 * phase / STANCE_FRACTION, 0.0)
    } else {
        let swing = (phase - STANCE_FRACTION) / (1.0 - STANCE_FRACTION);
        (-1.0 + 2.0 * swing, (std::f64::consts::PI * swing).sin())
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
    fn gait_is_periodic_and_clamps_commands() {
        let command = UnitreeG1GaitCommand {
            stride_rad: 9.0,
            foot_lift_rad: 9.0,
            cycle_steps: 20,
        };
        assert_eq!(
            unitree_g1_gait_targets(0, command),
            unitree_g1_gait_targets(40, command)
        );
        for target in unitree_g1_gait_targets(10, command) {
            assert!(target.position.is_finite());
            assert!(target.position.abs() <= 1.0);
        }
    }
}
