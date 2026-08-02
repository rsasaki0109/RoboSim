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

/// Contact-gated Fourier feed-forward torque overlay for the G1's hybrid
/// walking tick (hips and knees under torque PD, ankles and arms servo-held).
///
/// The scripted G1 gait is a near-stationary stepper across its entire
/// stable envelope, so transport must be *created* by stance torques — the
/// force-level freedom the Go2 transport search proved out. For each of the
/// eight proximal joints the overlay adds the same Fourier form as the Go2
/// overlay (a stance-gated constant plus full- and half-frequency harmonics
/// over the two-cycle gait phase) in newton-meters on top of the tracking
/// PD; the gate is that leg's *measured* foot contact.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct UnitreeG1TorqueOverlay {
    /// Per-joint `[stance_const, constant, sin, cos, half_sin, half_cos]`
    /// torque coefficients in N·m; joints ordered left hip pitch/roll/yaw,
    /// left knee, then the same for the right leg.
    pub coefficients: [[f64; 6]; 8],
}

impl UnitreeG1TorqueOverlay {
    /// The neutral overlay: reproduces the plain hybrid gait exactly.
    pub const ZERO: Self = Self {
        coefficients: [[0.0; 6]; 8],
    };

    /// The first G1 gait in these measurements that genuinely covers ground
    /// (`examples/62_g1_learned_stride -- --train`, seed 42, ensemble-median
    /// CEM with the anti-cheat window-displacement objective).
    ///
    /// The scripted G1 gait is a near-stationary stepper across its entire
    /// stable envelope; this overlay turns it into a slow but real walk —
    /// 0.22 m per 8 s window (over 2× the stepper), 0.64 m per 24 s, at full
    /// height (0.784 m) and dead straight, with ulp-perturbed replays inside
    /// a few centimeters of each other. The speed-envelope sweep uses the
    /// learned search winner at 66% feed-forward strength with a 0.07 rad,
    /// 0.10 rad, 100-step gait command. Pinned at the search state's
    /// 12-decimal precision per the chaos discipline;
    /// `learned_torques_make_the_g1_stride` pins the comparison.
    pub const LEARNED_STRIDE: Self = Self {
        coefficients: [
            [
                2.401766232113,
                1.209639963872,
                0.784961496308,
                8.361950984154,
                1.068444910014,
                -0.974861335217,
            ],
            [
                -2.466553499449,
                -0.146208972287,
                3.806744549080,
                -5.037352749578,
                -1.749591438981,
                1.040276181553,
            ],
            [
                0.950502511256,
                2.002565623718,
                -2.431353598553,
                -12.604115582628,
                0.078459056935,
                -0.383736313371,
            ],
            [
                -2.197758056703,
                -5.847472837774,
                -3.752157325820,
                -0.230921782589,
                -0.304970503130,
                2.300020847130,
            ],
            [
                -1.405609538874,
                -1.543349051077,
                -0.480486977714,
                -0.927744073973,
                1.032180989821,
                1.523823520803,
            ],
            [
                0.637280260250,
                -2.116539439527,
                2.035432479931,
                -1.535493816237,
                -0.073423517697,
                -0.390382920399,
            ],
            [
                -1.094001100257,
                0.221010430093,
                -13.200000000000,
                -1.281992063252,
                -2.630784814139,
                2.036457368260,
            ],
            [
                -1.051488597927,
                -9.188782208397,
                -1.843817972976,
                2.281156548618,
                -0.895711846245,
                0.341133308541,
            ],
        ],
    };

    /// Feed-forward joint torques at a two-cycle gait phase in `[0, 1)` with
    /// the measured per-leg stance gates `[left, right]`, each clamped to
    /// ±40 N·m so the overlay stays inside the hip actuators' authority.
    pub fn torques_nm(&self, two_cycle_phase: f64, stance: [bool; 2]) -> [f64; 8] {
        let full = 4.0 * std::f64::consts::PI * two_cycle_phase;
        let half = 2.0 * std::f64::consts::PI * two_cycle_phase;
        let (sin, cos) = full.sin_cos();
        let (half_sin, half_cos) = half.sin_cos();
        let mut torques = [0.0; 8];
        for (index, (torque, coefficient)) in
            torques.iter_mut().zip(self.coefficients.iter()).enumerate()
        {
            let gate = if stance[index / 4] { 1.0 } else { 0.0 };
            *torque = (coefficient[0] * gate
                + coefficient[1]
                + coefficient[2] * sin
                + coefficient[3] * cos
                + coefficient[4] * half_sin
                + coefficient[5] * half_cos)
                .clamp(-40.0, 40.0);
        }
        torques
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
