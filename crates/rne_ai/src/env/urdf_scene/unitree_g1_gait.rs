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
    /// 0.19 m per 8 s window (over 2× the stepper), 0.55 m per 24 s, at full
    /// height (0.785 m) and dead straight, with ulp-perturbed replays inside
    /// a few centimeters of each other. The learned search winner is scaled
    /// to 60% so the same gait remains inside the stable contact basin across
    /// physics code-generation targets. Pinned at the search state's
    /// 12-decimal precision per the chaos discipline;
    /// `learned_torques_make_the_g1_stride` pins the comparison.
    pub const LEARNED_STRIDE: Self = Self {
        coefficients: [
            [
                2.183423847375,
                1.099672694429,
                0.713601360280,
                7.601773621958,
                0.971313554558,
                -0.886237577470,
            ],
            [
                -2.242321363135,
                -0.132917247534,
                3.460676862800,
                -4.579411590525,
                -1.590537671801,
                0.945705619594,
            ],
            [
                0.864093192051,
                1.820514203380,
                -2.210321453230,
                -11.458286893298,
                0.071326415395,
                -0.348851193974,
            ],
            [
                -1.997961869730,
                -5.315884397976,
                -3.411052114382,
                -0.209928893263,
                -0.277245911936,
                2.090928042845,
            ],
            [
                -1.277826853522,
                -1.403044591888,
                -0.436806343376,
                -0.843403703612,
                0.938346354383,
                1.385294109821,
            ],
            [
                0.579345691136,
                -1.924126763206,
                1.850393163574,
                -1.395903469306,
                -0.066748652452,
                -0.354893563999,
            ],
            [
                -0.994546454779,
                0.200918572812,
                -12.000000000000,
                -1.165447330229,
                -2.391622558308,
                1.851324880236,
            ],
            [
                -0.955898725388,
                -8.353438371270,
                -1.676198157251,
                2.073778680562,
                -0.814283496586,
                0.310121189583,
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
