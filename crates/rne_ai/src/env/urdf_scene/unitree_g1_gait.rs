use super::UrdfJointPositionTarget;
use crate::LocomotionPolicy;

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

/// Typed observation consumed by the G1 feed-forward torque policy.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct UnitreeG1TorquePolicyInput {
    /// Continuous normalized phase over the two-cycle gait period.
    pub two_cycle_phase: f64,
    /// Measured left and right foot stance contacts.
    pub stance: [bool; 2],
}

/// Forward-speed and signed differential-steering command for the G1
/// locomotion policy.
///
/// The v0.1 G1 command contract supports forward motion, stopping, and
/// differential steering. `yaw_rate_rad_s` is retained as the conventional
/// command name and is also exposed to the policy as a yaw-rate request. The
/// v0.2 headless harness additionally evaluates it against a bounded body-
/// heading reference; the wider command range remains an input envelope, not
/// a promise of sustained heading tracking.
/// Reverse walking is deliberately outside this first contract because the
/// current gait generator has no validated backward contact schedule; negative
/// forward inputs clamp to zero.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct UnitreeG1VelocityCommand {
    /// Desired body-forward speed in m/s, clamped to `[0, 0.06]`.
    pub forward_m_s: f64,
    /// Desired yaw-rate/differential-steering request in rad/s, clamped to
    /// `[-0.35, 0.35]`.
    pub yaw_rate_rad_s: f64,
}

impl UnitreeG1VelocityCommand {
    /// Returns a finite command inside the validated v0.1 operating envelope.
    pub fn clamped(self) -> Self {
        Self {
            forward_m_s: if self.forward_m_s.is_finite() {
                self.forward_m_s.clamp(0.0, 0.06)
            } else {
                0.0
            },
            yaw_rate_rad_s: if self.yaw_rate_rad_s.is_finite() {
                self.yaw_rate_rad_s.clamp(-0.35, 0.35)
            } else {
                0.0
            },
        }
    }
}

/// State and command input for a command-conditioned G1 torque policy.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct UnitreeG1VelocityPolicyInput {
    /// Continuous phase over the two-cycle gait period in `[0, 1)`.
    pub two_cycle_phase: f64,
    /// Measured left and right foot contacts.
    pub stance: [bool; 2],
    /// Requested forward speed and signed differential-steering rate.
    pub command: UnitreeG1VelocityCommand,
    /// Measured body-forward velocity in m/s.
    pub measured_forward_velocity_m_s: f64,
    /// Measured body yaw rate in rad/s; this remains a diagnostic feedback
    /// channel even when the command is being used for path steering.
    pub measured_yaw_rate_rad_s: f64,
    /// Desired accumulated body heading at the current tick in radians.
    pub target_heading_rad: f64,
    /// Unwrapped accumulated body heading measured from rollout start.
    pub measured_heading_rad: f64,
    /// Heading tracking error `target_heading_rad - measured_heading_rad`.
    pub heading_error_rad: f64,
    /// Yaw-rate tracking error in rad/s.
    pub yaw_rate_error_rad_s: f64,
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
    /// 0.22 m per 8 s window (over 2× the stepper), 0.66 m per 24 s, at full
    /// height (0.784 m) and dead straight, with ulp-perturbed replays inside
    /// a few centimeters of each other. The speed-envelope sweep uses the
    /// learned search winner at 66% feed-forward strength with a 0.065 rad,
    /// 0.12 rad, 100-step gait command. Pinned at the search state's
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

    /// A pinned differential-steering overlay found by example 67's seeded
    /// CEM (seed `0x6701`, two generations, six candidates, and three replay
    /// members per score). It is intentionally separate from
    /// [`Self::LEARNED_STRIDE`]: the objective rewards opposite signed
    /// left/right path displacement while preserving uprightness, not body
    /// heading rotation. At the v0.1 `±0.05 rad/s` command this overlay is
    /// exercised through [`UnitreeG1CommandedTorquePolicy`]'s optional yaw
    /// overlay channel.
    pub const LEARNED_DIFFERENTIAL_STEERING: Self = Self {
        coefficients: [
            [
                -3.340066033819,
                0.893385596864,
                -0.395736721429,
                0.0,
                0.0,
                0.0,
            ],
            [
                -2.352076784768,
                -3.427014240405,
                0.339598669207,
                0.0,
                0.0,
                0.0,
            ],
            [
                1.320348081646,
                5.172334831800,
                -1.433905553678,
                0.0,
                0.0,
                0.0,
            ],
            [
                1.798281174386,
                -1.151498840654,
                -0.776769229652,
                0.0,
                0.0,
                0.0,
            ],
            [
                -3.244015754174,
                -0.407426467646,
                -1.646827542357,
                0.0,
                0.0,
                0.0,
            ],
            [
                -1.314956307119,
                2.595673995971,
                3.672827665542,
                0.0,
                0.0,
                0.0,
            ],
            [
                1.024295600454,
                1.560699382588,
                4.351233171055,
                0.0,
                0.0,
                0.0,
            ],
            [
                2.175154026608,
                -1.127685401140,
                -4.432681536334,
                0.0,
                0.0,
                0.0,
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

    /// Returns the sagittally mirrored overlay, swapping legs and reversing
    /// roll/yaw joint signs while preserving pitch and knee signs.
    pub fn mirrored_sagittally(&self) -> Self {
        const PAIR_SIGN: [[f64; 2]; 4] = [[1.0, 1.0], [-1.0, -1.0], [-1.0, -1.0], [1.0, 1.0]];
        let mut coefficients = [[0.0; 6]; 8];
        for (pair, signs) in PAIR_SIGN.iter().copied().enumerate() {
            let left = pair;
            let right = pair + 4;
            let left_source = self.coefficients[left];
            let right_source = self.coefficients[right];
            let mut left_output = [0.0; 6];
            let mut right_output = [0.0; 6];
            for (term, (left_value, right_value)) in left_output
                .iter_mut()
                .zip(right_output.iter_mut())
                .enumerate()
            {
                *left_value = signs[0] * right_source[term];
                *right_value = signs[1] * left_source[term];
            }
            coefficients[left] = left_output;
            coefficients[right] = right_output;
        }
        Self { coefficients }
    }
}

/// Command-conditioned hybrid torque policy for the official G1.
///
/// The policy preserves the validated learned overlay as its nominal forward
/// gait, scales its phase action from the requested/measured forward speed,
/// and adds bounded differential channels from the steering/yaw-rate and
/// accumulated-heading errors. Ankles and arms remain position-held by the
/// hybrid rollout. The optional stance gate routes the direct yaw correction
/// through the supporting leg while the learned overlay remains contact-gated
/// for every proximal joint.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct UnitreeG1CommandedTorquePolicy {
    /// Nominal phase-overlay gait.
    pub overlay: UnitreeG1TorqueOverlay,
    /// Speed represented by one full-strength overlay in m/s.
    pub nominal_forward_velocity_m_s: f64,
    /// Gain on forward-speed error used to scale the phase action.
    pub forward_velocity_feedback_gain: f64,
    /// Hip-yaw torque gain on yaw-rate error in N·m/(rad/s).
    pub yaw_rate_kp_nm_per_rad_s: f64,
    /// Differential hip-yaw torque gain on accumulated heading error in
    /// N·m/rad.
    pub heading_kp_nm_per_rad: f64,
    /// Maximum differential yaw torque in N·m.
    pub max_yaw_torque_nm: f64,
    /// Maximum differential heading-correction torque in N·m.
    pub max_heading_torque_nm: f64,
    /// Routes the commanded yaw torque through the currently supporting leg;
    /// swing-leg yaw torque is suppressed when enabled.
    pub stance_gated_yaw: bool,
    /// Differential hip-pitch torque gain for yaw-rate commands in N·m/(rad/s).
    pub yaw_hip_pitch_kp_nm_per_rad_s: f64,
    /// Differential hip-roll torque gain for yaw-rate commands in N·m/(rad/s).
    pub yaw_hip_roll_kp_nm_per_rad_s: f64,
    /// Differential knee torque gain for yaw-rate commands in N·m/(rad/s).
    pub yaw_knee_kp_nm_per_rad_s: f64,
    /// Per-proximal-joint yaw-rate error torque gains in N·m/(rad/s).
    ///
    /// The array follows the same left-leg-then-right-leg order as
    /// [`UnitreeG1TorqueOverlay::coefficients`]. It is the low-dimensional
    /// command channel that a candidate search may use when hip-yaw authority
    /// alone is insufficient to change the contact-generated turn.
    pub yaw_rate_torque_weights_nm_per_rad_s: [f64; 8],
    /// Sign applied to the right hip-yaw torque; `-1` is differential.
    pub yaw_torque_right_sign: f64,
    /// Optional command-scaled Fourier overlay used by a turn candidate.
    pub yaw_overlay: UnitreeG1TorqueOverlay,
    /// Gain multiplying `yaw_rate_rad_s * yaw_overlay`.
    pub yaw_overlay_gain: f64,
    /// Per-joint multipliers for the optional yaw overlay.
    pub yaw_overlay_joint_gains: [f64; 8],
    /// Mirrors the optional differential overlay torque for negative yaw
    /// commands. A heading candidate may disable this when its coefficients
    /// are already sign-conditioned by the commanded yaw-rate error.
    pub mirror_yaw_overlay_negative: bool,
}

impl Default for UnitreeG1CommandedTorquePolicy {
    fn default() -> Self {
        Self {
            overlay: UnitreeG1TorqueOverlay::LEARNED_STRIDE,
            nominal_forward_velocity_m_s: 0.0276,
            forward_velocity_feedback_gain: 0.25,
            yaw_rate_kp_nm_per_rad_s: 16.0,
            heading_kp_nm_per_rad: 0.0,
            max_yaw_torque_nm: 8.0,
            max_heading_torque_nm: 8.0,
            stance_gated_yaw: false,
            yaw_hip_pitch_kp_nm_per_rad_s: 0.0,
            yaw_hip_roll_kp_nm_per_rad_s: 0.0,
            yaw_knee_kp_nm_per_rad_s: 0.0,
            yaw_rate_torque_weights_nm_per_rad_s: [0.0; 8],
            yaw_torque_right_sign: -1.0,
            yaw_overlay: UnitreeG1TorqueOverlay::ZERO,
            yaw_overlay_gain: 1.0,
            yaw_overlay_joint_gains: [1.0; 8],
            mirror_yaw_overlay_negative: true,
        }
    }
}

impl UnitreeG1CommandedTorquePolicy {
    /// Computes eight proximal-joint feed-forward torques for one command.
    ///
    /// A zero forward command suppresses the learned phase action; callers
    /// should pair it with [`unitree_g1_gait_targets_for_velocity`] so the
    /// position-held joints also move to the standing pose. Every output is
    /// finite and clamped to `limit_nm` when that limit is finite and
    /// non-negative.
    pub fn torques_nm_for_command(
        &self,
        input: UnitreeG1VelocityPolicyInput,
        limit_nm: f64,
    ) -> [f64; 8] {
        let command = input.command.clamped();
        let nominal = if self.nominal_forward_velocity_m_s.is_finite()
            && self.nominal_forward_velocity_m_s > 0.0
        {
            self.nominal_forward_velocity_m_s
        } else {
            0.0276
        };
        let measured_forward_m_s = if input.measured_forward_velocity_m_s.is_finite() {
            input.measured_forward_velocity_m_s
        } else {
            0.0
        };
        let feedback_gain = if self.forward_velocity_feedback_gain.is_finite() {
            self.forward_velocity_feedback_gain.max(0.0)
        } else {
            0.0
        };
        let desired_magnitude = command.forward_m_s;
        let phase_scale = if desired_magnitude < 0.001 {
            0.0
        } else {
            let error_m_s = desired_magnitude - measured_forward_m_s;
            (desired_magnitude + feedback_gain * error_m_s)
                .max(0.0)
                .div_euclid(nominal)
                .clamp(0.0, 2.0)
        };
        let phase = if input.two_cycle_phase.is_finite() {
            input.two_cycle_phase.rem_euclid(1.0)
        } else {
            0.0
        };
        let phase_overlay = if command.yaw_rate_rad_s < 0.0 {
            self.overlay.mirrored_sagittally()
        } else {
            self.overlay
        };
        let mut torques = phase_overlay
            .torques_nm(phase, input.stance)
            .map(|torque| torque * phase_scale);
        let measured_yaw_rate_rad_s = if input.measured_yaw_rate_rad_s.is_finite() {
            input.measured_yaw_rate_rad_s
        } else {
            0.0
        };
        let yaw_gain = if self.yaw_rate_kp_nm_per_rad_s.is_finite() {
            self.yaw_rate_kp_nm_per_rad_s.max(0.0)
        } else {
            0.0
        };
        let yaw_limit = if self.max_yaw_torque_nm.is_finite() {
            self.max_yaw_torque_nm.max(0.0)
        } else {
            0.0
        };
        let yaw_rate_error = if input.yaw_rate_error_rad_s.is_finite() {
            input.yaw_rate_error_rad_s
        } else {
            command.yaw_rate_rad_s - measured_yaw_rate_rad_s
        };
        let yaw_torque = (yaw_gain * yaw_rate_error).clamp(-yaw_limit, yaw_limit);
        let heading_gain = if self.heading_kp_nm_per_rad.is_finite() {
            self.heading_kp_nm_per_rad.max(0.0)
        } else {
            0.0
        };
        let heading_limit = if self.max_heading_torque_nm.is_finite() {
            self.max_heading_torque_nm.max(0.0)
        } else {
            0.0
        };
        let heading_error = if input.heading_error_rad.is_finite() {
            input.heading_error_rad.clamp(-1.5, 1.5)
        } else {
            0.0
        };
        let heading_torque = (heading_gain * heading_error).clamp(-heading_limit, heading_limit);
        let yaw_rate = command.yaw_rate_rad_s;
        let yaw_error = command.yaw_rate_rad_s - measured_yaw_rate_rad_s;
        let hip_pitch_gain = if self.yaw_hip_pitch_kp_nm_per_rad_s.is_finite() {
            self.yaw_hip_pitch_kp_nm_per_rad_s
        } else {
            0.0
        };
        let hip_roll_gain = if self.yaw_hip_roll_kp_nm_per_rad_s.is_finite() {
            self.yaw_hip_roll_kp_nm_per_rad_s
        } else {
            0.0
        };
        let knee_gain = if self.yaw_knee_kp_nm_per_rad_s.is_finite() {
            self.yaw_knee_kp_nm_per_rad_s
        } else {
            0.0
        };
        // The differential channels are deliberately antisymmetric: they
        // bias the two legs in opposite directions without adding a common
        // lateral or vertical command.
        torques[0] += hip_pitch_gain * yaw_rate;
        torques[4] -= hip_pitch_gain * yaw_rate;
        torques[1] += hip_roll_gain * yaw_rate;
        torques[5] -= hip_roll_gain * yaw_rate;
        torques[3] += knee_gain * yaw_rate;
        torques[7] -= knee_gain * yaw_rate;
        for (torque, weight) in torques
            .iter_mut()
            .zip(self.yaw_rate_torque_weights_nm_per_rad_s)
        {
            *torque += if weight.is_finite() {
                weight * yaw_error
            } else {
                0.0
            };
        }
        let overlay_gain = if self.yaw_overlay_gain.is_finite() {
            self.yaw_overlay_gain.clamp(-8.0, 8.0)
        } else {
            0.0
        };
        let turn_overlay = if self.mirror_yaw_overlay_negative && command.yaw_rate_rad_s < 0.0 {
            self.yaw_overlay.mirrored_sagittally()
        } else {
            self.yaw_overlay
        };
        for (index, (torque, turn_torque)) in torques
            .iter_mut()
            .zip(turn_overlay.torques_nm(phase, input.stance))
            .enumerate()
        {
            let joint_gain = self.yaw_overlay_joint_gains[index];
            *torque += if joint_gain.is_finite() {
                overlay_gain * yaw_rate * joint_gain * turn_torque
            } else {
                0.0
            };
        }
        // Hip-yaw joints are indices 2 and 6 in the overlay order. The
        // opposite signs create a differential yaw couple while preserving
        // the nominal forward overlay on the other six proximal joints.
        let right_sign = if self.yaw_torque_right_sign.is_finite() {
            self.yaw_torque_right_sign.clamp(-1.0, 1.0)
        } else {
            -1.0
        };
        let mut left_yaw_torque = yaw_torque + heading_torque;
        let mut right_yaw_torque = right_sign * (yaw_torque + heading_torque);
        if self.stance_gated_yaw {
            match input.stance {
                [true, false] => right_yaw_torque = 0.0,
                [false, true] => left_yaw_torque = 0.0,
                [false, false] => {
                    left_yaw_torque = 0.0;
                    right_yaw_torque = 0.0;
                }
                [true, true] => {}
            }
        }
        torques[2] += left_yaw_torque;
        torques[6] += right_yaw_torque;
        let limit = if limit_nm.is_finite() && limit_nm >= 0.0 {
            limit_nm
        } else {
            f64::INFINITY
        };
        for torque in &mut torques {
            *torque = torque.clamp(-limit, limit);
        }
        torques
    }
}

impl LocomotionPolicy for UnitreeG1CommandedTorquePolicy {
    type Observation = UnitreeG1VelocityPolicyInput;
    type Action = [f64; 8];

    fn act(&mut self, observation: &Self::Observation) -> Self::Action {
        self.torques_nm_for_command(*observation, 88.0)
    }
}

/// Converts a velocity command into the validated hybrid G1 gait targets.
///
/// The base command supplies the nominal stride/lift/cycle. Forward speed
/// scales stride and lift together, zero speed returns the standing pose, and
/// the signed steering request applies a bounded differential hip-yaw target.
/// The returned array retains the same 23-joint order as
/// [`unitree_g1_gait_targets`].
pub fn unitree_g1_gait_targets_for_velocity(
    step: u64,
    base_command: UnitreeG1GaitCommand,
    command: UnitreeG1VelocityCommand,
) -> [UrdfJointPositionTarget<'static>; 23] {
    unitree_g1_gait_targets_for_velocity_with_yaw_stride(step, base_command, command, 0.0)
}

/// Converts a velocity command into hybrid G1 targets with an optional
/// differential stride channel.
///
/// `yaw_stride_scale_per_rad_s` changes the left/right dynamic leg excursion
/// in opposite directions. A positive value makes the right leg's excursion
/// larger and the left leg's excursion smaller for a positive yaw command.
/// The parameter is bounded to keep both legs inside the validated gait pose
/// envelope; zero reproduces [`unitree_g1_gait_targets_for_velocity`].
pub fn unitree_g1_gait_targets_for_velocity_with_yaw_stride(
    step: u64,
    base_command: UnitreeG1GaitCommand,
    command: UnitreeG1VelocityCommand,
    yaw_stride_scale_per_rad_s: f64,
) -> [UrdfJointPositionTarget<'static>; 23] {
    unitree_g1_gait_targets_for_velocity_with_yaw_stride_phase(
        step,
        base_command,
        command,
        yaw_stride_scale_per_rad_s,
        0.0,
    )
}

/// Converts a velocity command into hybrid G1 targets with differential stride
/// and a bounded right-leg phase offset.
///
/// The phase gain is expressed in seconds per rad/s. It shifts the right leg
/// by a whole simulation-tick amount, preserving deterministic contact timing
/// rather than interpolating joint positions.
pub fn unitree_g1_gait_targets_for_velocity_with_yaw_stride_phase(
    step: u64,
    base_command: UnitreeG1GaitCommand,
    command: UnitreeG1VelocityCommand,
    yaw_stride_scale_per_rad_s: f64,
    yaw_phase_offset_s_per_rad_s: f64,
) -> [UrdfJointPositionTarget<'static>; 23] {
    let command = command.clamped();
    let nominal = 0.0276;
    let speed_scale = (command.forward_m_s / nominal).clamp(0.0, 1.6);
    let stride = base_command.stride_rad.clamp(0.0, 0.35) * speed_scale;
    let lift = base_command.foot_lift_rad.clamp(0.0, 0.45) * speed_scale;
    let gait_command = UnitreeG1GaitCommand {
        stride_rad: stride,
        foot_lift_rad: lift,
        cycle_steps: base_command.cycle_steps,
    };
    let mut targets = unitree_g1_gait_targets(step, gait_command);
    let cycle = gait_command.cycle_steps.clamp(40, 180);
    let phase_gain = if yaw_phase_offset_s_per_rad_s.is_finite() {
        yaw_phase_offset_s_per_rad_s.clamp(-10.0, 10.0)
    } else {
        0.0
    };
    let phase_offset_steps = (phase_gain * command.yaw_rate_rad_s * cycle as f64).round() as i64;
    if phase_offset_steps != 0 {
        let shifted_step = if phase_offset_steps.is_positive() {
            step.wrapping_add(phase_offset_steps as u64)
        } else {
            step.wrapping_sub(phase_offset_steps.unsigned_abs())
        };
        let shifted = unitree_g1_gait_targets(shifted_step, gait_command);
        for index in [6, 7, 8, 9, 10, 11, 18] {
            targets[index].position = shifted[index].position;
        }
    }
    let stride_gain = if yaw_stride_scale_per_rad_s.is_finite() {
        yaw_stride_scale_per_rad_s.clamp(-4.0, 4.0)
    } else {
        0.0
    };
    let differential = (stride_gain * command.yaw_rate_rad_s).clamp(-0.75, 0.75);
    let left_scale = 1.0 - differential;
    let right_scale = 1.0 + differential;
    scale_dynamic_target(&mut targets[0], -0.18, left_scale);
    scale_dynamic_target(&mut targets[3], 0.36, left_scale);
    scale_dynamic_target(&mut targets[4], -0.18, left_scale);
    scale_dynamic_target(&mut targets[13], 0.0, left_scale);
    scale_dynamic_target(&mut targets[6], -0.18, right_scale);
    scale_dynamic_target(&mut targets[9], 0.36, right_scale);
    scale_dynamic_target(&mut targets[10], -0.18, right_scale);
    scale_dynamic_target(&mut targets[18], 0.0, right_scale);
    let yaw_target_rad = (0.45 * command.yaw_rate_rad_s).clamp(-0.20, 0.20);
    targets[2].position += yaw_target_rad;
    targets[8].position -= yaw_target_rad;
    targets
}

fn scale_dynamic_target(target: &mut UrdfJointPositionTarget<'static>, neutral: f64, scale: f64) {
    target.position = neutral + (target.position - neutral) * scale;
}

impl LocomotionPolicy for UnitreeG1TorqueOverlay {
    type Observation = UnitreeG1TorquePolicyInput;
    type Action = [f64; 8];

    fn act(&mut self, observation: &Self::Observation) -> Self::Action {
        self.torques_nm(observation.two_cycle_phase, observation.stance)
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

    #[test]
    fn velocity_command_clamps_to_the_v01_envelope() {
        let command = UnitreeG1VelocityCommand {
            forward_m_s: f64::NAN,
            yaw_rate_rad_s: f64::INFINITY,
        }
        .clamped();
        assert_eq!(command, UnitreeG1VelocityCommand::default());

        let command = UnitreeG1VelocityCommand {
            forward_m_s: -1.0,
            yaw_rate_rad_s: 1.0,
        }
        .clamped();
        assert_eq!(command.forward_m_s, 0.0);
        assert_eq!(command.yaw_rate_rad_s, 0.35);
    }

    #[test]
    fn commanded_policy_is_finite_limited_and_stops_phase_torque() {
        let mut policy = UnitreeG1CommandedTorquePolicy::default();
        let input = UnitreeG1VelocityPolicyInput {
            two_cycle_phase: 0.25,
            stance: [true, false],
            command: UnitreeG1VelocityCommand::default(),
            measured_forward_velocity_m_s: 0.0,
            measured_yaw_rate_rad_s: 0.0,
            target_heading_rad: 0.0,
            measured_heading_rad: 0.0,
            heading_error_rad: 0.0,
            yaw_rate_error_rad_s: 0.0,
        };
        let stopped = policy.act(&input);
        assert!(stopped.iter().all(|value| value.is_finite()));
        assert!(stopped.iter().all(|value| value.abs() < 1.0e-12));

        let moving = UnitreeG1VelocityPolicyInput {
            command: UnitreeG1VelocityCommand {
                forward_m_s: 0.0276,
                yaw_rate_rad_s: 0.35,
            },
            ..input
        };
        let torques = policy.act(&moving);
        assert!(torques.iter().all(|value| value.is_finite()));
        assert!(torques.iter().all(|value| value.abs() <= 88.0));
        assert_ne!(torques, stopped);
    }

    #[test]
    fn heading_torque_is_differential_and_can_be_stance_gated() {
        let policy = UnitreeG1CommandedTorquePolicy {
            overlay: UnitreeG1TorqueOverlay::ZERO,
            yaw_rate_kp_nm_per_rad_s: 1.0,
            heading_kp_nm_per_rad: 1.0,
            max_yaw_torque_nm: 10.0,
            max_heading_torque_nm: 10.0,
            stance_gated_yaw: true,
            ..UnitreeG1CommandedTorquePolicy::default()
        };
        let input = UnitreeG1VelocityPolicyInput {
            command: UnitreeG1VelocityCommand::default(),
            heading_error_rad: 0.2,
            yaw_rate_error_rad_s: 0.1,
            stance: [true, false],
            ..UnitreeG1VelocityPolicyInput {
                two_cycle_phase: 0.0,
                stance: [false, false],
                command: UnitreeG1VelocityCommand::default(),
                measured_forward_velocity_m_s: 0.0,
                measured_yaw_rate_rad_s: 0.0,
                target_heading_rad: 0.0,
                measured_heading_rad: 0.0,
                heading_error_rad: 0.0,
                yaw_rate_error_rad_s: 0.0,
            }
        };
        let left_stance = policy.torques_nm_for_command(input, 88.0);
        assert!((left_stance[2] - 0.3).abs() < 1.0e-12);
        assert_eq!(left_stance[6], 0.0);

        let right_stance = policy.torques_nm_for_command(
            UnitreeG1VelocityPolicyInput {
                stance: [false, true],
                ..input
            },
            88.0,
        );
        assert_eq!(right_stance[2], 0.0);
        assert!((right_stance[6] + 0.3).abs() < 1.0e-12);
    }

    #[test]
    fn pinned_steering_overlay_is_finite_and_mirror_involutive() {
        let overlay = UnitreeG1TorqueOverlay::LEARNED_DIFFERENTIAL_STEERING;
        assert!(overlay
            .coefficients
            .iter()
            .flatten()
            .all(|value| value.is_finite()));
        assert_eq!(overlay.mirrored_sagittally().mirrored_sagittally(), overlay);
    }

    #[test]
    fn velocity_targets_stop_and_turn_differentially() {
        let base = UnitreeG1GaitCommand {
            stride_rad: 0.065,
            foot_lift_rad: 0.12,
            cycle_steps: 100,
        };
        let stopped =
            unitree_g1_gait_targets_for_velocity(10, base, UnitreeG1VelocityCommand::default());
        assert_eq!(stopped[0].position, -0.18);
        assert_eq!(stopped[3].position, 0.36);
        let turned = unitree_g1_gait_targets_for_velocity(
            10,
            base,
            UnitreeG1VelocityCommand {
                forward_m_s: 0.0276,
                yaw_rate_rad_s: 0.2,
            },
        );
        assert!(turned[2].position > turned[8].position);

        let differential = unitree_g1_gait_targets_for_velocity_with_yaw_stride(
            10,
            base,
            UnitreeG1VelocityCommand {
                forward_m_s: 0.0276,
                yaw_rate_rad_s: 0.2,
            },
            2.0,
        );
        assert!(differential[0].position < turned[0].position);
        assert!(differential[6].position < turned[6].position);
    }
}
