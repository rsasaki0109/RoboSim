use super::UrdfJointPositionTarget;

/// Command for the official Unitree Go2 diagonal-pair trot generator.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct UnitreeGo2GaitCommand {
    /// Thigh swing amplitude in radians, clamped to `[0, 0.3]`.
    pub stride_rad: f64,
    /// Additional swing-leg calf flexion in radians, clamped to `[0, 0.4]`.
    pub foot_lift_rad: f64,
    /// Simulation steps per gait cycle, clamped to `[40, 180]`.
    pub cycle_steps: u64,
    /// Hip-abduction posture correction in radians, clamped to `[-0.8, 0.8]`
    /// (well inside the official Go2 hip range of ±1.05 rad).
    ///
    /// Applied with opposite signs on the left and right legs, so a positive value
    /// shifts the stance laterally. A balance controller feeds the measured body
    /// roll back through this term to lean the legs against a disturbance; the
    /// scripted open-loop trot leaves it at zero.
    pub roll_correction_rad: f64,
    /// Thigh-pitch posture correction in radians, clamped to `[-0.3, 0.3]`.
    ///
    /// Applied uniformly to every thigh, pitching the body against forward or
    /// backward disturbances.
    pub pitch_correction_rad: f64,
    /// Left/right differential calf extension in radians, clamped to `[-0.5, 0.5]`.
    ///
    /// A positive value straightens the left calves and flexes the right calves,
    /// so the leg-length asymmetry rolls the body toward the shorter side. This is
    /// the "push up with the downhill legs" recovery channel — measured to actuate
    /// the same lean axis as the hip correction, but independently of it, which is
    /// what lets a balance controller escape the hip-authority saturation.
    pub lateral_extension_rad: f64,
}

impl Default for UnitreeGo2GaitCommand {
    fn default() -> Self {
        Self {
            stride_rad: 0.12,
            foot_lift_rad: 0.16,
            cycle_steps: 90,
            roll_correction_rad: 0.0,
            pitch_correction_rad: 0.0,
            lateral_extension_rad: 0.0,
        }
    }
}

/// Generates one force-limited target pose for all 12 official Go2 joints.
pub fn unitree_go2_trot_targets(
    step: u64,
    command: UnitreeGo2GaitCommand,
) -> [UrdfJointPositionTarget<'static>; 12] {
    let stride = command.stride_rad.clamp(0.0, 0.3);
    let lift = command.foot_lift_rad.clamp(0.0, 0.4);
    let cycle = command.cycle_steps.clamp(40, 180);
    let roll = command.roll_correction_rad.clamp(-0.8, 0.8);
    let pitch = command.pitch_correction_rad.clamp(-0.3, 0.3);
    let extension = command.lateral_extension_rad.clamp(-0.5, 0.5);
    let phase = (step % cycle) as f64 / cycle as f64;
    let a = gait_wave(phase);
    let b = gait_wave((phase + 0.5) % 1.0);
    let leg = |prefix: &'static str, wave: (f64, f64)| {
        // The Go2 hip abduction axes are not mirrored between sides, so the same
        // joint angle on every hip shifts all four feet the same lateral direction
        // and the body leans the opposite way. (Mirrored signs would merely widen
        // the stance, which stabilizes nothing.)
        let (hip, thigh, calf, side) = match prefix {
            "FL" => ("FL_hip", "FL_thigh", "FL_calf", 1.0),
            "FR" => ("FR_hip", "FR_thigh", "FR_calf", -1.0),
            "RL" => ("RL_hip", "RL_thigh", "RL_calf", 1.0),
            _ => ("RR_hip", "RR_thigh", "RR_calf", -1.0),
        };
        [
            target(hip, roll),
            target(thigh, 0.8 + stride * wave.0 + pitch),
            target(calf, -1.5 - lift * wave.1 + side * extension),
        ]
    };
    let fl = leg("FL", a);
    let fr = leg("FR", b);
    let rl = leg("RL", b);
    let rr = leg("RR", a);
    [
        fl[0], fl[1], fl[2], fr[0], fr[1], fr[2], rl[0], rl[1], rl[2], rr[0], rr[1], rr[2],
    ]
}

/// Fourier joint-offset overlay on the official trot.
///
/// This is the search space for optimized gaits: for every joint the overlay
/// adds `c + s₁·sin(2π·p) + k₁·cos(2π·p) + s₂·sin(π·p) + k₂·cos(π·p)` to the
/// scripted trot target, where `p` is the gait phase over a **two-cycle**
/// period. The half-frequency terms deliberately break the trot's half-cycle
/// symmetry — the symmetry that cancels every hand-scripted steering pattern —
/// and the twelve joints × five coefficients give a 60-dimensional space that
/// can express per-leg phase, asymmetry, and cycle-to-cycle alternation (see
/// `docs/GO2_LOCOMOTION.md` for the measured steering boundary motivating it).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct UnitreeGo2GaitOverlay {
    /// Per-joint `[constant, sin, cos, half_sin, half_cos]` coefficients; legs
    /// ordered FL, FR, RL, RR, joints ordered hip, thigh, calf within each leg.
    pub coefficients: [[f64; 5]; 12],
}

impl UnitreeGo2GaitOverlay {
    /// The neutral overlay: reproduces the scripted trot exactly.
    pub const ZERO: Self = Self {
        coefficients: [[0.0; 5]; 12],
    };

    /// A turning gait found by deterministic cross-entropy search (seed 42,
    /// `examples/54_go2_learned_turn -- --train`) on the fast walking trot.
    ///
    /// Every hand-scripted steering mechanism measured in
    /// `docs/GO2_LOCOMOTION.md` fails to yaw this platform. This overlay is the
    /// first gait that turns it at a genuinely *sustained* rate — about
    /// 0.025 rad/s, positive across disjoint measurement windows — while
    /// staying upright; `learned_overlay_turns_the_walking_trot` pins that
    /// behavior. The magnitude is also the honest boundary: an order of
    /// magnitude short of practical steering, because additive joint offsets on
    /// a fixed contact schedule cannot re-sequence the contacts.
    pub const LEARNED_TURN: Self = Self {
        coefficients: [
            [0.129912, -0.217959, 0.002348, -0.189749, 0.064767],
            [0.071184, -0.071463, -0.212554, -0.046149, 0.032646],
            [-0.233136, -0.043006, -0.039531, 0.077401, -0.024237],
            [0.078918, -0.039693, 0.089503, -0.046611, 0.045946],
            [-0.166883, 0.103000, 0.176065, 0.094167, 0.000696],
            [0.001261, -0.017031, 0.062574, -0.052474, -0.150523],
            [-0.128368, -0.062886, 0.046139, -0.018919, 0.022867],
            [-0.020196, -0.053897, -0.047735, -0.066953, 0.051198],
            [0.008270, -0.045094, -0.144627, -0.154906, -0.014030],
            [-0.230480, 0.192577, 0.051289, -0.210476, -0.151049],
            [-0.080313, -0.050767, -0.048977, 0.080942, -0.007959],
            [0.045380, -0.051971, -0.030212, 0.129360, -0.120137],
        ],
    };

    /// Joint offsets at a two-cycle gait phase in `[0, 1)`, each clamped to
    /// ±0.5 rad so no candidate can command a joint far outside the gait's
    /// working range.
    pub fn offsets(&self, two_cycle_phase: f64) -> [f64; 12] {
        let full = 4.0 * std::f64::consts::PI * two_cycle_phase;
        let half = 2.0 * std::f64::consts::PI * two_cycle_phase;
        let (sin, cos) = full.sin_cos();
        let (half_sin, half_cos) = half.sin_cos();
        let mut offsets = [0.0; 12];
        for (offset, coefficient) in offsets.iter_mut().zip(self.coefficients.iter()) {
            *offset = (coefficient[0]
                + coefficient[1] * sin
                + coefficient[2] * cos
                + coefficient[3] * half_sin
                + coefficient[4] * half_cos)
                .clamp(-0.5, 0.5);
        }
        offsets
    }
}

/// The official trot with a Fourier overlay added joint by joint.
pub fn unitree_go2_trot_targets_with_overlay(
    step: u64,
    command: UnitreeGo2GaitCommand,
    overlay: &UnitreeGo2GaitOverlay,
) -> [UrdfJointPositionTarget<'static>; 12] {
    let mut targets = unitree_go2_trot_targets(step, command);
    let cycle = command.cycle_steps.clamp(40, 180);
    let two_cycle_phase = (step % (2 * cycle)) as f64 / (2 * cycle) as f64;
    let offsets = overlay.offsets(two_cycle_phase);
    for (target, offset) in targets.iter_mut().zip(offsets.iter()) {
        target.position += offset;
    }
    targets
}

fn gait_wave(phase: f64) -> (f64, f64) {
    if phase < 0.7 {
        (1.0 - 2.0 * phase / 0.7, 0.0)
    } else {
        let swing = (phase - 0.7) / 0.3;
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
    fn zero_overlay_reproduces_the_scripted_trot() {
        let command = UnitreeGo2GaitCommand::default();
        for step in [0, 17, 45, 89] {
            assert_eq!(
                unitree_go2_trot_targets(step, command),
                unitree_go2_trot_targets_with_overlay(step, command, &UnitreeGo2GaitOverlay::ZERO)
            );
        }
    }

    #[test]
    fn overlay_offsets_are_clamped() {
        let overlay = UnitreeGo2GaitOverlay {
            coefficients: [[1.0, 1.0, 1.0, 1.0, 1.0]; 12],
        };
        for offset in overlay.offsets(0.13) {
            assert!(offset.abs() <= 0.5);
        }
    }

    #[test]
    fn official_go2_trot_is_periodic_and_bounded() {
        let command = UnitreeGo2GaitCommand::default();
        assert_eq!(
            unitree_go2_trot_targets(0, command),
            unitree_go2_trot_targets(command.cycle_steps, command)
        );
        for target in unitree_go2_trot_targets(30, command) {
            assert!(target.position.is_finite());
            assert!(target.position.abs() <= 2.0);
        }
    }
}
