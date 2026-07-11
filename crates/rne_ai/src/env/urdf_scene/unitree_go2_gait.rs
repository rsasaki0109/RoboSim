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
}

impl Default for UnitreeGo2GaitCommand {
    fn default() -> Self {
        Self {
            stride_rad: 0.12,
            foot_lift_rad: 0.16,
            cycle_steps: 90,
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
    let phase = (step % cycle) as f64 / cycle as f64;
    let a = gait_wave(phase);
    let b = gait_wave((phase + 0.5) % 1.0);
    let leg = |prefix: &'static str, wave: (f64, f64)| {
        let (hip, thigh, calf) = match prefix {
            "FL" => ("FL_hip", "FL_thigh", "FL_calf"),
            "FR" => ("FR_hip", "FR_thigh", "FR_calf"),
            "RL" => ("RL_hip", "RL_thigh", "RL_calf"),
            _ => ("RR_hip", "RR_thigh", "RR_calf"),
        };
        [
            target(hip, 0.0),
            target(thigh, 0.8 + stride * wave.0),
            target(calf, -1.5 - lift * wave.1),
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
