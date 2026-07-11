use super::UrdfJointPositionTarget;

/// Foot link names in deterministic front-left, front-right, rear-left,
/// rear-right order.
pub const QUADRUPED_FOOT_LINKS: [&str; 4] = ["fl_foot", "fr_foot", "rl_foot", "rr_foot"];

/// Returns one tick of a deterministic diagonal-pair quadruped trot.
///
/// The stance portion sweeps each hip pitch slowly backward with a straight
/// knee; the shorter swing portion returns it forward with a flexed knee.
/// Front-left/rear-right and front-right/rear-left are half a cycle apart.
pub fn quadruped_trot_targets(step: u64) -> [UrdfJointPositionTarget<'static>; 12] {
    let phase = (step % 90) as f64 / 90.0;
    let leg_target = |offset: f64| {
        let p = (phase + offset).fract();
        if p < 0.8 {
            (-0.45 + 0.9 * (p / 0.8), 0.0)
        } else {
            (0.45 - 0.9 * ((p - 0.8) / 0.2), 0.8)
        }
    };
    let a = leg_target(0.0);
    let b = leg_target(0.5);
    [
        target("fl_hip", 0.0),
        target("fl_thigh", a.0),
        target("fl_foot", a.1),
        target("fr_hip", 0.0),
        target("fr_thigh", b.0),
        target("fr_foot", b.1),
        target("rl_hip", 0.0),
        target("rl_thigh", b.0),
        target("rl_foot", b.1),
        target("rr_hip", 0.0),
        target("rr_thigh", a.0),
        target("rr_foot", a.1),
    ]
}

fn target(link_name: &'static str, position: f64) -> UrdfJointPositionTarget<'static> {
    UrdfJointPositionTarget {
        link_name,
        position,
    }
}
