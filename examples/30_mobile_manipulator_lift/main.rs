//! Vertical lift demo on the `mm_lift` robot: a prismatic "torso" lift raises and
//! lowers the whole SCARA arm against gravity.
//!
//! The lift is a position (spring-damper) motor (see `rne_physics::JointMotor`
//! `stiffness`/`target_position`), so it holds the ~6 kg arm at a commanded height
//! without drift — a plain velocity motor sagged or oscillated instead, which is
//! why vertical lifting was previously infeasible.

use rne_ai::{MobileManipulatorAction, MobileManipulatorSim};

const SETTLE_STEPS: usize = 120;
const LIFT_STEPS: usize = 180;
const AVG_STEPS: usize = 30;
const LIFT_VELOCITY_M_S: f64 = 0.3;
/// Minimum end-effector rise to count the lift as working.
const MIN_RISE_M: f64 = 0.15;

/// Steps `count` times with `action`, returning the mean end-effector height over
/// the final `AVG_STEPS` steps (smooths the arm's settling transient).
fn mean_ee_height(
    sim: &mut MobileManipulatorSim,
    action: MobileManipulatorAction,
    count: usize,
) -> f64 {
    let mut sum = 0.0;
    for step in 0..count {
        let obs = sim.step(action);
        if step >= count - AVG_STEPS {
            sum += obs.ee_y_m;
        }
    }
    sum / AVG_STEPS as f64
}

fn main() {
    let smoke = std::env::args().any(|arg| arg == "--smoke");

    let mut sim = MobileManipulatorSim::new_mm_lift();
    let up = MobileManipulatorAction {
        lift_velocity_m_s: LIFT_VELOCITY_M_S,
        ..MobileManipulatorAction::default()
    };

    // Let the arm settle on the lift, then raise it.
    let baseline = mean_ee_height(&mut sim, MobileManipulatorAction::default(), SETTLE_STEPS);
    let raised = mean_ee_height(&mut sim, up, LIFT_STEPS);
    let rise = raised - baseline;

    if smoke {
        if rise > MIN_RISE_M {
            println!(
                "lift smoke ok: end-effector raised {rise:.3} m ({baseline:.3} -> {raised:.3})"
            );
            return;
        }
        eprintln!("smoke failed: lift raised the arm only {rise:.3} m (< {MIN_RISE_M} m)");
        std::process::exit(1);
    }

    println!("vertical lift: baseline={baseline:.3} m raised={raised:.3} m rise={rise:.3} m");
}
