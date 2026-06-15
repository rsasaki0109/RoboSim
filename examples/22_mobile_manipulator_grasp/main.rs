//! Parallel-jaw gripper contact smoke against a scene obstacle cube.

use rne_ai::{
    finger_contacts_named, mm_minimal_grasp_scene_path, MobileManipulatorAction,
    MobileManipulatorSim,
};

const CLOSE_STEPS: usize = 360;
const GRIPPER_CLOSE_RAD_S: f64 = -2.5;

fn run_grasp(sim: &mut MobileManipulatorSim) -> bool {
    let close = MobileManipulatorAction {
        gripper_velocity_rad_s: GRIPPER_CLOSE_RAD_S,
        ..MobileManipulatorAction::default()
    };

    for _ in 0..CLOSE_STEPS {
        sim.step(close);
        if finger_contacts_named(sim, "grasp_cube") {
            return true;
        }
    }
    false
}

fn main() {
    let smoke = std::env::args().any(|arg| arg == "--smoke");
    let scene_path = mm_minimal_grasp_scene_path();
    let mut sim =
        MobileManipulatorSim::from_scene_path(&scene_path).expect("load mm_minimal grasp scene");

    if smoke {
        if run_grasp(&mut sim) {
            let obs = sim.observe();
            println!(
                "smoke ok: finger contact with grasp_cube (gripper={:.3} rad, joints={})",
                obs.gripper_position_rad, obs.joint_state_count
            );
            return;
        }
        eprintln!("smoke failed: no finger contact with grasp_cube while closing gripper");
        std::process::exit(1);
    }

    let contacted = run_grasp(&mut sim);
    let obs = sim.observe();
    println!("grasp contact = {contacted}");
    println!(
        "gripper = {:.3} rad, joints published = {}",
        obs.gripper_position_rad, obs.joint_state_count
    );
}
