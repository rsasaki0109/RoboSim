//! Grasp a dynamic cube and transport it with an open-loop shoulder sweep.

use rne_ai::{
    body_moved_at_least_m, displacement_m, had_finger_contact, mm_minimal_transport_scene_path,
    named_translation_m, MobileManipulatorAction, MobileManipulatorSim, TRANSPORT_SUCCESS_M,
};

const CLOSE_STEPS: usize = 120;
const TRANSPORT_STEPS: usize = 600;
const GRIPPER_CLOSE_RAD_S: f64 = -2.5;
const TRANSPORT_GRIPPER_RAD_S: f64 = -2.0;
const TRANSPORT_SHOULDER_RAD_S: f64 = 4.0;

fn run_transport(sim: &mut MobileManipulatorSim) -> (bool, bool, f64) {
    let initial = named_translation_m(sim, "grasp_cube").expect("grasp_cube pose");
    let close = MobileManipulatorAction {
        gripper_velocity_rad_s: GRIPPER_CLOSE_RAD_S,
        ..MobileManipulatorAction::default()
    };
    let transport = MobileManipulatorAction {
        gripper_velocity_rad_s: TRANSPORT_GRIPPER_RAD_S,
        shoulder_velocity_rad_s: TRANSPORT_SHOULDER_RAD_S,
        ..MobileManipulatorAction::default()
    };

    let mut contacted = false;
    for _ in 0..CLOSE_STEPS {
        sim.step(close);
        contacted = had_finger_contact(sim, "grasp_cube", contacted);
    }
    for _ in 0..TRANSPORT_STEPS {
        sim.step(transport);
        contacted = had_finger_contact(sim, "grasp_cube", contacted);
    }

    let final_pose = named_translation_m(sim, "grasp_cube").expect("grasp_cube final pose");
    let moved = body_moved_at_least_m(sim, "grasp_cube", initial, TRANSPORT_SUCCESS_M);
    (contacted, moved, displacement_m(initial, final_pose))
}

fn main() {
    let smoke = std::env::args().any(|arg| arg == "--smoke");
    let scene_path = mm_minimal_transport_scene_path();
    let mut sim = MobileManipulatorSim::from_scene_path(&scene_path).expect("load transport scene");

    if smoke {
        let (contacted, moved, delta_m) = run_transport(&mut sim);
        if contacted && moved {
            println!(
                "smoke ok: grasp_cube moved {delta_m:.4} m with finger contact (min={TRANSPORT_SUCCESS_M} m)"
            );
            return;
        }
        eprintln!(
            "smoke failed: contact={contacted} moved={moved} delta={delta_m:.4} m (min={TRANSPORT_SUCCESS_M} m)"
        );
        std::process::exit(1);
    }

    let (contacted, moved, delta_m) = run_transport(&mut sim);
    println!("finger contact = {contacted}");
    println!("cube moved >= {TRANSPORT_SUCCESS_M} m = {moved}");
    println!("cube displacement = {delta_m:.4} m");
}
