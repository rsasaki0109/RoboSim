//! Full 3D pick-and-place on the `mm_lift` robot: lower the top-down claw over a
//! cube on the ground, grasp it, lift it, swing the arm to a new location, lower it
//! back down, and open the claw to release it.
//!
//! This exercises the whole manipulator redesign end to end: the column base lets the
//! lift reach the ground, position-controlled arm joints hold the commanded pose, and
//! the top-down claw grasps a ground object the side-grip could not.

use rne_ai::{
    mm_lift_pick_scene_path, LiftPickPlacePolicy, MobileManipulatorAction, MobileManipulatorSim,
};

/// Minimum horizontal distance the cube must be carried for the place to count.
const MIN_CARRY_M: f64 = 0.5;
/// Steps the scripted pick-and-place policy runs through (lower → grasp → lift → swing
/// → settle → lower → release).
const PICK_PLACE_STEPS: usize = 1030;

fn run_pick_place(sim: &mut MobileManipulatorSim) -> bool {
    // Settle the arm, then drive the shared scripted pick-and-place policy.
    for _ in 0..150 {
        sim.step(MobileManipulatorAction::default());
    }
    let mut policy = LiftPickPlacePolicy::new();
    for _ in 0..PICK_PLACE_STEPS {
        let obs = sim.observe();
        sim.step(policy.next_action(&obs));
    }
    !sim.is_grasping()
}

fn main() {
    let smoke = std::env::args().any(|arg| arg == "--smoke");

    let scene = mm_lift_pick_scene_path();
    let mut sim = MobileManipulatorSim::from_scene_path(&scene).expect("load mm_lift_pick scene");
    let start = sim.named_translation_m("lift_cube").expect("cube");

    let released = run_pick_place(&mut sim);
    let placed = sim.named_translation_m("lift_cube").expect("cube");
    let carried = ((placed.0 - start.0).powi(2) + (placed.2 - start.2).powi(2)).sqrt();
    let ok = released && carried > MIN_CARRY_M && placed.1 < 0.1;

    if smoke {
        if ok {
            println!(
                "pick-place smoke ok: cube carried {carried:.2} m and released at ({:.2}, {:.2}, {:.2})",
                placed.0, placed.1, placed.2
            );
            return;
        }
        eprintln!(
            "smoke failed: released={released} carried={carried:.2} m placed_y={:.3}",
            placed.1
        );
        std::process::exit(1);
    }

    println!(
        "pick-and-place: released={released} carried={carried:.2} m placed=({:.2}, {:.2}, {:.2})",
        placed.0, placed.1, placed.2
    );
}
