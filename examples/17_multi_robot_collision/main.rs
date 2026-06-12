//! Head-on multi-robot collision scenario with contact reporting.

use rne_ai::{
    head_on_collision_sim, inter_robot_contacts, robot_separation_m, robots_in_contact,
    DiffDriveAction, DiffDriveSim,
};

fn main() {
    let from_scene = run_scene_asset();
    run_head_on(from_scene, "scene asset");
    run_head_on(head_on_collision_sim(), "built-in scenario");
}

fn run_scene_asset() -> DiffDriveSim {
    let scene_path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../assets/scenes/multi_robot_collision.rne.scene.toml");
    DiffDriveSim::from_scene_path(&scene_path).expect("load collision scene")
}

fn run_head_on(mut sim: DiffDriveSim, label: &str) {
    assert_eq!(
        sim.robots().len(),
        2,
        "{label}: expected two robots in simulation"
    );

    let robot_a = sim.robots()[0].robot;
    let robot_b = sim.robots()[1].robot;
    let mut contact_step = None;

    for step in 0..400 {
        sim.step_robots_actions(&[
            (robot_a, DiffDriveAction::forward(6.0)),
            (robot_b, DiffDriveAction::forward(-6.0)),
        ]);

        if robots_in_contact(&sim, robot_a, robot_b) {
            contact_step = Some(step);
            break;
        }
    }

    let separation_m = robot_separation_m(&sim, robot_a, robot_b).unwrap_or(f64::INFINITY);
    let obs_a = sim.observe_robot(robot_a);
    let contacts = inter_robot_contacts(&sim);

    println!(
        "{label}: contact_step={:?} separation={separation_m:.2} m peer_delta_x={:.2} m contacts={}",
        contact_step,
        obs_a.peer_delta_x_m.unwrap_or(f64::NAN),
        contacts.len()
    );

    if contact_step.is_none() && separation_m > 0.55 {
        std::process::exit(1);
    }
}
