//! Headless standing and foot-load smoke for the built-in 12-DoF humanoid.

use rne_ai::{humanoid_scene_path, UrdfSceneSim};

fn main() {
    let mut sim = UrdfSceneSim::from_scene_path(&humanoid_scene_path())
        .expect("load built-in humanoid scene");
    sim.configure_position_motors(1800.0, 85.0, 80.0);
    for _ in 0..240 {
        sim.step_joint_position_targets(&[]);
    }

    let observation = sim.observe();
    let foot_impulses_ns = [
        sim.link_contact_impulse_ns("left_foot"),
        sim.link_contact_impulse_ns("right_foot"),
    ];
    let standing = observation.actuated_joint_count == 12
        && observation.base_y_m > 0.70
        && foot_impulses_ns.iter().all(|impulse| *impulse > 0.0);
    println!(
        "humanoid stand: standing={standing} joints={} base_y={:.3} m foot_impulses={foot_impulses_ns:?} N·s",
        observation.actuated_joint_count, observation.base_y_m
    );
    if !standing {
        std::process::exit(1);
    }
}
