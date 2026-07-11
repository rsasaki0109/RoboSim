//! Headless deterministic standing smoke for the built-in 12-DoF quadruped.

use rne_ai::{quadruped_scene_path, UrdfSceneSim};

const FOOT_LINKS: [&str; 4] = ["fl_foot", "fr_foot", "rl_foot", "rr_foot"];

fn main() {
    let mut sim = UrdfSceneSim::from_scene_path(&quadruped_scene_path())
        .expect("load built-in quadruped scene");
    sim.configure_position_motors(1200.0, 70.0, 40.0);
    for _ in 0..180 {
        sim.step_joint_position_targets(&[]);
    }

    let observation = sim.observe();
    let foot_impulses_ns = FOOT_LINKS.map(|foot| sim.link_contact_impulse_ns(foot));
    let standing = observation.actuated_joint_count == 12
        && observation.base_y_m > 0.35
        && foot_impulses_ns.iter().all(|impulse| *impulse > 0.0);

    println!(
        "quadruped stand: standing={standing} joints={} base_y={:.3} m foot_impulses={foot_impulses_ns:?} N·s",
        observation.actuated_joint_count, observation.base_y_m
    );
    if !standing {
        std::process::exit(1);
    }
}
