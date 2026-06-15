//! Wrist camera DataBus smoke for `mm_minimal`.

use rne_ai::{
    mm_minimal_scene_path, wrist_camera_image_valid, MobileManipulatorAction, MobileManipulatorSim,
};

const EXPECTED_RGBA8_BYTES: usize = 64 * 48 * 4;
const WARMUP_STEPS: usize = 12;

fn main() {
    let smoke = std::env::args().any(|arg| arg == "--smoke");
    let mut sim =
        MobileManipulatorSim::from_scene_path(&mm_minimal_scene_path()).expect("load mm_minimal");

    if smoke && !sim.wrist_camera_enabled() {
        eprintln!("smoke failed: mm_minimal scene has no wrist camera");
        std::process::exit(1);
    }

    for _ in 0..WARMUP_STEPS {
        sim.step(MobileManipulatorAction::default());
    }

    let obs = sim.observe();
    let image = sim.latest_wrist_camera();

    if smoke {
        if obs.wrist_camera_pixels >= EXPECTED_RGBA8_BYTES
            && image
                .as_ref()
                .map(|frame| wrist_camera_image_valid(frame, EXPECTED_RGBA8_BYTES))
                .unwrap_or(false)
        {
            println!(
                "smoke ok: wrist_camera pixels={} ee=({:.3}, {:.3}, {:.3})",
                obs.wrist_camera_pixels, obs.ee_x_m, obs.ee_y_m, obs.ee_z_m
            );
            return;
        }
        eprintln!(
            "smoke failed: wrist_camera pixels={} (expected >= {EXPECTED_RGBA8_BYTES})",
            obs.wrist_camera_pixels
        );
        std::process::exit(1);
    }

    println!("wrist camera enabled = {}", sim.wrist_camera_enabled());
    println!("wrist camera pixels = {}", obs.wrist_camera_pixels);
    if let Some(frame) = image {
        println!("image = {}x{}", frame.width, frame.height);
    }
}
