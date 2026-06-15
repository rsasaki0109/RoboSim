//! Renders a README hero still of the `mm_lift` robot performing a 3D pick-and-place:
//! the top-down claw has grasped a cube off the ground and the lift is carrying it
//! aloft toward the place location.
//!
//! Run (needs a GPU; set `RNE_SKIP_GPU=1` to skip):
//!   cargo run -p lift_pick_place_hero --example 32_lift_pick_place_hero

use std::fs;
use std::path::{Path, PathBuf};

use png::{BitDepth, ColorType, Encoder};
use rne_ai::{
    build_visual_render_scene, mm_lift_pick_scene_path, LiftPickPlacePolicy,
    MobileManipulatorAction, MobileManipulatorSim,
};
use rne_math::{Quat, Vec3};
use rne_render::{Camera, RenderBackend, RenderScene, VisualShape};
use rne_render_wgpu::{CameraOrbit, WgpuRenderBackend};
use rne_world::Transform3;

const CLEAR_COLOR: [f32; 4] = [0.05, 0.08, 0.12, 1.0];
const RENDER_WIDTH: u32 = 960;
const RENDER_HEIGHT: u32 = 540;
const SETTLE_STEPS: usize = 150;
/// Run the pick-and-place policy through the lift phase: lower (200) + grasp (120) +
/// lift (150), so the claw has just hoisted the cube off the ground.
const CAPTURE_STEP: usize = 470;

fn main() {
    if std::env::var("RNE_SKIP_GPU").is_ok() {
        eprintln!("RNE_SKIP_GPU set; skipping lift pick-place hero render");
        return;
    }

    let repo_root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
    let media_dir = repo_root.join("docs/media");
    fs::create_dir_all(&media_dir).expect("create media directory");

    let mut sim = MobileManipulatorSim::from_scene_path(&mm_lift_pick_scene_path())
        .expect("load mm_lift_pick scene");
    for _ in 0..SETTLE_STEPS {
        sim.step(MobileManipulatorAction::default());
    }
    let mut policy = LiftPickPlacePolicy::new();
    for _ in 0..CAPTURE_STEP {
        sim.step(policy.next_action());
    }
    assert!(
        sim.is_grasping(),
        "expected the claw to be holding the cube at the capture moment"
    );

    let mut backend = WgpuRenderBackend::new().expect("initialize wgpu backend");
    let mut scene = build_visual_render_scene(sim.world());
    // Ground the robot on a floor slab so it doesn't read as floating in space.
    scene.items.push(RenderScene::item_from_visual(
        Transform3::from_translation_rotation(Vec3::new(0.6, -0.05, 0.0), Quat::IDENTITY),
        VisualShape::Box {
            size_m: Vec3::new(6.0, 0.1, 6.0),
        },
        [0.10, 0.13, 0.15, 1.0],
        Transform3::IDENTITY,
    ));

    // Frame the whole robot: the column, the arm reaching out, and the carried cube.
    let camera = Camera::new(RENDER_WIDTH, RENDER_HEIGHT, std::f64::consts::FRAC_PI_4);
    // yaw≈0 keeps the orbit camera level (no roll), so the column reads as upright;
    // the arm extends sideways across the frame with the cube held at its tip.
    let orbit = CameraOrbit {
        focus: Vec3::new(0.62, 0.66, 0.0),
        yaw_rad: -0.72,
        pitch_rad: 0.36,
        distance_m: 2.5,
    };

    let output = backend
        .render_scene_camera(&camera, &orbit.camera_transform(), &scene, CLEAR_COLOR)
        .expect("render hero frame");

    let poster_path = media_dir.join("mm-lift-pickplace.png");
    write_png(
        &poster_path,
        &output.color.rgba8,
        output.color.width,
        output.color.height,
    )
    .expect("write poster png");

    let cube = sim.named_translation_m("lift_cube").expect("cube");
    println!(
        "rendered lift pick-place hero to {} (cube at y={:.2} m, grasping={})",
        poster_path.display(),
        cube.1,
        sim.is_grasping()
    );
}

fn write_png(path: &Path, rgba: &[u8], width: u32, height: u32) -> std::io::Result<()> {
    let file = fs::File::create(path)?;
    let mut encoder = Encoder::new(file, width, height);
    encoder.set_color(ColorType::Rgba);
    encoder.set_depth(BitDepth::Eight);
    let mut writer = encoder.write_header()?;
    writer
        .write_image_data(rgba)
        .map_err(std::io::Error::other)?;
    Ok(())
}
