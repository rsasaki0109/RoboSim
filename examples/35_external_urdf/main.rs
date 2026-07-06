//! Loads vendored external URDF scenes (SO-101 arm, minimal cart, LeKiwi base).

use rne_ai::{
    build_visual_render_scene, cart_minimal_scene_path, lekiwi_scene_path, so101_scene_path,
    UrdfArmAction, UrdfCartAction, UrdfKiwiAction, UrdfSceneSim,
};
use rne_render::VisualShape;

fn main() {
    let so101 = UrdfSceneSim::from_scene_path(&so101_scene_path()).expect("load so101");
    let so101_scene = build_visual_render_scene(so101.world());
    let so101_meshes = so101_scene
        .items
        .iter()
        .filter(|item| matches!(item.shape, VisualShape::Mesh { .. }))
        .count();
    println!(
        "so101: joints={} mesh_visuals={} roots={}",
        so101.observe().actuated_joint_count,
        so101_meshes,
        so101.mesh_package_roots().len()
    );

    let mut cart = UrdfSceneSim::from_scene_path(&cart_minimal_scene_path()).expect("load cart");
    let start = cart.observe().base_x_m;
    for _ in 0..120 {
        cart.step_cart(UrdfCartAction {
            left_velocity_rad_s: 3.0,
            right_velocity_rad_s: 3.0,
        });
    }
    let moved = (cart.observe().base_x_m - start).abs();
    println!("cart_minimal: |displacement_x|={moved:.3} m");

    let mut lekiwi = UrdfSceneSim::from_scene_path(&lekiwi_scene_path()).expect("load lekiwi");
    let lekiwi_start = lekiwi.observe();
    for _ in 0..120 {
        lekiwi.step_kiwi(UrdfKiwiAction {
            vx_m_s: 0.2,
            vz_m_s: 0.0,
            wz_rad_s: 0.0,
        });
    }
    let lekiwi_obs = lekiwi.observe();
    let lekiwi_dx = lekiwi_obs.base_x_m - lekiwi_start.base_x_m;
    let lekiwi_dz = lekiwi_obs.base_z_m - lekiwi_start.base_z_m;
    let lekiwi_planar = (lekiwi_dx * lekiwi_dx + lekiwi_dz * lekiwi_dz).sqrt();
    println!(
        "lekiwi: joints={} planar_displacement={:.3} m",
        lekiwi_obs.actuated_joint_count, lekiwi_planar
    );

    let mut arm = UrdfSceneSim::from_scene_path(&so101_scene_path()).expect("reload so101");
    for _ in 0..60 {
        arm.step_arm(UrdfArmAction {
            shoulder_pan_velocity_rad_s: 1.5,
        });
    }
    println!(
        "so101 teleop smoke: base_yaw={:.3} rad",
        arm.observe().base_yaw_rad
    );

    if so101_meshes < 5
        || moved < 0.02
        || lekiwi_obs.actuated_joint_count < 3
        || lekiwi_planar < 0.02
    {
        std::process::exit(1);
    }
}
