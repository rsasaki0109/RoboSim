//! Exports a diff-drive episode in the Dr Johnson 3DGS lab as EuRoC stereo+IMU.
//!
//! Rendering uses the photorealistic 3DGS house background
//! (`assets/environments/house_3dgs`) composited with an empty mesh foreground
//! via `render_hybrid_scene_camera`. Physics (including the body-frame IMU)
//! runs in a matching collision scene. Output layout matches example 111,
//! plus `preview_rgb/{cam0,cam1}/<ns>.png` for visualization.
//!
//! Usage: `113_drjohnson_euroc_export OUTPUT_DIR`

use std::error::Error;
use std::fs::{self, File};
use std::io::{self, BufWriter};
use std::path::{Path, PathBuf};

use rne_ai::{
    build_visual_render_scene, DiffDriveAction, DiffDriveEpisode, DiffDriveEpisodeConfig,
    DiffDriveRewardConfig, Episode,
};
use rne_math::{Quat, Transform3, Vec3};
use rne_render::{
    validate_gaussian_splat_manifest_with_override, Camera, HybridRenderScene, RenderScene,
};
use rne_render_3dgs::{load_gaussian_splat_background, render_hybrid_scene_camera};
use rne_render_wgpu::WgpuRenderBackend;
use rne_sensor::{sample_imu, ImuSpec};

const MAX_STEPS: u64 = 2_400; // 40 s at the 1/60 s fixed step
const CAMERA_PERIOD_STEPS: u64 = 3; // 20 Hz
const CAMERA_WIDTH: u32 = 640;
const CAMERA_HEIGHT: u32 = 480;
const CAMERA_FOV_Y_RAD: f64 = 1.2;
const CAMERA_FORWARD_M: f64 = 0.20;
const CAMERA_HEIGHT_OFFSET_M: f64 = 0.35;
const STEREO_BASELINE_M: f64 = 0.06;
const IMU_UPDATE_RATE_HZ: f64 = 60.0;
const CLEAR_COLOR: [f32; 4] = [0.05, 0.06, 0.08, 1.0];
// Fixed overview camera (third-person "sute-kame") for demo videos.
const FIXED_PERIOD_STEPS: u64 = 6;
const FIXED_FOV_Y_RAD: f64 = 0.9;
const FIXED_POS: [f64; 3] = [-1.2, 1.4, -2.9];
const FIXED_TARGET: [f64; 3] = [-2.2, 0.3, -4.4];

/// Look-at view for an RNE camera (forward -Z, up +Y).
fn fixed_view() -> Transform3 {
    let pos = Vec3::new(FIXED_POS[0], FIXED_POS[1], FIXED_POS[2]);
    let target = Vec3::new(FIXED_TARGET[0], FIXED_TARGET[1], FIXED_TARGET[2]);
    let dir = (target - pos).normalize();
    let yaw = (-dir.x).atan2(-dir.z);
    let pitch = dir.y.asin();
    let rotation = Quat::from_rotation_y(yaw) * Quat::from_rotation_x(pitch);
    Transform3::from_translation_rotation(pos, rotation)
}

fn focal_length_px() -> f64 {
    (CAMERA_HEIGHT as f64 / 2.0) / (CAMERA_FOV_Y_RAD / 2.0).tan()
}

/// Camera pose for one stereo side: base frame, lateral offset along Z.
fn camera_pose(base: Transform3, lateral_m: f64) -> Transform3 {
    base.mul_transform(&Transform3::from_translation_rotation(
        Vec3::new(CAMERA_FORWARD_M, CAMERA_HEIGHT_OFFSET_M, lateral_m),
        Quat::from_rotation_y(-std::f64::consts::FRAC_PI_2),
    ))
}

/// RGBA8 -> 8-bit luma (EuRoC grayscale convention).
fn rgba_to_luma(rgba: &[u8]) -> Vec<u8> {
    rgba.chunks_exact(4)
        .map(|p| {
            let luma = 0.299 * p[0] as f64 + 0.587 * p[1] as f64 + 0.114 * p[2] as f64;
            luma.round().clamp(0.0, 255.0) as u8
        })
        .collect()
}

fn write_gray_png(path: &Path, width: u32, height: u32, luma: &[u8]) -> Result<(), Box<dyn Error>> {
    let file = File::create(path)?;
    let writer = BufWriter::new(file);
    let mut encoder = png::Encoder::new(writer, width, height);
    encoder.set_color(png::ColorType::Grayscale);
    encoder.set_depth(png::BitDepth::Eight);
    let mut png_writer = encoder.write_header()?;
    png_writer.write_image_data(luma)?;
    Ok(())
}

fn write_rgb_png(path: &Path, width: u32, height: u32, rgba8: &[u8]) -> Result<(), Box<dyn Error>> {
    let rgb: Vec<u8> = rgba8.chunks_exact(4).flat_map(|p| [p[0], p[1], p[2]]).collect();
    let file = File::create(path)?;
    let writer = BufWriter::new(file);
    let mut encoder = png::Encoder::new(writer, width, height);
    encoder.set_color(png::ColorType::Rgb);
    encoder.set_depth(png::BitDepth::Eight);
    let mut png_writer = encoder.write_header()?;
    png_writer.write_image_data(&rgb)?;
    Ok(())
}

/// Render the robot at its spawn pose from the fixed camera (fast appearance check).
fn robot_preview(output: &Path) -> Result<(), Box<dyn Error>> {
    fs::create_dir_all(output)?;
    let workspace = Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
    let scene_path = workspace.join("assets/scenes/drjohnson_nav.rne.scene.toml");
    let manifest =
        workspace.join("assets/environments/voxel51_drjohnson_3dgs/voxel51_drjohnson.rne.splat.toml");
    let mut environment = DiffDriveEpisode::new(DiffDriveEpisodeConfig {
        max_steps: MAX_STEPS,
        goal_x_m: 1.0e9,
        reward: DiffDriveRewardConfig::default(),
        scene_path: Some(scene_path),
        rng_seed: 7,
        ..DiffDriveEpisodeConfig::default()
    });
    environment.reset();
    let world = environment.simulation().world();
    let mut backend = WgpuRenderBackend::new()
        .map_err(|error| io::Error::other(format!("wgpu unavailable: {error}")))?;
    let splat_env = validate_gaussian_splat_manifest_with_override(&manifest, None)
        .map_err(|error| io::Error::other(format!("splat manifest: {error}")))?;
    let mut background = load_gaussian_splat_background(backend.device(), &splat_env)
        .map_err(|error| io::Error::other(format!("splat background: {error}")))?;
    let mut foreground = build_visual_render_scene(world);
    foreground.items.retain(|item| {
        let s = item.transform.scale;
        s.x.max(s.y).max(s.z) < 2.0
    });
    let hybrid = HybridRenderScene::new(splat_env, foreground);
    let camera = Camera::new(CAMERA_WIDTH, CAMERA_HEIGHT, FIXED_FOV_Y_RAD);
    let pass = render_hybrid_scene_camera(
        &mut backend,
        &mut background,
        &camera,
        &fixed_view(),
        &hybrid,
        CLEAR_COLOR,
    )
    .map_err(|error| io::Error::other(format!("preview render: {error}")))?;
    write_rgb_png(
        &output.join("robot_preview.png"),
        pass.color.width,
        pass.color.height,
        &pass.color.rgba8,
    )?;
    println!("wrote {}", output.display());
    Ok(())
}

/// Render a few fixed candidate views without running the episode (fast framing check).
fn fixed_test(output: &Path) -> Result<(), Box<dyn Error>> {
    fs::create_dir_all(output)?;
    let workspace = Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
    let manifest =
        workspace.join("assets/environments/voxel51_drjohnson_3dgs/voxel51_drjohnson.rne.splat.toml");
    let mut backend = WgpuRenderBackend::new()
        .map_err(|error| io::Error::other(format!("wgpu unavailable: {error}")))?;
    let splat_env = validate_gaussian_splat_manifest_with_override(&manifest, None)
        .map_err(|error| io::Error::other(format!("splat manifest: {error}")))?;
    let mut background = load_gaussian_splat_background(backend.device(), &splat_env)
        .map_err(|error| io::Error::other(format!("splat background: {error}")))?;
    let hybrid = HybridRenderScene::new(splat_env, RenderScene::new());
    let camera = Camera::new(CAMERA_WIDTH, CAMERA_HEIGHT, FIXED_FOV_Y_RAD);
    let candidates: &[([f64; 3], [f64; 3])] = &[
        ([0.3, 2.0, -1.0], [-2.2, 0.3, -4.0]),
        ([-0.5, 1.6, -2.0], [-2.2, 0.3, -4.0]),
        ([-2.2, 1.8, -1.5], [-2.2, 0.3, -4.0]),
        ([0.8, 2.5, 3.0], [0.8, 0.0, -0.7]),
    ];
    for (index, (pos, target)) in candidates.iter().enumerate() {
        let dir = (Vec3::new(target[0], target[1], target[2])
            - Vec3::new(pos[0], pos[1], pos[2]))
        .normalize();
        let yaw = (-dir.x).atan2(-dir.z);
        let pitch = dir.y.asin();
        let view = Transform3::from_translation_rotation(
            Vec3::new(pos[0], pos[1], pos[2]),
            Quat::from_rotation_y(yaw) * Quat::from_rotation_x(pitch),
        );
        let pass = render_hybrid_scene_camera(
            &mut backend,
            &mut background,
            &camera,
            &view,
            &hybrid,
            CLEAR_COLOR,
        )
        .map_err(|error| io::Error::other(format!("fixed render: {error}")))?;
        write_rgb_png(
            &output.join(format!("fixed_{index}.png")),
            pass.color.width,
            pass.color.height,
            &pass.color.rgba8,
        )?;
    }
    println!("wrote {} fixed candidates to {}", candidates.len(), output.display());
    Ok(())
}

fn main() -> Result<(), Box<dyn Error>> {
    let raw_args: Vec<String> = std::env::args().skip(1).collect();
    if raw_args.iter().any(|a| a == "--robot-preview") {
        let output = raw_args
            .iter()
            .find(|a| !a.starts_with("--"))
            .map(PathBuf::from)
            .ok_or_else(|| {
                io::Error::other("usage: 113_drjohnson_euroc_export OUTPUT_DIR [--robot-preview]")
            })?;
        return robot_preview(&output);
    }
    if raw_args.iter().any(|a| a == "--fixed-test") {
        let output = raw_args
            .iter()
            .find(|a| !a.starts_with("--"))
            .map(PathBuf::from)
            .ok_or_else(|| {
                io::Error::other("usage: 113_drjohnson_euroc_export OUTPUT_DIR [--fixed-test]")
            })?;
        return fixed_test(&output);
    }
    let output = std::env::args_os()
        .nth(1)
        .map(PathBuf::from)
        .ok_or_else(|| io::Error::other("usage: 113_drjohnson_euroc_export OUTPUT_DIR"))?;
    for dir in [
        output.join("mav0/cam0/data"),
        output.join("mav0/cam1/data"),
        output.join("mav0/imu0"),
        output.join("preview_rgb/cam0"),
        output.join("preview_rgb/cam1"),
        output.join("preview_fixed"),
    ] {
        fs::create_dir_all(dir)?;
    }

    let workspace = Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
    let scene_path = workspace.join("assets/scenes/drjohnson_nav.rne.scene.toml");
    let manifest =
        workspace.join("assets/environments/voxel51_drjohnson_3dgs/voxel51_drjohnson.rne.splat.toml");

    // Start pose comes from the scene robot asset
    // (dataset_diff_drive_drjohnson: calibrated Dr Johnson spawn).
    let mut environment = DiffDriveEpisode::new(DiffDriveEpisodeConfig {
        max_steps: MAX_STEPS,
        goal_x_m: 1.0e9,
        reward: DiffDriveRewardConfig::default(),
        scene_path: Some(scene_path),
        rng_seed: 7,
        ..DiffDriveEpisodeConfig::default()
    });
    let initial = environment.reset();
    if initial.is_done() {
        return Err(io::Error::other("episode ended during reset").into());
    }
    let robot = *environment.simulation().robot();
    let mut backend = WgpuRenderBackend::new()
        .map_err(|error| io::Error::other(format!("wgpu unavailable: {error}")))?;
    let splat_env = validate_gaussian_splat_manifest_with_override(&manifest, None)
        .map_err(|error| io::Error::other(format!("splat manifest: {error}")))?;
    let mut background = load_gaussian_splat_background(backend.device(), &splat_env)
        .map_err(|error| io::Error::other(format!("splat background: {error}")))?;
    let hybrid = HybridRenderScene::new(splat_env.clone(), RenderScene::new());
    let camera = Camera::new(CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_FOV_Y_RAD);
    let imu_spec = ImuSpec::default();

    let mut cam0_csv = String::from("#timestamp [ns],filename\n");
    let mut cam1_csv = String::from("#timestamp [ns],filename\n");
    let mut imu_csv = String::from(
        "#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]\n",
    );
    let mut gt_tum = String::new();
    let mut frames = 0_u64;
    let mut imu_count = 0_u64;

    for step in 0..MAX_STEPS {
        // Small circle (~0.8 m diameter) around the calibrated spawn: stays on
        // the open rug, clear of the table/chairs; gives translation + rotation
        // (VIO) and frequent revisit (map matching).
        let action = DiffDriveAction {
            left_velocity_rad_s: 1.5,
            right_velocity_rad_s: 5.4,
        };
        let result = environment.step(action);
        let observation = result.observation;
        let sim = environment.simulation();
        let timestamp_ns = sim.sim_time().ticks();
        let world = sim.world();

        let imu = sample_imu(world, robot.base_link, &imu_spec);
        imu_csv.push_str(&format!(
            "{timestamp_ns},{},{},{},{},{},{}\n",
            imu.angular_velocity_rad_s.x,
            imu.angular_velocity_rad_s.y,
            imu.angular_velocity_rad_s.z,
            imu.linear_acceleration_m_s2.x,
            imu.linear_acceleration_m_s2.y,
            imu.linear_acceleration_m_s2.z,
        ));
        imu_count += 1;

        let base = Transform3::from_translation_rotation(
            Vec3::new(
                observation.base_x_m,
                observation.base_y_m,
                observation.base_z_m,
            ),
            Quat::from_rotation_y(observation.base_yaw_rad),
        );
        if step % CAMERA_PERIOD_STEPS == 0 {
            for (side, lateral, csv) in [
                (0_u16, -0.5 * STEREO_BASELINE_M, &mut cam0_csv),
                (1_u16, 0.5 * STEREO_BASELINE_M, &mut cam1_csv),
            ] {
                let pose = camera_pose(base, lateral);
                let pass = render_hybrid_scene_camera(
                    &mut backend,
                    &mut background,
                    &camera,
                    &pose,
                    &hybrid,
                    CLEAR_COLOR,
                )
                .map_err(|error| io::Error::other(format!("hybrid render: {error}")))?;
                let luma = rgba_to_luma(&pass.color.rgba8);
                let name = format!("{timestamp_ns}.png");
                let dir = if side == 0 { "cam0" } else { "cam1" };
                write_gray_png(
                    &output.join(format!("mav0/{dir}/data/{name}")),
                    pass.color.width,
                    pass.color.height,
                    &luma,
                )?;
                write_rgb_png(
                    &output.join(format!("preview_rgb/{dir}/{name}")),
                    pass.color.width,
                    pass.color.height,
                    &pass.color.rgba8,
                )?;
                csv.push_str(&format!("{timestamp_ns},{name}\n"));
            }
            frames += 1;
        }

        // Fixed overview camera: URDF LinkVisuals (base mesh, mast, bar).
        if step % FIXED_PERIOD_STEPS == 0 {
            let fixed_view = fixed_view();
            let fixed_camera = Camera::new(CAMERA_WIDTH, CAMERA_HEIGHT, FIXED_FOV_Y_RAD);
            let mut foreground = build_visual_render_scene(world);
            foreground.items.retain(|item| {
                let s = item.transform.scale;
                s.x.max(s.y).max(s.z) < 2.0
            });
            let hybrid_fixed =
                HybridRenderScene::new(splat_env.clone(), foreground);
            let fixed = render_hybrid_scene_camera(
                &mut backend,
                &mut background,
                &fixed_camera,
                &fixed_view,
                &hybrid_fixed,
                CLEAR_COLOR,
            )
            .map_err(|error| io::Error::other(format!("fixed render: {error}")))?;
            write_rgb_png(
                &output.join(format!("preview_fixed/{timestamp_ns}.png")),
                fixed.color.width,
                fixed.color.height,
                &fixed.color.rgba8,
            )?;
        }

        let q = Quat::from_rotation_y(observation.base_yaw_rad);
        gt_tum.push_str(&format!(
            "{timestamp_ns} {} {} {} {} {} {} {}\n",
            observation.base_x_m,
            observation.base_y_m,
            observation.base_z_m,
            q.x,
            q.y,
            q.z,
            q.w,
        ));

        if result.is_done() {
            break;
        }
    }

    fs::write(output.join("mav0/cam0/data.csv"), cam0_csv)?;
    fs::write(output.join("mav0/cam1/data.csv"), cam1_csv)?;
    fs::write(output.join("mav0/imu0/data.csv"), imu_csv)?;
    fs::write(output.join("gt.tum"), gt_tum)?;

    let f = focal_length_px();
    let (cx, cy) = (CAMERA_WIDTH as f64 / 2.0, CAMERA_HEIGHT as f64 / 2.0);
    // RNE's camera frame is OpenGL (x right, y up, -z forward); Basalt's camera
    // frame is OpenCV (x right, y down, +z forward), so compose with a 180 deg
    // flip about the camera x axis.
    let q = Quat::from_rotation_y(-std::f64::consts::FRAC_PI_2)
        * Quat::from_rotation_x(std::f64::consts::PI);
    let calib = format!(
        r#"{{
  "value0": {{
    "T_imu_cam": [
      {{"px": {fwd}, "py": {up}, "pz": {neg_half_b}, "qx": {qx}, "qy": {qy}, "qz": {qz}, "qw": {qw}}},
      {{"px": {fwd}, "py": {up}, "pz": {half_b}, "qx": {qx}, "qy": {qy}, "qz": {qz}, "qw": {qw}}}
    ],
    "intrinsics": [
      {{"camera_type": "ds", "intrinsics": {{"fx": {f}, "fy": {f}, "cx": {cx}, "cy": {cy}, "xi": 0.0, "alpha": 0.0}}}},
      {{"camera_type": "ds", "intrinsics": {{"fx": {f}, "fy": {f}, "cx": {cx}, "cy": {cy}, "xi": 0.0, "alpha": 0.0}}}}
    ],
    "resolution": [[{w}, {h}], [{w}, {h}]],
    "calib_accel_bias": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "calib_gyro_bias": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "imu_update_rate": {rate},
    "accel_noise_std": [0.016, 0.016, 0.016],
    "gyro_noise_std": [0.000282, 0.000282, 0.000282],
    "accel_bias_std": [0.001, 0.001, 0.001],
    "gyro_bias_std": [0.0001, 0.0001, 0.0001],
    "T_mocap_world": {{"px": 0.0, "py": 0.0, "pz": 0.0, "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0}},
    "T_imu_marker": {{"px": 0.0, "py": 0.0, "pz": 0.0, "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0}},
    "mocap_time_offset_ns": 0,
    "mocap_to_imu_offset_ns": 0,
    "cam_time_offset_ns": 0
  }}
}}
"#,
        fwd = CAMERA_FORWARD_M,
        up = CAMERA_HEIGHT_OFFSET_M,
        half_b = 0.5 * STEREO_BASELINE_M,
        neg_half_b = -0.5 * STEREO_BASELINE_M,
        qx = q.x,
        qy = q.y,
        qz = q.z,
        qw = q.w,
        w = CAMERA_WIDTH,
        h = CAMERA_HEIGHT,
        rate = IMU_UPDATE_RATE_HZ,
    );
    fs::write(output.join("visloc_calib.json"), calib)?;

    println!(
        "exported {frames} stereo frames, {imu_count} imu samples to {}",
        output.display()
    );
    Ok(())
}
