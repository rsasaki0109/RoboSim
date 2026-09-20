//! Exports a headless diff-drive episode as a EuRoC-format stereo+IMU dataset.
//!
//! visloc-rs consumes EuRoC `mav0/{cam0,cam1,imu0}` datasets.  This example runs
//! a deterministic RNE diff-drive episode in a feature-rich scene, samples a
//! synthesized stereo pair plus the body-frame IMU, and writes:
//!
//! ```text
//! OUTPUT_DIR/mav0/cam0/{data.csv,data/<ns>.png}
//! OUTPUT_DIR/mav0/cam1/{data.csv,data/<ns>.png}
//! OUTPUT_DIR/mav0/imu0/data.csv
//! OUTPUT_DIR/gt.tum
//! OUTPUT_DIR/visloc_calib.json
//! ```
//!
//! Usage: `111_diff_drive_euroc_export OUTPUT_DIR`

use std::error::Error;
use std::fs::{self, File};
use std::io::{self, BufWriter};
use std::path::{Path, PathBuf};

use rne_ai::{
    build_diff_drive_render_scene, build_visual_render_scene, DiffDriveAction, DiffDriveEpisode,
    DiffDriveEpisodeConfig, DiffDriveRewardConfig, Episode,
};
use rne_math::{Quat, Vec3};
use rne_render_wgpu::WgpuRenderBackend;
use rne_sensor::{sample_camera_rgbd_keyed, sample_imu, CameraSpec, ImuSpec};
use rne_world::Transform3 as WorldTransform3;

const MAX_STEPS: u64 = 2_400; // 40 s at the 1/60 s fixed step
const CAMERA_PERIOD_STEPS: u64 = 3; // 20 Hz
const CAMERA_WIDTH: u32 = 640;
const CAMERA_HEIGHT: u32 = 480;
const CAMERA_FOV_Y_RAD: f64 = 1.2;
const CAMERA_SEED: u64 = 111;
const CAMERA_FORWARD_M: f64 = 0.20;
const CAMERA_HEIGHT_OFFSET_M: f64 = 0.25;
const STEREO_BASELINE_M: f64 = 0.06;
const IMU_UPDATE_RATE_HZ: f64 = 60.0;

fn camera_spec() -> CameraSpec {
    CameraSpec {
        width: CAMERA_WIDTH,
        height: CAMERA_HEIGHT,
        fov_y_rad: CAMERA_FOV_Y_RAD,
        seed: CAMERA_SEED,
        ..CameraSpec::default()
    }
}

fn focal_length_px() -> f64 {
    (CAMERA_HEIGHT as f64 / 2.0) / (CAMERA_FOV_Y_RAD / 2.0).tan()
}

/// Camera pose for one stereo side: base frame, lateral offset along Z.
fn camera_pose(base: WorldTransform3, lateral_m: f64) -> WorldTransform3 {
    base.mul_transform(&WorldTransform3::from_translation_rotation(
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

fn main() -> Result<(), Box<dyn Error>> {
    let output = std::env::args_os()
        .nth(1)
        .map(PathBuf::from)
        .ok_or_else(|| io::Error::other("usage: 111_diff_drive_euroc_export OUTPUT_DIR"))?;
    for dir in [
        output.join("mav0/cam0/data"),
        output.join("mav0/cam1/data"),
        output.join("mav0/imu0"),
    ] {
        fs::create_dir_all(dir)?;
    }

    let workspace = Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
    let scene_path = workspace.join("assets/scenes/visloc_feature_field.rne.scene.toml");

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
    let mut render = WgpuRenderBackend::new()
        .map_err(|error| io::Error::other(format!("wgpu unavailable: {error}")))?;
    let imu_spec = ImuSpec::default();
    let spec = camera_spec();

    let mut cam0_csv = String::from("#timestamp [ns],filename\n");
    let mut cam1_csv = String::from("#timestamp [ns],filename\n");
    let mut imu_csv = String::from(
        "#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]\n",
    );
    let mut gt_tum = String::new();
    let mut frames = 0_u64;
    let mut imu_count = 0_u64;

    for step in 0..MAX_STEPS {
        // Gentle forward motion with a slow sinusoidal turn for VIO excitation.
        let phase = step as f64 * 0.01;
        let delta = 0.3 * phase.sin();
        let action = DiffDriveAction {
            left_velocity_rad_s: (4.0 + delta).clamp(-10.0, 10.0),
            right_velocity_rad_s: (4.0 - delta).clamp(-10.0, 10.0),
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

        if step % CAMERA_PERIOD_STEPS == 0 {
            let base = WorldTransform3::from_translation_rotation(
                Vec3::new(
                    observation.base_x_m,
                    observation.base_y_m,
                    observation.base_z_m,
                ),
                Quat::from_rotation_y(observation.base_yaw_rad),
            );
            let mut render_scene =
                build_diff_drive_render_scene(world, std::slice::from_ref(&robot));
            render_scene
                .items
                .extend(build_visual_render_scene(world).items);
            let key = rne_sensor::SensorNoiseKey::new(2026, CAMERA_SEED, 400, step);
            for (camera, lateral, csv) in [
                (0_u16, -0.5 * STEREO_BASELINE_M, &mut cam0_csv),
                (1_u16, 0.5 * STEREO_BASELINE_M, &mut cam1_csv),
            ] {
                let pose = camera_pose(base, lateral);
                let sample = sample_camera_rgbd_keyed(
                    &mut render,
                    &pose,
                    &spec,
                    sim.sim_time(),
                    &render_scene,
                    key,
                );
                let luma = rgba_to_luma(&sample.rgb.rgba8);
                let name = format!("{timestamp_ns}.png");
                let dir = if camera == 0 { "cam0" } else { "cam1" };
                write_gray_png(
                    &output.join(format!("mav0/{dir}/data/{name}")),
                    sample.rgb.width,
                    sample.rgb.height,
                    &luma,
                )?;
                csv.push_str(&format!("{timestamp_ns},{name}\n"));
            }
            frames += 1;
        }

        let q = Quat::from_rotation_y(observation.base_yaw_rad);
        gt_tum.push_str(&format!(
            "{timestamp_ns} {} {} {} {} {} {} {}\n",
            observation.base_x_m, observation.base_y_m, observation.base_z_m, q.x, q.y, q.z, q.w,
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
