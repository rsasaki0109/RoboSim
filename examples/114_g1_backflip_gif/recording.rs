//! Verified `MuJoCo` state playback through `RoboSim`'s world and wgpu renderer.
//! This path never advances the native physics world.

use super::*;
use image::codecs::gif::{GifEncoder, Repeat};
use serde_json::Value;
use sha2::{Digest, Sha256};

fn joint_link(name: &str) -> String {
    match name {
        "waist_yaw_joint" => "torso_link".into(),
        "left_wrist_roll_joint" => "left_wrist_roll_rubber_hand".into(),
        "right_wrist_roll_joint" => "right_wrist_roll_rubber_hand".into(),
        _ => format!(
            "{}_link",
            name.strip_suffix("_joint").expect("G1 joint suffix")
        ),
    }
}

fn base_pose(q: &[f64]) -> MathTransform {
    let z_up_to_y_up = Quat::from_rotation_x(BASE_ROTATION_X_RAD);
    MathTransform::from_translation_rotation(
        z_up_to_y_up * Vec3::new(q[0], q[1], q[2]),
        z_up_to_y_up * Quat::from_xyzw(q[4], q[5], q[6], q[3]),
    )
}

pub(super) fn run() {
    let args: Vec<_> = std::env::args().collect();
    let index = args.iter().position(|arg| arg == "--recording").unwrap();
    let directory = PathBuf::from(
        args.get(index + 1)
            .expect("--recording requires an evidence directory"),
    );
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
    let summary: Value =
        serde_json::from_slice(&fs::read(directory.join("summary.json")).expect("summary"))
            .expect("summary JSON");
    assert_eq!(
        summary["passed"], true,
        "recording must pass source-physics gates"
    );
    assert_eq!(
        summary["backend"], "mujoco",
        "expected MuJoCo state convention"
    );
    let bytes = fs::read(directory.join("rollout.json")).expect("recording");
    assert_eq!(
        format!("{:x}", Sha256::digest(&bytes)),
        summary["rollout_sha256"].as_str().unwrap(),
        "recording checksum mismatch"
    );
    let urdf = fs::read(root.join("assets/robots/g1_description/g1_23dof.urdf")).expect("G1 URDF");
    assert_eq!(
        format!("{:x}", Sha256::digest(&urdf)),
        summary["urdf_sha256"].as_str().unwrap(),
        "G1 model mismatch"
    );
    let frames: Vec<Value> = serde_json::from_slice(&bytes).expect("recorded frames");
    assert!(frames.len() > 1);
    let source_names: Vec<String> = serde_json::from_value(summary["joint_names"].clone()).unwrap();
    let source_links: Vec<_> = source_names.iter().map(|name| joint_link(name)).collect();
    let mut sim =
        UrdfSceneSim::from_scene_path(&unitree_g1_dynamic_scene_path()).expect("RoboSim G1 world");
    let (model, names) = build_chain(&mut sim);
    assert_eq!(source_links.len(), names.len(), "joint count mismatch");
    let indices: Vec<_> = names
        .iter()
        .map(|name| {
            source_links
                .iter()
                .position(|source| source == name)
                .expect("joint mapping")
        })
        .collect();
    let render = args.iter().any(|arg| arg == "--gif");
    let mut backend = render.then(|| WgpuRenderBackend::new().expect("wgpu renderer"));
    let output = root.join("docs/media/unitree-g1-robosim-replay.gif");
    let mut encoder = render.then(|| {
        let mut encoder =
            GifEncoder::new_with_speed(fs::File::create(&output).expect("GIF file"), 10);
        encoder.set_repeat(Repeat::Infinite).unwrap();
        encoder
    });
    let camera = Camera::new(PANEL_WIDTH, PANEL_HEIGHT, std::f64::consts::FRAC_PI_4);
    let roots = sim.mesh_package_roots().to_vec();
    let root_refs: Vec<_> = roots.iter().map(PathBuf::as_path).collect();
    let mut cache = MeshRenderCache::new();
    let mut previous_time = -1.0;
    for (index, frame) in frames.iter().enumerate() {
        let time = frame["time_s"].as_f64().expect("frame time");
        assert!(
            time.is_finite() && time > previous_time,
            "nonmonotonic time"
        );
        previous_time = time;
        let q: Vec<f64> = serde_json::from_value(frame["qpos"].clone()).expect("recorded qpos");
        assert_eq!(q.len(), 7 + names.len());
        assert!(q.iter().all(|v| v.is_finite()));
        assert!((q[3..7].iter().map(|v| v * v).sum::<f64>() - 1.0).abs() < 1e-6);
        let joints: Vec<_> = indices.iter().map(|i| q[7 + i]).collect();
        let base = base_pose(&q);
        apply_joint_pose(&mut sim, &model, &joints, base);
        let actual = sim.named_transform("pelvis").expect("projected pelvis");
        assert!((actual.translation - base.translation).length() < 1e-8);
        assert!(actual.rotation.dot(base.rotation).abs() > 1.0 - 1e-8);
        assert_eq!(
            sim.sim_time().ticks(),
            0,
            "playback must not step native physics"
        );
        if index % 3 == 0 {
            if let (Some(backend), Some(encoder)) = (&mut backend, &mut encoder) {
                let rgba = render_panel(backend, &camera, &mut cache, &root_refs, &sim, time);
                let end = (index + 3).min(frames.len() - 1);
                let delay_ms = ((frames[end]["time_s"].as_f64().unwrap() - time) * 1000.0)
                    .round()
                    .max(1.0) as u32;
                encoder
                    .encode_frame(image::Frame::from_parts(
                        image::RgbaImage::from_raw(PANEL_WIDTH, PANEL_HEIGHT, rgba).unwrap(),
                        0,
                        0,
                        image::Delay::from_numer_denom_ms(delay_ms, 1),
                    ))
                    .expect("encode recorded frame");
            }
        }
    }
    drop(encoder);
    if render {
        let raw = output.with_extension("raw.gif");
        fs::rename(&output, &raw).expect("stage GIF for labeling");
        let status = std::process::Command::new("ffmpeg")
            .args(["-v", "error", "-y", "-i"])
            .arg(&raw)
            .args(["-vf", "drawbox=x=0:y=0:w=iw:h=60:color=black@0.75:t=fill,drawtext=text='RoboSim world / MuJoCo state replay':x=16:y=10:fontsize=22:fontcolor=white,drawtext=text='G1 EDU 120 Nm - native physics not stepped':x=16:y=36:fontsize=17:fontcolor=white,split[a][b];[a]palettegen[p];[b][p]paletteuse"])
            .arg(&output)
            .status().expect("ffmpeg with drawtext is required for GIF labeling");
        assert!(status.success(), "GIF labeling failed; raw frames retained");
        fs::remove_file(raw).expect("remove temporary GIF");
    }
    println!(
        "{} verified MuJoCo frames projected into RoboSim; native physics was not stepped. GIF: {}",
        frames.len(),
        if render {
            output.display().to_string()
        } else {
            "disabled (headless check)".into()
        }
    );
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn source_z_up_maps_to_native_y_up_without_losing_rotation() {
        for angle in [0.0, 1.7, 3.2, 6.0] {
            let rotation = Quat::from_rotation_y(angle);
            let [x, y, z, w] = rotation.to_array();
            let base = base_pose(&[2.0, 3.0, 4.0, w, x, y, z]);
            assert!((base.translation - Vec3::new(2.0, 4.0, -3.0)).length() < 1e-12);
            let expected_forward = Vec3::new(angle.cos(), -angle.sin(), 0.0);
            assert!((base.rotation * Vec3::X - expected_forward).length() < 1e-12);
        }
    }
}
