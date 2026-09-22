//! Render hash-verified native dynamics recordings without replaying physics.

use super::*;
use image::codecs::gif::{GifEncoder, Repeat};
use serde_json::Value;
use sha2::{Digest, Sha256};

fn number(value: &Value, field: &str) -> f64 {
    let result = value[field].as_f64().expect("numeric native metric");
    assert!(result.is_finite(), "finite native metric required");
    result
}

// This checks a held landing for visualization. It does not override the
// recording's separate actuator/full-body qualification status.
fn held_landing(value: &Value) -> bool {
    value["backend"] == "RoboSim/Rapier"
        && value.get("failure") == Some(&Value::Null)
        && number(value, "completed_maneuver_time_s") >= 5.0
        && (number(value, "signed_rotation_rad") + std::f64::consts::TAU).abs() < 0.1
        && number(value, "final_second_min_upright") > 0.99
        && number(value, "final_second_max_base_speed_m_s") < 0.1
        && value["final_second_continuous_foot_contact"] == true
        && value["final_second_contact_diagnostics"]["no_active_ground_pair_steps"] == 0
}

fn native_pose(frame: &Value) -> MathTransform {
    let position: [f64; 3] = serde_json::from_value(frame["base_translation_m"].clone()).unwrap();
    let rotation: [f64; 4] = serde_json::from_value(frame["base_rotation_xyzw"].clone()).unwrap();
    assert!(position.iter().chain(&rotation).all(|v| v.is_finite()));
    let rotation = Quat::from_array(rotation);
    assert!((rotation.length_squared() - 1.0).abs() < 1e-6);
    MathTransform::from_translation_rotation(Vec3::from_array(position), rotation)
}

pub(super) fn run() {
    let args: Vec<_> = std::env::args().collect();
    let index = args
        .iter()
        .position(|arg| arg == "--native-recording")
        .unwrap();
    let directory = PathBuf::from(args.get(index + 1).expect("native evidence directory"));
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
    let manifest: Value = serde_json::from_slice(
        &fs::read(directory.join("render-manifest.json")).expect("native render manifest"),
    )
    .unwrap();
    let bytes = fs::read(directory.join("rollout.json")).expect("native rollout");
    assert_eq!(
        format!("{:x}", Sha256::digest(&bytes)),
        manifest["rollout_sha256"].as_str().unwrap(),
        "native recording checksum mismatch"
    );
    let urdf = fs::read(root.join("assets/robots/g1_description/g1_23dof.urdf")).unwrap();
    assert_eq!(
        format!("{:x}", Sha256::digest(&urdf)),
        manifest["visual_urdf_sha256"].as_str().unwrap(),
        "visual model checksum mismatch"
    );
    let recording: Value = serde_json::from_slice(&bytes).unwrap();
    assert!(
        held_landing(&recording),
        "recording must demonstrate a held native landing"
    );
    let frames = recording["frames"].as_array().expect("native frames");
    assert!(
        (2..=1700).contains(&frames.len()),
        "bounded native recording required"
    );
    let source_names: Vec<String> =
        serde_json::from_value(recording["joint_link_names"].clone()).unwrap();
    let mut sim = UrdfSceneSim::from_scene_path(&unitree_g1_dynamic_scene_path()).unwrap();
    let (model, names) = build_chain(&mut sim);
    assert_eq!(source_names.len(), names.len());
    let indices: Vec<_> = names
        .iter()
        .map(|name| {
            assert_eq!(source_names.iter().filter(|n| *n == name).count(), 1);
            source_names.iter().position(|n| n == name).unwrap()
        })
        .collect();
    let render = args.iter().any(|arg| arg == "--gif");
    let output = root.join("docs/media/unitree-g1-robosim-native-backflip.gif");
    let raw = output.with_extension("raw.gif");
    if render {
        assert!(!output.exists(), "native GIF already exists");
        let space = std::process::Command::new("df")
            .args(["-B1", "--output=avail"])
            .arg(&root)
            .output()
            .expect("disk reserve check");
        assert!(space.status.success());
        let available: u64 = String::from_utf8(space.stdout)
            .unwrap()
            .split_whitespace()
            .last()
            .unwrap()
            .parse()
            .unwrap();
        assert!(
            available >= 30 * 1024_u64.pow(3),
            "30 GiB disk reserve required"
        );
    }
    let mut backend = render.then(|| WgpuRenderBackend::new().expect("wgpu renderer"));
    let mut encoder = render.then(|| {
        let file = fs::OpenOptions::new()
            .write(true)
            .create_new(true)
            .open(&raw)
            .unwrap();
        let mut encoder = GifEncoder::new_with_speed(file, 10);
        encoder.set_repeat(Repeat::Infinite).unwrap();
        encoder
    });
    let camera = Camera::new(PANEL_WIDTH, PANEL_HEIGHT, std::f64::consts::FRAC_PI_4);
    let roots = sim.mesh_package_roots().to_vec();
    let root_refs: Vec<_> = roots.iter().map(PathBuf::as_path).collect();
    let mut cache = MeshRenderCache::new();
    let selected: Vec<usize> = frames
        .iter()
        .enumerate()
        .filter_map(|(i, f)| {
            let time = number(f, "time_s");
            (time >= -0.2 && (i % if time < 2.0 { 3 } else { 10 } == 0 || i == frames.len() - 1))
                .then_some(i)
        })
        .collect();
    let mut previous = f64::NEG_INFINITY;
    let mut render_index = 0;
    for (i, frame) in frames.iter().enumerate() {
        let time = number(frame, "time_s");
        assert!(time > previous && (-1.001..=15.001).contains(&time));
        previous = time;
        let joints: Vec<f64> =
            serde_json::from_value(frame["joint_positions_rad"].clone()).unwrap();
        assert_eq!(joints.len(), names.len());
        assert!(joints.iter().all(|v| v.is_finite()));
        let mapped: Vec<_> = indices.iter().map(|i| joints[*i]).collect();
        let pose = native_pose(frame);
        apply_joint_pose(&mut sim, &model, &mapped, pose);
        let actual = sim.named_transform("pelvis").unwrap();
        assert!((actual.translation - pose.translation).length() < 1e-8);
        assert!(actual.rotation.dot(pose.rotation).abs() > 1.0 - 1e-8);
        assert_eq!(
            sim.sim_time().ticks(),
            0,
            "recording playback must not step physics"
        );
        if selected.get(render_index) == Some(&i) {
            if let (Some(backend), Some(encoder)) = (&mut backend, &mut encoder) {
                let rgba = render_panel(backend, &camera, &mut cache, &root_refs, &sim, time);
                let next = selected.get(render_index + 1).copied().unwrap_or(i);
                let delay_ms = ((number(&frames[next], "time_s") - time) * 1000.0)
                    .round()
                    .max(10.0) as u32;
                encoder
                    .encode_frame(image::Frame::from_parts(
                        image::RgbaImage::from_raw(PANEL_WIDTH, PANEL_HEIGHT, rgba).unwrap(),
                        0,
                        0,
                        image::Delay::from_numer_denom_ms(delay_ms, 1),
                    ))
                    .unwrap();
            }
            render_index += 1;
        }
    }
    drop(encoder);
    let limits = number(&recording, "peak_joint_speed_ratio") <= 1.05
        && number(&recording, "max_joint_position_excess_rad") <= 0.02;
    if render {
        let label = format!(
            "Foot-contact model / peak speed {:.3}x / limits {}",
            number(&recording, "peak_joint_speed_ratio"),
            if limits { "passed" } else { "NOT passed" }
        );
        let filter = format!("drawbox=x=0:y=0:w=iw:h=65:color=black@0.8:t=fill,drawtext=text='RoboSim / Rapier recorded dynamics':x=12:y=8:fontsize=22:fontcolor=white,drawtext=text='{label}':x=12:y=37:fontsize=15:fontcolor=white,split[a][b];[a]palettegen[p];[b][p]paletteuse");
        let status = std::process::Command::new("ffmpeg")
            .args(["-v", "error", "-n", "-i"])
            .arg(&raw)
            .args(["-vf", &filter])
            .arg(&output)
            .status()
            .expect("ffmpeg labeling");
        assert!(status.success(), "labeling failed; raw GIF retained");
        fs::remove_file(raw).unwrap();
    }
    println!("{} native dynamics frames verified; playback physics ticks=0; measured joint limits passed={limits}; full-body qualification pending", frames.len());
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    #[test]
    fn native_recording_requires_held_native_motion_without_promoting_joint_limits() {
        let mut record = json!({"backend":"RoboSim/Rapier", "failure":null,
            "completed_maneuver_time_s":15.0,"signed_rotation_rad":-std::f64::consts::TAU,
            "final_second_min_upright":1.0,"final_second_max_base_speed_m_s":0.01,
            "final_second_contact_diagnostics":{"no_active_ground_pair_steps":0},
            "final_second_continuous_foot_contact":true,
            "qualified_backflip":false,"peak_joint_speed_ratio":1.2});
        assert!(held_landing(&record));
        record["failure"] = json!("collapse");
        assert!(!held_landing(&record));
        record["failure"] = Value::Null;
        record["backend"] = json!("mujoco");
        assert!(!held_landing(&record));
    }

    #[test]
    fn native_pose_keeps_y_up_coordinates_and_full_quaternion() {
        let q = Quat::from_rotation_x(-std::f64::consts::FRAC_PI_2) * Quat::from_rotation_y(2.0);
        let pose = native_pose(
            &json!({"base_translation_m":[-0.5,0.8,0.1],"base_rotation_xyzw":q.to_array()}),
        );
        assert_eq!(pose.translation, Vec3::new(-0.5, 0.8, 0.1));
        assert!(pose.rotation.dot(q).abs() > 1.0 - 1e-12);
    }
}
