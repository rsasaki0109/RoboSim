//! G1 backflip visualization and native motor-only transfer probe.
//!
//! `--recording DIR --gif` projects a hash-verified `MuJoCo` rollout into the
//! `RoboSim` world and renderer; it does not advance Rapier. Omit `--gif` for a
//! headless validation of every recorded configuration and joint mapping.
//! `--native-probe` runs a separate motor-only Rapier transfer experiment.
//! `--smoke` / `--gif` without a recording retain the synthetic reference
//! animation. Neither reference animation nor state playback proves a native
//! physics backflip. See `docs/G1_CONTACT_BACKFLIP.md`.

mod native;
mod native_recording;
mod recording;
mod structural_contact;

use std::fs;
use std::path::{Path, PathBuf};
use std::sync::{Arc, OnceLock};

use png::{BitDepth, ColorType, Encoder};
use rne_ai::{build_visual_render_scene, unitree_g1_dynamic_scene_path, UrdfSceneSim};
use rne_dynamics::ArticulatedModel;
use rne_math::{Quat, Transform3 as MathTransform, Vec3};
use rne_render::{
    Camera, ImageFrame, MeshRenderCache, PbrMaterial, RenderBackend, RenderScene, RenderSceneItem,
    TriangleMesh, VisualShape,
};
use rne_render_wgpu::{CameraOrbit, WgpuRenderBackend};
use rne_robot::{FloatingBase, Robot};
use rne_world::Transform3;

const BASE_ROTATION_X_RAD: f64 = -std::f64::consts::FRAC_PI_2;
const PANEL_WIDTH: u32 = 720;
const PANEL_HEIGHT: u32 = 720;
const FRAME_COUNT: usize = 112;
const FPS: f64 = 14.0;
const CLEAR_COLOR: [f32; 4] = [0.07, 0.085, 0.11, 1.0];

fn to_math(transform: Transform3) -> MathTransform {
    MathTransform {
        translation: transform.translation,
        rotation: transform.rotation,
        scale: transform.scale,
    }
}

fn smoothstep(t: f64) -> f64 {
    let t = t.clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

fn side_sign(name: &str) -> f64 {
    if name.contains("left") {
        1.0
    } else if name.contains("right") {
        -1.0
    } else {
        0.0
    }
}

fn stand_angle(name: &str) -> f64 {
    if name.contains("hip_pitch") {
        -0.18
    } else if name.contains("hip_roll") {
        0.05 * side_sign(name)
    } else if name.contains("knee") {
        0.36
    } else if name.contains("ankle_pitch") {
        -0.18
    } else if name.contains("ankle_roll") {
        -0.03 * side_sign(name)
    } else if name.contains("shoulder_roll") {
        0.20 * side_sign(name)
    } else if name.contains("elbow") {
        0.42
    } else {
        0.0
    }
}

fn crouch_angle(name: &str) -> f64 {
    if name.contains("hip_pitch") {
        -0.60
    } else if name.contains("knee") {
        1.15
    } else if name.contains("ankle_pitch") {
        -0.55
    } else {
        stand_angle(name)
    }
}

fn tuck_angle(name: &str) -> f64 {
    if name.contains("hip_pitch") {
        -1.15
    } else if name.contains("knee") {
        1.95
    } else if name.contains("ankle_pitch") {
        -0.65
    } else if name.contains("shoulder_pitch") {
        -0.70
    } else if name.contains("elbow") {
        1.10
    } else {
        stand_angle(name)
    }
}

fn lerp(a: f64, b: f64, t: f64) -> f64 {
    a + (b - a) * t
}

/// Joint angles at normalized animation time `tau` in `[0, 1]`, plus the base
/// height and flip angle.
fn pose_at(joint_names: &[String], tau: f64) -> (f64, f64, Vec<f64>) {
    // Timeline: crouch 0.00-0.22, extend 0.22-0.30, flight/flip 0.30-0.72,
    // land 0.72-0.85, settle 0.85-1.00.
    let (base_y, flip, blend): (f64, f64, (f64, f64, f64)) = if tau < 0.22 {
        let t = smoothstep(tau / 0.22);
        (lerp(0.82, 0.66, t), 0.0, (1.0 - t, t, 0.0))
    } else if tau < 0.30 {
        let t = smoothstep((tau - 0.22) / 0.08);
        (lerp(0.66, 0.92, t), 0.0, (1.0 - t, 0.0, t))
    } else if tau < 0.72 {
        let t = (tau - 0.30) / 0.42;
        // Ballistic arc peaking mid-flight.
        let arc = (std::f64::consts::PI * t).sin();
        let height = 0.92 + 0.42 * arc;
        let flip = -2.0 * std::f64::consts::PI * smoothstep((t - 0.05) / 0.9);
        let tuck = smoothstep(t / 0.18).min(smoothstep((1.0 - t) / 0.18));
        (height, flip, (0.0, 1.0 - tuck, tuck))
    } else if tau < 0.86 {
        let t = smoothstep((tau - 0.72) / 0.14);
        (
            lerp(0.92, 0.82, t),
            -2.0 * std::f64::consts::PI,
            (t, 0.0, 1.0 - t),
        )
    } else {
        (0.82, -2.0 * std::f64::consts::PI, (1.0, 0.0, 0.0))
    };
    let (w_stand, w_crouch, w_tuck) = blend;
    let joints = joint_names
        .iter()
        .map(|name| {
            w_stand * stand_angle(name) + w_crouch * crouch_angle(name) + w_tuck * tuck_angle(name)
        })
        .collect();
    (base_y, flip, joints)
}

fn main() {
    if std::env::args().any(|arg| arg == "--native-probe") {
        native::run();
        return;
    }
    if std::env::args().any(|arg| arg == "--native-recording") {
        native_recording::run();
        return;
    }
    if std::env::args().any(|arg| arg == "--recording") {
        recording::run();
        return;
    }
    if std::env::args().any(|argument| argument == "--smoke") {
        run_smoke();
        return;
    }
    if !std::env::args().any(|argument| argument == "--gif") {
        run_smoke();
        return;
    }
    if std::env::var("RNE_SKIP_GPU").is_ok() {
        return;
    }

    let repo_root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
    let media_dir = repo_root.join("docs/media");
    let frames_dir = media_dir.join("unitree-g1-backflip-frames");
    let _ = fs::remove_dir_all(&frames_dir);
    fs::create_dir_all(&frames_dir).expect("create G1 backflip frame directory");

    let mut sim =
        UrdfSceneSim::from_scene_path(&unitree_g1_dynamic_scene_path()).expect("load dynamic G1");
    let (model, joint_names) = build_chain(&mut sim);

    let mut backend = WgpuRenderBackend::new().expect("initialize wgpu");
    let camera = Camera::new(PANEL_WIDTH, PANEL_HEIGHT, std::f64::consts::FRAC_PI_4);
    let mesh_roots: Vec<PathBuf> = sim.mesh_package_roots().to_vec();
    let mesh_root_refs: Vec<&Path> = mesh_roots.iter().map(PathBuf::as_path).collect();
    let mut mesh_cache = MeshRenderCache::new();

    for frame in 0..FRAME_COUNT {
        let tau = frame as f64 / (FRAME_COUNT - 1) as f64;
        apply_pose(&mut sim, &model, &joint_names, tau);
        let rgba = render_panel(
            &mut backend,
            &camera,
            &mut mesh_cache,
            &mesh_root_refs,
            &sim,
            tau,
        );
        write_png(
            &frames_dir.join(format!("frame-{frame:03}.png")),
            &rgba,
            PANEL_WIDTH,
            PANEL_HEIGHT,
        )
        .expect("write G1 backflip frame");
    }

    let gif_path = media_dir.join("unitree-g1-backflip.gif");
    build_gif(&frames_dir, &gif_path).expect("encode G1 backflip gif");
    image::open(frames_dir.join(format!("frame-{:03}.png", FRAME_COUNT - 1)))
        .expect("read G1 backflip poster frame")
        .save(media_dir.join("unitree-g1-backflip.png"))
        .expect("write G1 backflip poster");
    let _ = fs::remove_dir_all(&frames_dir);
    println!(
        "rendered G1 kinematic backflip media to {}",
        gif_path.display()
    );
}

/// Builds the floating model from the live simulator world (so entity ids
/// match) and returns the tuck-pose kinematic chain relative to the base frame.
fn build_chain(sim: &mut UrdfSceneSim) -> (ArticulatedModel, Vec<String>) {
    let world = sim.world_mut();
    let robot = world
        .iter_entities()
        .find_map(|entity| entity.get::<Robot>().map(|_| entity.id()))
        .expect("robot");
    let base = world.get::<Robot>(robot).expect("robot").base_link;
    let saved = world.get::<Transform3>(base).copied().unwrap_or_default();
    world.entity_mut(base).insert((
        FloatingBase,
        Transform3::from_translation_rotation(
            Vec3::ZERO,
            Quat::from_rotation_x(BASE_ROTATION_X_RAD),
        ),
    ));
    let model = ArticulatedModel::from_robot(&*world, robot).expect("floating G1 model");
    world.entity_mut(base).insert(saved);

    let joint_names: Vec<String> = model
        .kinematic()
        .movable_joint_entities()
        .iter()
        .map(|joint| {
            let child = model.kinematic().joint_child_link(*joint).expect("child");
            let index = model.kinematic().link_index(child).expect("index");
            model
                .kinematic()
                .link_name(index)
                .expect("name")
                .to_string()
        })
        .collect();

    (model, joint_names)
}

fn apply_pose(sim: &mut UrdfSceneSim, model: &ArticulatedModel, joint_names: &[String], tau: f64) {
    let (base_y, flip, joints) = pose_at(joint_names, tau);
    let base = MathTransform::from_translation_rotation(
        Vec3::new(0.0, base_y, 0.0),
        Quat::from_rotation_z(flip) * Quat::from_rotation_x(BASE_ROTATION_X_RAD),
    );
    apply_joint_pose(sim, model, &joints, base);
}

fn apply_joint_pose(
    sim: &mut UrdfSceneSim,
    model: &ArticulatedModel,
    joints: &[f64],
    base_anim: MathTransform,
) {
    let world = sim.world_mut();

    // Tucked joint chain at the base identity, then the animated base transform.
    let nv = model.nv();
    let mut q = vec![0.0; nv];
    for (dof, joint) in joints.iter().enumerate() {
        q[6 + dof] = *joint;
    }
    let fk = model.kinematic().forward_kinematics(&q).expect("fk");
    let worlds: Vec<MathTransform> = fk.transforms().iter().map(|t| to_math(*t)).collect();
    let base_inverse = worlds[0].inverse();
    let chain: Vec<MathTransform> = worlds
        .iter()
        .map(|w| base_inverse.mul_transform(w))
        .collect();

    let world_math: Vec<MathTransform> = chain
        .iter()
        .map(|local| base_anim.mul_transform(local))
        .collect();

    for index in 0..model.link_count() {
        let Some(entity) = model.link_entity(index) else {
            continue;
        };
        let value = if let Some(parent) = model.link_parent(index) {
            let inverse = world_math[parent].inverse();
            inverse.mul_transform(&world_math[index])
        } else {
            world_math[index]
        };
        world
            .entity_mut(entity)
            .insert(Transform3::from_translation_rotation(
                value.translation,
                value.rotation,
            ));
    }
}

fn render_panel(
    backend: &mut WgpuRenderBackend,
    camera: &Camera,
    mesh_cache: &mut MeshRenderCache,
    mesh_root_refs: &[&Path],
    sim: &UrdfSceneSim,
    _tau: f64,
) -> Vec<u8> {
    let mut scene = build_visual_render_scene(sim.world());
    scene
        .items
        .retain(|item| !matches!(item.shape, VisualShape::Box { .. }));
    append_backdrop(&mut scene);
    mesh_cache
        .resolve_scene(&mut scene, mesh_root_refs)
        .expect("resolve official G1 meshes");
    // Side view so the sagittal flip is fully visible.
    let orbit = CameraOrbit {
        focus: Vec3::new(-0.55, 0.7, 0.0),
        yaw_rad: 0.0,
        pitch_rad: 1.30,
        distance_m: 3.0,
    };
    let output = backend
        .render_scene_camera(camera, &orbit.camera_transform(), &scene, CLEAR_COLOR)
        .expect("render G1 backflip panel");
    output.color.rgba8
}

/// Loads a PBR texture used by the test-bay floor.
fn load_texture(path: &Path) -> Arc<ImageFrame> {
    let rgba = image::open(path)
        .unwrap_or_else(|error| panic!("load render texture {}: {error}", path.display()))
        .into_rgba8();
    Arc::new(ImageFrame::from_rgba8(
        rgba.width(),
        rgba.height(),
        rgba.into_raw(),
    ))
}

/// The surfaces the flip is read against.
struct Backdrop {
    floor: Arc<ImageFrame>,
    floor_normal: Arc<ImageFrame>,
    floor_roughness: Arc<ImageFrame>,
}

/// Loads the backdrop once; every frame draws the same three textures.
fn backdrop() -> &'static Backdrop {
    static BACKDROP: OnceLock<Backdrop> = OnceLock::new();
    BACKDROP.get_or_init(|| {
        let assets = Path::new(env!("CARGO_MANIFEST_DIR"))
            .parent()
            .expect("examples directory")
            .join("63_g1_stride_gif/assets/photoreal_test_bay");
        Backdrop {
            floor: load_texture(&assets.join("concrete_floor_basecolor.png")),
            floor_normal: load_texture(&assets.join("concrete_floor_normal.png")),
            floor_roughness: load_texture(&assets.join("concrete_floor_roughness.png")),
        }
    })
}

/// Draws the concrete floor and the wall behind the robot.
///
/// The checkerboard this replaces was 441 small boxes: high-frequency noise
/// that competed with the figure for attention, and a floor the eye could not
/// judge height against because every tile looked like every other tile. One
/// textured slab reads as a floor, and a wall gives the flip a horizon to
/// rotate against instead of an empty void.
fn append_backdrop(scene: &mut RenderScene) {
    const FOOTPRINT_M: Vec3 = Vec3::new(9.0, 0.0, 7.0);
    const WALL: [f32; 4] = [0.085, 0.10, 0.135, 1.0];
    const WALL_PANEL: [f32; 4] = [0.13, 0.16, 0.21, 1.0];
    const SAFETY_YELLOW: [f32; 4] = [0.72, 0.62, 0.22, 1.0];
    const BASEBOARD: [f32; 4] = [0.26, 0.31, 0.39, 1.0];

    let backdrop = backdrop();
    let half_x = (FOOTPRINT_M.x * 0.5) as f32;
    let half_z = (FOOTPRINT_M.z * 0.5) as f32;
    let repeat_x = (FOOTPRINT_M.x / 1.5) as f32;
    let repeat_z = (FOOTPRINT_M.z / 1.5) as f32;
    let mesh = TriangleMesh {
        positions: vec![
            [-half_x, 0.0, -half_z],
            [half_x, 0.0, -half_z],
            [half_x, 0.0, half_z],
            [-half_x, 0.0, half_z],
        ],
        normals: vec![[0.0, 1.0, 0.0]; 4],
        texcoords: vec![
            [0.0, 0.0],
            [repeat_x, 0.0],
            [repeat_x, repeat_z],
            [0.0, repeat_z],
        ],
        indices: vec![0, 1, 2, 0, 2, 3],
        skinning: None,
    };
    scene.items.push(RenderSceneItem {
        transform: MathTransform {
            translation: Vec3::new(-0.55, -0.004, 0.0),
            rotation: Quat::IDENTITY,
            scale: Vec3::ONE,
        },
        shape: VisualShape::DynamicMesh,
        color_rgba: [1.0; 4],
        mesh: Some(Arc::new(mesh)),
        base_color_texture: Some(Arc::clone(&backdrop.floor)),
        material: PbrMaterial::new([1.0; 4], 0.9, 0.0, [0.0; 3]).with_texture_maps(
            Some(Arc::clone(&backdrop.floor_normal)),
            Some(Arc::clone(&backdrop.floor_roughness)),
        ),
    });

    // The wall sits behind the robot along the camera's view axis. The flip is
    // a rotation, and a rotation needs a static reference to be read against.
    push_backdrop_box(
        scene,
        Vec3::new(-0.55, 1.6, -2.6),
        Vec3::new(9.0, 3.2, 0.12),
        WALL,
    );
    // Recessed panels and a floor-level kick strip give the wall a scale the
    // eye can measure the robot's apex against.
    for x_offset in [-3.0, -1.5, 0.0, 1.5, 3.0] {
        push_backdrop_box(
            scene,
            Vec3::new(-0.55 + x_offset, 1.75, -2.53),
            Vec3::new(1.15, 2.1, 0.02),
            WALL_PANEL,
        );
    }
    // A baseboard where the wall meets the floor. Without it the two surfaces
    // are the same value from this angle and the room reads as one flat card.
    push_backdrop_box(
        scene,
        Vec3::new(-0.55, 0.055, -2.5),
        Vec3::new(9.0, 0.11, 0.06),
        BASEBOARD,
    );
    // A landing lane on the floor: the flip travels backwards, and a marked
    // lane makes how far it travelled legible without a caption.
    for z_offset in [-0.55, 0.55] {
        push_backdrop_box(
            scene,
            Vec3::new(-0.55, 0.003, z_offset),
            Vec3::new(4.2, 0.006, 0.035),
            SAFETY_YELLOW,
        );
    }
}

fn push_backdrop_box(
    scene: &mut RenderScene,
    translation: Vec3,
    size_m: Vec3,
    color_rgba: [f32; 4],
) {
    scene.items.push(RenderSceneItem {
        transform: MathTransform {
            translation,
            rotation: Quat::IDENTITY,
            scale: size_m,
        },
        shape: VisualShape::Box { size_m: Vec3::ONE },
        color_rgba,
        mesh: None,
        base_color_texture: None,
        material: Default::default(),
    });
}

fn run_smoke() {
    let mut sim =
        UrdfSceneSim::from_scene_path(&unitree_g1_dynamic_scene_path()).expect("load dynamic G1");
    let (model, joint_names) = build_chain(&mut sim);
    let mut max_flip = 0.0_f64;
    let mut min_y = f64::MAX;
    let mut max_y = f64::MIN;
    for frame in 0..FRAME_COUNT {
        let tau = frame as f64 / (FRAME_COUNT - 1) as f64;
        let (base_y, flip, _) = pose_at(&joint_names, tau);
        max_flip = max_flip.max(flip.abs());
        min_y = min_y.min(base_y);
        max_y = max_y.max(base_y);
        apply_pose(&mut sim, &model, &joint_names, tau);
        let base = sim.named_transform("pelvis").expect("pelvis");
        assert!(
            base.translation.is_finite() && base.rotation.is_finite(),
            "backflip transform must be finite at frame {frame}"
        );
    }
    // A backflip must complete a full rotation and leave the ground.
    assert!(
        max_flip >= 2.0 * std::f64::consts::PI - 1.0e-6,
        "flip {max_flip}"
    );
    assert!(max_y - min_y > 0.35, "vertical travel {:.3}", max_y - min_y);
    println!(
        "g1 kinematic backflip smoke passed: flip={max_flip:.2} rad travel={:.2} m",
        max_y - min_y
    );
}

fn build_gif(frames_dir: &Path, gif_path: &Path) -> std::io::Result<()> {
    let status = std::process::Command::new("ffmpeg")
        .args([
            "-y",
            "-framerate",
            &format!("{FPS}"),
            "-i",
            &frames_dir.join("frame-%03d.png").to_string_lossy(),
            "-vf",
            &format!("fps={FPS},scale=640:-1:flags=lanczos,split[s0][s1];[s0]palettegen=max_colors=192[p];[s1][p]paletteuse=dither=bayer:bayer_scale=3"),
            &gif_path.to_string_lossy(),
        ])
        .status()?;
    status
        .success()
        .then_some(())
        .ok_or_else(|| std::io::Error::other("ffmpeg G1 backflip gif encode failed"))
}

fn write_png(path: &Path, rgba: &[u8], width: u32, height: u32) -> std::io::Result<()> {
    let file = fs::File::create(path)?;
    let mut encoder = Encoder::new(file, width, height);
    encoder.set_color(ColorType::Rgba);
    encoder.set_depth(BitDepth::Eight);
    let mut writer = encoder.write_header()?;
    writer.write_image_data(rgba).map_err(std::io::Error::other)
}
