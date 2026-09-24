//! Renders a real URDF robot arm following an `rne_planning` plan as the README
//! motion-planning GIF.
//!
//! The arm is the RNE-converted `OpenArm` v2 left arm (7-DOF, GLB meshes). A
//! collision object blocks the straight joint interpolation; the capture runs
//! the actual RRT-Connect planner and plays the resulting trajectory through
//! the wgpu renderer.
//!
//! ```text
//! cargo run --release -p motion_planning_media --example 102_motion_planning_media
//! cargo run -p motion_planning_media --example 102_motion_planning_media -- --smoke
//! cargo run -p motion_planning_media --example 102_motion_planning_media -- --diag
//! ```

use std::fs;
use std::path::{Path, PathBuf};

use png::{BitDepth, ColorType, Encoder};
use rne_ai::build_visual_render_scene;
use rne_ecs::{Entity, World};
use rne_math::{Quat, Vec3};
use rne_planning::{
    trajectory_is_feasible, GoalConstraint, JointInterpolationPlanner, MotionPlanRequest,
    MotionPlanner, PlanningGroup, PlanningOptions, PlanningScene, RrtConnectPlanner,
};
use rne_render::{Camera, MeshRenderCache, RenderBackend, RenderScene, VisualShape};
use rne_render_wgpu::{CameraOrbit, WgpuRenderBackend};
use rne_robot::{CollisionPrimitive, Joint, JointKind, KinematicModel, Transform3};
use rne_urdf_import::{parse_urdf, spawn_urdf_robot_with_config, UrdfSpawnConfig};

const WIDTH: u32 = 960;
const HEIGHT: u32 = 540;
const FRAME_COUNT: usize = 96;
const START_ARM: [f64; 7] = [0.0, 0.35, -0.7, 0.9, 0.0, 0.4, 0.0];
const GOAL_ARM: [f64; 7] = [1.1, -0.5, 0.9, -0.4, 0.7, 0.2, 0.0];
const OBSTACLE_CENTER: [f64; 3] = [-0.174, 0.160, -0.462];
const OBSTACLE_RADIUS_M: f64 = 0.09;
const CLEAR_COLOR: [f32; 4] = [0.035, 0.045, 0.06, 1.0];
const URDF: &str = include_str!("../../assets/robots/openarm_description/openarm_v2_left.rne.urdf");
const MESH_ROOT: &str = "assets/robots/openarm_description";
const BASE_LINK: &str = "openarm_left_base_link";
const TIP_LINK: &str = "openarm_left_ee_base_link";

struct Plan {
    trajectory: Vec<Vec<f64>>,
    feasible: bool,
    interpolated: bool,
}

fn start_configuration(model: &KinematicModel) -> Vec<f64> {
    let mut start = vec![0.0; model.dof()];
    for (index, value) in START_ARM.iter().enumerate() {
        if index < start.len() {
            start[index] = *value;
        }
    }
    start
}

fn build() -> (
    World,
    Entity,
    KinematicModel,
    PlanningScene,
    MotionPlanRequest,
) {
    let urdf = parse_urdf(URDF).expect("parse OpenArm URDF");
    let mut world = World::new();
    let repo_root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
    let config = UrdfSpawnConfig {
        mesh_assets_root: Some(repo_root.join(MESH_ROOT)),
        self_collisions: false,
        ..UrdfSpawnConfig::default()
    };
    let spawned =
        spawn_urdf_robot_with_config(&mut world, &urdf, config).expect("spawn URDF robot");
    let model = KinematicModel::from_robot(&world, spawned.robot).expect("kinematic model");

    let group = PlanningGroup::chain(
        &model,
        "arm",
        spawned.links[BASE_LINK],
        spawned.links[TIP_LINK],
    )
    .expect("arm group");
    let scene = PlanningScene::from_world(&world, spawned.robot)
        .expect("planning scene")
        .with_group(group);

    let options = PlanningOptions {
        seed: 7,
        step_size: 0.25,
        goal_bias: 0.2,
        max_iterations: 8_000,
        collision_check_steps: 20,
        neighbor_radius: 0.6,
        waypoint_count: 90,
        acceleration_limits: vec![2.0; model.dof()],
        ..PlanningOptions::default()
    };
    let request = MotionPlanRequest::new(
        start_configuration(&model),
        GoalConstraint::joint(GOAL_ARM.to_vec()),
        options,
    )
    .with_group("arm");

    (world, spawned.robot, model, scene, request)
}

fn plan_arm() -> (
    World,
    Entity,
    KinematicModel,
    Plan,
    PlanningScene,
    MotionPlanRequest,
) {
    let (world, robot, model, mut scene, request) = build();
    scene.add_collision_object(
        "obstacle",
        CollisionPrimitive::Sphere {
            center_m: Vec3::new(OBSTACLE_CENTER[0], OBSTACLE_CENTER[1], OBSTACLE_CENTER[2]),
            radius_m: OBSTACLE_RADIUS_M,
        },
    );
    let interpolated = JointInterpolationPlanner::new()
        .plan(&scene, &request)
        .is_ok();
    let response = RrtConnectPlanner::new()
        .plan(&scene, &request)
        .expect("rrt-connect plan");
    let feasible =
        trajectory_is_feasible(&scene, &request, &response.trajectory).expect("feasibility");
    let trajectory = response
        .trajectory
        .points()
        .iter()
        .map(|point| point.positions.clone())
        .collect();
    (
        world,
        robot,
        model,
        Plan {
            trajectory,
            feasible,
            interpolated,
        },
        scene,
        request,
    )
}

fn main() {
    if std::env::args().any(|argument| argument == "--diag") {
        run_diag();
        return;
    }
    let (mut world, robot, model, plan, scene, request) = plan_arm();
    if !plan.feasible {
        panic!("RRT-Connect returned an infeasible trajectory");
    }
    if std::env::args().any(|argument| argument == "--smoke") {
        assert!(
            !plan.interpolated,
            "the obstacle should block straight joint interpolation"
        );
        println!(
            "motion-planning smoke ok: interpolation blocked, rrt-connect feasible over {} waypoints (dof {})",
            plan.trajectory.len(),
            model.dof()
        );
        let _ = (robot, scene, request);
        return;
    }
    if std::env::var("RNE_SKIP_GPU").is_ok() {
        println!("RNE_SKIP_GPU set; skipping motion-planning capture");
        return;
    }

    let repo_root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
    let media_dir = repo_root.join("docs/media");
    let mesh_root = repo_root.join(MESH_ROOT);
    let frames_dir =
        PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../target/rne-motion-planning-frames");
    let _ = fs::remove_dir_all(&frames_dir);
    fs::create_dir_all(&frames_dir).expect("create frame directory");

    let mut backend = WgpuRenderBackend::new().expect("initialize wgpu");
    let camera = Camera::new(WIDTH, HEIGHT, std::f64::consts::FRAC_PI_4);
    let orbit = CameraOrbit {
        yaw_rad: 0.9,
        pitch_rad: 0.95,
        distance_m: 1.05,
        focus: Vec3::new(-0.02, 0.32, -0.12),
    };
    let mut mesh_cache = MeshRenderCache::new();
    let mesh_roots = [mesh_root.as_path()];

    let bindings = LinkBindings::capture(&world, &model);
    let samples = resample(&plan.trajectory, FRAME_COUNT);
    let mut trail: Vec<Vec3> = Vec::new();
    let mut last_rgba = Vec::new();

    for (frame, q) in samples.iter().enumerate() {
        bindings.apply(&mut world, q);
        let tip = model
            .forward_kinematics(q)
            .expect("forward kinematics")
            .link_transform(bindings.tip_link)
            .expect("tip transform")
            .translation;
        trail.push(Vec3::new(tip.x, tip.y, tip.z));

        let mut scene = build_visual_render_scene(&world);
        append_set(&mut scene, &trail);
        mesh_cache
            .resolve_scene(&mut scene, &mesh_roots)
            .expect("resolve OpenArm meshes");
        let output = backend
            .render_scene_camera(&camera, &orbit.camera_transform(), &scene, CLEAR_COLOR)
            .expect("render motion-planning frame");
        write_png(
            &frames_dir.join(format!("frame-{frame:03}.png")),
            &output.color.rgba8,
        )
        .expect("write frame");
        last_rgba = output.color.rgba8;
    }

    let gif_path = media_dir.join("motion-planning.gif");
    build_gif(&frames_dir, &gif_path).expect("encode motion-planning gif");
    write_png(&media_dir.join("motion-planning.png"), &last_rgba).expect("write poster");
    let _ = fs::remove_dir_all(&frames_dir);
    println!(
        "rendered motion-planning media to {} ({} frames, interpolation blocked, rrt-connect feasible)",
        gif_path.display(),
        FRAME_COUNT
    );
}

fn run_diag() {
    let (world, robot, model, scene, request) = build();
    let start = &request.start;
    let goal = request.goal.clone();
    let tip = model.link_entity(model.link_count() - 1).unwrap();
    let start_ee = model
        .forward_kinematics(start)
        .expect("fk start")
        .link_transform(tip)
        .unwrap()
        .translation;
    if let GoalConstraint::Joint { positions } = goal {
        let full = scene
            .group("arm")
            .map(|group| group.embed(start, &positions));
        if let Some(full) = full {
            let goal_ee = model
                .forward_kinematics(&full)
                .expect("fk goal")
                .link_transform(model.link_entity(model.link_count() - 1).unwrap())
                .unwrap()
                .translation;
            eprintln!(
                "dof={} start_ee={start_ee:?} goal_ee={goal_ee:?}",
                model.dof()
            );
        }
    }
    if let Some(group) = scene.group("arm") {
        let full = group.embed(start, &GOAL_ARM);
        for t in [0.25_f64, 0.5, 0.75] {
            let q: Vec<f64> = start
                .iter()
                .zip(&full)
                .map(|(a, b)| a * (1.0 - t) + b * t)
                .collect();
            let ee = model
                .forward_kinematics(&q)
                .unwrap()
                .link_transform(tip)
                .unwrap()
                .translation;
            eprintln!("t={t} ee={ee:?}");
        }
    }
    eprintln!("movable={:?}", model.movable_joint_names());
    eprintln!("limits={:?}", model.joint_limits());
    let scene = PlanningScene::from_world(&world, robot).unwrap();
    eprintln!("start valid={:?}", scene.is_state_valid(start));
}

struct LinkBindings {
    joint_to_child: Vec<(usize, Entity, Transform3, JointKind, Vec3)>,
    tip_link: Entity,
}

impl LinkBindings {
    fn capture(world: &World, model: &KinematicModel) -> Self {
        let mut joint_to_child = Vec::new();
        for (dof, joint_entity) in model.movable_joint_entities().iter().enumerate() {
            let Some(joint) = world.get::<Joint>(*joint_entity) else {
                continue;
            };
            let origin = world
                .get::<Transform3>(joint.child_link)
                .copied()
                .unwrap_or(Transform3::IDENTITY);
            joint_to_child.push((dof, joint.child_link, origin, joint.kind, joint.axis));
        }
        let tip_link = model.link_entity(model.link_count() - 1).expect("tip link");
        Self {
            joint_to_child,
            tip_link,
        }
    }

    fn apply(&self, world: &mut World, q: &[f64]) {
        for (dof, child, origin, kind, axis) in &self.joint_to_child {
            let displacement = q.get(*dof).copied().unwrap_or(0.0);
            let motion = match kind {
                JointKind::Revolute | JointKind::Continuous => {
                    Transform3::from_translation_rotation(
                        Vec3::ZERO,
                        Quat::from_axis_angle(axis.normalize_or_zero(), displacement),
                    )
                }
                JointKind::Prismatic => {
                    Transform3::from_translation_rotation(*axis * displacement, Quat::IDENTITY)
                }
                JointKind::Fixed => Transform3::IDENTITY,
            };
            world
                .entity_mut(*child)
                .insert(origin.mul_transform(&motion));
        }
    }
}

fn resample(waypoints: &[Vec<f64>], frames: usize) -> Vec<Vec<f64>> {
    if waypoints.len() < 2 || frames < 2 {
        return waypoints.to_vec();
    }
    (0..frames)
        .map(|frame| {
            let t = frame as f64 / (frames - 1) as f64 * (waypoints.len() - 1) as f64;
            let low = (t.floor() as usize).min(waypoints.len() - 2);
            let blend = t - low as f64;
            waypoints[low]
                .iter()
                .zip(&waypoints[low + 1])
                .map(|(a, b)| a * (1.0 - blend) + b * blend)
                .collect()
        })
        .collect()
}

fn append_set(scene: &mut RenderScene, trail: &[Vec3]) {
    scene.items.push(RenderScene::item_from_visual(
        Transform3 {
            translation: Vec3::new(0.2, -0.02, 0.0),
            rotation: Quat::IDENTITY,
            scale: Vec3::new(1.2, 0.02, 1.2),
        },
        VisualShape::Box { size_m: Vec3::ONE },
        [0.14, 0.16, 0.18, 1.0],
        Transform3::IDENTITY,
    ));
    scene.items.push(RenderScene::item_from_visual(
        Transform3 {
            translation: Vec3::new(OBSTACLE_CENTER[0], OBSTACLE_CENTER[1], OBSTACLE_CENTER[2]),
            rotation: Quat::IDENTITY,
            scale: Vec3::ONE,
        },
        VisualShape::Sphere {
            radius_m: OBSTACLE_RADIUS_M,
        },
        [0.9, 0.22, 0.16, 1.0],
        Transform3::IDENTITY,
    ));
    for point in trail {
        scene.items.push(RenderScene::item_from_visual(
            Transform3 {
                translation: *point,
                rotation: Quat::IDENTITY,
                scale: Vec3::splat(0.012),
            },
            VisualShape::Box { size_m: Vec3::ONE },
            [0.13, 0.82, 0.92, 1.0],
            Transform3::IDENTITY,
        ));
    }
}

fn build_gif(frames_dir: &Path, gif_path: &Path) -> std::io::Result<()> {
    let status = std::process::Command::new("ffmpeg")
        .args([
            "-y",
            "-framerate",
            "12",
            "-i",
            &frames_dir.join("frame-%03d.png").to_string_lossy(),
            "-vf",
            "fps=15,scale=880:-1:flags=lanczos,split[s0][s1];[s0]palettegen=max_colors=192[p];[s1][p]paletteuse=dither=bayer:bayer_scale=3",
            &gif_path.to_string_lossy(),
        ])
        .status()?;
    status
        .success()
        .then_some(())
        .ok_or_else(|| std::io::Error::other("ffmpeg motion-planning gif encode failed"))
}

fn write_png(path: &Path, rgba: &[u8]) -> std::io::Result<()> {
    let file = fs::File::create(path)?;
    let mut encoder = Encoder::new(file, WIDTH, HEIGHT);
    encoder.set_color(ColorType::Rgba);
    encoder.set_depth(BitDepth::Eight);
    let mut writer = encoder.write_header()?;
    writer.write_image_data(rgba).map_err(std::io::Error::other)
}
