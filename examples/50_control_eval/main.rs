//! Multi-seed statistical controller evaluation.
//!
//! One pure-pursuit controller, two plants, ten seeds. Each seed perturbs what a real
//! evaluation must survive — tire-road friction, initial lateral offset, and steering
//! actuator lag — through deterministic `KeyedRandom` draws, then runs the identical
//! closed loop on the no-slip kinematic bicycle and on the dynamic bicycle with tire
//! saturation. `rne_ai::control_eval` turns each run into standard metrics and
//! aggregates them across seeds, so the verdict is a mean and a spread, not an
//! anecdote from one lucky run.
//!
//! ```bash
//! cargo run --release -p control_eval_demo --example 50_control_eval
//! RNE_SKIP_GPU=1 cargo run -p control_eval_demo --example 50_control_eval
//! ```

use png::{BitDepth, ColorType, Encoder};
use rne_ai::{ControlEvalReport, ControlMetrics, ControlTrackingSample};
use rne_core::{KeyedRandom, SimDuration};
use rne_ecs::{spawn_named, World};
use rne_math::{Quat, Seconds, Vec3};
use rne_physics::RigidBody;
use rne_render::{Camera, RenderBackend, RenderScene, RenderSceneItem, VisualShape};
use rne_render_wgpu::{CameraOrbit, WgpuRenderBackend};
use rne_robot::{
    ackermann_kinematics, pure_pursuit_steering, vehicle_dynamics, AckermannDrive, VehicleDynamics,
};
use rne_world::Transform3;
use std::collections::BTreeMap;
use std::fs;
use std::path::Path;

const WIDTH: u32 = 1_280;
const HEIGHT: u32 = 720;
const FRAME_COUNT: usize = 144;
const RENDER_HZ: usize = 12;
const SIM_HZ: usize = 240;
const SIM_STEPS_PER_FRAME: usize = SIM_HZ / RENDER_HZ;
const _: () = assert!(SIM_HZ.is_multiple_of(RENDER_HZ));
const CLEAR_COLOR: [f32; 4] = [0.06, 0.07, 0.10, 1.0];

/// Evaluation seeds; ascending and unique so reports are byte-stable.
const SEEDS: [u64; 10] = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9];
/// Root seed of the domain-randomization draws.
const EVAL_ROOT_SEED: u64 = 50;
/// Course cruise speed in meters per second.
///
/// Chosen so the sweeper's demand (`v^2 / R` ~ 8.7 m/s²) sits inside the randomized
/// grip range (`mu g` between 7.1 and 9.3): low-friction seeds saturate and understeer
/// while high-friction seeds hold the line. That spread is what a statistical
/// evaluation exists to measure — at a gentler speed both plants score identically and
/// the comparison says nothing.
const CRUISE_SPEED_M_S: f64 = 12.5;
/// Pure-pursuit lookahead in meters.
const LOOKAHEAD_M: f64 = 6.0;
/// Settling band for the tracking metrics, in meters.
const SETTLING_BAND_M: f64 = 0.5;

/// Per-seed randomized conditions.
#[derive(Clone, Copy, Debug, PartialEq)]
struct SeedConditions {
    friction_coefficient: f64,
    initial_offset_m: f64,
    steering_lag_s: f64,
}

/// One evaluated seed: its conditions, metrics, and rendered trail.
#[derive(Clone, Debug, PartialEq)]
struct SeedRun {
    seed: u64,
    conditions: SeedConditions,
    metrics: ControlMetrics,
    trail_m: Vec<Vec3>,
}

fn main() {
    let course = course_waypoints();
    let dynamic_runs = evaluate_plant(&course, Plant::Dynamic);
    let kinematic_runs = evaluate_plant(&course, Plant::Kinematic);

    let dynamic_report = build_report(&dynamic_runs, "dynamic_bicycle");
    let kinematic_report = build_report(&kinematic_runs, "kinematic_bicycle");

    assert_eq!(dynamic_runs.len(), SEEDS.len());
    assert!(
        kinematic_report.rms_error_m.mean < dynamic_report.rms_error_m.mean,
        "the no-slip plant must flatter the controller: {:.3} vs {:.3}",
        kinematic_report.rms_error_m.mean,
        dynamic_report.rms_error_m.mean,
    );
    assert_eq!(kinematic_report.total_violations, 0);

    println!(
        "control evaluation ready: seeds={} cruise_m_s={} settling_band_m={}",
        SEEDS.len(),
        CRUISE_SPEED_M_S,
        SETTLING_BAND_M,
    );
    for (name, report) in [
        ("kinematic", &kinematic_report),
        ("dynamic", &dynamic_report),
    ] {
        println!(
            "  {name:>9}: rms={:.3}+/-{:.3} m  max={:.3} m (worst seed)  effort={:.2}+/-{:.2}  saturated={:.1}%  unsettled={}",
            report.rms_error_m.mean,
            report.rms_error_m.stddev,
            report.max_error_m.max,
            report.effort.mean,
            report.effort.stddev,
            report.saturated_fraction.mean * 100.0,
            report.unsettled_seeds,
        );
    }

    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
    let artifacts_dir = root.join("target/control-eval");
    fs::create_dir_all(&artifacts_dir).expect("create control-eval artifact directory");
    for report in [&dynamic_report, &kinematic_report] {
        fs::write(
            artifacts_dir.join(format!("{}.json", report.controller)),
            report.to_json_pretty().expect("serialize report"),
        )
        .expect("write control-eval report");
    }
    println!("wrote reports to {}", artifacts_dir.display());

    if std::env::var("RNE_SKIP_GPU").is_ok() {
        println!("RNE_SKIP_GPU set; headless control evaluation completed");
        return;
    }

    let frames_dir = root.join("target/control-eval/frames");
    let media_dir = root.join("docs/media");
    if frames_dir.exists() {
        fs::remove_dir_all(&frames_dir).expect("remove old control-eval frames");
    }
    fs::create_dir_all(&frames_dir).expect("create control-eval frame directory");
    fs::create_dir_all(&media_dir).expect("create media directory");

    let mut backend = match WgpuRenderBackend::new() {
        Ok(backend) => backend,
        Err(error) => {
            eprintln!("wgpu unavailable after successful headless evaluation: {error}");
            return;
        }
    };
    let camera = Camera::new(WIDTH, HEIGHT, 0.9);
    let orbit = overview_camera();

    for frame in 0..FRAME_COUNT {
        let scene = evaluation_scene(&course, &dynamic_runs, frame);
        let output = backend
            .render_scene_camera(&camera, &orbit.camera_transform(), &scene, CLEAR_COLOR)
            .expect("render control-eval frame");
        write_png(
            &frames_dir.join(format!("frame-{frame:03}.png")),
            &output.color.rgba8,
            output.color.width,
            output.color.height,
        )
        .expect("write control-eval frame");
    }

    let gif_path = media_dir.join("control-eval.gif");
    build_gif(&frames_dir, &gif_path).expect("encode control-eval GIF");
    let poster_frame = FRAME_COUNT - 1;
    image::open(frames_dir.join(format!("frame-{poster_frame:03}.png")))
        .expect("read control-eval poster frame")
        .save(media_dir.join("control-eval.png"))
        .expect("write control-eval poster");
    fs::remove_dir_all(&frames_dir).expect("remove control-eval frame directory");
    println!("rendered control-eval media to {}", gif_path.display());
}

/// Which plant answers the controller.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Plant {
    Kinematic,
    Dynamic,
}

/// Deterministic per-seed evaluation conditions.
fn seed_conditions(seed: u64) -> SeedConditions {
    let random = KeyedRandom::new(EVAL_ROOT_SEED, 0x4556_414C_5F52_4E45);
    SeedConditions {
        friction_coefficient: random.sample_f64(seed, 0, 0, 0.72, 0.95),
        initial_offset_m: random.sample_f64(seed, 0, 1, -1.5, 1.5),
        steering_lag_s: random.sample_f64(seed, 0, 2, 0.05, 0.18),
    }
}

/// Runs every seed on one plant.
fn evaluate_plant(course: &[Vec3], plant: Plant) -> Vec<SeedRun> {
    SEEDS
        .iter()
        .map(|seed| run_seed(course, *seed, plant))
        .collect()
}

/// Runs one closed-loop seed and computes its metrics.
fn run_seed(course: &[Vec3], seed: u64, plant: Plant) -> SeedRun {
    let conditions = seed_conditions(seed);
    let mut world = World::new();
    let vehicle = spawn_named(&mut world, "eval_car");
    let drive = AckermannDrive {
        wheelbase_m: VehicleDynamics::default().wheelbase_m(),
        max_speed_m_s: 30.0,
        max_steering_rad: 0.6,
        max_acceleration_m_s2: 5.0,
        max_deceleration_m_s2: 6.0,
        max_steering_rate_rad_s: 6.0,
        target_speed_m_s: CRUISE_SPEED_M_S,
        ..AckermannDrive::default()
    };
    let start = Transform3::from_translation_rotation(
        Vec3::new(0.0, 0.0, -conditions.initial_offset_m),
        Quat::IDENTITY,
    );
    match plant {
        Plant::Kinematic => {
            world
                .entity_mut(vehicle)
                .insert((drive, start, RigidBody::default()));
        }
        Plant::Dynamic => {
            world.entity_mut(vehicle).insert((
                drive,
                VehicleDynamics {
                    friction_coefficient: conditions.friction_coefficient,
                    steering_lag_s: conditions.steering_lag_s,
                    ..VehicleDynamics::default()
                },
                start,
                RigidBody::default(),
            ));
        }
    }

    let dt = SimDuration::from_seconds(Seconds::new(1.0 / SIM_HZ as f64));
    let dt_s = dt.as_seconds().value();
    let mut samples = Vec::new();
    let mut trail_m = Vec::with_capacity(FRAME_COUNT + 1);
    trail_m.push(start.translation);

    for frame in 0..FRAME_COUNT {
        for step in 0..SIM_STEPS_PER_FRAME {
            let transform = *world.get::<Transform3>(vehicle).expect("transform");
            let target = lookahead_target(course, transform.translation);
            let steering = pure_pursuit_steering(
                &transform,
                target,
                VehicleDynamics::default().wheelbase_m(),
                LOOKAHEAD_M,
            );
            let course_end = *course.last().expect("course has waypoints");
            let remaining_m = (course_end - transform.translation).length();
            let current_speed = world
                .get::<AckermannDrive>(vehicle)
                .expect("drive")
                .speed_m_s;
            let braking_distance_m = current_speed * current_speed / (2.0 * 6.0);
            {
                let mut drive = world.get_mut::<AckermannDrive>(vehicle).expect("drive");
                drive.target_steering_rad =
                    steering.clamp(-drive.max_steering_rad, drive.max_steering_rad);
                drive.target_speed_m_s = if remaining_m < braking_distance_m + LOOKAHEAD_M {
                    0.0
                } else {
                    CRUISE_SPEED_M_S
                };
            }

            ackermann_kinematics(&mut world, dt);
            vehicle_dynamics(&mut world, dt);

            let transform = *world.get::<Transform3>(vehicle).expect("transform");
            let saturated = world
                .get::<VehicleDynamics>(vehicle)
                .map(|dynamics| dynamics.front_saturated || dynamics.rear_saturated)
                .unwrap_or(false);
            let command = world
                .get::<AckermannDrive>(vehicle)
                .expect("drive")
                .steering_rad;
            samples.push(ControlTrackingSample {
                time_s: (frame * SIM_STEPS_PER_FRAME + step + 1) as f64 * dt_s,
                tracking_error_m: course_distance_m(course, transform.translation),
                command,
                saturated,
                // Leaving the course corridor entirely counts as a violation.
                violation: course_distance_m(course, transform.translation) > 6.0,
            });
        }
        trail_m.push(
            world
                .get::<Transform3>(vehicle)
                .expect("transform")
                .translation,
        );
    }

    let metrics = ControlMetrics::from_samples(&samples, SETTLING_BAND_M).expect("enough samples");
    SeedRun {
        seed,
        conditions,
        metrics,
        trail_m,
    }
}

fn build_report(runs: &[SeedRun], controller: &str) -> ControlEvalReport {
    let seeds: BTreeMap<u64, ControlMetrics> =
        runs.iter().map(|run| (run.seed, run.metrics)).collect();
    ControlEvalReport::from_seed_metrics("sweeper_course", controller, seeds)
}

/// Same course family as example 49: approach, constant-radius sweeper, exit.
fn course_waypoints() -> Vec<Vec3> {
    let mut points = Vec::new();
    for index in 0..=8 {
        points.push(Vec3::new(index as f64 * 5.0, 0.0, 0.0));
    }
    let radius = 18.0;
    let center = Vec3::new(40.0, 0.0, -radius);
    for index in 1..=12 {
        let angle = index as f64 / 12.0 * std::f64::consts::PI;
        points.push(center + Vec3::new(radius * angle.sin(), 0.0, radius * angle.cos()));
    }
    for index in 1..=8 {
        points.push(Vec3::new(40.0 - index as f64 * 5.0, 0.0, -2.0 * radius));
    }
    points
}

fn course_distance_m(course: &[Vec3], point: Vec3) -> f64 {
    course
        .windows(2)
        .map(|segment| {
            let (a, b) = (segment[0], segment[1]);
            let ab = b - a;
            let t = ((point - a).dot(ab) / ab.length_squared()).clamp(0.0, 1.0);
            (point - (a + ab * t)).length()
        })
        .fold(f64::INFINITY, f64::min)
}

fn lookahead_target(course: &[Vec3], position: Vec3) -> Vec3 {
    let mut best_index = 0;
    let mut best_distance = f64::INFINITY;
    for (index, waypoint) in course.iter().enumerate() {
        let distance = (position - *waypoint).length();
        if distance < best_distance {
            best_distance = distance;
            best_index = index;
        }
    }
    for waypoint in course.iter().skip(best_index) {
        if (*waypoint - position).length() >= LOOKAHEAD_M {
            return *waypoint;
        }
    }
    *course.last().expect("course has waypoints")
}

fn overview_camera() -> CameraOrbit {
    CameraOrbit {
        focus: Vec3::new(34.0, 0.0, -24.0),
        yaw_rad: 0.25,
        pitch_rad: 0.45,
        distance_m: 80.0,
    }
}

/// Renders every dynamic seed's trail fanning out over the shared course.
fn evaluation_scene(course: &[Vec3], runs: &[SeedRun], frame: usize) -> RenderScene {
    const COURSE_COLOR: [f32; 4] = [0.45, 0.48, 0.55, 1.0];
    const GROUND_COLOR: [f32; 4] = [0.13, 0.15, 0.19, 1.0];
    // Seed trail palette cycles through distinguishable hues.
    const SEED_COLORS: [[f32; 4]; 5] = [
        [1.0, 0.55, 0.10, 1.0],
        [0.10, 0.75, 0.95, 1.0],
        [0.90, 0.30, 0.80, 1.0],
        [0.65, 0.90, 0.20, 1.0],
        [0.95, 0.80, 0.15, 1.0],
    ];

    let mut scene = RenderScene::new();
    scene.items.push(box_item(
        Vec3::new(34.0, -0.35, -24.0),
        Vec3::new(420.0, 0.2, 420.0),
        GROUND_COLOR,
    ));
    for waypoint in course {
        scene.items.push(box_item(
            *waypoint + Vec3::new(0.0, 0.02, 0.0),
            Vec3::new(0.9, 0.08, 0.9),
            COURSE_COLOR,
        ));
    }

    for (run_index, run) in runs.iter().enumerate() {
        let color = SEED_COLORS[run_index % SEED_COLORS.len()];
        let visible = frame.min(run.trail_m.len().saturating_sub(1));
        // Decimate the trail: ten seeds at full frame resolution would exceed the
        // renderer's scene item budget, and every fourth pose still reads as a line.
        for position in run.trail_m[..=visible].iter().step_by(4) {
            scene.items.push(box_item(
                *position + Vec3::new(0.0, 0.12, 0.0),
                Vec3::splat(0.65),
                color,
            ));
        }
        if let Some(current) = run.trail_m.get(visible) {
            scene.items.push(box_item(
                *current + Vec3::new(0.0, 0.7, 0.0),
                Vec3::new(1.2, 1.1, 0.8),
                color,
            ));
        }
    }

    scene
}

fn box_item(center_m: Vec3, size_m: Vec3, color: [f32; 4]) -> RenderSceneItem {
    RenderScene::item_from_visual(
        Transform3::from_translation_rotation(center_m, Quat::IDENTITY),
        VisualShape::Box { size_m },
        color,
        Transform3::IDENTITY,
    )
}

fn write_png(path: &Path, rgba: &[u8], width: u32, height: u32) -> std::io::Result<()> {
    let file = fs::File::create(path)?;
    let mut encoder = Encoder::new(file, width, height);
    encoder.set_color(ColorType::Rgba);
    encoder.set_depth(BitDepth::Eight);
    let mut writer = encoder.write_header()?;
    writer.write_image_data(rgba).map_err(std::io::Error::other)
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
            "fps=12,scale=960:-1:flags=lanczos,split[s0][s1];[s0]palettegen=max_colors=224:stats_mode=diff[p];[s1][p]paletteuse=dither=bayer:bayer_scale=5:diff_mode=rectangle",
            &gif_path.to_string_lossy(),
        ])
        .status()?;
    status
        .success()
        .then_some(())
        .ok_or_else(|| std::io::Error::other("ffmpeg control-eval GIF encode failed"))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn seed_conditions_are_deterministic_and_bounded() {
        for seed in SEEDS {
            let first = seed_conditions(seed);
            let second = seed_conditions(seed);
            assert_eq!(first, second);
            assert!((0.72..=0.95).contains(&first.friction_coefficient));
            assert!((-1.5..=1.5).contains(&first.initial_offset_m));
            assert!((0.05..=0.18).contains(&first.steering_lag_s));
        }
        // Seeds actually differ from each other.
        assert_ne!(seed_conditions(0), seed_conditions(1));
    }

    #[test]
    fn evaluation_is_deterministic() {
        let course = course_waypoints();
        let first = run_seed(&course, 3, Plant::Dynamic);
        let second = run_seed(&course, 3, Plant::Dynamic);
        assert_eq!(first, second);
    }

    #[test]
    fn kinematic_plant_flatters_the_controller() {
        let course = course_waypoints();
        let kinematic = build_report(&evaluate_plant(&course, Plant::Kinematic), "kinematic");
        let dynamic = build_report(&evaluate_plant(&course, Plant::Dynamic), "dynamic");

        assert!(kinematic.rms_error_m.mean < dynamic.rms_error_m.mean);
        // The kinematic plant cannot saturate a tire, by construction.
        assert_eq!(kinematic.saturated_fraction.mean, 0.0);
        assert!(dynamic.saturated_fraction.mean > 0.0);
    }

    #[test]
    fn dynamic_metrics_react_to_the_randomized_conditions() {
        let course = course_waypoints();
        let runs = evaluate_plant(&course, Plant::Dynamic);
        let rms: Vec<f64> = runs.iter().map(|run| run.metrics.rms_error_m).collect();
        let spread = rms.iter().cloned().fold(f64::NEG_INFINITY, f64::max)
            - rms.iter().cloned().fold(f64::INFINITY, f64::min);
        // Friction and lag differences must be visible in the metrics.
        assert!(spread > 0.01, "seed conditions changed nothing: {rms:?}");
    }

    #[test]
    fn report_serializes_with_all_seeds() {
        let course = course_waypoints();
        let report = build_report(&evaluate_plant(&course, Plant::Dynamic), "dynamic");
        assert_eq!(report.seeds.len(), SEEDS.len());
        let json = report.to_json_pretty().unwrap();
        assert!(json.contains("rms_error_m"));
    }
}
