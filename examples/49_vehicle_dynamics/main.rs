//! Kinematic versus dynamic vehicle model under the same controller.
//!
//! Two identical vehicles follow the same waypoint course with the same pure-pursuit
//! controller and the same speed profile. One uses the no-slip kinematic bicycle
//! ([`rne_robot::ackermann_kinematics`]); the other carries [`VehicleDynamics`] and
//! answers the same commands through tire forces. At low speed the two lines overlap.
//! In the fast corner the dynamic car's front axle saturates and it runs wide —
//! understeer the kinematic model cannot produce, which is exactly the gap a controller
//! evaluation must include.
//!
//! ```bash
//! cargo run --release -p vehicle_dynamics_compare --example 49_vehicle_dynamics
//! RNE_SKIP_GPU=1 cargo run -p vehicle_dynamics_compare --example 49_vehicle_dynamics
//! ```

use png::{BitDepth, ColorType, Encoder};
use rne_core::SimDuration;
use rne_ecs::{spawn_named, World};
use rne_math::{Quat, Seconds, Vec3};
use rne_physics::RigidBody;
use rne_render::{Camera, RenderBackend, RenderScene, RenderSceneItem, VisualShape};
use rne_render_wgpu::{CameraOrbit, WgpuRenderBackend};
use rne_robot::{
    ackermann_kinematics, pure_pursuit_steering, vehicle_dynamics, AckermannDrive, VehicleDynamics,
};
use rne_world::Transform3;
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

/// Course speed on the straights and through the fast corner, in meters per second.
///
/// Chosen so the sweeper is beyond the dynamic car's grip but recoverably so: at this
/// speed the friction-limited turn radius (`v^2 / (mu g)` ~ 22 m) exceeds the course
/// radius (18 m), so the dynamic car must run wide — yet it can rejoin on the exit.
const CRUISE_SPEED_M_S: f64 = 14.0;
/// Pure-pursuit lookahead in meters.
const LOOKAHEAD_M: f64 = 6.0;

/// One recorded pose of both vehicles.
#[derive(Clone, Copy, Debug, PartialEq)]
struct ComparisonSample {
    kinematic_position_m: Vec3,
    dynamic_position_m: Vec3,
    front_saturated: bool,
}

/// Result of one paired run.
#[derive(Clone, Debug, PartialEq)]
struct ComparisonRun {
    samples: Vec<ComparisonSample>,
    maximum_gap_m: f64,
    saturated_steps: usize,
    kinematic_course_error_m: f64,
    dynamic_course_error_m: f64,
}

fn main() {
    let course = course_waypoints();
    let run = run_comparison(&course);

    assert_eq!(run.samples.len(), FRAME_COUNT + 1);
    assert!(
        run.kinematic_course_error_m < 1.0,
        "the kinematic car must hold the course, got {:.2} m",
        run.kinematic_course_error_m
    );
    assert!(
        run.saturated_steps > 0,
        "the fast corner must saturate the dynamic front axle"
    );
    assert!(
        run.maximum_gap_m > 2.0,
        "understeer must open a visible gap, got {:.2} m",
        run.maximum_gap_m
    );

    println!(
        "vehicle dynamics comparison ready: frames={} cruise_m_s={} max_gap_m={:.2} saturated_steps={} kinematic_course_error_m={:.2} dynamic_course_error_m={:.2}",
        run.samples.len(),
        CRUISE_SPEED_M_S,
        run.maximum_gap_m,
        run.saturated_steps,
        run.kinematic_course_error_m,
        run.dynamic_course_error_m,
    );

    if std::env::var("RNE_SKIP_GPU").is_ok() {
        println!("RNE_SKIP_GPU set; headless vehicle dynamics comparison completed");
        return;
    }

    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
    let frames_dir = root.join("target/vehicle-dynamics/frames");
    let media_dir = root.join("docs/media");
    if frames_dir.exists() {
        fs::remove_dir_all(&frames_dir).expect("remove old vehicle dynamics frames");
    }
    fs::create_dir_all(&frames_dir).expect("create vehicle dynamics frame directory");
    fs::create_dir_all(&media_dir).expect("create media directory");

    let mut backend = match WgpuRenderBackend::new() {
        Ok(backend) => backend,
        Err(error) => {
            eprintln!("wgpu unavailable after successful headless run: {error}");
            return;
        }
    };
    let camera = Camera::new(WIDTH, HEIGHT, 0.9);
    let orbit = overview_camera();

    for frame in 0..FRAME_COUNT {
        let scene = comparison_scene(&course, &run, frame);
        let output = backend
            .render_scene_camera(&camera, &orbit.camera_transform(), &scene, CLEAR_COLOR)
            .expect("render vehicle dynamics frame");
        write_png(
            &frames_dir.join(format!("frame-{frame:03}.png")),
            &output.color.rgba8,
            output.color.width,
            output.color.height,
        )
        .expect("write vehicle dynamics frame");
    }

    let gif_path = media_dir.join("vehicle-dynamics.gif");
    build_gif(&frames_dir, &gif_path).expect("encode vehicle dynamics GIF");
    let poster_frame = FRAME_COUNT - 1;
    image::open(frames_dir.join(format!("frame-{poster_frame:03}.png")))
        .expect("read vehicle dynamics poster frame")
        .save(media_dir.join("vehicle-dynamics.png"))
        .expect("write vehicle dynamics poster");
    fs::remove_dir_all(&frames_dir).expect("remove vehicle dynamics frame directory");
    println!("rendered vehicle dynamics media to {}", gif_path.display());
}

/// Waypoints of the course: a long approach, a fast sweeper, and an exit straight.
fn course_waypoints() -> Vec<Vec3> {
    let mut points = Vec::new();
    // Approach straight along +X.
    for index in 0..=8 {
        points.push(Vec3::new(index as f64 * 5.0, 0.0, 0.0));
    }
    // Constant-radius left sweeper, tight enough to exceed the friction limit at speed.
    let radius = 18.0;
    let center = Vec3::new(40.0, 0.0, -radius);
    for index in 1..=12 {
        let angle = index as f64 / 12.0 * std::f64::consts::PI;
        points.push(center + Vec3::new(radius * angle.sin(), 0.0, radius * angle.cos()));
    }
    // Exit straight back along -X.
    for index in 1..=8 {
        points.push(Vec3::new(40.0 - index as f64 * 5.0, 0.0, -2.0 * radius));
    }
    points
}

/// Returns the closest distance from a point to the polyline course.
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

/// Advances a lookahead target along the course for pure pursuit.
fn lookahead_target(course: &[Vec3], position: Vec3) -> Vec3 {
    // Find the closest vertex, then walk forward until the lookahead distance opens.
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

fn race_drive() -> AckermannDrive {
    AckermannDrive {
        wheelbase_m: VehicleDynamics::default().wheelbase_m(),
        max_speed_m_s: 30.0,
        max_steering_rad: 0.6,
        max_acceleration_m_s2: 5.0,
        max_deceleration_m_s2: 6.0,
        max_steering_rate_rad_s: 6.0,
        target_speed_m_s: CRUISE_SPEED_M_S,
        ..AckermannDrive::default()
    }
}

/// Runs both vehicles through the course under the identical controller.
fn run_comparison(course: &[Vec3]) -> ComparisonRun {
    let mut world = World::new();
    let kinematic = spawn_named(&mut world, "kinematic_car");
    world
        .entity_mut(kinematic)
        .insert((race_drive(), Transform3::IDENTITY, RigidBody::default()));
    let dynamic = spawn_named(&mut world, "dynamic_car");
    world.entity_mut(dynamic).insert((
        race_drive(),
        VehicleDynamics::default(),
        Transform3::IDENTITY,
        RigidBody::default(),
    ));

    let dt = SimDuration::from_seconds(Seconds::new(1.0 / SIM_HZ as f64));
    let mut samples = Vec::with_capacity(FRAME_COUNT + 1);
    let mut maximum_gap_m = 0.0_f64;
    let mut saturated_steps = 0_usize;
    let mut kinematic_course_error_m = 0.0_f64;
    let mut dynamic_course_error_m = 0.0_f64;

    let record = |world: &World, saturated: &mut usize| -> ComparisonSample {
        let dynamics = world.get::<VehicleDynamics>(dynamic).expect("dynamics");
        if dynamics.front_saturated {
            *saturated += 1;
        }
        ComparisonSample {
            kinematic_position_m: world
                .get::<Transform3>(kinematic)
                .expect("kinematic transform")
                .translation,
            dynamic_position_m: world
                .get::<Transform3>(dynamic)
                .expect("dynamic transform")
                .translation,
            front_saturated: dynamics.front_saturated,
        }
    };

    samples.push(record(&world, &mut saturated_steps));

    for _frame in 0..FRAME_COUNT {
        for _ in 0..SIM_STEPS_PER_FRAME {
            // Identical controller for both vehicles: pure pursuit toward the same
            // course with the same lookahead and cruise speed.
            for vehicle in [kinematic, dynamic] {
                let transform = *world.get::<Transform3>(vehicle).expect("transform");
                let target = lookahead_target(course, transform.translation);
                let steering = pure_pursuit_steering(
                    &transform,
                    target,
                    VehicleDynamics::default().wheelbase_m(),
                    LOOKAHEAD_M,
                );
                // Brake to a stop as the final waypoint approaches so neither car
                // drives off the end of the course. The trigger distance is the
                // physical stopping distance at the current speed, not a constant.
                let course_end = *course.last().expect("course has waypoints");
                let remaining_m = (course_end - transform.translation).length();
                let current_speed = world
                    .get::<AckermannDrive>(vehicle)
                    .expect("drive")
                    .speed_m_s;
                let braking_distance_m =
                    current_speed * current_speed / (2.0 * race_drive().max_deceleration_m_s2);
                let target_speed = if remaining_m < braking_distance_m + LOOKAHEAD_M {
                    0.0
                } else {
                    CRUISE_SPEED_M_S
                };
                let mut drive = world.get_mut::<AckermannDrive>(vehicle).expect("drive");
                let clamped = steering.clamp(-drive.max_steering_rad, drive.max_steering_rad);
                drive.target_steering_rad = clamped;
                drive.target_speed_m_s = target_speed;
            }

            ackermann_kinematics(&mut world, dt);
            vehicle_dynamics(&mut world, dt);
        }

        let sample = record(&world, &mut saturated_steps);
        if std::env::var("RNE_DEBUG_TRACE").is_ok() {
            println!(
                "frame={_frame:03} kin=({:6.1},{:6.1}) dyn=({:6.1},{:6.1}) kin_err={:5.2} sat={}",
                sample.kinematic_position_m.x,
                sample.kinematic_position_m.z,
                sample.dynamic_position_m.x,
                sample.dynamic_position_m.z,
                course_distance_m(course, sample.kinematic_position_m),
                sample.front_saturated,
            );
        }
        maximum_gap_m =
            maximum_gap_m.max((sample.dynamic_position_m - sample.kinematic_position_m).length());
        kinematic_course_error_m =
            kinematic_course_error_m.max(course_distance_m(course, sample.kinematic_position_m));
        dynamic_course_error_m =
            dynamic_course_error_m.max(course_distance_m(course, sample.dynamic_position_m));
        samples.push(sample);
    }

    ComparisonRun {
        samples,
        maximum_gap_m,
        saturated_steps,
        kinematic_course_error_m,
        dynamic_course_error_m,
    }
}

fn overview_camera() -> CameraOrbit {
    CameraOrbit {
        focus: Vec3::new(34.0, 0.0, -24.0),
        yaw_rad: 0.25,
        pitch_rad: 0.45,
        distance_m: 80.0,
    }
}

/// Builds the scene for one frame: course line, both trails, and current poses.
fn comparison_scene(course: &[Vec3], run: &ComparisonRun, frame: usize) -> RenderScene {
    const COURSE_COLOR: [f32; 4] = [0.45, 0.48, 0.55, 1.0];
    const KINEMATIC_COLOR: [f32; 4] = [0.10, 0.85, 0.55, 1.0];
    const DYNAMIC_COLOR: [f32; 4] = [1.0, 0.55, 0.10, 1.0];
    const SATURATED_COLOR: [f32; 4] = [0.95, 0.20, 0.30, 1.0];
    const GROUND_COLOR: [f32; 4] = [0.13, 0.15, 0.19, 1.0];

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

    let visible = frame.min(run.samples.len().saturating_sub(1));
    for sample in &run.samples[..=visible] {
        scene.items.push(box_item(
            sample.kinematic_position_m + Vec3::new(0.0, 0.15, 0.0),
            Vec3::splat(0.5),
            KINEMATIC_COLOR,
        ));
        // The dynamic trail turns red wherever the front axle is beyond its grip.
        let trail_color = if sample.front_saturated {
            SATURATED_COLOR
        } else {
            DYNAMIC_COLOR
        };
        scene.items.push(box_item(
            sample.dynamic_position_m + Vec3::new(0.0, 0.15, 0.0),
            Vec3::splat(0.5),
            trail_color,
        ));
    }

    if let Some(current) = run.samples.get(visible) {
        scene.items.push(box_item(
            current.kinematic_position_m + Vec3::new(0.0, 0.9, 0.0),
            Vec3::new(1.6, 1.4, 1.0),
            KINEMATIC_COLOR,
        ));
        scene.items.push(box_item(
            current.dynamic_position_m + Vec3::new(0.0, 0.9, 0.0),
            Vec3::new(1.6, 1.4, 1.0),
            if current.front_saturated {
                SATURATED_COLOR
            } else {
                DYNAMIC_COLOR
            },
        ));
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
        .ok_or_else(|| std::io::Error::other("ffmpeg vehicle dynamics GIF encode failed"))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn course_is_continuous_and_bends_left() {
        let course = course_waypoints();
        assert!(course.len() > 20);
        // No two consecutive waypoints coincide.
        assert!(course
            .windows(2)
            .all(|pair| (pair[1] - pair[0]).length() > 1.0));
        // The sweeper carries the course to negative Z.
        assert!(course.last().unwrap().z < -30.0);
    }

    #[test]
    fn identical_controller_diverges_only_through_the_vehicle_model() {
        let course = course_waypoints();
        let run = run_comparison(&course);

        // The kinematic car tracks the course; the dynamic car understeers wide.
        assert!(run.kinematic_course_error_m < 1.0);
        assert!(run.dynamic_course_error_m > run.kinematic_course_error_m);
        assert!(run.maximum_gap_m > 2.0);
        assert!(run.saturated_steps > 0);
    }

    #[test]
    fn comparison_is_deterministic() {
        let course = course_waypoints();
        let first = run_comparison(&course);
        let second = run_comparison(&course);
        assert_eq!(first, second);
    }

    #[test]
    fn scene_marks_saturated_stretches() {
        let course = course_waypoints();
        let run = run_comparison(&course);
        assert!(run.samples.iter().any(|sample| sample.front_saturated));
        let scene = comparison_scene(&course, &run, FRAME_COUNT);
        assert!(scene.items.len() > course.len() + 2);
    }
}
