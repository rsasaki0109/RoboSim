//! Sensor latency inside the control loop.
//!
//! Every earlier closed-loop example let the controller read the vehicle pose straight
//! out of the simulator — data a real controller could never have. Here the pose
//! travels through the DataBus: a localization source publishes `PoseSample` frames
//! stamped with transport latency, and the pure-pursuit controller may only read what
//! [`DataBus::latest_available`] says has arrived. The controller steers the present
//! vehicle from a pose of the past.
//!
//! Three otherwise identical runs differ only in that latency, and the result has the
//! shape feedback delay actually has: it is a threshold, not a linear tax. 0 ms and
//! 120 ms both sit inside the loop's phase margin and track almost identically;
//! 240 ms exceeds it and the loop never settles again — measured with
//! `rne_ai::control_eval` metrics.
//!
//! ```bash
//! cargo run --release -p latency_in_the_loop --example 51_latency_in_the_loop
//! RNE_SKIP_GPU=1 cargo run -p latency_in_the_loop --example 51_latency_in_the_loop
//! ```

use png::{BitDepth, ColorType, Encoder};
use rne_ai::{ControlMetrics, ControlTrackingSample};
use rne_core::{SimDuration, SimTime};
use rne_data::{DataBus, Frame, InMemoryDataBus, PoseSample, StreamId};
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

/// Cruise speed; inside the tire grip so any degradation is latency, not saturation.
const CRUISE_SPEED_M_S: f64 = 12.0;
/// Pure-pursuit lookahead in meters.
///
/// Deliberately tight, but stable without delay. A long lookahead is a strong phase
/// lead and hides moderate feedback delay; at 5 m the vehicle covers most of the
/// lookahead within the largest evaluated latency (12 m/s x 240 ms = 2.9 m), which is
/// the regime where delayed feedback turns into oscillation while on-time feedback
/// still tracks cleanly.
const LOOKAHEAD_M: f64 = 5.0;
/// Initial lateral offset in meters, so every run starts with a step to recover.
const INITIAL_OFFSET_M: f64 = 1.5;
/// Settling band for the tracking metrics, in meters.
const SETTLING_BAND_M: f64 = 0.5;
/// Localization publish rate in hertz.
const LOCALIZATION_HZ: usize = 60;
const LOCALIZATION_STREAM: StreamId = StreamId(51_100);
/// Evaluated localization latencies in milliseconds.
const LATENCIES_MS: [u64; 3] = [0, 120, 240];

/// One evaluated latency: its trail and metrics.
#[derive(Clone, Debug, PartialEq)]
struct LatencyRun {
    latency_ms: u64,
    metrics: ControlMetrics,
    trail_m: Vec<Vec3>,
}

fn main() {
    let course = course_waypoints();
    let runs: Vec<LatencyRun> = LATENCIES_MS
        .iter()
        .map(|latency_ms| run_latency(&course, *latency_ms))
        .collect();

    println!(
        "latency in the loop ready: cruise_m_s={CRUISE_SPEED_M_S} localization_hz={LOCALIZATION_HZ}"
    );
    for run in &runs {
        println!(
            "  {:>3} ms: rms={:.3} m  max={:.3} m  smoothness={:.2}  settled={}",
            run.latency_ms,
            run.metrics.rms_error_m,
            run.metrics.max_error_m,
            run.metrics.smoothness,
            run.metrics.settling_time_s.is_some(),
        );
    }

    // Feedback delay is a threshold phenomenon, not a linear tax: inside the loop's
    // phase margin it costs almost nothing, past it the loop goes unstable. The
    // asserts encode exactly that shape.
    let on_time = &runs[0].metrics;
    let moderate = &runs[1].metrics;
    let heavy = &runs[2].metrics;
    assert!(on_time.settling_time_s.is_some() && on_time.rms_error_m < 1.0);
    assert!(moderate.settling_time_s.is_some() && moderate.rms_error_m < 1.0);
    assert!(
        heavy.settling_time_s.is_none(),
        "past the phase margin the loop must never settle"
    );
    assert!(
        heavy.rms_error_m > on_time.rms_error_m * 2.0,
        "unstable tracking must dwarf the on-time loop: {:.3} vs {:.3}",
        heavy.rms_error_m,
        on_time.rms_error_m
    );

    if std::env::var("RNE_SKIP_GPU").is_ok() {
        println!("RNE_SKIP_GPU set; headless latency evaluation completed");
        return;
    }

    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
    let frames_dir = root.join("target/latency-loop/frames");
    let media_dir = root.join("docs/media");
    if frames_dir.exists() {
        fs::remove_dir_all(&frames_dir).expect("remove old latency frames");
    }
    fs::create_dir_all(&frames_dir).expect("create latency frame directory");
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
        let scene = latency_scene(&course, &runs, frame);
        let output = backend
            .render_scene_camera(&camera, &orbit.camera_transform(), &scene, CLEAR_COLOR)
            .expect("render latency frame");
        write_png(
            &frames_dir.join(format!("frame-{frame:03}.png")),
            &output.color.rgba8,
            output.color.width,
            output.color.height,
        )
        .expect("write latency frame");
    }

    let gif_path = media_dir.join("latency-loop.gif");
    build_gif(&frames_dir, &gif_path).expect("encode latency GIF");
    let poster_frame = FRAME_COUNT - 1;
    image::open(frames_dir.join(format!("frame-{poster_frame:03}.png")))
        .expect("read latency poster frame")
        .save(media_dir.join("latency-loop.png"))
        .expect("write latency poster");
    fs::remove_dir_all(&frames_dir).expect("remove latency frame directory");
    println!("rendered latency media to {}", gif_path.display());
}

/// Runs one closed loop whose pose feedback carries the given latency.
fn run_latency(course: &[Vec3], latency_ms: u64) -> LatencyRun {
    let mut world = World::new();
    let vehicle = spawn_named(&mut world, "latency_car");
    world.entity_mut(vehicle).insert((
        AckermannDrive {
            wheelbase_m: VehicleDynamics::default().wheelbase_m(),
            max_speed_m_s: 30.0,
            max_steering_rad: 0.6,
            max_acceleration_m_s2: 5.0,
            max_deceleration_m_s2: 6.0,
            max_steering_rate_rad_s: 6.0,
            target_speed_m_s: CRUISE_SPEED_M_S,
            ..AckermannDrive::default()
        },
        VehicleDynamics::default(),
        Transform3::from_translation_rotation(
            Vec3::new(0.0, 0.0, -INITIAL_OFFSET_M),
            Quat::IDENTITY,
        ),
        RigidBody::default(),
    ));

    let mut bus = InMemoryDataBus::new();
    let latency = SimDuration::from_seconds(Seconds::new(latency_ms as f64 / 1_000.0));
    let dt = SimDuration::from_seconds(Seconds::new(1.0 / SIM_HZ as f64));
    let dt_s = dt.as_seconds().value();
    let publish_every = SIM_HZ / LOCALIZATION_HZ;

    let mut samples = Vec::new();
    let mut trail_m = vec![Vec3::new(0.0, 0.0, -INITIAL_OFFSET_M)];
    let mut sequence = 0_u64;

    for frame in 0..FRAME_COUNT {
        for step in 0..SIM_STEPS_PER_FRAME {
            let tick = frame * SIM_STEPS_PER_FRAME + step;
            let now = SimTime::from_seconds(Seconds::new(tick as f64 * dt_s));
            let truth = *world.get::<Transform3>(vehicle).expect("transform");

            // The localization source samples the true pose and publishes it with
            // transport latency, exactly like a real estimator output.
            if tick.is_multiple_of(publish_every) {
                sequence += 1;
                let forward = truth.rotation * Vec3::X;
                bus.publish(
                    Frame::new(
                        LOCALIZATION_STREAM,
                        vehicle,
                        sequence,
                        now,
                        PoseSample {
                            position_m: truth.translation,
                            yaw_rad: (-forward.z).atan2(forward.x),
                        },
                    )
                    .with_latency(latency),
                );
            }

            // The controller may only act on what has arrived. Until the first frame
            // lands it holds straight — a real system's cold-start behaviour.
            if let Some(pose) = bus.latest_available::<PoseSample>(LOCALIZATION_STREAM, now) {
                let believed = Transform3::from_translation_rotation(
                    pose.payload.position_m,
                    Quat::from_rotation_y(pose.payload.yaw_rad),
                );
                let target = lookahead_target(course, believed.translation);
                let steering = pure_pursuit_steering(
                    &believed,
                    target,
                    VehicleDynamics::default().wheelbase_m(),
                    LOOKAHEAD_M,
                );
                let mut drive = world.get_mut::<AckermannDrive>(vehicle).expect("drive");
                drive.target_steering_rad =
                    steering.clamp(-drive.max_steering_rad, drive.max_steering_rad);
            }

            ackermann_kinematics(&mut world, dt);
            vehicle_dynamics(&mut world, dt);

            let after = *world.get::<Transform3>(vehicle).expect("transform");
            let dynamics = world
                .get::<VehicleDynamics>(vehicle)
                .expect("vehicle dynamics");
            let command = world
                .get::<AckermannDrive>(vehicle)
                .expect("drive")
                .steering_rad;
            samples.push(ControlTrackingSample {
                time_s: (tick + 1) as f64 * dt_s,
                tracking_error_m: course_distance_m(course, after.translation),
                command,
                saturated: dynamics.front_saturated || dynamics.rear_saturated,
                violation: false,
            });
        }
        trail_m.push(
            world
                .get::<Transform3>(vehicle)
                .expect("transform")
                .translation,
        );
    }

    LatencyRun {
        latency_ms,
        metrics: ControlMetrics::from_samples(&samples, SETTLING_BAND_M).expect("enough samples"),
        trail_m,
    }
}

/// Same course family as examples 49 and 50.
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

/// Renders the three latency trails over the shared course.
fn latency_scene(course: &[Vec3], runs: &[LatencyRun], frame: usize) -> RenderScene {
    const COURSE_COLOR: [f32; 4] = [0.45, 0.48, 0.55, 1.0];
    const GROUND_COLOR: [f32; 4] = [0.13, 0.15, 0.19, 1.0];
    /// Green for on-time feedback, amber for moderate lag, red for heavy lag.
    const LATENCY_COLORS: [[f32; 4]; 3] = [
        [0.10, 0.85, 0.55, 1.0],
        [1.0, 0.75, 0.10, 1.0],
        [0.95, 0.20, 0.30, 1.0],
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
        let color = LATENCY_COLORS[run_index % LATENCY_COLORS.len()];
        let visible = frame.min(run.trail_m.len().saturating_sub(1));
        for position in run.trail_m[..=visible].iter().step_by(2) {
            scene.items.push(box_item(
                *position + Vec3::new(0.0, 0.12, 0.0),
                Vec3::splat(0.45),
                color,
            ));
        }
        if let Some(current) = run.trail_m.get(visible) {
            scene.items.push(box_item(
                *current + Vec3::new(0.0, 0.8, 0.0),
                Vec3::new(1.4, 1.2, 0.9),
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
        .ok_or_else(|| std::io::Error::other("ffmpeg latency GIF encode failed"))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn zero_latency_matches_a_direct_read_loop() {
        // With zero latency the bus path must not change behaviour: the frame
        // published this tick is available this tick.
        let course = course_waypoints();
        let run = run_latency(&course, 0);
        assert!(
            run.metrics.rms_error_m < 1.0,
            "on-time feedback must track, got {:.3} m",
            run.metrics.rms_error_m
        );
        assert!(run.metrics.settling_time_s.is_some());
    }

    #[test]
    fn latency_inside_the_phase_margin_is_nearly_free() {
        let course = course_waypoints();
        let on_time = run_latency(&course, 0);
        let moderate = run_latency(&course, LATENCIES_MS[1]);

        assert!(moderate.metrics.settling_time_s.is_some());
        // The moderate delay costs at most a small factor over on-time feedback.
        assert!(moderate.metrics.rms_error_m < on_time.metrics.rms_error_m * 1.5);
    }

    #[test]
    fn latency_past_the_phase_margin_destabilizes_the_loop() {
        let course = course_waypoints();
        let on_time = run_latency(&course, 0);
        let heavy = run_latency(&course, LATENCIES_MS[2]);

        assert!(heavy.metrics.settling_time_s.is_none());
        assert!(heavy.metrics.rms_error_m > on_time.metrics.rms_error_m * 2.0);
        assert!(heavy.metrics.max_error_m > on_time.metrics.max_error_m * 2.0);
    }

    #[test]
    fn latency_runs_are_deterministic() {
        let course = course_waypoints();
        assert_eq!(run_latency(&course, 120), run_latency(&course, 120));
    }
}
