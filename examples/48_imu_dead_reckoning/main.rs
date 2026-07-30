//! IMU dead reckoning: how far an unaided inertial estimate drifts.
//!
//! A vehicle drives a known circular arc. Its IMU is sampled through the full
//! physics-aware error model — angle/velocity random walk, Gauss-Markov bias
//! instability, rate random walk, turn-on bias, scale factor, axis misalignment,
//! saturation and quantization — and the resulting measurements are integrated with a
//! textbook strapdown update. Nothing corrects the estimate, so the error it accumulates
//! is exactly the error the sensor model produces.
//!
//! ```bash
//! cargo run --release -p imu_dead_reckoning --example 48_imu_dead_reckoning
//! RNE_SKIP_GPU=1 cargo run -p imu_dead_reckoning --example 48_imu_dead_reckoning
//! ```

use png::{BitDepth, ColorType, Encoder};
use rne_core::{SimDuration, SimTime};
use rne_ecs::{spawn_named, Entity, World};
use rne_math::{Quat, Seconds, Transform3 as MathTransform3, Vec3};
use rne_physics::RigidBody;
use rne_render::{Camera, RenderBackend, RenderScene, RenderSceneItem, VisualShape};
use rne_render_wgpu::{CameraOrbit, WgpuRenderBackend};
use rne_sensor::{
    sample_imu_stateful, ImuAxisErrors, ImuSpec, ImuState, SensorNoiseKey, GRAVITY_M_S2,
};
use rne_world::Transform3;
use std::fs;
use std::path::Path;

const WIDTH: u32 = 1_280;
const HEIGHT: u32 = 720;
/// Rendered frames, twelve seconds at twelve frames per second.
const FRAME_COUNT: usize = 144;
const RENDER_HZ: usize = 12;
/// IMU rate; a real part runs far faster than the render loop.
///
/// This must stay an exact multiple of [`RENDER_HZ`], otherwise the integrated time and
/// the frame time diverge and the mismatch shows up as fake drift.
const IMU_HZ: usize = 240;
const IMU_STEPS_PER_FRAME: usize = IMU_HZ / RENDER_HZ;
const _: () = assert!(
    IMU_HZ.is_multiple_of(RENDER_HZ),
    "IMU rate must divide the frame rate"
);
const CLEAR_COLOR: [f32; 4] = [0.06, 0.07, 0.10, 1.0];

/// Arc radius in meters.
const TRACK_RADIUS_M: f64 = 20.0;
/// Constant ground speed in meters per second.
const TRACK_SPEED_M_S: f64 = 8.0;
const WORLD_SEED: u64 = 48;
const IMU_STREAM_ID: u64 = 48_100;

/// One integration sample of truth and estimate.
#[derive(Clone, Copy, Debug, PartialEq)]
struct DeadReckoningSample {
    time_s: f64,
    truth_position_m: Vec3,
    estimated_position_m: Vec3,
}

impl DeadReckoningSample {
    fn position_error_m(&self) -> f64 {
        (self.estimated_position_m - self.truth_position_m).length()
    }
}

/// Result of one dead-reckoning run.
#[derive(Clone, Debug, PartialEq)]
struct DeadReckoningRun {
    samples: Vec<DeadReckoningSample>,
    stable_hash: u64,
    final_error_m: f64,
    distance_travelled_m: f64,
}

impl DeadReckoningRun {
    /// Returns the final error as a fraction of distance travelled.
    fn error_fraction_of_distance(&self) -> f64 {
        if self.distance_travelled_m <= 0.0 {
            0.0
        } else {
            self.final_error_m / self.distance_travelled_m
        }
    }
}

fn main() {
    let truth_track = CircularTrack::default();
    let consumer = run_dead_reckoning(&truth_track, consumer_grade_imu());
    let ideal = run_dead_reckoning(&truth_track, ImuSpec::default());

    assert_eq!(consumer.samples.len(), FRAME_COUNT + 1);
    assert!(
        ideal.final_error_m < 0.5,
        "an ideal IMU must integrate back onto the truth, got {:.3} m",
        ideal.final_error_m
    );
    assert!(
        consumer.final_error_m > ideal.final_error_m,
        "the modeled IMU must drift further than an ideal one"
    );

    println!(
        "imu dead reckoning ready: imu_hz={} frames={} distance_m={:.1} ideal_error_m={:.3} modeled_error_m={:.2} drift_fraction={:.4} stable_hash={}",
        IMU_HZ,
        consumer.samples.len(),
        consumer.distance_travelled_m,
        ideal.final_error_m,
        consumer.final_error_m,
        consumer.error_fraction_of_distance(),
        consumer.stable_hash,
    );

    if std::env::var("RNE_SKIP_GPU").is_ok() {
        println!("RNE_SKIP_GPU set; headless IMU dead-reckoning run completed");
        return;
    }

    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
    let frames_dir = root.join("target/imu-dead-reckoning/frames");
    let media_dir = root.join("docs/media");
    if frames_dir.exists() {
        fs::remove_dir_all(&frames_dir).expect("remove old IMU frames");
    }
    fs::create_dir_all(&frames_dir).expect("create IMU frame directory");
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
        let scene = drift_scene(&consumer, frame);
        let output = backend
            .render_scene_camera(&camera, &orbit.camera_transform(), &scene, CLEAR_COLOR)
            .expect("render IMU drift frame");
        write_png(
            &frames_dir.join(format!("frame-{frame:03}.png")),
            &output.color.rgba8,
            output.color.width,
            output.color.height,
        )
        .expect("write IMU drift frame");
    }

    let gif_path = media_dir.join("imu-dead-reckoning.gif");
    build_gif(&frames_dir, &gif_path).expect("encode IMU drift GIF");
    let poster_frame = FRAME_COUNT - 1;
    image::open(frames_dir.join(format!("frame-{poster_frame:03}.png")))
        .expect("read IMU drift poster frame")
        .save(media_dir.join("imu-dead-reckoning.png"))
        .expect("write IMU drift poster");
    fs::remove_dir_all(&frames_dir).expect("remove IMU frame directory");
    println!(
        "rendered IMU dead-reckoning media to {}",
        gif_path.display()
    );
}

/// A constant-speed circular arc with a yaw rate that matches its heading.
#[derive(Clone, Copy, Debug)]
struct CircularTrack {
    radius_m: f64,
    speed_m_s: f64,
}

impl Default for CircularTrack {
    fn default() -> Self {
        Self {
            radius_m: TRACK_RADIUS_M,
            speed_m_s: TRACK_SPEED_M_S,
        }
    }
}

impl CircularTrack {
    /// Yaw rate in radians per second.
    fn yaw_rate_rad_s(&self) -> f64 {
        self.speed_m_s / self.radius_m
    }

    fn position_m(&self, time_s: f64) -> Vec3 {
        let angle = self.yaw_rate_rad_s() * time_s;
        Vec3::new(
            self.radius_m * angle.sin(),
            0.0,
            self.radius_m * angle.cos() - self.radius_m,
        )
    }

    fn velocity_m_s(&self, time_s: f64) -> Vec3 {
        let angle = self.yaw_rate_rad_s() * time_s;
        Vec3::new(
            self.speed_m_s * angle.cos(),
            0.0,
            -self.speed_m_s * angle.sin(),
        )
    }

    /// Body orientation; the vehicle faces along its velocity with `+X` forward.
    fn rotation(&self, time_s: f64) -> Quat {
        Quat::from_rotation_y(self.yaw_rate_rad_s() * time_s)
    }

    /// World-frame angular velocity, which is yaw only.
    fn angular_velocity_rad_s(&self) -> Vec3 {
        Vec3::new(0.0, self.yaw_rate_rad_s(), 0.0)
    }
}

/// Returns error parameters representative of a consumer-grade MEMS IMU.
fn consumer_grade_imu() -> ImuSpec {
    ImuSpec {
        seed: IMU_STREAM_ID,
        gyro: ImuAxisErrors {
            random_walk: 0.0022,
            bias_instability: 0.0016,
            bias_correlation_time_s: 20.0,
            rate_random_walk: 0.00025,
            turn_on_bias: Vec3::new(0.0009, -0.0013, 0.0006),
            scale_factor_error: Vec3::new(0.002, -0.0015, 0.0018),
            misalignment_rad: Vec3::new(0.0010, 0.0008, -0.0012),
        },
        accel: ImuAxisErrors {
            random_walk: 0.020,
            bias_instability: 0.022,
            bias_correlation_time_s: 60.0,
            rate_random_walk: 0.0022,
            turn_on_bias: Vec3::new(0.030, -0.040, 0.022),
            scale_factor_error: Vec3::new(0.003, 0.0025, -0.002),
            misalignment_rad: Vec3::new(0.0012, -0.0009, 0.0011),
        },
        gyro_range_rad_s: 8.7,
        accel_range_m_s2: 156.0,
        gyro_resolution_rad_s: 0.000_133,
        accel_resolution_m_s2: 0.002_4,
        ..ImuSpec::default()
    }
}

/// Integrates the modeled IMU with a strapdown update and no aiding.
fn run_dead_reckoning(track: &CircularTrack, spec: ImuSpec) -> DeadReckoningRun {
    let mut world = World::new();
    let sensor = spawn_named(&mut world, "imu");
    world
        .entity_mut(sensor)
        .insert((Transform3::IDENTITY, RigidBody::default()));

    let dt = SimDuration::from_hertz(rne_math::Hertz::new(IMU_HZ as f64));
    let dt_s = dt.as_seconds().value();
    let mut imu_state = ImuState::default();

    // The estimate starts perfectly aligned; everything after this is sensor error.
    let mut estimated_rotation = track.rotation(0.0);
    let mut estimated_velocity_m_s = track.velocity_m_s(0.0);
    let mut estimated_position_m = track.position_m(0.0);

    let mut samples = vec![DeadReckoningSample {
        time_s: 0.0,
        truth_position_m: track.position_m(0.0),
        estimated_position_m,
    }];
    let mut stable_hash = 0xcbf29ce484222325_u64;
    let mut distance_travelled_m = 0.0;
    let mut step = 0_u64;

    for frame in 1..=FRAME_COUNT {
        for _ in 0..IMU_STEPS_PER_FRAME {
            step += 1;
            let time_s = step as f64 * dt_s;
            let previous_time_s = time_s - dt_s;

            set_truth_state(&mut world, sensor, track, time_s);
            let sample = sample_imu_stateful(
                &world,
                sensor,
                &spec,
                SensorNoiseKey::new(WORLD_SEED, spec.seed, IMU_STREAM_ID, step),
                SimTime::from_seconds(Seconds::new(time_s)),
                &mut imu_state,
            );

            // Strapdown update: propagate attitude with the measured body rate, rotate
            // the specific force into the world frame, restore gravity, then integrate.
            // The accelerometer resolves its measurement in the body frame at the sample
            // instant, so the estimate rotates it back with the attitude at that same
            // instant. Position uses the trapezoid rule; a right-endpoint sum would add
            // integration error that reads as sensor drift.
            let delta = sample.angular_velocity_rad_s * dt_s;
            estimated_rotation = (estimated_rotation * small_angle_quat(delta)).normalize();

            let acceleration_world =
                estimated_rotation * sample.linear_acceleration_m_s2 + GRAVITY_M_S2;
            let previous_velocity_m_s = estimated_velocity_m_s;
            estimated_velocity_m_s += acceleration_world * dt_s;
            estimated_position_m += (previous_velocity_m_s + estimated_velocity_m_s) * 0.5 * dt_s;

            distance_travelled_m +=
                (track.position_m(time_s) - track.position_m(previous_time_s)).length();
            stable_hash = hash_vec3(stable_hash, sample.angular_velocity_rad_s);
            stable_hash = hash_vec3(stable_hash, sample.linear_acceleration_m_s2);
        }

        let time_s = frame as f64 / RENDER_HZ as f64;
        samples.push(DeadReckoningSample {
            time_s,
            truth_position_m: track.position_m(time_s),
            estimated_position_m,
        });
    }

    let final_error_m = samples
        .last()
        .map(DeadReckoningSample::position_error_m)
        .unwrap_or_default();

    DeadReckoningRun {
        samples,
        stable_hash,
        final_error_m,
        distance_travelled_m,
    }
}

/// Writes the analytic truth pose and velocities onto the sensor entity.
fn set_truth_state(world: &mut World, sensor: Entity, track: &CircularTrack, time_s: f64) {
    if let Some(mut transform) = world.get_mut::<Transform3>(sensor) {
        *transform =
            Transform3::from_translation_rotation(track.position_m(time_s), track.rotation(time_s));
    }
    if let Some(mut body) = world.get_mut::<RigidBody>(sensor) {
        body.linear_velocity_m_s = track.velocity_m_s(time_s);
        body.angular_velocity_rad_s = track.angular_velocity_rad_s();
    }
}

/// Returns the rotation for a small body-frame rotation vector.
fn small_angle_quat(delta_rad: Vec3) -> Quat {
    let angle = delta_rad.length();
    if angle <= f64::EPSILON {
        return Quat::IDENTITY;
    }
    Quat::from_axis_angle(delta_rad / angle, angle)
}

fn hash_vec3(mut hash: u64, value: Vec3) -> u64 {
    for component in [value.x, value.y, value.z] {
        for byte in component.to_bits().to_le_bytes() {
            hash ^= u64::from(byte);
            hash = hash.wrapping_mul(0x100000001b3);
        }
    }
    hash
}

/// Overview camera framing the whole arc from above.
///
/// The track is a circle of [`TRACK_RADIUS_M`] centred one radius behind the origin, so
/// the camera looks at that centre rather than at the start of the arc.
fn track_center_m() -> Vec3 {
    Vec3::new(0.0, 0.0, -TRACK_RADIUS_M)
}

fn overview_camera() -> CameraOrbit {
    CameraOrbit {
        focus: track_center_m(),
        yaw_rad: 0.35,
        pitch_rad: 0.42,
        distance_m: 58.0,
    }
}

/// Builds the scene for one frame: truth trail, estimate trail, and the error link.
fn drift_scene(run: &DeadReckoningRun, frame: usize) -> RenderScene {
    const TRUTH_COLOR: [f32; 4] = [0.10, 0.85, 0.55, 1.0];
    const ESTIMATE_COLOR: [f32; 4] = [1.0, 0.55, 0.10, 1.0];
    const ERROR_COLOR: [f32; 4] = [0.95, 0.20, 0.30, 1.0];
    const GROUND_COLOR: [f32; 4] = [0.13, 0.15, 0.19, 1.0];

    let mut scene = RenderScene::new();
    // The ground is wide enough that its edge never enters the framed view.
    scene.items.push(box_item(
        track_center_m() + Vec3::new(0.0, -0.35, 0.0),
        Vec3::new(220.0, 0.2, 220.0),
        Quat::IDENTITY,
        GROUND_COLOR,
    ));

    let visible = frame.min(run.samples.len().saturating_sub(1));
    for sample in &run.samples[..=visible] {
        scene.items.push(box_item(
            sample.truth_position_m + Vec3::new(0.0, 0.15, 0.0),
            Vec3::splat(0.55),
            Quat::IDENTITY,
            TRUTH_COLOR,
        ));
        scene.items.push(box_item(
            sample.estimated_position_m + Vec3::new(0.0, 0.15, 0.0),
            Vec3::splat(0.55),
            Quat::IDENTITY,
            ESTIMATE_COLOR,
        ));
    }

    if let Some(current) = run.samples.get(visible) {
        // Current poses stand taller than the trail so the head of each path reads.
        scene.items.push(box_item(
            current.truth_position_m + Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(1.1, 2.0, 1.1),
            Quat::IDENTITY,
            TRUTH_COLOR,
        ));
        scene.items.push(box_item(
            current.estimated_position_m + Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(1.1, 2.0, 1.1),
            Quat::IDENTITY,
            ESTIMATE_COLOR,
        ));
        scene.items.extend(error_link(
            current.truth_position_m,
            current.estimated_position_m,
            ERROR_COLOR,
        ));
    }

    scene
}

/// Returns a chain of small markers spanning the current position error.
fn error_link(truth_m: Vec3, estimate_m: Vec3, color: [f32; 4]) -> Vec<RenderSceneItem> {
    let delta = estimate_m - truth_m;
    let length = delta.length();
    if length <= 1.0e-3 {
        return Vec::new();
    }
    let steps = (length / 0.6).ceil().min(64.0) as usize;
    (0..=steps)
        .map(|index| {
            let t = index as f64 / steps.max(1) as f64;
            box_item(
                truth_m + delta * t + Vec3::new(0.0, 1.6, 0.0),
                Vec3::splat(0.3),
                Quat::IDENTITY,
                color,
            )
        })
        .collect()
}

fn box_item(center_m: Vec3, size_m: Vec3, rotation: Quat, color: [f32; 4]) -> RenderSceneItem {
    RenderScene::item_from_visual(
        Transform3::from_translation_rotation(center_m, rotation),
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
        .ok_or_else(|| std::io::Error::other("ffmpeg IMU GIF encode failed"))
}

/// Keeps the unused-import warning away when the renderer feature set changes.
#[allow(dead_code)]
fn unused_math_marker(_: MathTransform3) {}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn circular_track_speed_and_heading_are_consistent() {
        let track = CircularTrack::default();
        for time_s in [0.0, 1.3, 4.7, 11.9] {
            let velocity = track.velocity_m_s(time_s);
            assert!((velocity.length() - TRACK_SPEED_M_S).abs() < 1e-9);
            // The body +X axis points along the velocity.
            let forward = track.rotation(time_s) * Vec3::X;
            assert!(forward.dot(velocity.normalize()) > 0.999_999);
        }
        // The arc starts at the origin.
        assert!(track.position_m(0.0).length() < 1e-12);
    }

    #[test]
    fn an_ideal_imu_integrates_back_onto_the_truth() {
        let run = run_dead_reckoning(&CircularTrack::default(), ImuSpec::default());

        // Only first-order integration error remains, which stays sub-meter.
        assert!(
            run.final_error_m < 0.5,
            "ideal dead reckoning drifted {:.3} m",
            run.final_error_m
        );
        assert!(run.distance_travelled_m > 90.0);
    }

    #[test]
    fn a_modeled_imu_drifts_and_the_error_grows() {
        let run = run_dead_reckoning(&CircularTrack::default(), consumer_grade_imu());

        assert!(run.final_error_m > 1.0, "expected visible drift");
        // Error accumulates rather than staying flat.
        let early = run.samples[FRAME_COUNT / 4].position_error_m();
        let late = run.samples[FRAME_COUNT].position_error_m();
        assert!(late > early * 2.0, "error {early:.3} -> {late:.3} m");
    }

    #[test]
    fn dead_reckoning_is_deterministic() {
        let track = CircularTrack::default();
        let first = run_dead_reckoning(&track, consumer_grade_imu());
        let second = run_dead_reckoning(&track, consumer_grade_imu());

        assert_eq!(first, second);
        assert_eq!(first.stable_hash, second.stable_hash);
    }

    #[test]
    fn drift_scene_grows_with_the_frame_index() {
        let run = run_dead_reckoning(&CircularTrack::default(), consumer_grade_imu());

        let early = drift_scene(&run, 4).items.len();
        let late = drift_scene(&run, 120).items.len();
        assert!(late > early);
        // Every frame draws the ground plane plus both trails.
        assert!(early > 4);
    }

    #[test]
    fn small_angle_quat_matches_axis_angle_rotation() {
        let delta = Vec3::new(0.0, 0.02, 0.0);
        let rotated = small_angle_quat(delta) * Vec3::X;
        let expected = Quat::from_rotation_y(0.02) * Vec3::X;

        assert!((rotated - expected).length() < 1e-12);
        assert_eq!(small_angle_quat(Vec3::ZERO), Quat::IDENTITY);
    }
}
