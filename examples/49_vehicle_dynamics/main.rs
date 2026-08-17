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
const GIF_MAX_BYTE_SIZE: u64 = 4 * 1024 * 1024;

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
    kinematic_transform: Transform3,
    dynamic_transform: Transform3,
    kinematic_speed_m_s: f64,
    dynamic_speed_m_s: f64,
    dynamic_steering_rad: f64,
    dynamic_front_slip_rad: f64,
    dynamic_yaw_rate_rad_s: f64,
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
        let mut annotated_rgba8 = output.color.rgba8;
        annotate_frame(
            &mut annotated_rgba8,
            output.color.width,
            output.color.height,
            &run.samples[frame + 1],
        );
        write_png(
            &frames_dir.join(format!("frame-{frame:03}.png")),
            &annotated_rgba8,
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
        let kinematic_drive = world
            .get::<AckermannDrive>(kinematic)
            .expect("kinematic drive");
        let dynamic_drive = world.get::<AckermannDrive>(dynamic).expect("dynamic drive");
        if dynamics.front_saturated {
            *saturated += 1;
        }
        ComparisonSample {
            kinematic_transform: *world
                .get::<Transform3>(kinematic)
                .expect("kinematic transform"),
            dynamic_transform: *world.get::<Transform3>(dynamic).expect("dynamic transform"),
            kinematic_speed_m_s: kinematic_drive.speed_m_s,
            dynamic_speed_m_s: dynamic_drive.speed_m_s,
            dynamic_steering_rad: dynamic_drive.steering_rad,
            dynamic_front_slip_rad: dynamics.front_slip_rad,
            dynamic_yaw_rate_rad_s: dynamics.yaw_rate_rad_s,
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
                sample.kinematic_transform.translation.x,
                sample.kinematic_transform.translation.z,
                sample.dynamic_transform.translation.x,
                sample.dynamic_transform.translation.z,
                course_distance_m(course, sample.kinematic_transform.translation),
                sample.front_saturated,
            );
        }
        maximum_gap_m = maximum_gap_m.max(
            (sample.dynamic_transform.translation - sample.kinematic_transform.translation)
                .length(),
        );
        kinematic_course_error_m = kinematic_course_error_m.max(course_distance_m(
            course,
            sample.kinematic_transform.translation,
        ));
        dynamic_course_error_m = dynamic_course_error_m.max(course_distance_m(
            course,
            sample.dynamic_transform.translation,
        ));
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
        pitch_rad: 0.58,
        distance_m: 72.0,
    }
}

/// Builds the scene for one frame: course line, both trails, and current poses.
fn comparison_scene(course: &[Vec3], run: &ComparisonRun, frame: usize) -> RenderScene {
    const ROAD_COLOR: [f32; 4] = [0.20, 0.22, 0.27, 1.0];
    const LANE_COLOR: [f32; 4] = [0.72, 0.74, 0.76, 1.0];
    const KINEMATIC_COLOR: [f32; 4] = [0.10, 0.85, 0.55, 1.0];
    const DYNAMIC_COLOR: [f32; 4] = [1.0, 0.55, 0.10, 1.0];
    const SATURATED_COLOR: [f32; 4] = [0.95, 0.20, 0.30, 1.0];
    const GROUND_COLOR: [f32; 4] = [0.09, 0.11, 0.14, 1.0];

    let mut scene = RenderScene::new();
    scene.items.push(box_item(
        Vec3::new(34.0, -0.35, -24.0),
        Vec3::new(420.0, 0.2, 420.0),
        GROUND_COLOR,
    ));
    for segment in course.windows(2) {
        let delta = segment[1] - segment[0];
        let center = (segment[0] + segment[1]) * 0.5 + Vec3::new(0.0, 0.01, 0.0);
        let rotation = Quat::from_rotation_y((-delta.z).atan2(delta.x));
        scene.items.push(oriented_box_item(
            center,
            rotation,
            Vec3::new(delta.length() + 0.5, 0.10, 6.0),
            ROAD_COLOR,
        ));
    }
    for waypoint in course.iter().step_by(2) {
        scene.items.push(box_item(
            *waypoint + Vec3::new(0.0, 0.08, 0.0),
            Vec3::new(1.5, 0.035, 0.10),
            LANE_COLOR,
        ));
    }

    let visible = frame.min(run.samples.len().saturating_sub(1));
    for sample in run.samples[..=visible].iter().step_by(2) {
        scene.items.push(box_item(
            sample.kinematic_transform.translation + Vec3::new(0.0, 0.16, 0.0),
            Vec3::splat(0.34),
            KINEMATIC_COLOR,
        ));
        // The dynamic trail turns red wherever the front axle is beyond its grip.
        let trail_color = if sample.front_saturated {
            SATURATED_COLOR
        } else {
            DYNAMIC_COLOR
        };
        scene.items.push(box_item(
            sample.dynamic_transform.translation + Vec3::new(0.0, 0.16, 0.0),
            Vec3::splat(0.34),
            trail_color,
        ));
    }

    if let Some(current) = run.samples.get(visible) {
        append_car(
            &mut scene,
            current.kinematic_transform,
            0.0,
            KINEMATIC_COLOR,
        );
        append_car(
            &mut scene,
            current.dynamic_transform,
            current.dynamic_steering_rad,
            if current.front_saturated {
                SATURATED_COLOR
            } else {
                DYNAMIC_COLOR
            },
        );
    }

    scene
}

fn append_car(
    scene: &mut RenderScene,
    transform: Transform3,
    front_steering_rad: f64,
    color: [f32; 4],
) {
    const TIRE_COLOR: [f32; 4] = [0.025, 0.03, 0.04, 1.0];
    const GLASS_COLOR: [f32; 4] = [0.10, 0.18, 0.24, 1.0];
    const LIGHT_COLOR: [f32; 4] = [1.0, 0.92, 0.55, 1.0];
    let rotation = transform.rotation;
    let position = transform.translation;

    scene.items.push(oriented_box_item(
        position + rotation * Vec3::new(0.0, 0.62, 0.0),
        rotation,
        Vec3::new(4.5, 0.62, 1.9),
        color,
    ));
    scene.items.push(oriented_box_item(
        position + rotation * Vec3::new(-0.35, 1.15, 0.0),
        rotation,
        Vec3::new(2.05, 0.72, 1.55),
        GLASS_COLOR,
    ));
    scene.items.push(oriented_box_item(
        position + rotation * Vec3::new(2.27, 0.64, 0.0),
        rotation,
        Vec3::new(0.10, 0.22, 1.25),
        LIGHT_COLOR,
    ));

    for (x_m, z_m, steerable) in [
        (-1.35, -1.00, false),
        (-1.35, 1.00, false),
        (1.35, -1.00, true),
        (1.35, 1.00, true),
    ] {
        let steering = if steerable { front_steering_rad } else { 0.0 };
        let wheel_transform = Transform3::from_translation_rotation(
            position + rotation * Vec3::new(x_m, 0.42, z_m),
            rotation * Quat::from_rotation_y(steering),
        );
        scene.items.push(RenderScene::item_from_visual(
            wheel_transform,
            VisualShape::Cylinder {
                radius_m: 0.43,
                length_m: 0.28,
            },
            TIRE_COLOR,
            Transform3::IDENTITY,
        ));
    }
}

fn box_item(center_m: Vec3, size_m: Vec3, color: [f32; 4]) -> RenderSceneItem {
    oriented_box_item(center_m, Quat::IDENTITY, size_m, color)
}

fn oriented_box_item(
    center_m: Vec3,
    rotation: Quat,
    size_m: Vec3,
    color: [f32; 4],
) -> RenderSceneItem {
    RenderScene::item_from_visual(
        Transform3::from_translation_rotation(center_m, rotation),
        VisualShape::Box { size_m },
        color,
        Transform3::IDENTITY,
    )
}

fn annotate_frame(rgba8: &mut [u8], width: u32, height: u32, sample: &ComparisonSample) {
    assert_eq!(rgba8.len(), width as usize * height as usize * 4);
    let white = [238, 243, 248, 255];
    let muted = [171, 181, 194, 255];
    let green = [35, 222, 148, 255];
    let orange = [255, 142, 32, 255];
    let red = [244, 51, 76, 255];

    blend_rect(rgba8, width, height, (0, 0, width, 112), [9, 13, 20, 218]);
    draw_text(
        rgba8,
        width,
        height,
        (32, 24),
        3,
        "SAME CONTROLLER / TWO VEHICLE MODELS",
        white,
    );
    fill_rect(rgba8, width, height, (34, 70, 16, 16), green);
    draw_text(
        rgba8,
        width,
        height,
        (62, 70),
        2,
        &format!("KINEMATIC  {:4.1} M/S  NO-SLIP", sample.kinematic_speed_m_s),
        muted,
    );
    fill_rect(rgba8, width, height, (478, 70, 16, 16), orange);
    draw_text(
        rgba8,
        width,
        height,
        (506, 70),
        2,
        &format!(
            "DYNAMIC  {:4.1} M/S  TIRE-LIMITED",
            sample.dynamic_speed_m_s
        ),
        muted,
    );

    let panel_x = width.saturating_sub(372);
    let panel_y = height.saturating_sub(148);
    blend_rect(
        rgba8,
        width,
        height,
        (panel_x, panel_y, 340, 116),
        [9, 13, 20, 224],
    );
    let status_color = if sample.front_saturated { red } else { green };
    fill_rect(
        rgba8,
        width,
        height,
        (panel_x, panel_y, 6, 116),
        status_color,
    );
    draw_text(
        rgba8,
        width,
        height,
        (panel_x + 22, panel_y + 16),
        2,
        "DYNAMIC TELEMETRY",
        white,
    );
    draw_text(
        rgba8,
        width,
        height,
        (panel_x + 22, panel_y + 42),
        2,
        &format!(
            "FRONT SLIP  {:4.1} DEG",
            sample.dynamic_front_slip_rad.to_degrees().abs()
        ),
        muted,
    );
    draw_text(
        rgba8,
        width,
        height,
        (panel_x + 22, panel_y + 66),
        2,
        &format!("YAW RATE    {:4.2} RAD/S", sample.dynamic_yaw_rate_rad_s),
        muted,
    );
    draw_text(
        rgba8,
        width,
        height,
        (panel_x + 22, panel_y + 90),
        2,
        if sample.front_saturated {
            "FRONT AXLE SATURATED"
        } else {
            "FRONT GRIP OK"
        },
        status_color,
    );
}

fn blend_rect(
    rgba8: &mut [u8],
    width: u32,
    height: u32,
    rect: (u32, u32, u32, u32),
    color: [u8; 4],
) {
    let (x, y, rect_width, rect_height) = rect;
    let max_x = x.saturating_add(rect_width).min(width);
    let max_y = y.saturating_add(rect_height).min(height);
    let alpha = u16::from(color[3]);
    for pixel_y in y.min(height)..max_y {
        for pixel_x in x.min(width)..max_x {
            let index = ((pixel_y * width + pixel_x) * 4) as usize;
            for channel in 0..3 {
                let source = u16::from(color[channel]);
                let destination = u16::from(rgba8[index + channel]);
                rgba8[index + channel] =
                    ((source * alpha + destination * (255 - alpha)) / 255) as u8;
            }
            rgba8[index + 3] = 255;
        }
    }
}

fn fill_rect(
    rgba8: &mut [u8],
    width: u32,
    height: u32,
    rect: (u32, u32, u32, u32),
    color: [u8; 4],
) {
    blend_rect(rgba8, width, height, rect, color);
}

fn draw_text(
    rgba8: &mut [u8],
    width: u32,
    height: u32,
    origin: (u32, u32),
    scale: u32,
    text: &str,
    color: [u8; 4],
) {
    let (x, y) = origin;
    let mut cursor_x = x;
    for character in text.chars() {
        let glyph = glyph_5x7(character);
        for (row, bits) in glyph.iter().copied().enumerate() {
            for column in 0..5 {
                if bits & (1 << (4 - column)) != 0 {
                    fill_rect(
                        rgba8,
                        width,
                        height,
                        (
                            cursor_x + column * scale,
                            y + row as u32 * scale,
                            scale,
                            scale,
                        ),
                        color,
                    );
                }
            }
        }
        cursor_x = cursor_x.saturating_add(6 * scale);
        if cursor_x >= width {
            break;
        }
    }
}

fn glyph_5x7(character: char) -> [u8; 7] {
    match character.to_ascii_uppercase() {
        'A' => [14, 17, 17, 31, 17, 17, 17],
        'B' => [30, 17, 17, 30, 17, 17, 30],
        'C' => [14, 17, 16, 16, 16, 17, 14],
        'D' => [30, 17, 17, 17, 17, 17, 30],
        'E' => [31, 16, 16, 30, 16, 16, 31],
        'F' => [31, 16, 16, 30, 16, 16, 16],
        'G' => [14, 17, 16, 23, 17, 17, 15],
        'H' => [17, 17, 17, 31, 17, 17, 17],
        'I' => [31, 4, 4, 4, 4, 4, 31],
        'J' => [7, 2, 2, 2, 18, 18, 12],
        'K' => [17, 18, 20, 24, 20, 18, 17],
        'L' => [16, 16, 16, 16, 16, 16, 31],
        'M' => [17, 27, 21, 21, 17, 17, 17],
        'N' => [17, 25, 21, 19, 17, 17, 17],
        'O' => [14, 17, 17, 17, 17, 17, 14],
        'P' => [30, 17, 17, 30, 16, 16, 16],
        'Q' => [14, 17, 17, 17, 21, 18, 13],
        'R' => [30, 17, 17, 30, 20, 18, 17],
        'S' => [15, 16, 16, 14, 1, 1, 30],
        'T' => [31, 4, 4, 4, 4, 4, 4],
        'U' => [17, 17, 17, 17, 17, 17, 14],
        'V' => [17, 17, 17, 17, 17, 10, 4],
        'W' => [17, 17, 17, 21, 21, 21, 10],
        'X' => [17, 17, 10, 4, 10, 17, 17],
        'Y' => [17, 17, 10, 4, 4, 4, 4],
        'Z' => [31, 1, 2, 4, 8, 16, 31],
        '0' => [14, 17, 19, 21, 25, 17, 14],
        '1' => [4, 12, 4, 4, 4, 4, 14],
        '2' => [14, 17, 1, 2, 4, 8, 31],
        '3' => [30, 1, 1, 14, 1, 1, 30],
        '4' => [2, 6, 10, 18, 31, 2, 2],
        '5' => [31, 16, 16, 30, 1, 1, 30],
        '6' => [14, 16, 16, 30, 17, 17, 14],
        '7' => [31, 1, 2, 4, 8, 8, 8],
        '8' => [14, 17, 17, 14, 17, 17, 14],
        '9' => [14, 17, 17, 15, 1, 1, 14],
        '-' => [0, 0, 0, 31, 0, 0, 0],
        '/' => [1, 1, 2, 4, 8, 16, 16],
        '.' => [0, 0, 0, 0, 0, 12, 12],
        ':' => [0, 12, 12, 0, 12, 12, 0],
        _ => [0; 7],
    }
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
        .ok_or_else(|| std::io::Error::other("ffmpeg vehicle dynamics GIF encode failed"))?;
    let byte_size = fs::metadata(gif_path)?.len();
    if byte_size > GIF_MAX_BYTE_SIZE {
        return Err(std::io::Error::other(format!(
            "vehicle dynamics GIF exceeds size budget: {byte_size} bytes > {GIF_MAX_BYTE_SIZE} bytes"
        )));
    }
    Ok(())
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
        assert_eq!(
            scene
                .items
                .iter()
                .filter(|item| matches!(item.shape, VisualShape::Cylinder { .. }))
                .count(),
            8,
            "both procedural cars must expose four visible wheels"
        );
    }

    #[test]
    fn comparison_records_pose_and_dynamic_telemetry() {
        let run = run_comparison(&course_waypoints());
        assert!(run
            .samples
            .iter()
            .any(|sample| sample.kinematic_transform.rotation != Quat::IDENTITY));
        assert!(run
            .samples
            .iter()
            .any(|sample| sample.dynamic_front_slip_rad.abs() > 0.01));
        assert!(run
            .samples
            .iter()
            .any(|sample| sample.dynamic_yaw_rate_rad_s.abs() > 0.01));
    }

    #[test]
    fn telemetry_overlay_changes_the_frame_without_a_gpu() {
        let run = run_comparison(&course_waypoints());
        let sample = run
            .samples
            .iter()
            .find(|sample| sample.front_saturated)
            .expect("scenario reaches front-axle saturation");
        let mut rgba8 = vec![0; 640 * 360 * 4];
        annotate_frame(&mut rgba8, 640, 360, sample);
        assert!(rgba8.iter().any(|channel| *channel != 0));
        assert!(rgba8
            .chunks_exact(4)
            .any(|pixel| pixel[0] > 200 && pixel[1] < 100));
    }
}
