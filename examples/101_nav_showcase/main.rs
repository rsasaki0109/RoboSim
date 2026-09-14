//! Generates README showcase media for RNE navigation and SLAM.
//!
//! Three deterministic, GPU-free top-down visualizations are written to
//! `docs/media/` (or `target/nav-showcase/` with `--smoke`):
//!
//! * `nav-slam.gif` — online 2D SLAM: the occupancy map grows while the
//!   corrected trajectory closes a loop on the return trip.
//! * `nav-multi-robot.gif` — three robots cross a shared plane, each avoiding
//!   the others with `rne_nav::avoid_velocities`.
//! * `nav-elevation.png` — a 2.5D elevation map shaded by slope, with a wall
//!   and a ramp classified for traversability.
//!
//! Run with `cargo run -p nav_showcase --example 101_nav_showcase`
//! (or `-- --smoke`).

use image::{Rgba, RgbaImage};
use rne_math::Vec3;
use rne_nav::{
    avoid_velocities, AvoidanceConfig, CircularObstacle, ElevationConfig, ElevationMap, FrameId,
    GridCoord, LaserScan2d, OccupancyGrid, Pose2d, VelocityCommand2d,
};
use rne_slam::{Slam2d, SlamConfig};
use std::f64::consts::TAU;
use std::path::{Path, PathBuf};
use std::process::Command;

const FPS: u32 = 12;
const ROBOT_RADIUS_M: f64 = 0.25;
const MAX_LINEAR_M_S: f64 = 0.8;
const MAX_ANGULAR_RAD_S: f64 = 1.6;

type Color = Rgba<u8>;

fn main() {
    let smoke = std::env::args().any(|argument| argument == "--smoke");
    let root = repo_root();
    let media = if smoke {
        root.join("target/nav-showcase")
    } else {
        root.join("docs/media")
    };
    let frames = root.join("target/nav-showcase/frames");
    std::fs::create_dir_all(&media).expect("create media dir");
    std::fs::create_dir_all(&frames).expect("create frames dir");

    slam_visual(&media, &frames);
    multi_robot_visual(&media, &frames);
    elevation_visual(&media);

    println!("nav showcase media written to {}", media.display());
}

fn repo_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(Path::parent)
        .expect("repo root")
        .to_path_buf()
}

// ---------------------------------------------------------------------------
// Canvas helpers
// ---------------------------------------------------------------------------

struct View {
    min_x: f64,
    min_y: f64,
    max_x: f64,
    max_y: f64,
    width: u32,
    height: u32,
}

impl View {
    fn px(&self, x: f64) -> i64 {
        let fraction = (x - self.min_x) / (self.max_x - self.min_x);
        (fraction * self.width as f64).round() as i64
    }

    fn py(&self, y: f64) -> i64 {
        let fraction = (self.max_y - y) / (self.max_y - self.min_y);
        (fraction * self.height as f64).round() as i64
    }
}

fn new_canvas(width: u32, height: u32, background: Color) -> RgbaImage {
    RgbaImage::from_pixel(width, height, background)
}

fn put(image: &mut RgbaImage, x: i64, y: i64, color: Color) {
    if x >= 0 && y >= 0 && (x as u32) < image.width() && (y as u32) < image.height() {
        image.put_pixel(x as u32, y as u32, color);
    }
}

fn fill_rect(image: &mut RgbaImage, x0: i64, y0: i64, x1: i64, y1: i64, color: Color) {
    for y in y0.min(y1)..=y0.max(y1) {
        for x in x0.min(x1)..=x0.max(x1) {
            put(image, x, y, color);
        }
    }
}

fn draw_line(image: &mut RgbaImage, x0: f64, y0: f64, x1: f64, y1: f64, color: Color, width: i64) {
    let dx = x1 - x0;
    let dy = y1 - y0;
    let steps = dx.abs().max(dy.abs()).max(1.0).ceil() as i64;
    for step in 0..=steps {
        let t = step as f64 / steps as f64;
        let x = x0 + dx * t;
        let y = y0 + dy * t;
        for oy in -width..=width {
            for ox in -width..=width {
                if ox * ox + oy * oy <= width * width {
                    put(image, x.round() as i64 + ox, y.round() as i64 + oy, color);
                }
            }
        }
    }
}

fn draw_disc(image: &mut RgbaImage, cx: f64, cy: f64, radius: f64, color: Color, filled: bool) {
    let r = radius.ceil() as i64;
    for oy in -r..=r {
        for ox in -r..=r {
            let distance = ((ox * ox + oy * oy) as f64).sqrt();
            let inside = if filled {
                distance <= radius
            } else {
                (distance - radius).abs() <= 1.0
            };
            if inside {
                put(image, cx.round() as i64 + ox, cy.round() as i64 + oy, color);
            }
        }
    }
}

fn save_frame(image: &RgbaImage, path: &Path) {
    image.save(path).expect("save frame");
}

fn build_gif(frames_dir: &Path, gif_path: &Path, width: u32) {
    let status = Command::new("ffmpeg")
        .args([
            "-y",
            "-framerate",
            &FPS.to_string(),
            "-i",
            &frames_dir.join("frame-%03d.png").to_string_lossy(),
            "-vf",
            &format!(
                "scale={width}:-1:flags=lanczos,split[s0][s1];\
                 [s0]palettegen=max_colors=224:stats_mode=diff[p];\
                 [s1][p]paletteuse=dither=bayer:bayer_scale=5:diff_mode=rectangle"
            ),
            &gif_path.to_string_lossy(),
        ])
        .status()
        .expect("run ffmpeg");
    assert!(status.success(), "ffmpeg failed for {}", gif_path.display());
}

fn reset_dir(dir: &Path) {
    if dir.exists() {
        std::fs::remove_dir_all(dir).expect("remove frames");
    }
    std::fs::create_dir_all(dir).expect("create frames");
}

// ---------------------------------------------------------------------------
// SLAM mapping
// ---------------------------------------------------------------------------

const BEAMS: usize = 360;

fn slam_visual(media: &Path, frames: &Path) {
    let grid = OccupancyGrid::new(240, 160, 0.05, Pose2d::new(-6.0, -4.0, 0.0)).expect("grid");
    let mut slam = Slam2d::new(grid, SlamConfig::default());
    let view = View {
        min_x: -6.0,
        min_y: -4.0,
        max_x: 6.0,
        max_y: 4.0,
        width: 720,
        height: 480,
    };
    let frames_dir = frames.join("slam");
    reset_dir(&frames_dir);

    let mut true_pose = Pose2d::new(-3.0, 0.0, 0.0);
    let mut odom = true_pose;
    let mut truth_trail = vec![(true_pose.x_m, true_pose.y_m)];
    let mut estimate_trail = vec![(true_pose.x_m, true_pose.y_m)];
    let mut last_scan = room_scan(true_pose.x_m, true_pose.y_m, 0.0);

    let route: Vec<f64> = std::iter::repeat_n(0.25, 20)
        .chain(std::iter::repeat_n(-0.25, 20))
        .collect();

    let mut frame = 0u32;
    let render = |slam: &Slam2d,
                  truth: Pose2d,
                  estimate: Pose2d,
                  scan: &LaserScan2d,
                  truth_trail: &[(f64, f64)],
                  estimate_trail: &[(f64, f64)],
                  frame: u32| {
        let image = render_slam_frame(
            slam.grid(),
            &view,
            truth,
            estimate,
            scan,
            truth_trail,
            estimate_trail,
        );
        save_frame(&image, &frames_dir.join(format!("frame-{frame:03}.png")));
    };

    for (index, step) in route.iter().enumerate() {
        let scan = room_scan(true_pose.x_m, true_pose.y_m, index as f64 * 0.05);
        let update = slam
            .process(&scan, odom, Pose2d::IDENTITY)
            .expect("slam step");
        last_scan = scan.clone();
        truth_trail.push((true_pose.x_m, true_pose.y_m));
        estimate_trail.push((update.pose.x_m, update.pose.y_m));
        render(
            &slam,
            true_pose,
            update.pose,
            &scan,
            &truth_trail,
            &estimate_trail,
            frame,
        );
        frame += 1;
        true_pose.x_m += step;
        odom.x_m += step;
        odom.yaw_rad += 0.008;
    }
    // Hold the finished map for a beat so the loop closure reads.
    let final_truth = *truth_trail.last().unwrap();
    let final_estimate = *estimate_trail.last().unwrap();
    for _ in 0..8 {
        render(
            &slam,
            Pose2d::new(final_truth.0, final_truth.1, 0.0),
            Pose2d::new(final_estimate.0, final_estimate.1, 0.0),
            &last_scan,
            &truth_trail,
            &estimate_trail,
            frame,
        );
        frame += 1;
    }

    build_gif(&frames_dir, &media.join("nav-slam.gif"), 640);
    std::fs::copy(
        frames_dir.join(format!("frame-{:03}.png", frame - 1)),
        media.join("nav-slam.png"),
    )
    .expect("poster");
    println!("nav-slam.gif: {} frames", frame);
}

fn render_slam_frame(
    grid: &OccupancyGrid,
    view: &View,
    truth: Pose2d,
    estimate: Pose2d,
    scan: &LaserScan2d,
    truth_trail: &[(f64, f64)],
    estimate_trail: &[(f64, f64)],
) -> RgbaImage {
    let mut image = new_canvas(view.width, view.height, Rgba([12, 14, 20, 255]));
    let cell = grid.resolution_m();
    for y in 0..grid.height() {
        for x in 0..grid.width() {
            let coord = GridCoord {
                x: x as isize,
                y: y as isize,
            };
            let color = if grid.is_occupied(coord) {
                Some(Rgba([236, 122, 58, 255]))
            } else if grid.is_known(coord) {
                Some(Rgba([34, 40, 52, 255]))
            } else {
                None
            };
            if let Some(color) = color {
                let wx = grid.origin().x_m + x as f64 * cell;
                let wy = grid.origin().y_m + y as f64 * cell;
                fill_rect(
                    &mut image,
                    view.px(wx),
                    view.py(wy + cell),
                    view.px(wx + cell),
                    view.py(wy),
                    color,
                );
            }
        }
    }

    // A light sample of the latest scan rays gives texture without bloating
    // the GIF.
    let (sin, cos) = truth.yaw_rad.sin_cos();
    for index in (0..scan.ranges_m.len()).step_by(8) {
        let range = scan.ranges_m[index];
        if !range.is_finite() || range <= scan.range_min_m || range >= scan.range_max_m {
            continue;
        }
        let angle = scan.angle_min_rad + scan.angle_increment_rad * index as f64;
        let end_x = truth.x_m + (cos * angle.cos() - sin * angle.sin()) * range;
        let end_y = truth.y_m + (sin * angle.cos() + cos * angle.sin()) * range;
        draw_line(
            &mut image,
            view.px(truth.x_m) as f64,
            view.py(truth.y_m) as f64,
            view.px(end_x) as f64,
            view.py(end_y) as f64,
            Rgba([70, 150, 200, 90]),
            0,
        );
    }

    for pair in truth_trail.windows(2) {
        draw_line(
            &mut image,
            view.px(pair[0].0) as f64,
            view.py(pair[0].1) as f64,
            view.px(pair[1].0) as f64,
            view.py(pair[1].1) as f64,
            Rgba([90, 220, 235, 255]),
            1,
        );
    }
    for pair in estimate_trail.windows(2) {
        draw_line(
            &mut image,
            view.px(pair[0].0) as f64,
            view.py(pair[0].1) as f64,
            view.px(pair[1].0) as f64,
            view.py(pair[1].1) as f64,
            Rgba([250, 170, 70, 220]),
            1,
        );
    }
    draw_disc(
        &mut image,
        view.px(estimate.x_m) as f64,
        view.py(estimate.y_m) as f64,
        5.0,
        Rgba([255, 255, 255, 255]),
        true,
    );
    draw_disc(
        &mut image,
        view.px(truth.x_m) as f64,
        view.py(truth.y_m) as f64,
        6.0,
        Rgba([90, 220, 235, 200]),
        false,
    );
    image
}

fn room_scan(x_m: f64, y_m: f64, time_s: f64) -> LaserScan2d {
    let mut ranges_m = Vec::with_capacity(BEAMS);
    for beam in 0..BEAMS {
        let angle = TAU * beam as f64 / BEAMS as f64;
        ranges_m.push(ray_to_wall(x_m, y_m, angle).min(ray_to_pillar(x_m, y_m, angle)));
    }
    LaserScan2d {
        time_s,
        frame: FrameId::new("laser"),
        angle_min_rad: 0.0,
        angle_increment_rad: TAU / BEAMS as f64,
        range_min_m: 0.05,
        range_max_m: 30.0,
        ranges_m,
    }
}

fn ray_to_wall(x: f64, y: f64, angle: f64) -> f64 {
    let (dx, dy) = (angle.cos(), angle.sin());
    let mut best = f64::INFINITY;
    for (bound, position, direction) in [(5.0, x, dx), (-5.0, x, dx), (3.0, y, dy), (-3.0, y, dy)] {
        if direction.abs() > 1.0e-9 {
            let t = (bound - position) / direction;
            if t > 0.0 {
                best = best.min(t);
            }
        }
    }
    best
}

fn ray_to_pillar(x: f64, y: f64, angle: f64) -> f64 {
    let pillars = [
        (2.0_f64, 1.2_f64, 0.25_f64),
        (-1.0, -1.6, 0.3),
        (3.5, -1.0, 0.2),
    ];
    let (dx, dy) = (angle.cos(), angle.sin());
    let mut best = f64::INFINITY;
    for (px, py, radius) in pillars {
        let (fx, fy) = (px - x, py - y);
        let projection = fx * dx + fy * dy;
        if projection <= 0.0 {
            continue;
        }
        let closest = fx.hypot(fy);
        let perpendicular = (closest * closest - projection * projection)
            .max(0.0)
            .sqrt();
        if perpendicular <= radius {
            let entry = projection
                - (radius * radius - perpendicular * perpendicular)
                    .max(0.0)
                    .sqrt();
            if entry > 0.0 {
                best = best.min(entry);
            }
        }
    }
    best
}

// ---------------------------------------------------------------------------
// Multi-robot avoidance
// ---------------------------------------------------------------------------

struct Robot {
    pose: Pose2d,
    goal: Vec3,
    command: VelocityCommand2d,
    trail: Vec<(f64, f64)>,
}

fn multi_robot_visual(media: &Path, frames: &Path) {
    let view = View {
        min_x: -2.0,
        min_y: -2.0,
        max_x: 2.0,
        max_y: 2.0,
        width: 640,
        height: 640,
    };
    let frames_dir = frames.join("multi");
    reset_dir(&frames_dir);
    let colors = [
        Rgba([255, 96, 96, 255]),
        Rgba([96, 168, 255, 255]),
        Rgba([120, 230, 140, 255]),
    ];
    let mut robots = vec![
        Robot {
            pose: Pose2d::new(-1.5, -0.25, 0.0),
            goal: Vec3::new(1.5, -0.25, 0.0),
            command: VelocityCommand2d::ZERO,
            trail: vec![(-1.5, -0.25)],
        },
        Robot {
            pose: Pose2d::new(1.5, 0.25, std::f64::consts::PI),
            goal: Vec3::new(-1.5, 0.25, 0.0),
            command: VelocityCommand2d::ZERO,
            trail: vec![(1.5, 0.25)],
        },
        Robot {
            pose: Pose2d::new(0.0, -1.5, std::f64::consts::FRAC_PI_2),
            goal: Vec3::new(0.0, 1.5, 0.0),
            command: VelocityCommand2d::ZERO,
            trail: vec![(0.0, -1.5)],
        },
    ];
    let config = AvoidanceConfig {
        time_horizon_s: 2.0,
        safety_margin_m: 0.1,
        ..AvoidanceConfig::default()
    };

    let mut frame = 0u32;
    let mut reached = 0;
    while frame < 90 && reached < robots.len() {
        let mut commands = Vec::with_capacity(robots.len());
        for (index, robot) in robots.iter().enumerate() {
            let obstacles: Vec<CircularObstacle> = robots
                .iter()
                .enumerate()
                .filter(|(other, _)| *other != index)
                .map(|(_, other)| CircularObstacle {
                    center_m: Vec3::new(other.pose.x_m, other.pose.y_m, 0.0),
                    velocity_m_s: Vec3::new(
                        other.command.linear_m_s * other.pose.yaw_rad.cos(),
                        other.command.linear_m_s * other.pose.yaw_rad.sin(),
                        0.0,
                    ),
                    radius_m: ROBOT_RADIUS_M,
                })
                .collect();
            let desired = seek(robot.pose, robot.goal);
            commands.push(avoid_velocities(
                robot.pose,
                ROBOT_RADIUS_M,
                &obstacles,
                desired,
                MAX_LINEAR_M_S,
                MAX_ANGULAR_RAD_S,
                &config,
            ));
        }
        reached = 0;
        for (index, robot) in robots.iter_mut().enumerate() {
            robot.command = commands[index];
            robot.pose.x_m += robot.command.linear_m_s * robot.pose.yaw_rad.cos() * 0.1;
            robot.pose.y_m += robot.command.linear_m_s * robot.pose.yaw_rad.sin() * 0.1;
            robot.pose.yaw_rad += robot.command.angular_rad_s * 0.1;
            robot.trail.push((robot.pose.x_m, robot.pose.y_m));
            let distance = (robot.goal.x - robot.pose.x_m).hypot(robot.goal.y - robot.pose.y_m);
            if distance < 0.15 {
                reached += 1;
            }
        }
        let image = render_multi_frame(&view, &robots, &colors);
        save_frame(&image, &frames_dir.join(format!("frame-{frame:03}.png")));
        frame += 1;
    }
    let image = render_multi_frame(&view, &robots, &colors);
    for _ in 0..8 {
        save_frame(&image, &frames_dir.join(format!("frame-{frame:03}.png")));
        frame += 1;
    }

    build_gif(&frames_dir, &media.join("nav-multi-robot.gif"), 640);
    std::fs::copy(
        frames_dir.join(format!("frame-{:03}.png", frame - 1)),
        media.join("nav-multi-robot.png"),
    )
    .expect("poster");
    println!("nav-multi-robot.gif: {} frames, {reached} robots", frame);
}

fn seek(pose: Pose2d, goal: Vec3) -> VelocityCommand2d {
    let dx = goal.x - pose.x_m;
    let dy = goal.y - pose.y_m;
    let distance = dx.hypot(dy);
    if distance < 0.1 {
        return VelocityCommand2d::ZERO;
    }
    let heading = dy.atan2(dx);
    let error = wrap_angle(heading - pose.yaw_rad);
    let angular = error.clamp(-MAX_ANGULAR_RAD_S, MAX_ANGULAR_RAD_S);
    let linear = if error.abs() < 0.6 {
        (0.7 * distance).min(MAX_LINEAR_M_S)
    } else {
        0.0
    };
    VelocityCommand2d::new(linear, angular)
}

fn wrap_angle(angle: f64) -> f64 {
    let two_pi = TAU;
    let mut wrapped = angle % two_pi;
    if wrapped > std::f64::consts::PI {
        wrapped -= two_pi;
    } else if wrapped <= -std::f64::consts::PI {
        wrapped += two_pi;
    }
    wrapped
}

fn render_multi_frame(view: &View, robots: &[Robot], colors: &[Color]) -> RgbaImage {
    let mut image = new_canvas(view.width, view.height, Rgba([16, 18, 24, 255]));
    // Grid every 0.5 m.
    let mut line = -2.0;
    while line <= 2.0 {
        draw_line(
            &mut image,
            view.px(line) as f64,
            0.0,
            view.px(line) as f64,
            view.height as f64,
            Rgba([30, 34, 42, 255]),
            0,
        );
        draw_line(
            &mut image,
            0.0,
            view.py(line) as f64,
            view.width as f64,
            view.py(line) as f64,
            Rgba([30, 34, 42, 255]),
            0,
        );
        line += 0.5;
    }
    for (index, robot) in robots.iter().enumerate() {
        let color = colors[index];
        // Goal marker.
        draw_disc(
            &mut image,
            view.px(robot.goal.x) as f64,
            view.py(robot.goal.y) as f64,
            0.15 / 2.0 * (view.width as f64 / (view.max_x - view.min_x)),
            Rgba([color[0], color[1], color[2], 120]),
            false,
        );
        // Trail.
        for pair in robot.trail.windows(2) {
            draw_line(
                &mut image,
                view.px(pair[0].0) as f64,
                view.py(pair[0].1) as f64,
                view.px(pair[1].0) as f64,
                view.py(pair[1].1) as f64,
                Rgba([color[0], color[1], color[2], 140]),
                1,
            );
        }
        // Robot body and heading.
        let radius = ROBOT_RADIUS_M / (view.max_x - view.min_x) * view.width as f64;
        let cx = view.px(robot.pose.x_m) as f64;
        let cy = view.py(robot.pose.y_m) as f64;
        draw_disc(&mut image, cx, cy, radius, color, true);
        draw_line(
            &mut image,
            cx,
            cy,
            cx + radius * robot.pose.yaw_rad.cos(),
            cy - radius * robot.pose.yaw_rad.sin(),
            Rgba([255, 255, 255, 255]),
            1,
        );
    }
    image
}

// ---------------------------------------------------------------------------
// Elevation map
// ---------------------------------------------------------------------------

fn elevation_visual(media: &Path) {
    let view = View {
        min_x: -2.0,
        min_y: -2.0,
        max_x: 2.0,
        max_y: 2.0,
        width: 720,
        height: 480,
    };
    let mut map = ElevationMap::new(
        40,
        40,
        0.1,
        Pose2d::new(-2.0, -2.0, 0.0),
        ElevationConfig::default(),
    )
    .expect("elevation map");
    let mut points = Vec::new();
    let mut x = -1.9;
    while x < 1.9 {
        let mut z = -1.9;
        while z < 1.9 {
            let waves = 0.35_f64 * (1.5_f64 * x).sin() * (1.3_f64 * z).cos();
            let hill = 0.45_f64 * (-((x - 0.7_f64).powi(2) + (z + 0.5_f64).powi(2)) / 0.5).exp();
            points.push(Vec3::new(x, waves + hill, z));
            z += 0.04;
        }
        x += 0.04;
    }
    // A wall: two very different heights in one cell exceed the step threshold.
    points.push(Vec3::new(-1.05, 0.0, 0.4));
    points.push(Vec3::new(-1.05, 1.0, 0.4));
    map.integrate(&points);

    let known: Vec<f64> = map
        .cells()
        .iter()
        .filter(|cell| cell.is_known())
        .map(|cell| cell.mean_y_m)
        .collect();
    let min_h = known.iter().cloned().fold(f64::INFINITY, f64::min);
    let max_h = known.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
    let span = (max_h - min_h).max(1.0e-6);

    let mut image = new_canvas(view.width, view.height, Rgba([12, 14, 20, 255]));
    for y in 0..map.height() {
        for x in 0..map.width() {
            let coord = GridCoord {
                x: x as isize,
                y: y as isize,
            };
            let Some(cell) = map.cell(coord).filter(|cell| cell.is_known()) else {
                continue;
            };
            let t = (cell.mean_y_m - min_h) / span;
            let slope = map.slope_at(coord).unwrap_or(0.0);
            let shade = (1.0 - (slope / 1.2).clamp(0.0, 1.0) * 0.55).clamp(0.4, 1.0);
            let base = height_color(t);
            let color = Rgba([
                (base[0] as f64 * shade) as u8,
                (base[1] as f64 * shade) as u8,
                (base[2] as f64 * shade) as u8,
                255,
            ]);
            let world_x = map.origin().x_m + x as f64 * map.resolution_m();
            let world_y = map.origin().y_m + y as f64 * map.resolution_m();
            fill_rect(
                &mut image,
                view.px(world_x),
                view.py(world_y + map.resolution_m()),
                view.px(world_x + map.resolution_m()),
                view.py(world_y),
                color,
            );
        }
    }
    // Mark non-traversable cells (wall, steep ramp) with a thin red outline.
    for y in 0..map.height() {
        for x in 0..map.width() {
            let coord = GridCoord {
                x: x as isize,
                y: y as isize,
            };
            if map.cell(coord).is_some_and(|cell| cell.is_known())
                && !map.is_traversable(coord, 0.7)
            {
                let world_x = map.origin().x_m + x as f64 * map.resolution_m();
                let world_y = map.origin().y_m + y as f64 * map.resolution_m();
                fill_rect(
                    &mut image,
                    view.px(world_x),
                    view.py(world_y),
                    view.px(world_x + map.resolution_m()),
                    view.py(world_y),
                    Rgba([255, 70, 70, 130]),
                );
            }
        }
    }

    image.save(media.join("nav-elevation.png")).expect("poster");
    println!("nav-elevation.png written");
}

fn height_color(t: f64) -> [u8; 3] {
    let stops = [
        (0.0, [40.0, 70.0, 130.0]),
        (0.35, [40.0, 150.0, 120.0]),
        (0.7, [230.0, 200.0, 90.0]),
        (1.0, [250.0, 250.0, 250.0]),
    ];
    let t = t.clamp(0.0, 1.0);
    for pair in stops.windows(2) {
        let (t0, c0) = pair[0];
        let (t1, c1) = pair[1];
        if t <= t1 {
            let f = (t - t0) / (t1 - t0).max(1.0e-9);
            return [
                (c0[0] + (c1[0] - c0[0]) * f) as u8,
                (c0[1] + (c1[1] - c0[1]) * f) as u8,
                (c0[2] + (c1[2] - c0[2]) * f) as u8,
            ];
        }
    }
    [250, 250, 250]
}
