//! Generates README showcase media for RNE navigation and SLAM in 3D.
//!
//! A small, deterministic software rasterizer (z-buffer, flat Lambert shading,
//! perspective camera) renders three orbiting 3D views to `docs/media/` (or
//! `target/nav-showcase/` with `--smoke`):
//!
//! * `nav-slam.gif` — the growing SLAM occupancy map as a 3D height field with
//!   the corrected trajectory closing a loop.
//! * `nav-multi-robot.gif` — three robots on a ground plane, each yielding to
//!   the others with `rne_nav::avoid_velocities`.
//! * `nav-elevation.gif` — the 2.5D elevation surface shaded by height and
//!   slope.
//!
//! No GPU is required. Run with
//! `cargo run -p nav_showcase --example 101_nav_showcase` (or `-- --smoke`).

use image::RgbaImage;
use rne_math::Vec3;
use rne_nav::{
    avoid_velocities, AvoidanceConfig, CircularObstacle, ElevationConfig, ElevationMap, FrameId,
    GridCoord, LaserScan2d, OccupancyGrid, Pose2d, VelocityCommand2d,
};
use rne_slam::{Slam2d, SlamConfig};
use std::f64::consts::TAU;
use std::path::{Path, PathBuf};
use std::process::Command;

const FPS: u32 = 15;
const ROBOT_RADIUS_M: f64 = 0.25;
const MAX_LINEAR_M_S: f64 = 0.8;
const MAX_ANGULAR_RAD_S: f64 = 1.6;

fn light() -> Vec3 {
    Vec3::new(0.45, 0.8, 0.35).normalize()
}

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
    elevation_visual(&media, &frames);

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
// Software 3D renderer
// ---------------------------------------------------------------------------

/// A perspective pinhole camera.
struct Camera {
    eye: Vec3,
    target: Vec3,
    up: Vec3,
    focal: f64,
    aspect: f64,
}

impl Camera {
    /// Builds a camera looking from `eye` at `target` with a vertical FOV.
    fn look_at(eye: Vec3, target: Vec3, fov_y_rad: f64, aspect: f64) -> Self {
        Self {
            eye,
            target,
            up: Vec3::Y,
            focal: 1.0 / (fov_y_rad * 0.5).tan(),
            aspect,
        }
    }

    fn axes(&self) -> (Vec3, Vec3, Vec3) {
        let forward = (self.target - self.eye).normalize();
        let right = forward.cross(self.up).normalize();
        let up = right.cross(forward);
        (forward, right, up)
    }

    /// Projects a world point to `(screen_x, screen_y, depth)`.
    fn project(&self, point: Vec3, width: u32, height: u32) -> Option<(f64, f64, f64)> {
        let (forward, right, up) = self.axes();
        let relative = point - self.eye;
        let depth = relative.dot(forward);
        if depth <= 1.0e-3 {
            return None;
        }
        let x = relative.dot(right);
        let y = relative.dot(up);
        let ndc_x = (x / depth) * self.focal / self.aspect;
        let ndc_y = (y / depth) * self.focal;
        Some((
            (ndc_x * 0.5 + 0.5) * width as f64,
            (0.5 - ndc_y * 0.5) * height as f64,
            depth,
        ))
    }

    fn orbit(target: Vec3, radius: f64, height: f64, angle_rad: f64) -> Self {
        let eye = target + Vec3::new(radius * angle_rad.cos(), height, radius * angle_rad.sin());
        Self::look_at(eye, target, 0.9, 4.0 / 3.0)
    }
}

struct Renderer {
    width: u32,
    height: u32,
    background: [u8; 3],
    color: Vec<[u8; 3]>,
    depth: Vec<f64>,
}

impl Renderer {
    fn new(width: u32, height: u32, background: [u8; 3]) -> Self {
        Self {
            width,
            height,
            background,
            color: vec![background; (width * height) as usize],
            depth: vec![f64::INFINITY; (width * height) as usize],
        }
    }

    fn clear(&mut self) {
        self.color.fill(self.background);
        self.depth.fill(f64::INFINITY);
    }

    fn blit(&self) -> RgbaImage {
        let mut image = RgbaImage::new(self.width, self.height);
        for (index, pixel) in self.color.iter().enumerate() {
            let x = (index as u32) % self.width;
            let y = (index as u32) / self.width;
            image.put_pixel(x, y, image::Rgba([pixel[0], pixel[1], pixel[2], 255]));
        }
        image
    }

    fn put(&mut self, x: i64, y: i64, z: f64, color: [u8; 3]) {
        if x < 0 || y < 0 || x >= self.width as i64 || y >= self.height as i64 {
            return;
        }
        let index = y as usize * self.width as usize + x as usize;
        if z < self.depth[index] {
            self.depth[index] = z;
            self.color[index] = color;
        }
    }

    /// Fills a flat-shaded world-space triangle.
    fn triangle(&mut self, camera: &Camera, a: Vec3, b: Vec3, c: Vec3, base: [f64; 3]) {
        let (Some(p0), Some(p1), Some(p2)) = (
            camera.project(a, self.width, self.height),
            camera.project(b, self.width, self.height),
            camera.project(c, self.width, self.height),
        ) else {
            return;
        };

        let mut normal = (b - a).cross(c - a);
        if normal.length_squared() < 1.0e-18 {
            return;
        }
        normal = normal.normalize();
        if normal.dot(camera.eye - a) < 0.0 {
            normal = -normal;
        }
        let intensity = 0.32 + 0.68 * normal.dot(light()).max(0.0);
        let shaded = [
            (base[0] * intensity).clamp(0.0, 255.0) as u8,
            (base[1] * intensity).clamp(0.0, 255.0) as u8,
            (base[2] * intensity).clamp(0.0, 255.0) as u8,
        ];

        let area = edge(p0, p1, p2);
        if area.abs() < 1.0e-9 {
            return;
        }
        let min_x = p0.0.min(p1.0).min(p2.0).floor().max(0.0) as i64;
        let max_x = p0.0.max(p1.0).max(p2.0).ceil().min(self.width as f64 - 1.0) as i64;
        let min_y = p0.1.min(p1.1).min(p2.1).floor().max(0.0) as i64;
        let max_y =
            p0.1.max(p1.1)
                .max(p2.1)
                .ceil()
                .min(self.height as f64 - 1.0) as i64;
        let inv_area = 1.0 / area;
        for y in min_y..=max_y {
            for x in min_x..=max_x {
                let sample = (x as f64 + 0.5, y as f64 + 0.5);
                let w0 = edge(p1, p2, (sample.0, sample.1, 0.0));
                let w1 = edge(p2, p0, (sample.0, sample.1, 0.0));
                let w2 = edge(p0, p1, (sample.0, sample.1, 0.0));
                let inside =
                    (w0 >= 0.0 && w1 >= 0.0 && w2 >= 0.0) || (w0 <= 0.0 && w1 <= 0.0 && w2 <= 0.0);
                if !inside {
                    continue;
                }
                let depth = (w0 * p0.2 + w1 * p1.2 + w2 * p2.2) * inv_area;
                if depth > 0.0 {
                    self.put(x, y, depth, shaded);
                }
            }
        }
    }

    fn line(&mut self, camera: &Camera, a: Vec3, b: Vec3, color: [u8; 3], thickness: i64) {
        let (Some(p0), Some(p1)) = (
            camera.project(a, self.width, self.height),
            camera.project(b, self.width, self.height),
        ) else {
            return;
        };
        let steps = (p1.0 - p0.0).abs().max((p1.1 - p0.1).abs()).max(1.0).ceil() as i64;
        for step in 0..=steps {
            let t = step as f64 / steps as f64;
            let x = p0.0 + (p1.0 - p0.0) * t;
            let y = p0.1 + (p1.1 - p0.1) * t;
            let z = p0.2 + (p1.2 - p0.2) * t;
            for oy in -thickness..=thickness {
                for ox in -thickness..=thickness {
                    if ox * ox + oy * oy <= thickness * thickness {
                        self.put(x.round() as i64 + ox, y.round() as i64 + oy, z, color);
                    }
                }
            }
        }
    }

    /// Draws an axis-aligned box as 12 flat-shaded triangles.
    fn box_3d(&mut self, camera: &Camera, center: Vec3, half: Vec3, base: [f64; 3]) {
        let corners = [
            center + Vec3::new(-half.x, -half.y, -half.z),
            center + Vec3::new(half.x, -half.y, -half.z),
            center + Vec3::new(half.x, half.y, -half.z),
            center + Vec3::new(-half.x, half.y, -half.z),
            center + Vec3::new(-half.x, -half.y, half.z),
            center + Vec3::new(half.x, -half.y, half.z),
            center + Vec3::new(half.x, half.y, half.z),
            center + Vec3::new(-half.x, half.y, half.z),
        ];
        let faces = [
            [0, 3, 2, 1],
            [4, 5, 6, 7],
            [0, 1, 5, 4],
            [2, 3, 7, 6],
            [0, 4, 7, 3],
            [1, 2, 6, 5],
        ];
        for face in faces {
            self.triangle(
                camera,
                corners[face[0]],
                corners[face[1]],
                corners[face[2]],
                base,
            );
            self.triangle(
                camera,
                corners[face[0]],
                corners[face[2]],
                corners[face[3]],
                base,
            );
        }
    }

    fn ground_grid(&mut self, camera: &Camera, half_extent_m: f64, step_m: f64, color: [u8; 3]) {
        let mut offset = -half_extent_m;
        while offset <= half_extent_m + 1.0e-9 {
            self.line(
                camera,
                Vec3::new(offset, 0.0, -half_extent_m),
                Vec3::new(offset, 0.0, half_extent_m),
                color,
                0,
            );
            self.line(
                camera,
                Vec3::new(-half_extent_m, 0.0, offset),
                Vec3::new(half_extent_m, 0.0, offset),
                color,
                0,
            );
            offset += step_m;
        }
    }
}

fn edge(a: (f64, f64, f64), b: (f64, f64, f64), c: (f64, f64, f64)) -> f64 {
    (b.0 - a.0) * (c.1 - a.1) - (b.1 - a.1) * (c.0 - a.0)
}

fn save_frame(renderer: &Renderer, path: &Path) {
    renderer.blit().save(path).expect("save frame");
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

fn angle_for_frame(frame: u32, total: u32, start: f64, sweep: f64) -> f64 {
    start + sweep * frame as f64 / total.max(1) as f64
}

// ---------------------------------------------------------------------------
// SLAM mapping
// ---------------------------------------------------------------------------

const BEAMS: usize = 360;

fn slam_visual(media: &Path, frames: &Path) {
    let grid = OccupancyGrid::new(240, 160, 0.05, Pose2d::new(-6.0, -4.0, 0.0)).expect("grid");
    let mut slam = Slam2d::new(grid, SlamConfig::default());
    let frames_dir = frames.join("slam");
    reset_dir(&frames_dir);

    let mut true_pose = Pose2d::new(-3.0, 0.0, 0.0);
    let mut odom = true_pose;
    let mut truth_trail = vec![(true_pose.x_m, true_pose.y_m)];
    let mut estimate_trail = vec![(true_pose.x_m, true_pose.y_m)];

    let route: Vec<f64> = std::iter::repeat_n(0.25, 20)
        .chain(std::iter::repeat_n(-0.25, 20))
        .collect();

    let mut frame = 0u32;
    let mut captured: Vec<(Pose2d, Pose2d, usize, usize)> = Vec::new();
    for (index, step) in route.iter().enumerate() {
        let scan = room_scan(true_pose.x_m, true_pose.y_m, index as f64 * 0.05);
        let update = slam
            .process(&scan, odom, Pose2d::IDENTITY)
            .expect("slam step");
        truth_trail.push((true_pose.x_m, true_pose.y_m));
        estimate_trail.push((update.pose.x_m, update.pose.y_m));
        captured.push((
            true_pose,
            update.pose,
            truth_trail.len(),
            estimate_trail.len(),
        ));
        true_pose.x_m += step;
        odom.x_m += step;
        odom.yaw_rad += 0.008;
    }
    let total = (captured.len() + 12) as u32;
    for (index, (truth, estimate, truth_len, estimate_len)) in captured.iter().enumerate() {
        let angle = angle_for_frame(index as u32, total, -2.4, 3.4);
        let camera = Camera::orbit(Vec3::new(0.0, 0.35, 0.0), 9.5, 6.2, angle);
        let mut renderer = Renderer::new(640, 480, [10, 12, 18]);
        render_slam_scene(
            &mut renderer,
            &camera,
            slam.grid(),
            *truth,
            *estimate,
            &truth_trail[..*truth_len.min(&truth_trail.len())],
            &estimate_trail[..*estimate_len.min(&estimate_trail.len())],
        );
        save_frame(&renderer, &frames_dir.join(format!("frame-{frame:03}.png")));
        frame += 1;
    }
    // Hold the finished map while the camera keeps moving.
    let last = *captured.last().unwrap();
    for hold in 0..12 {
        let index = captured.len() as u32 + hold;
        let angle = angle_for_frame(index, total, -2.4, 3.4);
        let camera = Camera::orbit(Vec3::new(0.0, 0.35, 0.0), 9.5, 6.2, angle);
        let mut renderer = Renderer::new(640, 480, [10, 12, 18]);
        render_slam_scene(
            &mut renderer,
            &camera,
            slam.grid(),
            last.0,
            last.1,
            &truth_trail,
            &estimate_trail,
        );
        save_frame(&renderer, &frames_dir.join(format!("frame-{frame:03}.png")));
        frame += 1;
    }

    build_gif(&frames_dir, &media.join("nav-slam.gif"), 640);
    std::fs::copy(
        frames_dir.join(format!("frame-{:03}.png", total - 1)),
        media.join("nav-slam.png"),
    )
    .expect("poster");
    println!("nav-slam.gif: {} frames", frame);
}

fn cell_height(grid: &OccupancyGrid, x: usize, y: usize) -> f64 {
    let coord = GridCoord {
        x: x as isize,
        y: y as isize,
    };
    if grid.is_occupied(coord) {
        0.7
    } else if grid.is_known(coord) {
        0.03
    } else {
        0.0
    }
}

fn render_slam_scene(
    renderer: &mut Renderer,
    camera: &Camera,
    grid: &OccupancyGrid,
    truth: Pose2d,
    estimate: Pose2d,
    truth_trail: &[(f64, f64)],
    estimate_trail: &[(f64, f64)],
) {
    renderer.clear();
    let stride = 4;
    let resolution = grid.resolution_m();
    let base_x = grid.origin().x_m;
    let base_y = grid.origin().y_m;
    let mut ix = 0;
    while ix + stride < grid.width() {
        let mut iy = 0;
        while iy + stride < grid.height() {
            // Skip fully unknown blocks to save triangles.
            let known = (0..stride).any(|dx| {
                (0..stride).any(|dy| {
                    grid.is_known(GridCoord {
                        x: (ix + dx) as isize,
                        y: (iy + dy) as isize,
                    })
                })
            });
            if known {
                let h = |x: usize, y: usize| cell_height(grid, x, y);
                let corner = |x: usize, y: usize| {
                    Vec3::new(
                        base_x + x as f64 * resolution,
                        h(x.min(grid.width() - 1), y.min(grid.height() - 1)),
                        base_y + y as f64 * resolution,
                    )
                };
                let a = corner(ix, iy);
                let b = corner(ix + stride, iy);
                let c = corner(ix + stride, iy + stride);
                let d = corner(ix, iy + stride);
                let occupied = (0..stride).any(|dx| {
                    (0..stride).any(|dy| {
                        grid.is_occupied(GridCoord {
                            x: (ix + dx) as isize,
                            y: (iy + dy) as isize,
                        })
                    })
                });
                let color = if occupied {
                    [235.0, 120.0, 55.0]
                } else {
                    [44.0, 58.0, 76.0]
                };
                renderer.triangle(camera, a, b, c, color);
                renderer.triangle(camera, a, c, d, color);
            }
            iy += stride;
        }
        ix += stride;
    }

    for trail in [truth_trail, estimate_trail] {
        for pair in trail.windows(2) {
            renderer.line(
                camera,
                Vec3::new(pair[0].0, 0.95, pair[0].1),
                Vec3::new(pair[1].0, 0.95, pair[1].1),
                [90, 220, 235],
                1,
            );
        }
    }
    renderer.box_3d(
        camera,
        Vec3::new(truth.x_m, 0.95, truth.y_m),
        Vec3::splat(0.1),
        [90.0, 220.0, 235.0],
    );
    renderer.box_3d(
        camera,
        Vec3::new(estimate.x_m, 0.95, estimate.y_m),
        Vec3::splat(0.12),
        [250.0, 180.0, 80.0],
    );
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
    let frames_dir = frames.join("multi");
    reset_dir(&frames_dir);
    let colors = [
        [255.0, 96.0, 96.0],
        [96.0, 168.0, 255.0],
        [120.0, 230.0, 140.0],
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
            if (robot.goal.x - robot.pose.x_m).hypot(robot.goal.y - robot.pose.y_m) < 0.15 {
                reached += 1;
            }
        }
        render_multi(&frames_dir, frame, &robots, &colors);
        frame += 1;
    }
    for _ in 0..10 {
        render_multi(&frames_dir, frame, &robots, &colors);
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

fn render_multi(frames_dir: &Path, frame: u32, robots: &[Robot], colors: &[[f64; 3]]) {
    let total = 100u32;
    let angle = angle_for_frame(frame, total, 0.6, 4.6);
    let camera = Camera::orbit(Vec3::new(0.0, 0.1, 0.0), 3.4, 2.4, angle);
    let mut renderer = Renderer::new(640, 480, [10, 12, 18]);
    renderer.clear();
    renderer.ground_grid(&camera, 2.0, 0.5, [34, 40, 50]);
    for (index, robot) in robots.iter().enumerate() {
        let color = colors[index];
        for pair in robot.trail.windows(2) {
            renderer.line(
                &camera,
                Vec3::new(pair[0].0, 0.04, pair[0].1),
                Vec3::new(pair[1].0, 0.04, pair[1].1),
                [
                    (color[0] * 0.7) as u8,
                    (color[1] * 0.7) as u8,
                    (color[2] * 0.7) as u8,
                ],
                1,
            );
        }
        renderer.box_3d(
            &camera,
            Vec3::new(robot.goal.x, 0.01, robot.goal.z),
            Vec3::new(0.14, 0.005, 0.14),
            [color[0] * 0.5, color[1] * 0.5, color[2] * 0.5],
        );
        renderer.box_3d(
            &camera,
            Vec3::new(robot.pose.x_m, 0.12, robot.pose.y_m),
            Vec3::new(0.13, 0.12, 0.13),
            color,
        );
    }
    save_frame(&renderer, &frames_dir.join(format!("frame-{frame:03}.png")));
}

fn seek(pose: Pose2d, goal: Vec3) -> VelocityCommand2d {
    let dx = goal.x - pose.x_m;
    let dy = goal.y - pose.y_m;
    let distance = dx.hypot(dy);
    if distance < 0.1 {
        return VelocityCommand2d::ZERO;
    }
    let error = wrap_angle(dy.atan2(dx) - pose.yaw_rad);
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

// ---------------------------------------------------------------------------
// Elevation map
// ---------------------------------------------------------------------------

fn elevation_visual(media: &Path, frames: &Path) {
    let frames_dir = frames.join("elevation");
    reset_dir(&frames_dir);
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
            let waves = 0.45_f64 * (1.7_f64 * x).sin() * (1.5_f64 * z).cos();
            let hill = 0.55_f64 * (-((x - 0.6_f64).powi(2) + (z + 0.5_f64).powi(2)) / 0.5).exp();
            points.push(Vec3::new(x, waves + hill, z));
            z += 0.04;
        }
        x += 0.04;
    }
    // A cliff: two very different heights in one cell exceed the step threshold.
    points.push(Vec3::new(-1.05, 0.0, 0.4));
    points.push(Vec3::new(-1.05, 1.1, 0.4));
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

    let mut frame = 0u32;
    let total = 45u32;
    for index in 0..total + 10 {
        let angle = angle_for_frame(index, total, -2.0, 2.6);
        let camera = Camera::orbit(Vec3::new(0.0, 0.3, 0.0), 4.4, 3.0, angle);
        let mut renderer = Renderer::new(640, 480, [10, 12, 18]);
        renderer.clear();
        render_elevation(&mut renderer, &camera, &map, min_h, span);
        save_frame(&renderer, &frames_dir.join(format!("frame-{frame:03}.png")));
        frame += 1;
    }

    build_gif(&frames_dir, &media.join("nav-elevation.gif"), 640);
    std::fs::copy(
        frames_dir.join(format!("frame-{:03}.png", frame - 1)),
        media.join("nav-elevation.png"),
    )
    .expect("poster");
    println!("nav-elevation.gif: {} frames", frame);
}

fn render_elevation(
    renderer: &mut Renderer,
    camera: &Camera,
    map: &ElevationMap,
    min_h: f64,
    span: f64,
) {
    let resolution = map.resolution_m();
    let base_x = map.origin().x_m;
    let base_y = map.origin().y_m;
    let height_scale = 2.2;
    let vertex = |x: usize, y: usize| -> Option<Vec3> {
        let cell = map.cell(GridCoord {
            x: x as isize,
            y: y as isize,
        })?;
        if !cell.is_known() {
            return None;
        }
        Some(Vec3::new(
            base_x + x as f64 * resolution,
            (cell.mean_y_m - min_h) * height_scale + 0.05,
            base_y + y as f64 * resolution,
        ))
    };
    for y in 0..map.height() - 1 {
        for x in 0..map.width() - 1 {
            let (Some(a), Some(b), Some(c), Some(d)) = (
                vertex(x, y),
                vertex(x + 1, y),
                vertex(x + 1, y + 1),
                vertex(x, y + 1),
            ) else {
                continue;
            };
            let mean = map
                .cell(GridCoord {
                    x: x as isize,
                    y: y as isize,
                })
                .map(|cell| (cell.mean_y_m - min_h) / span)
                .unwrap_or(0.0);
            let color = height_color(mean);
            renderer.triangle(camera, a, b, c, color);
            renderer.triangle(camera, a, c, d, color);
        }
    }
}

fn height_color(t: f64) -> [f64; 3] {
    let stops = [
        (0.0, [40.0, 70.0, 140.0]),
        (0.35, [40.0, 160.0, 130.0]),
        (0.7, [235.0, 205.0, 95.0]),
        (1.0, [250.0, 250.0, 250.0]),
    ];
    let t = t.clamp(0.0, 1.0);
    for pair in stops.windows(2) {
        let (t0, c0) = pair[0];
        let (t1, c1) = pair[1];
        if t <= t1 {
            let f = (t - t0) / (t1 - t0).max(1.0e-9);
            return [
                c0[0] + (c1[0] - c0[0]) * f,
                c0[1] + (c1[1] - c0[1]) * f,
                c0[2] + (c1[2] - c0[2]) * f,
            ];
        }
    }
    [250.0, 250.0, 250.0]
}

// ---------------------------------------------------------------------------
// Synthetic 2D LiDAR for the SLAM scene
// ---------------------------------------------------------------------------

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
