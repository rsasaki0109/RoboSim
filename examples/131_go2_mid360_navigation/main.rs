//! Navigates the welded Go2 between two rooms on what its Mid-360 sees: online
//! 2D SLAM from the recording-matched sensor model, A* on an inflated costmap,
//! and a lookahead path follower into the model-based trot. No map is given in advance.
//!
//! What the robot uses, per 0.1 s Mid-360 frame:
//!
//! * **Points in the sensor frame.** Each return is expressed in the sensor
//!   frame at its own emission time, which is what the real driver publishes.
//! * **IMU attitude.** Roll and pitch level the points, as an IMU's gravity
//!   estimate does; heading is never taken from the simulation.
//! * **Leg odometry.** Body velocity and yaw rate integrated with a 4 % scale
//!   error and a 0.02 rad/s yaw-rate bias, so dead reckoning drifts. It also
//!   de-skews each frame to the pose at the frame's end.
//! * **A 2D scan** of the returns 0.15–0.65 m above the floor, 720 beams.
//!
//! `rne_slam::Slam2d` builds the occupancy map and corrects the odometry by scan
//! matching on keyframes (every 0.15 m or 0.15 rad). Planning treats unknown space as traversable and replans every
//! second, so the robot heads for a goal it cannot see yet and routes through
//! the doorway once the partition shows up in the map.
//!
//! The gate compares the estimate with the simulation's truth: both goals
//! reached, localization error, clearance, and that the map has the walls.
//!
//! ```text
//! cargo run --release -p go2_mid360_navigation --example 131_go2_mid360_navigation -- --smoke
//! cargo run --release -p go2_mid360_navigation --example 131_go2_mid360_navigation
//! ```

use std::f64::consts::PI;
use std::fs;
use std::path::{Path, PathBuf};
use std::sync::Arc;

use png::{BitDepth, ColorType, Encoder};
use rne_ai::{
    build_visual_render_scene, unitree_go2_mid360_mount, UnitreeGo2ModelTrot,
    UnitreeGo2TrotCommand, UrdfSceneSim, UNITREE_GO2_MID360_FORWARD_OF_BASE_M,
    UNITREE_GO2_MODEL_TROT_CONTROL_HZ,
};
use rne_data::PointCloud;
use rne_math::{Quat, Transform3, Vec3};
use rne_nav::{
    plan_path, Costmap, CostmapConfig, FrameId, GlobalPlannerConfig, GridCoord, LaserScan2d,
    OccupancyGrid, Path2d, Pose2d,
};
use rne_render::{
    Camera, MeshRenderCache, RenderBackend, RenderScene, RenderSceneItem, TriangleMesh, VisualShape,
};
use rne_render_wgpu::{CameraOrbit, WgpuRenderBackend};
use rne_sensor::{
    livox_mid360_spec, LidarRigOcclusion, LidarSpec, LidarSweep, LivoxMid360Pattern, SensorNoiseKey,
};
use rne_slam::{Slam2d, SlamConfig};
use rne_world::Transform3 as WorldTransform3;

/// Goals in world `[x, z]`: the far corner of room B, then back into room A.
const GOALS: [[f64; 2]; 2] = [[6.2, 1.6], [-1.5, -0.3]];
const MAX_DURATION_S: f64 = 180.0;
/// Path following.
const LOOKAHEAD_M: f64 = 0.6;
const MAX_SPEED_M_S: f64 = 0.25;
const MAX_YAW_RATE_RAD_S: f64 = 0.5;
const HEADING_GAIN_PER_S: f64 = 1.5;
const SLOW_RADIUS_M: f64 = 0.6;
const GOAL_TOLERANCE_M: f64 = 0.25;
/// Control steps per Mid-360 frame: 0.1 s at 500 Hz.
const STEPS_PER_LIDAR_FRAME: u64 = 50;
const REPLAN_EVERY_FRAMES: u64 = 10;
/// SLAM keyframes: a scan is processed once the odometry has moved this far.
const KEYFRAME_TRANSLATION_M: f64 = 0.15;
const KEYFRAME_ROTATION_RAD: f64 = 0.15;

/// Leg odometry errors: speed scale and yaw-rate bias. Chosen, not measured.
const ODOM_SCALE_ERROR: f64 = 0.04;
const ODOM_YAW_RATE_BIAS_RAD_S: f64 = 0.02;

/// Scan band relative to the sensor, which rides 0.447 m above the floor.
const SCAN_MIN_HEIGHT_M: f64 = -0.30;
const SCAN_MAX_HEIGHT_M: f64 = 0.20;
/// Returns closer than this are the robot itself.
const SCAN_MIN_RANGE_M: f64 = 0.40;
const SCAN_MAX_RANGE_M: f64 = 20.0;
const SCAN_BEAMS: usize = 720;

/// Map extent in the navigation frame (x forward, y = -world z), 5 cm cells.
const MAP_ORIGIN: [f64; 2] = [-3.0, -3.0];
const MAP_SIZE_M: [f64; 2] = [10.5, 6.0];
const MAP_RESOLUTION_M: f64 = 0.05;

/// Scene walls and furniture in world `(centre [x, z], half extents [x, z])`.
const OBSTACLES: [([f64; 2], [f64; 2]); 11] = [
    ([-2.45, 0.0], [0.05, 2.5]),
    ([7.05, 0.0], [0.05, 2.5]),
    ([2.3, -2.45], [4.8, 0.05]),
    ([2.3, 2.45], [4.8, 0.05]),
    ([2.4, -1.1], [0.05, 1.3]),
    ([2.4, 1.8], [0.05, 0.6]),
    ([0.2, -1.3], [0.8, 0.45]),
    ([-1.5, 1.4], [0.3, 0.3]),
    ([4.3, -1.9], [0.8, 0.25]),
    ([5.0, 0.9], [0.25, 0.25]),
    ([6.4, -0.6], [0.3, 0.3]),
];

const WIDTH: u32 = 960;
const HEIGHT: u32 = 540;
const CLEAR_COLOR: [f32; 4] = [0.035, 0.05, 0.08, 1.0];
const LIDAR_FRAMES_PER_GIF_FRAME: u64 = 8;
const COLORMAP_BUCKETS: usize = 16;
const CUTAWAY_HEIGHT_M: f64 = 0.12;

fn repo_path(relative: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../..")
        .join(relative)
}

/// World `[x, z]` to the navigation frame's `(x, y)`.
fn nav_xy(world: [f64; 2]) -> (f64, f64) {
    (world[0], -world[1])
}

/// Yaw of a world rotation about +y, zero along +x, positive turning left.
fn yaw_of(rotation: Quat) -> f64 {
    let facing = rotation * Vec3::X;
    (-facing.z).atan2(facing.x)
}

fn wrap_angle(angle: f64) -> f64 {
    (angle + PI).rem_euclid(2.0 * PI) - PI
}

struct Nav {
    sim: UrdfSceneSim,
    trot: UnitreeGo2ModelTrot,
    spec: LidarSpec,
    rig: LidarRigOcclusion,
    pattern: LivoxMid360Pattern,
    frame_index: u64,
    frame_start_pose: WorldTransform3,
    slam: Slam2d,
    /// Dead-reckoned base pose from leg odometry.
    odom: Pose2d,
    odom_at_frame_start: Pose2d,
    /// Latest SLAM pose and the odometry it was computed at.
    slam_pose: Pose2d,
    odom_at_slam: Pose2d,
    path: Option<Path2d>,
    goal: usize,
    goal_true_error_m: Vec<f64>,
    frames_since_plan: u64,
    last_cloud: Option<PointCloud>,
    true_trail: Vec<[f64; 2]>,
    slam_error_sq: f64,
    slam_error_max_m: f64,
    odom_error_max_m: f64,
    error_samples: u64,
    min_clearance_m: f64,
    min_height_m: f64,
    matched_frames: u64,
    loop_closures: usize,
}

impl Nav {
    fn new() -> Self {
        let mut sim = UrdfSceneSim::from_scene_path(&repo_path(
            "assets/scenes/unitree_go2_two_rooms.rne.scene.toml",
        ))
        .expect("load two-room scene");
        let trot = UnitreeGo2ModelTrot::stand_up(&mut sim);
        let rig: LidarRigOcclusion = serde_json::from_str(
            &fs::read_to_string(repo_path(
                "assets/sensors/livox_mid360/go2_rig_occlusion.json",
            ))
            .expect("read rig table"),
        )
        .expect("parse rig table");
        let frame_start_pose = sim
            .named_mount_transform("base", &unitree_go2_mid360_mount())
            .expect("mount pose");
        let grid = OccupancyGrid::new(
            (MAP_SIZE_M[0] / MAP_RESOLUTION_M) as usize,
            (MAP_SIZE_M[1] / MAP_RESOLUTION_M) as usize,
            MAP_RESOLUTION_M,
            Pose2d::new(MAP_ORIGIN[0], MAP_ORIGIN[1], 0.0),
        )
        .expect("map grid");
        // Odometry starts at the true start pose, so the map frame is the world's.
        let start = true_pose(&sim);
        Self {
            sim,
            trot,
            spec: livox_mid360_spec(),
            rig,
            pattern: LivoxMid360Pattern::new(),
            frame_index: 0,
            frame_start_pose,
            slam: Slam2d::new(grid, SlamConfig::default()),
            odom: start,
            odom_at_frame_start: start,
            slam_pose: start,
            odom_at_slam: start,
            path: None,
            goal: 0,
            goal_true_error_m: Vec::new(),
            frames_since_plan: REPLAN_EVERY_FRAMES,
            last_cloud: None,
            true_trail: Vec::new(),
            slam_error_sq: 0.0,
            slam_error_max_m: 0.0,
            odom_error_max_m: 0.0,
            error_samples: 0,
            min_clearance_m: f64::MAX,
            min_height_m: f64::MAX,
            matched_frames: 0,
            loop_closures: 0,
        }
    }

    fn done(&self) -> bool {
        self.goal >= GOALS.len() || self.trot.time_s() >= MAX_DURATION_S
    }

    /// The robot's best pose estimate now: the last SLAM pose moved by the
    /// odometry since.
    fn estimate(&self) -> Pose2d {
        self.slam_pose
            .compose(self.odom_at_slam.inverse().compose(self.odom))
    }

    fn step_frame(&mut self) {
        for _ in 0..STEPS_PER_LIDAR_FRAME {
            self.step_control();
        }
        let end_pose = self
            .sim
            .named_mount_transform("base", &unitree_go2_mid360_mount())
            .expect("mount pose");
        let sweep = LidarSweep::new(self.frame_start_pose, end_pose);
        let cloud = self.sim.sample_livox_mid360(
            &sweep,
            &self.spec,
            &self.pattern,
            self.frame_index,
            Some(&self.rig),
            SensorNoiseKey::new(self.sim.world_seed(), self.spec.seed, 1, self.frame_index),
        );
        let scan = self.build_scan(&cloud, &sweep);
        let since_slam = self.odom_at_slam.inverse().compose(self.odom);
        let keyframe = self.slam.scans_processed() == 0
            || since_slam.x_m.hypot(since_slam.y_m) >= KEYFRAME_TRANSLATION_M
            || since_slam.yaw_rad.abs() >= KEYFRAME_ROTATION_RAD;
        if keyframe {
            self.process_keyframe(&scan);
        }
        self.odom_at_frame_start = self.odom;
        self.frame_start_pose = end_pose;
        self.last_cloud = Some(cloud);
        self.frame_index += 1;

        let truth = true_pose(&self.sim);
        let estimate = self.estimate();
        let slam_error = (truth.x_m - estimate.x_m).hypot(truth.y_m - estimate.y_m);
        let odom_error = (truth.x_m - self.odom.x_m).hypot(truth.y_m - self.odom.y_m);
        self.slam_error_sq += slam_error * slam_error;
        self.slam_error_max_m = self.slam_error_max_m.max(slam_error);
        self.odom_error_max_m = self.odom_error_max_m.max(odom_error);
        self.error_samples += 1;

        self.frames_since_plan += 1;
        if self.frames_since_plan >= REPLAN_EVERY_FRAMES {
            self.replan();
        }
    }

    fn process_keyframe(&mut self, scan: &LaserScan2d) {
        let sensor_from_base = Pose2d::new(UNITREE_GO2_MID360_FORWARD_OF_BASE_M, 0.0, 0.0);
        let update = self
            .slam
            .process(scan, self.odom, sensor_from_base)
            .expect("slam update");
        if update.matched {
            self.matched_frames += 1;
        }
        self.loop_closures += update.loop_closures;
        self.slam_pose = update.pose;
        self.odom_at_slam = self.odom;
    }

    /// Turns a frame's returns into a 2D scan at the sensor's pose at the frame end.
    fn build_scan(&self, cloud: &PointCloud, sweep: &LidarSweep) -> LaserScan2d {
        let frame_motion = self.odom_at_frame_start.inverse().compose(self.odom);
        let sensor_from_base = Pose2d::new(UNITREE_GO2_MID360_FORWARD_OF_BASE_M, 0.0, 0.0);
        let mut ranges = vec![f64::NAN; SCAN_BEAMS];
        let increment = 2.0 * PI / SCAN_BEAMS as f64;
        for (point, time_s) in cloud.points_m.iter().zip(cloud.timestamps_s.iter()) {
            let fraction = (time_s / self.spec.rotation_period_s).clamp(0.0, 1.0);
            // What the driver reports: the return in the sensor frame at emission.
            let sensor = sweep.pose_at(fraction);
            let raw = sensor.rotation.inverse() * (*point - sensor.translation);
            // IMU attitude: remove roll and pitch, keep the sensor's own heading.
            let yaw = yaw_of(sensor.rotation);
            let tilt = Quat::from_rotation_y(yaw).inverse() * sensor.rotation;
            let level = tilt * raw;
            let height = level.y;
            if !(SCAN_MIN_HEIGHT_M..=SCAN_MAX_HEIGHT_M).contains(&height) {
                continue;
            }
            // Leveled axes: +x forward, +y left.
            let at_emission = Vec3::new(level.x, -level.z, 0.0);
            if at_emission.length() < SCAN_MIN_RANGE_M {
                continue;
            }
            // De-skew with odometry: the base pose at emission, relative to the
            // frame end, interpolated along the frame's odometry motion.
            let partial = Pose2d::new(
                frame_motion.x_m * fraction,
                frame_motion.y_m * fraction,
                frame_motion.yaw_rad * fraction,
            );
            let emission_from_end = frame_motion.inverse().compose(partial);
            let base_point = sensor_from_base.transform_point(at_emission);
            let at_end = sensor_from_base
                .inverse()
                .transform_point(emission_from_end.transform_point(base_point));
            let range = at_end.x.hypot(at_end.y);
            let beam = ((at_end.y.atan2(at_end.x) + PI) / increment).floor() as usize % SCAN_BEAMS;
            if ranges[beam].is_nan() || range < ranges[beam] {
                ranges[beam] = range;
            }
        }
        LaserScan2d {
            time_s: self.trot.time_s(),
            frame: FrameId::new("mid360_level"),
            angle_min_rad: -PI + 0.5 * increment,
            angle_increment_rad: increment,
            range_min_m: SCAN_MIN_RANGE_M,
            range_max_m: SCAN_MAX_RANGE_M,
            ranges_m: ranges,
        }
    }

    fn replan(&mut self) {
        let Some(goal) = GOALS.get(self.goal) else {
            return;
        };
        self.frames_since_plan = 0;
        let costmap =
            Costmap::from_occupancy(self.slam.grid(), &costmap_config()).expect("costmap");
        let estimate = self.estimate();
        let (gx, gy) = nav_xy(*goal);
        let planner = GlobalPlannerConfig {
            allow_unknown: true,
            ..GlobalPlannerConfig::default()
        };
        if let Ok(path) = plan_path(
            &costmap,
            Vec3::new(estimate.x_m, estimate.y_m, 0.0),
            Vec3::new(gx, gy, 0.0),
            &planner,
        ) {
            self.path = Some(path.prune_collinear(0.02));
        }
    }

    fn step_control(&mut self) {
        let command = self.follow();
        self.trot.step(&mut self.sim, command);
        self.integrate_odometry();
        let observed = self.sim.observe();
        if self.trot.steps().is_multiple_of(125) {
            self.true_trail.push([observed.base_x_m, observed.base_z_m]);
        }
        self.min_height_m = self.min_height_m.min(observed.base_y_m);
        self.min_clearance_m = self
            .min_clearance_m
            .min(clearance_m([observed.base_x_m, observed.base_z_m]));
    }

    /// Heads for a point on the path 0.6 m ahead of the closest one: the yaw rate
    /// turns the bearing error, and the trot slows while that error is large and
    /// when the goal is near.
    fn follow(&mut self) -> UnitreeGo2TrotCommand {
        let Some(path) = &self.path else {
            return UnitreeGo2TrotCommand::default();
        };
        let estimate = self.estimate();
        let position = Vec3::new(estimate.x_m, estimate.y_m, 0.0);
        let (Some(goal), Some(closest)) = (path.goal(), path.closest_point(position)) else {
            return UnitreeGo2TrotCommand::default();
        };
        let to_goal_m = (goal.x_m - estimate.x_m).hypot(goal.y_m - estimate.y_m);
        if to_goal_m < GOAL_TOLERANCE_M {
            let truth = true_pose(&self.sim);
            let (gx, gy) = nav_xy(GOALS[self.goal]);
            self.goal_true_error_m
                .push((truth.x_m - gx).hypot(truth.y_m - gy));
            self.goal += 1;
            self.path = None;
            self.frames_since_plan = REPLAN_EVERY_FRAMES;
            return UnitreeGo2TrotCommand::default();
        }
        let lookahead = path
            .point_at_distance(closest.arc_length_m + LOOKAHEAD_M)
            .unwrap_or(Vec3::new(goal.x_m, goal.y_m, 0.0));
        let bearing = wrap_angle(
            (lookahead.y - estimate.y_m).atan2(lookahead.x - estimate.x_m) - estimate.yaw_rad,
        );
        let alignment = bearing.cos().max(0.0).powi(2);
        let approach = (to_goal_m / SLOW_RADIUS_M).clamp(0.4, 1.0);
        UnitreeGo2TrotCommand {
            forward_speed_m_s: if bearing.abs() > 1.0 {
                0.05
            } else {
                MAX_SPEED_M_S * alignment * approach
            },
            yaw_rate_rad_s: (HEADING_GAIN_PER_S * bearing)
                .clamp(-MAX_YAW_RATE_RAD_S, MAX_YAW_RATE_RAD_S),
        }
    }

    /// Leg odometry: the body's planar velocity and yaw rate with a scale error
    /// and a yaw-rate bias, integrated at the control rate.
    fn integrate_odometry(&mut self) {
        let observed = self.sim.observe();
        let base = self.sim.named_transform("base").expect("base pose");
        let yaw = yaw_of(base.rotation);
        let (vx, vy) = (
            observed.base_linear_velocity_x_m_s,
            -observed.base_linear_velocity_z_m_s,
        );
        let (sin, cos) = yaw.sin_cos();
        let forward = cos * vx + sin * vy;
        let left = -sin * vx + cos * vy;
        let dt = 1.0 / UNITREE_GO2_MODEL_TROT_CONTROL_HZ;
        let scale = 1.0 + ODOM_SCALE_ERROR;
        let delta = Pose2d::new(
            forward * scale * dt,
            left * scale * dt,
            (observed.base_angular_velocity_y_rad_s + ODOM_YAW_RATE_BIAS_RAD_S) * dt,
        );
        self.odom = self.odom.compose(delta);
    }

    fn slam_rms_m(&self) -> f64 {
        (self.slam_error_sq / self.error_samples.max(1) as f64).sqrt()
    }
}

fn costmap_config() -> CostmapConfig {
    CostmapConfig {
        // The Go2 is 0.31 m wide; keep its centre 0.3 m off obstacles.
        inscribed_radius_m: 0.30,
        inflation_radius_m: 0.75,
        ..CostmapConfig::default()
    }
}

/// The base's true planar pose in the navigation frame.
fn true_pose(sim: &UrdfSceneSim) -> Pose2d {
    let base = sim.named_transform("base").expect("base pose");
    Pose2d::new(
        base.translation.x,
        -base.translation.z,
        yaw_of(base.rotation),
    )
}

fn clearance_m(position: [f64; 2]) -> f64 {
    OBSTACLES
        .iter()
        .map(|(center, half)| {
            let dx = ((position[0] - center[0]).abs() - half[0]).max(0.0);
            let dz = ((position[1] - center[1]).abs() - half[1]).max(0.0);
            dx.hypot(dz)
        })
        .fold(f64::MAX, f64::min)
}

/// Occupied map cells, and the fraction lying within 10 cm of a real obstacle.
fn map_precision(grid: &OccupancyGrid) -> (usize, f64) {
    let mut occupied = 0;
    let mut on_obstacle = 0;
    for y in 0..grid.height() {
        for x in 0..grid.width() {
            let coord = GridCoord {
                x: x as isize,
                y: y as isize,
            };
            if !grid.is_occupied(coord) {
                continue;
            }
            occupied += 1;
            let center = grid.grid_to_world(coord);
            if clearance_m([center.x, -center.y]) < 0.10 {
                on_obstacle += 1;
            }
        }
    }
    (occupied, on_obstacle as f64 / occupied.max(1) as f64)
}

fn run(mut on_frame: impl FnMut(&Nav)) -> Nav {
    let mut nav = Nav::new();
    while !nav.done() {
        nav.step_frame();
        on_frame(&nav);
    }
    nav
}

fn report_and_gate(nav: &Nav) {
    let (occupied, precision) = map_precision(nav.slam.grid());
    println!(
        "reached {}/{} goals in {:.1} s (true error at arrival {:?} m); clearance >= {:.2} m, lowest body {:.3} m",
        nav.goal,
        GOALS.len(),
        nav.trot.time_s(),
        nav.goal_true_error_m
            .iter()
            .map(|e| (e * 100.0).round() / 100.0)
            .collect::<Vec<_>>(),
        nav.min_clearance_m,
        nav.min_height_m
    );
    println!(
        "localization: SLAM {:.3} m RMS ({:.3} max) vs odometry alone {:.3} m max; {} keyframes matched over {} frames, {} loop closures",
        nav.slam_rms_m(),
        nav.slam_error_max_m,
        nav.odom_error_max_m,
        nav.matched_frames,
        nav.frame_index,
        nav.loop_closures
    );
    println!(
        "map: {occupied} occupied cells, {:.1} % within 10 cm of a real obstacle",
        precision * 100.0
    );
    assert_eq!(nav.goal, GOALS.len(), "not every goal was reached");
    assert!(
        nav.goal_true_error_m.iter().all(|e| *e < 0.5),
        "arrived off goal: {:?}",
        nav.goal_true_error_m
    );
    assert!(
        nav.min_height_m > 0.2,
        "the walk sagged: {:.3} m",
        nav.min_height_m
    );
    assert!(
        nav.min_clearance_m > 0.2,
        "came within {:.2} m of an obstacle",
        nav.min_clearance_m
    );
    assert!(
        nav.slam_error_max_m < nav.odom_error_max_m,
        "SLAM did not beat odometry"
    );
    assert!(
        nav.slam_rms_m() < 0.15,
        "localization RMS {:.3} m",
        nav.slam_rms_m()
    );
    // Measured 87.9 %: with ~0.1 m localization error some wall cells land a cell
    // or two off the wall.
    assert!(precision > 0.85, "map precision {:.3}", precision);
}

fn main() {
    if std::env::args().any(|arg| arg == "--smoke") {
        let nav = run(|_| {});
        report_and_gate(&nav);
        println!("smoke ok: the Go2 mapped and navigated both rooms on its Mid-360");
        return;
    }
    render_media();
}

fn render_media() {
    let view = CameraOrbit {
        focus: Vec3::new(2.3, 0.0, 0.2),
        yaw_rad: -2.05,
        pitch_rad: 0.95,
        distance_m: 8.4,
    };
    let mut backend = WgpuRenderBackend::new().expect("initialize wgpu");
    let camera = Camera::new(WIDTH, HEIGHT, std::f64::consts::FRAC_PI_4);
    let mut mesh_cache = MeshRenderCache::new();
    let frames_dir = repo_path("target/rne-go2-mid360-navigation-frames");
    let _ = fs::remove_dir_all(&frames_dir);
    fs::create_dir_all(&frames_dir).expect("create frame directory");
    let mut frame = 0_usize;
    let nav = run(|nav| {
        if !nav.frame_index.is_multiple_of(LIDAR_FRAMES_PER_GIF_FRAME) {
            return;
        }
        let rgba = render_frame(&mut backend, &camera, &mut mesh_cache, nav, &view);
        write_png(&frames_dir.join(format!("frame-{frame:03}.png")), &rgba).expect("write frame");
        frame += 1;
    });
    report_and_gate(&nav);
    let media_dir = repo_path("docs/media");
    let gif_path = media_dir.join("go2-mid360-navigation.gif");
    build_gif(&frames_dir, &gif_path).expect("encode gif");
    image::open(frames_dir.join(format!("frame-{:03}.png", frame * 3 / 5)))
        .expect("read poster frame")
        .save(media_dir.join("go2-mid360-navigation.png"))
        .expect("write poster");
    println!("wrote {} ({frame} frames)", gif_path.display());
}

fn render_frame(
    backend: &mut WgpuRenderBackend,
    camera: &Camera,
    mesh_cache: &mut MeshRenderCache,
    nav: &Nav,
    view: &CameraOrbit,
) -> Vec<u8> {
    let mut scene = build_visual_render_scene(nav.sim.world());
    scene
        .items
        .retain(|item| !matches!(item.shape, VisualShape::Box { size_m } if size_m.x >= 20.0));
    // Cutaway: walls and furniture drawn 12 cm tall, so the map on the floor and
    // the returns on the walls show. Their true heights are in the scene.
    for item in &mut scene.items {
        if let VisualShape::Box { size_m } = item.shape {
            let height_m = size_m.y * item.transform.scale.y;
            if height_m > CUTAWAY_HEIGHT_M {
                item.transform.scale.y *= CUTAWAY_HEIGHT_M / height_m;
                item.transform.translation.y = CUTAWAY_HEIGHT_M * 0.5;
            }
            for channel in &mut item.color_rgba[..3] {
                *channel *= 0.6;
            }
        }
    }
    scene.items.push(box_item(
        Vec3::new(2.3, -0.005, 0.0),
        Vec3::new(9.6, 0.01, 5.0),
        [0.06, 0.07, 0.09, 1.0],
    ));
    append_occupancy(&mut scene, nav.slam.grid());
    if let Some(path) = &nav.path {
        let mut mesh = QuadMesh::default();
        for pair in path.waypoints().windows(2) {
            mesh.add_segment(
                Vec3::new(pair[0].x_m, 0.02, -pair[0].y_m),
                Vec3::new(pair[1].x_m, 0.02, -pair[1].y_m),
                0.035,
            );
        }
        push_mesh(&mut scene, mesh, [0.98, 0.84, 0.25, 1.0]);
    }
    for (index, goal) in GOALS.iter().enumerate() {
        let color = if index < nav.goal {
            [0.35, 0.85, 0.45, 1.0]
        } else {
            [0.95, 0.35, 0.30, 1.0]
        };
        scene.items.push(box_item(
            Vec3::new(goal[0], 0.015, goal[1]),
            Vec3::new(0.22, 0.02, 0.22),
            color,
        ));
    }
    let mut trail = QuadMesh::default();
    for pair in nav.true_trail.windows(2) {
        trail.add_segment(
            Vec3::new(pair[0][0], 0.012, pair[0][1]),
            Vec3::new(pair[1][0], 0.012, pair[1][1]),
            0.02,
        );
    }
    push_mesh(&mut scene, trail, [0.55, 0.62, 0.72, 1.0]);
    if let Some(cloud) = &nav.last_cloud {
        append_height_colored_points(&mut scene, cloud);
    }
    let roots: Vec<&Path> = nav
        .sim
        .mesh_package_roots()
        .iter()
        .map(PathBuf::as_path)
        .collect();
    mesh_cache
        .resolve_scene(&mut scene, &roots)
        .expect("resolve official Go2 meshes");
    backend
        .render_scene_camera(camera, &view.camera_transform(), &scene, CLEAR_COLOR)
        .expect("render frame")
        .color
        .rgba8
}

/// The SLAM map on the floor: occupied cells bright, known-free cells dim.
fn append_occupancy(scene: &mut RenderScene, grid: &OccupancyGrid) {
    let mut occupied = QuadMesh::default();
    let mut free = QuadMesh::default();
    let half = grid.resolution_m() * 0.5;
    for y in 0..grid.height() {
        for x in 0..grid.width() {
            let coord = GridCoord {
                x: x as isize,
                y: y as isize,
            };
            let center = grid.grid_to_world(coord);
            let world = Vec3::new(center.x, 0.0, -center.y);
            if grid.is_occupied(coord) {
                occupied.add_floor_square(world + Vec3::Y * 0.008, half);
            } else if grid.is_free(coord) {
                free.add_floor_square(world + Vec3::Y * 0.004, half);
            }
        }
    }
    push_mesh(scene, free, [0.16, 0.27, 0.34, 1.0]);
    push_mesh(scene, occupied, [0.92, 0.36, 0.55, 1.0]);
}

fn append_height_colored_points(scene: &mut RenderScene, cloud: &PointCloud) {
    let mut buckets: Vec<QuadMesh> = (0..COLORMAP_BUCKETS).map(|_| QuadMesh::default()).collect();
    for point in &cloud.points_m {
        let t = point.y.clamp(0.0, 1.0);
        let bucket =
            ((t * (COLORMAP_BUCKETS - 1) as f64).round() as usize).min(COLORMAP_BUCKETS - 1);
        buckets[bucket].add_marker(*point, 0.02);
    }
    for (bucket, mesh) in buckets.into_iter().enumerate() {
        let t = bucket as f64 / (COLORMAP_BUCKETS - 1) as f64;
        push_mesh(scene, mesh, turbo_colormap(0.1 + 0.85 * t));
    }
}

fn box_item(translation: Vec3, scale: Vec3, color: [f32; 4]) -> RenderSceneItem {
    RenderSceneItem {
        transform: Transform3 {
            translation,
            rotation: Quat::IDENTITY,
            scale,
        },
        shape: VisualShape::Box { size_m: Vec3::ONE },
        color_rgba: color,
        mesh: None,
        base_color_texture: None,
        material: Default::default(),
    }
}

#[derive(Default)]
struct QuadMesh {
    positions: Vec<[f32; 3]>,
    normals: Vec<[f32; 3]>,
    texcoords: Vec<[f32; 2]>,
    indices: Vec<u32>,
}

impl QuadMesh {
    fn add_marker(&mut self, center: Vec3, radius_m: f64) {
        let x = Vec3::X * radius_m;
        let y = Vec3::Y * radius_m;
        let z = Vec3::Z * radius_m;
        self.add_quad(center - x, center + y, center - y, center + x);
        self.add_quad(center - z, center + y, center - y, center + z);
        self.add_quad(center - x, center + z, center - z, center + x);
    }

    fn add_floor_square(&mut self, center: Vec3, half_m: f64) {
        self.add_quad(
            center + Vec3::new(-half_m, 0.0, -half_m),
            center + Vec3::new(half_m, 0.0, -half_m),
            center + Vec3::new(-half_m, 0.0, half_m),
            center + Vec3::new(half_m, 0.0, half_m),
        );
    }

    fn add_segment(&mut self, start: Vec3, end: Vec3, width_m: f64) {
        let delta = end - start;
        let side = Vec3::new(-delta.z, 0.0, delta.x).normalize_or_zero() * width_m * 0.5;
        self.add_quad(start - side, start + side, end - side, end + side);
    }

    fn add_quad(&mut self, first: Vec3, second: Vec3, third: Vec3, fourth: Vec3) {
        let base = self.positions.len() as u32;
        self.positions
            .extend([first, second, third, fourth].map(|p| [p.x as f32, p.y as f32, p.z as f32]));
        self.normals.extend([[0.0, 1.0, 0.0]; 4]);
        self.texcoords.extend([[0.0, 0.0]; 4]);
        // Both windings, so the quad shows from either side.
        self.indices.extend([
            base,
            base + 1,
            base + 2,
            base + 2,
            base + 1,
            base + 3,
            base,
            base + 2,
            base + 1,
            base + 2,
            base + 3,
            base + 1,
        ]);
    }
}

fn push_mesh(scene: &mut RenderScene, mesh: QuadMesh, color_rgba: [f32; 4]) {
    if mesh.indices.is_empty() {
        return;
    }
    scene.items.push(RenderSceneItem {
        transform: Transform3::IDENTITY,
        shape: VisualShape::DynamicMesh,
        color_rgba,
        mesh: Some(Arc::new(TriangleMesh {
            positions: mesh.positions,
            normals: mesh.normals,
            texcoords: mesh.texcoords,
            indices: mesh.indices,
            skinning: None,
        })),
        base_color_texture: None,
        material: Default::default(),
    });
}

/// Google's Turbo colormap, polynomial approximation.
fn turbo_colormap(t: f64) -> [f32; 4] {
    let t = t.clamp(0.0, 1.0);
    let polynomial = |c: [f64; 6]| -> f32 {
        (c[0] + t * (c[1] + t * (c[2] + t * (c[3] + t * (c[4] + t * c[5]))))).clamp(0.0, 1.0) as f32
    };
    [
        polynomial([
            0.135_721_38,
            4.615_392_60,
            -42.660_322_58,
            132.131_082_34,
            -152.942_393_96,
            59.286_379_43,
        ]),
        polynomial([
            0.091_402_61,
            2.194_188_39,
            4.842_966_58,
            -14.185_033_33,
            4.277_298_57,
            2.829_566_04,
        ]),
        polynomial([
            0.106_673_30,
            12.641_946_08,
            -60.582_048_36,
            110.362_767_71,
            -89.903_109_12,
            27.348_249_73,
        ]),
        1.0,
    ]
}

fn build_gif(frames_dir: &Path, gif_path: &Path) -> std::io::Result<()> {
    let status = std::process::Command::new("ffmpeg")
        .args(["-y", "-loglevel", "error", "-framerate", "8", "-i"])
        .arg(frames_dir.join("frame-%03d.png"))
        .args([
            "-vf",
            "scale=720:-1:flags=lanczos,split[a][b];[a]palettegen=max_colors=96:stats_mode=diff[p];[b][p]paletteuse=dither=none:diff_mode=rectangle",
        ])
        .arg(gif_path)
        .status()?;
    status
        .success()
        .then_some(())
        .ok_or_else(|| std::io::Error::other("ffmpeg gif encode failed"))
}

fn write_png(path: &Path, rgba: &[u8]) -> std::io::Result<()> {
    let file = fs::File::create(path)?;
    let mut encoder = Encoder::new(file, WIDTH, HEIGHT);
    encoder.set_color(ColorType::Rgba);
    encoder.set_depth(BitDepth::Eight);
    encoder
        .write_header()?
        .write_image_data(rgba)
        .map_err(std::io::Error::other)
}
