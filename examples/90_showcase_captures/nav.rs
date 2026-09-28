//! Office AGV navigation showcase: the AGV drives to the desk through a corridor
//! it shares with a second AGV, a pedestrian crossing, and a hand truck left in
//! the aisle. None of them is on its map. It sees them with its LiDAR, tracks
//! them, replans its `plan_path` route around what the tracks predict, follows
//! it with pure pursuit, and gives way with reciprocal velocity avoidance.

use super::media::{
    capture_frames, push_box, push_sphere, CameraEvidence, CaptureFrame, ShowcaseMetadata,
    SimulationEvidence, FRAME_COUNT,
};
use super::nav_world::{
    avoidance_config, densify_lidar, detect, push_hand_truck, Corridor, PlanarPose, AGV_HALF_M,
    AGV_RADIUS_M, HAND_TRUCK,
};
use super::office;
use anyhow::{Context, Result};
use rne_ai::{office_agv_delivery_scene_path, DiffDriveAction, DiffDriveSim};
use rne_math::Vec3;
use rne_nav::{
    avoid_velocities, plan_path, pure_pursuit_follow, CircularObstacle, Costmap, CostmapConfig,
    FollowResult, GlobalPlannerConfig, GridCoord, ObstacleTracker, ObstacleTrackerConfig,
    OccupancyGrid, Path2d, Pose2d, PurePursuitConfig, Track, VelocityCommand2d, COST_LETHAL,
};
use rne_physics::hash_physics_state;
use rne_render::{grid_mesh, GridMeshSpec, MeshRenderCache, RenderScene};
use rne_render_wgpu::CameraOrbit;
use serde_json::to_vec_pretty;
use std::fs;
use std::path::Path;

const ENVIRONMENT_ID: &str = "nav";
const SUBJECT: &str =
    "office AGV navigation: LiDAR tracking, replanning and reciprocal avoidance in a shared corridor";
const CAMERA: CameraEvidence = CameraEvidence {
    fov_y_rad: std::f64::consts::FRAC_PI_4,
    yaw_rad: 0.32,
    pitch_rad: 0.78,
    distance_m: 6.4,
};

/// Drives the office AGV to the desk along replanned routes and captures it
/// with navigation overlays.
pub fn run(repo_root: &Path, capture: bool) -> Result<ShowcaseMetadata> {
    let first = rollout(repo_root, false, None)?;
    let replay = rollout(repo_root, false, Some(first.steps))?;
    anyhow::ensure!(
        first.final_digest == replay.final_digest,
        "Nav replay digest mismatch: {:#x} != {:#x}",
        first.final_digest,
        replay.final_digest
    );
    let evidence = SimulationEvidence {
        scenario: "office AGV route following (examples/90_showcase_captures/nav.rs)",
        steps: first.steps,
        initial_state_digest: first.initial_digest,
        final_state_digest: first.final_digest,
        replay_final_state_digest: replay.final_digest,
        replay_match: true,
        outcome: first.outcome.clone(),
    };
    let capture_evidence = if capture {
        let captured = rollout(repo_root, true, Some(first.steps))?;
        let orbit = CameraOrbit {
            focus: Vec3::new(4.65, 0.55, 0.0),
            yaw_rad: CAMERA.yaw_rad,
            pitch_rad: CAMERA.pitch_rad,
            distance_m: CAMERA.distance_m,
        };
        Some(capture_frames(
            repo_root,
            ENVIRONMENT_ID,
            &captured.frames,
            orbit,
            [0.32, 0.36, 0.42, 1.0],
            FRAME_COUNT / 2,
        )?)
    } else {
        None
    };
    let metadata = ShowcaseMetadata {
        kind: "rne_showcase_environment_metadata",
        schema_version: 1,
        environment_id: ENVIRONMENT_ID,
        subject: SUBJECT,
        visual_state_sync: "The orange AGV is the office scene's physics diff-drive robot, driven by wheel speeds. The blue AGV, the pedestrian and the hand truck are kinematic bodies in the same physics world, so the orange AGV's LiDAR hits them; red dots are that scan's returns the static map does not explain, and rings are the tracker's tracks. Route, costmap and trails are the planner's own state drawn as overlays.",
        simulation: evidence,
        capture: capture_evidence,
        camera: CAMERA,
        provenance: vec![
            "assets/scenes/office_agv_delivery.rne.scene.toml",
            "assets/fixtures/rigged_figure/RiggedFigure.glb",
            "assets/props/polyhaven_warehouse/hand_truck",
            "crates/rne_nav/src/planner.rs",
            "crates/rne_nav/src/control.rs",
            "crates/rne_nav/src/dynamic.rs",
            "crates/rne_nav/src/avoidance.rs",
            "examples/90_showcase_captures/nav.rs",
            "examples/90_showcase_captures/nav_world.rs",
        ],
        reproduce_smoke: "cargo run --locked -p showcase_captures --example 90_showcase_captures -- --smoke --environment nav",
        reproduce_capture: "cargo run --release --locked -p showcase_captures --example 90_showcase_captures -- --capture --environment nav",
    };
    if capture {
        let path = repo_root.join("docs/media/showcase-nav.json");
        fs::write(&path, to_vec_pretty(&metadata)?)
            .with_context(|| format!("write {}", path.display()))?;
    }
    Ok(metadata)
}

struct Rollout {
    steps: u64,
    initial_digest: u64,
    final_digest: u64,
    outcome: String,
    frames: Vec<CaptureFrame>,
}

/// Where the AGV parks: in front of the desk, facing it.
const GOAL_XZ_M: (f64, f64) = (6.5, 0.0);
/// Wheel radius and track of `office_agv_delivery.rne.robot.toml`.
const WHEEL_RADIUS_M: f64 = 0.1;
const TRACK_WIDTH_M: f64 = 0.45;
const MAX_WHEEL_RAD_S: f64 = 10.0;
/// How far ahead a moving track's footprint is swept into the costmap.
const PREDICTION_S: f64 = 2.0;
/// Tracks slower than this are treated as standing still.
const MOVING_M_S: f64 = 0.15;
/// Perception rate (the LiDAR's) and replan rate, as step intervals at 60 Hz.
const PERCEIVE_EVERY_STEPS: u64 = 6;
const REPLAN_EVERY_STEPS: u64 = 12;
/// Heading error at which the docked AGV counts as facing the desk.
const DOCKED_YAW_RAD: f64 = 0.02;
/// Steps held stopped at the goal before the episode ends.
const SETTLE_STEPS: u64 = 30;
const MAX_STEPS: u64 = 4_000;

/// The static corridor, the ego's tracks, and the latest route.
struct Navigator {
    corridor: OccupancyGrid,
    grid: OccupancyGrid,
    costmap: Costmap,
    path: Path2d,
    tracker: ObstacleTracker,
    /// Returns from the latest scan that the static map does not explain.
    unexplained: Vec<Vec3>,
    replans: u32,
}

impl Navigator {
    fn new() -> Result<Self> {
        let corridor = corridor_grid()?;
        let (costmap, path) = plan_route(&corridor, Vec3::new(0.5, 0.0, 0.0))?;
        Ok(Self {
            grid: corridor.clone(),
            corridor,
            costmap,
            path,
            tracker: ObstacleTracker::new(ObstacleTrackerConfig {
                gate_m: 0.6,
                max_missed: 4,
                velocity_gain: 0.4,
            })
            .context("obstacle tracker")?,
            unexplained: Vec::new(),
            replans: 0,
        })
    }

    /// Segments the latest scan and updates the tracks.
    fn perceive(&mut self, sim: &DiffDriveSim, dt_s: f64) -> Result<()> {
        let Some(cloud) = sim.latest_lidar_cloud() else {
            return Ok(());
        };
        let explained = |x: f64, z: f64| explained_by_map(x, z);
        self.unexplained = cloud
            .points_m
            .iter()
            .copied()
            .filter(|point| !explained(point.x, point.z))
            .collect();
        let detections = detect(&cloud.points_m, explained);
        self.tracker
            .update(&detections, dt_s)
            .context("track update")?;
        Ok(())
    }

    /// Writes every track into a copy of the corridor map, a moving one swept
    /// over the prediction horizon, and plans again from `start`. A failed plan
    /// keeps the previous route.
    fn replan(&mut self, start: Vec3) {
        let mut grid = self.corridor.clone();
        for track in self.tracker.tracks() {
            let velocity = track.velocity_m_s;
            let moving = velocity.length() > MOVING_M_S;
            let horizon_s = if moving { PREDICTION_S } else { 0.0 };
            let samples = (horizon_s / 0.2).ceil() as usize;
            for sample in 0..=samples {
                let ahead = velocity * (sample as f64 * 0.2).min(horizon_s);
                mark_disk(
                    &mut grid,
                    track.position_m.x + ahead.x,
                    track.position_m.y + ahead.y,
                    track.radius_m,
                );
            }
        }
        if let Ok((costmap, path)) = plan_route(&grid, start) {
            self.costmap = costmap;
            self.path = path;
        }
        self.grid = grid;
        self.replans += 1;
    }

    /// The moving tracks, as avoidance obstacles.
    fn moving_obstacles(&self) -> Vec<CircularObstacle> {
        self.tracker
            .tracks()
            .iter()
            .filter(|track| track.velocity_m_s.length() > MOVING_M_S)
            .map(|track: &Track| CircularObstacle {
                center_m: track.position_m,
                velocity_m_s: track.velocity_m_s,
                radius_m: track.radius_m,
            })
            .collect()
    }
}

/// Keeps the curvature pure pursuit chose but not its turn slowdown, which
/// crawls through the corners of a grid route: the AGV holds cruise speed
/// unless the yaw-rate limit or the approach to the goal slows it.
fn regulated(follow: FollowResult, config: &PurePursuitConfig) -> VelocityCommand2d {
    let command = follow.command;
    if command.linear_m_s <= 1.0e-6 {
        return command;
    }
    let curvature = command.angular_rad_s / command.linear_m_s;
    let approach = (follow.distance_to_goal_m / config.slow_radius_m).clamp(0.15, 1.0);
    let linear = (config.max_linear_m_s * approach)
        .min(config.max_angular_rad_s / curvature.abs().max(1.0e-6));
    VelocityCommand2d::new(linear, linear * curvature)
}

/// Gap between two AGV footprints, each bounded by the box around it at its
/// heading; zero or less means contact. Errs on the small side.
fn footprint_gap_m(a: (f64, f64, f64), b: (f64, f64, f64)) -> f64 {
    let bound = |heading: f64| {
        let (sin, cos) = heading.sin_cos();
        (
            AGV_HALF_M.0 * cos.abs() + AGV_HALF_M.1 * sin.abs(),
            AGV_HALF_M.0 * sin.abs() + AGV_HALF_M.1 * cos.abs(),
        )
    };
    let (ax, az) = bound(a.2);
    let (bx, bz) = bound(b.2);
    let gap_x = (a.0 - b.0).abs() - ax - bx;
    let gap_z = (a.1 - b.1).abs() - az - bz;
    if gap_x > 0.0 && gap_z > 0.0 {
        gap_x.hypot(gap_z)
    } else {
        gap_x.max(gap_z)
    }
}

/// What the rollout measured.
#[derive(Default)]
struct Tally {
    min_agv_gap_m: f64,
    min_pedestrian_gap_m: f64,
    yield_steps: u32,
    max_tracks: usize,
    truck_route_clearance_m: f64,
}

/// Converts a command in the planner's (x, z) plane to wheel speeds. There the
/// AGV's heading is `-yaw`, so a positive turn is a negative yaw rate.
fn wheel_action(command: VelocityCommand2d) -> DiffDriveAction {
    let half_track = -command.angular_rad_s * TRACK_WIDTH_M / 2.0;
    DiffDriveAction {
        left_velocity_rad_s: ((command.linear_m_s - half_track) / WHEEL_RADIUS_M)
            .clamp(-MAX_WHEEL_RAD_S, MAX_WHEEL_RAD_S),
        right_velocity_rad_s: ((command.linear_m_s + half_track) / WHEEL_RADIUS_M)
            .clamp(-MAX_WHEEL_RAD_S, MAX_WHEEL_RAD_S),
    }
}

fn rollout(repo_root: &Path, capture: bool, expected_steps: Option<u64>) -> Result<Rollout> {
    let mut sim =
        DiffDriveSim::from_scene_path(&office_agv_delivery_scene_path()).context("office scene")?;
    densify_lidar(&mut sim)?;
    let mut corridor = Corridor::spawn(&mut sim, repo_root)?;
    let dt_s = sim.fixed_delta().as_seconds().value();
    let mut navigator = Navigator::new()?;
    let pursuit = PurePursuitConfig {
        lookahead_m: 0.8,
        max_linear_m_s: 0.5,
        max_angular_rad_s: 1.5,
        goal_tolerance_m: 0.05,
        slow_radius_m: 0.6,
    };
    let initial_digest = hash_physics_state(sim.world());
    let sample_steps: Vec<u64> = match (capture, expected_steps) {
        (true, Some(total)) => (1..=FRAME_COUNT)
            .map(|index| ((index as u64 * total).div_ceil(FRAME_COUNT as u64)).max(1))
            .collect(),
        (true, None) => anyhow::bail!("nav capture needs discovered step count"),
        _ => Vec::new(),
    };
    let mut frames = Vec::new();
    let mut mesh_cache = MeshRenderCache::new();
    let props_root = repo_root.join("assets/props/polyhaven_warehouse");
    let mut trails = Trails::default();
    let mut tally = Tally {
        min_agv_gap_m: f64::INFINITY,
        min_pedestrian_gap_m: f64::INFINITY,
        truck_route_clearance_m: f64::INFINITY,
        ..Tally::default()
    };
    let mut command = VelocityCommand2d::ZERO;
    let (mut settled, mut docked, mut step) = (0, false, 0_u64);
    while settled < SETTLE_STEPS {
        anyhow::ensure!(step < MAX_STEPS, "nav AGV did not reach the desk");
        let observed = sim.observe();
        let pose = PlanarPose {
            x_m: observed.base_x_m,
            z_m: observed.base_z_m,
            heading_rad: -observed.base_yaw_rad,
        };
        corridor.advance(&mut sim, step as f64 * dt_s, dt_s, (pose, command));
        if step.is_multiple_of(PERCEIVE_EVERY_STEPS) {
            navigator.perceive(&sim, PERCEIVE_EVERY_STEPS as f64 * dt_s)?;
            tally.max_tracks = tally.max_tracks.max(navigator.tracker.len());
        }
        if step.is_multiple_of(REPLAN_EVERY_STEPS) {
            navigator.replan(Vec3::new(pose.x_m, pose.z_m, 0.0));
        }
        let plane_pose = Pose2d::new(pose.x_m, pose.z_m, pose.heading_rad);
        let follow =
            pure_pursuit_follow(&navigator.path, plane_pose, &pursuit).context("pure pursuit")?;
        docked |= follow.reached;
        command = if docked {
            // Pure pursuit stops on position only; once there, the AGV turns
            // in place to face the desk.
            VelocityCommand2d::new(0.0, (-2.0 * pose.heading_rad).clamp(-0.6, 0.6))
        } else {
            let desired = regulated(follow, &pursuit);
            let safe = avoid_velocities(
                plane_pose,
                AGV_RADIUS_M,
                &navigator.moving_obstacles(),
                desired,
                pursuit.max_linear_m_s,
                pursuit.max_angular_rad_s,
                &avoidance_config(),
            );
            if safe.linear_m_s < 0.5 * desired.linear_m_s {
                tally.yield_steps += 1;
            }
            safe
        };
        let observed = sim.step_action(wheel_action(command));
        step += 1;
        if docked && observed.base_yaw_rad.abs() < DOCKED_YAW_RAD {
            settled += 1;
        }
        let ego = (observed.base_x_m, observed.base_z_m, -observed.base_yaw_rad);
        measure(&mut tally, ego, &corridor, &navigator);
        trails.push(ego, &corridor);
        if sample_steps
            .get(frames.len())
            .is_some_and(|sample| step >= *sample)
        {
            let mut scene = office::render_office(
                sim.world(),
                (ego.0, ego.1, observed.base_yaw_rad),
                (
                    corridor.oncoming.pose.x_m,
                    corridor.oncoming.pose.z_m,
                    corridor.oncoming.pose.sim_yaw_rad(),
                ),
            );
            Corridor::hide_bodies(&mut scene);
            office::push_doorway(&mut scene, super::nav_world::DOORWAY_X_M);
            super::nav_world::push_near_floor(&mut scene);
            push_hand_truck(&mut scene);
            super::nav_world::push_totes(&mut scene, corridor.oncoming.pose);
            corridor.push_pedestrian(&mut scene)?;
            append_navigation_overlays(&mut scene, &navigator, &trails, ego);
            mesh_cache
                .resolve_scene(&mut scene, &[props_root.as_path()])
                .context("resolve nav props")?;
            frames.push(CaptureFrame {
                step,
                phase: if docked { "docked" } else { "driving" }.into(),
                scene,
            });
        }
    }
    let observed = sim.observe();
    let goal_error_m = (observed.base_x_m - GOAL_XZ_M.0).hypot(observed.base_z_m - GOAL_XZ_M.1);
    anyhow::ensure!(
        tally.min_agv_gap_m > 0.0,
        "the AGVs touched: gap {:.3} m",
        tally.min_agv_gap_m
    );
    anyhow::ensure!(
        tally.min_pedestrian_gap_m > 0.0,
        "the AGV touched the pedestrian: gap {:.3} m",
        tally.min_pedestrian_gap_m
    );
    anyhow::ensure!(
        !capture || frames.len() == FRAME_COUNT,
        "nav capture sampled {} of {} frames",
        frames.len(),
        FRAME_COUNT
    );
    let outcome = format!(
        "goal_error_m={goal_error_m:.3}; min_agv_footprint_gap_m={:.3}; min_pedestrian_gap_m={:.3}; min_hand_truck_gap_m={:.3}; ego_yield_s={:.2}; oncoming_yield_s={:.2}; max_tracks={}; replans={}; replay deterministic",
        tally.min_agv_gap_m,
        tally.min_pedestrian_gap_m,
        tally.truck_route_clearance_m,
        f64::from(tally.yield_steps) * dt_s,
        f64::from(corridor.oncoming.yielded_steps) * dt_s,
        tally.max_tracks,
        navigator.replans
    );
    Ok(Rollout {
        steps: step,
        initial_digest,
        final_digest: hash_physics_state(sim.world()),
        outcome,
        frames,
    })
}

/// Records the closest approaches this step.
fn measure(tally: &mut Tally, ego: (f64, f64, f64), corridor: &Corridor, _navigator: &Navigator) {
    let agv = corridor.oncoming.pose;
    tally.min_agv_gap_m = tally
        .min_agv_gap_m
        .min(footprint_gap_m(ego, (agv.x_m, agv.z_m, agv.heading_rad)));
    // Pedestrian: a 0.4 x 0.32 m body against the ego's bounding circle.
    let walker = &corridor.pedestrian;
    let to_walker = (ego.0 - walker.x_m).hypot(ego.1 - walker.z_m);
    tally.min_pedestrian_gap_m = tally
        .min_pedestrian_gap_m
        .min(to_walker - AGV_RADIUS_M - 0.26);
    let (truck_x, truck_z, _) = HAND_TRUCK;
    tally.truck_route_clearance_m = tally
        .truck_route_clearance_m
        .min((ego.0 - truck_x).hypot(ego.1 - truck_z) - AGV_RADIUS_M - 0.36);
}

/// Where each vehicle has been.
#[derive(Default)]
struct Trails {
    ego: Vec<(f64, f64)>,
    oncoming: Vec<(f64, f64)>,
    ticks: u64,
}

impl Trails {
    fn push(&mut self, ego: (f64, f64, f64), corridor: &Corridor) {
        self.ticks += 1;
        if self.ticks.is_multiple_of(6) {
            self.ego.push((ego.0, ego.1));
            self.oncoming
                .push((corridor.oncoming.pose.x_m, corridor.oncoming.pose.z_m));
        }
    }
}

/// Whether the static map explains a return at `(x, z)`: it lies on a wall or
/// the desk, give or take the scan's own error.
fn explained_by_map(x: f64, z: f64) -> bool {
    const TOLERANCE_M: f64 = 0.12;
    CORRIDOR_OBSTACLES.iter().any(|(min, max)| {
        x >= min.0 - TOLERANCE_M
            && x <= max.0 + TOLERANCE_M
            && z >= min.1 - TOLERANCE_M
            && z <= max.1 + TOLERANCE_M
    }) || z.abs() > 1.1
}

/// Marks every cell within `radius` (plus the footprint pad) of `(x, z)`.
fn mark_disk(grid: &mut OccupancyGrid, x: f64, z: f64, radius_m: f64) {
    let reach = radius_m + FOOTPRINT_PAD_M;
    let min = grid.world_to_grid(Vec3::new(x - reach, z - reach, 0.0));
    let max = grid.world_to_grid(Vec3::new(x + reach, z + reach, 0.0));
    let (Some(min), Some(max)) = (min, max) else {
        return;
    };
    for row in min.y..=max.y {
        for column in min.x..=max.x {
            let coord = GridCoord { x: column, y: row };
            let centre = grid.grid_to_world(coord);
            if (centre.x - x).hypot(centre.y - z) <= reach {
                for _ in 0..16 {
                    grid.mark_occupied(coord);
                }
            }
        }
    }
}

/// The corridor walls and the desk, taken from
/// `office_agv_delivery.rne.scene.toml` as `(min_x, min_z), (max_x, max_z)`.
///
/// The pickup dock is a 0.1 m platform the AGV drives over, so it is not an
/// obstacle.
const CORRIDOR_OBSTACLES: [((f64, f64), (f64, f64)); 3] = [
    ((-0.5, 1.1), (9.5, 1.2)),   // wall_north
    ((-0.5, -1.2), (9.5, -1.1)), // wall_south
    ((7.1, -0.7), (7.8, 0.7)),   // delivery_desk
];
/// Occupancy resolution for the corridor map, in meters.
const MAP_RESOLUTION_M: f64 = 0.05;
/// Obstacles grow by the AGV's half width plus a margin before they go on the
/// map, so a route that stays off lethal cells keeps the whole AGV clear. The
/// planner only refuses lethal cells; inflation is a cost it may pay.
const FOOTPRINT_PAD_M: f64 = 0.32;
/// Costmap inflation beyond the padded obstacles.
const MAP_INFLATION_M: f64 = 0.45;

/// Builds an occupancy grid of the corridor from the scene's own collision boxes.
fn corridor_grid() -> Result<OccupancyGrid> {
    let origin = Pose2d::new(-0.6, -1.4, 0.0);
    let width = ((10.8 / MAP_RESOLUTION_M).ceil()) as usize;
    let height = ((2.8 / MAP_RESOLUTION_M).ceil()) as usize;
    let mut grid = OccupancyGrid::new(width, height, MAP_RESOLUTION_M, origin)
        .context("corridor occupancy grid")?;
    for row in 0..height as isize {
        for column in 0..width as isize {
            let coord = GridCoord { x: column, y: row };
            for _ in 0..8 {
                grid.mark_free(coord);
            }
        }
    }
    for (min, max) in CORRIDOR_OBSTACLES {
        mark_rect(&mut grid, min, max);
    }
    Ok(grid)
}

/// Marks every cell whose centre lies in the `(x, z)` rectangle, padded by
/// [`FOOTPRINT_PAD_M`], as occupied.
fn mark_rect(grid: &mut OccupancyGrid, min: (f64, f64), max: (f64, f64)) {
    for row in 0..grid.height() as isize {
        for column in 0..grid.width() as isize {
            let coord = GridCoord { x: column, y: row };
            let centre = grid.grid_to_world(coord);
            let x_range = min.0 - FOOTPRINT_PAD_M..=max.0 + FOOTPRINT_PAD_M;
            let z_range = min.1 - FOOTPRINT_PAD_M..=max.1 + FOOTPRINT_PAD_M;
            if x_range.contains(&centre.x) && z_range.contains(&centre.y) {
                for _ in 0..16 {
                    grid.mark_occupied(coord);
                }
            }
        }
    }
}

/// Plans from `start` to the desk over `grid`.
fn plan_route(grid: &OccupancyGrid, start: Vec3) -> Result<(Costmap, Path2d)> {
    let costmap = Costmap::from_occupancy(
        grid,
        &CostmapConfig {
            inscribed_radius_m: 0.0,
            inflation_radius_m: MAP_INFLATION_M,
            ..CostmapConfig::default()
        },
    )
    .context("corridor costmap")?;
    let path = plan_path(
        &costmap,
        start,
        Vec3::new(GOAL_XZ_M.0, GOAL_XZ_M.1, 0.0),
        &GlobalPlannerConfig {
            cost_weight: 3.0,
            ..GlobalPlannerConfig::default()
        },
    )
    .context("corridor route")?;
    Ok((costmap, path))
}

/// Adds the costmap, the planned route, both vehicles' trails, the scan
/// returns the map does not explain, the tracks, a heading arrow, and the
/// docking goal to the office scene.
fn append_navigation_overlays(
    scene: &mut RenderScene,
    map: &Navigator,
    trails: &Trails,
    ego: (f64, f64, f64),
) {
    const ROUTE: [f32; 4] = [0.92, 0.25, 0.85, 1.0];
    const TRAIL: [f32; 4] = [0.10, 0.85, 0.95, 1.0];
    const ONCOMING_TRAIL: [f32; 4] = [0.35, 0.55, 1.0, 1.0];
    const GOAL: [f32; 4] = [0.15, 0.95, 0.45, 1.0];
    const ARROW: [f32; 4] = [1.0, 1.0, 1.0, 1.0];
    const INFLATION: [f32; 4] = [0.98, 0.72, 0.22, 0.55];
    const SCAN: [f32; 4] = [1.0, 0.18, 0.12, 1.0];
    const TRACK: [f32; 4] = [1.0, 0.85, 0.20, 1.0];

    // The costs the planner charged for, drawn where it charged them.
    let spec = GridMeshSpec {
        columns: map.grid.width(),
        rows: map.grid.height(),
        cell_size_m: map.grid.resolution_m(),
        origin_m: Vec3::new(map.grid.origin().x_m, 0.0, map.grid.origin().y_m),
        height_m: 0.012,
        fill: 0.9,
    };
    scene.items.push(RenderScene::item_from_dynamic_mesh(
        grid_mesh(&spec, |column, row| {
            let coord = GridCoord {
                x: column as isize,
                y: row as isize,
            };
            // Only inside the corridor: inflation drawn beyond the walls
            // describes space the robot cannot reach and reads as spill.
            let centre = map.grid.grid_to_world(coord);
            if centre.y.abs() > 1.1 {
                return false;
            }
            map.costmap
                .cost_at(coord)
                .is_some_and(|cost| cost > 0 && cost < COST_LETHAL)
        }),
        INFLATION,
    ));
    // Planned route: what `plan_path` returned, not a line drawn along the aisle.
    for waypoint in map.path.waypoints().iter().step_by(3) {
        push_sphere(
            scene,
            Vec3::new(waypoint.x_m, 0.07, waypoint.y_m),
            0.055,
            ROUTE,
        );
    }
    for (dx, dz) in [(-0.45, 0.0), (0.45, 0.0), (0.0, -0.45), (0.0, 0.45)] {
        push_sphere(
            scene,
            Vec3::new(GOAL_XZ_M.0 + dx, 0.09, GOAL_XZ_M.1 + dz),
            0.06,
            GOAL,
        );
    }
    for (trail, color) in [(&trails.ego, TRAIL), (&trails.oncoming, ONCOMING_TRAIL)] {
        for (x, z) in trail.iter().step_by(2) {
            push_sphere(scene, Vec3::new(*x, 0.03, *z), 0.035, color);
        }
    }
    // What the LiDAR saw that the map does not explain, where it saw it.
    for point in &map.unexplained {
        push_sphere(scene, *point, 0.035, SCAN);
    }
    // Tracks: a ring at the tracked radius and a marker 1 s ahead.
    for track in map.tracker.tracks() {
        let centre = Vec3::new(track.position_m.x, 0.05, track.position_m.y);
        for index in 0..16 {
            let angle = f64::from(index) * std::f64::consts::TAU / 16.0;
            push_sphere(
                scene,
                centre + Vec3::new(angle.cos(), 0.0, angle.sin()) * track.radius_m,
                0.022,
                TRACK,
            );
        }
        let velocity = track.velocity_m_s;
        if velocity.length() > MOVING_M_S {
            for step in 1..=4 {
                let ahead = f64::from(step) * 0.25;
                push_sphere(
                    scene,
                    centre + Vec3::new(velocity.x, 0.0, velocity.y) * ahead,
                    0.03,
                    TRACK,
                );
            }
        }
    }
    let forward = Vec3::new(ego.2.cos(), 0.0, ego.2.sin());
    push_box(
        scene,
        Vec3::new(ego.0, 0.42, ego.1) + forward * 0.5,
        Vec3::new(0.14, 0.05, 0.06),
        ARROW,
    );
}
