//! Office AGV navigation showcase: the AGV follows the route `plan_path`
//! returns with pure pursuit, and replans around a second AGV coming the other
//! way down the corridor.

use super::media::{
    capture_frames, push_box, push_sphere, CameraEvidence, CaptureFrame, ShowcaseMetadata,
    SimulationEvidence, FRAME_COUNT,
};
use super::office;
use anyhow::{Context, Result};
use rne_ai::{office_agv_delivery_scene_path, DiffDriveAction, DiffDriveSim};
use rne_math::Vec3;
use rne_nav::{
    plan_path, pure_pursuit_follow, Costmap, CostmapConfig, FollowResult, GlobalPlannerConfig,
    GridCoord, OccupancyGrid, Path2d, Pose2d, PurePursuitConfig, VelocityCommand2d, COST_LETHAL,
};
use rne_physics::hash_physics_state;
use rne_render::{grid_mesh, GridMeshSpec, RenderScene};
use rne_render_wgpu::CameraOrbit;
use serde_json::to_vec_pretty;
use std::fs;
use std::path::Path;

const ENVIRONMENT_ID: &str = "nav";
const SUBJECT: &str =
    "office AGV navigation: pure pursuit on a replanned route past an oncoming AGV";
const CAMERA: CameraEvidence = CameraEvidence {
    fov_y_rad: std::f64::consts::FRAC_PI_4,
    yaw_rad: 0.32,
    pitch_rad: 0.78,
    distance_m: 6.4,
};

/// Drives the office AGV to the desk along replanned routes and captures it
/// with navigation overlays.
pub fn run(repo_root: &Path, capture: bool) -> Result<ShowcaseMetadata> {
    let first = rollout(false, None)?;
    let replay = rollout(false, Some(first.steps))?;
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
        let captured = rollout(true, Some(first.steps))?;
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
        visual_state_sync: "The orange AGV is the physics-driven diff-drive robot of the office scene, steered by wheel speeds from pure pursuit. The blue AGV is a kinematic proxy that is not in the physics world; its footprint is written into the costmap the route is planned on. Route, costmap, trail, and goal are the planner's own state drawn as overlays.",
        simulation: evidence,
        capture: capture_evidence,
        camera: CAMERA,
        provenance: vec![
            "assets/scenes/office_agv_delivery.rne.scene.toml",
            "crates/rne_nav/src/planner.rs",
            "crates/rne_nav/src/control.rs",
            "examples/90_showcase_captures/nav.rs",
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
/// Planar half extents of either AGV (x along its heading, z across it).
const AGV_HALF_M: (f64, f64) = (0.25, 0.2);
/// The oncoming AGV keeps to its own side of the corridor centre line, which
/// still leaves half of it in the ego's straight-line path.
const ONCOMING_LANE_Z_M: f64 = -0.2;
const ONCOMING_START_X_M: f64 = 5.8;
const ONCOMING_END_X_M: f64 = -1.8;
const ONCOMING_SPEED_M_S: f64 = 0.45;
const ONCOMING_DEPARTURE_S: f64 = 3.0;
/// How far ahead the oncoming AGV's footprint is swept into the costmap.
const ONCOMING_PREDICTION_S: f64 = 2.0;
/// Replan rate, as a step interval.
const REPLAN_EVERY_STEPS: u64 = 12;
/// Heading error at which the docked AGV counts as facing the desk.
const DOCKED_YAW_RAD: f64 = 0.02;
/// Steps held stopped at the goal before the episode ends.
const SETTLE_STEPS: u64 = 30;
const MAX_STEPS: u64 = 3_000;

/// The static corridor and the latest route planned across it.
struct Navigator {
    corridor: OccupancyGrid,
    grid: OccupancyGrid,
    costmap: Costmap,
    path: Path2d,
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
            replans: 0,
        })
    }

    /// Writes the oncoming AGV's footprint, swept over the prediction horizon,
    /// into a copy of the corridor map and plans again from `start`. A failed
    /// plan keeps the previous route.
    fn replan(&mut self, start: Vec3, oncoming_x_m: f64, oncoming_moving: bool) -> Result<()> {
        let mut grid = self.corridor.clone();
        let sweep_m = if oncoming_moving {
            ONCOMING_SPEED_M_S * ONCOMING_PREDICTION_S
        } else {
            0.0
        };
        mark_rect(
            &mut grid,
            (
                oncoming_x_m - AGV_HALF_M.0 - sweep_m,
                ONCOMING_LANE_Z_M - AGV_HALF_M.1,
            ),
            (
                oncoming_x_m + AGV_HALF_M.0,
                ONCOMING_LANE_Z_M + AGV_HALF_M.1,
            ),
        );
        if let Ok((costmap, path)) = plan_route(&grid, start) {
            self.costmap = costmap;
            self.path = path;
        }
        self.grid = grid;
        self.replans += 1;
        Ok(())
    }
}

fn oncoming_x_m(time_s: f64) -> (f64, bool) {
    let travelled_m = ((time_s - ONCOMING_DEPARTURE_S) * ONCOMING_SPEED_M_S).max(0.0);
    let x_m = (ONCOMING_START_X_M - travelled_m).max(ONCOMING_END_X_M);
    (x_m, x_m > ONCOMING_END_X_M && time_s > ONCOMING_DEPARTURE_S)
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

/// Gap between two axis-aligned AGV footprints; zero or less means contact.
/// The ego's footprint is rotated to its heading and bounded, so this errs on
/// the small side.
fn footprint_gap_m(ego: (f64, f64, f64), other: (f64, f64)) -> f64 {
    let (sin, cos) = ego.2.sin_cos();
    let ego_half_x = AGV_HALF_M.0 * cos.abs() + AGV_HALF_M.1 * sin.abs();
    let ego_half_z = AGV_HALF_M.0 * sin.abs() + AGV_HALF_M.1 * cos.abs();
    let gap_x = (ego.0 - other.0).abs() - ego_half_x - AGV_HALF_M.0;
    let gap_z = (ego.1 - other.1).abs() - ego_half_z - AGV_HALF_M.1;
    if gap_x > 0.0 && gap_z > 0.0 {
        gap_x.hypot(gap_z)
    } else {
        gap_x.max(gap_z)
    }
}

fn rollout(capture: bool, expected_steps: Option<u64>) -> Result<Rollout> {
    let mut sim =
        DiffDriveSim::from_scene_path(&office_agv_delivery_scene_path()).context("office scene")?;
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
    let mut frames = Vec::new();
    let mut trajectory: Vec<(f64, f64)> = Vec::new();
    let sample_steps: Vec<u64> = match (capture, expected_steps) {
        (true, Some(total)) => (1..=FRAME_COUNT)
            .map(|index| ((index as u64 * total).div_ceil(FRAME_COUNT as u64)).max(1))
            .collect(),
        (true, None) => anyhow::bail!("nav capture needs discovered step count"),
        _ => Vec::new(),
    };
    let mut sample_index = 0;
    let mut min_gap_m = f64::INFINITY;
    let mut max_offset_m: f64 = 0.0;
    let mut max_tracking_error_m: f64 = 0.0;
    let mut settled = 0;
    let mut docked = false;
    let mut step = 0;
    while settled < SETTLE_STEPS {
        anyhow::ensure!(step < MAX_STEPS, "nav AGV did not reach the desk");
        let pose = sim.observe();
        let time_s = step as f64 * dt_s;
        let (other_x_m, other_moving) = oncoming_x_m(time_s);
        if step % REPLAN_EVERY_STEPS == 0 {
            navigator.replan(
                Vec3::new(pose.base_x_m, pose.base_z_m, 0.0),
                other_x_m,
                other_moving,
            )?;
        }
        let follow = pure_pursuit_follow(
            &navigator.path,
            Pose2d::new(pose.base_x_m, pose.base_z_m, -pose.base_yaw_rad),
            &pursuit,
        )
        .context("pure pursuit")?;
        docked |= follow.reached;
        // Pure pursuit stops on position only; once there, the AGV turns in
        // place to face the desk.
        let command = if docked {
            VelocityCommand2d::new(0.0, (2.0 * pose.base_yaw_rad).clamp(-0.6, 0.6))
        } else {
            regulated(follow, &pursuit)
        };
        // The planner works in the (x, z) plane, where the AGV's heading is
        // `(cos yaw, -sin yaw)`: its heading angle there is `-yaw`, so a
        // positive turn in the plane is a negative yaw rate.
        let half_track = -command.angular_rad_s * TRACK_WIDTH_M / 2.0;
        let action = DiffDriveAction {
            left_velocity_rad_s: ((command.linear_m_s - half_track) / WHEEL_RADIUS_M)
                .clamp(-MAX_WHEEL_RAD_S, MAX_WHEEL_RAD_S),
            right_velocity_rad_s: ((command.linear_m_s + half_track) / WHEEL_RADIUS_M)
                .clamp(-MAX_WHEEL_RAD_S, MAX_WHEEL_RAD_S),
        };
        let pose = sim.step_action(action);
        step += 1;
        if docked && pose.base_yaw_rad.abs() < DOCKED_YAW_RAD {
            settled += 1;
        }
        let ego = (pose.base_x_m, pose.base_z_m, pose.base_yaw_rad);
        min_gap_m = min_gap_m.min(footprint_gap_m(ego, (other_x_m, ONCOMING_LANE_Z_M)));
        max_offset_m = max_offset_m.max(pose.base_z_m.abs());
        max_tracking_error_m = max_tracking_error_m.max(follow.closest.distance_m);
        trajectory.push((pose.base_x_m, pose.base_z_m));
        if sample_index < sample_steps.len() && step >= sample_steps[sample_index] {
            let mut scene = office::render_office(
                sim.world(),
                ego,
                (other_x_m, ONCOMING_LANE_Z_M, std::f64::consts::PI),
            );
            append_navigation_overlays(&mut scene, &navigator, &trajectory, pose.base_yaw_rad);
            frames.push(CaptureFrame {
                step,
                phase: if docked {
                    "docked".into()
                } else {
                    "following".into()
                },
                scene,
            });
            sample_index += 1;
        }
    }
    let pose = sim.observe();
    let goal_error_m = (pose.base_x_m - GOAL_XZ_M.0).hypot(pose.base_z_m - GOAL_XZ_M.1);
    anyhow::ensure!(
        min_gap_m > 0.0,
        "nav AGV touched the oncoming AGV: gap {min_gap_m:.3} m"
    );
    anyhow::ensure!(
        !capture || frames.len() == FRAME_COUNT,
        "nav capture sampled {} of {} frames",
        frames.len(),
        FRAME_COUNT
    );
    let outcome = format!(
        "goal_error_m={goal_error_m:.3}; min_footprint_gap_m={min_gap_m:.3}; max_lateral_offset_m={max_offset_m:.3}; max_tracking_error_m={max_tracking_error_m:.3}; replans={}; replay deterministic",
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

/// Adds the map, the planned route, the driven trajectory, a heading arrow, and
/// the docking goal to the office scene.
fn append_navigation_overlays(
    scene: &mut RenderScene,
    map: &Navigator,
    trajectory: &[(f64, f64)],
    base_yaw_rad: f64,
) {
    const ROUTE: [f32; 4] = [0.92, 0.25, 0.85, 1.0];
    const TRAIL: [f32; 4] = [0.10, 0.85, 0.95, 1.0];
    const GOAL: [f32; 4] = [0.15, 0.95, 0.45, 1.0];
    const ARROW: [f32; 4] = [1.0, 1.0, 1.0, 1.0];
    const INFLATION: [f32; 4] = [0.98, 0.72, 0.22, 0.55];

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
            let occupied = map
                .grid
                .probability(coord)
                .is_some_and(|probability| probability >= 0.6);
            !occupied
                && map
                    .costmap
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
            0.065,
            ROUTE,
        );
    }
    // Docking goal ring at the desk.
    for (dx, dz) in [(-0.45, 0.0), (0.45, 0.0), (0.0, -0.45), (0.0, 0.45)] {
        push_sphere(scene, Vec3::new(6.5 + dx, 0.09, dz), 0.06, GOAL);
    }
    // Driven trajectory history.
    for (index, (x, z)) in trajectory.iter().enumerate() {
        if index % 6 == 0 {
            push_sphere(scene, Vec3::new(*x, 0.34, *z), 0.05, TRAIL);
        }
    }
    // Heading arrow in front of the AGV.
    if let Some((x, z)) = trajectory.last() {
        let forward = Vec3::new(base_yaw_rad.cos(), 0.0, -base_yaw_rad.sin());
        let tip = Vec3::new(*x, 0.34, *z) + forward * 0.55;
        push_box(scene, tip, Vec3::new(0.18, 0.06, 0.08), ARROW);
    }
}
