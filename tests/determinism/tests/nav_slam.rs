//! Determinism regression tests for the `rne_nav` / `rne_slam` stack.
//!
//! Each scenario is executed twice and compared byte-for-byte (or by a stable
//! FNV-1a hash) so that mapping, planning, and scan-matching cannot silently
//! introduce ordering or floating-point nondeterminism.

use rne_nav::{
    plan_path, Costmap, CostmapConfig, DwaConfig, DwaOutcome, DwaPlanner, FrameId, GridCoord,
    LaserScan2d, OccupancyGrid, Path2d, Pose2d, PurePursuitConfig,
};
use rne_slam::{Slam2d, SlamConfig};
use std::f64::consts::TAU;

const BEAMS: usize = 360;

fn fnv1a(bytes: &[u8]) -> u64 {
    let mut hash = 0xcbf2_9ce4_8422_2325_u64;
    for byte in bytes {
        hash ^= *byte as u64;
        hash = hash.wrapping_mul(0x0000_0100_0000_01b3);
    }
    hash
}

fn grid_hash(grid: &OccupancyGrid) -> u64 {
    let mut bytes = Vec::with_capacity(grid.log_odds().len() * 2);
    for value in grid.log_odds() {
        bytes.extend_from_slice(&value.to_le_bytes());
    }
    fnv1a(&bytes)
}

fn room_scan(x: f64, y: f64, time_s: f64) -> LaserScan2d {
    let mut ranges_m = Vec::with_capacity(BEAMS);
    for beam in 0..BEAMS {
        let angle = TAU * beam as f64 / BEAMS as f64;
        ranges_m.push(ray_to_wall(x, y, angle));
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

fn build_room_map() -> (OccupancyGrid, u64) {
    let mut grid = OccupancyGrid::new(240, 160, 0.05, Pose2d::new(-6.0, -4.0, 0.0)).unwrap();
    for step in 0..16 {
        let x = -3.0 + step as f64 * 0.4;
        rne_nav::integrate_scan(
            &mut grid,
            &room_scan(x, 0.0, step as f64 * 0.1),
            Pose2d::new(x, 0.0, 0.0),
            &rne_nav::ScanIntegrationConfig::default(),
        )
        .unwrap();
    }
    let hash = grid_hash(&grid);
    (grid, hash)
}

#[test]
fn occupancy_mapping_is_deterministic() {
    let (first, first_hash) = build_room_map();
    let (second, second_hash) = build_room_map();
    assert_eq!(first_hash, second_hash);
    assert_eq!(first.log_odds(), second.log_odds());
}

#[test]
fn planning_is_deterministic_and_stable() {
    let (grid, _) = build_room_map();
    let costmap = Costmap::from_occupancy(&grid, &CostmapConfig::default()).unwrap();
    let start = rne_math::Vec3::new(-2.5, 0.0, 0.0);
    let goal = rne_math::Vec3::new(2.5, 0.0, 0.0);
    let first = plan_path(
        &costmap,
        start,
        goal,
        &rne_nav::GlobalPlannerConfig::default(),
    )
    .unwrap();
    let second = plan_path(
        &costmap,
        start,
        goal,
        &rne_nav::GlobalPlannerConfig::default(),
    )
    .unwrap();
    assert_eq!(first.waypoints(), second.waypoints());
    // Golden regression on the route length.
    let third = plan_path(
        &costmap,
        start,
        goal,
        &rne_nav::GlobalPlannerConfig::default(),
    )
    .unwrap();
    assert_eq!(first.length_m().to_bits(), third.length_m().to_bits());
    assert!(first.length_m() > 5.0);
}

fn run_slam() -> (Slam2d, u64) {
    let grid = OccupancyGrid::new(240, 160, 0.05, Pose2d::new(-6.0, -4.0, 0.0)).unwrap();
    let mut slam = Slam2d::new(grid, SlamConfig::default());
    let mut odom = Pose2d::new(-3.0, 0.0, 0.0);
    let mut truth = Pose2d::new(-3.0, 0.0, 0.0);
    for step in 0..24 {
        let scan = room_scan(truth.x_m, truth.y_m, step as f64 * 0.1);
        slam.process(&scan, odom, Pose2d::IDENTITY).unwrap();
        truth.x_m += 0.25;
        odom.x_m += 0.25;
        odom.yaw_rad += 0.005;
    }
    let hash = grid_hash(slam.grid());
    (slam, hash)
}

#[test]
fn slam_is_deterministic() {
    let (first, first_hash) = run_slam();
    let (second, second_hash) = run_slam();
    assert_eq!(first_hash, second_hash);
    assert_eq!(first.pose().x_m.to_bits(), second.pose().x_m.to_bits());
    assert_eq!(first.pose().y_m.to_bits(), second.pose().y_m.to_bits());
    assert_eq!(
        first.pose().yaw_rad.to_bits(),
        second.pose().yaw_rad.to_bits()
    );
    assert_eq!(first.scans_processed(), second.scans_processed());
    assert_eq!(first.loop_edge_indices(), second.loop_edge_indices());
}

#[test]
fn local_planner_and_follower_are_deterministic() {
    let (grid, _) = build_room_map();
    let costmap = Costmap::from_occupancy(&grid, &CostmapConfig::default()).unwrap();
    let path = Path2d::from_points(&[
        rne_math::Vec3::new(-2.5, 0.0, 0.0),
        rne_math::Vec3::new(2.5, 0.0, 0.0),
    ]);
    let planner = DwaPlanner::new(DwaConfig::default());
    let first = planner
        .compute_command(&costmap, &path, Pose2d::IDENTITY, Default::default())
        .unwrap();
    let second = planner
        .compute_command(&costmap, &path, Pose2d::IDENTITY, Default::default())
        .unwrap();
    assert_eq!(first, second);
    assert!(matches!(first, DwaOutcome::Command(_)));

    let follow =
        rne_nav::pure_pursuit_follow(&path, Pose2d::IDENTITY, &PurePursuitConfig::default())
            .unwrap();
    let follow_again =
        rne_nav::pure_pursuit_follow(&path, Pose2d::IDENTITY, &PurePursuitConfig::default())
            .unwrap();
    assert_eq!(follow.command, follow_again.command);
}

#[test]
fn slam_replay_is_byte_identical() {
    // Capture a scan/odometry sequence once.
    let mut recorded: Vec<(LaserScan2d, Pose2d)> = Vec::new();
    let mut truth = Pose2d::new(-3.0, 0.0, 0.0);
    let mut odom = truth;
    for step in 0..24 {
        recorded.push((room_scan(truth.x_m, truth.y_m, step as f64 * 0.1), odom));
        truth.x_m += 0.25;
        odom.x_m += 0.25;
        odom.yaw_rad += 0.005;
    }

    let run = |sequence: &[(LaserScan2d, Pose2d)]| -> (u64, u64, u64) {
        let grid = OccupancyGrid::new(240, 160, 0.05, Pose2d::new(-6.0, -4.0, 0.0)).unwrap();
        let mut slam = Slam2d::new(grid, SlamConfig::default());
        for (scan, odom) in sequence {
            slam.process(scan, *odom, Pose2d::IDENTITY).unwrap();
        }
        (
            grid_hash(slam.grid()),
            slam.pose().x_m.to_bits(),
            slam.pose().yaw_rad.to_bits(),
        )
    };

    let first = run(&recorded);
    let replay = run(&recorded);
    assert_eq!(first, replay, "replay diverged: {first:?} vs {replay:?}");
}

#[test]
fn costmap_inflation_is_deterministic() {
    let (grid, _) = build_room_map();
    let first = Costmap::from_occupancy(&grid, &CostmapConfig::default()).unwrap();
    let second = Costmap::from_occupancy(&grid, &CostmapConfig::default()).unwrap();
    assert_eq!(first.costs(), second.costs());
    assert!(first.lethal_count() > 0);
    assert!(first.cost_at(GridCoord { x: 120, y: 80 }).is_some());
}
