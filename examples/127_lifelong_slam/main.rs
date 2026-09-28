//! Lifelong SLAM: a robot maps the same warehouse on four days while pallets
//! move between them, and keeps one map that describes the warehouse as it is
//! now.
//!
//! Each day the robot starts somewhere else with no idea where it is. It runs
//! `Slam2d` in its own odometry frame on a physics `LiDAR`, then recognizes the
//! day's scans in the lifelong map (`discover_session_constraints`), merges the
//! day into the lifelong pose graph, rebuilds the map from every keyframe at
//! the optimized poses with recency weighting (`build_recency_map`), reports
//! which cells appeared and vanished, and prunes the nodes the new day has
//! superseded (`LifelongPoseGraph::prune_superseded`).
//!
//! Every number printed is measured against the warehouse's ground truth.
//!
//! ```text
//! cargo run --release -p lifelong_slam --example 127_lifelong_slam -- --smoke
//! ```

mod render;
mod warehouse;

use rne_core::SimDuration;
use rne_data::PointCloud;
use rne_math::{Hertz, Quat, Vec3};
use rne_nav::{GridCoord, LaserScan2d, OccupancyGrid, Pose2d};
use rne_physics::{PhysicsBackend, PhysicsWorldDesc};
use rne_physics_rapier::{step_physics, RapierBackend};
use rne_sensor::{sample_lidar, LidarSpec};
use rne_slam::{
    build_recency_map, discover_session_constraints, register_session_densely,
    DenseRegistrationConfig, DiscoveryConfig, LifelongPoseGraph, MapKeyframe, MergeOptions,
    PoseGraph, PruneConfig, RecencyMap, RecencyMapConfig, SessionChanges, SessionScan, Slam2d,
    SlamConfig,
};
use rne_world::Transform3;
use std::path::PathBuf;
use warehouse::{pallets, Footprint, DAYS, ROUTES, SLOTS, STATIC};

const SCAN_HZ: f64 = 10.0;
const DRIVE_M_S: f64 = 0.6;
const TURN_RAD_S: f64 = 0.8;
const LIDAR_HEIGHT_M: f64 = 0.5;
/// A keyframe is taken after this much odometry travel or turn.
const KEYFRAME_M: f64 = 0.3;
const KEYFRAME_RAD: f64 = 0.3;
/// Every how many keyframes a scan is offered for place recognition.
const RECOGNITION_STRIDE: usize = 12;
/// Every how many beams a recognition scan keeps.
const RECOGNITION_BEAM_STRIDE: usize = 3;
/// Map extent and resolution. Each day's own SLAM grid is centred on its
/// start, which can be anywhere in the warehouse, so it spans the building
/// twice over.
const RESOLUTION_M: f64 = 0.05;
const SESSION_GRID_M: f64 = 32.0;

/// One day's drive: the session SLAM produced and the truth behind it.
struct DayRun {
    graph: PoseGraph,
    /// Keyframe scans in the base frame, one per session node.
    scans: Vec<LaserScan2d>,
    /// True world pose at each session node.
    truth: Vec<Pose2d>,
    /// Final odometry error against truth, in meters.
    odometry_error_m: f64,
    loop_closures: usize,
}

fn main() {
    let smoke = std::env::args().any(|argument| argument == "--smoke");
    // The smoke runs the first two days, which exercise every stage; the full
    // run is the week.
    let week = run_week(if smoke { 2 } else { DAYS.len() });
    report(&week);
    check(&week);
    if smoke {
        println!("smoke ok");
        return;
    }
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
    let gif = root.join("docs/media/lifelong-slam.gif");
    render::render_week(
        &week.visuals,
        start_pose(0),
        &root.join("target/rne-lifelong-slam-frames"),
        &gif,
    )
    .expect("render the week");
    println!("wrote {}", gif.display());
}

/// What the week produced, day by day.
struct Week {
    days: Vec<DayReport>,
    visuals: Vec<render::DayVisual>,
}

struct DayReport {
    loop_closures: usize,
    odometry_error_m: f64,
    recognitions: usize,
    relocalization_error_m: f64,
    trajectory_error_m: f64,
    /// The whole lifelong graph against truth after rigid alignment.
    aligned_error_m: f64,
    nodes_before_prune: usize,
    nodes_after_prune: usize,
    changes: Option<ChangeScore>,
    ghost_cells: usize,
}

struct ChangeScore {
    changed_pallets: usize,
    detected_pallets: usize,
    flagged_cells: usize,
    flagged_on_changes: usize,
}

fn run_week(day_count: usize) -> Week {
    let template = OccupancyGrid::new(
        (SESSION_GRID_M / RESOLUTION_M) as usize,
        (SESSION_GRID_M / RESOLUTION_M) as usize,
        RESOLUTION_M,
        Pose2d::new(-0.5 * SESSION_GRID_M, -0.5 * SESSION_GRID_M, 0.0),
    )
    .expect("map template");
    let map_config = RecencyMapConfig::default();
    let mut lifelong: Option<LifelongPoseGraph> = None;
    let mut keyframes: Vec<MapKeyframe> = Vec::new();
    // Ground truth of each keyframe, kept parallel to `keyframes`.
    let mut keyframe_truth: Vec<Pose2d> = Vec::new();
    let mut map: Option<RecencyMap> = None;
    // The first day's map is the reference every later day is registered
    // against; its session is never pruned. Registering against the latest
    // map instead let the map frame random-walk by about 7 cm a day, each day
    // fitted to the previous day's copy of the building.
    let mut reference: Option<OccupancyGrid> = None;
    // The map frame is the first day's odometry frame, which starts at the
    // first day's true start pose.
    let world_from_map = start_pose(0);
    let mut days = Vec::new();
    let mut visuals = Vec::new();

    for day in 0..day_count {
        let run = drive_day(day, &template);
        let truth_map: Vec<Pose2d> = run
            .truth
            .iter()
            .map(|pose| world_from_map.inverse().compose(*pose))
            .collect();
        let (recognitions, relocalization_error_m) = match lifelong.as_mut() {
            None => {
                lifelong = Some(LifelongPoseGraph::from_first_session(run.graph.clone()));
                (0, 0.0)
            }
            Some(graph) => merge_day(
                graph,
                reference.as_ref().expect("reference map"),
                &run,
                &truth_map,
            ),
        };
        let graph = lifelong.as_mut().expect("lifelong graph");
        let offset = graph.graph().node_count() - run.graph.node_count();
        let new_frames: Vec<MapKeyframe> = run
            .scans
            .iter()
            .enumerate()
            .map(|(node, scan)| MapKeyframe {
                node: offset + node,
                scan: scan.clone(),
                sensor_from_base: Pose2d::IDENTITY,
            })
            .collect();
        keyframes.extend(new_frames);
        keyframe_truth.extend(truth_map.iter().copied());

        let trajectory_error_m = mean_distance(&graph.graph().nodes()[offset..], &truth_map);
        let node_estimates: Vec<Pose2d> = keyframes
            .iter()
            .map(|frame| graph.graph().nodes()[frame.node])
            .collect();
        let aligned_error_m = aligned_distance(&node_estimates, &keyframe_truth);
        let built = build_recency_map(&template, graph, &keyframes, &map_config).expect("map");
        let (appeared, vanished) = built
            .changes
            .last()
            .map_or((Vec::new(), Vec::new()), |last| {
                (last.appeared.clone(), last.vanished.clone())
            });
        let changes = (day > 0).then(|| {
            score_changes(
                built.changes.last().expect("session changes"),
                &built.grid,
                day,
                world_from_map,
            )
        });

        let nodes_before_prune = graph.graph().node_count();
        let pruned = graph.prune_superseded(PruneConfig {
            reference_sessions: 1,
            ..PruneConfig::default()
        });
        (keyframes, keyframe_truth) = keyframes
            .into_iter()
            .zip(keyframe_truth)
            .filter_map(|(keyframe, truth)| {
                pruned.index_map[keyframe.node]
                    .map(|node| (MapKeyframe { node, ..keyframe }, truth))
            })
            .unzip();
        let rebuilt = build_recency_map(&template, graph, &keyframes, &map_config).expect("map");
        let ghost_cells = ghost_cells(&rebuilt.grid, day, world_from_map);
        days.push(DayReport {
            loop_closures: run.loop_closures,
            odometry_error_m: run.odometry_error_m,
            recognitions,
            relocalization_error_m,
            trajectory_error_m,
            aligned_error_m,
            nodes_before_prune,
            nodes_after_prune: graph.graph().node_count(),
            changes,
            ghost_cells,
        });
        if reference.is_none() {
            reference = Some(rebuilt.grid.clone());
        }
        visuals.push(render::DayVisual {
            day,
            path: truth_path(day),
            keyframes: run.truth.iter().copied().zip(run.scans).collect(),
            prior: map.take().map(|previous| previous.grid),
            after: rebuilt.grid.clone(),
            appeared,
            vanished,
        });
        map = Some(rebuilt);
    }

    Week { days, visuals }
}

/// Recognizes a day in the lifelong map and merges it. Returns the number of
/// recognitions and their mean error against truth.
fn merge_day(
    lifelong: &mut LifelongPoseGraph,
    prior_map: &OccupancyGrid,
    run: &DayRun,
    truth_map: &[Pose2d],
) -> (usize, f64) {
    let scans: Vec<SessionScan> = run
        .scans
        .iter()
        .enumerate()
        .step_by(RECOGNITION_STRIDE)
        .map(|(node, scan)| SessionScan {
            node,
            points_sensor_m: scan_points(scan)
                .into_iter()
                .step_by(RECOGNITION_BEAM_STRIDE)
                .collect(),
            sensor_from_base: Pose2d::IDENTITY,
        })
        .collect();
    // The relocalizer searches its whole grid; the building covers a fraction
    // of the session-sized template, so search only what has been mapped.
    let prior_map = crop_to_known(prior_map, 1.0);
    let recognitions = discover_session_constraints(
        &prior_map,
        lifelong,
        &run.graph,
        &scans,
        &DiscoveryConfig::default(),
    )
    .expect("discovery");
    assert!(
        !recognitions.is_empty(),
        "the day was not recognized in the lifelong map"
    );
    // The global search places a few keyframes; the best of them seeds a
    // local registration of every keyframe of the day.
    let seed = recognitions
        .iter()
        .max_by(|a, b| a.score.total_cmp(&b.score))
        .expect("recognized");
    let every_scan: Vec<SessionScan> = run
        .scans
        .iter()
        .enumerate()
        .map(|(node, scan)| SessionScan {
            node,
            points_sensor_m: scan_points(scan),
            sensor_from_base: Pose2d::IDENTITY,
        })
        .collect();
    let registered = register_session_densely(
        &prior_map,
        lifelong,
        &run.graph,
        &every_scan,
        seed,
        &DenseRegistrationConfig {
            // A local match against the prior map is good to about 3 cm,
            // tighter than the day's own trajectory is over any distance.
            information_scale: (5000.0, 5000.0, 5000.0),
            ..DenseRegistrationConfig::default()
        },
    )
    .expect("dense registration");
    let error_m = registered
        .iter()
        .map(|recognition| {
            let truth = truth_map[recognition.constraint.session_node];
            (recognition.recognized_pose.x_m - truth.x_m)
                .hypot(recognition.recognized_pose.y_m - truth.y_m)
        })
        .sum::<f64>()
        / registered.len().max(1) as f64;
    let constraints: Vec<_> = registered
        .iter()
        .map(|recognition| recognition.constraint)
        .collect();
    lifelong
        .merge_session_onto_map(&run.graph, &constraints, MergeOptions::default())
        .expect("merge");
    (registered.len(), error_m)
}

/// The part of `grid` that holds any evidence, plus `margin_m` all round.
fn crop_to_known(grid: &OccupancyGrid, margin_m: f64) -> OccupancyGrid {
    let (width, height) = (grid.width(), grid.height());
    let (mut min_x, mut min_y, mut max_x, mut max_y) = (width, height, 0, 0);
    for (index, value) in grid.log_odds().iter().enumerate() {
        if *value != 0 {
            let (x, y) = (index % width, index / width);
            min_x = min_x.min(x);
            min_y = min_y.min(y);
            max_x = max_x.max(x);
            max_y = max_y.max(y);
        }
    }
    let margin = (margin_m / grid.resolution_m()).ceil() as usize;
    let (x0, y0) = (min_x.saturating_sub(margin), min_y.saturating_sub(margin));
    let (x1, y1) = (
        (max_x + margin).min(width - 1),
        (max_y + margin).min(height - 1),
    );
    let (w, h) = (x1 - x0 + 1, y1 - y0 + 1);
    let mut cells = Vec::with_capacity(w * h);
    for y in y0..=y1 {
        cells.extend_from_slice(&grid.log_odds()[y * width + x0..=y * width + x1]);
    }
    let corner = grid.grid_to_world(GridCoord {
        x: x0 as isize,
        y: y0 as isize,
    });
    let half = 0.5 * grid.resolution_m();
    OccupancyGrid::from_log_odds(
        w,
        h,
        grid.resolution_m(),
        Pose2d::new(corner.x - half, corner.y - half, 0.0),
        cells,
    )
    .expect("cropped grid")
}

fn start_pose(day: usize) -> Pose2d {
    let route = ROUTES[day];
    let (x, z) = route[0];
    let (next_x, next_z) = route[1];
    Pose2d::new(x, z, (next_z - z).atan2(next_x - x))
}

/// The true pose at every scan of a day: straight legs at driving speed and
/// turns in place at each corner.
fn truth_path(day: usize) -> Vec<Pose2d> {
    let dt = 1.0 / SCAN_HZ;
    let route = ROUTES[day];
    let mut pose = start_pose(day);
    let mut path = vec![pose];
    for window in route.windows(2) {
        let (x0, z0) = window[0];
        let (x1, z1) = window[1];
        let heading = (z1 - z0).atan2(x1 - x0);
        loop {
            let error = wrap(heading - pose.yaw_rad);
            if error.abs() < 1e-9 {
                break;
            }
            pose.yaw_rad += error.clamp(-TURN_RAD_S * dt, TURN_RAD_S * dt);
            path.push(pose);
        }
        pose.yaw_rad = heading;
        let length = (x1 - x0).hypot(z1 - z0);
        let steps = (length / (DRIVE_M_S * dt)).ceil() as usize;
        for step in 1..=steps {
            let t = step as f64 / steps as f64;
            pose.x_m = x0 + (x1 - x0) * t;
            pose.y_m = z0 + (z1 - z0) * t;
            path.push(pose);
        }
    }
    path
}

fn wrap(angle: f64) -> f64 {
    (angle + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU) - std::f64::consts::PI
}

/// Drives one day in a physics warehouse with that day's pallets, running
/// `Slam2d` on keyframes with drifting odometry.
fn drive_day(day: usize, template: &OccupancyGrid) -> DayRun {
    let mut world = rne_ecs::World::new();
    let mut backend = RapierBackend::new();
    let physics_world = backend
        .create_world(PhysicsWorldDesc::default())
        .expect("physics world");
    warehouse::spawn(&mut world, &STATIC, "structure");
    warehouse::spawn(&mut world, &pallets(day), "pallet");
    backend
        .sync_from_ecs(&mut world, physics_world)
        .expect("sync");
    step_physics(
        &mut backend,
        &mut world,
        physics_world,
        SimDuration::from_hertz(Hertz::new(SCAN_HZ)),
    )
    .expect("step");
    let spec = LidarSpec {
        ray_count: 360,
        min_angle_rad: -std::f64::consts::PI,
        max_angle_rad: std::f64::consts::PI,
        max_range_m: 15.0,
        height_offset_m: 0.0,
        ..LidarSpec::default()
    };
    let mut slam = Slam2d::new(
        template.clone(),
        SlamConfig {
            max_beams: 360,
            ..SlamConfig::default()
        },
    );
    let path = truth_path(day);
    // Odometry starts at the origin of the robot's own frame: it does not
    // know where in the warehouse it is.
    let mut odom = Pose2d::IDENTITY;
    let mut last_keyframe: Option<Pose2d> = None;
    let (mut scans, mut truth) = (Vec::new(), Vec::new());
    for (index, pose) in path.iter().enumerate() {
        if index > 0 {
            odom = odom.compose(drifting_increment(path[index - 1], *pose));
        }
        let due = last_keyframe.is_none_or(|last| {
            let delta = last.inverse().compose(odom);
            delta.x_m.hypot(delta.y_m) >= KEYFRAME_M || delta.yaw_rad.abs() >= KEYFRAME_RAD
        });
        if !due {
            continue;
        }
        let scan = scan_at(&backend, physics_world, *pose, &spec);
        slam.process(&scan, odom, Pose2d::IDENTITY)
            .expect("slam update");
        scans.push(scan);
        truth.push(*pose);
        last_keyframe = Some(odom);
    }
    let start = path[0];
    let end = *path.last().expect("path");
    let truth_end = start.inverse().compose(end);
    DayRun {
        graph: matched_session(slam.graph()),
        scans,
        truth,
        odometry_error_m: (odom.x_m - truth_end.x_m).hypot(odom.y_m - truth_end.y_m),
        loop_closures: slam.loop_edge_indices().len(),
    }
}

/// Information of an edge between consecutive scan-matched poses: a match is
/// good to a few centimetres and about a degree.
const MATCHED_INFORMATION: (f64, f64, f64) = (400.0, 400.0, 2000.0);

/// The session as `Slam2d` solved it, for merging.
///
/// `Slam2d`'s own graph links consecutive nodes by the *raw odometry* between
/// them; the scan matcher's correction lives only in the node estimates. Merged
/// as it is, a re-optimization pulls each day back toward its drifting
/// odometry between the few places it was recognized: the merged trajectory
/// error grew from 0.31 m to 0.66 m over three days. Here consecutive nodes are
/// linked by the matched relative pose instead, and `Slam2d`'s loop closures
/// are kept.
fn matched_session(solved: &PoseGraph) -> PoseGraph {
    let mut graph = PoseGraph::new();
    for pose in solved.nodes() {
        graph.add_node(*pose);
    }
    for (index, pair) in solved.nodes().windows(2).enumerate() {
        graph.add_edge(rne_slam::PoseGraphEdge {
            from: index,
            to: index + 1,
            measurement: pair[0].inverse().compose(pair[1]),
            information: MATCHED_INFORMATION,
            loop_closure: false,
        });
    }
    for edge in solved.edges().iter().filter(|edge| edge.loop_closure) {
        graph.add_edge(*edge);
    }
    graph
}

/// The odometry increment between two true poses, with a 2 % scale error and
/// a yaw bias: over a 30 m loop it drifts by more than a meter.
fn drifting_increment(from: Pose2d, to: Pose2d) -> Pose2d {
    let delta = from.inverse().compose(to);
    let travelled = delta.x_m.hypot(delta.y_m);
    Pose2d::new(
        delta.x_m * 1.02,
        delta.y_m * 1.02,
        delta.yaw_rad * 1.03 + 0.004 * travelled,
    )
}

fn base_transform(pose: Pose2d) -> Transform3 {
    // The SLAM plane is (x, z) with yaw turning x toward z, which is a
    // rotation of -yaw about +y.
    Transform3::from_translation_rotation(
        Vec3::new(pose.x_m, LIDAR_HEIGHT_M, pose.y_m),
        Quat::from_rotation_y(-pose.yaw_rad),
    )
}

fn scan_at(
    backend: &RapierBackend,
    physics_world: rne_physics::PhysicsWorldId,
    pose: Pose2d,
    spec: &LidarSpec,
) -> LaserScan2d {
    let base = base_transform(pose);
    let cloud = sample_lidar(backend, physics_world, &base, spec);
    cloud_to_scan(&cloud, &base, spec)
}

/// Bins a world-frame point cloud into a planar scan in the base frame.
fn cloud_to_scan(cloud: &PointCloud, base_world: &Transform3, spec: &LidarSpec) -> LaserScan2d {
    let count = spec.ray_count.max(1) as usize;
    let increment = (spec.max_angle_rad - spec.min_angle_rad) / count as f64;
    let mut ranges = vec![f64::INFINITY; count];
    let inverse = rne_math::Transform3 {
        translation: base_world.translation,
        rotation: base_world.rotation,
        scale: base_world.scale,
    }
    .inverse();
    for point in &cloud.points_m {
        let local = inverse.transform_point(*point);
        let range = local.x.hypot(local.z);
        if range < spec.min_range_m || range > spec.max_range_m {
            continue;
        }
        let angle = local.z.atan2(local.x);
        let index = (((angle - spec.min_angle_rad) / increment).round() as isize)
            .rem_euclid(count as isize) as usize;
        ranges[index] = ranges[index].min(range);
    }
    LaserScan2d {
        time_s: 0.0,
        frame: rne_nav::FrameId::new("lidar"),
        angle_min_rad: spec.min_angle_rad,
        angle_increment_rad: increment,
        range_min_m: spec.min_range_m,
        range_max_m: spec.max_range_m,
        ranges_m: ranges,
    }
}

fn scan_points(scan: &LaserScan2d) -> Vec<Vec3> {
    scan.ranges_m
        .iter()
        .enumerate()
        .filter(|(_, range)| range.is_finite() && **range <= scan.range_max_m)
        .map(|(beam, range)| {
            let angle = scan.angle_min_rad + scan.angle_increment_rad * beam as f64;
            Vec3::new(range * angle.cos(), range * angle.sin(), 0.0)
        })
        .collect()
}

/// Mean distance after the best rigid alignment of `estimates` onto `truth`
/// (2D Umeyama without scale): the error a map frame's own offset does not
/// explain.
fn aligned_distance(estimates: &[Pose2d], truth: &[Pose2d]) -> f64 {
    let n = estimates.len().min(truth.len()).max(1) as f64;
    let mean = |poses: &[Pose2d]| {
        (
            poses.iter().map(|p| p.x_m).sum::<f64>() / n,
            poses.iter().map(|p| p.y_m).sum::<f64>() / n,
        )
    };
    let (ex, ey) = mean(estimates);
    let (tx, ty) = mean(truth);
    let (mut cross, mut dot) = (0.0, 0.0);
    for (e, t) in estimates.iter().zip(truth) {
        let (ax, ay) = (e.x_m - ex, e.y_m - ey);
        let (bx, by) = (t.x_m - tx, t.y_m - ty);
        dot += ax * bx + ay * by;
        cross += ax * by - ay * bx;
    }
    let angle = cross.atan2(dot);
    let (sin, cos) = angle.sin_cos();
    estimates
        .iter()
        .zip(truth)
        .map(|(e, t)| {
            let (ax, ay) = (e.x_m - ex, e.y_m - ey);
            let x = tx + cos * ax - sin * ay;
            let y = ty + sin * ax + cos * ay;
            (x - t.x_m).hypot(y - t.y_m)
        })
        .sum::<f64>()
        / n
}

fn mean_distance(estimates: &[Pose2d], truth: &[Pose2d]) -> f64 {
    estimates
        .iter()
        .zip(truth)
        .map(|(a, b)| (a.x_m - b.x_m).hypot(a.y_m - b.y_m))
        .sum::<f64>()
        / estimates.len().max(1) as f64
}

/// The map cell's centre in world coordinates.
fn cell_world(grid: &OccupancyGrid, coord: GridCoord, world_from_map: Pose2d) -> (f64, f64) {
    let point = world_from_map.transform_point(grid.grid_to_world(coord));
    (point.x, point.y)
}

/// Pallets that arrived or left between the previous day and `day`.
fn changed_pallets(day: usize) -> Vec<Footprint> {
    let before: Vec<usize> = DAYS[day - 1].to_vec();
    let after: Vec<usize> = DAYS[day].to_vec();
    let mut changed: Vec<usize> = before
        .iter()
        .filter(|slot| !after.contains(slot))
        .chain(after.iter().filter(|slot| !before.contains(slot)))
        .copied()
        .collect();
    changed.sort_unstable();
    changed
        .into_iter()
        .map(|slot| {
            let (x, z) = SLOTS[slot];
            Footprint {
                x,
                z,
                half_x: warehouse::PALLET_HALF_M,
                half_z: warehouse::PALLET_HALF_M,
                height: warehouse::PALLET_HEIGHT_M,
            }
        })
        .collect()
}

/// Scores a day's reported changes against the pallets that really moved: a
/// changed pallet counts as detected when at least three flagged cells lie on
/// it, and a flagged cell counts as correct when it lies on a changed pallet.
fn score_changes(
    changes: &SessionChanges,
    grid: &OccupancyGrid,
    day: usize,
    world_from_map: Pose2d,
) -> ChangeScore {
    const MARGIN_M: f64 = 0.15;
    let changed = changed_pallets(day);
    let flagged: Vec<(f64, f64)> = changes
        .appeared
        .iter()
        .chain(&changes.vanished)
        .map(|coord| cell_world(grid, *coord, world_from_map))
        .collect();
    let detected_pallets = changed
        .iter()
        .filter(|pallet| {
            flagged
                .iter()
                .filter(|(x, z)| pallet.contains(*x, *z, MARGIN_M))
                .count()
                >= 3
        })
        .count();
    let flagged_on_changes = flagged
        .iter()
        .filter(|(x, z)| {
            changed
                .iter()
                .any(|pallet| pallet.contains(*x, *z, MARGIN_M))
        })
        .count();
    ChangeScore {
        changed_pallets: changed.len(),
        detected_pallets,
        flagged_cells: flagged.len(),
        flagged_on_changes,
    }
}

/// Occupied map cells on a slot that is empty on `day`: pallets the map still
/// remembers after they left.
fn ghost_cells(grid: &OccupancyGrid, day: usize, world_from_map: Pose2d) -> usize {
    let empty: Vec<Footprint> = (0..SLOTS.len())
        .filter(|slot| !DAYS[day].contains(slot))
        .map(|slot| {
            let (x, z) = SLOTS[slot];
            Footprint {
                x,
                z,
                half_x: warehouse::PALLET_HALF_M,
                half_z: warehouse::PALLET_HALF_M,
                height: warehouse::PALLET_HEIGHT_M,
            }
        })
        .collect();
    let mut count = 0;
    for row in 0..grid.height() {
        for column in 0..grid.width() {
            let coord = GridCoord {
                x: column as isize,
                y: row as isize,
            };
            if !grid.is_occupied(coord) {
                continue;
            }
            let (x, z) = cell_world(grid, coord, world_from_map);
            if empty.iter().any(|slot| slot.contains(x, z, 0.05)) {
                count += 1;
            }
        }
    }
    count
}

/// The measured properties the example stands behind.
fn check(week: &Week) {
    for (day, report) in week.days.iter().enumerate() {
        // Held to the first day's frame: registering every day against the
        // reference map keeps the frame where day 0 put it (0.15 m from truth
        // on every day measured).
        assert!(
            report.trajectory_error_m < 0.25,
            "day {day} drifted from the map frame: {:.3} m",
            report.trajectory_error_m
        );
        // Geometrically consistent: 0.025 to 0.032 m after rigid alignment.
        assert!(
            report.aligned_error_m < 0.05,
            "day {day} map is inconsistent: {:.3} m",
            report.aligned_error_m
        );
        assert_eq!(
            report.ghost_cells, 0,
            "day {day} remembers a pallet that left"
        );
        if let Some(score) = &report.changes {
            assert_eq!(
                score.detected_pallets, score.changed_pallets,
                "day {day} missed a pallet that moved"
            );
            // Measured 95 to 100 percent.
            assert!(
                score.flagged_on_changes * 10 >= score.flagged_cells * 9,
                "day {day} flagged {} cells, only {} on pallets that moved",
                score.flagged_cells,
                score.flagged_on_changes
            );
        }
    }
    // Pruning holds the graph at the reference day plus the latest day.
    let first = week.days[0].nodes_after_prune;
    for report in &week.days {
        assert!(
            report.nodes_after_prune <= 2 * first + 10,
            "the graph kept growing"
        );
    }
}

fn report(week: &Week) {
    println!("day  loops  odom_err  regist  reg_err  traj_err  aligned  nodes(pre->post)  changes(detected/changed, on-change/flagged)  ghost_cells");
    for (day, report) in week.days.iter().enumerate() {
        let changes = report.changes.as_ref().map_or("-".to_string(), |score| {
            format!(
                "{}/{} pallets, {}/{} cells",
                score.detected_pallets,
                score.changed_pallets,
                score.flagged_on_changes,
                score.flagged_cells
            )
        });
        println!(
            "{day:>3}  {:>5}  {:>7.2}m  {:>5}  {:>8.3}m  {:>7.3}m  {:>6.3}m  {:>5} -> {:<5}  {changes:<40}  {:>5}",
            report.loop_closures,
            report.odometry_error_m,
            report.recognitions,
            report.relocalization_error_m,
            report.trajectory_error_m,
            report.aligned_error_m,
            report.nodes_before_prune,
            report.nodes_after_prune,
            report.ghost_cells,
        );
    }
}
