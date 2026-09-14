//! Deterministic online 2D SLAM: scan matching corrects drifting odometry while
//! building an occupancy map, then a revisit closes a loop and the pose graph
//! smooths the trajectory. Driven through the `rne_slam` ECS resources.
//!
//! Run with `cargo run -p nav_slam_mapping --example 97_nav_slam_mapping`.

use rne_nav::{FrameId, GridCoord, LaserScan2d, OccupancyGrid, Pose2d};
use rne_slam::{slam_step, PendingSlamScans, Slam2d, SlamConfig, SlamState};
use std::f64::consts::TAU;

const BEAMS: usize = 360;

fn main() {
    let mut world = rne_ecs::World::new();
    let grid = OccupancyGrid::new(240, 160, 0.05, Pose2d::new(-6.0, -4.0, 0.0)).unwrap();
    world.insert_resource(SlamState::new(Slam2d::new(grid, SlamConfig::default())));
    world.insert_resource(PendingSlamScans::new());

    // Loop test files: outbound then back along the same corridor, with
    // odometry yaw drift the SLAM front-end must reject.
    let mut true_pose = Pose2d::new(-3.0, 0.0, 0.0);
    let mut odom_pose = true_pose;
    let mut last_slam_pose = true_pose;
    let mut loop_closures = 0;
    let step = 0.25;

    for _ in 0..16 {
        let scan = room_scan(true_pose.x_m, true_pose.y_m, 0.0);
        {
            let mut pending = world.resource_mut::<PendingSlamScans>();
            pending.push(scan, odom_pose, Pose2d::IDENTITY);
        }
        let report = slam_step(&mut world).expect("slam step");
        last_slam_pose = report.last_pose;
        true_pose.x_m += step;
        odom_pose.x_m += step;
        odom_pose.yaw_rad += 0.01;
    }
    // Reverse back toward the start.
    for _ in 0..16 {
        true_pose.x_m -= step;
        odom_pose.x_m -= step;
        odom_pose.yaw_rad += 0.01;
        let scan = room_scan(true_pose.x_m, true_pose.y_m, 0.0);
        {
            let mut pending = world.resource_mut::<PendingSlamScans>();
            pending.push(scan, odom_pose, Pose2d::IDENTITY);
        }
        let report = slam_step(&mut world).expect("slam step");
        last_slam_pose = report.last_pose;
        loop_closures += report.matched;
    }

    let odom_error = pose_error(odom_pose, true_pose);
    let slam_error = pose_error(last_slam_pose, true_pose);
    println!(
        "truth final : ({:.2}, {:.2}, {:.3})",
        true_pose.x_m, true_pose.y_m, true_pose.yaw_rad
    );
    println!(
        "odom  final : ({:.2}, {:.2}, {:.3})  error = {:.2} m",
        odom_pose.x_m, odom_pose.y_m, odom_pose.yaw_rad, odom_error
    );
    println!(
        "slam  final : ({:.2}, {:.2}, {:.3})  error = {:.2} m",
        last_slam_pose.x_m, last_slam_pose.y_m, last_slam_pose.yaw_rad, slam_error
    );

    let estimator = &world.resource::<SlamState>().estimator;
    println!(
        "scans processed = {}, matched = {}, odometry edges = {}, loop closures = {}",
        estimator.scans_processed(),
        loop_closures,
        estimator.graph().edge_count() - estimator.loop_edge_indices().len(),
        estimator.loop_edge_indices().len()
    );
    println!("map occupied cells = {}", count_occupied(estimator.grid()));

    println!("\nSLAM occupancy map ('#' occupied, '.' free, ' ' unknown):");
    print!("{}", render(estimator.grid(), 72, 24));
}

fn pose_error(pose: Pose2d, truth: Pose2d) -> f64 {
    ((pose.x_m - truth.x_m).powi(2) + (pose.y_m - truth.y_m).powi(2)).sqrt()
}

fn count_occupied(grid: &OccupancyGrid) -> usize {
    (0..grid.height())
        .flat_map(|y| (0..grid.width()).map(move |x| (x, y)))
        .filter(|(x, y)| {
            grid.is_occupied(GridCoord {
                x: *x as isize,
                y: *y as isize,
            })
        })
        .count()
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

/// Asymmetric pillars make position observable so the revisit closes a loop.
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

fn render(grid: &OccupancyGrid, columns: usize, rows: usize) -> String {
    let mut output = String::new();
    for row in (0..rows).rev() {
        for column in 0..columns {
            let x0 = column * grid.width() / columns;
            let x1 = ((column + 1) * grid.width() / columns).max(x0 + 1);
            let y0 = row * grid.height() / rows;
            let y1 = ((row + 1) * grid.height() / rows).max(y0 + 1);
            let mut occupied = false;
            let mut known = false;
            for y in y0..y1 {
                for x in x0..x1 {
                    let coord = GridCoord {
                        x: x as isize,
                        y: y as isize,
                    };
                    occupied |= grid.is_occupied(coord);
                    known |= grid.is_known(coord);
                }
            }
            output.push(if occupied {
                '#'
            } else if known {
                '.'
            } else {
                ' '
            });
        }
        output.push('\n');
    }
    output
}
