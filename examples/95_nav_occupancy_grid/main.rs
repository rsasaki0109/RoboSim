//! Deterministic occupancy mapping, costmap inflation, and transform-tree
//! lookup using the `rne_nav` foundation — the ROS-free base for a Nav2/SLAM
//! stack.
//!
//! Run with `cargo run -p nav_occupancy_grid --example 95_nav_occupancy_grid`.

use rne_math::{Transform3, Vec3};
use rne_nav::{
    integrate_pending_scans, CostmapConfig, FrameId, LaserScan2d, NavMap, OccupancyGrid,
    PendingScans, Pose2d, ScanIntegrationConfig, StampedTransform, TfBuffer,
};
use std::f64::consts::TAU;

const ROOM_HALF_X_M: f64 = 5.0;
const ROOM_HALF_Y_M: f64 = 3.0;
const BEAM_COUNT: usize = 360;

fn main() {
    let resolution_m = 0.05;
    let grid = OccupancyGrid::new(
        (12.0 / resolution_m) as usize,
        (8.0 / resolution_m) as usize,
        resolution_m,
        Pose2d::new(-6.0, -4.0, 0.0),
    )
    .expect("grid");
    let mut map = NavMap::new(
        grid,
        CostmapConfig::default(),
        ScanIntegrationConfig::default(),
    )
    .expect("nav map");

    // Drive through the room and integrate one synthetic 360° scan per pose.
    let poses = [-3.0, -1.5, 0.0, 1.5, 3.0];
    for (step, x) in poses.iter().enumerate() {
        let pose = Pose2d::new(*x, 0.0, 0.0);
        let scan = synthetic_room_scan(*x, 0.0, step as f64 * 0.1);
        map.integrate(&scan, pose).expect("integrate scan");
    }
    map.refresh_costmap().expect("costmap");

    let (width, height) = map.dimensions();
    let occupied = (0..height)
        .flat_map(|y| (0..width).map(move |x| (x, y)))
        .filter(|(x, y)| {
            map.grid.is_occupied(rne_nav::GridCoord {
                x: *x as isize,
                y: *y as isize,
            })
        })
        .count();
    println!(
        "grid {}x{} @ {:.2} m, occupied cells = {}, lethal cost cells = {}",
        width,
        height,
        map.grid.resolution_m(),
        occupied,
        map.costmap.lethal_count()
    );

    println!("\noccupancy map ('#' occupied, '.' free, ' ' unknown):");
    print!("{}", render_occupancy(&map.grid, 72, 22));

    // Map -> odom -> base_link time interpolation.
    let mut tf = TfBuffer::new();
    tf.set_transform(StampedTransform::new(
        "map",
        "odom",
        0.0,
        Transform3::from_translation_rotation(Vec3::ZERO, rne_math::Quat::IDENTITY),
    ))
    .expect("static tf");
    tf.set_transform(StampedTransform::new(
        "odom",
        "base_link",
        0.0,
        Transform3::from_translation_rotation(Vec3::new(0.0, 0.0, 0.0), rne_math::Quat::IDENTITY),
    ))
    .expect("odom tf t0");
    tf.set_transform(StampedTransform::new(
        "odom",
        "base_link",
        1.0,
        Transform3::from_translation_rotation(Vec3::new(2.0, 0.0, 0.0), rne_math::Quat::IDENTITY),
    ))
    .expect("odom tf t1");
    let base_in_map = tf
        .lookup(&FrameId::new("map"), &FrameId::new("base_link"), 0.5)
        .expect("lookup");
    println!(
        "\nTF base_link in map at t=0.5 s: x={:.3} m",
        base_in_map.translation.x
    );

    // The same pipeline through the ECS resource/system boundary.
    let mut world = rne_ecs::World::new();
    let mut ecs_map = NavMap::new(
        OccupancyGrid::new(21, 21, 0.2, Pose2d::new(-2.0, -2.0, 0.0)).expect("grid"),
        CostmapConfig::default(),
        ScanIntegrationConfig::default(),
    )
    .expect("nav map");
    ecs_map.grid.clear();
    world.insert_resource(ecs_map);
    let mut pending = PendingScans::new();
    pending.push(synthetic_ring_scan(1.0, 0.0), Pose2d::IDENTITY);
    pending.push(synthetic_ring_scan(1.0, 0.2), Pose2d::IDENTITY);
    world.insert_resource(pending);
    let report = integrate_pending_scans(&mut world).expect("system");
    println!(
        "ECS system: beams={} free_updates={} occupied_updates={}",
        report.beams_processed, report.free_updates, report.occupied_updates
    );
}

fn synthetic_room_scan(x_m: f64, y_m: f64, time_s: f64) -> LaserScan2d {
    let mut ranges_m = Vec::with_capacity(BEAM_COUNT);
    for beam in 0..BEAM_COUNT {
        let angle = TAU * beam as f64 / BEAM_COUNT as f64;
        ranges_m.push(ray_to_walls(x_m, y_m, angle));
    }
    LaserScan2d {
        time_s,
        frame: FrameId::new("laser"),
        angle_min_rad: 0.0,
        angle_increment_rad: TAU / BEAM_COUNT as f64,
        range_min_m: 0.05,
        range_max_m: 30.0,
        ranges_m,
    }
}

fn ray_to_walls(x_m: f64, y_m: f64, angle_rad: f64) -> f64 {
    let (dx, dy) = (angle_rad.cos(), angle_rad.sin());
    let mut best = f64::INFINITY;
    if dx > 1.0e-9 {
        best = best.min((ROOM_HALF_X_M - x_m) / dx);
    } else if dx < -1.0e-9 {
        best = best.min((-ROOM_HALF_X_M - x_m) / dx);
    }
    if dy > 1.0e-9 {
        best = best.min((ROOM_HALF_Y_M - y_m) / dy);
    } else if dy < -1.0e-9 {
        best = best.min((-ROOM_HALF_Y_M - y_m) / dy);
    }
    best
}

fn synthetic_ring_scan(radius_m: f64, time_s: f64) -> LaserScan2d {
    LaserScan2d {
        time_s,
        frame: FrameId::new("laser"),
        angle_min_rad: 0.0,
        angle_increment_rad: TAU / BEAM_COUNT as f64,
        range_min_m: 0.05,
        range_max_m: 10.0,
        ranges_m: vec![radius_m; BEAM_COUNT],
    }
}

fn render_occupancy(grid: &OccupancyGrid, columns: usize, rows: usize) -> String {
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
                    let coord = rne_nav::GridCoord {
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
