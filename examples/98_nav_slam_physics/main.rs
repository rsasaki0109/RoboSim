//! End-to-end navigation SLAM on real simulated LiDAR.
//!
//! A Rapier world with a walled room and asymmetric pillars, a kinematic
//! differential-drive base, and a range LiDAR sampled through `rne_sensor`.
//! Each point cloud is converted to a 2D scan and fed to `rne_slam`, which
//! corrects drifting odometry and closes a loop on the return trip.
//!
//! Run with `cargo run -p nav_slam_physics --example 98_nav_slam_physics`.

use rne_core::SimDuration;
use rne_data::PointCloud;
use rne_math::{Hertz, Quat, Vec3};
use rne_nav::{
    Costmap, CostmapConfig, ElevationConfig, ElevationMap, GridCoord, LaserScan2d, OccupancyGrid,
    Pose2d, VoxelConfig, VoxelLayer,
};
use rne_physics::{
    Collider, ColliderShape, PhysicsBackend, PhysicsWorldDesc, RigidBody, RigidBodyType,
};
use rne_physics_rapier::{step_physics, RapierBackend};
use rne_robot::{spawn_diff_drive_robot, DiffDriveConfig, DiffDriveDriveMode};
use rne_sensor::{sample_lidar, LidarSpec};
use rne_slam::{Slam2d, SlamConfig};
use rne_world::Transform3;
use std::f64::consts::PI;

const BASE_HEIGHT_M: f64 = 0.2;
const LIDAR_OFFSET_M: f64 = 0.3;
const STEP_M: f64 = 0.25;

fn main() {
    let mut world = rne_ecs::World::new();
    let mut backend = RapierBackend::new();
    let physics_world = backend
        .create_world(PhysicsWorldDesc::default())
        .expect("physics world");

    build_room(&mut world);
    let spawned = spawn_diff_drive_robot(
        &mut world,
        &DiffDriveConfig {
            drive_mode: DiffDriveDriveMode::Kinematic,
            initial_translation_m: Vec3::new(-3.0, BASE_HEIGHT_M, 0.0),
            ..DiffDriveConfig::default()
        },
    );
    let base = spawned.base_link;
    backend
        .sync_from_ecs(&mut world, physics_world)
        .expect("initial sync");

    let spec = LidarSpec {
        ray_count: 360,
        min_angle_rad: -PI,
        max_angle_rad: PI,
        max_range_m: 20.0,
        height_offset_m: 0.0,
        ..LidarSpec::default()
    };
    let dt = SimDuration::from_hertz(Hertz::new(20.0));

    let grid = OccupancyGrid::new(240, 160, 0.05, Pose2d::new(-6.0, -4.0, 0.0)).unwrap();
    let mut slam = Slam2d::new(
        grid,
        SlamConfig {
            max_beams: 360,
            ..SlamConfig::default()
        },
    );

    // Truth drives out and back along +X; odometry accumulates yaw drift.
    let mut truth = Pose2d::new(-3.0, 0.0, 0.0);
    let mut odom = truth;
    let mut loop_closures = 0;
    let mut scans = 0;
    let mut last_pose = truth;

    let route: Vec<f64> = std::iter::repeat_n(STEP_M, 24)
        .chain(std::iter::repeat_n(-STEP_M, 24))
        .collect();

    let mut last_truth = truth;
    let mut last_odom = odom;

    // The 3D sensor-to-navigation path: the same LiDAR returns feed a sparse
    // voxel obstacle layer and a 2.5D elevation map.
    let mut voxel_layer = VoxelLayer::new(VoxelConfig::default()).unwrap();
    let mut elevation = ElevationMap::new(
        240,
        160,
        0.05,
        Pose2d::new(-6.0, -4.0, 0.0),
        ElevationConfig::default(),
    )
    .unwrap();

    for step in route {
        set_base_pose(&mut world, base, truth);
        step_physics(&mut backend, &mut world, physics_world, dt).expect("step");

        let lidar_world =
            base_transform(truth).mul_transform(&Transform3::from_translation_rotation(
                Vec3::new(0.0, LIDAR_OFFSET_M, 0.0),
                Quat::IDENTITY,
            ));
        let cloud = sample_lidar(&backend, physics_world, &lidar_world, &spec);
        let scan = cloud_to_scan(&cloud, &base_transform(truth), &spec, scans as f64 * 0.05);

        let lidar_math = math_transform(&lidar_world);
        let world_points: Vec<Vec3> = cloud
            .points_m
            .iter()
            .map(|point| lidar_math.transform_point(*point))
            .collect();
        voxel_layer.integrate(&world_points);
        elevation.integrate(&world_points);

        // Process the scan with the odometry at acquisition time, then advance.
        last_truth = truth;
        last_odom = odom;
        let update = slam
            .process(&scan, odom, Pose2d::IDENTITY)
            .expect("slam update");
        loop_closures += update.loop_closures;
        last_pose = update.pose;
        scans += 1;

        truth.x_m += step;
        odom.x_m += step;
        odom.y_m += 0.004;
        odom.yaw_rad += 0.002;
    }

    let odom_error = (last_odom.x_m - last_truth.x_m).hypot(last_odom.y_m - last_truth.y_m);
    let slam_error = (last_pose.x_m - last_truth.x_m).hypot(last_pose.y_m - last_truth.y_m);
    let optimized = slam
        .graph()
        .node(slam.graph().node_count() - 1)
        .unwrap_or(last_pose);
    let optimized_error = (optimized.x_m - last_truth.x_m).hypot(optimized.y_m - last_truth.y_m);
    println!("scans processed     = {scans}");
    println!(
        "truth final         = ({:.2}, {:.2}, {:.3})",
        last_truth.x_m, last_truth.y_m, last_truth.yaw_rad
    );
    println!(
        "odom  final         = ({:.2}, {:.2}, {:.3})  error = {:.2} m",
        last_odom.x_m, last_odom.y_m, last_odom.yaw_rad, odom_error
    );
    println!(
        "slam  final         = ({:.2}, {:.2}, {:.3})  error = {:.2} m",
        last_pose.x_m, last_pose.y_m, last_pose.yaw_rad, slam_error
    );
    println!(
        "slam optimized      = ({:.2}, {:.2}, {:.3})  error = {:.2} m",
        optimized.x_m, optimized.y_m, optimized.yaw_rad, optimized_error
    );
    println!(
        "loop closures       = {loop_closures}, map occupied cells = {}",
        count_occupied(slam.grid())
    );

    let layer_grid = OccupancyGrid::new(240, 160, 0.05, Pose2d::new(-6.0, -4.0, 0.0)).unwrap();
    let mut layer_costmap =
        Costmap::from_occupancy(&layer_grid, &CostmapConfig::default()).unwrap();
    let obstacle_cells = voxel_layer.to_costmap_layer(&mut layer_costmap);
    let known_cells = elevation
        .cells()
        .iter()
        .filter(|cell| cell.is_known())
        .count();
    println!(
        "3D sensor->nav: {} voxels, {} elevation cells, {} obstacle cells",
        voxel_layer.len(),
        known_cells,
        obstacle_cells
    );
    assert!(!voxel_layer.is_empty());
    assert!(known_cells > 0);
    println!("\nSLAM occupancy map from simulated LiDAR ('#' occupied, '.' free):");
    print!("{}", render(slam.grid(), 72, 24));

    let map_dir =
        std::env::var("RNE_SLAM_MAP_DIR").unwrap_or_else(|_| "target/rne-slam-map".to_string());
    match write_ros_map(std::path::Path::new(&map_dir), slam.grid()) {
        Ok(path) => println!("wrote ROS map: {}", path.display()),
        Err(error) => eprintln!("failed to write ROS map: {error}"),
    }
}

/// Writes the occupancy grid as a ROS `map_server` PGM + YAML pair.
///
/// Returns the YAML path. Unknown cells map to 205, free to 254, occupied to 0
/// (ROS `map_server` convention); rows are written top-to-bottom.
fn write_ros_map(
    directory: &std::path::Path,
    grid: &OccupancyGrid,
) -> std::io::Result<std::path::PathBuf> {
    std::fs::create_dir_all(directory)?;
    let width = grid.width();
    let height = grid.height();
    let mut pgm = format!("P5\n{width} {height}\n255\n").into_bytes();
    for y in (0..height).rev() {
        for x in 0..width {
            let coord = GridCoord {
                x: x as isize,
                y: y as isize,
            };
            let value = if grid.is_occupied(coord) {
                0_u8
            } else if grid.is_free(coord) {
                254
            } else {
                205
            };
            pgm.push(value);
        }
    }
    let pmg_path = directory.join("map.pgm");
    std::fs::write(&pmg_path, &pgm)?;

    let origin = grid.origin();
    let yaml = format!(
        "image: map.pgm\nresolution: {}\norigin: [{}, {}, {}]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n",
        grid.resolution_m(),
        origin.x_m,
        origin.y_m,
        origin.yaw_rad,
    );
    let yaml_path = directory.join("map.yaml");
    std::fs::write(&yaml_path, yaml)?;
    Ok(yaml_path)
}

fn build_room(world: &mut rne_ecs::World) {
    // Walls are vertical in Y and lie in the X-Z ground plane.
    let wall_height_m = 1.0;
    let thickness_m = 0.2;
    mark_wall(
        world,
        Vec3::new(0.0, wall_height_m * 0.5, 3.0),
        Vec3::new(4.2, wall_height_m * 0.5, thickness_m * 0.5),
    );
    mark_wall(
        world,
        Vec3::new(0.0, wall_height_m * 0.5, -3.0),
        Vec3::new(4.2, wall_height_m * 0.5, thickness_m * 0.5),
    );
    mark_wall(
        world,
        Vec3::new(4.0, wall_height_m * 0.5, 0.0),
        Vec3::new(thickness_m * 0.5, wall_height_m * 0.5, 3.2),
    );
    mark_wall(
        world,
        Vec3::new(-4.0, wall_height_m * 0.5, 0.0),
        Vec3::new(thickness_m * 0.5, wall_height_m * 0.5, 3.2),
    );
    // Asymmetric pillars break the room's symmetry for loop closure.
    for (x, z) in [(2.0, 1.2), (-1.0, -1.6), (3.5, -1.0)] {
        mark_wall(
            world,
            Vec3::new(x, wall_height_m * 0.5, z),
            Vec3::new(0.3, wall_height_m * 0.5, 0.3),
        );
    }
}

fn mark_wall(world: &mut rne_ecs::World, translation_m: Vec3, half_extents_m: Vec3) {
    let entity = rne_ecs::spawn_named(world, "wall");
    world.entity_mut(entity).insert((
        RigidBody {
            body_type: RigidBodyType::Fixed,
            ..RigidBody::default()
        },
        Collider {
            shape: ColliderShape::Cuboid { half_extents_m },
            ..Collider::default()
        },
        Transform3::from_translation_rotation(translation_m, Quat::IDENTITY),
    ));
}

fn set_base_pose(world: &mut rne_ecs::World, base: rne_ecs::Entity, pose: Pose2d) {
    if let Some(mut transform) = world.get_mut::<Transform3>(base) {
        *transform = base_transform(pose);
    }
}

fn base_transform(pose: Pose2d) -> Transform3 {
    Transform3::from_translation_rotation(
        Vec3::new(pose.x_m, BASE_HEIGHT_M, pose.y_m),
        Quat::from_rotation_y(pose.yaw_rad),
    )
}

/// Bins a world-frame point cloud into a planar scan in the base frame.
///
/// The LiDAR horizontal plane is sensor X-Z, so the beam angle is
/// `atan2(local.z, local.x)`. The SLAM plane maps `x -> x`, `y -> z`, with
/// `sensor_from_base` = identity and the base yaw carried by the pose.
fn cloud_to_scan(
    cloud: &PointCloud,
    base_world: &Transform3,
    spec: &LidarSpec,
    time_s: f64,
) -> LaserScan2d {
    let count = spec.ray_count.max(1) as usize;
    let angle_min = spec.min_angle_rad;
    let angle_max = spec.max_angle_rad;
    let increment = (angle_max - angle_min) / count as f64;
    let no_return = spec.max_range_m * 2.0;
    let mut ranges = vec![no_return; count];

    let inverse = math_transform(base_world).inverse();
    for point in &cloud.points_m {
        let local = inverse.transform_point(*point);
        let range = (local.x * local.x + local.z * local.z).sqrt();
        if range < spec.min_range_m || range > spec.max_range_m {
            continue;
        }
        let angle = local.z.atan2(local.x);
        let mut index = ((angle - angle_min) / increment).round() as isize;
        index = index.rem_euclid(count as isize);
        let slot = &mut ranges[index as usize];
        if range < *slot {
            *slot = range;
        }
    }

    LaserScan2d {
        time_s,
        frame: rne_nav::FrameId::new("lidar"),
        angle_min_rad: angle_min,
        angle_increment_rad: increment,
        range_min_m: spec.min_range_m,
        range_max_m: spec.max_range_m,
        ranges_m: ranges,
    }
}

fn math_transform(transform: &Transform3) -> rne_math::Transform3 {
    rne_math::Transform3 {
        translation: transform.translation,
        rotation: transform.rotation,
        scale: transform.scale,
    }
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
