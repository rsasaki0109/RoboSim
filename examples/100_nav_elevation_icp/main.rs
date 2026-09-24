//! 3D `LiDAR` / point-cloud example: a 2.5D elevation map with traversability
//! queries and deterministic 3D point-to-point ICP registration.
//!
//! Run with `cargo run -p nav_elevation_icp --example 100_nav_elevation_icp`.

use rne_math::{Quat, Transform3, Vec3};
use rne_nav::{
    apply_terrain_layer, Costmap, CostmapConfig, ElevationConfig, ElevationMap, GridCoord,
    OccupancyGrid, Pose2d, TerrainConfig,
};
use rne_slam::{Icp3d, IcpConfig};

fn main() {
    elevation_map();
    terrain_costmap();
    icp_registration();
}

fn terrain_height(x: f64, _z: f64) -> f64 {
    if (0.5..1.5).contains(&x) {
        (x - 0.5) * 0.6
    } else if x >= 1.5 {
        0.6
    } else {
        0.0
    }
}

fn elevation_map() {
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
            points.push(Vec3::new(x, terrain_height(x, z), z));
            z += 0.05;
        }
        x += 0.05;
    }
    // A wall: two very different heights in one cell exceed the step threshold.
    points.push(Vec3::new(-1.05, 0.0, 0.4));
    points.push(Vec3::new(-1.05, 1.0, 0.4));

    let report = map.integrate(&points);
    let mut traversable = 0;
    let mut blocked = 0;
    for y in 0..map.height() {
        for x in 0..map.width() {
            let coord = GridCoord {
                x: x as isize,
                y: y as isize,
            };
            if map.cell(coord).map(|cell| cell.is_known()).unwrap_or(false) {
                if map.is_traversable(coord, 0.6) {
                    traversable += 1;
                } else {
                    blocked += 1;
                }
            }
        }
    }
    println!(
        "elevation: {}x{} @ {:.2} m, {} points observed, {} traversable / {} blocked cells",
        map.width(),
        map.height(),
        map.resolution_m(),
        report.points_processed,
        traversable,
        blocked
    );
    assert!(report.points_processed > 0);
    assert!(traversable > 0);
    assert!(blocked > 0);
}

fn terrain_costmap() {
    let origin = Pose2d::new(-2.0, -2.0, 0.0);
    let grid = OccupancyGrid::new(40, 40, 0.1, origin).expect("grid");
    let mut costmap = Costmap::from_occupancy(&grid, &CostmapConfig::default()).expect("costmap");
    let mut elevation =
        ElevationMap::new(40, 40, 0.1, origin, ElevationConfig::default()).expect("elevation");
    let mut points = Vec::new();
    let mut x = -1.9;
    while x < 1.9 {
        let mut z = -1.9;
        while z < 1.9 {
            let y = if (0.5..1.0).contains(&x) {
                (x - 0.5) * 2.0
            } else {
                0.0
            };
            points.push(Vec3::new(x, y, z));
            z += 0.05;
        }
        x += 0.05;
    }
    // A wall step larger than the traversability threshold.
    points.push(Vec3::new(-1.05, 0.0, 0.4));
    points.push(Vec3::new(-1.05, 1.2, 0.4));

    elevation.integrate(&points);
    let report = apply_terrain_layer(&mut costmap, &elevation, &TerrainConfig::default())
        .expect("terrain layer");
    println!(
        "terrain: {} lethal cells, {} graded cost cells",
        report.lethal_cells, report.cost_cells
    );
    assert!(report.lethal_cells > 0);
}

fn icp_registration() {
    let mut source = Vec::new();
    for i in 0..8 {
        for j in 0..8 {
            for k in 0..3 {
                let x = 0.2 * i as f64 + 0.03 * ((i * 3 + j) % 4) as f64;
                let y = 0.2 * j as f64 + 0.03 * ((j * 5 + k) % 4) as f64;
                let z = 0.3 * k as f64 + 0.02 * ((i + k) % 3) as f64;
                source.push(Vec3::new(x, y, z));
            }
        }
    }
    let truth = Transform3::from_translation_rotation(
        Vec3::new(0.15, -0.1, 0.05),
        Quat::from_rotation_z(0.2) * Quat::from_rotation_y(0.05),
    );
    let target: Vec<Vec3> = source
        .iter()
        .map(|point| truth.transform_point(*point))
        .collect();
    let initial = Transform3::from_translation_rotation(
        Vec3::new(-0.03, 0.02, 0.0),
        Quat::from_rotation_z(-0.05),
    )
    .mul_transform(&truth);

    let result =
        Icp3d::align(&source, &target, initial, &IcpConfig::default()).expect("icp alignment");
    println!(
        "icp: converged={} iterations={} correspondences={} mean residual={:.3e} m",
        result.converged, result.iterations, result.correspondences, result.mean_residual_m
    );
    assert!(result.converged);
    assert!(result.mean_residual_m < 1.0e-3);
}
