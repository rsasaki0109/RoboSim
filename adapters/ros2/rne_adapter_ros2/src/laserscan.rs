//! 2D laser scan mapping helpers.

use crate::messages::{RosHeader, RosLaserScan};
use rne_core::SimTime;
use rne_data::PointCloud;
use rne_math::Transform3 as MathTransform3;
use rne_sensor::LidarSpec;
use rne_world::Transform3;

/// Converts a world-space point cloud into a binned `LaserScan` in the sensor frame.
pub fn pointcloud_to_laserscan(
    cloud: &PointCloud,
    lidar_world: &Transform3,
    spec: &LidarSpec,
    sim_time: SimTime,
    frame_id: &str,
) -> RosLaserScan {
    let count = spec.ray_count as usize;
    let inv = to_math_transform(lidar_world).inverse();
    let angle_min = spec.min_angle_rad as f32;
    let angle_max = spec.max_angle_rad as f32;
    let angle_increment = if count <= 1 {
        0.0
    } else {
        (angle_max - angle_min) / (count as f32 - 1.0)
    };
    let mut ranges = vec![0.0_f32; count];
    let mut filled = vec![false; count];

    for point in &cloud.points_m {
        let local = inv.transform_point(*point);
        let range = (local.x * local.x + local.z * local.z).sqrt() as f32;
        if range <= 0.0 || range > spec.max_range_m as f32 {
            continue;
        }
        let angle = local.z.atan2(local.x) as f32;
        let index = angle_to_index(angle, angle_min, angle_max, count);
        if filled[index] {
            ranges[index] = ranges[index].min(range);
        } else {
            ranges[index] = range;
            filled[index] = true;
        }
    }

    RosLaserScan {
        header: RosHeader {
            stamp: crate::clock::to_ros_time(sim_time),
            frame_id: frame_id.to_string(),
        },
        angle_min,
        angle_max,
        angle_increment,
        time_increment: 0.0,
        scan_time: 0.0,
        range_min: 0.0,
        range_max: spec.max_range_m as f32,
        ranges,
        intensities: Vec::new(),
    }
}

fn angle_to_index(angle: f32, min: f32, max: f32, count: usize) -> usize {
    if count <= 1 {
        return 0;
    }
    let span = max - min;
    if span <= f32::EPSILON {
        return 0;
    }
    let t = ((angle - min) / span).clamp(0.0, 1.0);
    (t * (count as f32 - 1.0)).round() as usize
}

fn to_math_transform(transform: &Transform3) -> MathTransform3 {
    MathTransform3 {
        translation: transform.translation,
        rotation: transform.rotation,
        scale: transform.scale,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rne_math::{Quat, Vec3};

    #[test]
    fn laserscan_bins_world_points_by_azimuth() {
        let spec = LidarSpec {
            ray_count: 4,
            min_angle_rad: -std::f64::consts::FRAC_PI_2,
            max_angle_rad: std::f64::consts::FRAC_PI_2,
            max_range_m: 20.0,
            height_offset_m: 0.0,
        };
        let lidar_world = Transform3::from_translation_rotation(Vec3::ZERO, Quat::IDENTITY);
        let cloud = PointCloud {
            points_m: vec![Vec3::new(5.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 4.0)],
        };

        let scan =
            pointcloud_to_laserscan(&cloud, &lidar_world, &spec, SimTime::from_ticks(1), "lidar");
        assert_eq!(scan.ranges.len(), 4);
        assert!((scan.ranges[2] - 5.0).abs() < 1e-4);
        assert!((scan.ranges[3] - 4.0).abs() < 1e-4);
    }
}
