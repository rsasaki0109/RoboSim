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
    let has_intensity = cloud.intensities.len() == cloud.points_m.len();
    let mut intensities = has_intensity.then(|| vec![0.0_f32; count]);

    for (point_index, point) in cloud.points_m.iter().enumerate() {
        let local = inv.transform_point(*point);
        let range = (local.x * local.x + local.z * local.z).sqrt() as f32;
        if range <= 0.0 || range > spec.max_range_m as f32 {
            continue;
        }
        let angle = local.z.atan2(local.x) as f32;
        let index = angle_to_index(angle, angle_min, angle_max, count);
        if filled[index] {
            if range < ranges[index] {
                ranges[index] = range;
                if let Some(intensities) = &mut intensities {
                    intensities[index] = cloud.intensities[point_index];
                }
            }
        } else {
            ranges[index] = range;
            filled[index] = true;
            if let Some(intensities) = &mut intensities {
                intensities[index] = cloud.intensities[point_index];
            }
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
        range_min: spec.min_range_m as f32,
        range_max: spec.max_range_m as f32,
        ranges,
        intensities: intensities.unwrap_or_default(),
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
            ..LidarSpec::default()
        };
        let lidar_world = Transform3::from_translation_rotation(Vec3::ZERO, Quat::IDENTITY);
        let cloud = PointCloud {
            points_m: vec![Vec3::new(5.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 4.0)],
            ..PointCloud::default()
        };

        let scan =
            pointcloud_to_laserscan(&cloud, &lidar_world, &spec, SimTime::from_ticks(1), "lidar");
        assert_eq!(scan.ranges.len(), 4);
        assert!((scan.ranges[2] - 5.0).abs() < 1e-4);
        assert!((scan.ranges[3] - 4.0).abs() < 1e-4);
        assert!(scan.intensities.is_empty());
    }

    #[test]
    fn laserscan_keeps_intensity_of_nearest_return() {
        let spec = LidarSpec {
            ray_count: 1,
            min_angle_rad: 0.0,
            max_angle_rad: 0.0,
            ..LidarSpec::default()
        };
        let mut cloud = PointCloud::new();
        cloud.push_return(Vec3::new(5.0, 0.0, 0.0), 0.2, 0, 2);
        cloud.push_return(Vec3::new(3.0, 0.0, 0.0), 0.7, 0, 1);

        let scan =
            pointcloud_to_laserscan(&cloud, &Transform3::IDENTITY, &spec, SimTime::ZERO, "lidar");
        assert_eq!(scan.ranges, vec![3.0]);
        assert_eq!(scan.intensities, vec![0.7]);
    }
}
