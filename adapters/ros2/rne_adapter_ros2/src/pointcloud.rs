//! LiDAR point cloud mapping helpers.

use crate::messages::{RosHeader, RosPointCloud2, RosPointField};
use rne_core::SimTime;
use rne_data::PointCloud;
use rne_math::Vec3;

const FIELD_X: u8 = 7;
const FIELD_Y: u8 = 7;
const FIELD_Z: u8 = 7;

/// Converts an RNE point cloud to a ROS `PointCloud2` message.
pub fn to_ros_pointcloud2(cloud: &PointCloud, sim_time: SimTime, frame_id: &str) -> RosPointCloud2 {
    let point_step = 12_u32;
    let width = cloud.points_m.len() as u32;
    let mut data = Vec::with_capacity(cloud.points_m.len() * point_step as usize);

    for point in &cloud.points_m {
        append_f32(&mut data, point.x as f32);
        append_f32(&mut data, point.y as f32);
        append_f32(&mut data, point.z as f32);
    }

    RosPointCloud2 {
        header: RosHeader {
            stamp: crate::clock::to_ros_time(sim_time),
            frame_id: frame_id.to_string(),
        },
        height: 1,
        width,
        fields: vec![
            RosPointField {
                name: field_name(b"x"),
                offset: 0,
                datatype: FIELD_X,
                count: 1,
            },
            RosPointField {
                name: field_name(b"y"),
                offset: 4,
                datatype: FIELD_Y,
                count: 1,
            },
            RosPointField {
                name: field_name(b"z"),
                offset: 8,
                datatype: FIELD_Z,
                count: 1,
            },
        ],
        point_step,
        row_step: point_step * width,
        data,
        is_dense: true,
    }
}

fn append_f32(buffer: &mut Vec<u8>, value: f32) {
    buffer.extend_from_slice(&value.to_le_bytes());
}

fn field_name(bytes: &[u8]) -> [u8; 32] {
    let mut name = [0_u8; 32];
    let len = bytes.len().min(32);
    name[..len].copy_from_slice(&bytes[..len]);
    name
}

/// Returns XYZ points decoded from a ROS `PointCloud2` message.
pub fn decode_xyz_points(message: &RosPointCloud2) -> Vec<Vec3> {
    let mut points = Vec::with_capacity(message.width as usize);
    for index in 0..message.width as usize {
        let start = index * message.point_step as usize;
        let end = start + message.point_step as usize;
        if end > message.data.len() {
            break;
        }
        let chunk = &message.data[start..end];
        points.push(Vec3::new(
            f32_from_bytes(&chunk[0..4]) as f64,
            f32_from_bytes(&chunk[4..8]) as f64,
            f32_from_bytes(&chunk[8..12]) as f64,
        ));
    }
    points
}

fn f32_from_bytes(bytes: &[u8]) -> f32 {
    let mut array = [0_u8; 4];
    array.copy_from_slice(bytes);
    f32::from_le_bytes(array)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pointcloud_roundtrip_preserves_points() {
        let cloud = PointCloud {
            points_m: vec![Vec3::new(1.0, 0.0, 0.5), Vec3::new(0.0, 2.0, 0.0)],
        };
        let ros = to_ros_pointcloud2(&cloud, SimTime::from_ticks(42), "lidar");
        let decoded = decode_xyz_points(&ros);

        assert_eq!(decoded.len(), 2);
        assert!((decoded[0].x - 1.0).abs() < 1e-5);
        assert!((decoded[1].y - 2.0).abs() < 1e-5);
        assert_eq!(ros.header.frame_id, "lidar");
    }
}
