//! LiDAR point cloud mapping helpers.

use crate::messages::{RosHeader, RosPointCloud2, RosPointField};
use rne_core::SimTime;
use rne_data::PointCloud;
use rne_math::Vec3;

/// `sensor_msgs/PointField::FLOAT32`.
const FIELD_FLOAT32: u8 = 7;
/// `sensor_msgs/PointField::UINT16`.
const FIELD_UINT16: u8 = 4;

/// Converts an RNE point cloud to a ROS `PointCloud2` message.
///
/// The layout grows with the attributes the cloud actually carries, so legacy
/// XYZ-only and XYZ+intensity clouds keep their historical `point_step`:
///
/// | attributes present | fields | `point_step` |
/// | --- | --- | --- |
/// | points only | `x y z` | 12 |
/// | + intensity | `x y z intensity` | 16 |
/// | + channel indices | `x y z intensity ring` | 20 |
/// | + timestamps | `x y z intensity ring time` | 24 |
///
/// `ring` and `time` follow the field names used by the Velodyne and Ouster ROS
/// drivers, so existing de-skewing nodes consume the cloud unchanged. `time` is the
/// per-point emission offset from the start of the scan, in seconds.
pub fn to_ros_pointcloud2(cloud: &PointCloud, sim_time: SimTime, frame_id: &str) -> RosPointCloud2 {
    let point_count = cloud.points_m.len();
    let has_intensity = cloud.intensities.len() == point_count;
    let has_ring = has_intensity && cloud.channel_indices.len() == point_count;
    let has_time = has_ring && cloud.timestamps_s.len() == point_count;

    let mut point_step = 12_u32;
    if has_intensity {
        point_step += 4;
    }
    if has_ring {
        point_step += 4;
    }
    if has_time {
        point_step += 4;
    }
    let width = point_count as u32;
    let mut data = Vec::with_capacity(point_count * point_step as usize);

    for (index, point) in cloud.points_m.iter().enumerate() {
        append_f32(&mut data, point.x as f32);
        append_f32(&mut data, point.y as f32);
        append_f32(&mut data, point.z as f32);
        if has_intensity {
            append_f32(&mut data, cloud.intensities[index]);
        }
        if has_ring {
            // UINT16 padded to a 4-byte slot so every field stays naturally aligned.
            data.extend_from_slice(&cloud.channel_indices[index].to_le_bytes());
            data.extend_from_slice(&[0_u8; 2]);
        }
        if has_time {
            append_f32(&mut data, cloud.timestamps_s[index] as f32);
        }
    }

    let mut fields = vec![
        RosPointField {
            name: field_name(b"x"),
            offset: 0,
            datatype: FIELD_FLOAT32,
            count: 1,
        },
        RosPointField {
            name: field_name(b"y"),
            offset: 4,
            datatype: FIELD_FLOAT32,
            count: 1,
        },
        RosPointField {
            name: field_name(b"z"),
            offset: 8,
            datatype: FIELD_FLOAT32,
            count: 1,
        },
    ];
    if has_intensity {
        fields.push(RosPointField {
            name: field_name(b"intensity"),
            offset: 12,
            datatype: FIELD_FLOAT32,
            count: 1,
        });
    }
    if has_ring {
        fields.push(RosPointField {
            name: field_name(b"ring"),
            offset: 16,
            datatype: FIELD_UINT16,
            count: 1,
        });
    }
    if has_time {
        fields.push(RosPointField {
            name: field_name(b"time"),
            offset: 20,
            datatype: FIELD_FLOAT32,
            count: 1,
        });
    }
    RosPointCloud2 {
        header: RosHeader {
            stamp: crate::clock::to_ros_time(sim_time),
            frame_id: frame_id.to_string(),
        },
        height: 1,
        width,
        fields,
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
            ..PointCloud::default()
        };
        let ros = to_ros_pointcloud2(&cloud, SimTime::from_ticks(42), "lidar");
        let decoded = decode_xyz_points(&ros);

        assert_eq!(decoded.len(), 2);
        assert!((decoded[0].x - 1.0).abs() < 1e-5);
        assert!((decoded[1].y - 2.0).abs() < 1e-5);
        assert_eq!(ros.header.frame_id, "lidar");
        assert_eq!(ros.point_step, 12);
        assert_eq!(ros.fields.len(), 3);
    }

    #[test]
    fn pointcloud_exports_aligned_intensity_ring_and_time() {
        let mut cloud = PointCloud::new();
        cloud.push_return(Vec3::new(2.0, 1.0, 0.5), 0.625, 3, 1, 11, 0.025);
        let ros = to_ros_pointcloud2(&cloud, SimTime::from_ticks(7), "lidar");

        assert_eq!(ros.point_step, 24);
        assert_eq!(ros.fields.len(), 6);
        assert_eq!(ros.fields[3].name, field_name(b"intensity"));
        assert_eq!(ros.fields[4].name, field_name(b"ring"));
        assert_eq!(ros.fields[5].name, field_name(b"time"));
        assert_eq!(ros.fields[4].datatype, FIELD_UINT16);
        assert!((f32_from_bytes(&ros.data[12..16]) - 0.625).abs() < f32::EPSILON);
        assert_eq!(u16::from_le_bytes([ros.data[16], ros.data[17]]), 11);
        assert!((f32_from_bytes(&ros.data[20..24]) - 0.025).abs() < 1e-7);
        // XYZ decoding must stay independent of the trailing attribute fields.
        let decoded = decode_xyz_points(&ros);
        assert_eq!(decoded.len(), 1);
        assert!((decoded[0].x - 2.0).abs() < 1e-5);
    }

    #[test]
    fn pointcloud_without_scan_attributes_keeps_the_legacy_intensity_layout() {
        let cloud = PointCloud {
            points_m: vec![Vec3::new(1.0, 0.0, 0.0)],
            intensities: vec![0.5],
            ..PointCloud::new()
        };
        let ros = to_ros_pointcloud2(&cloud, SimTime::from_ticks(3), "lidar");

        assert_eq!(ros.point_step, 16);
        assert_eq!(ros.fields.len(), 4);
    }
}
