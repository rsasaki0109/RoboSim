//! Convert `rne_adapter_ros2` message shapes into `rclrs` ROS message types.

use rne_adapter_ros2::{
    RosClock, RosHeader, RosLaserScan, RosPointCloud2, RosPointField, RosQuaternion, RosTfMessage,
    RosTime, RosTransform, RosTransformStamped, RosVector3,
};

/// Maps adapter clock to `rosgraph_msgs/Clock`.
pub fn to_clock_message(clock: &RosClock) -> rosgraph_msgs::msg::Clock {
    rosgraph_msgs::msg::Clock {
        clock: to_time(&clock.clock),
    }
}

/// Maps adapter point cloud to `sensor_msgs/PointCloud2`.
pub fn to_pointcloud2_message(cloud: &RosPointCloud2) -> sensor_msgs::msg::PointCloud2 {
    sensor_msgs::msg::PointCloud2 {
        header: to_header(&cloud.header),
        height: cloud.height,
        width: cloud.width,
        fields: cloud.fields.iter().map(to_point_field).collect(),
        is_bigendian: false,
        point_step: cloud.point_step,
        row_step: cloud.row_step,
        data: cloud.data.clone(),
        is_dense: cloud.is_dense,
    }
}

/// Maps adapter laser scan to `sensor_msgs/LaserScan`.
pub fn to_laserscan_message(scan: &RosLaserScan) -> sensor_msgs::msg::LaserScan {
    sensor_msgs::msg::LaserScan {
        header: to_header(&scan.header),
        angle_min: scan.angle_min,
        angle_max: scan.angle_max,
        angle_increment: scan.angle_increment,
        time_increment: scan.time_increment,
        scan_time: scan.scan_time,
        range_min: scan.range_min,
        range_max: scan.range_max,
        ranges: scan.ranges.clone(),
        intensities: scan.intensities.clone(),
    }
}

/// Maps adapter TF message to `tf2_msgs/TFMessage`.
pub fn to_tf_message(tf: &RosTfMessage) -> tf2_msgs::msg::TFMessage {
    tf2_msgs::msg::TFMessage {
        transforms: tf.transforms.iter().map(to_transform_stamped).collect(),
    }
}

fn to_time(time: &RosTime) -> builtin_interfaces::msg::Time {
    builtin_interfaces::msg::Time {
        sec: time.sec,
        nanosec: time.nanosec,
    }
}

fn to_header(header: &RosHeader) -> std_msgs::msg::Header {
    std_msgs::msg::Header {
        stamp: to_time(&header.stamp),
        frame_id: header.frame_id.clone(),
    }
}

fn to_point_field(field: &RosPointField) -> sensor_msgs::msg::PointField {
    sensor_msgs::msg::PointField {
        name: field_name(&field.name),
        offset: field.offset,
        datatype: field.datatype,
        count: field.count,
    }
}

fn to_transform_stamped(transform: &RosTransformStamped) -> geometry_msgs::msg::TransformStamped {
    geometry_msgs::msg::TransformStamped {
        header: to_header(&transform.header),
        child_frame_id: transform.child_frame_id.clone(),
        transform: to_transform(&transform.transform),
    }
}

fn to_transform(transform: &RosTransform) -> geometry_msgs::msg::Transform {
    geometry_msgs::msg::Transform {
        translation: to_vector3(&transform.translation),
        rotation: to_quaternion(&transform.rotation),
    }
}

fn to_vector3(vector: &RosVector3) -> geometry_msgs::msg::Vector3 {
    geometry_msgs::msg::Vector3 {
        x: vector.x,
        y: vector.y,
        z: vector.z,
    }
}

fn to_quaternion(quaternion: &RosQuaternion) -> geometry_msgs::msg::Quaternion {
    geometry_msgs::msg::Quaternion {
        x: quaternion.x,
        y: quaternion.y,
        z: quaternion.z,
        w: quaternion.w,
    }
}

fn field_name(bytes: &[u8; 32]) -> String {
    let end = bytes
        .iter()
        .position(|&byte| byte == 0)
        .unwrap_or(bytes.len());
    String::from_utf8_lossy(&bytes[..end]).into_owned()
}

#[cfg(test)]
mod tests {
    use super::*;
    use rne_adapter_ros2::{to_ros_clock, to_ros_pointcloud2, to_ros_transform_stamped};
    use rne_core::SimTime;
    use rne_data::PointCloud;
    use rne_math::{Quat, Vec3};
    use rne_world::Transform3;

    #[test]
    fn clock_roundtrip_fields() {
        let ros = to_ros_clock(SimTime::from_ticks(1_500_000_000));
        let msg = to_clock_message(&ros);
        assert_eq!(msg.clock.sec, 1);
        assert_eq!(msg.clock.nanosec, 500_000_000);
    }

    #[test]
    fn pointcloud_preserves_xyz_layout() {
        let cloud = PointCloud {
            points_m: vec![Vec3::new(3.0, 0.5, 0.0)],
        };
        let ros = to_ros_pointcloud2(&cloud, SimTime::from_ticks(42), "lidar");
        let msg = to_pointcloud2_message(&ros);
        assert_eq!(msg.width, 1);
        assert_eq!(msg.fields.len(), 3);
        assert_eq!(msg.fields[0].name, "x");
        assert_eq!(msg.data.len(), 12);
    }

    #[test]
    fn tf_message_maps_child_frame() {
        let ros = to_ros_transform_stamped(
            "world",
            "base_link",
            Transform3::from_translation_rotation(Vec3::new(1.0, 0.0, 0.0), Quat::IDENTITY),
            SimTime::from_ticks(10),
        );
        let tf = to_tf_message(&RosTfMessage {
            transforms: vec![ros],
        });
        assert_eq!(tf.transforms.len(), 1);
        assert_eq!(tf.transforms[0].child_frame_id, "base_link");
    }
}
