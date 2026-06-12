//! ROS2 adapter mappings for Robot Native Engine.

#![deny(missing_docs)]

pub mod clock;
pub mod messages;
pub mod pointcloud;
pub mod tf;

pub use clock::{to_ros_clock, to_ros_time};
pub use messages::{
    RosClock, RosHeader, RosPointCloud2, RosPointField, RosQuaternion, RosTfMessage, RosTime,
    RosTransform, RosTransformStamped, RosVector3,
};
pub use pointcloud::{decode_xyz_points, to_ros_pointcloud2};
pub use tf::{
    to_ros_tf_message, to_ros_transform, to_ros_transform_from_matrix, to_ros_transform_stamped,
};

/// Adapter boundary marker for future ROS2 node integration.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Ros2Adapter;

impl Ros2Adapter {
    /// Returns true when the optional `ros2` feature is enabled.
    pub const fn ros2_runtime_enabled() -> bool {
        cfg!(feature = "ros2")
    }
}
