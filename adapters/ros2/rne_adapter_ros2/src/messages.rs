//! ROS2-compatible message shapes used by the adapter layer.

/// ROS time stamp.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct RosTime {
    /// Seconds since epoch.
    pub sec: i32,
    /// Nanoseconds within the current second.
    pub nanosec: u32,
}

/// `rosgraph_msgs/Clock` compatible message.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RosClock {
    /// Current simulation clock value.
    pub clock: RosTime,
}

/// Standard ROS message header.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RosHeader {
    /// Timestamp.
    pub stamp: RosTime,
    /// Coordinate frame identifier.
    pub frame_id: String,
}

/// `sensor_msgs/PointField` compatible field descriptor.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RosPointField {
    /// Field name.
    pub name: [u8; 32],
    /// Byte offset within a point.
    pub offset: u32,
    /// Numeric datatype identifier.
    pub datatype: u8,
    /// Number of elements in the field.
    pub count: u32,
}

/// `sensor_msgs/PointCloud2` compatible point cloud message.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RosPointCloud2 {
    /// Message header.
    pub header: RosHeader,
    /// Image height (1 for unorganized clouds).
    pub height: u32,
    /// Image width (number of points for unorganized clouds).
    pub width: u32,
    /// Field descriptors.
    pub fields: Vec<RosPointField>,
    /// Bytes per point.
    pub point_step: u32,
    /// Bytes per row.
    pub row_step: u32,
    /// Raw point data.
    pub data: Vec<u8>,
    /// Whether the cloud is dense.
    pub is_dense: bool,
}

/// `geometry_msgs/Quaternion` compatible orientation.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RosQuaternion {
    /// X component.
    pub x: f64,
    /// Y component.
    pub y: f64,
    /// Z component.
    pub z: f64,
    /// W component.
    pub w: f64,
}

/// `geometry_msgs/Vector3` compatible vector.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RosVector3 {
    /// X component.
    pub x: f64,
    /// Y component.
    pub y: f64,
    /// Z component.
    pub z: f64,
}

/// `geometry_msgs/Transform` compatible transform.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RosTransform {
    /// Translation.
    pub translation: RosVector3,
    /// Rotation.
    pub rotation: RosQuaternion,
}

/// `geometry_msgs/TransformStamped` compatible transform message.
#[derive(Clone, Debug, PartialEq)]
pub struct RosTransformStamped {
    /// Message header for the child frame.
    pub header: RosHeader,
    /// Parent frame identifier.
    pub child_frame_id: String,
    /// Transform from parent to child.
    pub transform: RosTransform,
}

/// `tf2_msgs/TFMessage` compatible transform array.
#[derive(Clone, Debug, PartialEq)]
pub struct RosTfMessage {
    /// Transform list.
    pub transforms: Vec<RosTransformStamped>,
}

/// `sensor_msgs/LaserScan` compatible 2D range scan.
#[derive(Clone, Debug, PartialEq)]
pub struct RosLaserScan {
    /// Message header.
    pub header: RosHeader,
    /// Start angle of the scan in radians.
    pub angle_min: f32,
    /// End angle of the scan in radians.
    pub angle_max: f32,
    /// Angular distance between measurements in radians.
    pub angle_increment: f32,
    /// Time between measurements in seconds.
    pub time_increment: f32,
    /// Time between scans in seconds.
    pub scan_time: f32,
    /// Minimum range value in meters.
    pub range_min: f32,
    /// Maximum range value in meters.
    pub range_max: f32,
    /// Range measurements in meters.
    pub ranges: Vec<f32>,
    /// Intensity measurements (optional).
    pub intensities: Vec<f32>,
}

/// `sensor_msgs/JointState` compatible joint measurement message.
#[derive(Clone, Debug, PartialEq)]
pub struct RosJointState {
    /// Message header.
    pub header: RosHeader,
    /// Joint names.
    pub names: Vec<String>,
    /// Joint positions in radians.
    pub positions: Vec<f64>,
    /// Joint velocities in radians per second.
    pub velocities: Vec<f64>,
    /// Joint efforts in newton-meters (optional).
    pub efforts: Vec<f64>,
}

/// `sensor_msgs/Image` compatible camera frame.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RosImage {
    /// Message header.
    pub header: RosHeader,
    /// Image height in pixels.
    pub height: u32,
    /// Image width in pixels.
    pub width: u32,
    /// Pixel encoding string.
    pub encoding: String,
    /// Whether data is big-endian.
    pub is_bigendian: bool,
    /// Full row length in bytes.
    pub step: u32,
    /// Raw image bytes.
    pub data: Vec<u8>,
}
