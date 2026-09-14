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
#[derive(Clone, Copy, Debug, Default, PartialEq)]
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

/// `geometry_msgs/Point` compatible point.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct RosPoint {
    /// X coordinate.
    pub x: f64,
    /// Y coordinate.
    pub y: f64,
    /// Z coordinate.
    pub z: f64,
}

/// `geometry_msgs/Pose` compatible planar pose.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RosPose {
    /// Position.
    pub position: RosPoint,
    /// Orientation.
    pub orientation: RosQuaternion,
}

/// `geometry_msgs/PoseWithCovariance` compatible pose with covariance.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RosPoseWithCovariance {
    /// Pose.
    pub pose: RosPose,
    /// Row-major 6x6 covariance.
    pub covariance: [f64; 36],
}

/// `geometry_msgs/Twist` compatible velocity.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct RosTwist {
    /// Linear velocity.
    pub linear: RosVector3,
    /// Angular velocity.
    pub angular: RosVector3,
}

/// `geometry_msgs/TwistWithCovariance` compatible velocity with covariance.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RosTwistWithCovariance {
    /// Twist.
    pub twist: RosTwist,
    /// Row-major 6x6 covariance.
    pub covariance: [f64; 36],
}

/// `nav_msgs/Odometry` compatible odometry message.
#[derive(Clone, Debug, PartialEq)]
pub struct RosOdometry {
    /// Message header.
    pub header: RosHeader,
    /// Child frame identifier (typically `base_link`).
    pub child_frame_id: String,
    /// Estimated pose.
    pub pose: RosPoseWithCovariance,
    /// Estimated velocity.
    pub twist: RosTwistWithCovariance,
}

/// `nav_msgs/MapMetaData` compatible occupancy grid metadata.
#[derive(Clone, Debug, PartialEq)]
pub struct RosMapMetaData {
    /// Time the map was loaded.
    pub map_load_time: RosTime,
    /// Cell size in meters.
    pub resolution: f32,
    /// Grid width in cells.
    pub width: u32,
    /// Grid height in cells.
    pub height: u32,
    /// Pose of cell `(0, 0)`'s center in the map frame.
    pub origin: RosPose,
}

/// `nav_msgs/OccupancyGrid` compatible occupancy grid.
#[derive(Clone, Debug, PartialEq)]
pub struct RosOccupancyGrid {
    /// Message header.
    pub header: RosHeader,
    /// Grid metadata.
    pub info: RosMapMetaData,
    /// Row-major cell values: `-1` unknown, `0` free, `100` occupied.
    pub data: Vec<i8>,
}

/// `geometry_msgs/PoseStamped` compatible stamped pose.
#[derive(Clone, Debug, PartialEq)]
pub struct RosPoseStamped {
    /// Message header.
    pub header: RosHeader,
    /// Pose.
    pub pose: RosPose,
}

/// `nav_msgs/Path` compatible path message.
#[derive(Clone, Debug, PartialEq)]
pub struct RosPath {
    /// Message header.
    pub header: RosHeader,
    /// Ordered poses.
    pub poses: Vec<RosPoseStamped>,
}
