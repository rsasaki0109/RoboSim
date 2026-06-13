//! Observation types returned by environments.

/// Observation from a differential drive simulation step.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DiffDriveObservation {
    /// Base link X position in meters.
    pub base_x_m: f64,
    /// Base link Y position in meters.
    pub base_y_m: f64,
    /// Base link Z position in meters.
    pub base_z_m: f64,
    /// Base link yaw around world Y in radians.
    pub base_yaw_rad: f64,
    /// Commanded left wheel velocity in radians per second.
    pub left_wheel_velocity_rad_s: f64,
    /// Commanded right wheel velocity in radians per second.
    pub right_wheel_velocity_rad_s: f64,
    /// IMU linear acceleration Y in meters per second squared.
    pub imu_ay_m_s2: f64,
    /// Latest LiDAR point count when available.
    pub lidar_points: usize,
    /// Goal X minus base X when a goal was provided during observation.
    pub goal_delta_x_m: Option<f64>,
    /// Nearest peer base X minus this base X when multiple robots are present.
    pub peer_delta_x_m: Option<f64>,
    /// Nearest peer base Z minus this base Z when multiple robots are present.
    pub peer_delta_z_m: Option<f64>,
    /// Euclidean distance to the nearest peer robot base link.
    pub peer_separation_m: Option<f64>,
}

impl Default for DiffDriveObservation {
    fn default() -> Self {
        Self {
            base_x_m: 0.0,
            base_y_m: 0.0,
            base_z_m: 0.0,
            base_yaw_rad: 0.0,
            left_wheel_velocity_rad_s: 0.0,
            right_wheel_velocity_rad_s: 0.0,
            imu_ay_m_s2: 0.0,
            lidar_points: 0,
            goal_delta_x_m: None,
            peer_delta_x_m: None,
            peer_delta_z_m: None,
            peer_separation_m: None,
        }
    }
}
