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
    /// IMU linear acceleration Y in meters per second squared.
    pub imu_ay_m_s2: f64,
    /// Latest LiDAR point count when available.
    pub lidar_points: usize,
}
