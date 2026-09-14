//! `nav_msgs/Odometry` and `geometry_msgs` pose mapping helpers.

use crate::messages::{
    RosHeader, RosOdometry, RosPoint, RosPose, RosPoseWithCovariance, RosQuaternion, RosTwist,
    RosTwistWithCovariance, RosVector3,
};
use rne_core::SimTime;
use rne_nav::{Pose2d, VelocityCommand2d};

/// Converts a planar pose into a `geometry_msgs/Pose` shape.
pub fn to_ros_pose(pose: Pose2d) -> RosPose {
    RosPose {
        position: RosPoint {
            x: pose.x_m,
            y: pose.y_m,
            z: 0.0,
        },
        orientation: RosQuaternion {
            x: 0.0,
            y: 0.0,
            z: (pose.yaw_rad * 0.5).sin(),
            w: (pose.yaw_rad * 0.5).cos(),
        },
    }
}

/// Converts a velocity command into a `geometry_msgs/Twist` shape.
pub fn to_ros_twist(command: VelocityCommand2d) -> RosTwist {
    RosTwist {
        linear: RosVector3 {
            x: command.linear_m_s,
            y: 0.0,
            z: 0.0,
        },
        angular: RosVector3 {
            x: 0.0,
            y: 0.0,
            z: command.angular_rad_s,
        },
    }
}

/// Converts a `geometry_msgs/Twist` shape into a planar velocity command.
///
/// Only the forward linear and yaw angular components are used; lateral and
/// off-axis components of a differential drive are ignored.
pub fn from_ros_twist(twist: &RosTwist) -> VelocityCommand2d {
    VelocityCommand2d::new(twist.linear.x, twist.angular.z)
}

/// Converts a pose and velocity command into a `nav_msgs/Odometry` shape.
pub fn to_ros_odometry(
    pose: Pose2d,
    command: VelocityCommand2d,
    sim_time: SimTime,
    frame_id: &str,
    child_frame_id: &str,
) -> RosOdometry {
    let mut pose_covariance = [0.0; 36];
    // x, y, yaw diagonal entries of a row-major 6x6 covariance.
    pose_covariance[0] = 1.0e-3;
    pose_covariance[7] = 1.0e-3;
    pose_covariance[35] = 1.0e-3;
    let mut twist_covariance = [0.0; 36];
    twist_covariance[0] = 1.0e-2;
    twist_covariance[35] = 1.0e-2;

    RosOdometry {
        header: RosHeader {
            stamp: crate::clock::to_ros_time(sim_time),
            frame_id: frame_id.to_string(),
        },
        child_frame_id: child_frame_id.to_string(),
        pose: RosPoseWithCovariance {
            pose: to_ros_pose(pose),
            covariance: pose_covariance,
        },
        twist: RosTwistWithCovariance {
            twist: to_ros_twist(command),
            covariance: twist_covariance,
        },
    }
}

/// Converts a yaw rotation into a planar pose at the origin.
pub fn yaw_to_pose(yaw_rad: f64) -> Pose2d {
    Pose2d::new(0.0, 0.0, yaw_rad)
}

/// Returns the yaw angle in radians from a `geometry_msgs/Pose` orientation.
///
/// Uses the planar yaw formula `atan2(2(wz + xy), 1 - 2(y² + z²))`, which is
/// exact for a pure-z rotation and robust to small off-axis components.
pub fn yaw_from_ros_quaternion(orientation: RosQuaternion) -> f64 {
    let (x, y, z, w) = (orientation.x, orientation.y, orientation.z, orientation.w);
    (2.0 * (w * z + x * y)).atan2(1.0 - 2.0 * (y * y + z * z))
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn odometry_round_trips_pose_and_twist() {
        let pose = Pose2d::new(1.5, -0.5, 0.7);
        let command = VelocityCommand2d::new(0.4, -0.2);
        let message = to_ros_odometry(pose, command, SimTime::from_ticks(30), "odom", "base_link");
        assert_eq!(message.header.frame_id, "odom");
        assert_eq!(message.child_frame_id, "base_link");
        assert_relative_eq!(message.pose.pose.position.x, 1.5);
        assert_relative_eq!(message.twist.twist.linear.x, 0.4);
        assert_eq!(from_ros_twist(&message.twist.twist), command);
        assert_relative_eq!(
            yaw_from_ros_quaternion(message.pose.pose.orientation),
            0.7,
            epsilon = 1e-12
        );
    }

    #[test]
    fn twist_field_mapping_is_planar() {
        let twist = to_ros_twist(VelocityCommand2d::new(1.0, 0.5));
        assert_eq!(twist.linear.x, 1.0);
        assert_eq!(twist.linear.y, 0.0);
        assert_eq!(twist.angular.z, 0.5);
    }
}
