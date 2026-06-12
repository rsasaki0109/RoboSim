//! Joint validation helpers.

use crate::components::{Joint, JointKind, JointLimits};

/// Validates a joint position against its limits.
pub fn validate_joint_position(joint: &Joint, position: f64) -> Result<f64, JointValidationError> {
    if !joint.limits.lower.is_finite() && !joint.limits.upper.is_finite() {
        return Ok(position);
    }

    match joint.kind {
        JointKind::Continuous => Ok(position),
        JointKind::Fixed => {
            if position.abs() > f64::EPSILON {
                Err(JointValidationError::FixedJointNonZero)
            } else {
                Ok(0.0)
            }
        }
        JointKind::Revolute | JointKind::Prismatic => {
            if position < joint.limits.lower || position > joint.limits.upper {
                Err(JointValidationError::PositionOutOfLimits {
                    position,
                    lower: joint.limits.lower,
                    upper: joint.limits.upper,
                })
            } else {
                Ok(position)
            }
        }
    }
}

/// Validates joint velocity against limits.
pub fn validate_joint_velocity(joint: &Joint, velocity: f64) -> Result<f64, JointValidationError> {
    if velocity.abs() > joint.limits.max_velocity {
        return Err(JointValidationError::VelocityOutOfLimits {
            velocity,
            max_velocity: joint.limits.max_velocity,
        });
    }
    Ok(velocity)
}

/// Validates joint limits configuration.
pub fn validate_joint_limits(limits: &JointLimits) -> Result<(), JointValidationError> {
    if limits.lower > limits.upper {
        return Err(JointValidationError::InvalidLimits {
            lower: limits.lower,
            upper: limits.upper,
        });
    }
    if limits.max_velocity < 0.0 {
        return Err(JointValidationError::NegativeMaxVelocity);
    }
    Ok(())
}

/// Joint validation error.
#[derive(Clone, Copy, Debug, PartialEq, thiserror::Error)]
pub enum JointValidationError {
    /// Fixed joint received a non-zero command.
    #[error("fixed joint cannot move")]
    FixedJointNonZero,
    /// Position exceeds joint limits.
    #[error("joint position {position} outside [{lower}, {upper}]")]
    PositionOutOfLimits {
        /// Commanded position.
        position: f64,
        /// Lower limit.
        lower: f64,
        /// Upper limit.
        upper: f64,
    },
    /// Velocity exceeds joint limits.
    #[error("joint velocity {velocity} exceeds max {max_velocity}")]
    VelocityOutOfLimits {
        /// Commanded velocity.
        velocity: f64,
        /// Maximum allowed velocity.
        max_velocity: f64,
    },
    /// Invalid limit range.
    #[error("invalid joint limits [{lower}, {upper}]")]
    InvalidLimits {
        /// Lower limit.
        lower: f64,
        /// Upper limit.
        upper: f64,
    },
    /// Negative max velocity.
    #[error("max velocity must be non-negative")]
    NegativeMaxVelocity,
}

#[cfg(test)]
mod tests {
    use super::*;
    use rne_ecs::{spawn_named, World};
    use rne_math::Vec3;

    #[test]
    fn joint_limit_validation() {
        let limits = JointLimits {
            lower: -1.0,
            upper: 1.0,
            max_velocity: 2.0,
            max_effort: 10.0,
        };
        validate_joint_limits(&limits).unwrap();

        let mut world = World::new();
        let robot = spawn_named(&mut world, "robot");
        let parent = spawn_named(&mut world, "parent");
        let child = spawn_named(&mut world, "child");
        let joint = Joint {
            robot,
            parent_link: parent,
            child_link: child,
            kind: JointKind::Revolute,
            limits,
            axis: Vec3::Y,
            position: 0.0,
            velocity: 0.0,
        };

        assert_eq!(validate_joint_position(&joint, 0.5).unwrap(), 0.5);
        assert!(validate_joint_position(&joint, 2.0).is_err());
    }
}
