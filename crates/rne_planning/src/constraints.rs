//! Goal and path constraints for a motion plan.
//!
//! Goal constraints mirror the MoveIt kinematic constraints that matter for a
//! single articulated chain: an exact joint-space goal and an end-link pose,
//! position, or orientation goal resolved by inverse kinematics. Path
//! constraints must hold at every configuration along a motion.

use crate::error::PlanningError;
use crate::scene::PlanningScene;
use rne_ecs::Entity;
use rne_math::{Pose3, Quat, Vec3};

/// Goal specification for a motion plan.
#[derive(Clone, Debug, PartialEq)]
pub enum GoalConstraint {
    /// Exact joint-space goal in degree-of-freedom order.
    Joint {
        /// Target joint positions in degree-of-freedom order.
        positions: Vec<f64>,
    },
    /// Full-pose goal for an end link, solved with a kinematics solver.
    Pose {
        /// End link whose pose is constrained.
        end_link: Entity,
        /// Desired pose of the end link in the model base frame.
        target: Pose3,
    },
    /// Position-only goal for an end link; orientation is left unconstrained.
    Position {
        /// End link whose position is constrained.
        end_link: Entity,
        /// Desired position of the end link in the model base frame.
        target: Vec3,
    },
    /// Orientation-only goal for an end link; position is left unconstrained.
    Orientation {
        /// End link whose orientation is constrained.
        end_link: Entity,
        /// Desired orientation of the end link in the model base frame.
        target: Quat,
    },
}

impl GoalConstraint {
    /// Creates a joint-space goal.
    pub fn joint(positions: Vec<f64>) -> Self {
        Self::Joint { positions }
    }

    /// Creates a full-pose goal.
    pub fn pose(end_link: Entity, target: Pose3) -> Self {
        Self::Pose { end_link, target }
    }

    /// Creates a position-only goal.
    pub fn position(end_link: Entity, target: Vec3) -> Self {
        Self::Position { end_link, target }
    }

    /// Creates an orientation-only goal.
    pub fn orientation(end_link: Entity, target: Quat) -> Self {
        Self::Orientation { end_link, target }
    }
}

/// A constraint that must hold at every configuration along a motion.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum PathConstraint {
    /// Keep an end link's orientation within `tolerance_rad` of `target`.
    Orientation {
        /// End link whose orientation is constrained.
        end_link: Entity,
        /// Desired orientation in the model base frame.
        target: Quat,
        /// Allowed orientation error in radians.
        tolerance_rad: f64,
    },
    /// Keep an end link's position within `tolerance_m` of `target`.
    Position {
        /// End link whose position is constrained.
        end_link: Entity,
        /// Desired position in the model base frame.
        target: Vec3,
        /// Allowed position error in meters.
        tolerance_m: f64,
    },
    /// Keep a target point visible from a sensor link.
    ///
    /// The sensor link's local `+X` axis is its view direction. The target must
    /// lie within `tolerance_rad` of that axis and the segment from the sensor
    /// origin to the target must be free of world and robot geometry (the sensor
    /// link itself is ignored).
    Visibility {
        /// Link carrying the sensor.
        sensor_link: Entity,
        /// Target point in the model base frame.
        target: Vec3,
        /// Maximum view-angle error in radians.
        tolerance_rad: f64,
    },
}

impl PathConstraint {
    /// Whether the constraint holds at configuration `q`.
    pub fn is_satisfied(&self, scene: &PlanningScene, q: &[f64]) -> Result<bool, PlanningError> {
        let forward = scene.model().forward_kinematics(q)?;
        let Some(transform) = forward.link_transform(self.end_link()) else {
            return Ok(false);
        };
        match self {
            PathConstraint::Orientation {
                target,
                tolerance_rad,
                ..
            } => Ok(quaternion_angle(transform.rotation, *target) <= *tolerance_rad),
            PathConstraint::Position {
                target,
                tolerance_m,
                ..
            } => Ok((transform.translation - *target).length() <= *tolerance_m),
            PathConstraint::Visibility {
                sensor_link,
                target,
                tolerance_rad,
            } => {
                let origin = transform.translation;
                let direction = *target - origin;
                let length = direction.length();
                if length <= 1.0e-9 {
                    return Ok(true);
                }
                let axis = transform.rotation * Vec3::X;
                let cosine = axis.dot(direction / length).clamp(-1.0, 1.0);
                if cosine.acos() > *tolerance_rad {
                    return Ok(false);
                }
                scene.line_of_sight_clear(q, origin, *target, &[*sensor_link])
            }
        }
    }

    /// End link the constraint applies to.
    pub fn end_link(&self) -> Entity {
        match self {
            PathConstraint::Orientation { end_link, .. }
            | PathConstraint::Position { end_link, .. } => *end_link,
            PathConstraint::Visibility { sensor_link, .. } => *sensor_link,
        }
    }
}

fn quaternion_angle(a: Quat, b: Quat) -> f64 {
    let dot = a.dot(b).abs().min(1.0);
    2.0 * dot.acos()
}
