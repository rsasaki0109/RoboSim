//! Physics ECS components.

use bevy_ecs::prelude::Component;
use rne_ecs::Entity;
use rne_math::{Quat, Vec3};
use rne_world::Transform3;
use serde::{Deserialize, Serialize};

/// Rigid body motion type.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum RigidBodyType {
    /// Fully simulated dynamic body.
    #[default]
    Dynamic,
    /// Immovable static body.
    Fixed,
    /// User-driven body with collision response.
    Kinematic,
}

/// Rigid body simulation properties.
#[derive(Component, Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct RigidBody {
    /// Motion type.
    pub body_type: RigidBodyType,
    /// Mass in kilograms. Ignored for fixed bodies.
    pub mass_kg: f64,
    /// Linear velocity in meters per second.
    pub linear_velocity_m_s: Vec3,
    /// Angular velocity in radians per second.
    pub angular_velocity_rad_s: Vec3,
}

impl Default for RigidBody {
    fn default() -> Self {
        Self {
            body_type: RigidBodyType::Dynamic,
            mass_kg: 1.0,
            linear_velocity_m_s: Vec3::ZERO,
            angular_velocity_rad_s: Vec3::ZERO,
        }
    }
}

/// Exact rigid-body centre of mass and symmetric inertia tensor.
///
/// When present, [`RigidBody::mass_kg`] together with this component defines the
/// body's complete inertial properties. Physics backends must not add collider
/// mass or infer a replacement tensor. Tensor entries are expressed about
/// [`Self::center_of_mass_local_m`] in the rigid body's local frame.
#[derive(Component, Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct RigidBodyInertia {
    /// Centre of mass in the rigid body's local frame, in metres.
    pub center_of_mass_local_m: Vec3,
    /// Inertia tensor x-x entry in kg·m².
    pub ixx_kg_m2: f64,
    /// Inertia tensor x-y entry in kg·m².
    pub ixy_kg_m2: f64,
    /// Inertia tensor x-z entry in kg·m².
    pub ixz_kg_m2: f64,
    /// Inertia tensor y-y entry in kg·m².
    pub iyy_kg_m2: f64,
    /// Inertia tensor y-z entry in kg·m².
    pub iyz_kg_m2: f64,
    /// Inertia tensor z-z entry in kg·m².
    pub izz_kg_m2: f64,
}

impl RigidBodyInertia {
    /// Returns true when every value is finite, the symmetric tensor is
    /// positive definite, and its principal moments satisfy the physical
    /// triangle inequalities.
    pub fn is_valid(self) -> bool {
        let values = [
            self.center_of_mass_local_m.x,
            self.center_of_mass_local_m.y,
            self.center_of_mass_local_m.z,
            self.ixx_kg_m2,
            self.ixy_kg_m2,
            self.ixz_kg_m2,
            self.iyy_kg_m2,
            self.iyz_kg_m2,
            self.izz_kg_m2,
        ];
        if values.into_iter().any(|value| !value.is_finite()) {
            return false;
        }
        let leading_minor_2 = self.ixx_kg_m2 * self.iyy_kg_m2 - self.ixy_kg_m2 * self.ixy_kg_m2;
        let determinant = self.ixx_kg_m2
            * (self.iyy_kg_m2 * self.izz_kg_m2 - self.iyz_kg_m2 * self.iyz_kg_m2)
            - self.ixy_kg_m2 * (self.ixy_kg_m2 * self.izz_kg_m2 - self.iyz_kg_m2 * self.ixz_kg_m2)
            + self.ixz_kg_m2 * (self.ixy_kg_m2 * self.iyz_kg_m2 - self.iyy_kg_m2 * self.ixz_kg_m2);
        if !(self.ixx_kg_m2 > 0.0 && leading_minor_2 > 0.0 && determinant > 0.0) {
            return false;
        }

        // Positive-definite symmetric matrices are not automatically
        // realizable inertia tensors. Principal moments must also satisfy the
        // triangle inequalities. Equivalently, trace(I)/2 * identity - I is a
        // positive-semidefinite second-moment matrix.
        let half_trace = (self.ixx_kg_m2 + self.iyy_kg_m2 + self.izz_kg_m2) * 0.5;
        let covariance = [
            [
                half_trace - self.ixx_kg_m2,
                -self.ixy_kg_m2,
                -self.ixz_kg_m2,
            ],
            [
                -self.ixy_kg_m2,
                half_trace - self.iyy_kg_m2,
                -self.iyz_kg_m2,
            ],
            [
                -self.ixz_kg_m2,
                -self.iyz_kg_m2,
                half_trace - self.izz_kg_m2,
            ],
        ];
        let tolerance = half_trace.abs().max(1.0) * 1.0e-12;
        let principal_minor_xy =
            covariance[0][0] * covariance[1][1] - covariance[0][1] * covariance[0][1];
        let principal_minor_xz =
            covariance[0][0] * covariance[2][2] - covariance[0][2] * covariance[0][2];
        let principal_minor_yz =
            covariance[1][1] * covariance[2][2] - covariance[1][2] * covariance[1][2];
        let covariance_determinant = covariance[0][0]
            * (covariance[1][1] * covariance[2][2] - covariance[1][2] * covariance[1][2])
            - covariance[0][1]
                * (covariance[0][1] * covariance[2][2] - covariance[1][2] * covariance[0][2])
            + covariance[0][2]
                * (covariance[0][1] * covariance[1][2] - covariance[1][1] * covariance[0][2]);
        covariance[0][0] >= -tolerance
            && covariance[1][1] >= -tolerance
            && covariance[2][2] >= -tolerance
            && principal_minor_xy >= -tolerance * tolerance
            && principal_minor_xz >= -tolerance * tolerance
            && principal_minor_yz >= -tolerance * tolerance
            && covariance_determinant >= -tolerance * tolerance * tolerance
    }
}

/// Collision shape definition.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum ColliderShape {
    /// Sphere with radius in meters.
    Sphere {
        /// Radius in meters.
        radius_m: f64,
    },
    /// Axis-aligned box half extents in meters.
    Cuboid {
        /// Half extents in meters.
        half_extents_m: Vec3,
    },
    /// Capsule aligned with the Y axis.
    Capsule {
        /// Half height in meters (excluding hemispheres).
        half_height_m: f64,
        /// Radius in meters.
        radius_m: f64,
    },
    /// Infinite plane with outward normal.
    Plane {
        /// Unit normal vector.
        normal: Vec3,
    },
}

impl Default for ColliderShape {
    fn default() -> Self {
        Self::Cuboid {
            half_extents_m: Vec3::splat(0.5),
        }
    }
}

/// Collider attached to an entity.
#[derive(Component, Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Collider {
    /// Shape definition.
    pub shape: ColliderShape,
    /// Surface material properties.
    pub material: PhysicsMaterial,
    /// Pose relative to the entity transform.
    pub local_offset: Transform3,
    /// Whether this collider reports overlap without applying contact forces.
    ///
    /// Sensor overlaps are exposed as zero-impulse [`crate::ContactEvent`] values.
    #[serde(default)]
    pub sensor: bool,
}

impl Default for Collider {
    fn default() -> Self {
        Self {
            shape: ColliderShape::default(),
            material: PhysicsMaterial::default(),
            local_offset: Transform3::IDENTITY,
            sensor: false,
        }
    }
}

impl Collider {
    /// Creates a cuboid collider with the given half extents.
    pub fn cuboid(half_extents_m: Vec3) -> Self {
        Self {
            shape: ColliderShape::Cuboid { half_extents_m },
            material: PhysicsMaterial::default(),
            local_offset: Transform3::IDENTITY,
            sensor: false,
        }
    }

    /// Creates a sphere collider with the given radius.
    pub fn sphere(radius_m: f64) -> Self {
        Self {
            shape: ColliderShape::Sphere { radius_m },
            material: PhysicsMaterial::default(),
            local_offset: Transform3::IDENTITY,
            sensor: false,
        }
    }
}

/// Pairwise collider filtering masks.
///
/// Two colliders interact when each collider's membership overlaps the other
/// collider's filter. Missing components are treated as [`Self::default`].
#[derive(Component, Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct CollisionGroups {
    /// Groups this collider belongs to.
    pub memberships: u32,
    /// Groups this collider accepts interactions from.
    pub filter: u32,
}

impl Default for CollisionGroups {
    fn default() -> Self {
        Self {
            memberships: u32::MAX,
            filter: u32::MAX,
        }
    }
}

impl CollisionGroups {
    /// Creates masks that disable interactions with colliders in the same group.
    pub const fn without_self_collision(group_bit: u32) -> Self {
        Self {
            memberships: group_bit,
            filter: !group_bit,
        }
    }
}

/// Physical surface material.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct PhysicsMaterial {
    /// Coulomb friction coefficient.
    pub friction: f32,
    /// Coefficient of restitution.
    pub restitution: f32,
}

impl Default for PhysicsMaterial {
    fn default() -> Self {
        Self {
            friction: 0.5,
            restitution: 0.0,
        }
    }
}

/// Revolute joint description for physics backends.
#[derive(Component, Clone, Copy, Debug, PartialEq)]
pub struct RevoluteJointDesc {
    /// Parent rigid body entity.
    pub parent: Entity,
    /// Joint axis in parent-local coordinates.
    pub axis: Vec3,
    /// Anchor point in the parent body's local frame.
    pub anchor_parent_m: Vec3,
    /// Anchor point in the child body's local frame.
    pub anchor_child_m: Vec3,
    /// Optional lower angle limit in radians.
    pub lower_rad: Option<f64>,
    /// Optional upper angle limit in radians.
    pub upper_rad: Option<f64>,
}

/// Prismatic (linear sliding) joint description for physics backends.
///
/// The single free degree of freedom translates the child body along `axis`,
/// expressed in the parent body's local frame.
#[derive(Component, Clone, Copy, Debug, PartialEq)]
pub struct PrismaticJointDesc {
    /// Parent rigid body entity.
    pub parent: Entity,
    /// Sliding axis in parent-local coordinates.
    pub axis: Vec3,
    /// Anchor point in the parent body's local frame.
    pub anchor_parent_m: Vec3,
    /// Anchor point in the child body's local frame.
    pub anchor_child_m: Vec3,
    /// Optional lower translation limit in meters.
    pub lower_m: Option<f64>,
    /// Optional upper translation limit in meters.
    pub upper_m: Option<f64>,
}

/// Fixed (weld) joint description for physics backends.
///
/// Rigidly locks the child body to the parent at the relative pose implied by the
/// anchors and `relative_rotation`, removing all six relative degrees of freedom.
/// Inserting this component attaches the weld on the next sync; removing it releases
/// the weld. Intended for attach-on-contact grasping (weld a grasped object to the
/// gripper at its current relative pose so it neither snaps nor drifts).
#[derive(Component, Clone, Copy, Debug, PartialEq)]
pub struct FixedJointDesc {
    /// Parent rigid body entity (e.g. the gripper link).
    pub parent: Entity,
    /// Anchor point in the parent body's local frame.
    pub anchor_parent_m: Vec3,
    /// Anchor point in the child body's local frame.
    pub anchor_child_m: Vec3,
    /// Orientation of the child frame relative to the parent frame.
    pub relative_rotation: Quat,
}

/// Marks a rigid body and its joint as part of a reduced-coordinate multibody.
///
/// Backends that support multibodies use this instead of an impulse joint. The
/// marker may also be placed on the root so it is simulated without a collider.
#[derive(Component, Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct MultibodyLink;

/// Velocity motor command applied to a joint before each physics step.
///
/// The value is interpreted as an angular velocity (rad/s) for revolute joints
/// and as a linear velocity (m/s) for prismatic joints.
#[derive(Component, Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct JointMotor {
    /// Target velocity: radians per second (revolute) or meters per second (prismatic).
    pub velocity_rad_s: f64,
    /// Velocity-tracking gain (motor damping factor). Higher values track the target
    /// velocity more stiffly under load — e.g. a joint holding weight against gravity —
    /// up to the backend's motor force cap. Defaults to `1.0`.
    #[serde(default = "default_motor_gain")]
    pub gain: f64,
    /// Position-tracking stiffness. When `0.0` (the default) the motor is a pure
    /// velocity motor. When positive, the motor also pulls the joint toward
    /// [`Self::target_position`] like a spring (with `gain` acting as its damping),
    /// which lets a joint *hold* a load against gravity without drift — required
    /// for a stable vertical lift carrying a multi-link arm.
    #[serde(default)]
    pub stiffness: f64,
    /// Position target the motor pulls toward when [`Self::stiffness`] is positive:
    /// radians (revolute) or meters along the slide axis (prismatic).
    #[serde(default)]
    pub target_position: f64,
    /// Maximum force/torque the motor may apply. `0.0` (the default) uses the backend's
    /// per-joint-type cap; a positive value overrides it (e.g. a heavier arm joint that
    /// needs more torque to track its target quickly).
    #[serde(default)]
    pub max_force: f64,
}

/// Selects how a backend interprets position/velocity servo gains.
///
/// The default preserves the historical mass-normalized response. Controllers
/// that declare gains in force or torque units must opt into [`Self::ForceBased`]
/// and retain that choice as part of their configuration evidence.
#[derive(Component, Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum JointMotorGainModel {
    /// Gains produce a target acceleration and are normalized by body inertia.
    #[default]
    AccelerationBased,
    /// Gains produce force/torque directly in the units declared by actuation.
    ForceBased,
}

/// Backend-neutral passive dynamics of a single-degree-of-freedom joint.
///
/// The component describes plant loss independently from actuator servo gains.
/// Physics backends apply viscous damping against joint velocity. Coulomb loss
/// uses the backend-neutral regularization `-magnitude * tanh(velocity /
/// transition_velocity)`. This is a smooth kinetic-friction model, not a claim
/// of true set-valued static friction or breakaway behavior.
#[derive(Component, Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum JointPassiveDynamics {
    /// Passive dynamics of a revolute joint.
    Revolute {
        /// Viscous damping coefficient in newton-metre-seconds per radian.
        viscous_damping_nm_s_per_rad: f64,
        /// Requested Coulomb-friction magnitude in newton-metres.
        coulomb_friction_nm: f64,
        /// Velocity scale of the smooth Coulomb transition in radians per second.
        #[serde(default)]
        coulomb_transition_velocity_rad_s: f64,
    },
    /// Passive dynamics of a prismatic joint.
    Prismatic {
        /// Viscous damping coefficient in newton-seconds per metre.
        viscous_damping_n_s_per_m: f64,
        /// Requested Coulomb-friction magnitude in newtons.
        coulomb_friction_n: f64,
        /// Velocity scale of the smooth Coulomb transition in metres per second.
        #[serde(default)]
        coulomb_transition_velocity_m_s: f64,
    },
}

impl JointPassiveDynamics {
    /// Returns true when every coefficient is finite and non-negative.
    pub fn has_valid_values(self) -> bool {
        match self {
            Self::Revolute {
                viscous_damping_nm_s_per_rad,
                coulomb_friction_nm,
                coulomb_transition_velocity_rad_s,
            } => {
                non_negative(viscous_damping_nm_s_per_rad)
                    && valid_coulomb_transition(
                        coulomb_friction_nm,
                        coulomb_transition_velocity_rad_s,
                    )
            }
            Self::Prismatic {
                viscous_damping_n_s_per_m,
                coulomb_friction_n,
                coulomb_transition_velocity_m_s,
            } => {
                non_negative(viscous_damping_n_s_per_m)
                    && valid_coulomb_transition(coulomb_friction_n, coulomb_transition_velocity_m_s)
            }
        }
    }

    /// Computes the signed generalized Coulomb-loss effort opposing `velocity`.
    ///
    /// Callers must first require [`Self::has_valid_values`] and a finite
    /// velocity. The result is a torque in newton-metres for revolute joints and
    /// a force in newtons for prismatic joints.
    pub fn regularized_coulomb_effort(self, velocity: f64) -> f64 {
        let (magnitude, transition_velocity) = match self {
            Self::Revolute {
                coulomb_friction_nm,
                coulomb_transition_velocity_rad_s,
                ..
            } => (coulomb_friction_nm, coulomb_transition_velocity_rad_s),
            Self::Prismatic {
                coulomb_friction_n,
                coulomb_transition_velocity_m_s,
                ..
            } => (coulomb_friction_n, coulomb_transition_velocity_m_s),
        };
        if magnitude == 0.0 {
            0.0
        } else {
            -magnitude * (velocity / transition_velocity).tanh()
        }
    }
}

fn valid_coulomb_transition(magnitude: f64, transition_velocity: f64) -> bool {
    non_negative(magnitude)
        && non_negative(transition_velocity)
        && (magnitude == 0.0 || transition_velocity > 0.0)
}

/// Unit-explicit actuation command for a one-degree-of-freedom joint.
///
/// Physics backends apply this component before each fixed step. Commands with
/// non-finite values, negative gains, negative limits, or a revolute/prismatic
/// mode that disagrees with the joint description must be rejected before the
/// step. A zero maximum effort/force disables output rather than meaning
/// unbounded output.
#[derive(Component, Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(tag = "mode", rename_all = "snake_case", deny_unknown_fields)]
pub enum JointActuation {
    /// No actuator output.
    #[default]
    Disabled,
    /// Revolute position servo.
    RevolutePosition {
        /// Desired angular position in radians.
        target_position_rad: f64,
        /// Proportional stiffness in newton-metres per radian.
        stiffness_nm_per_rad: f64,
        /// Damping in newton-metre-seconds per radian.
        damping_nm_s_per_rad: f64,
        /// Symmetric output limit in newton-metres.
        max_effort_nm: f64,
    },
    /// Revolute velocity servo.
    RevoluteVelocity {
        /// Desired angular velocity in radians per second.
        target_velocity_rad_s: f64,
        /// Velocity error gain in newton-metre-seconds per radian.
        gain_nm_s_per_rad: f64,
        /// Symmetric output limit in newton-metres.
        max_effort_nm: f64,
    },
    /// Direct revolute torque command.
    RevoluteEffort {
        /// Requested torque in newton-metres.
        effort_nm: f64,
        /// Symmetric output limit in newton-metres.
        max_effort_nm: f64,
    },
    /// Prismatic position servo.
    PrismaticPosition {
        /// Desired translation in metres.
        target_position_m: f64,
        /// Proportional stiffness in newtons per metre.
        stiffness_n_per_m: f64,
        /// Damping in newton-seconds per metre.
        damping_n_s_per_m: f64,
        /// Symmetric output limit in newtons.
        max_force_n: f64,
    },
    /// Prismatic velocity servo.
    PrismaticVelocity {
        /// Desired linear velocity in metres per second.
        target_velocity_m_s: f64,
        /// Velocity error gain in newton-seconds per metre.
        gain_n_s_per_m: f64,
        /// Symmetric output limit in newtons.
        max_force_n: f64,
    },
    /// Direct prismatic force command.
    PrismaticEffort {
        /// Requested force in newtons.
        force_n: f64,
        /// Symmetric output limit in newtons.
        max_force_n: f64,
    },
}

impl JointActuation {
    /// Returns true when every value is finite and every gain/limit is non-negative.
    pub fn has_valid_values(self) -> bool {
        match self {
            Self::Disabled => true,
            Self::RevolutePosition {
                target_position_rad,
                stiffness_nm_per_rad,
                damping_nm_s_per_rad,
                max_effort_nm,
            } => {
                target_position_rad.is_finite()
                    && non_negative(stiffness_nm_per_rad)
                    && non_negative(damping_nm_s_per_rad)
                    && non_negative(max_effort_nm)
            }
            Self::RevoluteVelocity {
                target_velocity_rad_s,
                gain_nm_s_per_rad,
                max_effort_nm,
            } => {
                target_velocity_rad_s.is_finite()
                    && non_negative(gain_nm_s_per_rad)
                    && non_negative(max_effort_nm)
            }
            Self::RevoluteEffort {
                effort_nm,
                max_effort_nm,
            } => effort_nm.is_finite() && non_negative(max_effort_nm),
            Self::PrismaticPosition {
                target_position_m,
                stiffness_n_per_m,
                damping_n_s_per_m,
                max_force_n,
            } => {
                target_position_m.is_finite()
                    && non_negative(stiffness_n_per_m)
                    && non_negative(damping_n_s_per_m)
                    && non_negative(max_force_n)
            }
            Self::PrismaticVelocity {
                target_velocity_m_s,
                gain_n_s_per_m,
                max_force_n,
            } => {
                target_velocity_m_s.is_finite()
                    && non_negative(gain_n_s_per_m)
                    && non_negative(max_force_n)
            }
            Self::PrismaticEffort {
                force_n,
                max_force_n,
            } => force_n.is_finite() && non_negative(max_force_n),
        }
    }

    /// Returns true for disabled or revolute commands.
    pub const fn supports_revolute(self) -> bool {
        matches!(
            self,
            Self::Disabled
                | Self::RevolutePosition { .. }
                | Self::RevoluteVelocity { .. }
                | Self::RevoluteEffort { .. }
        )
    }

    /// Returns true for disabled or prismatic commands.
    pub const fn supports_prismatic(self) -> bool {
        matches!(
            self,
            Self::Disabled
                | Self::PrismaticPosition { .. }
                | Self::PrismaticVelocity { .. }
                | Self::PrismaticEffort { .. }
        )
    }
}

fn non_negative(value: f64) -> bool {
    value.is_finite() && value >= 0.0
}

/// Backend-neutral completed-step state of a single-degree-of-freedom joint.
///
/// Physics backends insert or update this component during
/// [`crate::PhysicsBackend::sync_to_ecs`]. The enum keeps revolute and
/// prismatic units explicit instead of overloading one untyped coordinate.
#[derive(Component, Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum JointState {
    /// Angular coordinate and velocity of a revolute joint.
    Revolute {
        /// Joint position in radians.
        position_rad: f64,
        /// Joint velocity in radians per second.
        velocity_rad_s: f64,
    },
    /// Linear coordinate and velocity of a prismatic joint.
    Prismatic {
        /// Joint position in metres.
        position_m: f64,
        /// Joint velocity in metres per second.
        velocity_m_s: f64,
    },
    /// A fixed joint with no free coordinate.
    Fixed,
}

impl JointState {
    /// Returns the revolute position in radians when this is a revolute joint.
    pub const fn position_rad(self) -> Option<f64> {
        match self {
            Self::Revolute { position_rad, .. } => Some(position_rad),
            Self::Prismatic { .. } | Self::Fixed => None,
        }
    }

    /// Returns the prismatic position in metres when this is a prismatic joint.
    pub const fn position_m(self) -> Option<f64> {
        match self {
            Self::Prismatic { position_m, .. } => Some(position_m),
            Self::Revolute { .. } | Self::Fixed => None,
        }
    }
}

fn default_motor_gain() -> f64 {
    1.0
}

impl Default for JointMotor {
    fn default() -> Self {
        Self {
            velocity_rad_s: 0.0,
            gain: default_motor_gain(),
            stiffness: 0.0,
            target_position: 0.0,
            max_force: 0.0,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{JointActuation, JointPassiveDynamics, RigidBodyInertia};
    use rne_math::Vec3;

    #[test]
    fn passive_joint_dynamics_require_finite_non_negative_coefficients() {
        assert!(JointPassiveDynamics::Revolute {
            viscous_damping_nm_s_per_rad: 2.5,
            coulomb_friction_nm: 0.4,
            coulomb_transition_velocity_rad_s: 0.01,
        }
        .has_valid_values());
        assert!(!JointPassiveDynamics::Prismatic {
            viscous_damping_n_s_per_m: f64::NAN,
            coulomb_friction_n: 0.0,
            coulomb_transition_velocity_m_s: 0.0,
        }
        .has_valid_values());
        assert!(!JointPassiveDynamics::Revolute {
            viscous_damping_nm_s_per_rad: 0.0,
            coulomb_friction_nm: -0.1,
            coulomb_transition_velocity_rad_s: 0.01,
        }
        .has_valid_values());
        assert!(!JointPassiveDynamics::Revolute {
            viscous_damping_nm_s_per_rad: 0.0,
            coulomb_friction_nm: 0.1,
            coulomb_transition_velocity_rad_s: 0.0,
        }
        .has_valid_values());
    }

    #[test]
    fn regularized_coulomb_effort_is_smooth_bounded_and_opposes_motion() {
        let dynamics = JointPassiveDynamics::Revolute {
            viscous_damping_nm_s_per_rad: 0.0,
            coulomb_friction_nm: 0.4,
            coulomb_transition_velocity_rad_s: 0.02,
        };
        assert_eq!(dynamics.regularized_coulomb_effort(0.0), 0.0);
        let positive = dynamics.regularized_coulomb_effort(0.02);
        let negative = dynamics.regularized_coulomb_effort(-0.02);
        assert!(positive < 0.0);
        assert_eq!(positive, -negative);
        assert!(positive.abs() < 0.4);
        assert!(dynamics.regularized_coulomb_effort(1.0).abs() <= 0.4);
    }

    #[test]
    fn exact_inertia_requires_positive_definite_and_physically_realizable_tensor() {
        let valid = RigidBodyInertia {
            center_of_mass_local_m: Vec3::new(0.01, -0.02, 0.03),
            ixx_kg_m2: 0.4,
            ixy_kg_m2: 0.01,
            ixz_kg_m2: -0.02,
            iyy_kg_m2: 0.5,
            iyz_kg_m2: 0.03,
            izz_kg_m2: 0.6,
        };
        assert!(valid.is_valid());
        assert!(!RigidBodyInertia {
            iyy_kg_m2: -0.5,
            ..valid
        }
        .is_valid());
        assert!(!RigidBodyInertia {
            ixx_kg_m2: 1.0,
            ixy_kg_m2: 0.0,
            ixz_kg_m2: 0.0,
            iyy_kg_m2: 1.0,
            iyz_kg_m2: 0.0,
            izz_kg_m2: 3.0,
            ..valid
        }
        .is_valid());
    }

    #[test]
    fn joint_actuation_has_explicit_units_and_fail_closed_values() {
        let command = JointActuation::PrismaticVelocity {
            target_velocity_m_s: 0.5,
            gain_n_s_per_m: 12.0,
            max_force_n: 40.0,
        };
        assert!(command.has_valid_values());
        assert!(command.supports_prismatic());
        assert!(!command.supports_revolute());
        assert_eq!(
            serde_json::to_value(command).unwrap(),
            serde_json::json!({
                "mode": "prismatic_velocity",
                "target_velocity_m_s": 0.5,
                "gain_n_s_per_m": 12.0,
                "max_force_n": 40.0,
            })
        );

        assert!(!JointActuation::RevoluteEffort {
            effort_nm: f64::NAN,
            max_effort_nm: 10.0,
        }
        .has_valid_values());
        assert!(!JointActuation::PrismaticPosition {
            target_position_m: 0.0,
            stiffness_n_per_m: -1.0,
            damping_n_s_per_m: 1.0,
            max_force_n: 10.0,
        }
        .has_valid_values());
    }
}
