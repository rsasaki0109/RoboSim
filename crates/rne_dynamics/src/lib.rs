//! Backend-neutral articulated-body dynamics for Robot Native Engine.
//!
//! `rne_dynamics` is the native analogue of a rigid-body dynamics library: it
//! derives a tree model directly from the [`rne_robot`] link/joint graph and
//! evaluates the standard algorithms without any physics backend, renderer, or
//! wall-clock dependency.
//!
//! # Conventions
//!
//! * Spatial motion vectors are ordered `[linear; angular]`, matching
//!   [`rne_robot::Jacobian`].
//! * Spatial force vectors (wrenches) are ordered `[force; torque]`, so the
//!   dot product of a wrench with a motion vector is power.
//! * A joint connects a parent link to a child link, and the child link's local
//!   [`rne_world::Transform3`] is the joint origin. The joint axis is expressed
//!   in the child frame, exactly as produced by `rne_urdf_import`.
//!
//! # Floating base
//!
//! When the base link carries [`rne_robot::FloatingBase`], the model gains six
//! degrees of freedom. The base configuration is `(x, y, z, roll, pitch, yaw)`
//! with fixed-axis roll-pitch-yaw, while the base *generalized velocity and
//! acceleration* are the body-frame spatial twist and its derivative. This
//! mirrors the free-flyer separation between configuration and tangent space
//! used by native dynamics libraries: `q` drives the transforms, `qd`/`qdd`
//! drive the Newton-Euler recursion.

#![deny(missing_docs)]

pub mod algorithms;
pub mod model;
pub mod spatial;

pub use algorithms::{
    center_of_mass, centroidal_momentum, com_jacobian, constrained_forward_dynamics,
    forward_dynamics, forward_dynamics_gradient, frame_jacobian, gravity_torque, impulse_velocity,
    integrate_configuration, link_motions, link_pose_body_twist_derivatives, link_pose_derivatives,
    mass_matrix, mass_matrix_gradient, non_linear_effects, non_linear_effects_gradient, rnea,
    ContactSpec, DenseMatrix, ForwardDynamicsGradient, LinkMotion, LinkPoseDerivative,
    NonLinearEffectsGradient, CONTACT_REGULARIZATION,
};
pub use model::{ArticulatedModel, DynamicsError};
pub use spatial::{SpatialInertia, SpatialVec};
