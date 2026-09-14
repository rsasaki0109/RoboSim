//! Backend-neutral whole-body control for Robot Native Engine.
//!
//! `rne_wbc` closes the loop around [`rne_dynamics`]: given a floating-base
//! articulated model, a set of active contact points, and task-space
//! objectives, it solves for the joint accelerations and contact wrenches that
//! realize the tasks while satisfying the floating-base equations of motion and
//! keeping the contacts fixed.
//!
//! The controller is a weighted least-squares inverse-dynamics solve over the
//! stacked unknowns `[qdd; contact forces]`. The floating-base dynamics and the
//! contact no-slip rows are assigned high weights so they act as constraints,
//! and the solution is projected into the Coulomb friction cone before the
//! required joint torques are recovered. Everything is deterministic: the same
//! inputs produce the same solution, with no backend, renderer, or wall-clock
//! dependency.
//!
//! # Coordinates
//!
//! RNE worlds are Y-up. Contact and task quantities are expressed in world
//! coordinates unless a name ends in `_local_m`.

#![deny(missing_docs)]

pub mod contact;
pub mod controller;

pub use contact::{ContactPoint, FrictionCone};
pub use controller::{
    ComTask, PostureTask, WbcError, WholeBodyConfig, WholeBodyController, WholeBodySolution,
};
