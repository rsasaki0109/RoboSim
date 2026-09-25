//! Backend-neutral legged walking pattern generation for Robot Native Engine.
//!
//! `rne_legged` is the deterministic template layer that sits above the native
//! articulated dynamics crate: it turns a footstep sequence into a physically
//! meaningful center-of-mass trajectory using the Linear Inverted Pendulum
//! Model (LIPM), the Divergent Component of Motion (DCM), capture-point foot
//! placement, and Kajita-style ZMP preview control.
//!
//! Nothing in this crate depends on a physics backend, a renderer, ROS 2, or
//! wall-clock time. Every plan is a plain value: the same request produces the
//! same trajectory, which makes walking patterns replayable and testable
//! headlessly.
//!
//! # Coordinates
//!
//! RNE worlds are Y-up: the vertical axis is `Y` and the walking plane is
//! `X`/`Z`. [`Horizontal`] stores the two horizontal components. Positions are
//! in meters and angles in radians, following the repository unit convention.

#![deny(missing_docs)]

pub mod centroidal;
pub mod error;
pub mod footstep;
pub mod horizontal;
pub mod lipm;
pub mod pattern;
pub mod preview;
pub mod support;

pub use centroidal::{
    distribute_contact_forces, flight_apex_height_m, flight_duration_s, raibert_foot_placement,
    CentroidalModel, CentroidalState, CentroidalTarget, ContactAllocationConfig,
    ContactForceSolution, GroundContact, SwingTrajectory,
};
pub use error::LeggedError;
pub use footstep::{
    plan_straight_walk, FootSide, Footstep, FootstepPlan, GaitSchedule, StraightWalkRequest,
    ZmpSegment,
};
pub use horizontal::Horizontal;
pub use lipm::{
    capture_point, com_velocity_from_dcm, dcm_step, footstep_from_dcm, propagate_constant_zmp,
    LimpParams, LimpState,
};
pub use pattern::{plan_walking_pattern, WalkingPattern};
pub use preview::{PreviewGains, PreviewTrajectory, ZmpPreviewController};
pub use support::{StabilityMargin, SupportPolygon};
