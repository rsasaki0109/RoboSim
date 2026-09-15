//! Backend-neutral multi-contact optimal control for Robot Native Engine.
//!
//! `rne_oc` is the native analogue of a differential-dynamic-programming
//! optimal-control toolbox: a discrete shooting problem over a state/control
//! trajectory, solved by DDP with Levenberg-Marquardt regularization and a
//! backtracking line search. Dynamics derivatives are computed by central
//! differences and the articulated adapter reuses `rne_dynamics` for the
//! forward dynamics.
//!
//! The solver is deterministic and has no backend, renderer, or wall-clock
//! dependency, so a maneuver is reproducible from an initial guess and a
//! configuration.

#![deny(missing_docs)]

pub mod articulated;
pub mod constrained;
pub mod ddp;
pub mod matrix;

pub use articulated::ArticulatedDynamics;
pub use constrained::{ConstrainedArticulatedDynamics, ContactPhase, ContactSequenceDynamics};
pub use ddp::{
    dynamics_derivatives, solve, CostDerivatives, CostModel, DdpConfig, DdpSolution,
    DiscreteDynamics, DynamicsDerivatives, OcError, QuadraticCost, ShootingDynamics,
    TerminalDerivatives,
};
