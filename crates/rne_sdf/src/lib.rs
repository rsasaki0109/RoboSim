//! Minimal SDF model import for Robot Native Engine.
//!
//! The importer converts a strict subset of [Gazebo SDF] into a URDF document
//! that the existing `rne_urdf_import` pipeline consumes, so SDF models reuse
//! the same articulation, collision, and actuator path as URDF robots.
//!
//! Supported subset:
//!
//! - `<sdf version>` root containing exactly one `<model>`
//! - `<link>` with `<inertial>` (mass/inertia), `<visual>` and `<collision>`
//!   geometry (`box`, `sphere`, `cylinder`, `mesh`) and diffuse material color
//! - `<joint>` of type `revolute`, `continuous`, `prismatic`, or `fixed`, with
//!   `<parent>`/`<child>`, `<origin>`, `<axis>`, and `<limit>`
//!
//! Worlds, multiple models, link/model `<pose>`, and unsupported geometry or
//! material elements are rejected with a clear error instead of being silently
//! dropped.
//!
//! [Gazebo SDF]: http://sdformat.org/

#![deny(missing_docs)]

pub mod error;
pub mod model;

pub use error::SdfError;
pub use model::{sdf_to_urdf, sdf_to_urdf_file};
