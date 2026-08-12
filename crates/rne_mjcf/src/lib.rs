//! Minimal MuJoCo MJCF model import for Robot Native Engine.
//!
//! The importer converts a strict subset of the [MuJoCo MJCF] format into a
//! URDF document that the existing `rne_urdf_import` pipeline consumes, so MJCF
//! models reuse the same articulation, collision, and actuator path as URDF and
//! SDF robots.
//!
//! Supported subset:
//!
//! - `<mujoco model>` with an optional `<compiler angle>` (degree or radian)
//! - one root `<worldbody><body>` with nested `<body>` elements
//! - `hinge` and `slide` joints with `axis` and `range` (converted to radians
//!   under the degree convention)
//! - `box`, `sphere`, and `cylinder` geoms with `pos` and `rgba`
//!
//! Body/geom rotations (`quat`, `euler`, `zaxis`), free/ball/universal joints,
//! meshes, capsules, and assets are rejected with a clear error instead of being
//! silently dropped. Inertial is not derived from geoms; the URDF importer
//! assigns its default masses.
//!
//! [MuJoCo MJCF]: https://mujoco.readthedocs.io/en/stable/XMLreference.html

#![deny(missing_docs)]

pub mod error;
pub mod model;

pub use error::MjcfError;
pub use model::{mjcf_to_urdf, mjcf_to_urdf_file, MJCF_MAX_INPUT_BYTES};
