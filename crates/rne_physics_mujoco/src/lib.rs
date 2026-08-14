//! Optional MuJoCo backend for Robot Native Engine.
//!
//! The crate deliberately keeps MuJoCo behind the `mujoco` feature.  A normal
//! workspace build therefore does not require a MuJoCo runtime or a native
//! library.  The feature-gated backend is an intentionally small conformance
//! spike: it loads a known MJCF free-joint sphere fixture and exposes only the
//! backend-neutral [`rne_physics::PhysicsBackend`] trait.

#![deny(missing_docs)]

/// The MuJoCo feature is enabled in this build.
pub const MUJOCO_FEATURE_ENABLED: bool = cfg!(feature = "mujoco");

/// The MuJoCo ABI line expected by this crate.
pub const EXPECTED_MUJOCO_VERSION_PREFIX: &str = "3.9.";

#[cfg(feature = "mujoco")]
mod backend;

#[cfg(feature = "mujoco")]
pub use backend::{MuJoCoBackend, MuJoCoBodyHandle, MuJoCoColliderHandle, MuJoCoError};

#[cfg(all(test, not(feature = "mujoco")))]
mod tests {
    use super::MUJOCO_FEATURE_ENABLED;

    #[test]
    fn default_build_does_not_require_the_runtime() {
        let enabled = std::hint::black_box(MUJOCO_FEATURE_ENABLED);
        assert!(!enabled);
    }
}
