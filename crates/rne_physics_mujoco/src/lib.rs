//! Optional MuJoCo backend for Robot Native Engine.
//!
//! The crate deliberately keeps MuJoCo behind the `mujoco` feature.  A normal
//! workspace build therefore does not require a MuJoCo runtime or a native
//! library. The feature-gated backend compiles backend-neutral ECS rigid bodies
//! into a backend-private MJCF model before step 0, then exposes simulation only
//! through the [`rne_physics::PhysicsBackend`] trait. A caller-owned MJCF
//! constructor remains available for the original bounded compatibility fixture.

#![deny(missing_docs)]

use rne_physics::{PhysicsBackendManifest, PhysicsBackendRepeatability, PhysicsCapability};

#[cfg(any(feature = "mujoco", test))]
mod compiler;

/// The MuJoCo feature is enabled in this build.
pub const MUJOCO_FEATURE_ENABLED: bool = cfg!(feature = "mujoco");

/// The MuJoCo ABI line expected by this crate.
pub const EXPECTED_MUJOCO_VERSION_PREFIX: &str = "3.9.";

/// Returns the versioned conformance manifest without loading the native runtime.
pub fn backend_manifest() -> PhysicsBackendManifest {
    PhysicsBackendManifest::new(
        "mujoco",
        env!("CARGO_PKG_VERSION"),
        "mujoco",
        "3.9.0",
        [
            PhysicsCapability::RigidBody,
            PhysicsCapability::Articulation,
            PhysicsCapability::ContactForce,
        ],
        PhysicsBackendRepeatability::ToleranceBounded,
    )
    .expect("the built-in MuJoCo backend manifest is valid")
}

#[cfg(feature = "mujoco")]
mod backend;

#[cfg(feature = "mujoco")]
pub use backend::{MuJoCoBackend, MuJoCoBodyHandle, MuJoCoColliderHandle, MuJoCoError};

#[cfg(all(test, not(feature = "mujoco")))]
mod tests {
    use super::{backend_manifest, MUJOCO_FEATURE_ENABLED};
    use rne_physics::{PhysicsBackendRepeatability, PhysicsCapability};

    #[test]
    fn default_build_does_not_require_the_runtime() {
        let enabled = std::hint::black_box(MUJOCO_FEATURE_ENABLED);
        assert!(!enabled);
    }

    #[test]
    fn manifest_is_available_without_loading_mujoco() {
        let manifest = backend_manifest();
        assert_eq!(manifest.backend_id, "mujoco");
        assert_eq!(
            manifest.capabilities,
            vec![
                PhysicsCapability::RigidBody,
                PhysicsCapability::Articulation,
                PhysicsCapability::ContactForce,
            ]
        );
        assert_eq!(
            manifest.repeatability,
            PhysicsBackendRepeatability::ToleranceBounded
        );
        manifest.validate().expect("static manifest validates");
    }
}
