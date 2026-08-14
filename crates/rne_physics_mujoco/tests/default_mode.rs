#![cfg(not(feature = "mujoco"))]

use rne_physics_mujoco::MUJOCO_FEATURE_ENABLED;

#[test]
fn default_mode_has_no_native_runtime_requirement() {
    let enabled = std::hint::black_box(MUJOCO_FEATURE_ENABLED);
    assert!(!enabled);
}
