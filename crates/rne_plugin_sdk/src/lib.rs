//! Dependency-free Rust authoring surface for RNE controller plugins.
//!
//! This crate contains only stable C-ABI constants, `repr(C)` data structures,
//! and callback signatures. It has no dependency on the RNE host, ECS,
//! renderer, physics backend, robotics adapter, or allocator boundary. A
//! plugin may depend on this crate or vendor [`RNE_PLUGIN_SDK_RUST_SOURCE`].

#![deny(missing_docs)]

mod abi;

pub use abi::*;

/// Exact standalone SDK module source vendored by the offline plugin scaffold.
pub const RNE_PLUGIN_SDK_RUST_SOURCE: &str = include_str!("abi.rs");

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn capability_bits_are_disjoint_and_complete() {
        let bits = [
            RNE_CONTROLLER_CAP_JOINT_POSITION_OBSERVATION,
            RNE_CONTROLLER_CAP_JOINT_VELOCITY_OBSERVATION,
            RNE_CONTROLLER_CAP_JOINT_VELOCITY_COMMAND,
            RNE_CONTROLLER_CAP_MULTI_ROBOT,
        ];
        assert!(bits.iter().all(|bit| bit.count_ones() == 1));
        assert_eq!(
            bits.into_iter().fold(0, |mask, bit| mask | bit),
            RNE_CONTROLLER_KNOWN_CAPABILITY_MASK
        );
    }

    #[test]
    fn vendored_source_contains_the_current_contract_without_self_include() {
        assert!(RNE_PLUGIN_SDK_RUST_SOURCE.contains("pub const RNE_PLUGIN_SDK_VERSION: u32 = 1;"));
        assert!(RNE_PLUGIN_SDK_RUST_SOURCE.contains("pub const RNE_PLUGIN_ABI_VERSION: u32 = 3;"));
        assert!(RNE_PLUGIN_SDK_RUST_SOURCE.contains("pub struct RneJointObservationV3"));
        assert!(RNE_PLUGIN_SDK_RUST_SOURCE.contains("pub type RneControllerStepV3Fn"));
        assert!(!RNE_PLUGIN_SDK_RUST_SOURCE.contains("include_str!"));
    }
}
