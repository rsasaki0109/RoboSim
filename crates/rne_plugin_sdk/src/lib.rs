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
/// Exact C header shipped to native controller-plugin authors.
pub const RNE_PLUGIN_SDK_C_HEADER: &str = include_str!("../include/rne_plugin_sdk.h");

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

    #[test]
    fn c_header_tracks_constants_structures_and_every_required_symbol() {
        for expected in [
            "#define RNE_PLUGIN_SDK_VERSION UINT32_C(1)",
            "#define RNE_CONTROLLER_C_ABI_LAYOUT_SCHEMA_VERSION UINT32_C(1)",
            "#define RNE_PLUGIN_MIN_ABI_VERSION UINT32_C(2)",
            "#define RNE_PLUGIN_ABI_VERSION UINT32_C(3)",
            "typedef struct RneJointObservationV3",
            "typedef struct RneControllerStepResultV3",
            "rne_plugin_abi_version(void)",
            "rne_plugin_name(void)",
            "rne_plugin_capabilities(void)",
            "rne_controller_create(",
            "rne_controller_destroy(",
            "rne_controller_step(",
            "rne_controller_configure_v3(",
            "rne_controller_reset_v3(",
            "rne_controller_step_v3(",
            "rne_controller_shutdown_v3(",
        ] {
            assert!(
                RNE_PLUGIN_SDK_C_HEADER.contains(expected),
                "C header omitted {expected}"
            );
        }
    }

    #[test]
    fn sixty_four_bit_c_layout_is_explicit_and_stable() {
        if usize::BITS != 64 {
            return;
        }
        assert_eq!(std::mem::size_of::<RneJointPosition>(), 16);
        assert_eq!(std::mem::align_of::<RneJointPosition>(), 8);
        assert_eq!(std::mem::offset_of!(RneJointPosition, name), 0);
        assert_eq!(std::mem::offset_of!(RneJointPosition, position_rad), 8);
        assert_eq!(std::mem::size_of::<RneJointVelocity>(), 16);
        assert_eq!(std::mem::offset_of!(RneJointVelocity, velocity_rad_s), 8);
        assert_eq!(std::mem::size_of::<RneJointObservationV3>(), 40);
        assert_eq!(std::mem::align_of::<RneJointObservationV3>(), 8);
        assert_eq!(std::mem::offset_of!(RneJointObservationV3, robot_id), 0);
        assert_eq!(std::mem::offset_of!(RneJointObservationV3, name), 8);
        assert_eq!(
            std::mem::offset_of!(RneJointObservationV3, position_rad),
            16
        );
        assert_eq!(
            std::mem::offset_of!(RneJointObservationV3, velocity_rad_s),
            24
        );
        assert_eq!(
            std::mem::offset_of!(RneJointObservationV3, has_velocity),
            32
        );
        assert_eq!(std::mem::offset_of!(RneJointObservationV3, reserved), 33);
        assert_eq!(std::mem::size_of::<RneJointVelocityV3>(), 24);
        assert_eq!(std::mem::offset_of!(RneJointVelocityV3, velocity_rad_s), 16);
        assert_eq!(std::mem::size_of::<RneControllerStepResultV3>(), 16);
        assert_eq!(std::mem::align_of::<RneControllerStepResultV3>(), 8);
        assert_eq!(std::mem::offset_of!(RneControllerStepResultV3, status), 0);
        assert_eq!(
            std::mem::offset_of!(RneControllerStepResultV3, output_count),
            8
        );
    }
}
