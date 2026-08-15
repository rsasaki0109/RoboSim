//! Stable C-ABI definitions shared by RNE controller-plugin authors and hosts.

use std::ffi::{c_char, c_void};

/// Version of the dependency-free Rust authoring surface.
pub const RNE_PLUGIN_SDK_VERSION: u32 = 1;
/// Oldest controller-plugin ABI accepted by the current host.
pub const RNE_PLUGIN_MIN_ABI_VERSION: u32 = 2;
/// Current controller-plugin ABI for new plugins.
pub const RNE_PLUGIN_ABI_VERSION: u32 = 3;
/// Original flat joint controller ABI retained for compatibility.
pub const RNE_PLUGIN_ABI_VERSION_V2: u32 = 2;

/// ABI-v3 bit for named joint-position observations.
pub const RNE_CONTROLLER_CAP_JOINT_POSITION_OBSERVATION: u64 = 1 << 0;
/// ABI-v3 bit for named joint-velocity observations.
pub const RNE_CONTROLLER_CAP_JOINT_VELOCITY_OBSERVATION: u64 = 1 << 1;
/// ABI-v3 bit for named joint-velocity commands.
pub const RNE_CONTROLLER_CAP_JOINT_VELOCITY_COMMAND: u64 = 1 << 2;
/// ABI-v3 bit for robot-scoped multi-robot frames.
pub const RNE_CONTROLLER_CAP_MULTI_ROBOT: u64 = 1 << 3;
/// Mask containing every capability bit defined by the current ABI.
pub const RNE_CONTROLLER_KNOWN_CAPABILITY_MASK: u64 = RNE_CONTROLLER_CAP_JOINT_POSITION_OBSERVATION
    | RNE_CONTROLLER_CAP_JOINT_VELOCITY_OBSERVATION
    | RNE_CONTROLLER_CAP_JOINT_VELOCITY_COMMAND
    | RNE_CONTROLLER_CAP_MULTI_ROBOT;

/// A joint-position observation used by the ABI-v2 callback.
///
/// `name` is a NUL-terminated UTF-8 string owned by the host and valid for the
/// duration of the callback.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct RneJointPosition {
    /// Joint name as a NUL-terminated UTF-8 string.
    pub name: *const c_char,
    /// Joint position in radians.
    pub position_rad: f64,
}

/// A joint-velocity command returned by the ABI-v2 callback.
///
/// `name` must stay valid until the host copies it.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct RneJointVelocity {
    /// Joint name as a NUL-terminated UTF-8 string.
    pub name: *const c_char,
    /// Commanded joint velocity in radians per second.
    pub velocity_rad_s: f64,
}

/// Robot-scoped joint observation passed to an ABI-v3 controller.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct RneJointObservationV3 {
    /// Stable robot ID as a host-owned NUL-terminated UTF-8 string.
    pub robot_id: *const c_char,
    /// Joint name as a host-owned NUL-terminated UTF-8 string.
    pub name: *const c_char,
    /// Joint position in radians.
    pub position_rad: f64,
    /// Joint velocity in radians per second, or zero when unavailable.
    pub velocity_rad_s: f64,
    /// One when `velocity_rad_s` is present, zero otherwise.
    pub has_velocity: u8,
    /// Reserved zero bytes for future compatible flags.
    pub reserved: [u8; 7],
}

/// Robot-scoped joint-velocity command returned by an ABI-v3 controller.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct RneJointVelocityV3 {
    /// Stable robot ID that must stay valid until the host copies it.
    pub robot_id: *const c_char,
    /// Joint name that must stay valid until the host copies it.
    pub name: *const c_char,
    /// Commanded joint velocity in radians per second.
    pub velocity_rad_s: f64,
}

/// Result returned by the ABI-v3 fixed-step callback.
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RneControllerStepResultV3 {
    /// Zero on success; non-zero when the plugin wrote an error message.
    pub status: i32,
    /// Number of initialized command entries in the host output buffer.
    pub output_count: usize,
}

/// Reports the ABI version implemented by the plugin.
pub type RnePluginAbiVersionFn = unsafe extern "C" fn() -> u32;
/// Reports the plugin logical name as a static NUL-terminated UTF-8 string.
pub type RnePluginNameFn = unsafe extern "C" fn() -> *const c_char;
/// Reports the ABI-v3 capability bitmask.
pub type RnePluginCapabilitiesFn = unsafe extern "C" fn() -> u64;

/// Creates a controller instance.
pub type RneControllerCreateFn = unsafe extern "C" fn(
    joint: *const c_char,
    target_rad: f64,
    gain: f64,
    max_velocity_rad_s: f64,
    error: *mut c_char,
    error_capacity: usize,
) -> *mut c_void;

/// Destroys a controller instance created by the create function.
pub type RneControllerDestroyFn = unsafe extern "C" fn(handle: *mut c_void);

/// Computes flat joint-velocity commands through the ABI-v2 callback.
pub type RneControllerStepFn = unsafe extern "C" fn(
    handle: *const c_void,
    observations: *const RneJointPosition,
    observation_count: usize,
    output: *mut RneJointVelocity,
    output_capacity: usize,
) -> usize;

/// Configures an ABI-v3 controller after capability negotiation.
pub type RneControllerConfigureV3Fn = unsafe extern "C" fn(
    handle: *mut c_void,
    required_capabilities: u64,
    error: *mut c_char,
    error_capacity: usize,
) -> i32;

/// Resets an ABI-v3 controller for one deterministic episode.
pub type RneControllerResetV3Fn = unsafe extern "C" fn(
    handle: *mut c_void,
    episode: u64,
    seed: u64,
    step: u64,
    sim_time_ticks: u64,
    error: *mut c_char,
    error_capacity: usize,
) -> i32;

/// Computes robot-scoped commands through the ABI-v3 callback.
pub type RneControllerStepV3Fn = unsafe extern "C" fn(
    handle: *mut c_void,
    step: u64,
    sim_time_ticks: u64,
    observations: *const RneJointObservationV3,
    observation_count: usize,
    output: *mut RneJointVelocityV3,
    output_capacity: usize,
    error: *mut c_char,
    error_capacity: usize,
) -> RneControllerStepResultV3;

/// Invokes the terminal ABI-v3 controller lifecycle hook.
pub type RneControllerShutdownV3Fn =
    unsafe extern "C" fn(handle: *mut c_void, error: *mut c_char, error_capacity: usize) -> i32;
