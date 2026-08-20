//! Example controller plugin compiled to a loadable shared library.
//!
//! This crate is the minimal reference implementation of the controller
//! plugin C ABI. It compiles to a `cdylib` that
//! `rne_plugin::cabi::load_controller_library` can open, and exposes the same
//! velocity-servo policy as the built-in `rne_plugin::VelocityServoController`.
//!
//! The plugin depends only on the host-independent `rne_plugin_sdk` ABI crate;
//! no host implementation type or allocator crosses the shared-library
//! boundary.

#![deny(missing_docs)]

pub use rne_plugin_sdk::{
    RneControllerStepResultV3, RneJointObservationV3, RneJointPosition, RneJointVelocity,
    RneJointVelocityV3,
};
use rne_plugin_sdk::{
    RNE_CONTROLLER_CAP_JOINT_POSITION_OBSERVATION, RNE_CONTROLLER_CAP_JOINT_VELOCITY_COMMAND,
    RNE_CONTROLLER_CAP_MULTI_ROBOT, RNE_PLUGIN_ABI_VERSION,
};
use std::ffi::{c_char, c_void, CStr, CString};

/// Current ABI version implemented by this plugin.
pub const ABI_VERSION: u32 = RNE_PLUGIN_ABI_VERSION;

const CAPABILITIES: u64 = RNE_CONTROLLER_CAP_JOINT_POSITION_OBSERVATION
    | RNE_CONTROLLER_CAP_JOINT_VELOCITY_COMMAND
    | RNE_CONTROLLER_CAP_MULTI_ROBOT;

/// Logical plugin name reported through [`rne_plugin_name`].
pub const PLUGIN_NAME: &str = "velocity_servo";

/// Controller state owned by the plugin for the lifetime of an instance.
struct VelocityServoState {
    name: CString,
    target_rad: f64,
    gain: f64,
    max_velocity_rad_s: f64,
    configured: bool,
    active: bool,
    shutdown: bool,
}

/// Pure velocity-servo policy: `gain * (target - position)`, clamped.
pub fn velocity_command(
    target_rad: f64,
    gain: f64,
    max_velocity_rad_s: f64,
    position_rad: f64,
) -> f64 {
    (gain * (target_rad - position_rad)).clamp(-max_velocity_rad_s, max_velocity_rad_s)
}

/// Reports the ABI version this plugin was built against.
#[no_mangle]
pub extern "C" fn rne_plugin_abi_version() -> u32 {
    ABI_VERSION
}

static PLUGIN_NAME_C: &[u8] = b"velocity_servo\0";

/// Reports the plugin's logical name as a static NUL-terminated UTF-8 string.
#[no_mangle]
pub extern "C" fn rne_plugin_name() -> *const c_char {
    PLUGIN_NAME_C.as_ptr().cast()
}

/// Reports the supported ABI-v3 capability mask.
#[no_mangle]
pub extern "C" fn rne_plugin_capabilities() -> u64 {
    CAPABILITIES
}

/// Creates a velocity-servo controller instance.
///
/// Returns a non-null opaque handle on success, or null after writing a
/// message into `error` (if non-null with capacity) on failure.
///
/// # Safety
///
/// `joint` must be null or point to a NUL-terminated UTF-8 string. `error` must
/// be null or point to a buffer of `error_capacity` bytes.
#[no_mangle]
pub unsafe extern "C" fn rne_controller_create(
    joint: *const c_char,
    target_rad: f64,
    gain: f64,
    max_velocity_rad_s: f64,
    error: *mut c_char,
    error_capacity: usize,
) -> *mut c_void {
    if joint.is_null() {
        write_error(error, error_capacity, "joint must not be null");
        return std::ptr::null_mut();
    }
    // SAFETY: `joint` is a valid NUL-terminated string by contract.
    let joint = match unsafe { CStr::from_ptr(joint) }.to_str() {
        Ok(text) => text,
        Err(_) => {
            write_error(error, error_capacity, "joint is not valid UTF-8");
            return std::ptr::null_mut();
        }
    };
    if joint.is_empty() {
        write_error(error, error_capacity, "joint must not be empty");
        return std::ptr::null_mut();
    }
    if !target_rad.is_finite() || !gain.is_finite() || !max_velocity_rad_s.is_finite() {
        write_error(error, error_capacity, "parameters must be finite");
        return std::ptr::null_mut();
    }
    if gain < 0.0 || max_velocity_rad_s < 0.0 {
        write_error(
            error,
            error_capacity,
            "gain and max_velocity_rad_s must be non-negative",
        );
        return std::ptr::null_mut();
    }
    let name = match CString::new(joint) {
        Ok(name) => name,
        Err(_) => {
            write_error(error, error_capacity, "joint contains a NUL byte");
            return std::ptr::null_mut();
        }
    };
    let state = Box::new(VelocityServoState {
        name,
        target_rad,
        gain,
        max_velocity_rad_s,
        configured: false,
        active: false,
        shutdown: false,
    });
    Box::into_raw(state).cast::<c_void>()
}

/// Destroys a controller instance created by `rne_controller_create`.
///
/// # Safety
///
/// `handle` must be a non-null pointer returned by `rne_controller_create` and
/// not already destroyed.
#[no_mangle]
pub unsafe extern "C" fn rne_controller_destroy(handle: *mut c_void) {
    // SAFETY: `handle` is a live instance by contract; the null pointer is a
    // no-op so failed creates can still be dropped.
    if handle.is_null() {
        return;
    }
    drop(unsafe { Box::from_raw(handle.cast::<VelocityServoState>()) });
}

/// Computes velocity commands for the current joint observations.
///
/// Returns the number of commands written into `output` (at most one: the
/// commanded joint), and writes at most `output_capacity` entries.
///
/// # Safety
///
/// `handle` must be a live instance, `observations` must point to
/// `observation_count` valid entries with NUL-terminated names, and `output`
/// must point to a buffer of `output_capacity` entries.
#[no_mangle]
pub unsafe extern "C" fn rne_controller_step(
    handle: *const c_void,
    observations: *const RneJointPosition,
    observation_count: usize,
    output: *mut RneJointVelocity,
    output_capacity: usize,
) -> usize {
    // SAFETY: `handle` is a live instance by contract.
    let state = unsafe { &*handle.cast::<VelocityServoState>() };
    if output_capacity == 0 {
        return 0;
    }
    // SAFETY: `observations` points to `observation_count` valid entries.
    let observations = unsafe { std::slice::from_raw_parts(observations, observation_count) };
    for observation in observations {
        if observation.name.is_null() {
            continue;
        }
        // SAFETY: `observation.name` is a valid NUL-terminated string.
        if unsafe { CStr::from_ptr(observation.name) }.to_bytes() != state.name.to_bytes() {
            continue;
        }
        let velocity = velocity_command(
            state.target_rad,
            state.gain,
            state.max_velocity_rad_s,
            observation.position_rad,
        );
        // SAFETY: `output` points to `output_capacity` entries and we wrote at
        // most one; the name pointer stays valid while the state is alive.
        unsafe {
            *output = RneJointVelocity {
                name: state.name.as_ptr(),
                velocity_rad_s: velocity,
            }
        };
        return 1;
    }
    0
}

/// Accepts the host's negotiated ABI-v3 capability requirements.
///
/// # Safety
///
/// `handle` must be a live instance and `error` must be null or point to
/// `error_capacity` writable bytes.
#[no_mangle]
pub unsafe extern "C" fn rne_controller_configure_v3(
    handle: *mut c_void,
    required_capabilities: u64,
    error: *mut c_char,
    error_capacity: usize,
) -> i32 {
    // SAFETY: `handle` is a live instance by contract.
    let state = unsafe { &mut *handle.cast::<VelocityServoState>() };
    if state.shutdown {
        write_error(error, error_capacity, "controller is shut down");
        return 1;
    }
    if required_capabilities & !CAPABILITIES != 0 {
        write_error(error, error_capacity, "unsupported required capability");
        return 1;
    }
    state.configured = true;
    0
}

/// Activates or resets one deterministic ABI-v3 episode.
///
/// # Safety
///
/// `handle` must be a live instance and `error` must be null or point to
/// `error_capacity` writable bytes.
#[no_mangle]
pub unsafe extern "C" fn rne_controller_reset_v3(
    handle: *mut c_void,
    _episode: u64,
    _seed: u64,
    _step: u64,
    _sim_time_ticks: u64,
    error: *mut c_char,
    error_capacity: usize,
) -> i32 {
    // SAFETY: `handle` is a live instance by contract.
    let state = unsafe { &mut *handle.cast::<VelocityServoState>() };
    if !state.configured || state.shutdown {
        write_error(
            error,
            error_capacity,
            "controller must be configured and not shut down",
        );
        return 1;
    }
    state.active = true;
    0
}

/// Computes robot-scoped velocity commands for one ABI-v3 fixed step.
///
/// # Safety
///
/// `handle` must be a live instance, `observations` and `output` must point to
/// arrays of their declared lengths, identifiers must be NUL-terminated, and
/// `error` must be null or point to `error_capacity` writable bytes.
#[no_mangle]
pub unsafe extern "C" fn rne_controller_step_v3(
    handle: *mut c_void,
    _step: u64,
    _sim_time_ticks: u64,
    observations: *const RneJointObservationV3,
    observation_count: usize,
    output: *mut RneJointVelocityV3,
    output_capacity: usize,
    error: *mut c_char,
    error_capacity: usize,
) -> RneControllerStepResultV3 {
    // SAFETY: `handle` is a live instance by contract.
    let state = unsafe { &mut *handle.cast::<VelocityServoState>() };
    if !state.active || state.shutdown {
        write_error(error, error_capacity, "controller is not active");
        return RneControllerStepResultV3 {
            status: 1,
            output_count: 0,
        };
    }
    // SAFETY: the host supplies arrays with the declared lengths.
    let observations = unsafe { std::slice::from_raw_parts(observations, observation_count) };
    let mut output_count = 0;
    for observation in observations {
        if observation.robot_id.is_null() || observation.name.is_null() {
            continue;
        }
        // SAFETY: observation names are NUL-terminated by contract.
        if unsafe { CStr::from_ptr(observation.name) }.to_bytes() != state.name.to_bytes() {
            continue;
        }
        if output_count >= output_capacity {
            write_error(error, error_capacity, "output capacity is too small");
            return RneControllerStepResultV3 {
                status: 1,
                output_count: 0,
            };
        }
        let velocity = velocity_command(
            state.target_rad,
            state.gain,
            state.max_velocity_rad_s,
            observation.position_rad,
        );
        // SAFETY: `output_count < output_capacity`; pointers stay valid until
        // the host copies the result immediately after this call.
        unsafe {
            *output.add(output_count) = RneJointVelocityV3 {
                robot_id: observation.robot_id,
                name: state.name.as_ptr(),
                velocity_rad_s: velocity,
            }
        };
        output_count += 1;
    }
    RneControllerStepResultV3 {
        status: 0,
        output_count,
    }
}

/// Terminates the ABI-v3 lifecycle before instance destruction.
///
/// # Safety
///
/// `handle` must be a live instance and `error` must be null or point to
/// `error_capacity` writable bytes.
#[no_mangle]
pub unsafe extern "C" fn rne_controller_shutdown_v3(
    handle: *mut c_void,
    error: *mut c_char,
    error_capacity: usize,
) -> i32 {
    // SAFETY: `handle` is a live instance by contract.
    let state = unsafe { &mut *handle.cast::<VelocityServoState>() };
    if state.shutdown {
        write_error(error, error_capacity, "controller is already shut down");
        return 1;
    }
    state.active = false;
    state.shutdown = true;
    0
}

/// Writes a NUL-terminated message into `error` when it has capacity.
///
/// # Safety
///
/// `error` must be null or point to a buffer of `error_capacity` bytes.
unsafe fn write_error(error: *mut c_char, error_capacity: usize, message: &str) {
    if error.is_null() || error_capacity == 0 {
        return;
    }
    let bytes = message.as_bytes();
    let length = bytes.len().min(error_capacity - 1);
    // SAFETY: `error` points to `error_capacity` bytes and `length + 1` fit.
    unsafe {
        std::ptr::copy_nonoverlapping(bytes.as_ptr().cast::<c_char>(), error, length);
        *error.add(length) = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn velocity_command_matches_the_built_in_policy() {
        assert_eq!(velocity_command(1.0, 2.0, 5.0, 0.25), 1.5);
        assert_eq!(velocity_command(1.0, 2.0, 5.0, -10.0), 5.0);
        assert_eq!(velocity_command(1.0, 2.0, 5.0, 1.0), 0.0);
    }
}
