//! Example controller plugin compiled to a loadable shared library.
//!
//! This crate is the minimal reference implementation of the controller
//! plugin C ABI. It compiles to a `cdylib` that
//! [`rne_plugin::cabi::load_controller_library`] can open, and exposes the same
//! velocity-servo policy as the built-in
//! [`rne_plugin::VelocityServoController`].
//!
//! The plugin intentionally has no dependency on `rne_plugin`: a stable ABI
//! means the host and the plugin each carry their own copy of the
//! `#[repr(C)]` interface, versioned by the `rne_plugin_abi_version` symbol.

#![deny(missing_docs)]

use std::ffi::{c_char, c_void, CStr, CString};

/// ABI version this plugin implements. Keep in sync with
/// `rne_plugin::RNE_PLUGIN_ABI_VERSION`; the loader rejects mismatches.
pub const ABI_VERSION: u32 = 2;

/// Logical plugin name reported through [`rne_plugin_name`].
pub const PLUGIN_NAME: &str = "velocity_servo";

/// Joint position observation, mirroring `rne_plugin::cabi::RneJointPosition`.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct RneJointPosition {
    /// Joint name as a NUL-terminated UTF-8 string.
    pub name: *const c_char,
    /// Joint position in radians.
    pub position_rad: f64,
}

/// Joint velocity command, mirroring `rne_plugin::cabi::RneJointVelocity`.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct RneJointVelocity {
    /// Joint name as a NUL-terminated UTF-8 string.
    pub name: *const c_char,
    /// Commanded joint velocity in radians per second.
    pub velocity_rad_s: f64,
}

/// Controller state owned by the plugin for the lifetime of an instance.
struct VelocityServoState {
    name: CString,
    target_rad: f64,
    gain: f64,
    max_velocity_rad_s: f64,
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
