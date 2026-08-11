//! Frozen controller plugin built against the original RNE C ABI v2.
//!
//! This crate intentionally has no dependency on `rne_plugin` and must not be
//! upgraded to newer symbols. It proves that the newest runtime still loads a
//! binary authored against the oldest supported ABI.

#![deny(missing_docs)]

use std::ffi::{c_char, c_void, CStr, CString};

/// Frozen ABI version implemented by this fixture.
pub const ABI_VERSION: u32 = 2;
/// Stable logical fixture name.
pub const PLUGIN_NAME: &str = "legacy_velocity_servo_v2";

/// ABI-v2 named joint-position observation.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct RneJointPosition {
    /// NUL-terminated joint name owned by the host during the call.
    pub name: *const c_char,
    /// Joint position in radians.
    pub position_rad: f64,
}

/// ABI-v2 named joint-velocity command.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct RneJointVelocity {
    /// NUL-terminated joint name owned by the fixture state.
    pub name: *const c_char,
    /// Commanded joint velocity in radians per second.
    pub velocity_rad_s: f64,
}

#[derive(Debug)]
struct State {
    joint: CString,
    target_rad: f64,
    gain: f64,
    max_velocity_rad_s: f64,
}

static PLUGIN_NAME_C: &[u8] = b"legacy_velocity_servo_v2\0";

/// Reports the frozen ABI-v2 version.
#[no_mangle]
pub extern "C" fn rne_plugin_abi_version() -> u32 {
    ABI_VERSION
}

/// Reports the fixture logical name.
#[no_mangle]
pub extern "C" fn rne_plugin_name() -> *const c_char {
    PLUGIN_NAME_C.as_ptr().cast()
}

/// Creates one frozen ABI-v2 velocity-servo instance.
///
/// # Safety
///
/// `joint` must point to a NUL-terminated UTF-8 string. `error` must be null or
/// point to `error_capacity` writable bytes.
#[no_mangle]
pub unsafe extern "C" fn rne_controller_create(
    joint: *const c_char,
    target_rad: f64,
    gain: f64,
    max_velocity_rad_s: f64,
    error: *mut c_char,
    error_capacity: usize,
) -> *mut c_void {
    if joint.is_null()
        || !target_rad.is_finite()
        || !gain.is_finite()
        || !max_velocity_rad_s.is_finite()
        || gain < 0.0
        || max_velocity_rad_s < 0.0
    {
        write_error(error, error_capacity, "invalid ABI-v2 create parameters");
        return std::ptr::null_mut();
    }
    // SAFETY: `joint` is a NUL-terminated string by contract.
    let joint = match unsafe { CStr::from_ptr(joint) }.to_str() {
        Ok(joint) if !joint.is_empty() => joint,
        _ => {
            write_error(error, error_capacity, "joint is not valid UTF-8");
            return std::ptr::null_mut();
        }
    };
    let Ok(joint) = CString::new(joint) else {
        write_error(error, error_capacity, "joint contains a NUL byte");
        return std::ptr::null_mut();
    };
    Box::into_raw(Box::new(State {
        joint,
        target_rad,
        gain,
        max_velocity_rad_s,
    }))
    .cast()
}

/// Destroys one frozen ABI-v2 instance.
///
/// # Safety
///
/// `handle` must be null or a live pointer returned by
/// [`rne_controller_create`] that has not already been destroyed.
#[no_mangle]
pub unsafe extern "C" fn rne_controller_destroy(handle: *mut c_void) {
    if !handle.is_null() {
        // SAFETY: `handle` is a unique live fixture instance by contract.
        drop(unsafe { Box::from_raw(handle.cast::<State>()) });
    }
}

/// Computes the frozen ABI-v2 velocity-servo command.
///
/// # Safety
///
/// `handle` must be live, `observations` must contain `observation_count`
/// entries, and `output` must contain `output_capacity` writable entries.
#[no_mangle]
pub unsafe extern "C" fn rne_controller_step(
    handle: *const c_void,
    observations: *const RneJointPosition,
    observation_count: usize,
    output: *mut RneJointVelocity,
    output_capacity: usize,
) -> usize {
    if handle.is_null() || output_capacity == 0 {
        return 0;
    }
    // SAFETY: pointers and counts are valid by contract.
    let state = unsafe { &*handle.cast::<State>() };
    let observations = unsafe { std::slice::from_raw_parts(observations, observation_count) };
    for observation in observations {
        if observation.name.is_null() {
            continue;
        }
        // SAFETY: the host supplies a NUL-terminated name.
        if unsafe { CStr::from_ptr(observation.name) }.to_bytes() != state.joint.to_bytes() {
            continue;
        }
        let velocity = (state.gain * (state.target_rad - observation.position_rad))
            .clamp(-state.max_velocity_rad_s, state.max_velocity_rad_s);
        // SAFETY: output capacity is at least one and the state-owned name
        // remains valid until instance destruction.
        unsafe {
            *output = RneJointVelocity {
                name: state.joint.as_ptr(),
                velocity_rad_s: velocity,
            }
        };
        return 1;
    }
    0
}

/// Writes one NUL-terminated error message into a caller-owned buffer.
///
/// # Safety
///
/// `error` must be null or point to `error_capacity` writable bytes.
unsafe fn write_error(error: *mut c_char, error_capacity: usize, message: &str) {
    if error.is_null() || error_capacity == 0 {
        return;
    }
    let bytes = message.as_bytes();
    let length = bytes.len().min(error_capacity - 1);
    // SAFETY: `length + 1` fits the declared caller-owned buffer.
    unsafe {
        std::ptr::copy_nonoverlapping(bytes.as_ptr().cast(), error, length);
        *error.add(length) = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fixture_remains_frozen_at_abi_v2() {
        assert_eq!(ABI_VERSION, 2);
        assert_eq!(rne_plugin_abi_version(), 2);
    }
}
