//! Frozen controller plugin implementing the RNE controller C ABI v2.
//!
//! This crate is an independently defined compatibility fixture, not the
//! current plugin-authoring example. It deliberately has no dependency on any
//! RNE crate and owns its copy of every ABI type and exported symbol. Do not
//! update this source to match a later ABI: newer runtimes must continue to
//! load it through their ABI v2 compatibility path.

#![deny(missing_docs)]

use std::ffi::{c_char, c_void, CStr, CString};

/// Frozen ABI version implemented by this fixture.
pub const ABI_VERSION: u32 = 2;

/// Logical name reported by this fixture.
pub const PLUGIN_NAME: &str = "frozen_velocity_servo_v2";

/// ABI v2 joint-position observation passed from the host.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct RneJointPosition {
    /// Joint name as a NUL-terminated UTF-8 string.
    pub name: *const c_char,
    /// Joint position in radians.
    pub position_rad: f64,
}

/// ABI v2 joint-velocity command returned to the host.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct RneJointVelocity {
    /// Joint name as a NUL-terminated UTF-8 string.
    pub name: *const c_char,
    /// Commanded joint velocity in radians per second.
    pub velocity_rad_s: f64,
}

#[derive(Debug)]
struct VelocityServoState {
    name: CString,
    target_rad: f64,
    gain: f64,
    max_velocity_rad_s: f64,
}

/// Reports the frozen ABI version implemented by this fixture.
#[no_mangle]
pub extern "C" fn rne_plugin_abi_version() -> u32 {
    ABI_VERSION
}

static PLUGIN_NAME_C: &[u8] = b"frozen_velocity_servo_v2\0";

/// Reports the fixture's logical name as a static C string.
#[no_mangle]
pub extern "C" fn rne_plugin_name() -> *const c_char {
    PLUGIN_NAME_C.as_ptr().cast()
}

/// Creates an ABI v2 velocity-servo controller instance.
///
/// Returns a non-null opaque handle on success. On failure it returns null and
/// writes a NUL-terminated message to `error` when a buffer is supplied.
///
/// # Safety
///
/// `joint` must point to a NUL-terminated string. `error` must be null or point
/// to a writable buffer containing `error_capacity` bytes.
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
        // SAFETY: The caller owns the optional error buffer under this
        // function's safety contract.
        unsafe { write_error(error, error_capacity, "joint must not be null") };
        return std::ptr::null_mut();
    }

    // SAFETY: `joint` is a valid NUL-terminated string by contract.
    let joint = match unsafe { CStr::from_ptr(joint) }.to_str() {
        Ok(text) if !text.is_empty() => text,
        Ok(_) => {
            // SAFETY: The caller owns the optional error buffer.
            unsafe { write_error(error, error_capacity, "joint must not be empty") };
            return std::ptr::null_mut();
        }
        Err(_) => {
            // SAFETY: The caller owns the optional error buffer.
            unsafe { write_error(error, error_capacity, "joint is not valid UTF-8") };
            return std::ptr::null_mut();
        }
    };

    if !target_rad.is_finite() || !gain.is_finite() || !max_velocity_rad_s.is_finite() {
        // SAFETY: The caller owns the optional error buffer.
        unsafe { write_error(error, error_capacity, "parameters must be finite") };
        return std::ptr::null_mut();
    }
    if gain < 0.0 || max_velocity_rad_s < 0.0 {
        // SAFETY: The caller owns the optional error buffer.
        unsafe {
            write_error(
                error,
                error_capacity,
                "gain and max_velocity_rad_s must be non-negative",
            )
        };
        return std::ptr::null_mut();
    }

    let name = match CString::new(joint) {
        Ok(name) => name,
        Err(_) => {
            // SAFETY: The caller owns the optional error buffer.
            unsafe { write_error(error, error_capacity, "joint contains a NUL byte") };
            return std::ptr::null_mut();
        }
    };
    Box::into_raw(Box::new(VelocityServoState {
        name,
        target_rad,
        gain,
        max_velocity_rad_s,
    }))
    .cast::<c_void>()
}

/// Destroys an ABI v2 controller instance.
///
/// # Safety
///
/// `handle` must be null or a live pointer returned by
/// [`rne_controller_create`] that has not already been destroyed.
#[no_mangle]
pub unsafe extern "C" fn rne_controller_destroy(handle: *mut c_void) {
    if handle.is_null() {
        return;
    }
    // SAFETY: The handle is uniquely owned and live by contract.
    drop(unsafe { Box::from_raw(handle.cast::<VelocityServoState>()) });
}

/// Computes at most one ABI v2 velocity command for the configured joint.
///
/// # Safety
///
/// `handle` must be a live controller instance, `observations` must contain
/// `observation_count` valid entries, and `output` must contain
/// `output_capacity` writable entries. Every name must be NUL-terminated.
#[no_mangle]
pub unsafe extern "C" fn rne_controller_step(
    handle: *const c_void,
    observations: *const RneJointPosition,
    observation_count: usize,
    output: *mut RneJointVelocity,
    output_capacity: usize,
) -> usize {
    if output_capacity == 0 {
        return 0;
    }
    // SAFETY: The handle is live and has the state type created above.
    let state = unsafe { &*handle.cast::<VelocityServoState>() };
    // SAFETY: The observation array is valid for the declared count.
    let observations = unsafe { std::slice::from_raw_parts(observations, observation_count) };

    for observation in observations {
        if observation.name.is_null() {
            continue;
        }
        // SAFETY: Each observation name is a NUL-terminated string by contract.
        if unsafe { CStr::from_ptr(observation.name) }.to_bytes() != state.name.to_bytes() {
            continue;
        }

        let velocity_rad_s = (state.gain * (state.target_rad - observation.position_rad))
            .clamp(-state.max_velocity_rad_s, state.max_velocity_rad_s);
        // SAFETY: At least one writable output entry is available. The name is
        // owned by the live state and remains valid until the host copies it.
        unsafe {
            *output = RneJointVelocity {
                name: state.name.as_ptr(),
                velocity_rad_s,
            }
        };
        return 1;
    }
    0
}

unsafe fn write_error(error: *mut c_char, error_capacity: usize, message: &str) {
    if error.is_null() || error_capacity == 0 {
        return;
    }
    let bytes = message.as_bytes();
    let length = bytes.len().min(error_capacity - 1);
    // SAFETY: The destination has `error_capacity` bytes and `length + 1`
    // never exceeds it.
    unsafe {
        std::ptr::copy_nonoverlapping(bytes.as_ptr().cast::<c_char>(), error, length);
        *error.add(length) = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn frozen_v2_exports_round_trip_a_controller_instance() {
        assert_eq!(rne_plugin_abi_version(), 2);
        // SAFETY: The exported name is a static NUL-terminated string.
        let plugin_name = unsafe { CStr::from_ptr(rne_plugin_name()) };
        assert_eq!(plugin_name.to_bytes(), PLUGIN_NAME.as_bytes());

        let joint = CString::new("shoulder_joint").expect("joint name");
        let mut error = [0_i8; 128];
        // SAFETY: The joint and error pointers satisfy the ABI v2 create
        // contract for the duration of this call.
        let handle = unsafe {
            rne_controller_create(
                joint.as_ptr(),
                1.0,
                2.0,
                5.0,
                error.as_mut_ptr(),
                error.len(),
            )
        };
        assert!(!handle.is_null());

        let observation = RneJointPosition {
            name: joint.as_ptr(),
            position_rad: 0.25,
        };
        let mut output = RneJointVelocity {
            name: std::ptr::null(),
            velocity_rad_s: 0.0,
        };
        // SAFETY: The live handle and one-element input/output arrays satisfy
        // the ABI v2 step contract.
        let count = unsafe { rne_controller_step(handle, &observation, 1, &mut output, 1) };
        assert_eq!(count, 1);
        assert_eq!(output.velocity_rad_s, 1.5);
        // SAFETY: The output name is owned by the still-live controller state.
        let output_name = unsafe { CStr::from_ptr(output.name) };
        assert_eq!(output_name.to_bytes(), b"shoulder_joint");

        // SAFETY: The handle is live and is destroyed exactly once.
        unsafe { rne_controller_destroy(handle) };
    }
}
