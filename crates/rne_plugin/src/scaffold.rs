//! Plugin authoring scaffolds.
//!
//! [`scaffold_controller_plugin`] generates a complete, compilable controller
//! plugin crate (a `cdylib` implementing the versioned controller-plugin C ABI
//! from [`crate::cabi`]) plus a [`crate::PluginManifest`], so third-party
//! authors can start from a working shared library instead of a blank page.

use crate::PluginManifest;
use std::fs;
use std::path::{Path, PathBuf};

/// Scaffold validation or I/O failure.
#[derive(Debug, thiserror::Error)]
pub enum ScaffoldError {
    /// The plugin name is not a valid identifier.
    #[error("invalid plugin name `{name}`: use ASCII letters, digits, and underscores")]
    InvalidName {
        /// Requested plugin name.
        name: String,
    },
    /// A scaffold file could not be written.
    #[error("write plugin scaffold {path}: {message}")]
    Write {
        /// File path.
        path: String,
        /// Underlying I/O error.
        message: String,
    },
    /// The plugin directory already exists.
    #[error("plugin directory {path} already exists")]
    Exists {
        /// Existing directory path.
        path: String,
    },
    /// The scaffold manifest could not be serialized.
    #[error("serialize plugin manifest: {0}")]
    Manifest(#[from] crate::PluginError),
}

/// Validates a plugin name for use as a crate name and ABI name.
pub fn validate_plugin_name(name: &str) -> Result<(), ScaffoldError> {
    if !name.is_empty()
        && name
            .chars()
            .all(|character| character.is_ascii_alphanumeric() || character == '_')
    {
        Ok(())
    } else {
        Err(ScaffoldError::InvalidName {
            name: name.to_string(),
        })
    }
}

/// Generates a compilable controller-plugin crate under `parent_dir/<name>`.
///
/// Creates `Cargo.toml`, `src/lib.rs` (a `cdylib` implementing the
/// controller-plugin C ABI, initially a velocity-servo policy), and
/// `rne-plugin.json` (a versioned [`crate::PluginManifest`]). Returns the
/// created crate directory. Errors if the directory already exists.
pub fn scaffold_controller_plugin(name: &str, parent_dir: &Path) -> Result<PathBuf, ScaffoldError> {
    validate_plugin_name(name)?;
    let crate_dir = parent_dir.join(name);
    if crate_dir.exists() {
        return Err(ScaffoldError::Exists {
            path: crate_dir.display().to_string(),
        });
    }
    fs::create_dir_all(crate_dir.join("src")).map_err(|error| ScaffoldError::Write {
        path: crate_dir.display().to_string(),
        message: error.to_string(),
    })?;
    write_scaffold_file(&crate_dir.join("Cargo.toml"), &cargo_manifest(name))?;
    write_scaffold_file(&crate_dir.join("src/lib.rs"), &lib_source(name))?;
    let manifest = PluginManifest::controller(name);
    let manifest_json = format!("{}\n", manifest.to_json()?);
    write_scaffold_file(&crate_dir.join("rne-plugin.json"), &manifest_json)?;
    Ok(crate_dir)
}

fn write_scaffold_file(path: &Path, contents: &str) -> Result<(), ScaffoldError> {
    fs::write(path, contents).map_err(|error| ScaffoldError::Write {
        path: path.display().to_string(),
        message: error.to_string(),
    })
}

fn cargo_manifest(name: &str) -> String {
    format!(
        r#"[package]
name = "{name}"
description = "RNE controller plugin scaffold"
version = "0.1.0"
edition = "2021"
license = "MIT OR Apache-2.0"

[lib]
crate-type = ["cdylib", "lib"]

# Standalone workspace root so `cargo build` here never resolves against an
# enclosing workspace.
[workspace]
"#
    )
}

fn lib_source(name: &str) -> String {
    format!(
        r#"//! RNE controller plugin `{name}`.
//!
//! This scaffold compiles to a shared library implementing the versioned
//! controller-plugin C ABI (`rne_plugin::cabi`, ABI version 3). Replace the
//! velocity-servo policy in `rne_controller_step_v3` with your controller.

use std::ffi::{{c_char, c_void, CStr, CString}};

/// ABI version implemented by this plugin.
pub const ABI_VERSION: u32 = 3;

/// Logical plugin name reported through [`rne_plugin_name`].
pub const PLUGIN_NAME: &str = "{name}";

const CAP_JOINT_POSITION_OBSERVATION: u64 = 1 << 0;
const CAP_JOINT_VELOCITY_COMMAND: u64 = 1 << 2;
const CAP_MULTI_ROBOT: u64 = 1 << 3;
const CAPABILITIES: u64 =
    CAP_JOINT_POSITION_OBSERVATION | CAP_JOINT_VELOCITY_COMMAND | CAP_MULTI_ROBOT;

/// Joint position observation.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct RneJointPosition {{
    /// Joint name as a NUL-terminated UTF-8 string.
    pub name: *const c_char,
    /// Joint position in radians.
    pub position_rad: f64,
}}

/// Joint velocity command.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct RneJointVelocity {{
    /// Joint name as a NUL-terminated UTF-8 string.
    pub name: *const c_char,
    /// Commanded joint velocity in radians per second.
    pub velocity_rad_s: f64,
}}

/// Robot-scoped ABI-v3 joint observation.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct RneJointObservationV3 {{
    pub robot_id: *const c_char,
    pub name: *const c_char,
    pub position_rad: f64,
    pub velocity_rad_s: f64,
    pub has_velocity: u8,
    pub reserved: [u8; 7],
}}

/// Robot-scoped ABI-v3 joint velocity command.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct RneJointVelocityV3 {{
    pub robot_id: *const c_char,
    pub name: *const c_char,
    pub velocity_rad_s: f64,
}}

/// ABI-v3 fixed-step result.
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RneControllerStepResultV3 {{
    pub status: i32,
    pub output_count: usize,
}}

/// Controller state owned by the plugin.
struct ControllerState {{
    name: CString,
    target_rad: f64,
    gain: f64,
    max_velocity_rad_s: f64,
    configured: bool,
    active: bool,
    shutdown: bool,
}}

static PLUGIN_NAME_C: &[u8] = b"{name}\0";

/// Reports the ABI version this plugin was built against.
#[no_mangle]
pub extern "C" fn rne_plugin_abi_version() -> u32 {{
    ABI_VERSION
}}

/// Reports the plugin's logical name as a static NUL-terminated UTF-8 string.
#[no_mangle]
pub extern "C" fn rne_plugin_name() -> *const c_char {{
    PLUGIN_NAME_C.as_ptr().cast()
}}

/// Reports the supported ABI-v3 capability mask.
#[no_mangle]
pub extern "C" fn rne_plugin_capabilities() -> u64 {{
    CAPABILITIES
}}

/// Creates a controller instance.
///
/// Returns a non-null opaque handle on success, or null after writing a
/// message into `error` (if non-null with capacity) on failure.
///
/// # Safety
///
/// `joint` must be null or point to a NUL-terminated UTF-8 string. `error`
/// must be null or point to a buffer of `error_capacity` bytes.
#[no_mangle]
pub unsafe extern "C" fn rne_controller_create(
    joint: *const c_char,
    target_rad: f64,
    gain: f64,
    max_velocity_rad_s: f64,
    error: *mut c_char,
    error_capacity: usize,
) -> *mut c_void {{
    if joint.is_null() {{
        write_error(error, error_capacity, "joint must not be null");
        return std::ptr::null_mut();
    }}
    let Ok(joint) = (|| -> Result<String, ()> {{
        // SAFETY: `joint` is a valid NUL-terminated string by contract.
        let joint = unsafe {{ CStr::from_ptr(joint) }}.to_str().map_err(|_| ())?;
        if joint.is_empty() {{
            return Err(());
        }}
        if !target_rad.is_finite() || !gain.is_finite() || !max_velocity_rad_s.is_finite() {{
            return Err(());
        }}
        if gain < 0.0 || max_velocity_rad_s < 0.0 {{
            return Err(());
        }}
        Ok(joint.to_string())
    }})() else {{
        write_error(error, error_capacity, "invalid create parameters");
        return std::ptr::null_mut();
    }};
    let name = match CString::new(joint) {{
        Ok(name) => name,
        Err(_) => {{
            write_error(error, error_capacity, "joint contains a NUL byte");
            return std::ptr::null_mut();
        }}
    }};
    let state = Box::new(ControllerState {{
        name,
        target_rad,
        gain,
        max_velocity_rad_s,
        configured: false,
        active: false,
        shutdown: false,
    }});
    Box::into_raw(state).cast::<c_void>()
}}

/// Destroys a controller instance created by `rne_controller_create`.
///
/// # Safety
///
/// `handle` must be a non-null pointer returned by `rne_controller_create`
/// and not already destroyed.
#[no_mangle]
pub unsafe extern "C" fn rne_controller_destroy(handle: *mut c_void) {{
    if handle.is_null() {{
        return;
    }}
    // SAFETY: `handle` is a unique live instance by contract.
    drop(unsafe {{ Box::from_raw(handle.cast::<ControllerState>()) }});
}}

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
) -> usize {{
    if output_capacity == 0 {{
        return 0;
    }}
    // SAFETY: the handle and input array satisfy the callback contract.
    let state = unsafe {{ &*handle.cast::<ControllerState>() }};
    let observations = unsafe {{ std::slice::from_raw_parts(observations, observation_count) }};
    for observation in observations {{
        if observation.name.is_null() {{
            continue;
        }}
        // SAFETY: observation names are NUL-terminated by contract.
        if unsafe {{ CStr::from_ptr(observation.name) }}.to_bytes() != state.name.to_bytes() {{
            continue;
        }}
        let velocity = (state.gain * (state.target_rad - observation.position_rad))
            .clamp(-state.max_velocity_rad_s, state.max_velocity_rad_s);
        // SAFETY: output capacity is non-zero and the state-owned name remains
        // valid until instance destruction.
        unsafe {{
            *output = RneJointVelocity {{
                name: state.name.as_ptr(),
                velocity_rad_s: velocity,
            }}
        }};
        return 1;
    }}
    0
}}

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
) -> i32 {{
    // SAFETY: `handle` is a live instance by contract.
    let state = unsafe {{ &mut *handle.cast::<ControllerState>() }};
    if state.shutdown {{
        write_error(error, error_capacity, "controller is shut down");
        return 1;
    }}
    if required_capabilities & !CAPABILITIES != 0 {{
        write_error(error, error_capacity, "unsupported required capability");
        return 1;
    }}
    state.configured = true;
    0
}}

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
) -> i32 {{
    // SAFETY: `handle` is a live instance by contract.
    let state = unsafe {{ &mut *handle.cast::<ControllerState>() }};
    if !state.configured || state.shutdown {{
        write_error(
            error,
            error_capacity,
            "controller must be configured and not shut down",
        );
        return 1;
    }}
    state.active = true;
    0
}}

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
) -> RneControllerStepResultV3 {{
    // SAFETY: `handle` is a live instance by contract.
    let state = unsafe {{ &mut *handle.cast::<ControllerState>() }};
    if !state.active || state.shutdown {{
        write_error(error, error_capacity, "controller is not active");
        return RneControllerStepResultV3 {{
            status: 1,
            output_count: 0,
        }};
    }}
    // SAFETY: the host supplies an array with the declared length.
    let observations = unsafe {{ std::slice::from_raw_parts(observations, observation_count) }};
    let mut output_count = 0;
    for observation in observations {{
        if observation.robot_id.is_null() || observation.name.is_null() {{
            continue;
        }}
        // SAFETY: observation names are NUL-terminated by contract.
        if unsafe {{ CStr::from_ptr(observation.name) }}.to_bytes() != state.name.to_bytes() {{
            continue;
        }}
        if output_count >= output_capacity {{
            write_error(error, error_capacity, "output capacity is too small");
            return RneControllerStepResultV3 {{
                status: 1,
                output_count: 0,
            }};
        }}
        let velocity = (state.gain * (state.target_rad - observation.position_rad))
            .clamp(-state.max_velocity_rad_s, state.max_velocity_rad_s);
        // SAFETY: `output_count < output_capacity`; both pointers stay valid
        // until the host copies the result immediately after this call.
        unsafe {{
            *output.add(output_count) = RneJointVelocityV3 {{
                robot_id: observation.robot_id,
                name: state.name.as_ptr(),
                velocity_rad_s: velocity,
            }}
        }};
        output_count += 1;
    }}
    RneControllerStepResultV3 {{
        status: 0,
        output_count,
    }}
}}

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
) -> i32 {{
    // SAFETY: `handle` is a live instance by contract.
    let state = unsafe {{ &mut *handle.cast::<ControllerState>() }};
    if state.shutdown {{
        write_error(error, error_capacity, "controller is already shut down");
        return 1;
    }}
    state.active = false;
    state.shutdown = true;
    0
}}

/// Writes a NUL-terminated message into `error` when it has capacity.
///
/// # Safety
///
/// `error` must be null or point to a buffer of `error_capacity` bytes.
unsafe fn write_error(error: *mut c_char, error_capacity: usize, message: &str) {{
    if error.is_null() || error_capacity == 0 {{
        return;
    }}
    let bytes = message.as_bytes();
    let length = bytes.len().min(error_capacity - 1);
    // SAFETY: `length + 1` fits the declared caller-owned buffer.
    unsafe {{
        std::ptr::copy_nonoverlapping(bytes.as_ptr().cast::<c_char>(), error, length);
        *error.add(length) = 0;
    }}
}}
"#
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::fs;

    #[test]
    fn scaffolds_a_compilable_crate() {
        let parent = std::env::temp_dir().join("rne-scaffold-test");
        let _ = fs::remove_dir_all(&parent);
        let name = "my_controller";
        let crate_dir = scaffold_controller_plugin(name, &parent).expect("scaffold");

        assert!(crate_dir.join("Cargo.toml").exists());
        assert!(crate_dir.join("src/lib.rs").exists());
        let manifest_path = crate_dir.join("rne-plugin.json");
        assert!(manifest_path.exists());
        let manifest_text = fs::read_to_string(&manifest_path).expect("read manifest");
        let manifest: crate::PluginManifest =
            serde_json::from_str(&manifest_text).expect("parse manifest");
        assert_eq!(manifest.name, name);

        let lib = fs::read_to_string(crate_dir.join("src/lib.rs")).expect("read lib");
        for symbol in [
            "rne_plugin_abi_version",
            "rne_plugin_name",
            "rne_plugin_capabilities",
            "rne_controller_create",
            "rne_controller_destroy",
            "rne_controller_step",
            "rne_controller_configure_v3",
            "rne_controller_reset_v3",
            "rne_controller_step_v3",
            "rne_controller_shutdown_v3",
        ] {
            assert!(lib.contains(symbol), "lib.rs must export `{symbol}`");
        }
        assert!(lib.contains("pub const ABI_VERSION: u32 = 3;"));
        assert!(lib.contains(&format!("pub const PLUGIN_NAME: &str = \"{name}\";")));

        let output_dir = parent.join("rustc-output");
        fs::create_dir_all(&output_dir).expect("create rustc output directory");
        let rustc = std::env::var_os("RUSTC").unwrap_or_else(|| "rustc".into());
        let status = std::process::Command::new(rustc)
            .arg("--edition=2021")
            .arg("--crate-type=lib")
            .arg("--out-dir")
            .arg(&output_dir)
            .arg(crate_dir.join("src/lib.rs"))
            .status()
            .expect("run rustc on generated plugin source");
        assert!(status.success(), "generated plugin source must compile");

        let _ = fs::remove_dir_all(&parent);
    }

    #[test]
    fn rejects_invalid_names_and_existing_directories() {
        assert!(matches!(
            validate_plugin_name(""),
            Err(ScaffoldError::InvalidName { .. })
        ));
        assert!(matches!(
            validate_plugin_name("my-plugin"),
            Err(ScaffoldError::InvalidName { .. })
        ));
        assert!(matches!(
            validate_plugin_name("a b"),
            Err(ScaffoldError::InvalidName { .. })
        ));
        assert!(validate_plugin_name("velocity_servo").is_ok());

        let parent = std::env::temp_dir().join("rne-scaffold-exists");
        let _ = fs::remove_dir_all(&parent);
        scaffold_controller_plugin("exists", &parent).expect("scaffold");
        assert!(matches!(
            scaffold_controller_plugin("exists", &parent),
            Err(ScaffoldError::Exists { .. })
        ));
        let _ = fs::remove_dir_all(&parent);
    }
}
