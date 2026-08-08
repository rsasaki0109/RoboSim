//! Versioned C ABI for dynamically loaded controller plugins.
//!
//! A plugin is a shared library exposing the symbols in this module. The host
//! ([`load_controller_library`]) opens the library, checks
//! [`RNE_PLUGIN_ABI_VERSION`], creates a controller instance, and calls its
//! step function each fixed step. All data crosses the boundary as plain
//! `#[repr(C)]` values and NUL-terminated UTF-8 strings; no Rust types or
//! allocator reach the plugin.
//!
//! The ABI is stable: symbol signatures and struct layouts are versioned by
//! [`RNE_PLUGIN_ABI_VERSION`], and a plugin whose version differs is rejected
//! at load time. The host copies every string returned by the plugin before
//! control returns to it.

use std::ffi::{c_char, c_void, CStr, CString};
use std::path::Path;

/// ABI version negotiated at load time.
///
/// Bump this whenever a symbol signature or a `#[repr(C)]` struct layout in
/// this module changes. Plugins reporting a different version are rejected.
pub const RNE_PLUGIN_ABI_VERSION: u32 = 1;

/// A joint position observation passed from the host to a plugin.
///
/// `name` is a NUL-terminated UTF-8 string owned by the host and valid for the
/// duration of the `rne_controller_step` call.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct RneJointPosition {
    /// Joint name as a NUL-terminated UTF-8 string.
    pub name: *const c_char,
    /// Joint position in radians.
    pub position_rad: f64,
}

/// A joint velocity command returned by a plugin to the host.
///
/// `name` is a NUL-terminated UTF-8 string that must stay valid until the host
/// copies it (for example a static string or one owned by the plugin's state).
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct RneJointVelocity {
    /// Joint name as a NUL-terminated UTF-8 string.
    pub name: *const c_char,
    /// Commanded joint velocity in radians per second.
    pub velocity_rad_s: f64,
}

/// Reports the [`RNE_PLUGIN_ABI_VERSION`] the plugin was built against.
pub type RnePluginAbiVersionFn = unsafe extern "C" fn() -> u32;

/// Creates a controller instance.
///
/// Returns a non-null opaque handle on success, or null after writing a
/// message into `error` (if non-null with capacity) on failure.
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

/// Computes joint velocity commands for the current observations.
///
/// Returns the number of velocity commands written into `output`, which has
/// `output_capacity` entries; the count is capped by the caller.
pub type RneControllerStepFn = unsafe extern "C" fn(
    handle: *const c_void,
    observations: *const RneJointPosition,
    observation_count: usize,
    output: *mut RneJointVelocity,
    output_capacity: usize,
) -> usize;

/// A controller plugin loaded from a shared library.
///
/// This wraps the loaded symbols and the opaque plugin handle behind the
/// [`crate::ControllerPlugin`] trait. It is safe to use from any thread as long
/// as the loaded plugin is reentrant, which the ABI contract requires; the
/// runner always calls it from a single thread.
#[derive(Debug)]
pub struct LoadedControllerPlugin {
    library: libloading::Library,
    destroy: RneControllerDestroyFn,
    step: RneControllerStepFn,
    handle: *mut c_void,
    name: String,
}

// SAFETY: The ABI contract requires plugins to be reentrant and free of
// thread-local state (the runner calls step from a single thread but the
// controller may be shared). The raw handle points to heap state owned by the
// plugin whose functions are invoked while `library` is alive, guarded by the
// `Drop` impl. See the module-level ABI documentation.
unsafe impl Send for LoadedControllerPlugin {}
// SAFETY: Same contract as `Send`; the plugin must tolerate concurrent reads of
// distinct instances and the runner never calls the same instance in parallel.
unsafe impl Sync for LoadedControllerPlugin {}

impl LoadedControllerPlugin {
    /// Loads a controller plugin from a shared library at `library_path`.
    ///
    /// Resolves and checks the ABI version, creates a controller instance with
    /// the velocity-servo parameters, and returns it behind the
    /// [`crate::ControllerPlugin`] boundary.
    pub fn load(
        library_path: &Path,
        joint: &str,
        target_rad: f64,
        gain: f64,
        max_velocity_rad_s: f64,
    ) -> Result<Self, PluginLoadError> {
        let library = unsafe { libloading::Library::new(library_path) }.map_err(|error| {
            PluginLoadError::Open {
                path: library_path.display().to_string(),
                message: error.to_string(),
            }
        })?;

        let abi_version_symbol: libloading::Symbol<'_, RnePluginAbiVersionFn> = unsafe {
            library.get(b"rne_plugin_abi_version")
        }
        .map_err(|error| PluginLoadError::Symbol {
            path: library_path.display().to_string(),
            symbol: "rne_plugin_abi_version",
            message: error.to_string(),
        })?;
        let abi_version = unsafe { abi_version_symbol() };
        if abi_version != RNE_PLUGIN_ABI_VERSION {
            return Err(PluginLoadError::AbiVersion {
                got: abi_version,
                expected: RNE_PLUGIN_ABI_VERSION,
            });
        }

        let create_symbol: libloading::Symbol<'_, RneControllerCreateFn> = unsafe {
            library.get(b"rne_controller_create")
        }
        .map_err(|error| PluginLoadError::Symbol {
            path: library_path.display().to_string(),
            symbol: "rne_controller_create",
            message: error.to_string(),
        })?;
        let destroy_symbol: libloading::Symbol<'_, RneControllerDestroyFn> = unsafe {
            library.get(b"rne_controller_destroy")
        }
        .map_err(|error| PluginLoadError::Symbol {
            path: library_path.display().to_string(),
            symbol: "rne_controller_destroy",
            message: error.to_string(),
        })?;
        let step_symbol: libloading::Symbol<'_, RneControllerStepFn> = unsafe {
            library.get(b"rne_controller_step")
        }
        .map_err(|error| PluginLoadError::Symbol {
            path: library_path.display().to_string(),
            symbol: "rne_controller_step",
            message: error.to_string(),
        })?;

        let create: RneControllerCreateFn = *create_symbol;
        let destroy: RneControllerDestroyFn = *destroy_symbol;
        let step: RneControllerStepFn = *step_symbol;

        let joint_c = CString::new(joint).map_err(|error| PluginLoadError::Create {
            message: format!("joint is not NUL-free UTF-8: {error}"),
        })?;
        let mut error_buffer = vec![0_u8; 256];
        let handle = unsafe {
            create(
                joint_c.as_ptr(),
                target_rad,
                gain,
                max_velocity_rad_s,
                error_buffer.as_mut_ptr().cast(),
                error_buffer.len(),
            )
        };
        if handle.is_null() {
            let message = CStr::from_bytes_until_nul(&error_buffer)
                .map(|cstr| cstr.to_string_lossy().into_owned())
                .unwrap_or_default();
            let message = if message.trim().is_empty() {
                "unknown plugin create failure".to_string()
            } else {
                message
            };
            return Err(PluginLoadError::Create { message });
        }

        let name = library_path
            .file_stem()
            .map(|stem| stem.to_string_lossy().into_owned())
            .unwrap_or_else(|| "loaded_plugin".to_string());

        Ok(Self {
            library,
            destroy,
            step,
            handle,
            name,
        })
    }
}

impl Drop for LoadedControllerPlugin {
    fn drop(&mut self) {
        // SAFETY: `handle` was returned by the plugin's create function, which
        // paired with `destroy`. `self.library` is referenced here so it stays
        // alive (and the destroy symbol loaded) while the handle is destroyed.
        let _library_alive = &self.library;
        unsafe { (self.destroy)(self.handle) }
    }
}

impl crate::ControllerPlugin for LoadedControllerPlugin {
    fn name(&self) -> &str {
        &self.name
    }

    fn joint_velocity_commands(
        &self,
        joint_names: &[&str],
        positions_rad: &[f64],
    ) -> Vec<(String, f64)> {
        let names_c: Vec<CString> = joint_names
            .iter()
            .map(|name| CString::new(*name).unwrap_or_default())
            .collect();
        let observations: Vec<RneJointPosition> = names_c
            .iter()
            .zip(positions_rad)
            .map(|(name, position_rad)| RneJointPosition {
                name: name.as_ptr(),
                position_rad: *position_rad,
            })
            .collect();
        let mut output = vec![
            RneJointVelocity {
                name: std::ptr::null(),
                velocity_rad_s: 0.0,
            };
            joint_names.len()
        ];
        // SAFETY: all pointers point to valid C strings/arrays with the declared
        // lengths, `output` has `joint_names.len()` entries, and the plugin
        // writes at most `output_capacity` entries (capped again below).
        let count = unsafe {
            (self.step)(
                self.handle,
                observations.as_ptr(),
                observations.len(),
                output.as_mut_ptr(),
                output.len(),
            )
        };
        let count = count.min(output.len());
        output[..count]
            .iter()
            .map(|command| {
                // SAFETY: the ABI requires the returned name pointer to stay
                // valid until the host copies it.
                let name = unsafe { CStr::from_ptr(command.name) }
                    .to_string_lossy()
                    .into_owned();
                (name, command.velocity_rad_s)
            })
            .collect()
    }
}

/// Loads a controller plugin from a shared library and returns it behind the
/// [`crate::ControllerPlugin`] boundary.
pub fn load_controller_library(
    library_path: &Path,
    joint: &str,
    target_rad: f64,
    gain: f64,
    max_velocity_rad_s: f64,
) -> Result<Box<dyn crate::ControllerPlugin>, PluginLoadError> {
    Ok(Box::new(LoadedControllerPlugin::load(
        library_path,
        joint,
        target_rad,
        gain,
        max_velocity_rad_s,
    )?))
}

/// A failure while opening, resolving, or invoking a plugin library.
#[derive(Debug, thiserror::Error)]
pub enum PluginLoadError {
    /// The shared library could not be opened.
    #[error("open plugin library {path}: {message}")]
    Open {
        /// Library path.
        path: String,
        /// Underlying loader error.
        message: String,
    },
    /// A required ABI symbol is missing.
    #[error("plugin library {path} is missing symbol `{symbol}`: {message}")]
    Symbol {
        /// Library path.
        path: String,
        /// Missing symbol name.
        symbol: &'static str,
        /// Underlying loader error.
        message: String,
    },
    /// The plugin reports an incompatible ABI version.
    #[error("plugin ABI version mismatch: plugin reports {got}, host requires {expected}")]
    AbiVersion {
        /// Version the plugin reports.
        got: u32,
        /// Version the host requires.
        expected: u32,
    },
    /// Creating the controller instance failed.
    #[error("plugin controller create failed: {message}")]
    Create {
        /// Error message reported by the plugin.
        message: String,
    },
}
