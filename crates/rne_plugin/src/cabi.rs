//! Backward-compatible C ABI for dynamically loaded controller plugins.
//!
//! ABI v2 is the oldest supported contract and retains the original flat
//! joint-position to joint-velocity callback. ABI v3 adds capability
//! negotiation, deterministic lifecycle hooks, and robot-scoped fixed-step
//! frames. The loader dispatches by the plugin-reported version; no Rust type
//! or allocator crosses the shared-library boundary.
//!
//! A loaded instance may move between host threads, but the host serializes
//! every callback for that instance. Plugins must not attach a controller
//! handle to the thread that created it.

use crate::{
    ControllerActionFrame, ControllerCapability, ControllerJointVelocityCommand,
    ControllerNegotiation, ControllerObservationFrame, ControllerPluginError,
    ControllerResetContext, ControllerRobotAction,
};
use std::collections::BTreeMap;
use std::ffi::{c_char, c_void, CStr, CString};
use std::path::{Path, PathBuf};
use std::sync::Mutex;

/// Oldest controller-plugin ABI accepted by this runtime.
pub const RNE_PLUGIN_MIN_ABI_VERSION: u32 = 2;
/// Current controller-plugin ABI emitted by examples and scaffolds.
pub const RNE_PLUGIN_ABI_VERSION: u32 = 3;
/// Original flat joint controller ABI retained for compatibility fixtures.
pub const RNE_PLUGIN_ABI_VERSION_V2: u32 = 2;

const CAP_JOINT_POSITION_OBSERVATION: u64 = 1 << 0;
const CAP_JOINT_VELOCITY_OBSERVATION: u64 = 1 << 1;
const CAP_JOINT_VELOCITY_COMMAND: u64 = 1 << 2;
const CAP_MULTI_ROBOT: u64 = 1 << 3;
const KNOWN_CAPABILITY_MASK: u64 = CAP_JOINT_POSITION_OBSERVATION
    | CAP_JOINT_VELOCITY_OBSERVATION
    | CAP_JOINT_VELOCITY_COMMAND
    | CAP_MULTI_ROBOT;

/// Converts a controller capability to its stable ABI-v3 bit.
pub const fn controller_capability_bit(capability: ControllerCapability) -> u64 {
    match capability {
        ControllerCapability::JointPositionObservation => CAP_JOINT_POSITION_OBSERVATION,
        ControllerCapability::JointVelocityObservation => CAP_JOINT_VELOCITY_OBSERVATION,
        ControllerCapability::JointVelocityCommand => CAP_JOINT_VELOCITY_COMMAND,
        ControllerCapability::MultiRobot => CAP_MULTI_ROBOT,
    }
}

/// A joint position observation used by the ABI-v2 callback.
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

/// A joint velocity command returned by the ABI-v2 callback.
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

/// Robot-scoped joint velocity command returned by an ABI-v3 controller.
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

/// Computes flat joint velocity commands through the ABI-v2 callback.
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

#[derive(Debug)]
enum LoadedControllerAbi {
    V2 {
        step: RneControllerStepFn,
    },
    V3 {
        legacy_step: RneControllerStepFn,
        capabilities: Vec<ControllerCapability>,
        configure: RneControllerConfigureV3Fn,
        reset: RneControllerResetV3Fn,
        step: RneControllerStepV3Fn,
        shutdown: RneControllerShutdownV3Fn,
    },
}

/// A controller plugin loaded from a shared library.
#[derive(Debug)]
pub struct LoadedControllerPlugin {
    library: libloading::Library,
    destroy: RneControllerDestroyFn,
    handle: *mut c_void,
    name: String,
    abi_version: u32,
    abi: LoadedControllerAbi,
    call_lock: Mutex<()>,
}

// SAFETY: The ABI contract permits an instance to move between host threads,
// `call_lock` serializes every callback, and `library` outlives the function
// pointers and opaque handle stored beside it.
unsafe impl Send for LoadedControllerPlugin {}
// SAFETY: Shared legacy calls also acquire `call_lock`, so no two threads can
// enter plugin-owned state concurrently. Mutable lifecycle calls are covered
// by the same lock and already require `&mut self` at the Rust boundary.
unsafe impl Sync for LoadedControllerPlugin {}

impl LoadedControllerPlugin {
    /// Loads and creates a controller plugin from one supported ABI version.
    pub fn load(
        library_path: &Path,
        joint: &str,
        target_rad: f64,
        gain: f64,
        max_velocity_rad_s: f64,
    ) -> Result<Self, PluginLoadError> {
        // SAFETY: Loading arbitrary code is the purpose of this API. Symbols
        // are validated below and the `Library` remains owned by the result.
        let library = unsafe { libloading::Library::new(library_path) }.map_err(|error| {
            PluginLoadError::Open {
                path: library_path.display().to_string(),
                message: error.to_string(),
            }
        })?;

        let abi_version_fn: RnePluginAbiVersionFn = copy_symbol(
            &library,
            library_path,
            b"rne_plugin_abi_version",
            "rne_plugin_abi_version",
        )?;
        // SAFETY: `copy_symbol` resolved the required zero-argument ABI symbol.
        let abi_version = unsafe { abi_version_fn() };
        if !(RNE_PLUGIN_MIN_ABI_VERSION..=RNE_PLUGIN_ABI_VERSION).contains(&abi_version) {
            return Err(PluginLoadError::AbiVersion {
                got: abi_version,
                minimum: RNE_PLUGIN_MIN_ABI_VERSION,
                maximum: RNE_PLUGIN_ABI_VERSION,
            });
        }

        let create: RneControllerCreateFn = copy_symbol(
            &library,
            library_path,
            b"rne_controller_create",
            "rne_controller_create",
        )?;
        let destroy: RneControllerDestroyFn = copy_symbol(
            &library,
            library_path,
            b"rne_controller_destroy",
            "rne_controller_destroy",
        )?;
        let legacy_step: RneControllerStepFn = copy_symbol(
            &library,
            library_path,
            b"rne_controller_step",
            "rne_controller_step",
        )?;
        let name_fn: RnePluginNameFn = copy_symbol(
            &library,
            library_path,
            b"rne_plugin_name",
            "rne_plugin_name",
        )?;
        // SAFETY: `copy_symbol` resolved the required metadata function.
        let name_pointer = unsafe { name_fn() };
        if name_pointer.is_null() {
            return Err(PluginLoadError::InvalidMetadata(
                "rne_plugin_name returned null".to_string(),
            ));
        }
        // SAFETY: Non-null static NUL-terminated UTF-8 is required by the ABI;
        // UTF-8 is checked immediately below.
        let name = unsafe { CStr::from_ptr(name_pointer) }
            .to_str()
            .map_err(|error| {
                PluginLoadError::InvalidMetadata(format!(
                    "rne_plugin_name is not valid UTF-8: {error}"
                ))
            })?
            .to_string();
        if name.trim().is_empty() {
            return Err(PluginLoadError::InvalidMetadata(
                "rne_plugin_name must not be empty".to_string(),
            ));
        }

        let abi = if abi_version == RNE_PLUGIN_ABI_VERSION_V2 {
            LoadedControllerAbi::V2 { step: legacy_step }
        } else {
            let capabilities_fn: RnePluginCapabilitiesFn = copy_symbol(
                &library,
                library_path,
                b"rne_plugin_capabilities",
                "rne_plugin_capabilities",
            )?;
            // SAFETY: ABI v3 requires this resolved zero-argument symbol.
            let capability_mask = unsafe { capabilities_fn() };
            let capabilities = decode_capability_mask(capability_mask)?;
            let configure = copy_symbol(
                &library,
                library_path,
                b"rne_controller_configure_v3",
                "rne_controller_configure_v3",
            )?;
            let reset = copy_symbol(
                &library,
                library_path,
                b"rne_controller_reset_v3",
                "rne_controller_reset_v3",
            )?;
            let step = copy_symbol(
                &library,
                library_path,
                b"rne_controller_step_v3",
                "rne_controller_step_v3",
            )?;
            let shutdown = copy_symbol(
                &library,
                library_path,
                b"rne_controller_shutdown_v3",
                "rne_controller_shutdown_v3",
            )?;
            LoadedControllerAbi::V3 {
                legacy_step,
                capabilities,
                configure,
                reset,
                step,
                shutdown,
            }
        };

        let joint_c = CString::new(joint).map_err(|error| PluginLoadError::Create {
            message: format!("joint is not NUL-free UTF-8: {error}"),
        })?;
        let mut error_buffer = vec![0_u8; 512];
        // SAFETY: Arguments follow the create contract, and all borrowed
        // buffers outlive the call. The returned handle is checked for null.
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
            return Err(PluginLoadError::Create {
                message: read_error(&error_buffer, "unknown plugin create failure"),
            });
        }

        Ok(Self {
            library,
            destroy,
            handle,
            name,
            abi_version,
            abi,
            call_lock: Mutex::new(()),
        })
    }

    /// Returns the ABI version reported by the loaded plugin.
    pub fn abi_version(&self) -> u32 {
        self.abi_version
    }

    fn legacy_step(&self) -> RneControllerStepFn {
        match &self.abi {
            LoadedControllerAbi::V2 { step } => *step,
            LoadedControllerAbi::V3 { legacy_step, .. } => *legacy_step,
        }
    }

    fn legacy_commands(&self, joint_names: &[&str], positions_rad: &[f64]) -> Vec<(String, f64)> {
        let Ok(_call) = self.call_lock.lock() else {
            return Vec::new();
        };
        let inputs = joint_names
            .iter()
            .zip(positions_rad)
            .filter_map(|(name, position_rad)| {
                CString::new(*name).ok().map(|name| (name, *position_rad))
            })
            .collect::<Vec<_>>();
        let observations = inputs
            .iter()
            .map(|(name, position_rad)| RneJointPosition {
                name: name.as_ptr(),
                position_rad: *position_rad,
            })
            .collect::<Vec<_>>();
        let mut output = vec![
            RneJointVelocity {
                name: std::ptr::null(),
                velocity_rad_s: 0.0,
            };
            observations.len()
        ];
        // SAFETY: Input/output slices own the declared initialized buffers for
        // the call, the opaque handle is live, and `_call` excludes races.
        let count = unsafe {
            (self.legacy_step())(
                self.handle,
                observations.as_ptr(),
                observations.len(),
                output.as_mut_ptr(),
                output.len(),
            )
        }
        .min(output.len());
        output[..count]
            .iter()
            .filter_map(|command| {
                if command.name.is_null() || !command.velocity_rad_s.is_finite() {
                    return None;
                }
                // SAFETY: A non-null NUL-terminated command name is required
                // to remain valid until the host copies it after the callback.
                let name = unsafe { CStr::from_ptr(command.name) }.to_str().ok()?;
                Some((name.to_string(), command.velocity_rad_s))
            })
            .collect()
    }

    fn v3_step(
        &mut self,
        observation: &ControllerObservationFrame,
        step_fn: RneControllerStepV3Fn,
    ) -> Result<ControllerActionFrame, ControllerPluginError> {
        let _call = self.call_lock.lock().map_err(|_| {
            ControllerPluginError::Rejected("controller callback lock is poisoned".to_string())
        })?;
        let mut robot_ids = Vec::new();
        let mut joint_names = Vec::new();
        let mut raw = Vec::new();
        for robot in &observation.robots {
            for joint in &robot.joints {
                let robot_id = CString::new(robot.robot_id.as_str()).map_err(|error| {
                    ControllerPluginError::Rejected(format!(
                        "robot ID contains a NUL byte: {error}"
                    ))
                })?;
                let joint_name = CString::new(joint.name.as_str()).map_err(|error| {
                    ControllerPluginError::Rejected(format!(
                        "joint name contains a NUL byte: {error}"
                    ))
                })?;
                robot_ids.push(robot_id);
                joint_names.push(joint_name);
                let robot_id = robot_ids.last().expect("just pushed").as_ptr();
                let name = joint_names.last().expect("just pushed").as_ptr();
                raw.push(RneJointObservationV3 {
                    robot_id,
                    name,
                    position_rad: joint.position_rad,
                    velocity_rad_s: joint.velocity_rad_s.unwrap_or(0.0),
                    has_velocity: u8::from(joint.velocity_rad_s.is_some()),
                    reserved: [0; 7],
                });
            }
        }

        let mut output = vec![
            RneJointVelocityV3 {
                robot_id: std::ptr::null(),
                name: std::ptr::null(),
                velocity_rad_s: 0.0,
            };
            raw.len()
        ];
        let mut error_buffer = vec![0_u8; 512];
        // SAFETY: The raw observation and output arrays own their declared
        // capacities through this call, the handle is live, and `_call`
        // serializes access to plugin-owned state.
        let result = unsafe {
            step_fn(
                self.handle,
                observation.step,
                observation.sim_time_ticks,
                raw.as_ptr(),
                raw.len(),
                output.as_mut_ptr(),
                output.len(),
                error_buffer.as_mut_ptr().cast(),
                error_buffer.len(),
            )
        };
        if result.status != 0 {
            return Err(ControllerPluginError::Rejected(read_error(
                &error_buffer,
                "ABI-v3 controller step failed",
            )));
        }
        if result.output_count > output.len() {
            return Err(ControllerPluginError::Rejected(format!(
                "ABI-v3 controller returned {} commands for capacity {}",
                result.output_count,
                output.len()
            )));
        }
        let mut commands = BTreeMap::<String, Vec<ControllerJointVelocityCommand>>::new();
        for command in &output[..result.output_count] {
            if command.robot_id.is_null() || command.name.is_null() {
                return Err(ControllerPluginError::Rejected(
                    "ABI-v3 controller returned a null command identifier".to_string(),
                ));
            }
            // SAFETY: The ABI requires each non-null returned identifier to
            // remain NUL-terminated and valid until this immediate copy.
            let robot_id = unsafe { CStr::from_ptr(command.robot_id) }
                .to_str()
                .map_err(|error| {
                    ControllerPluginError::Rejected(format!(
                        "ABI-v3 command robot ID is not UTF-8: {error}"
                    ))
                })?;
            // SAFETY: Same returned-identifier lifetime contract as robot ID.
            let name = unsafe { CStr::from_ptr(command.name) }
                .to_str()
                .map_err(|error| {
                    ControllerPluginError::Rejected(format!(
                        "ABI-v3 command joint name is not UTF-8: {error}"
                    ))
                })?;
            commands.entry(robot_id.to_string()).or_default().push(
                ControllerJointVelocityCommand::new(name, command.velocity_rad_s),
            );
        }
        let robots = commands
            .into_iter()
            .map(|(robot_id, commands)| ControllerRobotAction::new(robot_id, commands))
            .collect::<Result<Vec<_>, _>>()?;
        Ok(ControllerActionFrame::new(observation.step, robots)?)
    }
}

impl Drop for LoadedControllerPlugin {
    fn drop(&mut self) {
        let _library_alive = &self.library;
        // SAFETY: Drop has exclusive ownership, the handle came from create
        // and has not been destroyed, and `_library_alive` keeps code loaded.
        unsafe { (self.destroy)(self.handle) }
    }
}

impl crate::ControllerPlugin for LoadedControllerPlugin {
    fn name(&self) -> &str {
        &self.name
    }

    fn capabilities(&self) -> Vec<ControllerCapability> {
        match &self.abi {
            LoadedControllerAbi::V2 { .. } => vec![
                ControllerCapability::JointPositionObservation,
                ControllerCapability::JointVelocityCommand,
            ],
            LoadedControllerAbi::V3 { capabilities, .. } => capabilities.clone(),
        }
    }

    fn joint_velocity_commands(
        &self,
        joint_names: &[&str],
        positions_rad: &[f64],
    ) -> Vec<(String, f64)> {
        self.legacy_commands(joint_names, positions_rad)
    }

    fn on_configure(
        &mut self,
        negotiation: &ControllerNegotiation,
    ) -> Result<(), ControllerPluginError> {
        let LoadedControllerAbi::V3 { configure, .. } = self.abi else {
            return Ok(());
        };
        let required = negotiation
            .configuration
            .required_capabilities
            .iter()
            .copied()
            .map(controller_capability_bit)
            .fold(0_u64, |mask, bit| mask | bit);
        let _call = self.call_lock.lock().map_err(|_| {
            ControllerPluginError::Rejected("controller callback lock is poisoned".to_string())
        })?;
        let mut error_buffer = vec![0_u8; 512];
        // SAFETY: The handle is live, the error buffer is writable for its
        // declared length, and `_call` serializes plugin callbacks.
        let status = unsafe {
            configure(
                self.handle,
                required,
                error_buffer.as_mut_ptr().cast(),
                error_buffer.len(),
            )
        };
        status_result(status, &error_buffer, "ABI-v3 configure failed")
    }

    fn on_reset(&mut self, context: ControllerResetContext) -> Result<(), ControllerPluginError> {
        let LoadedControllerAbi::V3 { reset, .. } = self.abi else {
            return Ok(());
        };
        let _call = self.call_lock.lock().map_err(|_| {
            ControllerPluginError::Rejected("controller callback lock is poisoned".to_string())
        })?;
        let mut error_buffer = vec![0_u8; 512];
        // SAFETY: The handle and buffer satisfy the reset contract, and
        // `_call` serializes plugin callbacks.
        let status = unsafe {
            reset(
                self.handle,
                context.episode,
                context.seed,
                context.step,
                context.sim_time_ticks,
                error_buffer.as_mut_ptr().cast(),
                error_buffer.len(),
            )
        };
        status_result(status, &error_buffer, "ABI-v3 reset failed")
    }

    fn step_frame(
        &mut self,
        observation: &ControllerObservationFrame,
    ) -> Result<ControllerActionFrame, ControllerPluginError> {
        observation.validate()?;
        match &self.abi {
            LoadedControllerAbi::V2 { .. } => {
                let mut robot_actions = Vec::new();
                for robot in &observation.robots {
                    let names = robot
                        .joints
                        .iter()
                        .map(|joint| joint.name.as_str())
                        .collect::<Vec<_>>();
                    let positions = robot
                        .joints
                        .iter()
                        .map(|joint| joint.position_rad)
                        .collect::<Vec<_>>();
                    let commands = self
                        .legacy_commands(&names, &positions)
                        .into_iter()
                        .map(|(name, velocity)| ControllerJointVelocityCommand::new(name, velocity))
                        .collect();
                    robot_actions.push(ControllerRobotAction::new(
                        robot.robot_id.clone(),
                        commands,
                    )?);
                }
                Ok(ControllerActionFrame::new(observation.step, robot_actions)?)
            }
            LoadedControllerAbi::V3 { step, .. } => self.v3_step(observation, *step),
        }
    }

    fn on_shutdown(&mut self) -> Result<(), ControllerPluginError> {
        let LoadedControllerAbi::V3 { shutdown, .. } = self.abi else {
            return Ok(());
        };
        let _call = self.call_lock.lock().map_err(|_| {
            ControllerPluginError::Rejected("controller callback lock is poisoned".to_string())
        })?;
        let mut error_buffer = vec![0_u8; 512];
        // SAFETY: The handle and buffer satisfy the shutdown contract, and
        // `_call` serializes plugin callbacks.
        let status = unsafe {
            shutdown(
                self.handle,
                error_buffer.as_mut_ptr().cast(),
                error_buffer.len(),
            )
        };
        status_result(status, &error_buffer, "ABI-v3 shutdown failed")
    }
}

/// Loads a controller library behind the robot-native plugin boundary.
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

fn shared_library_extensions() -> &'static [&'static str] {
    if cfg!(target_os = "windows") {
        &["dll"]
    } else if cfg!(target_os = "macos") {
        &["dylib"]
    } else {
        &["so"]
    }
}

fn is_shared_library(file_name: &str) -> bool {
    shared_library_extensions()
        .iter()
        .any(|extension| file_name.ends_with(&format!(".{extension}")))
}

/// Discovers a controller by logical name in deterministic path order.
pub fn discover_controller_plugin(
    name: &str,
    search_paths: &[&Path],
    joint: &str,
    target_rad: f64,
    gain: f64,
    max_velocity_rad_s: f64,
) -> Result<Box<dyn crate::ControllerPlugin>, PluginLoadError> {
    let name_lower = name.to_ascii_lowercase();
    for path in search_paths {
        let entries = std::fs::read_dir(path).map_err(|error| PluginLoadError::Search {
            path: path.display().to_string(),
            message: error.to_string(),
        })?;
        let mut candidates = Vec::new();
        for entry in entries.flatten() {
            let file_name = entry.file_name().to_string_lossy().to_ascii_lowercase();
            if is_shared_library(&file_name) && file_name.contains(&name_lower) {
                candidates.push(entry.path());
            }
        }
        candidates.sort_unstable();
        for candidate in candidates {
            let Ok(plugin) =
                load_controller_library(&candidate, joint, target_rad, gain, max_velocity_rad_s)
            else {
                continue;
            };
            if plugin.name().eq_ignore_ascii_case(name) {
                return Ok(plugin);
            }
        }
    }
    if name == "velocity_servo" {
        return Ok(Box::new(
            crate::VelocityServoController::new(name, joint, target_rad, gain, max_velocity_rad_s)
                .map_err(|error| PluginLoadError::Create {
                    message: error.to_string(),
                })?,
        ));
    }
    Err(PluginLoadError::NotFound {
        name: name.to_string(),
        search_paths: search_paths
            .iter()
            .map(|path| path.display().to_string())
            .collect::<Vec<_>>()
            .join(", "),
    })
}

/// Reads a supported plugin library's logical name without creating it.
pub fn peek_plugin_name(library_path: &Path) -> Option<String> {
    // SAFETY: This read-only probe validates and invokes only mandatory
    // metadata symbols while retaining `library` for every invocation.
    let library = unsafe { libloading::Library::new(library_path) }.ok()?;
    let abi_version: libloading::Symbol<'_, RnePluginAbiVersionFn> =
        // SAFETY: The symbol is copied/invoked with its documented ABI type.
        unsafe { library.get(b"rne_plugin_abi_version") }.ok()?;
    // SAFETY: The resolved function has the mandatory zero-argument ABI.
    let abi_version = unsafe { abi_version() };
    if !(RNE_PLUGIN_MIN_ABI_VERSION..=RNE_PLUGIN_ABI_VERSION).contains(&abi_version) {
        return None;
    }
    let name: libloading::Symbol<'_, RnePluginNameFn> =
        // SAFETY: The symbol is invoked only while `library` remains alive.
        unsafe { library.get(b"rne_plugin_name") }.ok()?;
    // SAFETY: The resolved function has the mandatory metadata ABI.
    let pointer = unsafe { name() };
    if pointer.is_null() {
        return None;
    }
    // SAFETY: The non-null pointer must reference a static NUL-terminated name
    // under the ABI; UTF-8 is checked before returning it.
    let name = unsafe { CStr::from_ptr(pointer) }
        .to_str()
        .ok()?
        .to_string();
    (!name.trim().is_empty()).then_some(name)
}

/// Enumerates supported controller plugins in deterministic name/path order.
pub fn discover_plugin_names(
    search_paths: &[&Path],
) -> Result<Vec<(String, PathBuf)>, PluginLoadError> {
    let mut found = BTreeMap::new();
    for path in search_paths {
        let entries = std::fs::read_dir(path).map_err(|error| PluginLoadError::Search {
            path: path.display().to_string(),
            message: error.to_string(),
        })?;
        let mut candidates = Vec::new();
        for entry in entries.flatten() {
            let file_name = entry.file_name().to_string_lossy().to_ascii_lowercase();
            if is_shared_library(&file_name) {
                candidates.push(entry.path());
            }
        }
        candidates.sort_unstable();
        for candidate in candidates {
            if let Some(name) = peek_plugin_name(&candidate) {
                found.entry(name).or_insert(candidate);
            }
        }
    }
    Ok(found.into_iter().collect())
}

fn decode_capability_mask(mask: u64) -> Result<Vec<ControllerCapability>, PluginLoadError> {
    if mask & !KNOWN_CAPABILITY_MASK != 0 {
        return Err(PluginLoadError::InvalidMetadata(format!(
            "unknown ABI-v3 capability bits {:#x}",
            mask & !KNOWN_CAPABILITY_MASK
        )));
    }
    let capabilities = [
        ControllerCapability::JointPositionObservation,
        ControllerCapability::JointVelocityObservation,
        ControllerCapability::JointVelocityCommand,
        ControllerCapability::MultiRobot,
    ]
    .into_iter()
    .filter(|capability| mask & controller_capability_bit(*capability) != 0)
    .collect();
    Ok(capabilities)
}

fn read_error(buffer: &[u8], fallback: &str) -> String {
    let message = CStr::from_bytes_until_nul(buffer)
        .ok()
        .and_then(|message| message.to_str().ok())
        .unwrap_or_default();
    if message.trim().is_empty() {
        fallback.to_string()
    } else {
        message.to_string()
    }
}

fn status_result(
    status: i32,
    error_buffer: &[u8],
    fallback: &str,
) -> Result<(), ControllerPluginError> {
    if status == 0 {
        Ok(())
    } else {
        Err(ControllerPluginError::Rejected(read_error(
            error_buffer,
            fallback,
        )))
    }
}

fn copy_symbol<T: Copy>(
    library: &libloading::Library,
    library_path: &Path,
    symbol_bytes: &[u8],
    symbol_name: &'static str,
) -> Result<T, PluginLoadError> {
    let symbol: libloading::Symbol<'_, T> =
        // SAFETY: Callers provide the exact ABI function-pointer type for each
        // mandatory symbol; the value is copied while `library` is live.
        unsafe { library.get(symbol_bytes) }.map_err(|error| PluginLoadError::Symbol {
            path: library_path.display().to_string(),
            symbol: symbol_name,
            message: error.to_string(),
        })?;
    Ok(*symbol)
}

/// A failure while opening, resolving, negotiating, or invoking a plugin.
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
    /// The plugin reports an unsupported ABI version.
    #[error("plugin ABI version {got} is outside supported range {minimum}..={maximum}")]
    AbiVersion {
        /// Version reported by the plugin.
        got: u32,
        /// Oldest version accepted by the runtime.
        minimum: u32,
        /// Newest version accepted by the runtime.
        maximum: u32,
    },
    /// Static plugin metadata or capability bits are malformed.
    #[error("invalid plugin metadata: {0}")]
    InvalidMetadata(String),
    /// Creating the controller instance failed.
    #[error("plugin controller create failed: {message}")]
    Create {
        /// Error message reported by the plugin.
        message: String,
    },
    /// A plugin search directory could not be read.
    #[error("read plugin search directory {path}: {message}")]
    Search {
        /// Directory path.
        path: String,
        /// Underlying filesystem error.
        message: String,
    },
    /// No plugin with the requested name was found.
    #[error("no controller plugin named `{name}` found in [{search_paths}] or built-in")]
    NotFound {
        /// Requested plugin name.
        name: String,
        /// Search directories that were consulted.
        search_paths: String,
    },
}
