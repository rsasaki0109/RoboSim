//! Standalone conformance runner for dynamically loaded controller plugins.

use crate::{
    ControllerActionFrame, ControllerCapability, ControllerConfiguration, ControllerDescriptor,
    ControllerHost, ControllerJointObservation, ControllerObservationFrame, ControllerPlugin,
    ControllerResetContext, ControllerRobotObservation, LoadedControllerPlugin, PluginManifest,
    CONTROLLER_SCHEMA_VERSION, RNE_PLUGIN_ABI_VERSION, RNE_PLUGIN_MIN_ABI_VERSION,
};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::fs;
use std::path::Path;

/// Stable kind identifier for controller-plugin conformance reports.
pub const CONTROLLER_PLUGIN_CONFORMANCE_REPORT_KIND: &str =
    "rne_controller_plugin_conformance_report";
/// Current controller-plugin conformance report schema.
pub const CONTROLLER_PLUGIN_CONFORMANCE_REPORT_SCHEMA_VERSION: u32 = 1;

const CHECK_IDS: [&str; 6] = [
    "manifest_identity",
    "abi_symbols",
    "capability_negotiation",
    "fixed_step_schema",
    "reset_replay_exact",
    "lifecycle_shutdown",
];

/// Deterministic controller construction parameters used by the conformance runner.
#[derive(Clone, Debug, PartialEq)]
pub struct ControllerPluginConformanceConfig {
    /// Joint name passed to the versioned controller constructor.
    pub joint: String,
    /// Target joint angle in radians.
    pub target_rad: f64,
    /// Controller gain passed through the ABI.
    pub gain: f64,
    /// Maximum joint velocity in radians per second.
    pub max_velocity_rad_s: f64,
    /// Explicit seed supplied to both deterministic reset passes.
    pub seed: u64,
}

impl Default for ControllerPluginConformanceConfig {
    fn default() -> Self {
        Self {
            joint: "conformance_joint".to_string(),
            target_rad: 1.0,
            gain: 2.0,
            max_velocity_rad_s: 5.0,
            seed: 7,
        }
    }
}

/// Content-addressed plugin and manifest inputs tested by the runner.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ControllerPluginConformanceSubject {
    /// Shared-library file name without a host-specific parent path.
    pub library_file: String,
    /// SHA-256 digest of the shared-library bytes.
    pub library_sha256: String,
    /// Shared-library size in bytes.
    pub library_size_bytes: u64,
    /// Manifest file name without a host-specific parent path.
    pub manifest_file: String,
    /// SHA-256 digest of the manifest bytes.
    pub manifest_sha256: String,
}

/// Controller identity negotiated from the loaded binary.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ControllerPluginConformanceIdentity {
    /// Logical plugin name reported by the loaded binary.
    pub name: String,
    /// C ABI version reported by the plugin.
    pub abi_version: u32,
    /// Robot-native observation/action schema version tested by the host.
    pub controller_schema_version: u32,
    /// Canonically ordered capabilities reported by the plugin.
    pub capabilities: Vec<ControllerCapability>,
}

/// One named controller-plugin conformance verdict.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ControllerPluginConformanceCheck {
    /// Stable check identifier.
    pub id: String,
    /// `passed`, `failed`, or `not_run`.
    pub status: String,
    /// Bounded diagnostic associated with this verdict.
    pub detail: String,
}

/// Portable, machine-readable controller-plugin conformance report.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ControllerPluginConformanceReport {
    /// Report schema version.
    pub schema_version: u32,
    /// Stable report kind.
    pub kind: String,
    /// Aggregate `passed` or `failed` verdict.
    pub status: String,
    /// Content-addressed inputs tested by this run.
    pub subject: ControllerPluginConformanceSubject,
    /// Loaded identity, absent when the binary cannot be negotiated.
    pub controller: Option<ControllerPluginConformanceIdentity>,
    /// Checks in the canonical conformance order.
    pub checks: Vec<ControllerPluginConformanceCheck>,
}

impl ControllerPluginConformanceReport {
    /// Returns true only when every canonical check passed.
    pub fn passed(&self) -> bool {
        self.status == "passed"
            && self.checks.iter().all(|check| check.status == "passed")
            && self.controller.is_some()
    }

    /// Validates the report schema, ordering, statuses, and aggregate verdict.
    pub fn validate(&self) -> Result<(), ControllerPluginConformanceError> {
        if self.schema_version != CONTROLLER_PLUGIN_CONFORMANCE_REPORT_SCHEMA_VERSION {
            return Err(ControllerPluginConformanceError::InvalidReport(format!(
                "expected schema {}, got {}",
                CONTROLLER_PLUGIN_CONFORMANCE_REPORT_SCHEMA_VERSION, self.schema_version
            )));
        }
        if self.kind != CONTROLLER_PLUGIN_CONFORMANCE_REPORT_KIND {
            return Err(ControllerPluginConformanceError::InvalidReport(
                "report kind drifted".to_string(),
            ));
        }
        validate_subject_file("library_file", &self.subject.library_file)?;
        validate_subject_file("manifest_file", &self.subject.manifest_file)?;
        for digest in [
            self.subject.library_sha256.as_str(),
            self.subject.manifest_sha256.as_str(),
        ] {
            if digest.len() != 64
                || !digest
                    .bytes()
                    .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
            {
                return invalid_report("subject digest is not lowercase SHA-256 hex");
            }
        }
        if let Some(controller) = &self.controller {
            if !(RNE_PLUGIN_MIN_ABI_VERSION..=RNE_PLUGIN_ABI_VERSION)
                .contains(&controller.abi_version)
            {
                return invalid_report("controller ABI is outside the supported range");
            }
            let descriptor = ControllerDescriptor {
                schema_version: controller.controller_schema_version,
                name: controller.name.clone(),
                capabilities: controller.capabilities.clone(),
            };
            descriptor.validate().map_err(|error| {
                ControllerPluginConformanceError::InvalidReport(format!(
                    "controller identity is invalid: {error}"
                ))
            })?;
        }
        if self
            .checks
            .iter()
            .map(|check| check.id.as_str())
            .ne(CHECK_IDS)
        {
            return Err(ControllerPluginConformanceError::InvalidReport(
                "check registry is not canonical".to_string(),
            ));
        }
        if self.checks.iter().any(|check| {
            !matches!(check.status.as_str(), "passed" | "failed" | "not_run")
                || check.detail.chars().count() > 512
        }) {
            return Err(ControllerPluginConformanceError::InvalidReport(
                "check status or diagnostic is invalid".to_string(),
            ));
        }
        let expected_status = if self.checks.iter().all(|check| check.status == "passed")
            && self.controller.is_some()
        {
            "passed"
        } else {
            "failed"
        };
        if self.status != expected_status {
            return Err(ControllerPluginConformanceError::InvalidReport(
                "aggregate status does not match checks".to_string(),
            ));
        }
        if self.status == "passed" {
            if self.subject.library_size_bytes == 0 {
                return invalid_report("a passing report must bind a non-empty library");
            }
            let Some(controller) = self.controller.as_ref() else {
                return invalid_report("a passing report requires a controller identity");
            };
            for required in [
                ControllerCapability::JointPositionObservation,
                ControllerCapability::JointVelocityCommand,
            ] {
                if controller.capabilities.binary_search(&required).is_err() {
                    return invalid_report(
                        "a passing report is missing a required controller capability",
                    );
                }
            }
        }
        Ok(())
    }

    /// Serializes a validated report as stable, pretty JSON with a trailing newline.
    pub fn to_json_pretty(&self) -> Result<String, ControllerPluginConformanceError> {
        self.validate()?;
        let mut text = serde_json::to_string_pretty(self)?;
        text.push('\n');
        Ok(text)
    }

    fn new(subject: ControllerPluginConformanceSubject) -> Self {
        Self {
            schema_version: CONTROLLER_PLUGIN_CONFORMANCE_REPORT_SCHEMA_VERSION,
            kind: CONTROLLER_PLUGIN_CONFORMANCE_REPORT_KIND.to_string(),
            status: "failed".to_string(),
            subject,
            controller: None,
            checks: CHECK_IDS
                .iter()
                .map(|id| ControllerPluginConformanceCheck {
                    id: (*id).to_string(),
                    status: "not_run".to_string(),
                    detail: String::new(),
                })
                .collect(),
        }
    }

    fn verdict(&mut self, id: &str, passed: bool, detail: impl Into<String>) {
        let check = self
            .checks
            .iter_mut()
            .find(|check| check.id == id)
            .expect("canonical conformance check");
        check.status = if passed { "passed" } else { "failed" }.to_string();
        check.detail = detail.into().chars().take(512).collect();
        self.status = (if self.checks.iter().all(|check| check.status == "passed")
            && self.controller.is_some()
        {
            "passed"
        } else {
            "failed"
        })
        .to_string();
    }
}

fn validate_subject_file(field: &str, value: &str) -> Result<(), ControllerPluginConformanceError> {
    if value.is_empty()
        || value != value.trim()
        || value.chars().count() > 255
        || matches!(value, "." | "..")
        || value
            .chars()
            .any(|character| character.is_control() || matches!(character, '/' | '\\' | ':'))
    {
        return Err(ControllerPluginConformanceError::InvalidReport(format!(
            "{field} must be a portable basename"
        )));
    }
    Ok(())
}

fn invalid_report<T>(message: &str) -> Result<T, ControllerPluginConformanceError> {
    Err(ControllerPluginConformanceError::InvalidReport(
        message.to_string(),
    ))
}

/// Failure reading inputs or serializing a conformance report.
#[derive(Debug, thiserror::Error)]
pub enum ControllerPluginConformanceError {
    /// A conformance input could not be read.
    #[error("read conformance input {path}: {message}")]
    Read {
        /// Input path.
        path: String,
        /// Operating-system diagnostic.
        message: String,
    },
    /// A report invariant was violated.
    #[error("invalid controller-plugin conformance report: {0}")]
    InvalidReport(String),
    /// Report JSON serialization failed.
    #[error(transparent)]
    Json(#[from] serde_json::Error),
}

/// Runs manifest, ABI, lifecycle, schema, and deterministic-reset conformance.
///
/// Semantic plugin failures are recorded in a valid report and do not become
/// function errors. Errors are reserved for unreadable inputs or an invalid
/// report produced by the runner itself.
pub fn run_controller_plugin_conformance(
    library_path: &Path,
    manifest_path: &Path,
    config: &ControllerPluginConformanceConfig,
) -> Result<ControllerPluginConformanceReport, ControllerPluginConformanceError> {
    let library = read_input(library_path)?;
    let manifest_bytes = read_input(manifest_path)?;
    let subject = ControllerPluginConformanceSubject {
        library_file: file_name(library_path),
        library_sha256: sha256_hex(&library),
        library_size_bytes: u64::try_from(library.len()).unwrap_or(u64::MAX),
        manifest_file: file_name(manifest_path),
        manifest_sha256: sha256_hex(&manifest_bytes),
    };
    let mut report = ControllerPluginConformanceReport::new(subject);

    let manifest: PluginManifest = match serde_json::from_slice(&manifest_bytes) {
        Ok(manifest) => manifest,
        Err(error) => {
            report.verdict("manifest_identity", false, error.to_string());
            report.validate()?;
            return Ok(report);
        }
    };
    if let Err(error) = manifest.validate() {
        report.verdict("manifest_identity", false, error.to_string());
        report.validate()?;
        return Ok(report);
    }

    let loaded = match LoadedControllerPlugin::load(
        library_path,
        &config.joint,
        config.target_rad,
        config.gain,
        config.max_velocity_rad_s,
    ) {
        Ok(loaded) => loaded,
        Err(error) => {
            report.verdict("abi_symbols", false, error.to_string());
            report.validate()?;
            return Ok(report);
        }
    };
    let abi_version = loaded.abi_version();
    let binary_name = loaded.name().to_string();
    if manifest.name != binary_name {
        report.verdict(
            "manifest_identity",
            false,
            format!(
                "manifest name {:?} differs from binary name {:?}",
                manifest.name, binary_name
            ),
        );
    } else {
        report.verdict("manifest_identity", true, "manifest and binary names match");
    }
    report.verdict("abi_symbols", true, format!("loaded ABI v{abi_version}"));

    let mut host = match ControllerHost::new(Box::new(loaded)) {
        Ok(host) => host,
        Err(error) => {
            report.verdict("capability_negotiation", false, error.to_string());
            report.validate()?;
            return Ok(report);
        }
    };
    let descriptor = host.descriptor().clone();
    report.controller = Some(ControllerPluginConformanceIdentity {
        name: descriptor.name.clone(),
        abi_version,
        controller_schema_version: CONTROLLER_SCHEMA_VERSION,
        capabilities: descriptor.capabilities.clone(),
    });
    let mut required = vec![
        ControllerCapability::JointPositionObservation,
        ControllerCapability::JointVelocityCommand,
    ];
    if descriptor.supports(ControllerCapability::MultiRobot) {
        required.push(ControllerCapability::MultiRobot);
    }
    if let Err(error) = host.configure(ControllerConfiguration::new(required)) {
        report.verdict("capability_negotiation", false, error.to_string());
        finish_shutdown(&mut report, &mut host);
        report.validate()?;
        return Ok(report);
    }
    let reset = ControllerResetContext {
        episode: 0,
        seed: config.seed,
        step: 0,
        sim_time_ticks: 0,
    };
    if let Err(error) = host.activate(reset) {
        report.verdict("capability_negotiation", false, error.to_string());
        finish_shutdown(&mut report, &mut host);
        report.validate()?;
        return Ok(report);
    }
    report.verdict(
        "capability_negotiation",
        true,
        format!("{} canonical capabilities", descriptor.capabilities.len()),
    );

    let observations = match conformance_observations(
        &config.joint,
        descriptor.supports(ControllerCapability::MultiRobot),
    ) {
        Ok(observations) => observations,
        Err(error) => {
            report.verdict("fixed_step_schema", false, error);
            finish_shutdown(&mut report, &mut host);
            report.validate()?;
            return Ok(report);
        }
    };
    let first = match run_steps(&mut host, &observations) {
        Ok(actions) => {
            report.verdict(
                "fixed_step_schema",
                true,
                format!("{} validated action frames", actions.len()),
            );
            actions
        }
        Err(error) => {
            report.verdict("fixed_step_schema", false, error);
            finish_shutdown(&mut report, &mut host);
            report.validate()?;
            return Ok(report);
        }
    };
    match host
        .reset(reset)
        .map_err(|error| error.to_string())
        .and_then(|()| run_steps(&mut host, &observations))
    {
        Ok(second) if first == second => {
            report.verdict("reset_replay_exact", true, "identical reset replay actions")
        }
        Ok(_) => report.verdict(
            "reset_replay_exact",
            false,
            "action frames changed after identical reset",
        ),
        Err(error) => report.verdict("reset_replay_exact", false, error),
    }
    finish_shutdown(&mut report, &mut host);
    report.validate()?;
    Ok(report)
}

fn conformance_observations(
    joint: &str,
    multi_robot: bool,
) -> Result<Vec<ControllerObservationFrame>, String> {
    [0.25, 0.5]
        .into_iter()
        .enumerate()
        .map(|(step, position_rad)| {
            let mut robots = vec![ControllerRobotObservation::new(
                "robot_a",
                vec![ControllerJointObservation::position_velocity(
                    joint,
                    position_rad,
                    0.125,
                )],
            )
            .map_err(|error| error.to_string())?];
            if multi_robot {
                robots.push(
                    ControllerRobotObservation::new(
                        "robot_b",
                        vec![ControllerJointObservation::position_velocity(
                            joint,
                            -position_rad,
                            -0.125,
                        )],
                    )
                    .map_err(|error| error.to_string())?,
                );
            }
            ControllerObservationFrame::new(step as u64, step as u64 * 16_666_667, robots)
                .map_err(|error| error.to_string())
        })
        .collect()
}

fn run_steps(
    host: &mut ControllerHost,
    observations: &[ControllerObservationFrame],
) -> Result<Vec<ControllerActionFrame>, String> {
    observations
        .iter()
        .map(|observation| host.step(observation).map_err(|error| error.to_string()))
        .collect()
}

fn finish_shutdown(report: &mut ControllerPluginConformanceReport, host: &mut ControllerHost) {
    match host.shutdown() {
        Ok(()) => report.verdict("lifecycle_shutdown", true, "shutdown accepted exactly once"),
        Err(error) => report.verdict("lifecycle_shutdown", false, error.to_string()),
    }
}

fn read_input(path: &Path) -> Result<Vec<u8>, ControllerPluginConformanceError> {
    fs::read(path).map_err(|error| ControllerPluginConformanceError::Read {
        path: path.display().to_string(),
        message: error.to_string(),
    })
}

fn file_name(path: &Path) -> String {
    path.file_name()
        .unwrap_or(path.as_os_str())
        .to_string_lossy()
        .into_owned()
}

fn sha256_hex(bytes: &[u8]) -> String {
    format!("{:x}", Sha256::digest(bytes))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn passing_report() -> ControllerPluginConformanceReport {
        ControllerPluginConformanceReport {
            schema_version: CONTROLLER_PLUGIN_CONFORMANCE_REPORT_SCHEMA_VERSION,
            kind: CONTROLLER_PLUGIN_CONFORMANCE_REPORT_KIND.to_string(),
            status: "passed".to_string(),
            subject: ControllerPluginConformanceSubject {
                library_file: "reference_controller.dll".to_string(),
                library_sha256: "a".repeat(64),
                library_size_bytes: 4_096,
                manifest_file: "rne-plugin.json".to_string(),
                manifest_sha256: "b".repeat(64),
            },
            controller: Some(ControllerPluginConformanceIdentity {
                name: "reference_controller".to_string(),
                abi_version: RNE_PLUGIN_ABI_VERSION,
                controller_schema_version: CONTROLLER_SCHEMA_VERSION,
                capabilities: vec![
                    ControllerCapability::JointPositionObservation,
                    ControllerCapability::JointVelocityCommand,
                ],
            }),
            checks: CHECK_IDS
                .iter()
                .map(|id| ControllerPluginConformanceCheck {
                    id: (*id).to_string(),
                    status: "passed".to_string(),
                    detail: format!("{id} passed"),
                })
                .collect(),
        }
    }

    #[test]
    fn passing_report_requires_canonical_subject_and_controller_identity() {
        passing_report().validate().expect("valid passing report");

        let mut invalid = passing_report();
        invalid.subject.library_file = "plugins/reference_controller.dll".to_string();
        assert!(invalid.validate().is_err());

        let mut invalid = passing_report();
        invalid.subject.manifest_file = "C:\\rne-plugin.json".to_string();
        assert!(invalid.validate().is_err());

        let mut invalid = passing_report();
        invalid.subject.library_sha256 = "A".repeat(64);
        assert!(invalid.validate().is_err());

        let mut invalid = passing_report();
        invalid.subject.library_size_bytes = 0;
        assert!(invalid.validate().is_err());

        let mut invalid = passing_report();
        invalid.controller.as_mut().expect("identity").abi_version = RNE_PLUGIN_MIN_ABI_VERSION - 1;
        assert!(invalid.validate().is_err());

        let mut invalid = passing_report();
        invalid
            .controller
            .as_mut()
            .expect("identity")
            .controller_schema_version += 1;
        assert!(invalid.validate().is_err());

        let mut invalid = passing_report();
        invalid
            .controller
            .as_mut()
            .expect("identity")
            .capabilities
            .reverse();
        assert!(invalid.validate().is_err());

        let mut invalid = passing_report();
        invalid
            .controller
            .as_mut()
            .expect("identity")
            .capabilities
            .pop();
        assert!(invalid.validate().is_err());
    }

    #[test]
    fn failed_report_can_describe_an_empty_library() {
        let mut report = passing_report();
        report.status = "failed".to_string();
        report.subject.library_size_bytes = 0;
        report.controller = None;
        report.checks[0].status = "failed".to_string();
        report.checks[1..].iter_mut().for_each(|check| {
            check.status = "not_run".to_string();
            check.detail.clear();
        });

        report.validate().expect("valid semantic failure report");
    }
}
