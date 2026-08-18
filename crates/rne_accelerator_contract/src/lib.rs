//! Backend-neutral public contracts for out-of-process RNE accelerators.
//!
//! Vendor runtimes stay outside core crates. These serializable types let an
//! installed verifier check accelerator identity, pinned runtime requirements,
//! and capability evidence without importing CUDA, JAX, MuJoCo, or Python.

#![deny(missing_docs)]

use rne_ai::{TaskSpec, PORTABLE_BATCH_CHECKPOINT_VERSION, TASK_SPEC_SCHEMA_VERSION};
use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;
use thiserror::Error;

mod conformance;
mod scale;

pub use conformance::{
    AcceleratorConformanceActual, AcceleratorConformanceFaultInjection,
    AcceleratorConformanceMetrics, AcceleratorConformanceReference, AcceleratorConformanceReport,
    AcceleratorConformanceTolerances, ACCELERATOR_CONFORMANCE_REPORT_KIND,
};
pub use scale::{AcceleratorScaleReport, AcceleratorScaleRun, ACCELERATOR_SCALE_REPORT_KIND};

/// Stable capability-report discriminator.
pub const ACCELERATOR_CAPABILITY_REPORT_KIND: &str = "rne_accelerator_capability_report";
/// Current accelerator manifest schema.
pub const ACCELERATOR_MANIFEST_SCHEMA_VERSION: u32 = 1;
/// Current accelerator protocol schema.
pub const ACCELERATOR_PROTOCOL_SCHEMA_VERSION: u32 = 1;
/// Current capability-report schema.
pub const ACCELERATOR_CAPABILITY_REPORT_SCHEMA_VERSION: u32 = 1;
/// Current conformance-report schema advertised by capability v1.
pub const ACCELERATOR_CONFORMANCE_REPORT_SCHEMA_VERSION: u32 = 1;
/// Current runtime-contract schema.
pub const ACCELERATOR_RUNTIME_CONTRACT_SCHEMA_VERSION: u32 = 1;
/// Current scale-report schema advertised by capability v1.
pub const ACCELERATOR_SCALE_REPORT_SCHEMA_VERSION: u32 = 1;

/// Invalid accelerator manifest, runtime contract, or capability report.
#[derive(Debug, Error, PartialEq, Eq)]
pub enum AcceleratorContractError {
    /// A schema-derived invariant failed.
    #[error("invalid accelerator contract: {0}")]
    Invalid(String),
}

/// Versioned selection manifest for one out-of-process accelerator adapter.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct AcceleratorManifest {
    /// Manifest schema version.
    pub schema_version: u32,
    /// Stable adapter identifier.
    pub id: String,
    /// Whether this is the repository-selected adapter.
    pub selected: bool,
    /// Lifecycle status such as `experimental`.
    pub status: String,
    /// Stable runtime identifier.
    pub runtime: String,
    /// Numeric precision exposed by the adapter.
    pub precision: String,
    /// Process or transport boundary.
    pub execution_boundary: String,
    /// Whether accelerator dependencies enter core crates.
    pub core_dependency: bool,
    /// Whether the runtime requires an NVIDIA GPU.
    pub requires_nvidia_gpu: bool,
    /// Supported TaskSpec schema.
    pub task_spec_schema: u32,
    /// Supported portable batch checkpoint schema.
    pub batch_checkpoint_schema: u32,
    /// Supported wire protocol schema.
    pub protocol_schema: u32,
    /// Supported capability-report schema.
    pub capability_report_schema: u32,
    /// Supported conformance-report schema.
    pub conformance_report_schema: u32,
    /// Supported scale-report schema.
    pub scale_report_schema: u32,
    /// Ordered supported batch widths.
    pub supported_batch_widths: Vec<usize>,
    /// Repository-relative bound TaskSpec path.
    pub binding_task_spec: String,
    /// Repository-relative bound model path.
    pub binding_model: String,
    /// Repository-relative runtime-contract path.
    pub runtime_contract: String,
    /// Repository-relative package requirements path.
    pub requirements: String,
    /// Repository-relative selection ADR path.
    pub selection_adr: String,
    /// Authoritative upstream sources for the selection.
    pub official_sources: Vec<String>,
}

impl AcceleratorManifest {
    /// Validates portable manifest invariants without selecting a vendor policy.
    pub fn validate(&self) -> Result<(), AcceleratorContractError> {
        require(
            self.schema_version == ACCELERATOR_MANIFEST_SCHEMA_VERSION,
            "manifest schema mismatch",
        )?;
        require_identifier(&self.id, "adapter id")?;
        require_identifier(&self.runtime, "runtime id")?;
        require(!self.status.trim().is_empty(), "manifest status is empty")?;
        require(
            !self.precision.trim().is_empty(),
            "manifest precision is empty",
        )?;
        require(
            !self.execution_boundary.trim().is_empty(),
            "execution boundary is empty",
        )?;
        require(
            !self.core_dependency,
            "accelerator dependencies entered core",
        )?;
        require(
            self.task_spec_schema == TASK_SPEC_SCHEMA_VERSION,
            "TaskSpec schema mismatch",
        )?;
        require(
            self.batch_checkpoint_schema == PORTABLE_BATCH_CHECKPOINT_VERSION,
            "batch checkpoint schema mismatch",
        )?;
        require(
            self.protocol_schema == ACCELERATOR_PROTOCOL_SCHEMA_VERSION,
            "protocol schema mismatch",
        )?;
        require(
            self.capability_report_schema == ACCELERATOR_CAPABILITY_REPORT_SCHEMA_VERSION,
            "capability-report schema mismatch",
        )?;
        require(
            self.conformance_report_schema == ACCELERATOR_CONFORMANCE_REPORT_SCHEMA_VERSION,
            "conformance-report schema mismatch",
        )?;
        require(
            self.scale_report_schema == ACCELERATOR_SCALE_REPORT_SCHEMA_VERSION,
            "scale-report schema mismatch",
        )?;
        validate_ordered_positive_unique(&self.supported_batch_widths, "supported batch widths")?;
        for (value, label) in [
            (&self.binding_task_spec, "binding TaskSpec path"),
            (&self.binding_model, "binding model path"),
            (&self.runtime_contract, "runtime contract path"),
            (&self.requirements, "requirements path"),
            (&self.selection_adr, "selection ADR path"),
        ] {
            validate_relative_path(value, label)?;
        }
        validate_https_sources(&self.official_sources, "manifest official sources")
    }
}

/// Exact package pins required by an accelerator runtime.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct AcceleratorRuntimePackages {
    /// JAX package version.
    pub jax: String,
    /// JAXlib package version.
    pub jaxlib: String,
    /// JAX CUDA plugin package version.
    pub jax_cuda_plugin: String,
    /// MuJoCo package version.
    pub mujoco: String,
    /// MuJoCo MJX package version.
    pub mujoco_mjx: String,
    /// Warp package version.
    pub warp_lang: String,
}

impl AcceleratorRuntimePackages {
    fn validate(&self) -> Result<(), AcceleratorContractError> {
        for value in [
            &self.jax,
            &self.jaxlib,
            &self.jax_cuda_plugin,
            &self.mujoco,
            &self.mujoco_mjx,
            &self.warp_lang,
        ] {
            require(is_version(value), "runtime package version is invalid")?;
        }
        Ok(())
    }
}

/// Pinned host and package contract required by an accelerator runtime.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct AcceleratorRuntimeContract {
    /// Runtime-contract schema version.
    pub schema_version: u32,
    /// Required operating system.
    pub operating_system: String,
    /// Required machine architecture.
    pub architecture: String,
    /// Required Python version.
    pub python: String,
    /// Required CUDA major version.
    pub cuda_major: u32,
    /// Minimum NVIDIA driver major version.
    pub nvidia_driver_minimum: u32,
    /// Exact runtime package pins.
    pub packages: AcceleratorRuntimePackages,
    /// Authoritative upstream runtime sources.
    pub official_sources: Vec<String>,
}

impl AcceleratorRuntimeContract {
    /// Validates a standalone runtime contract.
    pub fn validate(&self) -> Result<(), AcceleratorContractError> {
        require(
            self.schema_version == ACCELERATOR_RUNTIME_CONTRACT_SCHEMA_VERSION,
            "runtime-contract schema mismatch",
        )?;
        require_identifier(&self.operating_system, "runtime operating system")?;
        require_identifier(&self.architecture, "runtime architecture")?;
        require(is_version(&self.python), "Python version is invalid")?;
        require(self.cuda_major > 0, "CUDA major must be positive")?;
        require(
            self.nvidia_driver_minimum > 0,
            "NVIDIA driver minimum must be positive",
        )?;
        self.packages.validate()?;
        validate_https_sources(&self.official_sources, "runtime official sources")
    }
}

/// Runtime values observed while probing an accelerator.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct AcceleratorRuntimeProbe {
    /// Observed Python version.
    pub python_version: String,
    /// Observed operating system.
    pub platform: String,
    /// Observed machine architecture.
    pub machine: String,
    /// Observed JAX version.
    pub jax_version: Option<String>,
    /// Observed JAXlib version.
    pub jaxlib_version: Option<String>,
    /// Observed JAX CUDA plugin version.
    pub jax_cuda_plugin_version: Option<String>,
    /// Observed MuJoCo version.
    pub mujoco_version: Option<String>,
    /// Observed MuJoCo MJX version.
    pub mujoco_mjx_version: Option<String>,
    /// Observed Warp version.
    pub warp_version: Option<String>,
    /// Observed JAX backend name.
    pub jax_backend: Option<String>,
    /// Ordered JAX device descriptions.
    pub jax_devices: Vec<String>,
    /// Observed NVIDIA driver version.
    pub nvidia_driver_version: Option<String>,
}

/// Versioned accelerator capability report.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct AcceleratorCapabilityReport {
    /// Stable report discriminator.
    pub kind: String,
    /// Capability-report schema version.
    pub schema_version: u32,
    /// Stable adapter identifier.
    pub adapter_id: String,
    /// Stable runtime identifier.
    pub runtime_id: String,
    /// `available`, `unavailable`, or dependency-free `test_only` status.
    pub status: String,
    /// Stable failure reason when unavailable.
    pub unavailable_reason_code: Option<String>,
    /// Process or transport boundary.
    pub execution_boundary: String,
    /// Numeric precision exposed by the adapter.
    pub precision: String,
    /// Supported wire protocol schema.
    pub protocol_schema: u32,
    /// Supported TaskSpec schema.
    pub task_spec_schema: u32,
    /// Supported portable batch checkpoint schema.
    pub batch_checkpoint_schema: u32,
    /// Supported conformance-report schema.
    pub conformance_report_schema: u32,
    /// Supported scale-report schema.
    pub scale_report_schema: u32,
    /// Ordered supported task identifiers.
    pub supported_task_ids: Vec<String>,
    /// Ordered supported batch widths.
    pub supported_batch_widths: Vec<usize>,
    /// Whether an NVIDIA GPU is required.
    pub requires_nvidia_gpu: bool,
    /// Ordered explicitly unsupported feature identifiers.
    pub unsupported_features: Vec<String>,
    /// Values observed during the runtime probe.
    pub runtime: AcceleratorRuntimeProbe,
    /// Exact runtime requirements used by the probe.
    pub runtime_contract: AcceleratorRuntimeContract,
    /// Runtime-contract schema repeated for negotiation.
    pub runtime_contract_schema: u32,
}

impl AcceleratorCapabilityReport {
    /// Validates standalone schema and status invariants.
    pub fn validate(&self) -> Result<(), AcceleratorContractError> {
        require(
            self.kind == ACCELERATOR_CAPABILITY_REPORT_KIND,
            "capability-report kind mismatch",
        )?;
        require(
            self.schema_version == ACCELERATOR_CAPABILITY_REPORT_SCHEMA_VERSION,
            "capability-report schema mismatch",
        )?;
        require_identifier(&self.adapter_id, "adapter id")?;
        require_identifier(&self.runtime_id, "runtime id")?;
        require(
            !self.execution_boundary.trim().is_empty(),
            "execution boundary is empty",
        )?;
        require(!self.precision.trim().is_empty(), "precision is empty")?;
        require(
            self.protocol_schema == ACCELERATOR_PROTOCOL_SCHEMA_VERSION,
            "protocol schema mismatch",
        )?;
        require(
            self.task_spec_schema == TASK_SPEC_SCHEMA_VERSION,
            "TaskSpec schema mismatch",
        )?;
        require(
            self.batch_checkpoint_schema == PORTABLE_BATCH_CHECKPOINT_VERSION,
            "batch checkpoint schema mismatch",
        )?;
        require(
            self.conformance_report_schema == ACCELERATOR_CONFORMANCE_REPORT_SCHEMA_VERSION,
            "conformance-report schema mismatch",
        )?;
        require(
            self.scale_report_schema == ACCELERATOR_SCALE_REPORT_SCHEMA_VERSION,
            "scale-report schema mismatch",
        )?;
        validate_ordered_identifiers(&self.supported_task_ids, "supported task ids")?;
        validate_ordered_positive_unique(&self.supported_batch_widths, "supported batch widths")?;
        validate_ordered_identifiers(&self.unsupported_features, "unsupported features")?;
        self.runtime_contract.validate()?;
        require(
            self.runtime_contract_schema == self.runtime_contract.schema_version,
            "runtime-contract schema repetition mismatch",
        )?;
        require(
            !self.runtime.python_version.trim().is_empty(),
            "runtime Python version is empty",
        )?;
        require(
            !self.runtime.platform.trim().is_empty(),
            "runtime platform is empty",
        )?;
        require(
            !self.runtime.machine.trim().is_empty(),
            "runtime machine is empty",
        )?;
        match self.status.as_str() {
            "available" => {
                require(
                    self.unavailable_reason_code.is_none(),
                    "available report has an unavailable reason",
                )?;
                require(
                    self.runtime.platform == self.runtime_contract.operating_system,
                    "available runtime operating system differs from contract",
                )?;
                require(
                    self.runtime.machine == self.runtime_contract.architecture,
                    "available runtime architecture differs from contract",
                )?;
                let python_prefix = format!("{}.", self.runtime_contract.python);
                require(
                    self.runtime.python_version == self.runtime_contract.python
                        || self.runtime.python_version.starts_with(&python_prefix),
                    "available Python version differs from contract",
                )?;
                let driver_major = self
                    .runtime
                    .nvidia_driver_version
                    .as_deref()
                    .and_then(|version| version.split('.').next())
                    .and_then(|major| major.parse::<u32>().ok());
                require(
                    driver_major
                        .is_some_and(|major| major >= self.runtime_contract.nvidia_driver_minimum),
                    "available NVIDIA driver is missing, invalid, or too old",
                )?;
                require(
                    self.runtime.jax_backend.as_deref() == Some("gpu"),
                    "available report did not select the GPU backend",
                )?;
                require(
                    !self.runtime.jax_devices.is_empty(),
                    "available report has no JAX devices",
                )?;
                self.validate_runtime_versions()?;
            }
            "unavailable" => {
                let reason = self.unavailable_reason_code.as_deref().unwrap_or_default();
                require_identifier(reason, "unavailable reason code")?;
            }
            "test_only" => {
                require(
                    self.unavailable_reason_code.is_none(),
                    "test-only report has an unavailable reason",
                )?;
                require(
                    self.runtime.jax_backend.is_none() && self.runtime.jax_devices.is_empty(),
                    "test-only report claims accelerator devices",
                )?;
                require(
                    self.runtime_versions()
                        .iter()
                        .all(|version| version.is_none()),
                    "test-only report claims accelerator package versions",
                )?;
            }
            _ => return Err(invalid("unknown capability status")),
        }
        Ok(())
    }

    /// Binds capability evidence to its exact manifest, runtime contract, and task.
    pub fn validate_against(
        &self,
        manifest: &AcceleratorManifest,
        runtime_contract: &AcceleratorRuntimeContract,
        task_spec: &TaskSpec,
    ) -> Result<(), AcceleratorContractError> {
        self.validate()?;
        manifest.validate()?;
        runtime_contract.validate()?;
        task_spec
            .validate()
            .map_err(|error| invalid(format!("bound TaskSpec is invalid: {error}")))?;
        require(
            self.adapter_id == manifest.id,
            "adapter id differs from manifest",
        )?;
        require(
            self.runtime_id == manifest.runtime,
            "runtime id differs from manifest",
        )?;
        require(
            self.execution_boundary == manifest.execution_boundary,
            "execution boundary differs from manifest",
        )?;
        require(
            self.precision == manifest.precision,
            "precision differs from manifest",
        )?;
        require(
            self.protocol_schema == manifest.protocol_schema,
            "protocol schema differs from manifest",
        )?;
        require(
            self.task_spec_schema == manifest.task_spec_schema,
            "TaskSpec schema differs from manifest",
        )?;
        require(
            self.batch_checkpoint_schema == manifest.batch_checkpoint_schema,
            "checkpoint schema differs from manifest",
        )?;
        require(
            self.conformance_report_schema == manifest.conformance_report_schema,
            "conformance schema differs from manifest",
        )?;
        require(
            self.scale_report_schema == manifest.scale_report_schema,
            "scale schema differs from manifest",
        )?;
        require(
            self.supported_batch_widths == manifest.supported_batch_widths,
            "batch widths differ from manifest",
        )?;
        require(
            self.requires_nvidia_gpu == manifest.requires_nvidia_gpu,
            "GPU requirement differs from manifest",
        )?;
        require(
            &self.runtime_contract == runtime_contract,
            "runtime contract differs from selected contract",
        )?;
        require(
            self.supported_task_ids
                .iter()
                .any(|task_id| task_id == &task_spec.task_id),
            "bound TaskSpec is not advertised",
        )
    }

    fn runtime_versions(&self) -> [&Option<String>; 6] {
        [
            &self.runtime.jax_version,
            &self.runtime.jaxlib_version,
            &self.runtime.jax_cuda_plugin_version,
            &self.runtime.mujoco_version,
            &self.runtime.mujoco_mjx_version,
            &self.runtime.warp_version,
        ]
    }

    fn validate_runtime_versions(&self) -> Result<(), AcceleratorContractError> {
        let expected = [
            &self.runtime_contract.packages.jax,
            &self.runtime_contract.packages.jaxlib,
            &self.runtime_contract.packages.jax_cuda_plugin,
            &self.runtime_contract.packages.mujoco,
            &self.runtime_contract.packages.mujoco_mjx,
            &self.runtime_contract.packages.warp_lang,
        ];
        require(
            self.runtime_versions()
                .iter()
                .zip(expected)
                .all(|(actual, expected)| actual.as_ref() == Some(expected)),
            "available runtime versions differ from pins",
        )
    }
}

fn validate_ordered_positive_unique(
    values: &[usize],
    label: &str,
) -> Result<(), AcceleratorContractError> {
    require(!values.is_empty(), format!("{label} is empty"))?;
    require(
        values.iter().all(|value| *value > 0),
        format!("{label} contains zero"),
    )?;
    require(
        values.windows(2).all(|pair| pair[0] < pair[1]),
        format!("{label} is not strictly ordered"),
    )
}

fn validate_ordered_identifiers(
    values: &[String],
    label: &str,
) -> Result<(), AcceleratorContractError> {
    require(!values.is_empty(), format!("{label} is empty"))?;
    for value in values {
        require_identifier(value, label)?;
    }
    let unique: BTreeSet<_> = values.iter().collect();
    require(
        unique.len() == values.len(),
        format!("{label} contains duplicates"),
    )?;
    require(
        values.windows(2).all(|pair| pair[0] < pair[1]),
        format!("{label} is not strictly ordered"),
    )
}

fn validate_relative_path(value: &str, label: &str) -> Result<(), AcceleratorContractError> {
    let normalized = value.replace('\\', "/");
    require(
        !normalized.is_empty()
            && !normalized.starts_with('/')
            && !normalized.contains(':')
            && normalized
                .split('/')
                .all(|part| !part.is_empty() && part != "." && part != ".."),
        format!("{label} is not a portable relative path"),
    )
}

fn validate_https_sources(values: &[String], label: &str) -> Result<(), AcceleratorContractError> {
    require(!values.is_empty(), format!("{label} is empty"))?;
    require(
        values
            .iter()
            .all(|value| value.starts_with("https://") && value.len() > 8),
        format!("{label} contains a non-HTTPS URL"),
    )
}

fn require_identifier(value: &str, label: &str) -> Result<(), AcceleratorContractError> {
    require(
        !value.is_empty()
            && value.len() <= 128
            && value
                .bytes()
                .all(|byte| byte.is_ascii_alphanumeric() || matches!(byte, b'_' | b'-' | b'.')),
        format!("{label} is invalid"),
    )
}

fn is_version(value: &str) -> bool {
    !value.is_empty()
        && value.len() <= 64
        && value
            .bytes()
            .all(|byte| byte.is_ascii_digit() || byte == b'.')
        && value.bytes().any(|byte| byte.is_ascii_digit())
}

fn require(condition: bool, message: impl Into<String>) -> Result<(), AcceleratorContractError> {
    if condition {
        Ok(())
    } else {
        Err(invalid(message))
    }
}

fn invalid(message: impl Into<String>) -> AcceleratorContractError {
    AcceleratorContractError::Invalid(message.into())
}

#[cfg(test)]
mod tests {
    use super::*;

    const MANIFEST: &str = include_str!("../../../adapters/mjx/accelerator.toml");
    const RUNTIME: &str = include_str!("../../../adapters/mjx/runtime.toml");
    const TASK: &str = include_str!("../../../adapters/mjx/fixtures/free-fall-task-spec-v1.json");
    const CAPABILITY: &str =
        include_str!("../../../tests/golden/accelerators/capability-report-v1.json");

    fn contracts() -> (
        AcceleratorManifest,
        AcceleratorRuntimeContract,
        TaskSpec,
        AcceleratorCapabilityReport,
    ) {
        (
            toml::from_str(MANIFEST).unwrap(),
            toml::from_str(RUNTIME).unwrap(),
            serde_json::from_str(TASK).unwrap(),
            serde_json::from_str(CAPABILITY).unwrap(),
        )
    }

    #[test]
    fn selected_capability_report_binds_to_all_contracts() {
        let (manifest, runtime, task, report) = contracts();
        report.validate_against(&manifest, &runtime, &task).unwrap();
    }

    #[test]
    fn status_and_runtime_claims_fail_closed() {
        let (manifest, runtime, task, mut report) = contracts();
        report.status = "available".into();
        assert!(report.validate_against(&manifest, &runtime, &task).is_err());
        report.status = "test_only".into();
        report.runtime.jax_backend = Some("gpu".into());
        assert!(report.validate().is_err());
    }

    #[test]
    fn available_status_requires_the_complete_host_contract() {
        let (_, _, _, mut report) = contracts();
        report.status = "available".into();
        report.runtime.python_version = "3.12.4".into();
        report.runtime.platform = "linux".into();
        report.runtime.machine = "x86_64".into();
        report.runtime.jax_version = Some(report.runtime_contract.packages.jax.clone());
        report.runtime.jaxlib_version = Some(report.runtime_contract.packages.jaxlib.clone());
        report.runtime.jax_cuda_plugin_version =
            Some(report.runtime_contract.packages.jax_cuda_plugin.clone());
        report.runtime.mujoco_version = Some(report.runtime_contract.packages.mujoco.clone());
        report.runtime.mujoco_mjx_version =
            Some(report.runtime_contract.packages.mujoco_mjx.clone());
        report.runtime.warp_version = Some(report.runtime_contract.packages.warp_lang.clone());
        report.runtime.jax_backend = Some("gpu".into());
        report.runtime.jax_devices = vec!["CUDA:0".into()];
        report.runtime.nvidia_driver_version = Some("580.42".into());
        report.validate().unwrap();

        report.runtime.nvidia_driver_version = Some("579.99".into());
        assert!(report.validate().is_err());
    }

    #[test]
    fn manifest_task_and_runtime_drift_fail_closed() {
        let (mut manifest, runtime, task, report) = contracts();
        manifest.supported_batch_widths.pop();
        assert!(report.validate_against(&manifest, &runtime, &task).is_err());

        let (manifest, mut runtime, task, report) = contracts();
        runtime.packages.jax = "0.10.3".into();
        assert!(report.validate_against(&manifest, &runtime, &task).is_err());

        let (manifest, runtime, mut task, report) = contracts();
        task.task_id = "rne.physics.other.v1".into();
        assert!(report.validate_against(&manifest, &runtime, &task).is_err());
    }

    #[test]
    fn unknown_report_field_is_rejected() {
        let mut value: serde_json::Value = serde_json::from_str(CAPABILITY).unwrap();
        value
            .as_object_mut()
            .unwrap()
            .insert("unknown".into(), true.into());
        assert!(serde_json::from_value::<AcceleratorCapabilityReport>(value).is_err());
    }
}
