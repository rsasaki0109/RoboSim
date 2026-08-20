//! Accelerator conformance-report v1 reader.

use super::{
    invalid, require, require_identifier, AcceleratorContractError, AcceleratorManifest,
    AcceleratorRuntimeContract, AcceleratorRuntimeProbe,
    ACCELERATOR_CONFORMANCE_REPORT_SCHEMA_VERSION,
};
use rne_ai::{
    derive_episode_seed, TaskSpec, PORTABLE_BATCH_CHECKPOINT_VERSION, TASK_SPEC_SCHEMA_VERSION,
};
use serde::{Deserialize, Serialize};
use serde_json::{value::RawValue, Value};
use sha2::{Digest, Sha256};
use std::fmt::Write as _;

/// Stable conformance-report discriminator.
pub const ACCELERATOR_CONFORMANCE_REPORT_KIND: &str = "rne_accelerator_conformance_report";

const DEFAULT_ROOT_SEED: u64 = 42;
const MAX_STEPS: u64 = 1_000_000;
const POSITION_TOLERANCE_M: f64 = 1.0e-9;
const VELOCITY_TOLERANCE_M_S: f64 = 1.0e-9;
const INITIAL_POSITION_Y_M: f64 = 5.0;
const GRAVITY_M_S2: f64 = -9.81;
const MAX_REPORT_BYTES: usize = 1024 * 1024;

/// CPU reference values retained by conformance-report v1.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct AcceleratorConformanceReference {
    /// Reference backend identifier.
    pub backend_id: String,
    /// Reference case identifier.
    pub case_id: String,
    /// Reference integration semantics.
    pub integration: String,
    /// Reference vertical position in metres.
    pub position_y_m: f64,
    /// Reference vertical velocity in metres per second.
    pub velocity_y_m_s: f64,
}

/// Observed lane-zero values retained by conformance-report v1.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct AcceleratorConformanceActual {
    /// Observed vertical position in metres.
    pub position_y_m: f64,
    /// Observed vertical velocity in metres per second.
    pub velocity_y_m_s: f64,
    /// Deterministically derived lane-zero episode seed.
    pub lane_zero_episode_seed: u64,
    /// Same-build diagnostic replay digest for lane zero.
    pub lane_zero_replay_digest: u64,
}

/// Frozen absolute tolerances for conformance-report v1.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct AcceleratorConformanceTolerances {
    /// Maximum absolute position error in metres.
    pub position_delta_m: f64,
    /// Maximum absolute velocity error in metres per second.
    pub velocity_delta_m_s: f64,
}

/// Recomputed absolute errors in conformance-report v1.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct AcceleratorConformanceMetrics {
    /// Absolute position error in metres.
    pub position_delta_m: f64,
    /// Absolute velocity error in metres per second.
    pub velocity_delta_m_s: f64,
}

/// Explicit test-only divergence applied while producing a report.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct AcceleratorConformanceFaultInjection {
    /// Position bias added to lane zero in metres.
    pub position_bias_m: f64,
}

/// Versioned CPU-parity evidence for one accelerator execution.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct AcceleratorConformanceReport {
    /// Stable report discriminator.
    pub kind: String,
    /// Conformance-report schema version.
    pub schema_version: u32,
    /// Stable adapter identifier.
    pub adapter_id: String,
    /// `contract_test` or physical `accelerator` evidence class.
    pub evidence_class: String,
    /// Capability status observed before execution.
    pub backend_status: String,
    /// Numeric precision used by both paths.
    pub precision: String,
    /// Bound task identifier.
    pub task_id: String,
    /// Bound TaskSpec schema.
    pub task_spec_schema: u32,
    /// Lowercase SHA-256 of canonical TaskSpec JSON.
    pub task_spec_sha256: String,
    /// Lowercase SHA-256 of the LF-normalized model text.
    pub model_sha256: String,
    /// Root seed used for lane derivation.
    pub root_seed: u64,
    /// Executed batch width.
    pub batch_width: usize,
    /// Executed simulation steps.
    pub steps: u64,
    /// Independently reproducible CPU reference.
    pub reference: AcceleratorConformanceReference,
    /// Observed lane-zero result.
    pub actual: AcceleratorConformanceActual,
    /// Frozen named tolerances.
    pub tolerances: AcceleratorConformanceTolerances,
    /// Recomputed named errors.
    pub metrics: AcceleratorConformanceMetrics,
    /// Explicit divergence injected for a negative test.
    pub fault_injection: AcceleratorConformanceFaultInjection,
    /// Exact runtime requirements used during probing.
    pub runtime_contract: AcceleratorRuntimeContract,
    /// Runtime values observed during probing.
    pub runtime: AcceleratorRuntimeProbe,
    /// Portable batch checkpoint schema returned by the session.
    pub checkpoint_schema: u32,
    /// Verdict recomputed from metrics and tolerances.
    pub passed: bool,
    /// Lowercase SHA-256 of Python-canonical report JSON excluding this field.
    pub content_sha256: String,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct RawConformanceReference {
    backend_id: String,
    case_id: String,
    integration: String,
    position_y_m: Box<RawValue>,
    velocity_y_m_s: Box<RawValue>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct RawConformanceActual {
    position_y_m: Box<RawValue>,
    velocity_y_m_s: Box<RawValue>,
    lane_zero_episode_seed: u64,
    lane_zero_replay_digest: u64,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct RawConformancePair {
    position_delta_m: Box<RawValue>,
    velocity_delta_m_s: Box<RawValue>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct RawConformanceFaultInjection {
    position_bias_m: Box<RawValue>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct RawConformanceReport {
    kind: String,
    schema_version: u32,
    adapter_id: String,
    evidence_class: String,
    backend_status: String,
    precision: String,
    task_id: String,
    task_spec_schema: u32,
    task_spec_sha256: String,
    model_sha256: String,
    root_seed: u64,
    batch_width: usize,
    steps: u64,
    reference: RawConformanceReference,
    actual: RawConformanceActual,
    tolerances: RawConformancePair,
    metrics: RawConformancePair,
    fault_injection: RawConformanceFaultInjection,
    runtime_contract: AcceleratorRuntimeContract,
    runtime: AcceleratorRuntimeProbe,
    checkpoint_schema: u32,
    passed: bool,
    content_sha256: String,
}

impl AcceleratorConformanceReport {
    /// Parses bounded JSON while preserving numeric lexemes for Python/Rust parity.
    pub fn from_json_slice(bytes: &[u8]) -> Result<Self, AcceleratorContractError> {
        require(
            !bytes.is_empty() && bytes.len() <= MAX_REPORT_BYTES,
            "conformance report JSON size is invalid",
        )?;
        let raw: RawConformanceReport = serde_json::from_slice(bytes)
            .map_err(|error| invalid(format!("parse conformance report JSON: {error}")))?;
        let report = Self {
            kind: raw.kind,
            schema_version: raw.schema_version,
            adapter_id: raw.adapter_id,
            evidence_class: raw.evidence_class,
            backend_status: raw.backend_status,
            precision: raw.precision,
            task_id: raw.task_id,
            task_spec_schema: raw.task_spec_schema,
            task_spec_sha256: raw.task_spec_sha256,
            model_sha256: raw.model_sha256,
            root_seed: raw.root_seed,
            batch_width: raw.batch_width,
            steps: raw.steps,
            reference: AcceleratorConformanceReference {
                backend_id: raw.reference.backend_id,
                case_id: raw.reference.case_id,
                integration: raw.reference.integration,
                position_y_m: parse_raw_f64(&raw.reference.position_y_m)?,
                velocity_y_m_s: parse_raw_f64(&raw.reference.velocity_y_m_s)?,
            },
            actual: AcceleratorConformanceActual {
                position_y_m: parse_raw_f64(&raw.actual.position_y_m)?,
                velocity_y_m_s: parse_raw_f64(&raw.actual.velocity_y_m_s)?,
                lane_zero_episode_seed: raw.actual.lane_zero_episode_seed,
                lane_zero_replay_digest: raw.actual.lane_zero_replay_digest,
            },
            tolerances: AcceleratorConformanceTolerances {
                position_delta_m: parse_raw_f64(&raw.tolerances.position_delta_m)?,
                velocity_delta_m_s: parse_raw_f64(&raw.tolerances.velocity_delta_m_s)?,
            },
            metrics: AcceleratorConformanceMetrics {
                position_delta_m: parse_raw_f64(&raw.metrics.position_delta_m)?,
                velocity_delta_m_s: parse_raw_f64(&raw.metrics.velocity_delta_m_s)?,
            },
            fault_injection: AcceleratorConformanceFaultInjection {
                position_bias_m: parse_raw_f64(&raw.fault_injection.position_bias_m)?,
            },
            runtime_contract: raw.runtime_contract,
            runtime: raw.runtime,
            checkpoint_schema: raw.checkpoint_schema,
            passed: raw.passed,
            content_sha256: raw.content_sha256,
        };
        report.validate()?;
        Ok(report)
    }

    /// Validates report-local identity, runtime, metric, verdict, and digest invariants.
    pub fn validate(&self) -> Result<(), AcceleratorContractError> {
        require(
            self.kind == ACCELERATOR_CONFORMANCE_REPORT_KIND,
            "conformance-report kind mismatch",
        )?;
        require(
            self.schema_version == ACCELERATOR_CONFORMANCE_REPORT_SCHEMA_VERSION,
            "conformance-report schema mismatch",
        )?;
        require_identifier(&self.adapter_id, "conformance adapter id")?;
        require(self.precision == "f64", "conformance precision must be f64")?;
        require_identifier(&self.task_id, "conformance task id")?;
        require(
            self.task_spec_schema == TASK_SPEC_SCHEMA_VERSION,
            "conformance TaskSpec schema mismatch",
        )?;
        validate_hex_sha256(&self.task_spec_sha256, "TaskSpec digest")?;
        validate_hex_sha256(&self.model_sha256, "model digest")?;
        validate_hex_sha256(&self.content_sha256, "content digest")?;
        require(
            self.root_seed == DEFAULT_ROOT_SEED,
            "conformance root seed mismatch",
        )?;
        require(
            self.batch_width > 0,
            "conformance batch width must be positive",
        )?;
        require(
            (1..=MAX_STEPS).contains(&self.steps),
            "conformance steps are out of bounds",
        )?;
        require(
            self.checkpoint_schema == PORTABLE_BATCH_CHECKPOINT_VERSION,
            "conformance checkpoint schema mismatch",
        )?;
        require(
            self.reference.backend_id == "mujoco_cpu"
                && self.reference.case_id == "mujoco.rigid_body.free_fall"
                && self.reference.integration == "f64_semi_implicit_euler",
            "conformance reference identity mismatch",
        )?;
        for value in [
            self.reference.position_y_m,
            self.reference.velocity_y_m_s,
            self.actual.position_y_m,
            self.actual.velocity_y_m_s,
            self.metrics.position_delta_m,
            self.metrics.velocity_delta_m_s,
            self.fault_injection.position_bias_m,
        ] {
            require(
                value.is_finite(),
                "conformance report contains a non-finite number",
            )?;
        }
        require(
            self.tolerances.position_delta_m == POSITION_TOLERANCE_M
                && self.tolerances.velocity_delta_m_s == VELOCITY_TOLERANCE_M_S,
            "conformance tolerances drifted",
        )?;
        let position_delta_m = (self.actual.position_y_m - self.reference.position_y_m).abs();
        let velocity_delta_m_s = (self.actual.velocity_y_m_s - self.reference.velocity_y_m_s).abs();
        require(
            self.metrics.position_delta_m == position_delta_m,
            format!(
                "conformance position metric was not recomputed: stored={:?}, recomputed={position_delta_m:?}",
                self.metrics.position_delta_m
            ),
        )?;
        require(
            self.metrics.velocity_delta_m_s == velocity_delta_m_s,
            format!(
                "conformance velocity metric was not recomputed: stored={:?}, recomputed={velocity_delta_m_s:?}",
                self.metrics.velocity_delta_m_s
            ),
        )?;
        let expected_passed = position_delta_m <= POSITION_TOLERANCE_M
            && velocity_delta_m_s <= VELOCITY_TOLERANCE_M_S;
        require(
            self.passed == expected_passed,
            "conformance verdict mismatch",
        )?;
        require(
            self.actual.lane_zero_episode_seed == derive_episode_seed(self.root_seed, 0, 0),
            "conformance lane-zero seed mismatch",
        )?;
        self.runtime_contract.validate()?;
        match self.evidence_class.as_str() {
            "contract_test" => {
                require(
                    self.backend_status == "test_only",
                    "contract-test evidence is not test-only",
                )?;
                validate_evidence_runtime("test_only", &self.runtime_contract, &self.runtime)?;
            }
            "accelerator" => {
                require(
                    self.backend_status == "available",
                    "accelerator evidence is not available",
                )?;
                validate_evidence_runtime("available", &self.runtime_contract, &self.runtime)?;
            }
            _ => return Err(invalid("unknown conformance evidence class")),
        }
        require(
            self.content_sha256 == self.recomputed_content_sha256()?,
            "conformance content digest mismatch",
        )
    }

    /// Binds a valid report to its exact manifest, runtime contract, TaskSpec, and model bytes.
    pub fn validate_against(
        &self,
        manifest: &AcceleratorManifest,
        runtime_contract: &AcceleratorRuntimeContract,
        task_spec: &TaskSpec,
        model_bytes: &[u8],
    ) -> Result<(), AcceleratorContractError> {
        self.validate()?;
        manifest.validate()?;
        runtime_contract.validate()?;
        task_spec
            .validate()
            .map_err(|error| invalid(format!("bound TaskSpec is invalid: {error}")))?;
        require(
            self.adapter_id == manifest.id,
            "conformance adapter differs from manifest",
        )?;
        require(
            self.precision == manifest.precision,
            "conformance precision differs from manifest",
        )?;
        require(
            self.task_spec_schema == manifest.task_spec_schema,
            "conformance TaskSpec schema differs from manifest",
        )?;
        require(
            self.checkpoint_schema == manifest.batch_checkpoint_schema,
            "conformance checkpoint schema differs from manifest",
        )?;
        require(
            manifest.supported_batch_widths.contains(&self.batch_width),
            "conformance batch width is not advertised",
        )?;
        require(
            &self.runtime_contract == runtime_contract,
            "conformance runtime contract differs from selected contract",
        )?;
        require(
            self.task_id == task_spec.task_id,
            "conformance task id differs from TaskSpec",
        )?;
        require(
            self.task_spec_sha256 == task_spec_sha256(task_spec)?,
            "conformance TaskSpec digest mismatch",
        )?;
        require(
            self.model_sha256 == normalized_model_sha256(model_bytes)?,
            "conformance model digest mismatch",
        )?;
        if let Some(max_steps) = task_spec.termination.max_episode_steps {
            require(
                self.steps <= max_steps,
                "conformance steps exceed TaskSpec episode bound",
            )?;
        }
        let steps = self.steps as f64;
        let expected_velocity_y_m_s = GRAVITY_M_S2 * task_spec.control_step_s * steps;
        let expected_position_y_m = INITIAL_POSITION_Y_M
            + GRAVITY_M_S2
                * task_spec.control_step_s
                * task_spec.control_step_s
                * steps
                * (steps + 1.0)
                / 2.0;
        require(
            self.reference.position_y_m == expected_position_y_m
                && self.reference.velocity_y_m_s == expected_velocity_y_m_s,
            "conformance CPU reference was not recomputed from TaskSpec",
        )
    }

    fn recomputed_content_sha256(&self) -> Result<String, AcceleratorContractError> {
        let mut value = serde_json::to_value(self)
            .map_err(|error| invalid(format!("serialize conformance report: {error}")))?;
        value
            .as_object_mut()
            .ok_or_else(|| invalid("conformance report did not serialize as an object"))?
            .remove("content_sha256");
        Ok(sha256(canonical_python_json(&value)?.as_bytes()))
    }
}

pub(super) fn validate_evidence_runtime(
    status: &str,
    contract: &AcceleratorRuntimeContract,
    runtime: &AcceleratorRuntimeProbe,
) -> Result<(), AcceleratorContractError> {
    match status {
        "test_only" => require(
            runtime.jax_backend.is_none()
                && runtime.jax_devices.is_empty()
                && runtime.jax_version.is_none()
                && runtime.jaxlib_version.is_none()
                && runtime.jax_cuda_plugin_version.is_none()
                && runtime.mujoco_version.is_none()
                && runtime.mujoco_mjx_version.is_none()
                && runtime.warp_version.is_none(),
            "test-only conformance claims accelerator runtime",
        ),
        "available" => {
            let python_prefix = format!("{}.", contract.python);
            let driver_major = runtime
                .nvidia_driver_version
                .as_deref()
                .and_then(|version| version.split('.').next())
                .and_then(|major| major.parse::<u32>().ok());
            let actual_versions = [
                runtime.jax_version.as_ref(),
                runtime.jaxlib_version.as_ref(),
                runtime.jax_cuda_plugin_version.as_ref(),
                runtime.mujoco_version.as_ref(),
                runtime.mujoco_mjx_version.as_ref(),
                runtime.warp_version.as_ref(),
            ];
            let expected_versions = [
                &contract.packages.jax,
                &contract.packages.jaxlib,
                &contract.packages.jax_cuda_plugin,
                &contract.packages.mujoco,
                &contract.packages.mujoco_mjx,
                &contract.packages.warp_lang,
            ];
            require(
                runtime.platform == contract.operating_system
                    && runtime.machine == contract.architecture
                    && (runtime.python_version == contract.python
                        || runtime.python_version.starts_with(&python_prefix))
                    && driver_major.is_some_and(|major| major >= contract.nvidia_driver_minimum)
                    && runtime.jax_backend.as_deref() == Some("gpu")
                    && !runtime.jax_devices.is_empty()
                    && actual_versions
                        .into_iter()
                        .zip(expected_versions)
                        .all(|(actual, expected)| actual == Some(expected)),
                "available conformance runtime differs from contract",
            )
        }
        _ => Err(invalid("unknown conformance runtime status")),
    }
}

fn parse_raw_f64(value: &RawValue) -> Result<f64, AcceleratorContractError> {
    let parsed = value
        .get()
        .parse::<f64>()
        .map_err(|_| invalid("conformance numeric field is not an f64"))?;
    require(
        parsed.is_finite(),
        "conformance numeric field is not finite",
    )?;
    Ok(parsed)
}

pub(super) fn task_spec_sha256(task_spec: &TaskSpec) -> Result<String, AcceleratorContractError> {
    let value = serde_json::to_value(task_spec)
        .map_err(|error| invalid(format!("serialize bound TaskSpec: {error}")))?;
    Ok(sha256(canonical_python_json(&value)?.as_bytes()))
}

pub(super) fn normalized_model_sha256(
    model_bytes: &[u8],
) -> Result<String, AcceleratorContractError> {
    require(
        model_bytes.len() <= 1024 * 1024,
        "accelerator model exceeds 1 MiB",
    )?;
    let text =
        std::str::from_utf8(model_bytes).map_err(|_| invalid("accelerator model is not UTF-8"))?;
    Ok(sha256(text.replace("\r\n", "\n").as_bytes()))
}

pub(super) fn validate_hex_sha256(
    value: &str,
    label: &str,
) -> Result<(), AcceleratorContractError> {
    require(
        value.len() == 64
            && value
                .bytes()
                .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte)),
        format!("{label} is not 64 lowercase hexadecimal characters"),
    )
}

pub(super) fn sha256(bytes: &[u8]) -> String {
    format!("{:x}", Sha256::digest(bytes))
}

pub(super) fn canonical_python_json(value: &Value) -> Result<String, AcceleratorContractError> {
    let mut output = String::new();
    write_canonical(value, &mut output)?;
    Ok(output)
}

fn write_canonical(value: &Value, output: &mut String) -> Result<(), AcceleratorContractError> {
    match value {
        Value::Null => output.push_str("null"),
        Value::Bool(value) => output.push_str(if *value { "true" } else { "false" }),
        Value::Number(value) => output.push_str(&python_number(value.to_string())?),
        Value::String(value) => output.push_str(
            &serde_json::to_string(value)
                .map_err(|error| invalid(format!("serialize canonical string: {error}")))?,
        ),
        Value::Array(values) => {
            output.push('[');
            for (index, value) in values.iter().enumerate() {
                if index > 0 {
                    output.push(',');
                }
                write_canonical(value, output)?;
            }
            output.push(']');
        }
        Value::Object(values) => {
            output.push('{');
            let mut entries: Vec<_> = values.iter().collect();
            entries.sort_unstable_by_key(|(key, _)| *key);
            for (index, (key, value)) in entries.into_iter().enumerate() {
                if index > 0 {
                    output.push(',');
                }
                output.push_str(
                    &serde_json::to_string(key)
                        .map_err(|error| invalid(format!("serialize canonical key: {error}")))?,
                );
                output.push(':');
                write_canonical(value, output)?;
            }
            output.push('}');
        }
    }
    Ok(())
}

fn python_number(mut number: String) -> Result<String, AcceleratorContractError> {
    let Some(index) = number.find(['e', 'E']) else {
        return Ok(number);
    };
    let exponent = number[index + 1..]
        .parse::<i32>()
        .map_err(|_| invalid("canonical JSON exponent is invalid"))?;
    number.truncate(index);
    write!(&mut number, "e{exponent:+03}")
        .map_err(|_| invalid("format canonical JSON exponent"))?;
    Ok(number)
}

#[cfg(test)]
mod tests {
    use super::*;

    const MANIFEST: &str = include_str!("../../../adapters/mjx/accelerator.toml");
    const RUNTIME: &str = include_str!("../../../adapters/mjx/runtime.toml");
    const TASK: &str = include_str!("../../../adapters/mjx/fixtures/free-fall-task-spec-v1.json");
    const MODEL: &[u8] = include_bytes!("../../../adapters/mjx/fixtures/free-fall-v1.xml");
    const REPORT: &str =
        include_str!("../../../tests/golden/accelerators/conformance-report-v1.json");

    fn contracts() -> (
        AcceleratorManifest,
        AcceleratorRuntimeContract,
        TaskSpec,
        AcceleratorConformanceReport,
    ) {
        (
            toml::from_str(MANIFEST).unwrap(),
            toml::from_str(RUNTIME).unwrap(),
            serde_json::from_str(TASK).unwrap(),
            AcceleratorConformanceReport::from_json_slice(REPORT.as_bytes()).unwrap(),
        )
    }

    #[test]
    fn selected_conformance_report_recomputes_every_binding() {
        let (manifest, runtime, task, report) = contracts();
        report
            .validate_against(&manifest, &runtime, &task, MODEL)
            .unwrap();
    }

    #[test]
    fn metric_verdict_seed_and_digest_tampering_fail_closed() {
        let (_, _, _, mut report) = contracts();
        report.metrics.position_delta_m = 0.0;
        assert!(report.validate().is_err());
        let (_, _, _, mut report) = contracts();
        report.passed = false;
        assert!(report.validate().is_err());
        let (_, _, _, mut report) = contracts();
        report.actual.lane_zero_episode_seed ^= 1;
        assert!(report.validate().is_err());
        let (_, _, _, mut report) = contracts();
        report.content_sha256.replace_range(..1, "0");
        assert!(report.validate().is_err());
    }

    #[test]
    fn task_model_and_runtime_status_tampering_fail_closed() {
        let (manifest, runtime, mut task, report) = contracts();
        task.task_id = "rne.physics.other.v1".into();
        assert!(report
            .validate_against(&manifest, &runtime, &task, MODEL)
            .is_err());
        let (manifest, runtime, task, report) = contracts();
        assert!(report
            .validate_against(&manifest, &runtime, &task, b"<mujoco/>")
            .is_err());
        let (_, _, _, mut report) = contracts();
        report.evidence_class = "accelerator".into();
        report.backend_status = "available".into();
        assert!(report.validate().is_err());
    }

    #[test]
    fn python_canonical_exponents_are_padded() {
        assert_eq!(python_number("1e-9".into()).unwrap(), "1e-09");
        assert_eq!(python_number("1e20".into()).unwrap(), "1e+20");
        assert_eq!(python_number("1e-15".into()).unwrap(), "1e-15");
    }
}
