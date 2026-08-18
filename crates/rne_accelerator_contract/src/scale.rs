//! Accelerator scale-report v1 reader.

use super::conformance::{
    canonical_python_json, normalized_model_sha256, sha256, task_spec_sha256,
    validate_evidence_runtime, validate_hex_sha256,
};
use super::{
    invalid, require, require_identifier, AcceleratorContractError, AcceleratorManifest,
    AcceleratorRuntimeContract, AcceleratorRuntimeProbe, ACCELERATOR_SCALE_REPORT_SCHEMA_VERSION,
};
use rne_ai::{derive_episode_seed, TaskSpec, TASK_SPEC_SCHEMA_VERSION};
use serde::{Deserialize, Serialize};
use serde_json::value::RawValue;

/// Stable scale-report discriminator.
pub const ACCELERATOR_SCALE_REPORT_KIND: &str = "rne_accelerator_scale_report";

const DEFAULT_ROOT_SEED: u64 = 42;
const MAX_STEPS: u64 = 1_000_000;
const MAX_REPORT_BYTES: usize = 1024 * 1024;
const SUPPORTED_BATCH_WIDTHS: [usize; 4] = [1, 16, 256, 4096];

/// One measured batch width in scale-report v1.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct AcceleratorScaleRun {
    /// Executed batch width.
    pub batch_width: usize,
    /// Exact `batch_width * measured_steps` transition count.
    pub transitions: u64,
    /// Measured monotonic duration in nanoseconds.
    pub elapsed_ns: u64,
    /// Recomputed transitions per second.
    pub throughput_transitions_s: f64,
    /// Same-build diagnostic lane-zero replay digest.
    pub lane_zero_replay_digest: u64,
    /// Lane-zero episode index after the measured steps.
    pub lane_zero_episode_index: u64,
    /// Deterministically derived seed for that episode index.
    pub lane_zero_episode_seed: u64,
}

/// Versioned batch-width and lane-isolation evidence for one accelerator.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct AcceleratorScaleReport {
    /// Stable report discriminator.
    pub kind: String,
    /// Scale-report schema version.
    pub schema_version: u32,
    /// Stable adapter identifier.
    pub adapter_id: String,
    /// `contract_test` or physical `accelerator` evidence class.
    pub evidence_class: String,
    /// Capability status observed before execution.
    pub backend_status: String,
    /// Numeric precision used by every run.
    pub precision: String,
    /// Timed API boundary.
    pub measurement_boundary: String,
    /// Bound task identifier.
    pub task_id: String,
    /// Bound TaskSpec schema.
    pub task_spec_schema: u32,
    /// Lowercase SHA-256 of canonical TaskSpec JSON.
    pub task_spec_sha256: String,
    /// Lowercase SHA-256 of LF-normalized model text.
    pub model_sha256: String,
    /// Root seed shared by all widths.
    pub root_seed: u64,
    /// Untimed warm-up steps per width.
    pub warmup_steps: u64,
    /// Timed measured steps per width.
    pub measured_steps: u64,
    /// Strictly increasing requested widths.
    pub requested_widths: Vec<usize>,
    /// Whether all promotion widths were measured.
    pub promotion_widths_complete: bool,
    /// Whether lane-zero replay identity is width-independent.
    pub lane_zero_digest_consistent: bool,
    /// Ordered measured runs, one per requested width.
    pub runs: Vec<AcceleratorScaleRun>,
    /// Exact runtime requirements used during probing.
    pub runtime_contract: AcceleratorRuntimeContract,
    /// Runtime values observed during probing.
    pub runtime: AcceleratorRuntimeProbe,
    /// Verdict recomputed from evidence class, widths, and lane identity.
    pub passed: bool,
    /// Lowercase SHA-256 of Python-canonical report JSON excluding this field.
    pub content_sha256: String,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct RawScaleRun {
    batch_width: usize,
    transitions: u64,
    elapsed_ns: u64,
    throughput_transitions_s: Box<RawValue>,
    lane_zero_replay_digest: u64,
    lane_zero_episode_index: u64,
    lane_zero_episode_seed: u64,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct RawScaleReport {
    kind: String,
    schema_version: u32,
    adapter_id: String,
    evidence_class: String,
    backend_status: String,
    precision: String,
    measurement_boundary: String,
    task_id: String,
    task_spec_schema: u32,
    task_spec_sha256: String,
    model_sha256: String,
    root_seed: u64,
    warmup_steps: u64,
    measured_steps: u64,
    requested_widths: Vec<usize>,
    promotion_widths_complete: bool,
    lane_zero_digest_consistent: bool,
    runs: Vec<RawScaleRun>,
    runtime_contract: AcceleratorRuntimeContract,
    runtime: AcceleratorRuntimeProbe,
    passed: bool,
    content_sha256: String,
}

impl AcceleratorScaleReport {
    /// Parses bounded JSON while preserving throughput number lexemes.
    pub fn from_json_slice(bytes: &[u8]) -> Result<Self, AcceleratorContractError> {
        require(
            !bytes.is_empty() && bytes.len() <= MAX_REPORT_BYTES,
            "scale report JSON size is invalid",
        )?;
        let raw: RawScaleReport = serde_json::from_slice(bytes)
            .map_err(|error| invalid(format!("parse scale report JSON: {error}")))?;
        let runs = raw
            .runs
            .into_iter()
            .map(|run| {
                Ok(AcceleratorScaleRun {
                    batch_width: run.batch_width,
                    transitions: run.transitions,
                    elapsed_ns: run.elapsed_ns,
                    throughput_transitions_s: parse_raw_f64(&run.throughput_transitions_s)?,
                    lane_zero_replay_digest: run.lane_zero_replay_digest,
                    lane_zero_episode_index: run.lane_zero_episode_index,
                    lane_zero_episode_seed: run.lane_zero_episode_seed,
                })
            })
            .collect::<Result<Vec<_>, AcceleratorContractError>>()?;
        let report = Self {
            kind: raw.kind,
            schema_version: raw.schema_version,
            adapter_id: raw.adapter_id,
            evidence_class: raw.evidence_class,
            backend_status: raw.backend_status,
            precision: raw.precision,
            measurement_boundary: raw.measurement_boundary,
            task_id: raw.task_id,
            task_spec_schema: raw.task_spec_schema,
            task_spec_sha256: raw.task_spec_sha256,
            model_sha256: raw.model_sha256,
            root_seed: raw.root_seed,
            warmup_steps: raw.warmup_steps,
            measured_steps: raw.measured_steps,
            requested_widths: raw.requested_widths,
            promotion_widths_complete: raw.promotion_widths_complete,
            lane_zero_digest_consistent: raw.lane_zero_digest_consistent,
            runs,
            runtime_contract: raw.runtime_contract,
            runtime: raw.runtime,
            passed: raw.passed,
            content_sha256: raw.content_sha256,
        };
        report.validate()?;
        Ok(report)
    }

    /// Validates report-local timing, lane identity, promotion, runtime, and digest invariants.
    pub fn validate(&self) -> Result<(), AcceleratorContractError> {
        require(
            self.kind == ACCELERATOR_SCALE_REPORT_KIND,
            "scale-report kind mismatch",
        )?;
        require(
            self.schema_version == ACCELERATOR_SCALE_REPORT_SCHEMA_VERSION,
            "scale-report schema mismatch",
        )?;
        require_identifier(&self.adapter_id, "scale adapter id")?;
        require(self.precision == "f64", "scale precision must be f64")?;
        require(
            self.measurement_boundary == "python_session_api",
            "scale measurement boundary mismatch",
        )?;
        require_identifier(&self.task_id, "scale task id")?;
        require(
            self.task_spec_schema == TASK_SPEC_SCHEMA_VERSION,
            "scale TaskSpec schema mismatch",
        )?;
        validate_hex_sha256(&self.task_spec_sha256, "scale TaskSpec digest")?;
        validate_hex_sha256(&self.model_sha256, "scale model digest")?;
        validate_hex_sha256(&self.content_sha256, "scale content digest")?;
        require(
            self.root_seed == DEFAULT_ROOT_SEED,
            "scale root seed mismatch",
        )?;
        require(
            self.warmup_steps <= MAX_STEPS && (1..=MAX_STEPS).contains(&self.measured_steps),
            "scale step counts are out of bounds",
        )?;
        require(
            !self.requested_widths.is_empty()
                && self
                    .requested_widths
                    .windows(2)
                    .all(|pair| pair[0] < pair[1])
                && self
                    .requested_widths
                    .iter()
                    .all(|width| SUPPORTED_BATCH_WIDTHS.contains(width)),
            "scale requested widths are not canonical",
        )?;
        require(
            self.runs.len() == self.requested_widths.len(),
            "scale run count differs from requested widths",
        )?;
        for (run, requested_width) in self.runs.iter().zip(&self.requested_widths) {
            require(
                run.batch_width == *requested_width,
                "scale run order differs from requested widths",
            )?;
            let transitions = u64::try_from(run.batch_width)
                .ok()
                .and_then(|width| width.checked_mul(self.measured_steps))
                .ok_or_else(|| invalid("scale transition count overflowed"))?;
            require(
                run.transitions == transitions,
                "scale transition count was not recomputed",
            )?;
            require(run.elapsed_ns > 0, "scale elapsed time must be positive")?;
            let expected_throughput =
                run.transitions as f64 * 1_000_000_000.0 / run.elapsed_ns as f64;
            require(
                run.throughput_transitions_s.is_finite()
                    && run.throughput_transitions_s == expected_throughput,
                "scale throughput was not recomputed",
            )?;
            require(
                run.lane_zero_episode_seed
                    == derive_episode_seed(self.root_seed, 0, run.lane_zero_episode_index),
                "scale lane-zero episode seed mismatch",
            )?;
        }
        let first_run = self
            .runs
            .first()
            .ok_or_else(|| invalid("scale report has no runs"))?;
        let lane_zero_consistent = self.runs.iter().all(|run| {
            run.lane_zero_replay_digest == first_run.lane_zero_replay_digest
                && run.lane_zero_episode_index == first_run.lane_zero_episode_index
                && run.lane_zero_episode_seed == first_run.lane_zero_episode_seed
        });
        require(
            self.lane_zero_digest_consistent == lane_zero_consistent,
            "scale lane-zero consistency verdict mismatch",
        )?;
        let promotion_complete = self.requested_widths == SUPPORTED_BATCH_WIDTHS;
        require(
            self.promotion_widths_complete == promotion_complete,
            "scale promotion-width verdict mismatch",
        )?;
        self.runtime_contract.validate()?;
        let expected_passed = match self.evidence_class.as_str() {
            "contract_test" => {
                require(
                    self.backend_status == "test_only",
                    "contract-test scale evidence is not test-only",
                )?;
                validate_evidence_runtime("test_only", &self.runtime_contract, &self.runtime)?;
                lane_zero_consistent
            }
            "accelerator" => {
                require(
                    self.backend_status == "available",
                    "accelerator scale evidence is not available",
                )?;
                validate_evidence_runtime("available", &self.runtime_contract, &self.runtime)?;
                lane_zero_consistent && promotion_complete
            }
            _ => return Err(invalid("unknown scale evidence class")),
        };
        require(self.passed == expected_passed, "scale verdict mismatch")?;
        require(
            self.content_sha256 == self.recomputed_content_sha256()?,
            "scale content digest mismatch",
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
            "scale adapter differs from manifest",
        )?;
        require(
            self.precision == manifest.precision,
            "scale precision differs from manifest",
        )?;
        require(
            self.task_spec_schema == manifest.task_spec_schema,
            "scale TaskSpec schema differs from manifest",
        )?;
        require(
            self.requested_widths
                .iter()
                .all(|width| manifest.supported_batch_widths.contains(width)),
            "scale width is not advertised by manifest",
        )?;
        require(
            &self.runtime_contract == runtime_contract,
            "scale runtime contract differs from selected contract",
        )?;
        require(
            self.task_id == task_spec.task_id,
            "scale task id differs from TaskSpec",
        )?;
        require(
            self.task_spec_sha256 == task_spec_sha256(task_spec)?,
            "scale TaskSpec digest mismatch",
        )?;
        require(
            self.model_sha256 == normalized_model_sha256(model_bytes)?,
            "scale model digest mismatch",
        )
    }

    fn recomputed_content_sha256(&self) -> Result<String, AcceleratorContractError> {
        let mut value = serde_json::to_value(self)
            .map_err(|error| invalid(format!("serialize scale report: {error}")))?;
        value
            .as_object_mut()
            .ok_or_else(|| invalid("scale report did not serialize as an object"))?
            .remove("content_sha256");
        Ok(sha256(canonical_python_json(&value)?.as_bytes()))
    }
}

fn parse_raw_f64(value: &RawValue) -> Result<f64, AcceleratorContractError> {
    let parsed = value
        .get()
        .parse::<f64>()
        .map_err(|_| invalid("scale throughput is not an f64"))?;
    require(parsed.is_finite(), "scale throughput is not finite")?;
    Ok(parsed)
}

#[cfg(test)]
mod tests {
    use super::*;

    const MANIFEST: &str = include_str!("../../../adapters/mjx/accelerator.toml");
    const RUNTIME: &str = include_str!("../../../adapters/mjx/runtime.toml");
    const TASK: &str = include_str!("../../../adapters/mjx/fixtures/free-fall-task-spec-v1.json");
    const MODEL: &[u8] = include_bytes!("../../../adapters/mjx/fixtures/free-fall-v1.xml");
    const REPORT: &str = include_str!("../../../tests/golden/accelerators/scale-report-v1.json");

    fn contracts() -> (
        AcceleratorManifest,
        AcceleratorRuntimeContract,
        TaskSpec,
        AcceleratorScaleReport,
    ) {
        (
            toml::from_str(MANIFEST).unwrap(),
            toml::from_str(RUNTIME).unwrap(),
            serde_json::from_str(TASK).unwrap(),
            AcceleratorScaleReport::from_json_slice(REPORT.as_bytes()).unwrap(),
        )
    }

    #[test]
    fn selected_scale_report_recomputes_every_binding() {
        let (manifest, runtime, task, report) = contracts();
        report
            .validate_against(&manifest, &runtime, &task, MODEL)
            .unwrap();
    }

    #[test]
    fn timing_width_and_verdict_tampering_fail_closed() {
        let (_, _, _, mut report) = contracts();
        report.runs[0].transitions += 1;
        assert!(report.validate().is_err());
        let (_, _, _, mut report) = contracts();
        report.runs[0].throughput_transitions_s += 1.0;
        assert!(report.validate().is_err());
        let (_, _, _, mut report) = contracts();
        report.promotion_widths_complete = true;
        assert!(report.validate().is_err());
        let (_, _, _, mut report) = contracts();
        report.passed = false;
        assert!(report.validate().is_err());
    }

    #[test]
    fn lane_identity_task_model_and_digest_tampering_fail_closed() {
        let (_, _, _, mut report) = contracts();
        report.runs[1].lane_zero_episode_seed ^= 1;
        assert!(report.validate().is_err());
        let (_, _, _, mut report) = contracts();
        report.runs[1].lane_zero_episode_index += 1;
        assert!(report.validate().is_err());
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
        report.content_sha256.replace_range(..1, "0");
        assert!(report.validate().is_err());
    }
}
