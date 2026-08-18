//! Accelerator JSONL protocol-v1 compatibility transcript reader.

use super::conformance::{task_spec_sha256, validate_hex_sha256};
use super::{
    invalid, require, AcceleratorCapabilityReport, AcceleratorContractError, AcceleratorManifest,
    AcceleratorRuntimeContract, ACCELERATOR_PROTOCOL_SCHEMA_VERSION,
};
use rne_ai::{
    derive_episode_seed, EpisodeSeedStrategy, PortableBatchCheckpoint, PortableBatchOperation,
    TaskSpec, PORTABLE_BATCH_CHECKPOINT_VERSION, TASK_SPEC_SCHEMA_VERSION,
};
use serde::{Deserialize, Serialize};
use serde_json::{Map, Value};
use std::collections::BTreeSet;

/// Stable protocol-transcript discriminator.
pub const ACCELERATOR_PROTOCOL_TRANSCRIPT_KIND: &str = "rne_accelerator_protocol_transcript";
/// Current protocol-transcript fixture schema.
pub const ACCELERATOR_PROTOCOL_TRANSCRIPT_SCHEMA_VERSION: u32 = 1;

const MAX_TRANSCRIPT_BYTES: usize = 1024 * 1024;
const SESSION_ID: &str = "contract";
const OPERATIONS: [&str; 9] = [
    "probe",
    "create",
    "reset_lanes",
    "step",
    "checkpoint",
    "restore",
    "close",
    "unsupported_v1_fixture",
    "shutdown",
];

/// One exact request and its correlated response.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct AcceleratorProtocolFrame {
    /// Request object sent as one JSONL line.
    pub request: Value,
    /// Response object returned as one JSONL line.
    pub response: Value,
}

/// Deterministic lifecycle transcript covering every accelerator operation family.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct AcceleratorProtocolTranscript {
    /// Stable transcript discriminator.
    pub kind: String,
    /// Transcript fixture schema version.
    pub schema_version: u32,
    /// Frozen accelerator protocol schema.
    pub protocol_schema: u32,
    /// Stable adapter identifier.
    pub adapter_id: String,
    /// Bound task identifier.
    pub task_id: String,
    /// Bound TaskSpec schema.
    pub task_spec_schema: u32,
    /// Lowercase SHA-256 of canonical TaskSpec JSON.
    pub task_spec_sha256: String,
    /// Root seed used by the transcript session.
    pub root_seed: u64,
    /// Batch width used by the transcript session.
    pub batch_width: usize,
    /// Ordered lifecycle exchanges.
    pub frames: Vec<AcceleratorProtocolFrame>,
}

impl AcceleratorProtocolTranscript {
    /// Parses and validates a bounded transcript JSON document.
    pub fn from_json_slice(bytes: &[u8]) -> Result<Self, AcceleratorContractError> {
        require(
            !bytes.is_empty() && bytes.len() <= MAX_TRANSCRIPT_BYTES,
            "accelerator protocol transcript JSON size is invalid",
        )?;
        let transcript: Self = serde_json::from_slice(bytes)
            .map_err(|error| invalid(format!("parse accelerator protocol transcript: {error}")))?;
        transcript.validate()?;
        Ok(transcript)
    }

    /// Validates operation order, exact envelopes, correlation, and replay relationships.
    pub fn validate(&self) -> Result<(), AcceleratorContractError> {
        require(
            self.kind == ACCELERATOR_PROTOCOL_TRANSCRIPT_KIND,
            "accelerator protocol transcript kind mismatch",
        )?;
        require(
            self.schema_version == ACCELERATOR_PROTOCOL_TRANSCRIPT_SCHEMA_VERSION,
            "accelerator protocol transcript schema mismatch",
        )?;
        require(
            self.protocol_schema == ACCELERATOR_PROTOCOL_SCHEMA_VERSION,
            "accelerator protocol schema mismatch",
        )?;
        require(
            self.task_spec_schema == TASK_SPEC_SCHEMA_VERSION,
            "accelerator protocol TaskSpec schema mismatch",
        )?;
        validate_hex_sha256(&self.task_spec_sha256, "protocol TaskSpec digest")?;
        require(
            self.batch_width == 1,
            "protocol transcript batch width must be one",
        )?;
        require(
            self.frames.len() == OPERATIONS.len(),
            "protocol transcript frame count mismatch",
        )?;
        for (request_id, (frame, operation)) in self.frames.iter().zip(OPERATIONS).enumerate() {
            validate_request_envelope(
                &frame.request,
                request_id as u64,
                operation,
                self.protocol_schema,
            )?;
            validate_response_envelope(
                &frame.response,
                request_id as u64,
                request_id != 7,
                self.protocol_schema,
            )?;
        }
        Ok(())
    }

    /// Binds the transcript to its manifest, runtime contract, and exact TaskSpec.
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
            "protocol transcript adapter differs from manifest",
        )?;
        require(
            self.protocol_schema == manifest.protocol_schema,
            "protocol transcript schema differs from manifest",
        )?;
        require(
            self.task_id == task_spec.task_id,
            "protocol transcript task differs from TaskSpec",
        )?;
        require(
            self.task_spec_sha256 == task_spec_sha256(task_spec)?,
            "protocol transcript TaskSpec digest mismatch",
        )?;
        require(
            manifest.supported_batch_widths.contains(&self.batch_width),
            "protocol transcript batch width is not advertised",
        )?;

        let probe: AcceleratorCapabilityReport = serde_json::from_value(result(0, self)?)
            .map_err(|error| invalid(format!("parse transcript capability report: {error}")))?;
        probe.validate_against(manifest, runtime_contract, task_spec)?;

        self.validate_create(task_spec)?;
        self.validate_reset_and_step()?;
        let checkpoint = self.validate_checkpoint(task_spec)?;
        self.validate_restore(&checkpoint)?;
        self.validate_close_error_shutdown()?;
        Ok(())
    }

    fn validate_create(&self, task_spec: &TaskSpec) -> Result<(), AcceleratorContractError> {
        let request = object(&self.frames[1].request, "create request")?;
        exact_keys(
            request,
            &[
                "kind",
                "schema_version",
                "request_id",
                "operation",
                "session_id",
                "task_spec",
                "root_seed",
                "batch_width",
                "auto_reset",
            ],
            "create request",
        )?;
        require(
            string(request, "session_id")? == SESSION_ID,
            "create session id mismatch",
        )?;
        require(
            unsigned(request, "root_seed")? == self.root_seed,
            "create root seed mismatch",
        )?;
        require(
            unsigned(request, "batch_width")? == self.batch_width as u64,
            "create batch width mismatch",
        )?;
        require(
            request.get("auto_reset") == Some(&Value::Bool(false)),
            "create auto-reset mismatch",
        )?;
        let embedded: TaskSpec = serde_json::from_value(
            request
                .get("task_spec")
                .cloned()
                .ok_or_else(|| invalid("create request omitted TaskSpec"))?,
        )
        .map_err(|error| invalid(format!("parse create TaskSpec: {error}")))?;
        embedded
            .validate()
            .map_err(|error| invalid(format!("create TaskSpec is invalid: {error}")))?;
        require(
            &embedded == task_spec,
            "create TaskSpec differs from binding",
        )?;
        validate_state_result(&result(1, self)?, self.root_seed, 0, true, true, false)
    }

    fn validate_reset_and_step(&self) -> Result<(), AcceleratorContractError> {
        let reset_request = object(&self.frames[2].request, "reset request")?;
        exact_keys(
            reset_request,
            &[
                "kind",
                "schema_version",
                "request_id",
                "operation",
                "session_id",
                "lane_ids",
            ],
            "reset request",
        )?;
        require(
            string(reset_request, "session_id")? == SESSION_ID,
            "reset session id mismatch",
        )?;
        require(
            reset_request.get("lane_ids") == Some(&serde_json::json!([0])),
            "reset lane ids mismatch",
        )?;
        validate_state_result(&result(2, self)?, self.root_seed, 1, true, false, false)?;

        let step_request = object(&self.frames[3].request, "step request")?;
        exact_keys(
            step_request,
            &[
                "kind",
                "schema_version",
                "request_id",
                "operation",
                "session_id",
                "actions",
            ],
            "step request",
        )?;
        require(
            string(step_request, "session_id")? == SESSION_ID,
            "step session id mismatch",
        )?;
        require(
            step_request.get("actions") == Some(&serde_json::json!([[0.0]])),
            "step action mismatch",
        )?;
        validate_state_result(&result(3, self)?, self.root_seed, 1, false, false, true)
    }

    fn validate_checkpoint(&self, task_spec: &TaskSpec) -> Result<Value, AcceleratorContractError> {
        let request = object(&self.frames[4].request, "checkpoint request")?;
        exact_keys(
            request,
            &[
                "kind",
                "schema_version",
                "request_id",
                "operation",
                "session_id",
            ],
            "checkpoint request",
        )?;
        require(
            string(request, "session_id")? == SESSION_ID,
            "checkpoint session id mismatch",
        )?;
        let value = result(4, self)?;
        let checkpoint: PortableBatchCheckpoint<Vec<f64>> =
            serde_json::from_value(value.clone())
                .map_err(|error| invalid(format!("parse transcript checkpoint: {error}")))?;
        require(
            checkpoint.schema_version == PORTABLE_BATCH_CHECKPOINT_VERSION,
            "transcript checkpoint schema mismatch",
        )?;
        require(
            checkpoint.seed == self.root_seed
                && checkpoint.num_envs == self.batch_width
                && !checkpoint.auto_reset
                && checkpoint.seed_strategy == Some(EpisodeSeedStrategy::SplitMix64LaneEpisodeV1)
                && checkpoint.task_spec.as_ref() == Some(task_spec),
            "transcript checkpoint identity mismatch",
        )?;
        require(
            checkpoint.lanes.len() == 1,
            "transcript checkpoint lane count mismatch",
        )?;
        let lane = &checkpoint.lanes[0];
        require(
            lane.lane_id == 0
                && lane.episode_index == 1
                && lane.episode_seed == Some(derive_episode_seed(self.root_seed, 0, 1))
                && !lane.pending_auto_reset,
            "transcript checkpoint lane metadata mismatch",
        )?;
        require(
            checkpoint.operations.len() == 2,
            "transcript checkpoint operation count mismatch",
        )?;
        match &checkpoint.operations[0] {
            PortableBatchOperation::ResetLanes { lane_ids } => {
                require(lane_ids == &[0], "checkpoint reset operation mismatch")?;
            }
            _ => return Err(invalid("checkpoint first operation is not reset_lanes")),
        }
        match &checkpoint.operations[1] {
            PortableBatchOperation::Step { actions } => {
                require(
                    actions == &[vec![0.0]],
                    "checkpoint step operation mismatch",
                )?;
            }
            _ => return Err(invalid("checkpoint second operation is not step")),
        }
        let step_result_value = result(3, self)?;
        let step_result = object(&step_result_value, "step result")?;
        require(
            step_result.get("replay_digest").and_then(Value::as_u64)
                == Some(checkpoint.replay_digest)
                && first_u64(step_result, "lane_replay_digests")? == lane.replay_digest,
            "checkpoint replay digest differs from step result",
        )?;
        Ok(value)
    }

    fn validate_restore(&self, checkpoint: &Value) -> Result<(), AcceleratorContractError> {
        let request = object(&self.frames[5].request, "restore request")?;
        exact_keys(
            request,
            &[
                "kind",
                "schema_version",
                "request_id",
                "operation",
                "session_id",
                "checkpoint",
            ],
            "restore request",
        )?;
        require(
            string(request, "session_id")? == SESSION_ID,
            "restore session id mismatch",
        )?;
        require(
            request.get("checkpoint") == Some(checkpoint),
            "restore checkpoint differs from response",
        )?;
        validate_state_result(&result(5, self)?, self.root_seed, 1, false, false, false)?;
        let restored_value = result(5, self)?;
        let stepped_value = result(3, self)?;
        let restored = object(&restored_value, "restore result")?;
        let stepped = object(&stepped_value, "step result")?;
        for field in [
            "episode_indices",
            "episode_seeds",
            "lane_ids",
            "lane_replay_digests",
            "observations",
            "replay_digest",
        ] {
            require(
                restored.get(field) == stepped.get(field),
                format!("restore field {field} differs from checkpointed step"),
            )?;
        }
        Ok(())
    }

    fn validate_close_error_shutdown(&self) -> Result<(), AcceleratorContractError> {
        let close_request = object(&self.frames[6].request, "close request")?;
        exact_keys(
            close_request,
            &[
                "kind",
                "schema_version",
                "request_id",
                "operation",
                "session_id",
            ],
            "close request",
        )?;
        require(
            string(close_request, "session_id")? == SESSION_ID,
            "close session id mismatch",
        )?;
        require(
            result(6, self)? == serde_json::json!({"closed": true, "session_id": SESSION_ID}),
            "close result mismatch",
        )?;
        let error_response = object(&self.frames[7].response, "unsupported response")?;
        let error = object(
            error_response
                .get("error")
                .ok_or_else(|| invalid("unsupported response omitted error"))?,
            "unsupported error",
        )?;
        exact_keys(error, &["code", "message", "details"], "unsupported error")?;
        require(
            string(error, "code")? == "unsupported_operation",
            "unsupported error code mismatch",
        )?;
        require(
            !string(error, "message")?.is_empty(),
            "unsupported error message is empty",
        )?;
        require(
            error.get("details") == Some(&serde_json::json!({})),
            "unsupported error details mismatch",
        )?;
        require(
            result(8, self)? == serde_json::json!({"shutdown": true}),
            "shutdown result mismatch",
        )
    }
}

pub(crate) fn validate_request_envelope(
    value: &Value,
    request_id: u64,
    operation: &str,
    schema: u32,
) -> Result<(), AcceleratorContractError> {
    let request = object(value, "protocol request")?;
    require(
        string(request, "kind")? == "rne_accelerator_request",
        "request kind mismatch",
    )?;
    require(
        unsigned(request, "schema_version")? == u64::from(schema),
        "request schema mismatch",
    )?;
    require(
        unsigned(request, "request_id")? == request_id,
        "request id mismatch",
    )?;
    require(
        string(request, "operation")? == operation,
        "request operation mismatch",
    )
}

pub(crate) fn validate_response_envelope(
    value: &Value,
    request_id: u64,
    success: bool,
    schema: u32,
) -> Result<(), AcceleratorContractError> {
    let response = object(value, "protocol response")?;
    let expected = if success {
        ["kind", "schema_version", "request_id", "ok", "result"].as_slice()
    } else {
        ["kind", "schema_version", "request_id", "ok", "error"].as_slice()
    };
    exact_keys(response, expected, "protocol response")?;
    require(
        string(response, "kind")? == "rne_accelerator_response",
        "response kind mismatch",
    )?;
    require(
        unsigned(response, "schema_version")? == u64::from(schema),
        "response schema mismatch",
    )?;
    require(
        unsigned(response, "request_id")? == request_id,
        "response id mismatch",
    )?;
    require(
        response.get("ok") == Some(&Value::Bool(success)),
        "response success flag mismatch",
    )
}

fn validate_state_result(
    value: &Value,
    root_seed: u64,
    episode_index: u64,
    reset: bool,
    has_session_id: bool,
    stepped: bool,
) -> Result<(), AcceleratorContractError> {
    let state = object(value, "protocol state result")?;
    let mut keys = vec![
        "lane_ids",
        "episode_indices",
        "episode_seeds",
        "reset",
        "observations",
        "lane_replay_digests",
        "replay_digest",
    ];
    if has_session_id {
        keys.push("session_id");
    }
    if stepped {
        keys.extend(["rewards", "terminated", "truncated"]);
    }
    exact_keys(state, &keys, "protocol state result")?;
    if has_session_id {
        require(
            string(state, "session_id")? == SESSION_ID,
            "state session id mismatch",
        )?;
    }
    require(
        state.get("lane_ids") == Some(&serde_json::json!([0])),
        "state lane ids mismatch",
    )?;
    require(
        state.get("episode_indices") == Some(&serde_json::json!([episode_index])),
        "state episode index mismatch",
    )?;
    require(
        state.get("episode_seeds")
            == Some(&serde_json::json!([derive_episode_seed(
                root_seed,
                0,
                episode_index
            )])),
        "state episode seed mismatch",
    )?;
    require(
        state.get("reset") == Some(&serde_json::json!([reset])),
        "state reset mask mismatch",
    )?;
    let observations = array(state, "observations")?;
    require(
        observations.len() == 1,
        "state observation lane count mismatch",
    )?;
    let observation = observations[0]
        .as_array()
        .ok_or_else(|| invalid("state observation is not an array"))?;
    require(
        observation.len() == 2
            && observation
                .iter()
                .all(|value| value.as_f64().is_some_and(f64::is_finite)),
        "state observation shape or value is invalid",
    )?;
    require(
        first_u64(state, "lane_replay_digests").is_ok(),
        "state lane digest is invalid",
    )?;
    require(
        unsigned(state, "replay_digest").is_ok(),
        "state replay digest is invalid",
    )?;
    if stepped {
        require(
            array(state, "rewards")?.len() == 1
                && first_f64(state, "rewards")?.is_finite()
                && state.get("terminated") == Some(&serde_json::json!([false]))
                && state.get("truncated") == Some(&serde_json::json!([false])),
            "step result fields are invalid",
        )?;
    }
    Ok(())
}

fn result(
    index: usize,
    transcript: &AcceleratorProtocolTranscript,
) -> Result<Value, AcceleratorContractError> {
    object(&transcript.frames[index].response, "protocol response")?
        .get("result")
        .cloned()
        .ok_or_else(|| invalid("successful protocol response omitted result"))
}

fn object<'a>(
    value: &'a Value,
    label: &str,
) -> Result<&'a Map<String, Value>, AcceleratorContractError> {
    value
        .as_object()
        .ok_or_else(|| invalid(format!("{label} is not an object")))
}

fn exact_keys(
    object: &Map<String, Value>,
    expected: &[&str],
    label: &str,
) -> Result<(), AcceleratorContractError> {
    let actual: BTreeSet<_> = object.keys().map(String::as_str).collect();
    let expected: BTreeSet<_> = expected.iter().copied().collect();
    require(
        actual == expected,
        format!("{label} fields do not match schema"),
    )
}

fn string<'a>(
    object: &'a Map<String, Value>,
    field: &str,
) -> Result<&'a str, AcceleratorContractError> {
    object
        .get(field)
        .and_then(Value::as_str)
        .ok_or_else(|| invalid(format!("{field} is not a string")))
}

fn unsigned(object: &Map<String, Value>, field: &str) -> Result<u64, AcceleratorContractError> {
    object
        .get(field)
        .and_then(Value::as_u64)
        .ok_or_else(|| invalid(format!("{field} is not an unsigned integer")))
}

fn array<'a>(
    object: &'a Map<String, Value>,
    field: &str,
) -> Result<&'a [Value], AcceleratorContractError> {
    object
        .get(field)
        .and_then(Value::as_array)
        .map(Vec::as_slice)
        .ok_or_else(|| invalid(format!("{field} is not an array")))
}

fn first_u64(object: &Map<String, Value>, field: &str) -> Result<u64, AcceleratorContractError> {
    array(object, field)?
        .first()
        .and_then(Value::as_u64)
        .ok_or_else(|| invalid(format!("{field}[0] is not an unsigned integer")))
}

fn first_f64(object: &Map<String, Value>, field: &str) -> Result<f64, AcceleratorContractError> {
    array(object, field)?
        .first()
        .and_then(Value::as_f64)
        .ok_or_else(|| invalid(format!("{field}[0] is not a number")))
}

#[cfg(test)]
mod tests {
    use super::*;

    const MANIFEST: &str = include_str!("../../../adapters/mjx/accelerator.toml");
    const RUNTIME: &str = include_str!("../../../adapters/mjx/runtime.toml");
    const TASK: &str = include_str!("../../../adapters/mjx/fixtures/free-fall-task-spec-v1.json");
    const TRANSCRIPT: &str =
        include_str!("../../../tests/golden/accelerators/protocol-transcript-v1.json");

    fn contracts() -> (
        AcceleratorManifest,
        AcceleratorRuntimeContract,
        TaskSpec,
        AcceleratorProtocolTranscript,
    ) {
        (
            toml::from_str(MANIFEST).unwrap(),
            toml::from_str(RUNTIME).unwrap(),
            serde_json::from_str(TASK).unwrap(),
            AcceleratorProtocolTranscript::from_json_slice(TRANSCRIPT.as_bytes()).unwrap(),
        )
    }

    #[test]
    fn committed_transcript_binds_every_operation() {
        let (manifest, runtime, task, transcript) = contracts();
        transcript
            .validate_against(&manifest, &runtime, &task)
            .unwrap();
    }

    #[test]
    fn correlation_operation_and_checkpoint_tampering_fail_closed() {
        let (_, _, _, mut transcript) = contracts();
        transcript.frames[3].response["request_id"] = Value::from(99);
        assert!(transcript.validate().is_err());
        let (_, _, _, mut transcript) = contracts();
        transcript.frames.swap(2, 3);
        assert!(transcript.validate().is_err());
        let (manifest, runtime, task, mut transcript) = contracts();
        transcript.frames[5].request["checkpoint"]["replay_digest"] = Value::from(0);
        assert!(transcript
            .validate_against(&manifest, &runtime, &task)
            .is_err());
    }

    #[test]
    fn task_lane_seed_and_unknown_fields_fail_closed() {
        let (manifest, runtime, task, mut transcript) = contracts();
        transcript.frames[2].response["result"]["episode_seeds"][0] = Value::from(0);
        assert!(transcript
            .validate_against(&manifest, &runtime, &task)
            .is_err());
        let (manifest, runtime, mut task, transcript) = contracts();
        task.task_id = "rne.physics.other.v1".into();
        assert!(transcript
            .validate_against(&manifest, &runtime, &task)
            .is_err());
        let mut value: Value = serde_json::from_str(TRANSCRIPT).unwrap();
        value
            .as_object_mut()
            .unwrap()
            .insert("unknown".into(), Value::Bool(true));
        assert!(serde_json::from_value::<AcceleratorProtocolTranscript>(value).is_err());
    }
}
