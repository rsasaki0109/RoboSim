//! Dependency-free accelerator adapter authoring scaffold.

use super::conformance::task_spec_sha256;
use super::{
    AcceleratorManifest, AcceleratorProtocolFrame, AcceleratorProtocolTranscript,
    AcceleratorRuntimeContract, ACCELERATOR_PROTOCOL_SCHEMA_VERSION,
    ACCELERATOR_PROTOCOL_TRANSCRIPT_KIND, ACCELERATOR_PROTOCOL_TRANSCRIPT_SCHEMA_VERSION,
};
use rne_ai::TaskSpec;
use serde_json::{json, Value};
use std::fs;
use std::path::{Path, PathBuf};

const TASK_JSON: &str = include_str!("scaffold/task.json");
const ADAPTER_PY: &str = include_str!("scaffold/adapter.py");
const ROOT_SEED: u64 = 42;
const EPISODE_ZERO_SEED: u64 = 1_298_720_818_104_676_741;
const EPISODE_ONE_SEED: u64 = 6_147_948_423_359_611_076;
const CREATE_LANE_DIGEST: u64 = 18_016_906_945_709_849_408;
const CREATE_REPLAY_DIGEST: u64 = 10_687_404_251_403_166_205;
const RESET_LANE_DIGEST: u64 = 7_933_392_803_188_615_342;
const RESET_REPLAY_DIGEST: u64 = 9_676_316_741_091_682_490;
const STEP_LANE_DIGEST: u64 = 1_025_258_777_869_343_529;
const STEP_REPLAY_DIGEST: u64 = 3_700_532_629_046_168_812;

/// Accelerator scaffold validation, contract, or I/O failure.
#[derive(Debug, thiserror::Error)]
pub enum AcceleratorScaffoldError {
    /// The requested adapter name is not a lowercase portable identifier.
    #[error("invalid accelerator adapter name {name:?}: use 1..=64 lowercase ASCII letters, digits, and underscores, starting with a letter")]
    InvalidName {
        /// Rejected adapter name.
        name: String,
    },
    /// The requested scaffold directory already exists.
    #[error("accelerator scaffold directory {path} already exists")]
    Exists {
        /// Existing directory path.
        path: String,
    },
    /// A scaffold directory or file could not be written.
    #[error("write accelerator scaffold {path}: {message}")]
    Write {
        /// Failed path.
        path: String,
        /// Underlying I/O diagnostic.
        message: String,
    },
    /// A generated typed contract failed its own validator.
    #[error("generated accelerator scaffold contract is invalid: {0}")]
    Contract(String),
    /// A generated JSON artifact could not be encoded.
    #[error("serialize accelerator scaffold JSON: {0}")]
    Json(#[from] serde_json::Error),
}

/// Validates a portable accelerator scaffold and adapter identifier.
pub fn validate_accelerator_scaffold_name(name: &str) -> Result<(), AcceleratorScaffoldError> {
    let valid = (1..=64).contains(&name.len())
        && name
            .bytes()
            .next()
            .is_some_and(|byte| byte.is_ascii_lowercase())
        && name
            .bytes()
            .all(|byte| byte.is_ascii_lowercase() || byte.is_ascii_digit() || byte == b'_');
    if valid {
        Ok(())
    } else {
        Err(AcceleratorScaffoldError::InvalidName {
            name: name.to_string(),
        })
    }
}

/// Creates an offline accelerator protocol-v1 authoring scaffold.
///
/// The new directory is `parent_dir/name` and contains a dependency-free
/// Python JSONL adapter, a passing transport fixture, TaskSpec, manifest,
/// runtime pins, model placeholder, requirements, selection record, and
/// authoring guide. The fixture is a test double, not independent evidence;
/// authors must replace its dispatch function with their own runtime backend.
pub fn scaffold_accelerator_adapter(
    name: &str,
    parent_dir: &Path,
) -> Result<PathBuf, AcceleratorScaffoldError> {
    validate_accelerator_scaffold_name(name)?;
    let directory = parent_dir.join(name);
    if directory.exists() {
        return Err(AcceleratorScaffoldError::Exists {
            path: directory.display().to_string(),
        });
    }

    let task: TaskSpec = serde_json::from_str(TASK_JSON)
        .map_err(|error| AcceleratorScaffoldError::Contract(error.to_string()))?;
    task.validate()
        .map_err(|error| AcceleratorScaffoldError::Contract(error.to_string()))?;
    let manifest_text = manifest_toml(name);
    let runtime_text = runtime_toml();
    let manifest: AcceleratorManifest = toml::from_str(&manifest_text)
        .map_err(|error| AcceleratorScaffoldError::Contract(error.to_string()))?;
    let runtime: AcceleratorRuntimeContract = toml::from_str(runtime_text)
        .map_err(|error| AcceleratorScaffoldError::Contract(error.to_string()))?;
    manifest
        .validate()
        .map_err(|error| AcceleratorScaffoldError::Contract(error.to_string()))?;
    runtime
        .validate()
        .map_err(|error| AcceleratorScaffoldError::Contract(error.to_string()))?;
    let transcript = scaffold_transcript(name, &runtime, &task)?;
    transcript
        .validate_against(&manifest, &runtime, &task)
        .map_err(|error| AcceleratorScaffoldError::Contract(error.to_string()))?;
    let mut transcript_json = serde_json::to_string_pretty(&transcript)?;
    transcript_json.push('\n');

    fs::create_dir_all(&directory).map_err(|error| write_error(&directory, error))?;
    for (file, contents) in [
        ("adapter.py", ADAPTER_PY.to_string()),
        ("accelerator.toml", manifest_text),
        ("runtime.toml", runtime_text.to_string()),
        ("task.json", TASK_JSON.to_string()),
        ("protocol-fixture.json", transcript_json),
        ("model.xml", "<mujoco model=\"replace_me\"/>\n".to_string()),
        ("requirements.txt", requirements_txt().to_string()),
        ("SELECTION.md", selection_md(name)),
        ("README.md", readme(name)),
    ] {
        let path = directory.join(file);
        fs::write(&path, contents).map_err(|error| write_error(&path, error))?;
    }
    Ok(directory)
}

fn write_error(path: &Path, error: std::io::Error) -> AcceleratorScaffoldError {
    AcceleratorScaffoldError::Write {
        path: path.display().to_string(),
        message: error.to_string(),
    }
}

fn manifest_toml(name: &str) -> String {
    format!(
        r#"schema_version = 1
id = "{name}"
selected = false
status = "experimental"
runtime = "mujoco_mjx_warp"
precision = "f64"
execution_boundary = "out_of_process_python"
core_dependency = false
requires_nvidia_gpu = true
task_spec_schema = 1
batch_checkpoint_schema = 2
protocol_schema = 1
capability_report_schema = 1
conformance_report_schema = 1
scale_report_schema = 1
supported_batch_widths = [1, 16, 256, 4096]
binding_task_spec = "task.json"
binding_model = "model.xml"
runtime_contract = "runtime.toml"
requirements = "requirements.txt"
selection_adr = "SELECTION.md"
official_sources = [
  "https://mujoco.readthedocs.io/en/latest/mjx.html",
  "https://genesis-world.readthedocs.io/en/latest/user_guide/overview/installation.html",
  "https://isaac-sim.github.io/IsaacLab/v2.3.2/source/setup/installation/index.html",
]
"#
    )
}

fn runtime_toml() -> &'static str {
    r#"schema_version = 1
operating_system = "linux"
architecture = "x86_64"
python = "3.12"
cuda_major = 13
nvidia_driver_minimum = 580
official_sources = [
  "https://docs.jax.dev/en/latest/installation.html",
  "https://pypi.org/pypi/jax/0.10.2/json",
  "https://pypi.org/pypi/mujoco-mjx/3.9.0/json",
]

[packages]
jax = "0.10.2"
jaxlib = "0.10.2"
jax_cuda_plugin = "0.10.2"
mujoco = "3.9.0"
mujoco_mjx = "3.9.0"
warp_lang = "1.12.1"
"#
}

fn requirements_txt() -> &'static str {
    "# Replace these pins only with a reviewed runtime contract transition.\n\
jax==0.10.2\n\
jaxlib==0.10.2\n\
jax-cuda13-plugin==0.10.2\n\
mujoco==3.9.0\n\
mujoco-mjx==3.9.0\n\
warp-lang==1.12.1\n"
}

fn selection_md(name: &str) -> String {
    format!(
        "# {name} runtime selection\n\nStatus: scaffold placeholder.\n\nRecord measured runtime alternatives, hardware, driver, precision, task parity, and the immutable revision selected for this independently maintained adapter.\n"
    )
}

fn readme(name: &str) -> String {
    format!(
        r#"# {name}

This offline scaffold proves the RNE accelerator protocol-v1 transport before
runtime work begins. `adapter.py` initially replays a typed fixture. It is not
an accelerator implementation and cannot qualify as independent evidence.

Run from this directory:

```bash
rne-accelerator-conformance \
  --adapter python3 \
  --adapter-arg adapter.py \
  --subject adapter.py \
  --manifest accelerator.toml \
  --runtime runtime.toml \
  --task task.json \
  --output conformance.json
```

Replace `dispatch` with your own bounded session/backend implementation, update
the model, runtime pins, supported widths, and selection record, then rerun the
standalone kit. Preserve JSONL correlation, deterministic lane seeds, partial
reset, portable checkpoint/restore, and fail-closed unsupported operations.
Do not add accelerator dependencies to RNE core crates.
"#
    )
}

fn scaffold_transcript(
    name: &str,
    runtime: &AcceleratorRuntimeContract,
    task: &TaskSpec,
) -> Result<AcceleratorProtocolTranscript, AcceleratorScaffoldError> {
    let task_value = serde_json::to_value(task)?;
    let task_digest = task_spec_sha256(task)
        .map_err(|error| AcceleratorScaffoldError::Contract(error.to_string()))?;
    let create = state_result(0, true, true, false);
    let reset = state_result(1, true, false, false);
    let step = state_result(1, false, false, true);
    let mut restored = step.clone();
    let restored_object = restored
        .as_object_mut()
        .expect("restored state is an object");
    restored_object.remove("rewards");
    restored_object.remove("terminated");
    restored_object.remove("truncated");
    let checkpoint = json!({
        "schema_version": 2,
        "seed": ROOT_SEED,
        "num_envs": 1,
        "auto_reset": false,
        "seed_strategy": "split_mix64_lane_episode_v1",
        "task_spec": task_value.clone(),
        "lanes": [{
            "lane_id": 0,
            "episode_index": 1,
            "episode_seed": EPISODE_ONE_SEED,
            "pending_auto_reset": false,
            "replay_digest": STEP_LANE_DIGEST
        }],
        "operations": [
            {"type": "reset_lanes", "lane_ids": [0]},
            {"type": "step", "actions": [[0.0]]}
        ],
        "replay_digest": STEP_REPLAY_DIGEST
    });
    let requests = vec![
        request(0, "probe", json!({})),
        request(
            1,
            "create",
            json!({
                "session_id": "contract", "task_spec": task_value, "root_seed": ROOT_SEED,
                "batch_width": 1, "auto_reset": false
            }),
        ),
        request(
            2,
            "reset_lanes",
            json!({"session_id": "contract", "lane_ids": [0]}),
        ),
        request(
            3,
            "step",
            json!({"session_id": "contract", "actions": [[0.0]]}),
        ),
        request(4, "checkpoint", json!({"session_id": "contract"})),
        request(
            5,
            "restore",
            json!({"session_id": "contract", "checkpoint": checkpoint.clone()}),
        ),
        request(6, "close", json!({"session_id": "contract"})),
        request(7, "unsupported_v1_fixture", json!({})),
        request(8, "shutdown", json!({})),
    ];
    let responses = vec![
        response(0, capability(name, runtime)),
        response(1, create),
        response(2, reset),
        response(3, step.clone()),
        response(4, checkpoint),
        response(5, restored),
        response(6, json!({"closed": true, "session_id": "contract"})),
        json!({
            "kind": "rne_accelerator_response", "schema_version": 1,
            "request_id": 7, "ok": false,
            "error": {"code": "unsupported_operation", "message": "unsupported operation 'unsupported_v1_fixture'", "details": {}}
        }),
        response(8, json!({"shutdown": true})),
    ];
    Ok(AcceleratorProtocolTranscript {
        kind: ACCELERATOR_PROTOCOL_TRANSCRIPT_KIND.to_string(),
        schema_version: ACCELERATOR_PROTOCOL_TRANSCRIPT_SCHEMA_VERSION,
        protocol_schema: ACCELERATOR_PROTOCOL_SCHEMA_VERSION,
        adapter_id: name.to_string(),
        task_id: task.task_id.clone(),
        task_spec_schema: task.schema_version,
        task_spec_sha256: task_digest,
        root_seed: ROOT_SEED,
        batch_width: 1,
        frames: requests
            .into_iter()
            .zip(responses)
            .map(|(request, response)| AcceleratorProtocolFrame { request, response })
            .collect(),
    })
}

fn request(request_id: u64, operation: &str, fields: Value) -> Value {
    let mut value = json!({
        "kind": "rne_accelerator_request", "schema_version": 1,
        "request_id": request_id, "operation": operation
    });
    value.as_object_mut().expect("request is an object").extend(
        fields
            .as_object()
            .expect("request fields are an object")
            .clone(),
    );
    value
}

fn response(request_id: u64, result: Value) -> Value {
    json!({
        "kind": "rne_accelerator_response", "schema_version": 1,
        "request_id": request_id, "ok": true, "result": result
    })
}

fn state_result(episode_index: u64, reset: bool, session: bool, stepped: bool) -> Value {
    let (seed, lane_digest, replay_digest, observation) = if episode_index == 0 {
        (
            EPISODE_ZERO_SEED,
            CREATE_LANE_DIGEST,
            CREATE_REPLAY_DIGEST,
            json!([5.0, 0.0]),
        )
    } else if stepped {
        (
            EPISODE_ONE_SEED,
            STEP_LANE_DIGEST,
            STEP_REPLAY_DIGEST,
            json!([4.997275000218, -0.16349999346000002]),
        )
    } else {
        (
            EPISODE_ONE_SEED,
            RESET_LANE_DIGEST,
            RESET_REPLAY_DIGEST,
            json!([5.0, 0.0]),
        )
    };
    let mut value = json!({
        "lane_ids": [0], "episode_indices": [episode_index], "episode_seeds": [seed],
        "reset": [reset], "observations": [observation],
        "lane_replay_digests": [lane_digest], "replay_digest": replay_digest
    });
    let object = value.as_object_mut().expect("state result is an object");
    if session {
        object.insert("session_id".to_string(), json!("contract"));
    }
    if stepped {
        object.extend([
            ("rewards".to_string(), json!([4.997275000218])),
            ("terminated".to_string(), json!([false])),
            ("truncated".to_string(), json!([false])),
        ]);
    }
    value
}

fn capability(name: &str, runtime: &AcceleratorRuntimeContract) -> Value {
    json!({
        "kind": "rne_accelerator_capability_report", "schema_version": 1,
        "adapter_id": name, "status": "test_only", "unavailable_reason_code": null,
        "runtime_id": "mujoco_mjx_warp", "precision": "f64",
        "execution_boundary": "out_of_process_python", "requires_nvidia_gpu": true,
        "task_spec_schema": 1, "batch_checkpoint_schema": 2, "protocol_schema": 1,
        "runtime_contract_schema": 1, "conformance_report_schema": 1, "scale_report_schema": 1,
        "supported_batch_widths": [1, 16, 256, 4096],
        "supported_task_ids": ["rne.physics.free_fall.mjx.v1"],
        "unsupported_features": [
            "automatic_differentiation", "midpoint_implicitfast_integrator", "noslip_solver",
            "pgs_solver", "plugin_sensors"
        ],
        "runtime_contract": runtime,
        "runtime": {
            "python_version": "<runtime>", "platform": "<runtime>", "machine": "<runtime>",
            "jax_version": null, "jaxlib_version": null, "jax_cuda_plugin_version": null,
            "mujoco_version": null, "mujoco_mjx_version": null, "warp_version": null,
            "jax_backend": null, "jax_devices": [], "nvidia_driver_version": null
        }
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn scaffold_is_typed_non_overwriting_and_explicitly_nonqualifying() {
        let parent =
            std::env::temp_dir().join(format!("rne-accelerator-scaffold-{}", std::process::id()));
        let _ = fs::remove_dir_all(&parent);
        let directory = scaffold_accelerator_adapter("external_accelerator", &parent).unwrap();
        for file in [
            "adapter.py",
            "accelerator.toml",
            "runtime.toml",
            "task.json",
            "protocol-fixture.json",
            "model.xml",
            "requirements.txt",
            "SELECTION.md",
            "README.md",
        ] {
            assert!(directory.join(file).is_file(), "missing scaffold {file}");
        }
        let readme = fs::read_to_string(directory.join("README.md")).unwrap();
        assert!(readme.contains("cannot qualify as independent evidence"));
        assert!(matches!(
            scaffold_accelerator_adapter("external_accelerator", &parent),
            Err(AcceleratorScaffoldError::Exists { .. })
        ));
        let _ = fs::remove_dir_all(&parent);
    }

    #[test]
    fn names_are_portable_and_path_inert() {
        for invalid in ["", "Upper", "with-dash", "../escape", "a b", "_prefix"] {
            assert!(validate_accelerator_scaffold_name(invalid).is_err());
        }
        assert!(validate_accelerator_scaffold_name("adapter_2").is_ok());
    }
}
