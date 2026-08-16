use anyhow::{ensure, Context};
use rne_ai::{
    mm_minimal_scene_path, MobileManipulatorSim, MobileManipulatorSimSnapshot, TaskSpec,
    MOBILE_MANIPULATOR_SIM_SNAPSHOT_MIN_VERSION, MOBILE_MANIPULATOR_SIM_SNAPSHOT_VERSION,
    TASK_SPEC_SCHEMA_VERSION, VECTORIZED_EPISODE_CHECKPOINT_VERSION,
};
use rne_compatibility_suite::{
    HISTORICAL_COMPATIBILITY_DECISION_SCHEMA_VERSION,
    HISTORICAL_MIGRATION_PROVENANCE_SCHEMA_VERSION,
};
use rne_data::transport::{
    negotiate_transport, ClientHello, NegotiationPolicy, SensorFrameMetadata,
    TransportCapabilities, TransportFrame, TransportMessageKind, TRANSPORT_PROTOCOL_MAJOR,
};
use rne_data::{
    encode_dataset_action, encode_dataset_annotation, encode_dataset_imu,
    encode_dataset_task_outcome, encode_dataset_transform, DatasetActionSample,
    DatasetGroundTruthAnnotation, DatasetManifest, DatasetTaskOutcomeSample, ImuSample, PoseSample,
    DATASET_BUNDLE_SCHEMA_VERSION, DATASET_PAYLOAD_SCHEMA_VERSION,
};
use rne_log::{FailureCapsule, FAILURE_CAPSULE_SCHEMA_VERSION};
use rne_math::Vec3;
use serde_json::{json, Value};
use sha2::{Digest, Sha256};
use std::env;
use std::fs;
use std::path::Path;

const HISTORICAL_MIGRATION_FLOAT_TOLERANCE: f64 = 1.0e-9;
const HISTORICAL_SOURCE_SCENE: &str = "assets/scenes/mm_minimal.rne.scene.toml";
const HISTORICAL_SOURCE_WORKSPACE_VERSION: &str = "0.8.0";
const HISTORICAL_SOURCE_GENERATION_STEPS: u64 = 7;
const HISTORICAL_V1_REVISION: &str = "47525b127a77cbffa9da27b1e0c127ee673aa641";
const HISTORICAL_V1_TREE: &str = "bb408cec26d34bd2a9b423dbf8b2a4d44cdf7013";
const HISTORICAL_V2_REVISION: &str = "2255cbefec9d1eb5040603fbb119a290ad855191";
const HISTORICAL_V2_TREE: &str = "373e5453c7ba94ee4efbeceb9985db4c97f5feff";
const HISTORICAL_VECTORIZED_V1_REVISION: &str = "bd4d44f5bd781fc41fd8305938001f0a858993a5";
const HISTORICAL_VECTORIZED_V1_TREE: &str = "23482add2c5d1de2978897d894d1ba745787bd06";
const HISTORICAL_SCENARIO_V2_REVISION: &str = "533729ddc78e53284eaa11d823afae18dcd110ab";
const HISTORICAL_SCENARIO_V2_TREE: &str = "b016841b2aed16bafc131f6a4698ee3b30cec34d";
const HISTORICAL_SCENARIO_V3_REVISION: &str = "e959e3ffe8426de3a8320d2d4c95e4e1438a50ad";
const HISTORICAL_SCENARIO_V3_TREE: &str = "17c6045624ccf2ed1271d19ea50926cb568ab337";
const HISTORICAL_TASK_SPEC_V1_REVISION: &str = "70a9ff35afbf0215803dd288103bdda79fa46891";
const HISTORICAL_TASK_SPEC_V1_TREE: &str = "94459bcb0c5090921bf6edbcf6f63246ebdd6a40";
const HISTORICAL_DATASET_V1_REVISION: &str = "aecafb62c99f432b2a76956575f4562c6047a6bc";
const HISTORICAL_DATASET_V1_TREE: &str = "0bc9d2d48185282da31dc80eb8857d84012a5928";
const HISTORICAL_FAILURE_CAPSULE_V1_REVISION: &str = "61d6c813e79d7eac6a8ab212776d620069f98905";
const HISTORICAL_FAILURE_CAPSULE_V1_TREE: &str = "5dac12166fe39da5a1207426f3e7520851e415d2";
const HISTORICAL_VECTORIZED_V1_REPLAY_DIGEST: u64 = 17_972_057_113_911_492_359;
const HISTORICAL_SCENARIO_STABLE_HASH: u64 = 8_877_782_128_690_619_681;
const HISTORICAL_DATASET_EVALUATION_SHA256: &str =
    "sha256:d09bf2d9079fe607373b6376c7fa4d0cbde3be4ce1787914702ccf4caa06aa16";
const SCENARIO_V2_MISSING_REQUIRED_FIELDS: &[&str] = &[
    "scenario_digest",
    "network_digest",
    "engine_version",
    "result.result_digest",
    "result.final_actors",
    "result.action_evidence",
    "result.unapplied_action_count",
    "result.minimum_observed_gap_m",
    "result.ownership",
];
const SCENARIO_V3_MISSING_REQUIRED_FIELDS: &[&str] = &[
    "result.result_digest",
    "result.final_actors",
    "result.action_evidence",
    "result.unapplied_action_count",
    "result.minimum_observed_gap_m",
    "result.ownership",
];

fn main() -> anyhow::Result<()> {
    let mut args = env::args_os().skip(1);
    let output = args.next().context(
        "usage: generate_binary_fixtures <output-directory> [<snapshot-v1.json> <snapshot-v2.json> [<vectorized-v1.json> <scenario-v2.json> <scenario-v3.json> [<task-spec-v1.json> <dataset-manifest-v1.json> <dataset-shard-v1.rnedata> <failure-capsule-v1.json>]]]",
    )?;
    let source_args = args.collect::<Vec<_>>();
    let (
        v1_source,
        v2_source,
        vectorized_v1,
        scenario_v2,
        scenario_v3,
        task_spec_v1,
        dataset_manifest_v1,
        dataset_shard_v1,
        failure_capsule_v1,
    ) = match source_args.as_slice() {
        [] => (
            committed_source_snapshot("mobile-manipulator-snapshot-v1-47525b1-to-v3.json")?,
            committed_source_snapshot("mobile-manipulator-snapshot-v2-2255cbe-to-v3.json")?,
            committed_decision_source("vectorized-episode-checkpoint-v1-bd4d44f.json")?,
            committed_decision_source("scenario-replay-v2-533729d-requires-rerun.json")?,
            committed_decision_source("scenario-replay-v3-e959e3f-requires-rerun.json")?,
            committed_decision_source("task-spec-v1-70a9ff3.json")?,
            committed_decision_source("dataset-bundle-v1-aecafb6.json")?,
            committed_decision_file("dataset-bundle-v1-aecafb6.json", "records.rnedata")?,
            committed_decision_source("failure-capsule-v1-61d6c81.json")?,
        ),
        [v1, v2] => (
            read_json(Path::new(v1))?,
            read_json(Path::new(v2))?,
            committed_decision_source("vectorized-episode-checkpoint-v1-bd4d44f.json")?,
            committed_decision_source("scenario-replay-v2-533729d-requires-rerun.json")?,
            committed_decision_source("scenario-replay-v3-e959e3f-requires-rerun.json")?,
            committed_decision_source("task-spec-v1-70a9ff3.json")?,
            committed_decision_source("dataset-bundle-v1-aecafb6.json")?,
            committed_decision_file("dataset-bundle-v1-aecafb6.json", "records.rnedata")?,
            committed_decision_source("failure-capsule-v1-61d6c81.json")?,
        ),
        [v1, v2, vectorized, scenario2, scenario3] => (
            read_json(Path::new(v1))?,
            read_json(Path::new(v2))?,
            read_json(Path::new(vectorized))?,
            read_json(Path::new(scenario2))?,
            read_json(Path::new(scenario3))?,
            committed_decision_source("task-spec-v1-70a9ff3.json")?,
            committed_decision_source("dataset-bundle-v1-aecafb6.json")?,
            committed_decision_file("dataset-bundle-v1-aecafb6.json", "records.rnedata")?,
            committed_decision_source("failure-capsule-v1-61d6c81.json")?,
        ),
        [v1, v2, vectorized, scenario2, scenario3, task, dataset, shard, capsule] => (
            read_json(Path::new(v1))?,
            read_json(Path::new(v2))?,
            read_json(Path::new(vectorized))?,
            read_json(Path::new(scenario2))?,
            read_json(Path::new(scenario3))?,
            read_json(Path::new(task))?,
            read_json(Path::new(dataset))?,
            fs::read(shard).with_context(|| {
                format!(
                    "read historical dataset shard {}",
                    Path::new(shard).display()
                )
            })?,
            read_json(Path::new(capsule))?,
        ),
        _ => anyhow::bail!("expected zero, two, five, or nine historical source paths"),
    };
    let output = Path::new(&output);
    fs::create_dir_all(output)
        .with_context(|| format!("create fixture output {}", output.display()))?;
    let dataset_source_files = [HistoricalDecisionFileSource {
        path: "records.rnedata",
        contents: &dataset_shard_v1,
    }];
    write_json(
        &output.join("frontend-transport-v1.json"),
        &frontend_fixture()?,
    )?;
    write_json(&output.join("dataset-payload-v1.json"), &dataset_fixture()?)?;
    write_json(
        &output.join("mobile-manipulator-snapshot-v1-to-v3.json"),
        &mobile_manipulator_snapshot_fixture()?,
    )?;
    write_json(
        &output.join("mobile-manipulator-snapshot-v1-47525b1-to-v3.json"),
        &historical_snapshot_provenance_fixture(
            v1_source,
            1,
            HISTORICAL_V1_REVISION,
            HISTORICAL_V1_TREE,
        )?,
    )?;
    write_json(
        &output.join("mobile-manipulator-snapshot-v2-2255cbe-to-v3.json"),
        &historical_snapshot_provenance_fixture(
            v2_source,
            2,
            HISTORICAL_V2_REVISION,
            HISTORICAL_V2_TREE,
        )?,
    )?;
    write_json(
        &output.join("vectorized-episode-checkpoint-v1-bd4d44f.json"),
        &historical_compatibility_decision_fixture(
            vectorized_v1,
            HistoricalDecisionSpec {
                artifact_contract: "vectorized_episode_checkpoint",
                source_schema_version: 1,
                current_schema_version: VECTORIZED_EPISODE_CHECKPOINT_VERSION,
                source_revision: HISTORICAL_VECTORIZED_V1_REVISION,
                source_tree: HISTORICAL_VECTORIZED_V1_TREE,
                source_workspace_version: "0.1.0",
                source_files: &[],
                expected_outcome: "accepted_and_restored",
                reason_code: "same_schema_replay_checkpoint",
                missing_required_fields: &[],
                expected_replay_digest: Some(HISTORICAL_VECTORIZED_V1_REPLAY_DIGEST),
                expected_error: None,
                expected_result_sha256: None,
            },
        )?,
    )?;
    write_json(
        &output.join("scenario-replay-v2-533729d-requires-rerun.json"),
        &historical_compatibility_decision_fixture(
            scenario_v2,
            HistoricalDecisionSpec {
                artifact_contract: "scenario_replay",
                source_schema_version: 2,
                current_schema_version: rne_openscenario::SCENARIO_REPLAY_SCHEMA_VERSION,
                source_revision: HISTORICAL_SCENARIO_V2_REVISION,
                source_tree: HISTORICAL_SCENARIO_V2_TREE,
                source_workspace_version: "0.13.0",
                source_files: &[],
                expected_outcome: "rejected_requires_rerun",
                reason_code: "missing_required_replay_evidence",
                missing_required_fields: SCENARIO_V2_MISSING_REQUIRED_FIELDS,
                expected_replay_digest: None,
                expected_error: Some(
                    "unsupported scenario replay schema version: expected 4, got 2",
                ),
                expected_result_sha256: None,
            },
        )?,
    )?;
    write_json(
        &output.join("scenario-replay-v3-e959e3f-requires-rerun.json"),
        &historical_compatibility_decision_fixture(
            scenario_v3,
            HistoricalDecisionSpec {
                artifact_contract: "scenario_replay",
                source_schema_version: 3,
                current_schema_version: rne_openscenario::SCENARIO_REPLAY_SCHEMA_VERSION,
                source_revision: HISTORICAL_SCENARIO_V3_REVISION,
                source_tree: HISTORICAL_SCENARIO_V3_TREE,
                source_workspace_version: "0.13.0",
                source_files: &[],
                expected_outcome: "rejected_requires_rerun",
                reason_code: "missing_required_replay_evidence",
                missing_required_fields: SCENARIO_V3_MISSING_REQUIRED_FIELDS,
                expected_replay_digest: None,
                expected_error: Some(
                    "unsupported scenario replay schema version: expected 4, got 3",
                ),
                expected_result_sha256: None,
            },
        )?,
    )?;
    write_json(
        &output.join("task-spec-v1-70a9ff3.json"),
        &historical_compatibility_decision_fixture(
            task_spec_v1,
            HistoricalDecisionSpec {
                artifact_contract: "task_spec",
                source_schema_version: 1,
                current_schema_version: TASK_SPEC_SCHEMA_VERSION,
                source_revision: HISTORICAL_TASK_SPEC_V1_REVISION,
                source_tree: HISTORICAL_TASK_SPEC_V1_TREE,
                source_workspace_version: "0.1.0",
                source_files: &[],
                expected_outcome: "accepted_and_restored",
                reason_code: "same_schema_validated_artifact",
                missing_required_fields: &[],
                expected_replay_digest: None,
                expected_error: None,
                expected_result_sha256: None,
            },
        )?,
    )?;
    write_json(
        &output.join("dataset-bundle-v1-aecafb6.json"),
        &historical_compatibility_decision_fixture(
            dataset_manifest_v1,
            HistoricalDecisionSpec {
                artifact_contract: "dataset_bundle",
                source_schema_version: 1,
                current_schema_version: DATASET_BUNDLE_SCHEMA_VERSION,
                source_revision: HISTORICAL_DATASET_V1_REVISION,
                source_tree: HISTORICAL_DATASET_V1_TREE,
                source_workspace_version: "0.1.0",
                source_files: &dataset_source_files,
                expected_outcome: "accepted_and_restored",
                reason_code: "same_schema_streaming_dataset_bundle",
                missing_required_fields: &[],
                expected_replay_digest: None,
                expected_error: None,
                expected_result_sha256: Some(HISTORICAL_DATASET_EVALUATION_SHA256),
            },
        )?,
    )?;
    write_json(
        &output.join("failure-capsule-v1-61d6c81.json"),
        &historical_compatibility_decision_fixture(
            failure_capsule_v1,
            HistoricalDecisionSpec {
                artifact_contract: "failure_capsule",
                source_schema_version: 1,
                current_schema_version: FAILURE_CAPSULE_SCHEMA_VERSION,
                source_revision: HISTORICAL_FAILURE_CAPSULE_V1_REVISION,
                source_tree: HISTORICAL_FAILURE_CAPSULE_V1_TREE,
                source_workspace_version: "0.1.0",
                source_files: &[],
                expected_outcome: "accepted_and_restored",
                reason_code: "same_schema_validated_artifact",
                missing_required_fields: &[],
                expected_replay_digest: None,
                expected_error: None,
                expected_result_sha256: None,
            },
        )?,
    )
}

#[derive(Clone, Copy, Debug)]
struct HistoricalDecisionSpec<'a> {
    artifact_contract: &'a str,
    source_schema_version: u32,
    current_schema_version: u32,
    source_revision: &'a str,
    source_tree: &'a str,
    source_workspace_version: &'a str,
    source_files: &'a [HistoricalDecisionFileSource<'a>],
    expected_outcome: &'a str,
    reason_code: &'a str,
    missing_required_fields: &'a [&'a str],
    expected_replay_digest: Option<u64>,
    expected_error: Option<&'a str>,
    expected_result_sha256: Option<&'a str>,
}

#[derive(Clone, Copy, Debug)]
struct HistoricalDecisionFileSource<'a> {
    path: &'a str,
    contents: &'a [u8],
}

fn historical_compatibility_decision_fixture(
    source: Value,
    spec: HistoricalDecisionSpec<'_>,
) -> anyhow::Result<Value> {
    ensure!(
        source.get("schema_version").and_then(Value::as_u64)
            == Some(u64::from(spec.source_schema_version)),
        "historical decision source schema mismatch"
    );
    match spec.artifact_contract {
        "vectorized_episode_checkpoint" => {
            ensure!(
                source.get("replay_digest").and_then(Value::as_u64) == spec.expected_replay_digest,
                "historical vectorized checkpoint digest mismatch"
            );
            ensure!(
                spec.source_schema_version == VECTORIZED_EPISODE_CHECKPOINT_VERSION
                    && spec.expected_error.is_none()
                    && spec.expected_result_sha256.is_none()
                    && spec.source_files.is_empty()
                    && spec.missing_required_fields.is_empty(),
                "historical vectorized checkpoint decision mismatch"
            );
        }
        "scenario_replay" => {
            ensure!(
                json_path(&source, "result.stable_hash").and_then(Value::as_u64)
                    == Some(HISTORICAL_SCENARIO_STABLE_HASH),
                "historical scenario replay stable hash mismatch"
            );
            for field in spec.missing_required_fields {
                ensure!(
                    json_path(&source, field).is_none(),
                    "historical scenario replay unexpectedly contains {field}"
                );
            }
            let error = rne_openscenario::ScenarioReplayArtifact::from_json(
                &serde_json::to_string(&source)?,
            )
            .expect_err("historical scenario replay must be rejected");
            let error_text = error.to_string();
            ensure!(
                spec.expected_error == Some(error_text.as_str()),
                "historical scenario replay rejection changed"
            );
            ensure!(
                spec.source_files.is_empty() && spec.expected_result_sha256.is_none(),
                "historical scenario replay decision unexpectedly embeds source files"
            );
        }
        "task_spec" => {
            let task: TaskSpec = serde_json::from_value(source.clone())?;
            task.validate()?;
            ensure!(
                task.schema_version == TASK_SPEC_SCHEMA_VERSION
                    && serde_json::to_value(&task)? == source
                    && spec.source_files.is_empty()
                    && spec.expected_result_sha256.is_none(),
                "historical TaskSpec decision mismatch"
            );
        }
        "failure_capsule" => {
            let capsule: FailureCapsule = serde_json::from_value(source.clone())?;
            capsule.validate()?;
            ensure!(
                capsule.schema_version == FAILURE_CAPSULE_SCHEMA_VERSION
                    && serde_json::to_value(&capsule)? == source
                    && spec.source_files.is_empty()
                    && spec.expected_result_sha256.is_none(),
                "historical Failure Capsule decision mismatch"
            );
        }
        "dataset_bundle" => {
            let manifest: DatasetManifest = serde_json::from_value(source.clone())?;
            manifest.validate()?;
            ensure!(
                manifest.schema_version == DATASET_BUNDLE_SCHEMA_VERSION
                    && serde_json::to_value(&manifest)? == source
                    && spec.source_files.len() == 1
                    && spec.source_files[0].path == "records.rnedata"
                    && manifest.shards.len() == 1
                    && manifest.shards[0].sha256 == sha256(spec.source_files[0].contents)
                    && manifest.shards[0].byte_len as usize == spec.source_files[0].contents.len()
                    && spec.expected_result_sha256 == Some(HISTORICAL_DATASET_EVALUATION_SHA256),
                "historical dataset bundle decision mismatch"
            );
        }
        other => anyhow::bail!("unsupported historical decision contract {other}"),
    }
    let source_artifact_sha256 = sha256(&serde_json::to_vec(&source)?);
    let mut fixture = json!({
        "kind": "rne_historical_compatibility_decision",
        "schema_version": HISTORICAL_COMPATIBILITY_DECISION_SCHEMA_VERSION,
        "artifact_contract": spec.artifact_contract,
        "source_schema_version": spec.source_schema_version,
        "current_schema_version": spec.current_schema_version,
        "source_revision": spec.source_revision,
        "source_tree": spec.source_tree,
        "source_workspace_version": spec.source_workspace_version,
        "expected_outcome": spec.expected_outcome,
        "reason_code": spec.reason_code,
        "missing_required_fields": spec.missing_required_fields,
        "source_artifact": source,
        "source_artifact_sha256": source_artifact_sha256,
        "expected_replay_digest": spec.expected_replay_digest,
        "expected_error": spec.expected_error,
    });
    let fixture_object = fixture
        .as_object_mut()
        .context("historical compatibility fixture must be an object")?;
    if !spec.source_files.is_empty() {
        fixture_object.insert(
            "source_files".to_string(),
            Value::Array(
                spec.source_files
                    .iter()
                    .map(|file| {
                        json!({
                            "path": file.path,
                            "size_bytes": file.contents.len(),
                            "sha256": sha256(file.contents),
                            "contents_hex": lower_hex(file.contents),
                        })
                    })
                    .collect(),
            ),
        );
    }
    if let Some(expected_result_sha256) = spec.expected_result_sha256 {
        fixture_object.insert(
            "expected_result_sha256".to_string(),
            Value::String(expected_result_sha256.to_string()),
        );
    }
    Ok(fixture)
}

fn json_path<'a>(value: &'a Value, path: &str) -> Option<&'a Value> {
    path.split('.')
        .try_fold(value, |current, key| current.get(key))
}

fn historical_snapshot_provenance_fixture(
    source: Value,
    source_schema_version: u32,
    source_revision: &str,
    source_tree: &str,
) -> anyhow::Result<Value> {
    ensure!(
        source.get("schema_version").and_then(Value::as_u64)
            == Some(u64::from(source_schema_version)),
        "historical source schema mismatch"
    );
    ensure!(
        source.get("step_count").and_then(Value::as_u64)
            == Some(HISTORICAL_SOURCE_GENERATION_STEPS),
        "historical source step count mismatch"
    );
    ensure!(
        source.get("sim_ticks").and_then(Value::as_u64).unwrap_or(0) > 0,
        "historical source must be captured after simulation advances"
    );
    ensure!(
        source
            .get("joint_state_frame")
            .is_some_and(|value| !value.is_null())
            && source
                .get("wrist_camera_frame")
                .is_some_and(|value| !value.is_null()),
        "historical source must retain joint-state and wrist-camera frames"
    );
    match source_schema_version {
        1 => ensure!(
            source.get("wrist_depth_frame").is_none(),
            "schema-v1 source unexpectedly contains wrist_depth_frame"
        ),
        2 => ensure!(
            source
                .get("wrist_depth_frame")
                .is_some_and(|value| !value.is_null()),
            "schema-v2 source must retain a populated wrist_depth_frame"
        ),
        other => anyhow::bail!("unsupported historical source schema {other}"),
    }
    ensure!(
        source.get("grasp_retarget").is_none(),
        "pre-v3 source unexpectedly contains grasp_retarget"
    );

    let snapshot: MobileManipulatorSimSnapshot = serde_json::from_value(source.clone())?;
    let scene = mm_minimal_scene_path()
        .canonicalize()
        .context("canonicalize historical snapshot restore scene")?;
    let mut restored = MobileManipulatorSim::from_scene_path(&scene)
        .context("load historical snapshot restore scene")?;
    restored
        .restore_snapshot(&snapshot)
        .map_err(|error| anyhow::anyhow!("restore historical snapshot: {error:?}"))?;
    let current = serde_json::to_value(restored.snapshot())?;
    ensure!(
        current.get("schema_version").and_then(Value::as_u64)
            == Some(u64::from(MOBILE_MANIPULATOR_SIM_SNAPSHOT_VERSION)),
        "restored snapshot did not normalize to the current schema"
    );
    let mut expected = snapshot;
    expected.schema_version = MOBILE_MANIPULATOR_SIM_SNAPSHOT_VERSION;
    ensure!(
        canonical_state_value(&serde_json::to_value(expected)?) == canonical_state_value(&current),
        "historical snapshot restore changed retained state"
    );
    let source_snapshot_sha256 = sha256(&serde_json::to_vec(&source)?);
    let current_snapshot_sha256 = sha256(&serde_json::to_vec(&canonical_state_value(&current))?);

    Ok(json!({
        "kind": "rne_historical_migration_case",
        "schema_version": HISTORICAL_MIGRATION_PROVENANCE_SCHEMA_VERSION,
        "artifact_contract": "mobile_manipulator_sim_snapshot",
        "source_schema_version": source_schema_version,
        "current_schema_version": MOBILE_MANIPULATOR_SIM_SNAPSHOT_VERSION,
        "source_revision": source_revision,
        "source_tree": source_tree,
        "source_workspace_version": HISTORICAL_SOURCE_WORKSPACE_VERSION,
        "source_scene": HISTORICAL_SOURCE_SCENE,
        "generation_steps": HISTORICAL_SOURCE_GENERATION_STEPS,
        "expected_outcome": "accepted_within_tolerance",
        "float_tolerance": HISTORICAL_MIGRATION_FLOAT_TOLERANCE,
        "source_snapshot": source,
        "source_snapshot_sha256": source_snapshot_sha256,
        "current_snapshot_sha256": current_snapshot_sha256,
    }))
}

fn read_json(path: &Path) -> anyhow::Result<Value> {
    let bytes =
        fs::read(path).with_context(|| format!("read historical snapshot {}", path.display()))?;
    serde_json::from_slice(&bytes)
        .with_context(|| format!("parse historical snapshot {}", path.display()))
}

fn committed_source_snapshot(file_name: &str) -> anyhow::Result<Value> {
    let root = Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(Path::parent)
        .context("resolve workspace root")?;
    let fixture_path = root.join("tests/golden/migrations").join(file_name);
    let fixture = read_json(&fixture_path)?;
    fixture.get("source_snapshot").cloned().with_context(|| {
        format!(
            "fixture omitted source_snapshot: {}",
            fixture_path.display()
        )
    })
}

fn committed_decision_source(file_name: &str) -> anyhow::Result<Value> {
    let root = Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(Path::parent)
        .context("resolve workspace root")?;
    let fixture_path = root.join("tests/golden/compatibility").join(file_name);
    let fixture = read_json(&fixture_path)?;
    fixture.get("source_artifact").cloned().with_context(|| {
        format!(
            "fixture omitted source_artifact: {}",
            fixture_path.display()
        )
    })
}

fn committed_decision_file(file_name: &str, path: &str) -> anyhow::Result<Vec<u8>> {
    let root = Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(Path::parent)
        .context("resolve workspace root")?;
    let fixture_path = root.join("tests/golden/compatibility").join(file_name);
    let fixture = read_json(&fixture_path)?;
    let files = fixture
        .get("source_files")
        .and_then(Value::as_array)
        .with_context(|| format!("fixture omitted source_files: {}", fixture_path.display()))?;
    let file = files
        .iter()
        .find(|file| file.get("path").and_then(Value::as_str) == Some(path))
        .with_context(|| {
            format!(
                "fixture omitted source file {path}: {}",
                fixture_path.display()
            )
        })?;
    let contents = file
        .get("contents_hex")
        .and_then(Value::as_str)
        .with_context(|| {
            format!(
                "source file omitted contents_hex: {}",
                fixture_path.display()
            )
        })?;
    decode_lower_hex(contents)
}

fn decode_lower_hex(text: &str) -> anyhow::Result<Vec<u8>> {
    ensure!(
        text.len().is_multiple_of(2) && !text.is_empty(),
        "historical source file hex must contain complete bytes"
    );
    text.as_bytes()
        .chunks_exact(2)
        .map(|pair| {
            let nibble = |byte| match byte {
                b'0'..=b'9' => Ok(byte - b'0'),
                b'a'..=b'f' => Ok(byte - b'a' + 10),
                _ => anyhow::bail!("historical source file hex must use lowercase ASCII"),
            };
            Ok((nibble(pair[0])? << 4) | nibble(pair[1])?)
        })
        .collect()
}

fn mobile_manipulator_snapshot_fixture() -> anyhow::Result<Value> {
    let scene = mm_minimal_scene_path()
        .canonicalize()
        .context("canonicalize historical snapshot scene")?;
    let source_sim = MobileManipulatorSim::from_scene_path(&scene)
        .context("load historical snapshot source scene")?;
    let mut source = serde_json::to_value(source_sim.snapshot())?;
    let source_object = source
        .as_object_mut()
        .context("mobile-manipulator snapshot must serialize as an object")?;
    source_object.insert(
        "schema_version".to_string(),
        MOBILE_MANIPULATOR_SIM_SNAPSHOT_MIN_VERSION.into(),
    );
    ensure!(
        source_object.remove("wrist_depth_frame").is_some(),
        "current snapshot omitted wrist_depth_frame"
    );
    ensure!(
        source_object.remove("grasp_retarget").is_some(),
        "current snapshot omitted grasp_retarget"
    );
    drop(source_sim);

    let historical: MobileManipulatorSimSnapshot = serde_json::from_value(source.clone())?;
    let mut restored = MobileManipulatorSim::from_scene_path(&scene)
        .context("load historical snapshot restore scene")?;
    restored
        .restore_snapshot(&historical)
        .map_err(|error| anyhow::anyhow!("restore historical snapshot: {error:?}"))?;
    let current = serde_json::to_value(restored.snapshot())?;
    ensure!(
        current.get("schema_version").and_then(Value::as_u64)
            == Some(u64::from(MOBILE_MANIPULATOR_SIM_SNAPSHOT_VERSION)),
        "restored snapshot did not normalize to the current schema"
    );
    let current = canonical_state_value(&current);

    Ok(json!({
        "kind": "rne_historical_migration_case",
        "schema_version": 1,
        "artifact_contract": "mobile_manipulator_sim_snapshot",
        "source_schema_version": MOBILE_MANIPULATOR_SIM_SNAPSHOT_MIN_VERSION,
        "current_schema_version": MOBILE_MANIPULATOR_SIM_SNAPSHOT_VERSION,
        "expected_outcome": "accepted_within_tolerance",
        "float_tolerance": HISTORICAL_MIGRATION_FLOAT_TOLERANCE,
        "source_snapshot": source,
        "current_snapshot_sha256": sha256(&serde_json::to_vec(&current)?),
    }))
}

fn frontend_fixture() -> anyhow::Result<Value> {
    let hello = ClientHello {
        min_protocol_major: TRANSPORT_PROTOCOL_MAJOR,
        max_protocol_major: TRANSPORT_PROTOCOL_MAJOR,
        capabilities: TransportCapabilities::ALL_V1,
        required_capabilities: TransportCapabilities::CONTROL.union(TransportCapabilities::STATUS),
        max_payload_bytes: 1024 * 1024,
        queue_frame_limit: 16,
        queue_byte_limit: 2 * 1024 * 1024,
        resume_after_sequence: Some(41),
    };
    let frame = TransportFrame::new(
        TransportMessageKind::ClientHello,
        42,
        0x1122_3344_5566_7788,
        hello.encode_payload(),
    );
    let negotiated = negotiate_transport(hello, NegotiationPolicy::default())
        .map_err(|reject| anyhow::anyhow!("reference hello rejected: {:?}", reject.code))?;
    Ok(json!({
        "schema_version": 1,
        "message_kind": "client_hello",
        "protocol_major": frame.protocol_major,
        "protocol_minor": frame.protocol_minor,
        "flags": frame.flags,
        "sequence": frame.sequence,
        "session_id": frame.session_id,
        "frame_hex": lower_hex(&frame.encode()?),
        "hello": {
            "min_protocol_major": hello.min_protocol_major,
            "max_protocol_major": hello.max_protocol_major,
            "capabilities_bits": hello.capabilities.bits(),
            "required_capabilities_bits": hello.required_capabilities.bits(),
            "max_payload_bytes": hello.max_payload_bytes,
            "queue_frame_limit": hello.queue_frame_limit,
            "queue_byte_limit": hello.queue_byte_limit,
            "resume_after_sequence": hello.resume_after_sequence,
        },
        "negotiated": {
            "protocol_major": negotiated.protocol_major,
            "protocol_minor": negotiated.protocol_minor,
            "capabilities_bits": negotiated.capabilities.bits(),
            "max_payload_bytes": negotiated.max_payload_bytes,
            "queue_frame_limit": negotiated.queue_frame_limit,
            "queue_byte_limit": negotiated.queue_byte_limit,
            "resume_after_sequence": negotiated.resume_after_sequence,
        }
    }))
}

fn dataset_fixture() -> anyhow::Result<Value> {
    let metadata = SensorFrameMetadata {
        stream_id: 17,
        sensor_sequence: 42,
        capture_ticks: 1_000_000,
        available_ticks: 1_250_000,
    };
    ensure!(
        metadata.available_ticks >= metadata.capture_ticks,
        "reference dataset timestamp order is invalid"
    );
    let imu = ImuSample {
        angular_velocity_rad_s: Vec3::new(0.125, -0.25, 0.5),
        linear_acceleration_m_s2: Vec3::new(0.0, 9.806_65, -1.5),
    };
    let transform = PoseSample {
        position_m: Vec3::new(1.25, -2.5, 0.75),
        yaw_rad: -0.625,
    };
    let action = DatasetActionSample {
        values: vec![-1.0, 0.25, 2.5],
    };
    let outcome = DatasetTaskOutcomeSample {
        episode_index: 7,
        step_in_episode: 19,
        reward: 0.75,
        cumulative_reward: 12.5,
        terminated: true,
        truncated: false,
        success: Some(true),
    };
    let annotation = DatasetGroundTruthAnnotation {
        class_id: 3,
        instance_id: 99,
        values: vec![1.5, -2.0, 0.125, 4.75],
    };
    Ok(json!({
        "schema_version": DATASET_PAYLOAD_SCHEMA_VERSION,
        "metadata": {
            "stream_id": metadata.stream_id,
            "sensor_sequence": metadata.sensor_sequence,
            "capture_ticks": metadata.capture_ticks,
            "available_ticks": metadata.available_ticks,
        },
        "imu": {
            "angular_velocity_rad_s": [imu.angular_velocity_rad_s.x, imu.angular_velocity_rad_s.y, imu.angular_velocity_rad_s.z],
            "linear_acceleration_m_s2": [imu.linear_acceleration_m_s2.x, imu.linear_acceleration_m_s2.y, imu.linear_acceleration_m_s2.z],
            "payload_hex": lower_hex(&encode_dataset_imu(metadata, &imu)?),
        },
        "transform": {
            "position_m": [transform.position_m.x, transform.position_m.y, transform.position_m.z],
            "yaw_rad": transform.yaw_rad,
            "payload_hex": lower_hex(&encode_dataset_transform(metadata, &transform)?),
        },
        "action": {
            "values": action.values,
            "payload_hex": lower_hex(&encode_dataset_action(metadata, &action)?),
        },
        "outcome": {
            "episode_index": outcome.episode_index,
            "step_in_episode": outcome.step_in_episode,
            "reward": outcome.reward,
            "cumulative_reward": outcome.cumulative_reward,
            "terminated": outcome.terminated,
            "truncated": outcome.truncated,
            "success": outcome.success,
            "payload_hex": lower_hex(&encode_dataset_task_outcome(metadata, &outcome)?),
        },
        "annotation": {
            "class_id": annotation.class_id,
            "instance_id": annotation.instance_id,
            "values": annotation.values,
            "payload_hex": lower_hex(&encode_dataset_annotation(metadata, &annotation)?),
        },
    }))
}

fn lower_hex(bytes: &[u8]) -> String {
    bytes.iter().map(|byte| format!("{byte:02x}")).collect()
}

fn sha256(bytes: &[u8]) -> String {
    format!("sha256:{:x}", Sha256::digest(bytes))
}

fn canonical_state_value(value: &Value) -> Value {
    match value {
        Value::Number(number) if number.is_f64() => {
            let value = number.as_f64().expect("JSON float");
            let rounded = (value * 1_000_000_000.0).round() / 1_000_000_000.0;
            Value::from(if rounded == 0.0 { 0.0 } else { rounded })
        }
        Value::Array(values) => Value::Array(values.iter().map(canonical_state_value).collect()),
        Value::Object(values) => {
            let mut entries = values.iter().collect::<Vec<_>>();
            entries.sort_by_key(|(key, _)| *key);
            Value::Object(
                entries
                    .into_iter()
                    .map(|(key, value)| (key.clone(), canonical_state_value(value)))
                    .collect(),
            )
        }
        _ => value.clone(),
    }
}

fn write_json(path: &Path, value: &Value) -> anyhow::Result<()> {
    let mut bytes = serde_json::to_vec_pretty(value)?;
    bytes.push(b'\n');
    let parsed: Value = serde_json::from_slice(&bytes)?;
    let canonical_digest = sha256(&serde_json::to_vec(&parsed)?);
    fs::write(path, bytes).with_context(|| format!("write fixture {}", path.display()))?;
    println!("{} {canonical_digest}", path.display());
    Ok(())
}
