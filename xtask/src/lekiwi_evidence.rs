//! Seal and verify complete physical LeKiwi reference-device evidence.

use anyhow::{Context, Result};
use rne_ai::{TaskSpec, TASK_SPEC_KIND, TASK_SPEC_SCHEMA_VERSION};
use rne_data::{
    DatasetBundle, DatasetStreamKind, DepthPairEvaluationReport, DATASET_BUNDLE_SCHEMA_VERSION,
    DATASET_OFFLINE_EVALUATION_SCHEMA_VERSION,
};
use rne_hardware_gateway::shadow::{
    ShadowComparisonReport, SHADOW_COMPARISON_REPORT_KIND, SHADOW_COMPARISON_SCHEMA_VERSION,
};
use rne_hardware_gateway::wire::{
    DeviceWirePayload, HardwareWireTrace, HardwareWireTraceEntry, HardwareWireTraceOutcome,
    HostWirePayload, HARDWARE_WIRE_TRACE_KIND,
};
use rne_hardware_gateway::{GatewayEvent, HardwareMode, SafetyReason};
use rne_hardware_lekiwi::physical_evidence::{
    EvidenceFileRef, HostTerminationDiagnostic, LeKiwiPhysicalEvidenceManifest,
    PowerIsolationDiagnostic, LEKIWI_CALIBRATION_EVIDENCE_KIND,
    LEKIWI_CAMERA_DATASET_MANIFEST_KIND, LEKIWI_CAMERA_OFFLINE_EVALUATION_KIND,
    LEKIWI_CLEAN_HOST_REPRODUCTION_KIND, LEKIWI_ELEVATED_SHADOW_MIN_SAMPLES,
    LEKIWI_FLOOR_LIVE_MAX_LINEAR_SPEED_M_S, LEKIWI_HOST_TERMINATION_DIAGNOSTIC_KIND,
    LEKIWI_PHYSICAL_DIAGNOSTIC_SCHEMA_VERSION, LEKIWI_POWER_ISOLATION_DIAGNOSTIC_KIND,
};
use rne_hardware_lekiwi::session::{
    LeKiwiReferenceSessionEvidence, LEKIWI_REFERENCE_SESSION_KIND,
    LEKIWI_REFERENCE_SESSION_SCHEMA_VERSION,
};
use rne_hardware_lekiwi::{
    lekiwi_reference_profile_v1, LeKiwiReferenceProfile, LEKIWI_REFERENCE_PROFILE_KIND,
    LEKIWI_REFERENCE_PROFILE_SCHEMA_VERSION,
};
use rne_log::{FailureCapsule, FAILURE_CAPSULE_KIND, FAILURE_CAPSULE_SCHEMA_VERSION};
use serde::de::DeserializeOwned;
use sha2::{Digest, Sha256};
use std::collections::BTreeSet;
use std::fs;
use std::io::Write;
use std::path::{Path, PathBuf};

const MAX_INDEXED_ARTIFACT_BYTES: u64 = 64 * 1024 * 1024;

/// Runs `lekiwi-evidence extract-trace|seal|verify`.
pub(crate) fn run(args: &mut impl Iterator<Item = String>) -> Result<()> {
    let command = args.next().ok_or_else(|| {
        anyhow::anyhow!(
            "lekiwi-evidence requires `extract-trace SESSION OUTPUT`, `seal DRAFT OUTPUT`, or `verify MANIFEST`; see docs/REFERENCE_HARDWARE_LEKIWI.md"
        )
    })?;
    match command.as_str() {
        "extract-trace" => {
            let session = required_arg(args, "extract-trace requires SESSION and OUTPUT")?;
            let output = required_arg(args, "extract-trace requires OUTPUT")?;
            no_more_args(args, "extract-trace")?;
            extract_trace(Path::new(&session), Path::new(&output))
        }
        "seal" => {
            let draft = required_arg(args, "seal requires DRAFT and OUTPUT")?;
            let output = required_arg(args, "seal requires OUTPUT")?;
            no_more_args(args, "seal")?;
            seal(Path::new(&draft), Path::new(&output))
        }
        "verify" => {
            let manifest = required_arg(args, "verify requires MANIFEST")?;
            no_more_args(args, "verify")?;
            verify(Path::new(&manifest))
        }
        "--help" | "-h" => {
            println!(
                "lekiwi-evidence extract-trace SESSION OUTPUT\nlekiwi-evidence seal DRAFT OUTPUT\nlekiwi-evidence verify MANIFEST"
            );
            Ok(())
        }
        other => anyhow::bail!(
            "unknown lekiwi-evidence command `{other}`; expected `extract-trace`, `seal`, or `verify`"
        ),
    }
}

fn required_arg(args: &mut impl Iterator<Item = String>, message: &str) -> Result<String> {
    args.next()
        .ok_or_else(|| anyhow::anyhow!(message.to_string()))
}

fn no_more_args(args: &mut impl Iterator<Item = String>, command: &str) -> Result<()> {
    anyhow::ensure!(
        args.next().is_none(),
        "lekiwi-evidence {command} received too many arguments"
    );
    Ok(())
}

fn extract_trace(session_path: &Path, output_path: &Path) -> Result<()> {
    let session: LeKiwiReferenceSessionEvidence =
        read_json_file(session_path, "LeKiwi reference session")?;
    session
        .validate()
        .map_err(|error| anyhow::anyhow!("invalid LeKiwi reference session: {error}"))?;
    write_new_json(output_path, &session.session.wire_trace)?;
    println!(
        "extracted hardware wire trace {} from session {}",
        output_path.display(),
        session.session.session_id
    );
    Ok(())
}

fn seal(draft_path: &Path, output_path: &Path) -> Result<()> {
    let bytes = read_regular_file(draft_path, "physical evidence draft")?;
    let mut manifest: LeKiwiPhysicalEvidenceManifest =
        serde_json::from_slice(&bytes).context("physical evidence draft is not valid JSON")?;
    anyhow::ensure!(
        manifest.content_sha256.is_empty(),
        "physical evidence draft content_sha256 must be empty before sealing"
    );
    let root = draft_path.parent().unwrap_or_else(|| Path::new("."));
    let output_root = output_path.parent().unwrap_or_else(|| Path::new("."));
    let canonical_root = fs::canonicalize(root)
        .with_context(|| format!("could not resolve draft root `{}`", root.display()))?;
    anyhow::ensure!(
        canonical_root == fs::canonicalize(output_root)?,
        "DRAFT and OUTPUT must share a directory because paths are manifest-relative"
    );

    for (role, artifact) in manifest.artifacts.all_mut() {
        artifact.sha256 = digest_draft_artifact(&canonical_root, root, role, artifact)?;
    }
    manifest.inventory.calibration.sha256 = digest_draft_artifact(
        &canonical_root,
        root,
        "calibration",
        &manifest.inventory.calibration,
    )?;
    manifest.power_isolation.diagnostic.sha256 = digest_draft_artifact(
        &canonical_root,
        root,
        "power_isolation_diagnostic",
        &manifest.power_isolation.diagnostic,
    )?;
    manifest.host_termination.diagnostic.sha256 = digest_draft_artifact(
        &canonical_root,
        root,
        "host_termination_diagnostic",
        &manifest.host_termination.diagnostic,
    )?;
    manifest
        .seal()
        .map_err(|error| anyhow::anyhow!("could not seal physical evidence manifest: {error}"))?;
    manifest
        .validate()
        .map_err(|error| anyhow::anyhow!("invalid sealed physical evidence manifest: {error}"))?;
    verify_declared_contracts(&manifest)?;
    write_new_json(output_path, &manifest)?;
    println!(
        "sealed LeKiwi physical evidence manifest {} ({})",
        output_path.display(),
        manifest.content_sha256
    );
    Ok(())
}

fn verify(manifest_path: &Path) -> Result<()> {
    let bytes = read_regular_file(manifest_path, "physical evidence manifest")?;
    let manifest: LeKiwiPhysicalEvidenceManifest =
        serde_json::from_slice(&bytes).context("physical evidence manifest is not valid JSON")?;
    manifest
        .validate()
        .map_err(|error| anyhow::anyhow!("invalid physical evidence manifest: {error}"))?;
    verify_declared_contracts(&manifest)?;

    let root = manifest_path.parent().unwrap_or_else(|| Path::new("."));
    let canonical_root = fs::canonicalize(root)
        .with_context(|| format!("could not resolve evidence root `{}`", root.display()))?;
    for (role, artifact) in manifest.artifacts.all().into_iter().chain([
        ("calibration", &manifest.inventory.calibration),
        (
            "power_isolation_diagnostic",
            &manifest.power_isolation.diagnostic,
        ),
        (
            "host_termination_diagnostic",
            &manifest.host_termination.diagnostic,
        ),
    ]) {
        checked_ref_path(&canonical_root, root, role, artifact)?;
    }

    let task: TaskSpec = read_ref_json(
        &canonical_root,
        root,
        "task_spec",
        &manifest.artifacts.task_spec,
    )?;
    task.validate()
        .map_err(|error| anyhow::anyhow!("invalid physical evidence TaskSpec: {error}"))?;
    let expected_profile = lekiwi_reference_profile_v1();
    anyhow::ensure!(
        task == expected_profile.task,
        "physical evidence TaskSpec differs from the built-in LeKiwi task"
    );
    let profile: LeKiwiReferenceProfile = read_ref_json(
        &canonical_root,
        root,
        "reference_profile",
        &manifest.artifacts.reference_profile,
    )?;
    profile
        .validate()
        .map_err(|error| anyhow::anyhow!("invalid physical reference profile: {error}"))?;
    anyhow::ensure!(
        profile == expected_profile,
        "physical evidence profile differs from the built-in LeKiwi profile"
    );

    let shadow = read_session(
        &canonical_root,
        root,
        "elevated_shadow_session",
        &manifest.artifacts.elevated_shadow_session,
        &manifest.inventory.device_id,
    )?;
    require_outcome(
        &shadow,
        HardwareMode::Shadow,
        HardwareWireTraceOutcome::Completed,
        "elevated_shadow_session",
    )?;
    let shadow_observations = observation_count(&shadow);
    anyhow::ensure!(
        shadow_observations >= LEKIWI_ELEVATED_SHADOW_MIN_SAMPLES,
        "elevated shadow has {shadow_observations} observations; {LEKIWI_ELEVATED_SHADOW_MIN_SAMPLES} required"
    );
    anyhow::ensure!(
        !shadow
            .session
            .wire_trace
            .entries
            .iter()
            .any(is_host_actuate),
        "elevated shadow contains a forbidden host Actuate frame"
    );
    verify_shadow_comparison(&canonical_root, root, &manifest, &task, shadow_observations)?;

    let mut sessions = vec![shadow];
    for (role, artifact, outcome) in [
        (
            "command_deadline_session",
            &manifest.artifacts.command_deadline_session,
            HardwareWireTraceOutcome::GatewaySafetyStopped {
                reason: SafetyReason::CommandDeadlineMissed,
            },
        ),
        (
            "device_watchdog_session",
            &manifest.artifacts.device_watchdog_session,
            HardwareWireTraceOutcome::SafetyStopped {
                reason: SafetyReason::CommandStale,
            },
        ),
        (
            "actuator_limit_session",
            &manifest.artifacts.actuator_limit_session,
            HardwareWireTraceOutcome::GatewaySafetyStopped {
                reason: SafetyReason::ActuatorLimit,
            },
        ),
        (
            "emergency_stop_session",
            &manifest.artifacts.emergency_stop_session,
            HardwareWireTraceOutcome::GatewaySafetyStopped {
                reason: SafetyReason::EmergencyStop,
            },
        ),
    ] {
        let session = read_session(
            &canonical_root,
            root,
            role,
            artifact,
            &manifest.inventory.device_id,
        )?;
        require_outcome(&session, HardwareMode::Hil, outcome, role)?;
        sessions.push(session);
    }

    let reconnect = read_session(
        &canonical_root,
        root,
        "reconnect_session",
        &manifest.artifacts.reconnect_session,
        &manifest.inventory.device_id,
    )?;
    require_outcome(
        &reconnect,
        HardwareMode::Hil,
        HardwareWireTraceOutcome::Completed,
        "reconnect_session",
    )?;
    validate_reconnect_rearm(&reconnect)?;
    let reconnect_session_id = reconnect.session.session_id.clone();
    sessions.push(reconnect);

    let live_success = read_session(
        &canonical_root,
        root,
        "low_speed_live_success_session",
        &manifest.artifacts.low_speed_live_success_session,
        &manifest.inventory.device_id,
    )?;
    require_outcome(
        &live_success,
        HardwareMode::Live,
        HardwareWireTraceOutcome::Completed,
        "low_speed_live_success_session",
    )?;
    validate_low_speed_motion(&live_success)?;
    sessions.push(live_success);

    let live_failure = read_session(
        &canonical_root,
        root,
        "low_speed_live_failure_session",
        &manifest.artifacts.low_speed_live_failure_session,
        &manifest.inventory.device_id,
    )?;
    anyhow::ensure!(
        live_failure.session.mode == HardwareMode::Live
            && live_failure.session.wire_trace.outcome != HardwareWireTraceOutcome::Completed,
        "low_speed_live_failure_session must be a safely terminated live session"
    );
    sessions.push(live_failure);

    let mut session_ids = BTreeSet::new();
    for session in &sessions {
        anyhow::ensure!(
            session_ids.insert(session.session.session_id.as_str()),
            "every physical stage must use a fresh session ID"
        );
    }
    verify_operator_diagnostics(
        &canonical_root,
        root,
        &manifest,
        &reconnect_session_id,
        &session_ids,
    )?;
    verify_camera_dataset(&canonical_root, root, &manifest)?;
    verify_failure_capsule(&canonical_root, root, &manifest)?;

    println!(
        "verified LeKiwi physical evidence {}: device={} sessions={} shadow_samples={}",
        manifest.run_id,
        manifest.inventory.device_id,
        sessions.len(),
        shadow_observations
    );
    Ok(())
}

fn verify_declared_contracts(manifest: &LeKiwiPhysicalEvidenceManifest) -> Result<()> {
    for (role, artifact, kind, schema) in [
        (
            "calibration",
            &manifest.inventory.calibration,
            LEKIWI_CALIBRATION_EVIDENCE_KIND,
            1,
        ),
        (
            "power_isolation_diagnostic",
            &manifest.power_isolation.diagnostic,
            LEKIWI_POWER_ISOLATION_DIAGNOSTIC_KIND,
            LEKIWI_PHYSICAL_DIAGNOSTIC_SCHEMA_VERSION,
        ),
        (
            "host_termination_diagnostic",
            &manifest.host_termination.diagnostic,
            LEKIWI_HOST_TERMINATION_DIAGNOSTIC_KIND,
            LEKIWI_PHYSICAL_DIAGNOSTIC_SCHEMA_VERSION,
        ),
        (
            "task_spec",
            &manifest.artifacts.task_spec,
            TASK_SPEC_KIND,
            TASK_SPEC_SCHEMA_VERSION,
        ),
        (
            "reference_profile",
            &manifest.artifacts.reference_profile,
            LEKIWI_REFERENCE_PROFILE_KIND,
            LEKIWI_REFERENCE_PROFILE_SCHEMA_VERSION,
        ),
        (
            "elevated_shadow_comparison",
            &manifest.artifacts.elevated_shadow_comparison,
            SHADOW_COMPARISON_REPORT_KIND,
            SHADOW_COMPARISON_SCHEMA_VERSION,
        ),
        (
            "camera_dataset_manifest",
            &manifest.artifacts.camera_dataset_manifest,
            LEKIWI_CAMERA_DATASET_MANIFEST_KIND,
            DATASET_BUNDLE_SCHEMA_VERSION,
        ),
        (
            "camera_offline_evaluation",
            &manifest.artifacts.camera_offline_evaluation,
            LEKIWI_CAMERA_OFFLINE_EVALUATION_KIND,
            DATASET_OFFLINE_EVALUATION_SCHEMA_VERSION,
        ),
        (
            "failure_capsule_manifest",
            &manifest.artifacts.failure_capsule_manifest,
            FAILURE_CAPSULE_KIND,
            FAILURE_CAPSULE_SCHEMA_VERSION,
        ),
        (
            "clean_host_reproduction",
            &manifest.artifacts.clean_host_reproduction,
            LEKIWI_CLEAN_HOST_REPRODUCTION_KIND,
            1,
        ),
    ] {
        require_ref_contract(role, artifact, kind, schema)?;
    }
    for (role, artifact) in [
        (
            "elevated_shadow_session",
            &manifest.artifacts.elevated_shadow_session,
        ),
        (
            "command_deadline_session",
            &manifest.artifacts.command_deadline_session,
        ),
        (
            "device_watchdog_session",
            &manifest.artifacts.device_watchdog_session,
        ),
        (
            "actuator_limit_session",
            &manifest.artifacts.actuator_limit_session,
        ),
        (
            "emergency_stop_session",
            &manifest.artifacts.emergency_stop_session,
        ),
        ("reconnect_session", &manifest.artifacts.reconnect_session),
        (
            "low_speed_live_success_session",
            &manifest.artifacts.low_speed_live_success_session,
        ),
        (
            "low_speed_live_failure_session",
            &manifest.artifacts.low_speed_live_failure_session,
        ),
    ] {
        require_ref_contract(
            role,
            artifact,
            LEKIWI_REFERENCE_SESSION_KIND,
            LEKIWI_REFERENCE_SESSION_SCHEMA_VERSION,
        )?;
    }
    Ok(())
}

fn require_ref_contract(
    role: &str,
    artifact: &EvidenceFileRef,
    expected_kind: &str,
    expected_schema: u32,
) -> Result<()> {
    anyhow::ensure!(
        artifact.kind == expected_kind && artifact.schema_version == expected_schema,
        "{role} declares {}/{}, expected {expected_kind}/{expected_schema}",
        artifact.kind,
        artifact.schema_version
    );
    Ok(())
}

fn verify_shadow_comparison(
    canonical_root: &Path,
    root: &Path,
    manifest: &LeKiwiPhysicalEvidenceManifest,
    task: &TaskSpec,
    observation_count: usize,
) -> Result<()> {
    let comparison: ShadowComparisonReport = read_ref_json(
        canonical_root,
        root,
        "elevated_shadow_comparison",
        &manifest.artifacts.elevated_shadow_comparison,
    )?;
    comparison
        .validate_against(task)
        .map_err(|error| anyhow::anyhow!("invalid elevated shadow comparison: {error}"))?;
    anyhow::ensure!(
        comparison.summary.passed
            && comparison.summary.compared_samples == observation_count
            && observation_count >= LEKIWI_ELEVATED_SHADOW_MIN_SAMPLES,
        "shadow comparison must pass and cover every required observation"
    );
    Ok(())
}

fn verify_operator_diagnostics(
    canonical_root: &Path,
    root: &Path,
    manifest: &LeKiwiPhysicalEvidenceManifest,
    reconnect_session_id: &str,
    complete_session_ids: &BTreeSet<&str>,
) -> Result<()> {
    let power: PowerIsolationDiagnostic = read_ref_json(
        canonical_root,
        root,
        "power_isolation_diagnostic",
        &manifest.power_isolation.diagnostic,
    )?;
    power
        .validate()
        .map_err(|error| anyhow::anyhow!("invalid power-isolation diagnostic: {error}"))?;
    anyhow::ensure!(
        power.run_id == manifest.run_id
            && power.device_id == manifest.inventory.device_id
            && power.primary_operator_id == manifest.power_isolation.primary_operator_id
            && power.safety_operator_id == manifest.power_isolation.safety_operator_id,
        "power-isolation diagnostic does not match manifest run, device, or operators"
    );

    let host: HostTerminationDiagnostic = read_ref_json(
        canonical_root,
        root,
        "host_termination_diagnostic",
        &manifest.host_termination.diagnostic,
    )?;
    host.validate()
        .map_err(|error| anyhow::anyhow!("invalid host-termination diagnostic: {error}"))?;
    anyhow::ensure!(
        host.run_id == manifest.run_id
            && host.device_id == manifest.inventory.device_id
            && host.observer_operator_id == manifest.host_termination.observer_operator_id
            && host.terminated_session_id == manifest.host_termination.terminated_session_id
            && host.reconnect_session_id == reconnect_session_id
            && host.safe_stop_observed == manifest.host_termination.safe_stop_observed
            && host.stop_latency_ms == manifest.host_termination.stop_latency_ms
            && host.measurement_uncertainty_ms
                == manifest.host_termination.measurement_uncertainty_ms,
        "host-termination diagnostic does not match manifest or reconnect session"
    );
    anyhow::ensure!(
        !complete_session_ids.contains(host.terminated_session_id.as_str()),
        "terminated host request was incorrectly presented as a complete session"
    );
    Ok(())
}

fn verify_camera_dataset(
    canonical_root: &Path,
    root: &Path,
    manifest: &LeKiwiPhysicalEvidenceManifest,
) -> Result<()> {
    let manifest_path = checked_ref_path(
        canonical_root,
        root,
        "camera_dataset_manifest",
        &manifest.artifacts.camera_dataset_manifest,
    )?;
    anyhow::ensure!(
        manifest_path.file_name().and_then(|name| name.to_str()) == Some("manifest.json"),
        "camera_dataset_manifest must name manifest.json"
    );
    let dataset_root = manifest_path
        .parent()
        .ok_or_else(|| anyhow::anyhow!("camera dataset has no parent directory"))?;
    let root_metadata = fs::symlink_metadata(dataset_root)?;
    anyhow::ensure!(
        !root_metadata.file_type().is_symlink() && root_metadata.is_dir(),
        "camera dataset root must be a regular non-symlink directory"
    );
    let bundle = DatasetBundle::open(dataset_root)
        .map_err(|error| anyhow::anyhow!("invalid camera dataset bundle: {error}"))?;
    let shard_path = dataset_root.join(Path::new(&bundle.manifest().shards[0].path));
    let shard_metadata = fs::symlink_metadata(&shard_path)?;
    anyhow::ensure!(
        !shard_metadata.file_type().is_symlink()
            && shard_metadata.is_file()
            && fs::canonicalize(&shard_path)?.starts_with(canonical_root),
        "camera dataset shard must be a regular file inside the evidence root"
    );
    bundle
        .verify()
        .map_err(|error| anyhow::anyhow!("camera dataset payload verification failed: {error}"))?;
    anyhow::ensure!(
        bundle.manifest().task_spec_sha256 == manifest.artifacts.task_spec.sha256,
        "camera dataset is not bound to the indexed TaskSpec bytes"
    );

    for camera in &lekiwi_reference_profile_v1().camera_streams {
        let stream = bundle
            .manifest()
            .streams
            .iter()
            .find(|stream| stream.name == camera.stream_name)
            .ok_or_else(|| anyhow::anyhow!("missing camera stream {:?}", camera.stream_name))?;
        anyhow::ensure!(
            stream.kind == DatasetStreamKind::Rgb8 && stream.calibration.is_some(),
            "camera stream {:?} must be calibrated RGB8",
            stream.name
        );
        let summary = bundle.manifest().shards[0]
            .streams
            .iter()
            .find(|summary| summary.stream_id == stream.stream_id)
            .ok_or_else(|| anyhow::anyhow!("camera stream {:?} has no summary", stream.name))?;
        anyhow::ensure!(
            summary.sample_count > 0,
            "camera stream contains no samples"
        );
    }

    let report: DepthPairEvaluationReport = read_ref_json(
        canonical_root,
        root,
        "camera_offline_evaluation",
        &manifest.artifacts.camera_offline_evaluation,
    )?;
    report
        .validate()
        .map_err(|error| anyhow::anyhow!("invalid camera offline evaluation: {error}"))?;
    anyhow::ensure!(
        report.passed && report.dataset_manifest_sha256 == bundle.manifest().content_sha256,
        "offline evaluation must pass and bind the verified dataset"
    );
    for stream_id in [report.predicted_stream, report.ground_truth_stream] {
        let stream = bundle
            .manifest()
            .streams
            .iter()
            .find(|stream| stream.stream_id == stream_id)
            .ok_or_else(|| anyhow::anyhow!("offline evaluation references an unknown stream"))?;
        anyhow::ensure!(
            stream.kind == DatasetStreamKind::DepthF32,
            "offline depth evaluation references a non-depth stream"
        );
    }
    Ok(())
}

fn verify_failure_capsule(
    canonical_root: &Path,
    root: &Path,
    manifest: &LeKiwiPhysicalEvidenceManifest,
) -> Result<()> {
    let path = checked_ref_path(
        canonical_root,
        root,
        "failure_capsule_manifest",
        &manifest.artifacts.failure_capsule_manifest,
    )?;
    anyhow::ensure!(
        path.file_name().and_then(|name| name.to_str()) == Some("capsule.json"),
        "failure_capsule_manifest must name capsule.json"
    );
    let capsule: FailureCapsule = read_json_file(&path, "failure_capsule_manifest")?;
    capsule
        .validate()
        .map_err(|error| anyhow::anyhow!("invalid Failure Capsule manifest: {error}"))?;
    anyhow::ensure!(
        capsule.build.git_commit == manifest.rne_commit,
        "Failure Capsule build commit differs from the physical evidence commit"
    );
    for (kind, expected) in [
        (TASK_SPEC_KIND, &manifest.artifacts.task_spec),
        (
            LEKIWI_REFERENCE_SESSION_KIND,
            &manifest.artifacts.elevated_shadow_session,
        ),
        (
            SHADOW_COMPARISON_REPORT_KIND,
            &manifest.artifacts.elevated_shadow_comparison,
        ),
    ] {
        let digest = expected
            .sha256
            .strip_prefix("sha256:")
            .expect("manifest validation guarantees prefix");
        anyhow::ensure!(
            capsule
                .artifacts
                .iter()
                .any(|artifact| artifact.kind == kind && artifact.sha256 == digest),
            "Failure Capsule does not contain the indexed {kind} bytes"
        );
    }
    anyhow::ensure!(
        capsule
            .artifacts
            .iter()
            .any(|artifact| artifact.kind == HARDWARE_WIRE_TRACE_KIND),
        "Failure Capsule has no standalone hardware wire trace"
    );
    anyhow::ensure!(
        capsule
            .artifacts
            .iter()
            .any(|artifact| matches!(artifact.kind.as_str(), "rne_replay" | "rne_behavior_replay")),
        "Failure Capsule has no simulation failure replay"
    );

    let capsule_root = path
        .parent()
        .ok_or_else(|| anyhow::anyhow!("Failure Capsule has no parent directory"))?;
    super::failure_capsule::verify_directory(capsule_root)?;
    let indexed_session: LeKiwiReferenceSessionEvidence = read_ref_json(
        canonical_root,
        root,
        "elevated_shadow_session",
        &manifest.artifacts.elevated_shadow_session,
    )?;
    let mut matching_trace = false;
    for artifact in capsule
        .artifacts
        .iter()
        .filter(|artifact| artifact.kind == HARDWARE_WIRE_TRACE_KIND)
    {
        let trace: HardwareWireTrace = read_json_file(
            &capsule_root.join(Path::new(&artifact.path)),
            "capsule hardware wire trace",
        )?;
        if trace == indexed_session.session.wire_trace {
            matching_trace = true;
            break;
        }
    }
    anyhow::ensure!(
        matching_trace,
        "Failure Capsule wire trace differs from the indexed shadow session trace"
    );
    Ok(())
}

fn read_session(
    canonical_root: &Path,
    root: &Path,
    role: &'static str,
    artifact: &EvidenceFileRef,
    expected_device_id: &str,
) -> Result<LeKiwiReferenceSessionEvidence> {
    let session: LeKiwiReferenceSessionEvidence =
        read_ref_json(canonical_root, root, role, artifact)?;
    session
        .validate()
        .map_err(|error| anyhow::anyhow!("invalid {role}: {error}"))?;
    anyhow::ensure!(
        session.device_id == expected_device_id,
        "{role} used device {:?}, expected {:?}",
        session.device_id,
        expected_device_id
    );
    Ok(session)
}

fn require_outcome(
    session: &LeKiwiReferenceSessionEvidence,
    mode: HardwareMode,
    outcome: HardwareWireTraceOutcome,
    role: &str,
) -> Result<()> {
    anyhow::ensure!(
        session.session.mode == mode && session.session.wire_trace.outcome == outcome,
        "{role} has mode/outcome {:?}/{:?}, expected {:?}/{:?}",
        session.session.mode,
        session.session.wire_trace.outcome,
        mode,
        outcome
    );
    Ok(())
}

fn observation_count(session: &LeKiwiReferenceSessionEvidence) -> usize {
    session
        .session
        .wire_trace
        .entries
        .iter()
        .filter(|entry| {
            matches!(
                entry,
                HardwareWireTraceEntry::Device { frame }
                    if matches!(frame.payload, DeviceWirePayload::Observation { .. })
            )
        })
        .count()
}

fn is_host_actuate(entry: &HardwareWireTraceEntry) -> bool {
    matches!(
        entry,
        HardwareWireTraceEntry::Host { frame }
            if matches!(frame.payload, HostWirePayload::Actuate { .. })
    )
}

fn validate_low_speed_motion(session: &LeKiwiReferenceSessionEvidence) -> Result<()> {
    let mut moving_commands = 0_usize;
    for entry in &session.session.wire_trace.entries {
        let HardwareWireTraceEntry::Host { frame: host } = entry else {
            continue;
        };
        let HostWirePayload::Actuate { frame } = &host.payload else {
            continue;
        };
        if frame.safety_stop {
            continue;
        }
        anyhow::ensure!(
            frame.values.len() == 3
                && frame.values[0].abs() <= LEKIWI_FLOOR_LIVE_MAX_LINEAR_SPEED_M_S
                && frame.values[1].abs() <= LEKIWI_FLOOR_LIVE_MAX_LINEAR_SPEED_M_S,
            "floor-live command exceeded the 0.02 m/s per-axis envelope"
        );
        if frame.values.iter().any(|value| *value != 0.0) {
            moving_commands += 1;
        }
    }
    anyhow::ensure!(moving_commands > 0, "floor-live success contains no motion");
    Ok(())
}

fn validate_reconnect_rearm(session: &LeKiwiReferenceSessionEvidence) -> Result<()> {
    let events = &session.session.gateway.events;
    anyhow::ensure!(
        events
            .iter()
            .any(|event| matches!(event, GatewayEvent::Armed { .. }))
            && events.iter().any(|event| matches!(
                event,
                GatewayEvent::ActuationDelivered {
                    safety_stop: false,
                    action_sequence: Some(_),
                }
            )),
        "reconnect_session must explicitly rearm and deliver a normal action"
    );
    Ok(())
}

fn read_ref_json<T: DeserializeOwned>(
    canonical_root: &Path,
    root: &Path,
    role: &'static str,
    artifact: &EvidenceFileRef,
) -> Result<T> {
    let path = checked_ref_path(canonical_root, root, role, artifact)?;
    read_json_file(&path, role)
}

fn read_json_file<T: DeserializeOwned>(path: &Path, role: &str) -> Result<T> {
    let bytes = read_regular_file(path, role)?;
    serde_json::from_slice(&bytes).with_context(|| format!("{role} is not valid expected JSON"))
}

fn checked_ref_path(
    canonical_root: &Path,
    root: &Path,
    role: &'static str,
    artifact: &EvidenceFileRef,
) -> Result<PathBuf> {
    let path = root.join(Path::new(&artifact.path));
    let canonical = fs::canonicalize(&path)
        .with_context(|| format!("could not resolve {role} `{}`", path.display()))?;
    anyhow::ensure!(
        canonical.starts_with(canonical_root),
        "{role} escapes the evidence root through a symlink"
    );
    let bytes = read_regular_file(&path, role)?;
    let actual = format!("sha256:{:x}", Sha256::digest(bytes));
    anyhow::ensure!(
        actual == artifact.sha256,
        "SHA-256 mismatch for {role}: expected {}, got {actual}",
        artifact.sha256
    );
    Ok(path)
}

fn digest_draft_artifact(
    canonical_root: &Path,
    root: &Path,
    role: &'static str,
    artifact: &EvidenceFileRef,
) -> Result<String> {
    EvidenceFileRef::new(
        artifact.kind.clone(),
        artifact.schema_version,
        artifact.path.clone(),
        format!("sha256:{}", "0".repeat(64)),
    )
    .validate()
    .map_err(|error| anyhow::anyhow!("invalid draft {role}: {error}"))?;
    let path = root.join(Path::new(&artifact.path));
    let canonical = fs::canonicalize(&path)
        .with_context(|| format!("could not resolve {role} `{}`", path.display()))?;
    anyhow::ensure!(
        canonical.starts_with(canonical_root),
        "{role} escapes the evidence root through a symlink"
    );
    Ok(format!(
        "sha256:{:x}",
        Sha256::digest(read_regular_file(&path, role)?)
    ))
}

fn read_regular_file(path: &Path, role: &str) -> Result<Vec<u8>> {
    let metadata = fs::symlink_metadata(path)
        .with_context(|| format!("could not inspect {role} `{}`", path.display()))?;
    anyhow::ensure!(
        !metadata.file_type().is_symlink() && metadata.is_file(),
        "{role} must be a regular non-symlink file"
    );
    anyhow::ensure!(
        metadata.len() > 0 && metadata.len() <= MAX_INDEXED_ARTIFACT_BYTES,
        "{role} must contain 1..={MAX_INDEXED_ARTIFACT_BYTES} bytes"
    );
    fs::read(path).with_context(|| format!("could not read {role} `{}`", path.display()))
}

fn write_new_json(path: &Path, value: &impl serde::Serialize) -> Result<()> {
    let mut bytes = serde_json::to_vec_pretty(value)?;
    bytes.push(b'\n');
    let mut file = fs::OpenOptions::new()
        .create_new(true)
        .write(true)
        .open(path)
        .with_context(|| format!("refusing to overwrite `{}`", path.display()))?;
    file.write_all(&bytes)?;
    file.sync_all()?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use rne_hardware_gateway::wire::{DeviceWireFrame, HostWireFrame};
    use rne_hardware_lekiwi::session::{
        LeKiwiMonotonicClock, LeKiwiReferenceSampleOutcome, LeKiwiReferenceSessionConfig,
        LeKiwiReferenceSessionRunner, LeKiwiTransportError, LeKiwiWireTransport,
    };

    #[derive(Debug)]
    struct FixedClock;

    impl LeKiwiMonotonicClock for FixedClock {
        fn now_ms(&mut self) -> u64 {
            0
        }
    }

    #[derive(Debug, Default)]
    struct PhysicalTransport {
        observation_sequence: u64,
    }

    impl LeKiwiWireTransport for PhysicalTransport {
        fn exchange(
            &mut self,
            request: &HostWireFrame,
        ) -> std::result::Result<DeviceWireFrame, LeKiwiTransportError> {
            let payload = match &request.payload {
                HostWirePayload::Open {
                    task_id,
                    observation_width,
                    action_width,
                    ..
                } => DeviceWirePayload::Ready {
                    device_id: "rne.lekiwi_so101.physical.v1:test-unit".to_string(),
                    task_id: task_id.clone(),
                    observation_width: *observation_width,
                    action_width: *action_width,
                },
                HostWirePayload::PollObservation => {
                    self.observation_sequence += 1;
                    DeviceWirePayload::Observation {
                        sequence: self.observation_sequence,
                        values: vec![0.0; 9],
                    }
                }
                HostWirePayload::Actuate { frame } => DeviceWirePayload::ActuationAccepted {
                    action_sequence: frame.action_sequence,
                    safety_stop: frame.safety_stop,
                },
                HostWirePayload::Close => DeviceWirePayload::Closed,
            };
            Ok(DeviceWireFrame::new(
                request.session_id.clone(),
                request.sequence,
                payload,
            ))
        }
    }

    fn completed_session(mode: HardwareMode, action: Vec<f64>) -> LeKiwiReferenceSessionEvidence {
        let mut runner = LeKiwiReferenceSessionRunner::new(
            PhysicalTransport::default(),
            FixedClock,
            LeKiwiReferenceSessionConfig::new("semantic-session", mode, 1),
        )
        .expect("build runner");
        runner.open().expect("open runner");
        assert!(matches!(
            runner.sample(action).expect("sample"),
            LeKiwiReferenceSampleOutcome::Sample(_)
        ));
        runner.close().expect("close runner")
    }

    #[test]
    fn host_actuate_classifier_does_not_confuse_poll() {
        let poll = HardwareWireTraceEntry::Host {
            frame: HostWireFrame::new("session-1", 1, HostWirePayload::PollObservation),
        };
        assert!(!is_host_actuate(&poll));
    }

    #[test]
    fn physical_evidence_file_bound_is_finite() {
        assert_eq!(MAX_INDEXED_ARTIFACT_BYTES, 64 * 1024 * 1024);
    }

    #[test]
    fn staged_semantics_distinguish_shadow_reconnect_and_live() {
        let shadow = completed_session(HardwareMode::Shadow, vec![0.0; 3]);
        assert_eq!(observation_count(&shadow), 1);
        assert!(!shadow
            .session
            .wire_trace
            .entries
            .iter()
            .any(is_host_actuate));

        let reconnect = completed_session(HardwareMode::Hil, vec![0.0; 3]);
        validate_reconnect_rearm(&reconnect).expect("reconnect explicitly rearmed");

        let live = completed_session(HardwareMode::Live, vec![0.02, 0.0, 0.0]);
        validate_low_speed_motion(&live).expect("bounded live motion");
        require_outcome(
            &live,
            HardwareMode::Live,
            HardwareWireTraceOutcome::Completed,
            "fixture",
        )
        .expect("completed live stage");

        let too_fast = completed_session(HardwareMode::Live, vec![0.020_001, 0.0, 0.0]);
        assert!(validate_low_speed_motion(&too_fast).is_err());
    }

    #[test]
    fn documented_drafts_track_the_public_schema_and_stay_unattested() {
        let draft: LeKiwiPhysicalEvidenceManifest = serde_json::from_str(include_str!(
            "../../docs/examples/lekiwi-physical-evidence-draft-v1.json"
        ))
        .expect("parse documented draft");
        verify_declared_contracts(&draft).expect("exact artifact contracts");
        assert!(draft.content_sha256.is_empty());
        assert!(!draft.clean_host_checkout);
        assert!(!draft.power_isolation.tested);
        assert!(!draft.host_termination.safe_stop_observed);

        let power: PowerIsolationDiagnostic = serde_json::from_str(include_str!(
            "../../docs/examples/lekiwi-power-isolation-diagnostic-v1.json"
        ))
        .expect("parse power diagnostic draft");
        assert!(power.validate().is_err());
        let host: HostTerminationDiagnostic = serde_json::from_str(include_str!(
            "../../docs/examples/lekiwi-host-termination-diagnostic-v1.json"
        ))
        .expect("parse host diagnostic draft");
        assert!(host.validate().is_err());
    }

    #[test]
    fn draft_hasher_reads_only_a_bounded_relative_regular_file() {
        let temp = tempfile::tempdir().expect("tempdir");
        fs::write(temp.path().join("artifact.json"), b"evidence\n").expect("write fixture");
        let root = fs::canonicalize(temp.path()).expect("canonical root");
        let artifact = EvidenceFileRef::new("rne_fixture", 1, "artifact.json", "");
        let digest = digest_draft_artifact(&root, temp.path(), "fixture", &artifact)
            .expect("hash regular file");
        assert_eq!(
            digest,
            format!("sha256:{:x}", Sha256::digest(b"evidence\n"))
        );
        let escaping = EvidenceFileRef::new("rne_fixture", 1, "../artifact.json", "");
        assert!(digest_draft_artifact(&root, temp.path(), "fixture", &escaping).is_err());
    }
}
