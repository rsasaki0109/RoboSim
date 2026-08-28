//! Seal and verify full-content flagship LeKiwi shadow evidence.

use anyhow::{Context, Result};
use rne_ai::{
    flagship_mobile_lift_task_spec, flagship_mobile_lift_task_spec_v2,
    FlagshipMobileLiftControllerContract, TaskSpec, FLAGSHIP_MOBILE_LIFT_CONTROL_PERIOD_TICKS,
};
use rne_hardware_gateway::wire::{
    DeviceWirePayload, HardwareWireTraceEntry, HardwareWireTraceOutcome, HostWirePayload,
};
use rne_hardware_gateway::HardwareMode;
use rne_hardware_lekiwi::flagship_observation::FlagshipTimedObservation;
use rne_hardware_lekiwi::flagship_rate::FLAGSHIP_LEKIWI_WRITE_PERIOD_TICKS;
use rne_hardware_lekiwi::flagship_shadow::{
    FlagshipActionProjectionStream, FlagshipControllerContract,
    FlagshipLeKiwiArmCalibrationArtifact, FlagshipLeKiwiShadowExecutionClass,
    FlagshipLeKiwiShadowManifest, FlagshipObservationFusionStream,
    FlagshipObservationFusionStreamV2, FlagshipObservationSourceContract,
    FlagshipObservationSourceRole, FlagshipRateDecisionStream,
};
use rne_hardware_lekiwi::physical_evidence::EvidenceFileRef;
use rne_hardware_lekiwi::session::LeKiwiReferenceSessionEvidence;
use rne_hardware_lekiwi::{lekiwi_reference_profile_v1, LeKiwiReferenceProfile};
use serde::de::DeserializeOwned;
use sha2::{Digest, Sha256};
use std::fs;
use std::io::Write;
use std::path::{Path, PathBuf};

const MAX_INDEXED_ARTIFACT_BYTES: u64 = 64 * 1024 * 1024;

/// Runs `flagship-lekiwi-shadow seal DRAFT OUTPUT` or `verify MANIFEST`.
pub(crate) fn run(args: &mut impl Iterator<Item = String>) -> Result<()> {
    let command = args.next().ok_or_else(|| {
        anyhow::anyhow!(
            "flagship-lekiwi-shadow requires `seal DRAFT OUTPUT` or `verify MANIFEST`; see docs/REFERENCE_HARDWARE_LEKIWI.md"
        )
    })?;
    match command.as_str() {
        "seal" => {
            let draft = required_arg(args, "seal requires DRAFT and OUTPUT")?;
            let output = required_arg(args, "seal requires OUTPUT")?;
            no_more_args(args, "seal")?;
            seal(Path::new(&draft), Path::new(&output))
        }
        "verify" => {
            let manifest = required_arg(args, "verify requires MANIFEST")?;
            no_more_args(args, "verify")?;
            verify_manifest(Path::new(&manifest))
        }
        "--help" | "-h" => {
            println!(
                "flagship-lekiwi-shadow seal DRAFT OUTPUT\nflagship-lekiwi-shadow verify MANIFEST"
            );
            Ok(())
        }
        other => anyhow::bail!(
            "unknown flagship-lekiwi-shadow command `{other}`; expected `seal` or `verify`"
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
        "flagship-lekiwi-shadow {command} received too many arguments"
    );
    Ok(())
}

fn seal(draft_path: &Path, output_path: &Path) -> Result<()> {
    let bytes = read_regular_file(draft_path, "flagship LeKiwi shadow draft")?;
    let mut manifest: FlagshipLeKiwiShadowManifest =
        serde_json::from_slice(&bytes).context("shadow draft is not valid expected JSON")?;
    anyhow::ensure!(
        manifest.content_sha256.is_empty(),
        "shadow draft content_sha256 must be empty before sealing"
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
    manifest
        .seal()
        .map_err(|error| anyhow::anyhow!("could not seal shadow manifest: {error}"))?;
    manifest
        .validate()
        .map_err(|error| anyhow::anyhow!("invalid sealed shadow manifest: {error}"))?;
    verify_contents(&canonical_root, root, &manifest)?;
    write_new_json(output_path, &manifest)?;
    println!(
        "sealed flagship LeKiwi shadow manifest {} ({})",
        output_path.display(),
        manifest.content_sha256
    );
    Ok(())
}

/// Verifies one sealed full-content shadow directory.
pub(crate) fn verify_manifest(manifest_path: &Path) -> Result<()> {
    let bytes = read_regular_file(manifest_path, "flagship LeKiwi shadow manifest")?;
    let manifest: FlagshipLeKiwiShadowManifest =
        serde_json::from_slice(&bytes).context("shadow manifest is not valid expected JSON")?;
    manifest
        .validate()
        .map_err(|error| anyhow::anyhow!("invalid shadow manifest: {error}"))?;
    let root = manifest_path.parent().unwrap_or_else(|| Path::new("."));
    let canonical_root = fs::canonicalize(root)
        .with_context(|| format!("could not resolve evidence root `{}`", root.display()))?;
    verify_contents(&canonical_root, root, &manifest)?;
    println!(
        "verified flagship LeKiwi {:?} shadow {}: {} parent decisions, zero actuator writes, {}",
        manifest.execution_class, manifest.run_id, manifest.sample_count, manifest.content_sha256
    );
    Ok(())
}

fn verify_contents(
    canonical_root: &Path,
    root: &Path,
    manifest: &FlagshipLeKiwiShadowManifest,
) -> Result<()> {
    for (role, artifact) in manifest.artifacts.all() {
        checked_ref_path(canonical_root, root, role, artifact)?;
    }

    let task: TaskSpec = read_ref_json(
        canonical_root,
        root,
        "task_spec",
        &manifest.artifacts.task_spec,
    )?;
    task.validate()
        .map_err(|error| anyhow::anyhow!("invalid flagship TaskSpec: {error}"))?;
    let expected_task = if manifest.schema_version == 1 {
        flagship_mobile_lift_task_spec(FLAGSHIP_MOBILE_LIFT_CONTROL_PERIOD_TICKS)
    } else {
        flagship_mobile_lift_task_spec_v2(FLAGSHIP_MOBILE_LIFT_CONTROL_PERIOD_TICKS)
    };
    anyhow::ensure!(
        task == expected_task,
        "shadow TaskSpec differs from the canonical release flagship contract"
    );

    if manifest.schema_version == 1 {
        let controller: FlagshipControllerContract = read_ref_json(
            canonical_root,
            root,
            "controller_contract",
            &manifest.artifacts.controller_contract,
        )?;
        controller
            .validate()
            .map_err(|error| anyhow::anyhow!("invalid controller contract: {error}"))?;
    } else {
        let controller: FlagshipMobileLiftControllerContract = read_ref_json(
            canonical_root,
            root,
            "controller_contract",
            &manifest.artifacts.controller_contract,
        )?;
        controller
            .validate()
            .map_err(|error| anyhow::anyhow!("invalid v2 controller contract: {error}"))?;
    }

    let profile: LeKiwiReferenceProfile = read_ref_json(
        canonical_root,
        root,
        "reference_profile",
        &manifest.artifacts.reference_profile,
    )?;
    profile
        .validate()
        .map_err(|error| anyhow::anyhow!("invalid reference profile: {error}"))?;
    anyhow::ensure!(
        profile == lekiwi_reference_profile_v1(),
        "shadow profile differs from the built-in LeKiwi reference profile"
    );

    let arm: FlagshipLeKiwiArmCalibrationArtifact = read_ref_json(
        canonical_root,
        root,
        "arm_calibration",
        &manifest.artifacts.arm_calibration,
    )?;
    arm.validate()
        .map_err(|error| anyhow::anyhow!("invalid arm calibration: {error}"))?;

    let sources = read_source_contracts(canonical_root, root, manifest)?;
    let session: LeKiwiReferenceSessionEvidence = read_ref_json(
        canonical_root,
        root,
        "physical_shadow_session",
        &manifest.artifacts.physical_shadow_session,
    )?;
    validate_shadow_session(&session, manifest)?;

    let actions: FlagshipActionProjectionStream = read_ref_json(
        canonical_root,
        root,
        "action_projection_stream",
        &manifest.artifacts.action_projection_stream,
    )?;
    actions
        .validate()
        .map_err(|error| anyhow::anyhow!("invalid action projection stream: {error}"))?;
    let rates: FlagshipRateDecisionStream = read_ref_json(
        canonical_root,
        root,
        "rate_decision_stream",
        &manifest.artifacts.rate_decision_stream,
    )?;
    rates
        .validate()
        .map_err(|error| anyhow::anyhow!("invalid rate decision stream: {error}"))?;
    if manifest.schema_version == 1 {
        let observations: FlagshipObservationFusionStream = read_ref_json(
            canonical_root,
            root,
            "observation_fusion_stream",
            &manifest.artifacts.observation_fusion_stream,
        )?;
        observations
            .validate(&arm.calibration)
            .map_err(|error| anyhow::anyhow!("invalid observation fusion stream: {error}"))?;
        validate_stream_links_v1(
            manifest,
            &session,
            &sources,
            &actions,
            &rates,
            &observations,
        )
    } else {
        let observations: FlagshipObservationFusionStreamV2 = read_ref_json(
            canonical_root,
            root,
            "observation_fusion_stream",
            &manifest.artifacts.observation_fusion_stream,
        )?;
        observations
            .validate(&arm.calibration)
            .map_err(|error| anyhow::anyhow!("invalid v2 observation fusion stream: {error}"))?;
        validate_stream_links_v2(
            manifest,
            &session,
            &sources,
            &actions,
            &rates,
            &observations,
        )
    }
}

fn read_source_contracts(
    canonical_root: &Path,
    root: &Path,
    manifest: &FlagshipLeKiwiShadowManifest,
) -> Result<[FlagshipObservationSourceContract; 4]> {
    let declarations = [
        (
            "localization_source",
            &manifest.artifacts.localization_source,
            FlagshipObservationSourceRole::Localization,
        ),
        (
            "perception_source",
            &manifest.artifacts.perception_source,
            FlagshipObservationSourceRole::Perception,
        ),
        (
            "traffic_source",
            &manifest.artifacts.traffic_source,
            FlagshipObservationSourceRole::Traffic,
        ),
        (
            "task_state_source",
            &manifest.artifacts.task_state_source,
            FlagshipObservationSourceRole::TaskState,
        ),
    ];
    let mut contracts = Vec::with_capacity(declarations.len());
    for (role, artifact, expected_role) in declarations {
        let contract: FlagshipObservationSourceContract =
            read_ref_json(canonical_root, root, role, artifact)?;
        contract
            .validate()
            .map_err(|error| anyhow::anyhow!("invalid {role}: {error}"))?;
        anyhow::ensure!(
            contract.role == expected_role,
            "{role} file declares the wrong source role"
        );
        contracts.push(contract);
    }
    contracts
        .try_into()
        .map_err(|_| anyhow::anyhow!("source contract count drift"))
}

fn validate_shadow_session(
    session: &LeKiwiReferenceSessionEvidence,
    manifest: &FlagshipLeKiwiShadowManifest,
) -> Result<()> {
    session
        .validate()
        .map_err(|error| anyhow::anyhow!("invalid LeKiwi shadow session: {error}"))?;
    anyhow::ensure!(
        session.session.mode == HardwareMode::Shadow
            && session.session.wire_trace.outcome == HardwareWireTraceOutcome::Completed,
        "bound LeKiwi session must be a completed shadow session"
    );
    anyhow::ensure!(
        session.device_id == manifest.device_id,
        "shadow session device identity differs from manifest"
    );
    match manifest.execution_class {
        FlagshipLeKiwiShadowExecutionClass::Mock => anyhow::ensure!(
            session.device_id == rne_hardware_lekiwi::LEKIWI_MOCK_DEVICE_ID,
            "mock manifest cannot bind a physical session"
        ),
        FlagshipLeKiwiShadowExecutionClass::PhysicalShadow => anyhow::ensure!(
            session
                .device_id
                .starts_with(rne_hardware_lekiwi::LEKIWI_PHYSICAL_DEVICE_ID_PREFIX),
            "physical-shadow manifest cannot bind a mock session"
        ),
    }
    anyhow::ensure!(
        !session
            .session
            .wire_trace
            .entries
            .iter()
            .any(|entry| matches!(
                entry,
                HardwareWireTraceEntry::Host { frame }
                    if matches!(frame.payload, HostWirePayload::Actuate { .. })
            )),
        "shadow session must not contain an Actuate host frame"
    );
    Ok(())
}

fn validate_stream_links_v1(
    manifest: &FlagshipLeKiwiShadowManifest,
    session: &LeKiwiReferenceSessionEvidence,
    sources: &[FlagshipObservationSourceContract; 4],
    actions: &FlagshipActionProjectionStream,
    rates: &FlagshipRateDecisionStream,
    observations: &FlagshipObservationFusionStream,
) -> Result<()> {
    anyhow::ensure!(
        actions.records.len() == manifest.sample_count
            && rates.records.len() == manifest.sample_count
            && observations.records.len() == manifest.sample_count,
        "all three boundary streams must equal manifest sample_count"
    );
    let physical = session_observations(session);
    let required_physical = manifest.sample_count.div_ceil(2);
    anyhow::ensure!(
        physical.len() == required_physical,
        "LeKiwi session must contain exactly {required_physical} physical observations"
    );

    for index in 0..manifest.sample_count {
        let action = &actions.records[index];
        let rate = &rates.records[index];
        let observation = &observations.records[index];
        anyhow::ensure!(
            action.parent_action == rate.parent_action
                && action.projection == rate.decision.projection,
            "action and rate streams diverge at parent sequence {index}"
        );
        let expected_sequence = u64::try_from(index)?;
        anyhow::ensure!(
            rate.decision.parent_sequence == expected_sequence
                && observation.fusion.parent_sequence == expected_sequence,
            "boundary stream parent sequence mismatch at index {index}"
        );

        let physical_slot = index / 2;
        let (device_sequence, device_values) = &physical[physical_slot];
        let sample = &observation.inputs.physical;
        anyhow::ensure!(
            sample.source_id == manifest.device_id
                && sample.source_sequence == *device_sequence
                && sample.sample_tick
                    == u64::try_from(physical_slot)? * FLAGSHIP_LEKIWI_WRITE_PERIOD_TICKS
                && sample.max_age_ticks == FLAGSHIP_LEKIWI_WRITE_PERIOD_TICKS
                && sample.source_contract_sha256 == manifest.artifacts.reference_profile.sha256
                && sample.value.values == *device_values,
            "physical source/session mismatch at parent sequence {index}"
        );
        if index % 2 == 1 {
            anyhow::ensure!(
                observation.inputs.physical == observations.records[index - 1].inputs.physical,
                "odd parent sequence {index} must hold the preceding 30 Hz physical sample"
            );
        }

        validate_aux_source(
            "localization",
            &observation.inputs.localization,
            &sources[0],
            &manifest.artifacts.localization_source.sha256,
        )?;
        validate_aux_source(
            "perception",
            &observation.inputs.perception,
            &sources[1],
            &manifest.artifacts.perception_source.sha256,
        )?;
        validate_aux_source(
            "traffic",
            &observation.inputs.traffic,
            &sources[2],
            &manifest.artifacts.traffic_source.sha256,
        )?;
        validate_aux_source(
            "task_state",
            &observation.inputs.task_state,
            &sources[3],
            &manifest.artifacts.task_state_source.sha256,
        )?;
    }
    Ok(())
}

fn validate_stream_links_v2(
    manifest: &FlagshipLeKiwiShadowManifest,
    session: &LeKiwiReferenceSessionEvidence,
    sources: &[FlagshipObservationSourceContract; 4],
    actions: &FlagshipActionProjectionStream,
    rates: &FlagshipRateDecisionStream,
    observations: &FlagshipObservationFusionStreamV2,
) -> Result<()> {
    anyhow::ensure!(
        actions.records.len() == manifest.sample_count
            && rates.records.len() == manifest.sample_count
            && observations.records.len() == manifest.sample_count,
        "all three boundary streams must equal manifest sample_count"
    );
    let physical = session_observations(session);
    let required_physical = manifest.sample_count.div_ceil(2);
    anyhow::ensure!(
        physical.len() == required_physical,
        "LeKiwi session must contain exactly {required_physical} physical observations"
    );

    for index in 0..manifest.sample_count {
        let action = &actions.records[index];
        let rate = &rates.records[index];
        let observation = &observations.records[index];
        anyhow::ensure!(
            action.parent_action == rate.parent_action
                && action.parent_action == observation.controller_action
                && action.projection == rate.decision.projection,
            "controller, action, and rate streams diverge at parent sequence {index}"
        );
        let expected_sequence = u64::try_from(index)?;
        anyhow::ensure!(
            rate.decision.parent_sequence == expected_sequence
                && observation.fusion.parent_sequence == expected_sequence,
            "boundary stream parent sequence mismatch at index {index}"
        );

        let physical_slot = index / 2;
        let (device_sequence, device_values) = &physical[physical_slot];
        let sample = &observation.inputs.physical;
        anyhow::ensure!(
            sample.source_id == manifest.device_id
                && sample.source_sequence == *device_sequence
                && sample.sample_tick
                    == u64::try_from(physical_slot)? * FLAGSHIP_LEKIWI_WRITE_PERIOD_TICKS
                && sample.max_age_ticks == FLAGSHIP_LEKIWI_WRITE_PERIOD_TICKS
                && sample.source_contract_sha256 == manifest.artifacts.reference_profile.sha256
                && sample.value.values == *device_values,
            "physical source/session mismatch at parent sequence {index}"
        );
        if index % 2 == 1 {
            anyhow::ensure!(
                observation.inputs.physical == observations.records[index - 1].inputs.physical,
                "odd parent sequence {index} must hold the preceding 30 Hz physical sample"
            );
        }

        validate_aux_source(
            "localization",
            &observation.inputs.localization,
            &sources[0],
            &manifest.artifacts.localization_source.sha256,
        )?;
        validate_aux_source(
            "perception",
            &observation.inputs.perception,
            &sources[1],
            &manifest.artifacts.perception_source.sha256,
        )?;
        validate_aux_source(
            "traffic",
            &observation.inputs.traffic,
            &sources[2],
            &manifest.artifacts.traffic_source.sha256,
        )?;
        validate_aux_source(
            "task_state",
            &observation.inputs.task_state,
            &sources[3],
            &manifest.artifacts.task_state_source.sha256,
        )?;
    }
    Ok(())
}

fn validate_aux_source<T>(
    role: &str,
    sample: &FlagshipTimedObservation<T>,
    contract: &FlagshipObservationSourceContract,
    artifact_sha256: &str,
) -> Result<()> {
    anyhow::ensure!(
        sample.source_id == contract.source_id
            && sample.max_age_ticks == contract.max_age_ticks
            && sample.source_contract_sha256 == artifact_sha256,
        "{role} timed sample does not match its content-bound source contract"
    );
    Ok(())
}

fn session_observations(session: &LeKiwiReferenceSessionEvidence) -> Vec<(u64, Vec<f64>)> {
    session
        .session
        .wire_trace
        .entries
        .iter()
        .filter_map(|entry| match entry {
            HardwareWireTraceEntry::Device { frame } => match &frame.payload {
                DeviceWirePayload::Observation { sequence, values } => {
                    Some((*sequence, values.clone()))
                }
                _ => None,
            },
            HardwareWireTraceEntry::Host { .. } => None,
        })
        .collect()
}

fn read_ref_json<T: DeserializeOwned>(
    canonical_root: &Path,
    root: &Path,
    role: &'static str,
    artifact: &EvidenceFileRef,
) -> Result<T> {
    let path = checked_ref_path(canonical_root, root, role, artifact)?;
    let bytes = read_regular_file(&path, role)?;
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
    use rne_ai::{
        FlagshipMobileLiftControllerV2, FLAGSHIP_MOBILE_LIFT_CONTROLLER_CONTRACT_KIND,
        FLAGSHIP_MOBILE_LIFT_CONTROLLER_CONTRACT_SCHEMA_VERSION,
        FLAGSHIP_MOBILE_LIFT_CONTROLLER_ID_V2, FLAGSHIP_MOBILE_LIFT_TASK_ID_V2, TASK_SPEC_KIND,
        TASK_SPEC_SCHEMA_VERSION,
    };
    use rne_hardware_gateway::wire::{DeviceWireFrame, HostWireFrame};
    use rne_hardware_lekiwi::flagship_observation::{
        FlagshipArmChannelCalibration, FlagshipLeKiwiArmCalibration,
        FlagshipLeKiwiObservationFuserV2, FlagshipLeKiwiObservationInputsV2,
        FlagshipLeKiwiPhysicalObservation, FlagshipLocalizationObservationV2,
        FlagshipPerceptionObservation, FlagshipTaskStateObservationV2, FlagshipTrafficObservation,
    };
    use rne_hardware_lekiwi::flagship_projection::project_flagship_action_to_lekiwi_v2;
    use rne_hardware_lekiwi::flagship_rate::FlagshipLeKiwiRateSchedulerV2;
    use rne_hardware_lekiwi::flagship_shadow::{
        FlagshipActionProjectionRecord, FlagshipLeKiwiShadowArtifacts,
        FlagshipObservationFusionRecordV2, FlagshipRateDecisionRecord,
        FLAGSHIP_ACTION_PROJECTION_STREAM_CURRENT_SCHEMA_VERSION,
        FLAGSHIP_ACTION_PROJECTION_STREAM_KIND, FLAGSHIP_LEKIWI_ARM_CALIBRATION_KIND,
        FLAGSHIP_LEKIWI_ARM_CALIBRATION_SCHEMA_VERSION,
        FLAGSHIP_LEKIWI_SHADOW_MANIFEST_CURRENT_SCHEMA_VERSION,
        FLAGSHIP_LEKIWI_SHADOW_MANIFEST_KIND,
        FLAGSHIP_OBSERVATION_FUSION_STREAM_CURRENT_SCHEMA_VERSION,
        FLAGSHIP_OBSERVATION_FUSION_STREAM_KIND, FLAGSHIP_OBSERVATION_SOURCE_CONTRACT_KIND,
        FLAGSHIP_OBSERVATION_SOURCE_CONTRACT_SCHEMA_VERSION,
        FLAGSHIP_RATE_DECISION_STREAM_CURRENT_SCHEMA_VERSION, FLAGSHIP_RATE_DECISION_STREAM_KIND,
    };
    use rne_hardware_lekiwi::session::{
        LeKiwiMonotonicClock, LeKiwiReferenceSampleOutcome, LeKiwiReferenceSessionConfig,
        LeKiwiReferenceSessionRunner, LeKiwiTransportError, LeKiwiWireTransport,
        LEKIWI_REFERENCE_SESSION_KIND, LEKIWI_REFERENCE_SESSION_SCHEMA_VERSION,
    };
    use rne_hardware_lekiwi::{
        LEKIWI_MOCK_DEVICE_ID, LEKIWI_REFERENCE_PROFILE_KIND,
        LEKIWI_REFERENCE_PROFILE_SCHEMA_VERSION,
    };

    #[derive(Debug, Default)]
    struct FixedClock;

    impl LeKiwiMonotonicClock for FixedClock {
        fn now_ms(&mut self) -> u64 {
            0
        }
    }

    #[derive(Debug, Default)]
    struct MockShadowTransport {
        observation_sequence: u64,
    }

    impl LeKiwiWireTransport for MockShadowTransport {
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
                    device_id: LEKIWI_MOCK_DEVICE_ID.to_string(),
                    task_id: task_id.clone(),
                    observation_width: *observation_width,
                    action_width: *action_width,
                },
                HostWirePayload::PollObservation => {
                    self.observation_sequence += 1;
                    DeviceWirePayload::Observation {
                        sequence: self.observation_sequence,
                        values: physical_values(self.observation_sequence),
                    }
                }
                HostWirePayload::Actuate { .. } => panic!("shadow must not actuate"),
                HostWirePayload::Close => DeviceWirePayload::Closed,
            };
            Ok(DeviceWireFrame::new(
                request.session_id.clone(),
                request.sequence,
                payload,
            ))
        }
    }

    fn physical_values(sequence: u64) -> Vec<f64> {
        let delta = sequence as f64 * 0.01;
        vec![
            0.1 + delta,
            0.2 + delta,
            0.3 + delta,
            0.4 + delta,
            0.5 + delta,
            25.0,
            0.0,
            0.0,
            0.0,
        ]
    }

    fn write_json(path: &Path, value: &impl serde::Serialize) {
        let mut bytes = serde_json::to_vec_pretty(value).unwrap();
        bytes.push(b'\n');
        fs::write(path, bytes).unwrap();
    }

    fn digest_file(path: &Path) -> String {
        format!("sha256:{:x}", Sha256::digest(fs::read(path).unwrap()))
    }

    fn source_contract(
        role: FlagshipObservationSourceRole,
        source_id: &str,
    ) -> FlagshipObservationSourceContract {
        FlagshipObservationSourceContract {
            kind: FLAGSHIP_OBSERVATION_SOURCE_CONTRACT_KIND.to_string(),
            schema_version: FLAGSHIP_OBSERVATION_SOURCE_CONTRACT_SCHEMA_VERSION,
            role,
            source_id: source_id.to_string(),
            clock_domain: "rne_sim_tick_ns".to_string(),
            max_age_ticks: FLAGSHIP_MOBILE_LIFT_CONTROL_PERIOD_TICKS,
            implementation_id: format!("{source_id}-implementation-v1"),
        }
    }

    fn timed<T>(
        contract: &FlagshipObservationSourceContract,
        contract_sha256: &str,
        sequence: u64,
        value: T,
    ) -> FlagshipTimedObservation<T> {
        FlagshipTimedObservation {
            source_id: contract.source_id.clone(),
            source_sequence: sequence,
            sample_tick: sequence * FLAGSHIP_MOBILE_LIFT_CONTROL_PERIOD_TICKS,
            max_age_ticks: contract.max_age_ticks,
            source_contract_sha256: contract_sha256.to_string(),
            value,
        }
    }

    fn reference(kind: &str, schema_version: u32, path: &str) -> EvidenceFileRef {
        EvidenceFileRef::new(kind, schema_version, path, "")
    }

    #[test]
    fn bounded_file_limit_and_help_contract_are_stable() {
        assert_eq!(MAX_INDEXED_ARTIFACT_BYTES, 64 * 1024 * 1024);
        let mut args = ["--help".to_string()].into_iter();
        run(&mut args).unwrap();
    }

    #[test]
    fn draft_hasher_rejects_escape_and_hashes_regular_file() {
        let temp = tempfile::tempdir().unwrap();
        fs::write(temp.path().join("artifact.json"), b"evidence\n").unwrap();
        let root = fs::canonicalize(temp.path()).unwrap();
        let artifact = EvidenceFileRef::new("rne_fixture", 1, "artifact.json", "");
        assert_eq!(
            digest_draft_artifact(&root, temp.path(), "fixture", &artifact).unwrap(),
            format!("sha256:{:x}", Sha256::digest(b"evidence\n"))
        );
        let escaping = EvidenceFileRef::new("rne_fixture", 1, "../artifact.json", "");
        assert!(digest_draft_artifact(&root, temp.path(), "fixture", &escaping).is_err());
    }

    #[test]
    fn complete_v2_mock_shadow_seals_verifies_and_rejects_tampering() {
        let temp = tempfile::tempdir().unwrap();
        let root = temp.path();

        write_json(
            &root.join("task.json"),
            &flagship_mobile_lift_task_spec_v2(FLAGSHIP_MOBILE_LIFT_CONTROL_PERIOD_TICKS),
        );
        write_json(
            &root.join("controller.json"),
            &FlagshipMobileLiftControllerContract::built_in(),
        );
        write_json(&root.join("profile.json"), &lekiwi_reference_profile_v1());
        let profile_sha256 = digest_file(&root.join("profile.json"));

        let calibration = FlagshipLeKiwiArmCalibration {
            calibration_id: "mock-shadow-calibration-v1".to_string(),
            channels: [
                FlagshipArmChannelCalibration {
                    physical_element: 0,
                    scale: 1.0,
                    offset_rad: 0.0,
                },
                FlagshipArmChannelCalibration {
                    physical_element: 1,
                    scale: 1.0,
                    offset_rad: 0.0,
                },
                FlagshipArmChannelCalibration {
                    physical_element: 2,
                    scale: 1.0,
                    offset_rad: 0.0,
                },
            ],
        };
        write_json(
            &root.join("arm-calibration.json"),
            &FlagshipLeKiwiArmCalibrationArtifact {
                kind: FLAGSHIP_LEKIWI_ARM_CALIBRATION_KIND.to_string(),
                schema_version: FLAGSHIP_LEKIWI_ARM_CALIBRATION_SCHEMA_VERSION,
                calibration: calibration.clone(),
            },
        );

        let localization = source_contract(
            FlagshipObservationSourceRole::Localization,
            "mock-localization",
        );
        let perception =
            source_contract(FlagshipObservationSourceRole::Perception, "mock-perception");
        let traffic = source_contract(FlagshipObservationSourceRole::Traffic, "mock-traffic");
        let task_state =
            source_contract(FlagshipObservationSourceRole::TaskState, "mock-task-state");
        for (path, contract) in [
            ("localization.json", &localization),
            ("perception.json", &perception),
            ("traffic.json", &traffic),
            ("task-state.json", &task_state),
        ] {
            write_json(&root.join(path), contract);
        }
        let localization_sha256 = digest_file(&root.join("localization.json"));
        let perception_sha256 = digest_file(&root.join("perception.json"));
        let traffic_sha256 = digest_file(&root.join("traffic.json"));
        let task_state_sha256 = digest_file(&root.join("task-state.json"));

        let mut runner = LeKiwiReferenceSessionRunner::new(
            MockShadowTransport::default(),
            FixedClock,
            LeKiwiReferenceSessionConfig::new("mock-shadow-session", HardwareMode::Shadow, 2),
        )
        .unwrap();
        runner.open().unwrap();
        for _ in 0..2 {
            assert!(matches!(
                runner.sample(vec![0.0; 3]).unwrap(),
                LeKiwiReferenceSampleOutcome::Sample(_)
            ));
        }
        let session = runner.close().unwrap();
        write_json(&root.join("session.json"), &session);

        let mut scheduler = FlagshipLeKiwiRateSchedulerV2::new();
        let mut fuser = FlagshipLeKiwiObservationFuserV2::new();
        let mut controller = FlagshipMobileLiftControllerV2::new();
        let mut action_records = Vec::new();
        let mut rate_records = Vec::new();
        let mut observation_records = Vec::new();
        for index in 0..3_usize {
            let sequence = index as u64;
            let physical_slot = index / 2;
            let physical_sequence = physical_slot as u64 + 1;
            let inputs = FlagshipLeKiwiObservationInputsV2 {
                physical: FlagshipTimedObservation {
                    source_id: LEKIWI_MOCK_DEVICE_ID.to_string(),
                    source_sequence: physical_sequence,
                    sample_tick: physical_slot as u64 * FLAGSHIP_LEKIWI_WRITE_PERIOD_TICKS,
                    max_age_ticks: FLAGSHIP_LEKIWI_WRITE_PERIOD_TICKS,
                    source_contract_sha256: profile_sha256.clone(),
                    value: FlagshipLeKiwiPhysicalObservation {
                        values: physical_values(physical_sequence),
                    },
                },
                localization: timed(
                    &localization,
                    &localization_sha256,
                    sequence,
                    FlagshipLocalizationObservationV2 {
                        base_position_m: [sequence as f64 * 0.01, 0.0, 0.0],
                        base_yaw_rad: 0.0,
                    },
                ),
                perception: timed(
                    &perception,
                    &perception_sha256,
                    sequence,
                    FlagshipPerceptionObservation {
                        payload_position_m: [0.5, 0.2, -0.4],
                        wrist_camera_pixel_count: 640 * 480,
                        wrist_depth_min_m: 0.4,
                        grasped: false,
                    },
                ),
                traffic: timed(
                    &traffic,
                    &traffic_sha256,
                    sequence,
                    FlagshipTrafficObservation {
                        actor_position_m: [1.0, 0.0, 0.0],
                        signal_green: true,
                        clear: true,
                    },
                ),
                task_state: timed(
                    &task_state,
                    &task_state_sha256,
                    sequence,
                    FlagshipTaskStateObservationV2 {
                        lift_position_m: 0.0,
                        gripper_position_m: 0.02,
                        place_target_position_m: [0.8, 0.02, 0.0],
                        policy_phase: controller.expected_policy_phase(),
                    },
                ),
            };
            let fusion = fuser.fuse(sequence, &inputs, &calibration).unwrap();
            let parent_action = controller.next_action(&fusion.observation_values).unwrap();
            action_records.push(FlagshipActionProjectionRecord {
                parent_action: parent_action.clone(),
                projection: project_flagship_action_to_lekiwi_v2(&parent_action).unwrap(),
            });
            rate_records.push(FlagshipRateDecisionRecord {
                parent_action: parent_action.clone(),
                decision: scheduler.ingest(sequence, &parent_action).unwrap(),
            });
            observation_records.push(FlagshipObservationFusionRecordV2 {
                fusion,
                inputs,
                controller_action: parent_action,
            });
        }
        write_json(
            &root.join("actions.json"),
            &FlagshipActionProjectionStream {
                kind: FLAGSHIP_ACTION_PROJECTION_STREAM_KIND.to_string(),
                schema_version: FLAGSHIP_ACTION_PROJECTION_STREAM_CURRENT_SCHEMA_VERSION,
                task_id: FLAGSHIP_MOBILE_LIFT_TASK_ID_V2.to_string(),
                controller_id: FLAGSHIP_MOBILE_LIFT_CONTROLLER_ID_V2.to_string(),
                records: action_records,
            },
        );
        write_json(
            &root.join("rates.json"),
            &FlagshipRateDecisionStream {
                kind: FLAGSHIP_RATE_DECISION_STREAM_KIND.to_string(),
                schema_version: FLAGSHIP_RATE_DECISION_STREAM_CURRENT_SCHEMA_VERSION,
                task_id: FLAGSHIP_MOBILE_LIFT_TASK_ID_V2.to_string(),
                controller_id: FLAGSHIP_MOBILE_LIFT_CONTROLLER_ID_V2.to_string(),
                records: rate_records,
            },
        );
        let observation_stream = FlagshipObservationFusionStreamV2 {
            kind: FLAGSHIP_OBSERVATION_FUSION_STREAM_KIND.to_string(),
            schema_version: FLAGSHIP_OBSERVATION_FUSION_STREAM_CURRENT_SCHEMA_VERSION,
            task_id: FLAGSHIP_MOBILE_LIFT_TASK_ID_V2.to_string(),
            controller_id: FLAGSHIP_MOBILE_LIFT_CONTROLLER_ID_V2.to_string(),
            records: observation_records,
        };
        write_json(&root.join("observations.json"), &observation_stream);
        let round_tripped: FlagshipObservationFusionStreamV2 =
            serde_json::from_slice(&fs::read(root.join("observations.json")).unwrap()).unwrap();
        assert_eq!(
            observation_stream.records[0]
                .fusion
                .observation_values
                .iter()
                .map(|value| value.to_bits())
                .collect::<Vec<_>>(),
            round_tripped.records[0]
                .fusion
                .observation_values
                .iter()
                .map(|value| value.to_bits())
                .collect::<Vec<_>>()
        );
        let replayed = FlagshipLeKiwiObservationFuserV2::new()
            .fuse(0, &round_tripped.records[0].inputs, &calibration)
            .unwrap();
        assert_eq!(replayed, round_tripped.records[0].fusion);

        let draft = FlagshipLeKiwiShadowManifest {
            kind: FLAGSHIP_LEKIWI_SHADOW_MANIFEST_KIND.to_string(),
            schema_version: FLAGSHIP_LEKIWI_SHADOW_MANIFEST_CURRENT_SCHEMA_VERSION,
            run_id: "mock-shadow-full-content-001".to_string(),
            rne_commit: "a".repeat(40),
            execution_class: FlagshipLeKiwiShadowExecutionClass::Mock,
            device_id: LEKIWI_MOCK_DEVICE_ID.to_string(),
            task_id: FLAGSHIP_MOBILE_LIFT_TASK_ID_V2.to_string(),
            controller_id: FLAGSHIP_MOBILE_LIFT_CONTROLLER_ID_V2.to_string(),
            sample_count: 3,
            actuator_writes_emitted: false,
            artifacts: FlagshipLeKiwiShadowArtifacts {
                task_spec: reference(TASK_SPEC_KIND, TASK_SPEC_SCHEMA_VERSION, "task.json"),
                controller_contract: reference(
                    FLAGSHIP_MOBILE_LIFT_CONTROLLER_CONTRACT_KIND,
                    FLAGSHIP_MOBILE_LIFT_CONTROLLER_CONTRACT_SCHEMA_VERSION,
                    "controller.json",
                ),
                reference_profile: reference(
                    LEKIWI_REFERENCE_PROFILE_KIND,
                    LEKIWI_REFERENCE_PROFILE_SCHEMA_VERSION,
                    "profile.json",
                ),
                arm_calibration: reference(
                    FLAGSHIP_LEKIWI_ARM_CALIBRATION_KIND,
                    FLAGSHIP_LEKIWI_ARM_CALIBRATION_SCHEMA_VERSION,
                    "arm-calibration.json",
                ),
                localization_source: reference(
                    FLAGSHIP_OBSERVATION_SOURCE_CONTRACT_KIND,
                    FLAGSHIP_OBSERVATION_SOURCE_CONTRACT_SCHEMA_VERSION,
                    "localization.json",
                ),
                perception_source: reference(
                    FLAGSHIP_OBSERVATION_SOURCE_CONTRACT_KIND,
                    FLAGSHIP_OBSERVATION_SOURCE_CONTRACT_SCHEMA_VERSION,
                    "perception.json",
                ),
                traffic_source: reference(
                    FLAGSHIP_OBSERVATION_SOURCE_CONTRACT_KIND,
                    FLAGSHIP_OBSERVATION_SOURCE_CONTRACT_SCHEMA_VERSION,
                    "traffic.json",
                ),
                task_state_source: reference(
                    FLAGSHIP_OBSERVATION_SOURCE_CONTRACT_KIND,
                    FLAGSHIP_OBSERVATION_SOURCE_CONTRACT_SCHEMA_VERSION,
                    "task-state.json",
                ),
                physical_shadow_session: reference(
                    LEKIWI_REFERENCE_SESSION_KIND,
                    LEKIWI_REFERENCE_SESSION_SCHEMA_VERSION,
                    "session.json",
                ),
                action_projection_stream: reference(
                    FLAGSHIP_ACTION_PROJECTION_STREAM_KIND,
                    FLAGSHIP_ACTION_PROJECTION_STREAM_CURRENT_SCHEMA_VERSION,
                    "actions.json",
                ),
                rate_decision_stream: reference(
                    FLAGSHIP_RATE_DECISION_STREAM_KIND,
                    FLAGSHIP_RATE_DECISION_STREAM_CURRENT_SCHEMA_VERSION,
                    "rates.json",
                ),
                observation_fusion_stream: reference(
                    FLAGSHIP_OBSERVATION_FUSION_STREAM_KIND,
                    FLAGSHIP_OBSERVATION_FUSION_STREAM_CURRENT_SCHEMA_VERSION,
                    "observations.json",
                ),
            },
            content_sha256: String::new(),
        };
        let draft_path = root.join("draft.json");
        let manifest_path = root.join("manifest.json");
        write_json(&draft_path, &draft);
        seal(&draft_path, &manifest_path).unwrap();
        verify_manifest(&manifest_path).unwrap();

        let mut divergent_actions: FlagshipActionProjectionStream =
            serde_json::from_slice(&fs::read(root.join("actions.json")).unwrap()).unwrap();
        divergent_actions.records[0].parent_action[2] += 0.01;
        divergent_actions.records[0].projection =
            project_flagship_action_to_lekiwi_v2(&divergent_actions.records[0].parent_action)
                .unwrap();
        let mut divergent_scheduler = FlagshipLeKiwiRateSchedulerV2::new();
        let divergent_rates = FlagshipRateDecisionStream {
            kind: FLAGSHIP_RATE_DECISION_STREAM_KIND.to_string(),
            schema_version: FLAGSHIP_RATE_DECISION_STREAM_CURRENT_SCHEMA_VERSION,
            task_id: FLAGSHIP_MOBILE_LIFT_TASK_ID_V2.to_string(),
            controller_id: FLAGSHIP_MOBILE_LIFT_CONTROLLER_ID_V2.to_string(),
            records: divergent_actions
                .records
                .iter()
                .enumerate()
                .map(|(index, record)| FlagshipRateDecisionRecord {
                    parent_action: record.parent_action.clone(),
                    decision: divergent_scheduler
                        .ingest(index as u64, &record.parent_action)
                        .unwrap(),
                })
                .collect(),
        };
        write_json(&root.join("actions.json"), &divergent_actions);
        write_json(&root.join("rates.json"), &divergent_rates);
        let divergent_draft_path = root.join("cross-link-draft.json");
        write_json(&divergent_draft_path, &draft);
        assert!(seal(
            &divergent_draft_path,
            &root.join("cross-link-manifest.json")
        )
        .is_err());

        fs::write(root.join("rates.json"), b"tampered\n").unwrap();
        assert!(verify_manifest(&manifest_path).is_err());
    }
}
