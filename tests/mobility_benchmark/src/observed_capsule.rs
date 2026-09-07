//! Common failure-capsule envelopes for verified terminal-voltage replay.

use crate::observed::{
    decode_sensor_observed_trace, replay_sensor_observed_trace, SensorObservedTrace,
    SENSOR_OBSERVED_TRACE_KIND, SENSOR_OBSERVED_TRACE_SCHEMA_VERSION,
};
use anyhow::{ensure, Context, Result};
use rne_core::{DeterminismContract, DeterminismScope};
use rne_log::{
    ArtifactRef, BackendMetadata, BuildMetadata, FailureCapsule, FailureMetadata, RunMetadata,
};
use rne_physics::{PhysicsBackend, PhysicsBackendManifest};
use sha2::{Digest, Sha256};

/// Conventional relative path of the voltage replay in a transported capsule.
pub const SENSOR_VOLTAGE_REPLAY_PATH: &str = "voltage-replay.json";
const CONTRACT: &str = "mobility.sensor_voltage_replay";
const MAX_CAPSULE_BYTES: usize = 64 * 1024;

/// Returns build-time provenance for CLI capsule production and verification.
/// Dirty/unversioned builds are refused rather than attributed to a clean commit.
/// These fields identify the verifying build; they are not a signed attestation.
pub fn compiled_capsule_build() -> Result<BuildMetadata> {
    ensure!(env!("RNE_MOBILITY_BUILD_CLEAN") == "true", "capsule CLI requires a binary built from a clean versioned checkout; commit changes and rebuild");
    ensure!(
        env!("RNE_MOBILITY_BUILD_REVISION") != "unavailable"
            && env!("RNE_MOBILITY_BUILD_COMPILER") != "unavailable",
        "build provenance unavailable"
    );
    Ok(BuildMetadata::new(
        env!("CARGO_PKG_VERSION"),
        env!("RNE_MOBILITY_BUILD_REVISION"),
        env!("RNE_MOBILITY_BUILD_PROFILE"),
        env!("RNE_MOBILITY_BUILD_TARGET"),
        env!("RNE_MOBILITY_BUILD_COMPILER"),
        env!("RNE_MOBILITY_BUILD_LOCK"),
    ))
}

/// A common capsule plus the exact bytes of its referenced replay artifact.
///
/// Build metadata is producer-supplied provenance, not independently attested by
/// this type. Consumers must supply their expected build to [`Self::verify_replay`].
#[derive(Clone, Debug, PartialEq)]
pub struct SensorObservedFailureBundle {
    /// Standard backend-neutral failure envelope.
    pub capsule: FailureCapsule,
    /// Exact UTF-8 JSON bytes referenced by the capsule's SHA-256.
    pub replay_bytes: Vec<u8>,
}

impl SensorObservedFailureBundle {
    /// Writes canonical files into a new directory without replacing existing data.
    /// A write failure may leave a partial directory; no existing paths are deleted.
    /// This checks metadata only; use [`Self::verify_replay`] for execution proof.
    pub fn write_new(&self, directory: &std::path::Path) -> Result<()> {
        use std::io::Write;
        self.validate_metadata()?;
        let mut capsule_bytes = serde_json::to_vec_pretty(&self.capsule)?;
        capsule_bytes.push(b'\n');
        ensure!(
            capsule_bytes.len() <= MAX_CAPSULE_BYTES,
            "capsule exceeds byte limit"
        );
        std::fs::create_dir(directory)
            .with_context(|| format!("create new capsule {}", directory.display()))?;
        for (name, bytes) in [
            ("capsule.json", capsule_bytes.as_slice()),
            (SENSOR_VOLTAGE_REPLAY_PATH, self.replay_bytes.as_slice()),
        ] {
            std::fs::OpenOptions::new()
                .write(true)
                .create_new(true)
                .open(directory.join(name))?
                .write_all(bytes)?;
        }
        Ok(())
    }

    /// Reads bounded canonical files and verifies the capsule-to-replay binding.
    /// Filesystem verification alone does not execute the recorded voltages.
    pub fn read(directory: &std::path::Path) -> Result<Self> {
        use std::io::Read;
        let mut capsule_bytes = Vec::new();
        std::fs::File::open(directory.join("capsule.json"))?
            .take(MAX_CAPSULE_BYTES as u64 + 1)
            .read_to_end(&mut capsule_bytes)?;
        ensure!(
            capsule_bytes.len() <= MAX_CAPSULE_BYTES,
            "capsule exceeds byte limit"
        );
        let capsule = serde_json::from_slice(&capsule_bytes).context("decode common capsule")?;
        let mut replay_bytes = Vec::new();
        std::fs::File::open(directory.join(SENSOR_VOLTAGE_REPLAY_PATH))?
            .take(crate::observed::MAX_SENSOR_OBSERVED_TRACE_BYTES as u64 + 1)
            .read_to_end(&mut replay_bytes)?;
        let bundle = Self {
            capsule,
            replay_bytes,
        };
        bundle.validate_metadata()?;
        Ok(bundle)
    }

    /// Creates a bundle only after a failed task's recorded voltages replay exactly.
    pub fn create<B: PhysicsBackend>(
        backend: B,
        manifest: PhysicsBackendManifest,
        source: &SensorObservedTrace,
        build: BuildMetadata,
    ) -> Result<Self> {
        source.validate()?;
        ensure!(
            !source.passed,
            "successful task cannot form a failure capsule"
        );
        replay_sensor_observed_trace(backend, manifest, source)?;
        let mut replay_bytes = serde_json::to_vec_pretty(source)?;
        replay_bytes.push(b'\n');
        let capsule = envelope(source, build, &replay_bytes)?;
        let bundle = Self {
            capsule,
            replay_bytes,
        };
        bundle.validate_metadata()?;
        Ok(bundle)
    }

    /// Checks bounded replay decoding, byte hash, failure timing, metrics and identity.
    /// This is metadata verification only, not execution or build attestation.
    pub fn validate_metadata(&self) -> Result<()> {
        self.capsule.validate()?;
        let source = decode_sensor_observed_trace(&self.replay_bytes)?;
        ensure!(
            self.capsule == envelope(&source, self.capsule.build.clone(), &self.replay_bytes)?,
            "failure capsule does not match its replay evidence"
        );
        Ok(())
    }

    /// Verifies metadata and expected build, then re-executes the recorded voltage stream.
    /// Successful verification means the failed task reproduced, not that it succeeded.
    pub fn verify_replay<B: PhysicsBackend>(
        &self,
        backend: B,
        manifest: PhysicsBackendManifest,
        expected_build: &BuildMetadata,
    ) -> Result<()> {
        self.validate_metadata()?;
        ensure!(
            self.capsule.build == *expected_build,
            "capsule build identity mismatch"
        );
        let source = decode_sensor_observed_trace(&self.replay_bytes)?;
        replay_sensor_observed_trace(backend, manifest, &source)?;
        Ok(())
    }
}

fn envelope(
    source: &SensorObservedTrace,
    build: BuildMetadata,
    bytes: &[u8],
) -> Result<FailureCapsule> {
    ensure!(
        !source.passed,
        "successful task cannot form a failure capsule"
    );
    let failed = source
        .metrics
        .iter()
        .find(|metric| !metric.passed)
        .context("failed metric missing")?;
    let failure = FailureMetadata::new(
        &failed.id,
        CONTRACT,
        format!(
            "final acceptance failed: {} ({} {} outside [{}, {}])",
            failed.id, failed.value, failed.unit, failed.minimum, failed.maximum
        ),
        source.steps.checked_sub(1).context("empty replay")?,
        source
            .steps
            .checked_mul(source.fixed_delta_ticks)
            .context("failure timestamp overflow")?,
        source.privileged_final_physics_state_hash_v2,
    );
    // Acceptance is evaluated at the end, not at the last sensor capture.
    // The common capsule uses zero-based step indices, while the trace counts
    // completed integration steps. Its timestamp remains the end of that step.
    let run = RunMetadata::new(
        format!("{}-{}", source.task_spec.task_id, source.content_digest),
        &source.task_spec.task_id,
        source.seed,
        source.fixed_delta_ticks,
        source.steps,
        source.samples.len() as u64,
    );
    let determinism = DeterminismContract::exact(
        CONTRACT,
        DeterminismScope::new(
            "sensor_voltage_replay",
            [
                "voltage_decisions",
                "sensor_observed_trace",
                "final.physics_state_v2",
            ],
            0,
            source.steps,
        )?,
    )?;
    let digest = format!("{:x}", Sha256::digest(bytes));
    Ok(FailureCapsule::new(
        failure,
        run,
        build,
        BackendMetadata::new(
            &source.backend.backend_id,
            format!(
                "adapter:{} engine:{}:{}",
                source.backend.adapter_version,
                source.backend.engine_id,
                source.backend.engine_version
            ),
        ),
        determinism,
        vec![ArtifactRef::new(
            "replay",
            SENSOR_OBSERVED_TRACE_KIND,
            SENSOR_OBSERVED_TRACE_SCHEMA_VERSION,
            SENSOR_VOLTAGE_REPLAY_PATH,
            digest,
        )?],
    )?)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::observed::{run_randomized_mobility_sensor_trace, run_sensor_observed_trace};
    use rne_physics_rapier::RapierBackend;

    fn fixture_build() -> BuildMetadata {
        BuildMetadata::new(
            "test",
            "synthetic-test-build",
            "test",
            "test-target",
            "test-compiler",
            "0".repeat(64),
        )
    }

    #[test]
    fn common_capsule_binds_failed_replay_and_rejects_tampering() {
        let source = run_randomized_mobility_sensor_trace(
            RapierBackend::new(),
            RapierBackend::manifest(),
            42,
        )
        .unwrap();
        let bundle = SensorObservedFailureBundle::create(
            RapierBackend::new(),
            RapierBackend::manifest(),
            &source,
            fixture_build(),
        )
        .unwrap();
        bundle
            .verify_replay(
                RapierBackend::new(),
                RapierBackend::manifest(),
                &fixture_build(),
            )
            .unwrap();
        let temporary = tempfile::tempdir().unwrap();
        let directory = temporary.path().join("capsule");
        bundle.write_new(&directory).unwrap();
        assert_eq!(
            SensorObservedFailureBundle::read(&directory).unwrap(),
            bundle
        );
        assert!(bundle.write_new(&directory).is_err());
        assert_eq!(
            SensorObservedFailureBundle::read(&directory).unwrap(),
            bundle
        );
        std::fs::write(
            directory.join("capsule.json"),
            vec![b' '; MAX_CAPSULE_BYTES + 1],
        )
        .unwrap();
        assert!(SensorObservedFailureBundle::read(&directory).is_err());
        assert_eq!(bundle.capsule.kind, "rne_failure_capsule");
        assert_eq!(bundle.capsule.failure.step, source.steps - 1);
        assert_eq!(
            bundle.capsule.failure.state_digest,
            source.privileged_final_physics_state_hash_v2
        );
        let mut bad = bundle.clone();
        bad.capsule.failure.state_digest ^= 1;
        assert!(bad.validate_metadata().is_err());
        let mut bad = bundle.clone();
        bad.replay_bytes.push(b' ');
        assert!(bad.validate_metadata().is_err());
        let mut wrong_build = fixture_build();
        wrong_build.git_commit = "different-test-build".into();
        assert!(bundle
            .verify_replay(
                RapierBackend::new(),
                RapierBackend::manifest(),
                &wrong_build
            )
            .is_err());
    }

    #[test]
    fn successful_task_cannot_be_packaged_as_a_failure() {
        let source =
            run_sensor_observed_trace(RapierBackend::new(), RapierBackend::manifest()).unwrap();
        assert!(source.passed);
        assert!(SensorObservedFailureBundle::create(
            RapierBackend::new(),
            RapierBackend::manifest(),
            &source,
            fixture_build()
        )
        .is_err());
    }
}
