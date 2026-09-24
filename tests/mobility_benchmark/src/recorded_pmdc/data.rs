//! Content-bound decoder and SI observation reconstruction for retained PMDC training data.

use super::{
    final_protocol::{
        pmdc_final_evaluation_protocol, PMDC_FINAL_ARTIFACT_BYTES, PMDC_FINAL_ARTIFACT_SHA256,
        PMDC_FINAL_RECORDS_SHA256,
    },
    pmdc_identification_protocol, PMDC_DEVELOPMENT_RECORDS_SHA256, PMDC_SOURCE_SHA256,
    PMDC_TRAINING_RECORDS_SHA256,
};
use anyhow::{ensure, Context, Result};
use serde::Deserialize;
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, BTreeSet};

/// Maximum accepted lossless training artifact size.
pub const MAX_PMDC_TRAINING_BYTES: usize = 8 * 1024 * 1024;
/// Maximum accepted lossless development artifact size.
pub const MAX_PMDC_DEVELOPMENT_BYTES: usize = 1024 * 1024;
/// Maximum accepted lossless final artifact size.
pub const MAX_PMDC_FINAL_BYTES: usize = 2 * 1024 * 1024;
// The pinned 2,223-byte manifest is the longest source line; data rows are <=361 bytes.
const MAX_LINE_BYTES: usize = 4096;
const CHANNELS: [&str; 12] = [
    "time",
    "encoderCount",
    "Velocity",
    "rawCurrent",
    "Current",
    "rawVoltageA1",
    "rawVoltageB1",
    "VoltageA1",
    "VoltageB1",
    "MotorVoltage",
    "MotorStatus",
    "PWM",
];

/// One losslessly traced source sample needed for effective identification.
#[derive(Clone, Debug, PartialEq)]
pub struct PmdcSample {
    /// One-based worksheet row in `PRBS9-MotorA`.
    pub source_row: u32,
    /// Unmodified source acquisition timestamp.
    pub source_time_us: u64,
    /// Unmodified cumulative quadrature count.
    pub encoder_count: i64,
    /// Author-derived ten-reading current aggregate.
    pub current_a: f64,
    /// Author-derived terminal potential difference.
    pub terminal_voltage_v: f64,
    /// Author-derived fixed-10-ms velocity, retained only as a diagnostic.
    pub source_velocity_rpm: f64,
}

/// One complete, unsplit source trial.
#[derive(Clone, Debug, PartialEq)]
pub struct PmdcRun {
    /// Frozen one-based PRBS9 trial number.
    pub trial: u8,
    /// Stable converter run identifier.
    pub run_id: String,
    /// All source samples in original row order.
    pub samples: Vec<PmdcSample>,
}

/// Exact retained training input bound to source and record-stream digests.
#[derive(Clone, Debug, PartialEq)]
pub struct PmdcTrainingSet {
    /// Published source workbook digest.
    pub source_sha256: String,
    /// Digest over canonical JSONL record lines.
    pub records_sha256: String,
    /// Eight complete training runs in frozen order.
    pub runs: Vec<PmdcRun>,
}

/// Exact one-run development input, kept distinct from training by type.
#[derive(Clone, Debug, PartialEq)]
pub struct PmdcDevelopmentSet {
    /// Published source workbook digest.
    pub source_sha256: String,
    /// Digest over canonical development JSONL record lines.
    pub records_sha256: String,
    /// Frozen trial 9 and all its unmodified samples.
    pub run: PmdcRun,
}

/// Exact two-run final input, distinct from training and development by type.
#[derive(Clone, Debug, PartialEq)]
pub struct PmdcFinalSet {
    /// Published source workbook digest.
    pub source_sha256: String,
    /// Frozen final-evaluation protocol digest recorded by the sealer.
    pub final_protocol_sha256: String,
    /// Digest over the canonical final JSONL record lines.
    pub records_sha256: String,
    /// Complete final trials 10 and 11 in frozen order.
    pub runs: Vec<PmdcRun>,
}

/// SI observation reconstructed at the later endpoint of an encoder interval.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PmdcObservation {
    /// Later source row that owns current and voltage values.
    pub source_row: u32,
    /// Later source timestamp in seconds without epoch interpretation.
    pub source_time_s: f64,
    /// Interval from the preceding raw source row.
    pub encoder_interval_s: f64,
    /// Reported aggregate current at the later row.
    pub current_a: f64,
    /// Reported terminal voltage at the later row.
    pub terminal_voltage_v: f64,
    /// Gear-output angular velocity reconstructed using the actual interval.
    pub output_speed_rad_s: f64,
}

#[derive(Deserialize)]
struct Manifest {
    kind: String,
    partition: String,
    source_sha256: String,
    sample_counts: BTreeMap<String, usize>,
    final_partition_read: bool,
    #[serde(default)]
    final_protocol_sha256: Option<String>,
    #[serde(default)]
    final_responses_evaluated: Option<bool>,
}

#[derive(Deserialize)]
struct Record {
    partition: String,
    run_id: String,
    source_row: u32,
    values: BTreeMap<String, String>,
}

#[derive(Deserialize)]
struct Trailer {
    kind: String,
    record_count: usize,
    records_sha256: String,
}

struct DecodeContract<'a> {
    manifest_kind: &'a str,
    trailer_kind: &'a str,
    partition: &'a str,
    source_sha256: &'a str,
    records_sha256: &'a str,
    trials: &'a [u8],
    samples_per_trial: usize,
    final_partition_read: bool,
    final_protocol_sha256: Option<&'a str>,
    final_responses_evaluated: Option<bool>,
    artifact_sha256: Option<&'a str>,
    maximum_bytes: usize,
}

fn parse_finite(values: &BTreeMap<String, String>, channel: &str, row: u32) -> Result<f64> {
    let value = values
        .get(channel)
        .with_context(|| format!("missing {channel} at PMDC source row {row}"))?
        .parse::<f64>()
        .with_context(|| format!("invalid {channel} at PMDC source row {row}"))?;
    ensure!(
        value.is_finite(),
        "nonfinite {channel} at PMDC source row {row}"
    );
    Ok(value)
}

#[allow(clippy::too_many_lines)] // TODO(cleanup): split (156/150 lines); see PR body
fn decode_with_contract(bytes: &[u8], contract: DecodeContract<'_>) -> Result<PmdcTrainingSet> {
    ensure!(
        !bytes.is_empty() && bytes.len() <= contract.maximum_bytes,
        "PMDC artifact exceeds bounded input"
    );
    if let Some(expected) = contract.artifact_sha256 {
        ensure!(
            format!("{:x}", Sha256::digest(bytes)) == expected,
            "PMDC artifact digest drift"
        );
    }
    let text = std::str::from_utf8(bytes).context("PMDC training artifact is not UTF-8")?;
    ensure!(
        text.ends_with('\n') && !text.contains('\r'),
        "PMDC training artifact must use canonical LF records"
    );
    let lines = text
        .strip_suffix('\n')
        .unwrap()
        .split('\n')
        .collect::<Vec<_>>();
    ensure!(lines.len() >= 3, "PMDC training artifact is incomplete");
    ensure!(
        lines.iter().all(|line| line.len() <= MAX_LINE_BYTES),
        "PMDC training JSONL line exceeds limit"
    );
    let manifest: Manifest = serde_json::from_str(lines[0]).context("decode PMDC manifest")?;
    ensure!(
        manifest.kind == contract.manifest_kind,
        "PMDC manifest kind drift"
    );
    ensure!(
        manifest.partition == contract.partition,
        "PMDC artifact partition drift"
    );
    ensure!(
        manifest.final_partition_read == contract.final_partition_read,
        "PMDC final-access state drift"
    );
    ensure!(
        manifest.final_protocol_sha256.as_deref() == contract.final_protocol_sha256,
        "PMDC final protocol binding drift"
    );
    ensure!(
        manifest.final_responses_evaluated == contract.final_responses_evaluated,
        "PMDC final evaluation-state drift"
    );
    ensure!(
        manifest.source_sha256 == contract.source_sha256,
        "PMDC source digest drift"
    );
    let trailer: Trailer =
        serde_json::from_str(lines.last().unwrap()).context("decode PMDC trailer")?;
    ensure!(
        trailer.kind == contract.trailer_kind,
        "PMDC trailer kind drift"
    );
    ensure!(
        trailer.record_count == lines.len() - 2,
        "PMDC record-count drift"
    );
    let mut digest = Sha256::new();
    for line in &lines[1..lines.len() - 1] {
        digest.update(line.as_bytes());
        digest.update(b"\n");
    }
    let records_sha256 = format!("{:x}", digest.finalize());
    ensure!(
        records_sha256 == contract.records_sha256
            && trailer.records_sha256 == contract.records_sha256,
        "PMDC training record digest drift"
    );

    let expected_channels = CHANNELS.into_iter().collect::<BTreeSet<_>>();
    let mut expected_counts = BTreeMap::new();
    for trial in contract.trials {
        expected_counts.insert(
            format!("prbs9_motor_a_trial_{trial:02}"),
            contract.samples_per_trial,
        );
    }
    ensure!(
        manifest.sample_counts == expected_counts,
        "PMDC manifest run-count drift"
    );

    let mut runs = contract
        .trials
        .iter()
        .map(|trial| PmdcRun {
            trial: *trial,
            run_id: format!("prbs9_motor_a_trial_{trial:02}"),
            samples: Vec::with_capacity(contract.samples_per_trial),
        })
        .collect::<Vec<_>>();
    let mut run_index = 0usize;
    for line in &lines[1..lines.len() - 1] {
        let record: Record = serde_json::from_str(line).context("decode PMDC record")?;
        ensure!(
            record.partition == contract.partition,
            "PMDC record partition drift"
        );
        while record.run_id != runs[run_index].run_id {
            ensure!(
                runs[run_index].samples.len() == contract.samples_per_trial,
                "PMDC run is interleaved or incomplete"
            );
            run_index += 1;
            ensure!(run_index < runs.len(), "unexpected PMDC run identifier");
        }
        let actual_channels = record
            .values
            .keys()
            .map(String::as_str)
            .collect::<BTreeSet<_>>();
        ensure!(
            actual_channels == expected_channels,
            "PMDC channel-set drift"
        );
        for channel in CHANNELS {
            parse_finite(&record.values, channel, record.source_row)?;
        }
        let source_time_us = record.values["time"]
            .parse::<u64>()
            .with_context(|| format!("invalid PMDC timestamp at row {}", record.source_row))?;
        let encoder_count = record.values["encoderCount"]
            .parse::<i64>()
            .with_context(|| format!("invalid PMDC encoder count at row {}", record.source_row))?;
        let sample = PmdcSample {
            source_row: record.source_row,
            source_time_us,
            encoder_count,
            current_a: parse_finite(&record.values, "Current", record.source_row)?,
            terminal_voltage_v: parse_finite(&record.values, "MotorVoltage", record.source_row)?,
            source_velocity_rpm: parse_finite(&record.values, "Velocity", record.source_row)?,
        };
        if let Some(previous) = runs[run_index].samples.last() {
            ensure!(
                sample.source_row > previous.source_row
                    && sample.source_time_us > previous.source_time_us,
                "PMDC rows or timestamps are not strictly increasing"
            );
        }
        runs[run_index].samples.push(sample);
        ensure!(
            runs[run_index].samples.len() <= contract.samples_per_trial,
            "PMDC run exceeds declared sample count"
        );
    }
    ensure!(
        runs.iter()
            .all(|run| run.samples.len() == contract.samples_per_trial),
        "PMDC training run-count mismatch"
    );
    Ok(PmdcTrainingSet {
        source_sha256: manifest.source_sha256,
        records_sha256,
        runs,
    })
}

/// Decode only the exact eight-run training artifact admitted by protocol v1.
pub fn decode_pmdc_training(bytes: &[u8]) -> Result<PmdcTrainingSet> {
    decode_with_contract(
        bytes,
        DecodeContract {
            manifest_kind: "rne_pmdc_prbs9_partition",
            trailer_kind: "rne_pmdc_prbs9_partition_end",
            partition: "training",
            source_sha256: PMDC_SOURCE_SHA256,
            records_sha256: PMDC_TRAINING_RECORDS_SHA256,
            trials: &pmdc_identification_protocol().training_trials,
            samples_per_trial: 2009,
            final_partition_read: false,
            final_protocol_sha256: None,
            final_responses_evaluated: None,
            artifact_sha256: None,
            maximum_bytes: MAX_PMDC_TRAINING_BYTES,
        },
    )
}

/// Decode only the exact trial-9 development artifact admitted by protocol v1.
pub fn decode_pmdc_development(bytes: &[u8]) -> Result<PmdcDevelopmentSet> {
    ensure!(
        bytes.len() <= MAX_PMDC_DEVELOPMENT_BYTES,
        "PMDC development artifact exceeds bounded input"
    );
    let decoded = decode_with_contract(
        bytes,
        DecodeContract {
            manifest_kind: "rne_pmdc_prbs9_partition",
            trailer_kind: "rne_pmdc_prbs9_partition_end",
            partition: "development",
            source_sha256: PMDC_SOURCE_SHA256,
            records_sha256: PMDC_DEVELOPMENT_RECORDS_SHA256,
            trials: &pmdc_identification_protocol().development_trials,
            samples_per_trial: 2009,
            final_partition_read: false,
            final_protocol_sha256: None,
            final_responses_evaluated: None,
            artifact_sha256: None,
            maximum_bytes: MAX_PMDC_DEVELOPMENT_BYTES,
        },
    )?;
    let mut runs = decoded.runs;
    ensure!(runs.len() == 1, "PMDC development run-count drift");
    Ok(PmdcDevelopmentSet {
        source_sha256: decoded.source_sha256,
        records_sha256: decoded.records_sha256,
        run: runs.remove(0),
    })
}

/// Decode only the exact two-run final artifact admitted by final protocol v1.
pub fn decode_pmdc_final(bytes: &[u8]) -> Result<PmdcFinalSet> {
    ensure!(
        bytes.len() == PMDC_FINAL_ARTIFACT_BYTES,
        "PMDC final artifact byte length drift"
    );
    let protocol = pmdc_final_evaluation_protocol();
    let protocol_sha256 = protocol.sha256()?;
    let decoded = decode_with_contract(
        bytes,
        DecodeContract {
            manifest_kind: "rne_pmdc_prbs9_final_partition",
            trailer_kind: "rne_pmdc_prbs9_final_partition_end",
            partition: "final",
            source_sha256: PMDC_SOURCE_SHA256,
            records_sha256: PMDC_FINAL_RECORDS_SHA256,
            trials: &protocol.final_trials,
            samples_per_trial: protocol.samples_per_run,
            final_partition_read: true,
            final_protocol_sha256: Some(&protocol_sha256),
            final_responses_evaluated: Some(false),
            artifact_sha256: Some(PMDC_FINAL_ARTIFACT_SHA256),
            maximum_bytes: MAX_PMDC_FINAL_BYTES,
        },
    )?;
    Ok(PmdcFinalSet {
        source_sha256: decoded.source_sha256,
        final_protocol_sha256: protocol_sha256,
        records_sha256: decoded.records_sha256,
        runs: decoded.runs,
    })
}

impl PmdcRun {
    /// Reconstruct SI output speed from counts and actual time without filtering.
    pub fn observations(&self) -> Result<Vec<PmdcObservation>> {
        ensure!(
            self.samples.len() >= 3,
            "PMDC run has too few source samples"
        );
        let protocol = pmdc_identification_protocol();
        let pulses = f64::from(protocol.encoder_pulses_per_revolution);
        let reduction = f64::from(protocol.gear_reduction_denominator)
            / f64::from(protocol.gear_reduction_numerator);
        let mut observations = Vec::with_capacity(self.samples.len() - 1);
        for pair in self.samples.windows(2) {
            let delta_us = pair[1]
                .source_time_us
                .checked_sub(pair[0].source_time_us)
                .context("nonpositive PMDC timestamp interval")?;
            ensure!(delta_us > 0, "zero PMDC timestamp interval");
            let interval_s = delta_us as f64 * 1e-6;
            let count_delta = pair[1]
                .encoder_count
                .checked_sub(pair[0].encoder_count)
                .context("PMDC encoder-count overflow")?;
            let motor_revolutions = -(count_delta as f64) / pulses;
            let output_revolutions = motor_revolutions / reduction;
            let output_speed_rad_s = output_revolutions * std::f64::consts::TAU / interval_s;
            ensure!(
                output_speed_rad_s.is_finite(),
                "nonfinite PMDC output speed"
            );
            observations.push(PmdcObservation {
                source_row: pair[1].source_row,
                source_time_s: pair[1].source_time_us as f64 * 1e-6,
                encoder_interval_s: interval_s,
                current_a: pair[1].current_a,
                terminal_voltage_v: pair[1].terminal_voltage_v,
                output_speed_rad_s,
            });
        }
        Ok(observations)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    fn fixture() -> (Vec<u8>, String) {
        let trials = [1_u8, 2];
        let mut lines = Vec::new();
        let mut counts = BTreeMap::new();
        for trial in trials {
            counts.insert(format!("prbs9_motor_a_trial_{trial:02}"), 3);
        }
        lines.push(
            serde_json::to_string(&json!({
                "kind": "rne_pmdc_prbs9_partition",
                "partition": "training",
                "source_sha256": "fixture-source",
                "sample_counts": counts,
                "final_partition_read": false
            }))
            .unwrap(),
        );
        let mut record_digest = Sha256::new();
        for trial in trials {
            for index in 0..3_u32 {
                let mut values = BTreeMap::new();
                for channel in CHANNELS {
                    values.insert(channel, "0".to_owned());
                }
                values.insert("time", (u64::from(index) * 10_000).to_string());
                values.insert("encoderCount", (i64::from(index) * -306).to_string());
                values.insert("Current", (f64::from(index) + 1.0).to_string());
                values.insert("MotorVoltage", "12".into());
                let line = serde_json::to_string(&json!({
                    "partition": "training",
                    "run_id": format!("prbs9_motor_a_trial_{trial:02}"),
                    "source_row": u32::from(trial) * 100 + index,
                    "values": values
                }))
                .unwrap();
                record_digest.update(line.as_bytes());
                record_digest.update(b"\n");
                lines.push(line);
            }
        }
        let digest = format!("{:x}", record_digest.finalize());
        lines.push(
            serde_json::to_string(&json!({
                "kind": "rne_pmdc_prbs9_partition_end",
                "record_count": 6,
                "records_sha256": digest
            }))
            .unwrap(),
        );
        ((lines.join("\n") + "\n").into_bytes(), digest)
    }

    #[test]
    fn bounded_decoder_binds_records_and_reconstructs_si_speed() {
        let (bytes, digest) = fixture();
        let trials = [1, 2];
        let decoded = decode_with_contract(
            &bytes,
            DecodeContract {
                manifest_kind: "rne_pmdc_prbs9_partition",
                trailer_kind: "rne_pmdc_prbs9_partition_end",
                partition: "training",
                source_sha256: "fixture-source",
                records_sha256: &digest,
                trials: &trials,
                samples_per_trial: 3,
                final_partition_read: false,
                final_protocol_sha256: None,
                final_responses_evaluated: None,
                artifact_sha256: None,
                maximum_bytes: MAX_PMDC_TRAINING_BYTES,
            },
        )
        .unwrap();
        assert_eq!(decoded.runs.len(), 2);
        let observations = decoded.runs[0].observations().unwrap();
        assert_eq!(observations.len(), 2);
        assert!((observations[0].output_speed_rad_s - std::f64::consts::TAU).abs() < 1e-12);
        assert_eq!(observations[0].encoder_interval_s, 0.01);
        assert_eq!(observations[0].current_a, 2.0);
    }

    #[test]
    fn digest_final_access_and_clock_mutations_fail_closed() {
        let (bytes, digest) = fixture();
        let trials = [1, 2];
        let decode = |input: &[u8], expected: &str| {
            decode_with_contract(
                input,
                DecodeContract {
                    manifest_kind: "rne_pmdc_prbs9_partition",
                    trailer_kind: "rne_pmdc_prbs9_partition_end",
                    partition: "training",
                    source_sha256: "fixture-source",
                    records_sha256: expected,
                    trials: &trials,
                    samples_per_trial: 3,
                    final_partition_read: false,
                    final_protocol_sha256: None,
                    final_responses_evaluated: None,
                    artifact_sha256: None,
                    maximum_bytes: MAX_PMDC_TRAINING_BYTES,
                },
            )
        };
        let mut changed = bytes.clone();
        let position = changed.iter().position(|byte| *byte == b'1').unwrap();
        changed[position] = b'2';
        assert!(decode(&changed, &digest).is_err());
        let final_read = String::from_utf8(bytes.clone()).unwrap().replacen(
            "\"final_partition_read\":false",
            "\"final_partition_read\":true",
            1,
        );
        assert!(decode(final_read.as_bytes(), &digest).is_err());
        assert!(decode(&vec![b'x'; MAX_PMDC_TRAINING_BYTES + 1], &digest).is_err());
    }

    #[test]
    fn final_decoder_rejects_any_unpinned_artifact_before_parsing() {
        assert!(decode_pmdc_final(b"not-the-pinned-final-artifact").is_err());
        assert!(decode_pmdc_final(&vec![b'x'; MAX_PMDC_FINAL_BYTES + 1]).is_err());
    }
}
