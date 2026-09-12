//! Verify and summarize the exact retained PMDC training artifact.

use anyhow::{ensure, Context, Result};
use rne_mobility_benchmark::recorded_pmdc::{
    data::{decode_pmdc_training, MAX_PMDC_TRAINING_BYTES},
    pmdc_identification_protocol,
};
use serde::Serialize;
use std::{fs::File, io::Read, path::PathBuf};

#[derive(Debug, Serialize)]
struct RunSummary {
    trial: u8,
    sample_count: usize,
    observation_count: usize,
    interval_min_s: f64,
    interval_max_s: f64,
    speed_min_rad_s: f64,
    speed_max_rad_s: f64,
}

#[derive(Debug, Serialize)]
struct Summary {
    kind: &'static str,
    source_sha256: String,
    records_sha256: String,
    protocol_sha256: String,
    runs: Vec<RunSummary>,
    final_partition_read: bool,
    physical_accuracy_validated: bool,
}

fn main() -> Result<()> {
    let path = std::env::args_os()
        .nth(1)
        .map(PathBuf::from)
        .context("usage: pmdc_training_check <training.jsonl>")?;
    ensure!(std::env::args_os().nth(2).is_none(), "unexpected argument");
    let metadata = path.metadata().context("stat PMDC training artifact")?;
    ensure!(
        metadata.is_file() && metadata.len() <= MAX_PMDC_TRAINING_BYTES as u64,
        "PMDC training artifact is not a bounded regular file"
    );
    let mut bytes = Vec::with_capacity(metadata.len() as usize);
    File::open(path)?
        .take(MAX_PMDC_TRAINING_BYTES as u64 + 1)
        .read_to_end(&mut bytes)?;
    let training = decode_pmdc_training(&bytes)?;
    let mut runs = Vec::with_capacity(training.runs.len());
    for run in &training.runs {
        let observations = run.observations()?;
        runs.push(RunSummary {
            trial: run.trial,
            sample_count: run.samples.len(),
            observation_count: observations.len(),
            interval_min_s: observations
                .iter()
                .map(|sample| sample.encoder_interval_s)
                .reduce(f64::min)
                .context("empty PMDC observation run")?,
            interval_max_s: observations
                .iter()
                .map(|sample| sample.encoder_interval_s)
                .reduce(f64::max)
                .context("empty PMDC observation run")?,
            speed_min_rad_s: observations
                .iter()
                .map(|sample| sample.output_speed_rad_s)
                .reduce(f64::min)
                .context("empty PMDC observation run")?,
            speed_max_rad_s: observations
                .iter()
                .map(|sample| sample.output_speed_rad_s)
                .reduce(f64::max)
                .context("empty PMDC observation run")?,
        });
    }
    let summary = Summary {
        kind: "rne_pmdc_training_check",
        source_sha256: training.source_sha256,
        records_sha256: training.records_sha256,
        protocol_sha256: pmdc_identification_protocol().sha256()?,
        runs,
        final_partition_read: false,
        physical_accuracy_validated: false,
    };
    println!("{}", serde_json::to_string_pretty(&summary)?);
    Ok(())
}
