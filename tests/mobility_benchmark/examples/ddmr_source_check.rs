//! Offline ingestion smoke: source integrity and disjoint prediction windows only.

use anyhow::{ensure, Context, Result};
use rne_mobility_benchmark::recorded_ddmr::{read_ddmr_samples, DdmrPartition};
use serde_json::{json, Value};
use std::{env, fs::File};

fn partition_summary(
    partition: &DdmrPartition<'_>,
    history: usize,
    horizon: usize,
) -> Result<Value> {
    let rows = partition.raw_rows();
    let windows = partition.windows(history, horizon)?;
    Ok(json!({
        "raw_rows_start_inclusive": rows.start,
        "raw_rows_end_exclusive": rows.end,
        "prediction_windows": windows.len(),
    }))
}

fn main() -> Result<()> {
    let args: Vec<_> = env::args().skip(1).collect();
    ensure!(args.len() == 5,
        "usage: ddmr_source_check <csv> <train_end_row> <validation_end_row> <history_rows> <horizon_rows>");
    let train_end = args[1].parse().context("train_end_row")?;
    let validation_end = args[2].parse().context("validation_end_row")?;
    let history = args[3].parse().context("history_rows")?;
    let horizon = args[4].parse().context("horizon_rows")?;
    let series = read_ddmr_samples(File::open(&args[0]).context("open source CSV")?)?;
    let split = series.split(train_end, validation_end)?;
    println!(
        "{}",
        serde_json::to_string_pretty(&json!({
            "kind": "rne_ddmr_source_check",
            "schema_version": 1,
            "source_sha256": series.source_sha256(),
            "source_bytes": series.source_bytes(),
            "source_rows": series.samples().len(),
            "physical_calibration_qualified": false,
            "capture_timing_qualified": false,
            "voltage_measurement_qualified": false,
            "independent_recording_sessions": false,
            "history_rows": history,
            "horizon_rows": horizon,
            "training": partition_summary(&split.training, history, horizon)?,
            "validation": partition_summary(&split.validation, history, horizon)?,
            "test": partition_summary(&split.test, history, horizon)?,
        }))?
    );
    Ok(())
}
