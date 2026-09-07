//! Fixed exploratory protocol; no model search or physical-parameter acceptance.

use anyhow::{ensure, Context, Result};
use rne_mobility_benchmark::recorded_ddmr::{
    read_ddmr_samples,
    response::{ResponseMode, WheelResponseModel, RESPONSE_ALGORITHM},
    DdmrPartition,
};
use serde_json::{json, Value};
use sha2::{Digest, Sha256};
use std::{
    env,
    fs::{File, OpenOptions},
    io::Write,
};

fn evaluate(model: &WheelResponseModel, partition: &DdmrPartition<'_>) -> Value {
    let outcomes: Vec<_> = [ResponseMode::OneStep, ResponseMode::FreeRun]
        .into_iter()
        .map(|mode| match model.evaluate(partition, mode) {
            Ok(metrics) => json!({"mode": mode, "metrics": metrics, "execution_error": null}),
            Err(error) => {
                json!({"mode": mode, "metrics": null, "execution_error": format!("{error:#}")})
            }
        })
        .collect();
    let rows = partition.raw_rows();
    json!({"raw_rows_start_inclusive": rows.start, "raw_rows_end_exclusive": rows.end, "outcomes": outcomes})
}

fn main() -> Result<()> {
    let args: Vec<_> = env::args().skip(1).collect();
    ensure!(
        args.len() == 2,
        "usage: ddmr_response_experiment <pinned Data.csv> <new output.json>"
    );
    let source = read_ddmr_samples(File::open(&args[0]).context("open source")?)?;
    ensure!(
        source.source_sha256()
            == "c278dde8bfc38974bb2b1cc054160349da51456817c898ea2e052656aa75f60e"
            && source.samples().len() == 338_550,
        "frozen protocol source mismatch"
    );
    let split = source.split(203_130, 270_840)?;
    let model = WheelResponseModel::fit(&split.training, 0.01, 1e-9)?;
    let report = json!({
        "kind": "rne_ddmr_response_experiment", "schema_version": 1,
        "algorithm": RESPONSE_ALGORITHM,
        "source_sha256": source.source_sha256(), "source_bytes": source.source_bytes(),
        "model": model, "stable_poles": model.stable_poles(),
        "physical_calibration_qualified": false, "independent_recording_sessions": false,
        "hyperparameter_search": false, "units": "rad/s",
        "training": evaluate(&model, &split.training),
        "validation": evaluate(&model, &split.validation),
        "test": evaluate(&model, &split.test),
        "build_revision": env!("RNE_MOBILITY_BUILD_REVISION"),
        "build_clean": env!("RNE_MOBILITY_BUILD_CLEAN"),
        "compiler": env!("RNE_MOBILITY_BUILD_COMPILER"),
        "target": env!("RNE_MOBILITY_BUILD_TARGET"),
        "lock_sha256": env!("RNE_MOBILITY_BUILD_LOCK"),
        "experiment_source_sha256": format!("{:x}", Sha256::digest(include_bytes!("ddmr_response_experiment.rs"))),
        "reader_source_sha256": format!("{:x}", Sha256::digest(include_bytes!("../src/recorded_ddmr.rs"))),
        "response_source_sha256": format!("{:x}", Sha256::digest(include_bytes!("../src/recorded_ddmr/response.rs"))),
    });
    let bytes = serde_json::to_vec_pretty(&report)?;
    let mut output = OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(&args[1])
        .context("create new evidence file; existing results are never overwritten")?;
    output.write_all(&bytes)?;
    output.sync_all()?;
    println!("{}", String::from_utf8(bytes)?);
    Ok(())
}
