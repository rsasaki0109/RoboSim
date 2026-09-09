//! Read-only reproduction of a descriptive fit, not independent validation.

use anyhow::{ensure, Result};
use rne_mobility_benchmark::recorded_openmct::{
    calibration::{reproduce_current_calibration, CalibrationRowUse},
    read_openmct,
};
use serde_json::json;
use std::fs::File;

fn main() -> Result<()> {
    let args: Vec<_> = std::env::args().skip(1).collect();
    ensure!(
        args.len() == 1,
        "usage: openmct_calibration_reproduction <raw-log.txt>"
    );
    let source = read_openmct(File::open(&args[0])?)?;
    let report = reproduce_current_calibration(&source)?;
    let count = |reason| report.row_use.iter().filter(|&&r| r == reason).count();
    println!(
        "{}",
        serde_json::to_string_pretty(&json!({
            "kind": "rne_openmct_calibration_reproduction", "schema_version": 1,
            "source_sha256": report.source_sha256,
            "rows": report.row_use.len(), "candidate_rows": report.candidate_rows.len(),
            "missing_dmm_rows": count(CalibrationRowUse::MissingDmm),
            "stale_dmm_rows": count(CalibrationRowUse::StaleDmm),
            "reused_dmm_rows": count(CalibrationRowUse::ReusedDmm),
            "retained_rows": count(CalibrationRowUse::Fit),
            "residual_rejected_rows": count(CalibrationRowUse::ResidualRejected),
            "slope_a_per_count": report.law.slope_a_per_count,
            "intercept_a": report.law.intercept_a,
            "retained_rmse_a": report.retained_rmse_a,
            "all_candidate_rmse_a": report.all_candidate_rmse_a,
            "independent_validation": false, "physical_calibration_qualified": false
        }))?
    );
    Ok(())
}
