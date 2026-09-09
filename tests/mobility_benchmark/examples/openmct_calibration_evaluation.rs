//! Read-only separate-capture evaluation; no refitting on evaluation values.

use anyhow::{ensure, Result};
use rne_mobility_benchmark::recorded_openmct::{
    evaluation::evaluate_current_calibration, read_openmct,
};
use serde_json::json;
use std::fs::File;

fn main() -> Result<()> {
    let args: Vec<_> = std::env::args().skip(1).collect();
    ensure!(
        args.len() == 2,
        "usage: openmct_calibration_evaluation <calibration.txt> <evaluation.txt>"
    );
    let training = read_openmct(File::open(&args[0])?)?;
    let evaluation = read_openmct(File::open(&args[1])?)?;
    let report = evaluate_current_calibration(&training, &evaluation)?;
    let rows: Vec<_> = report
        .rows
        .iter()
        .map(|row| {
            json!({
                "source_row": row.source_row,
                "predicted_magnitude_a": row.predicted_magnitude_a,
                "residual_a": row.residual_a,
                "selected": row.selected,
                "current_adc_counts": row.current_adc_counts,
                "extrapolated": row.extrapolated,
                "dmm": row.dmm.as_ref().map(|d| json!({
                    "sample_id": d.sample_id, "current_a": d.current_a,
                    "time_s": d.time_s, "age_ms": d.age_ms
                }))
            })
        })
        .collect();
    println!(
        "{}",
        serde_json::to_string(&json!({
            "kind": "rne_openmct_frozen_calibration_evaluation", "schema_version": 1,
            "training_sha256": report.training.source_sha256,
            "evaluation_sha256": report.evaluation_sha256,
            "slope_a_per_count": report.training.law.slope_a_per_count,
            "intercept_a": report.training.law.intercept_a,
            "selected_count": report.selected_count, "rmse_a": report.rmse_a,
            "max_absolute_residual_a": report.max_absolute_residual_a,
            "fitted_adc_range_counts": report.fitted_adc_range_counts,
            "physical_calibration_qualified": false, "rows": rows
        }))?
    );
    Ok(())
}
