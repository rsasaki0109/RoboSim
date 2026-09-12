//! Read-only residual-lag diagnostics for a supplied, fixed response model.

use anyhow::{ensure, Result};
use rne_mobility_benchmark::recorded_openmct::{
    read_openmct,
    response::{diagnose_one_step_residual_lags, evaluate_speed_response, FirstOrderSpeedModel},
};
use serde_json::json;
use std::fs::File;

fn main() -> Result<()> {
    let args: Vec<_> = std::env::args().skip(1).collect();
    ensure!(
        args.len() == 4,
        "usage: openmct_speed_residuals <raw-log.txt> <gain_rpm_per_pwm_count> <time_constant_s> <max_lag_rows>"
    );
    let source = read_openmct(File::open(&args[0])?)?;
    let response = evaluate_speed_response(
        &source,
        FirstOrderSpeedModel {
            gain_rpm_per_pwm_count: args[1].parse()?,
            time_constant_s: args[2].parse()?,
        },
    )?;
    let report = diagnose_one_step_residual_lags(&response, args[3].parse()?)?;
    let lags: Vec<_> = report
        .lags
        .iter()
        .map(|lag| {
            json!({
                "lag_rows": lag.lag_rows,
                "pair_count": lag.pair_count,
                "residual_autocorrelation": lag.residual_autocorrelation,
                "residual_past_pwm_correlation": lag.residual_past_pwm_correlation,
                "input_age_min_s": lag.input_age_min_s,
                "input_age_mean_s": lag.input_age_mean_s,
                "input_age_max_s": lag.input_age_max_s,
            })
        })
        .collect();
    println!(
        "{}",
        serde_json::to_string(&json!({
            "kind": "rne_openmct_speed_residual_lags",
            "schema_version": 1,
            "source_sha256": report.source_sha256,
            "transition_count": report.transition_count,
            "residual_mean_rpm": report.residual_mean_rpm,
            "pwm_mean_count": report.pwm_mean_count,
            "normalization": "full_record_centered_energies",
            "confidence_tested": false,
            "physical_qualified": false,
            "lags": lags,
        }))?
    );
    Ok(())
}
