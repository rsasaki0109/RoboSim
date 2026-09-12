//! Read-only fixed-model diagnostic. The model is supplied, never fitted here.

use anyhow::{ensure, Result};
use rne_mobility_benchmark::recorded_openmct::{
    read_openmct,
    response::{evaluate_speed_response, FirstOrderSpeedModel},
};
use serde_json::json;
use std::fs::File;

fn main() -> Result<()> {
    let args: Vec<_> = std::env::args().skip(1).collect();
    ensure!(
        args.len() == 3,
        "usage: openmct_speed_response <raw-log.txt> <gain_rpm_per_pwm_count> <time_constant_s>"
    );
    let source = read_openmct(File::open(&args[0])?)?;
    let model = FirstOrderSpeedModel {
        gain_rpm_per_pwm_count: args[1].parse()?,
        time_constant_s: args[2].parse()?,
    };
    let report = evaluate_speed_response(&source, model)?;
    let transitions: Vec<_> = report
        .transitions
        .iter()
        .map(|r| {
            json!({
                "target_row": r.target_row, "interval_s": r.interval_s,
                "pwm_command": r.pwm_command, "measured_speed_rpm": r.measured_speed_rpm,
                "free_running_speed_rpm": r.free_running_speed_rpm,
                "one_step_speed_rpm": r.one_step_speed_rpm,
                "free_running_residual_rpm": r.free_running_residual_rpm,
                "one_step_residual_rpm": r.one_step_residual_rpm,
                "persistence_residual_rpm": r.persistence_residual_rpm
            })
        })
        .collect();
    println!(
        "{}",
        serde_json::to_string(&json!({
            "kind": "rne_openmct_fixed_speed_response", "schema_version": 1,
            "source_sha256": report.source_sha256,
            "gain_rpm_per_pwm_count": model.gain_rpm_per_pwm_count,
            "time_constant_s": model.time_constant_s,
            "alignment": "preceding_row_pwm_held_over_preceding_declared_interval",
            "initial_speed_rpm": report.initial_speed_rpm,
            "free_running_rmse_rpm": report.free_running_rmse_rpm,
            "one_step_rmse_rpm": report.one_step_rmse_rpm,
            "persistence_rmse_rpm": report.persistence_rmse_rpm,
            "physical_qualified": false, "transitions": transitions
        }))?
    );
    Ok(())
}
