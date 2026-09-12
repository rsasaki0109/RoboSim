//! Identify from one training capture; no validation capture is opened here.

use anyhow::{ensure, Result};
use rne_mobility_benchmark::recorded_openmct::{identification::identify_speed, read_openmct};
use serde_json::json;
use std::fs::File;

fn main() -> Result<()> {
    let args: Vec<_> = std::env::args().skip(1).collect();
    ensure!(
        args.len() == 1,
        "usage: openmct_speed_identification <training-log.txt>"
    );
    let training = read_openmct(File::open(&args[0])?)?;
    let fit = identify_speed(&training)?;
    let (continuous, conversion_error) = match fit.continuous_model() {
        Ok(model) => (
            json!({
                "gain_rpm_per_pwm_count": model.gain_rpm_per_pwm_count,
                "time_constant_s": model.time_constant_s,
            }),
            None,
        ),
        Err(error) => (serde_json::Value::Null, Some(error.to_string())),
    };
    println!(
        "{}",
        serde_json::to_string(&json!({
            "kind": "rne_openmct_speed_identification", "schema_version": 1,
            "algorithm": "zero_offset_arx_1_1_1_ols_v1",
            "training_sha256": fit.training_sha256,
            "training_transitions": fit.training_transitions,
            "interval_s": fit.interval_s,
            "pole": fit.pole,
            "input_gain_rpm_per_pwm_count": fit.input_gain_rpm_per_pwm_count,
            "normalized_determinant": fit.normalized_determinant,
            "continuous_model": continuous, "conversion_error": conversion_error,
            "physical_qualified": false,
        }))?
    );
    Ok(())
}
