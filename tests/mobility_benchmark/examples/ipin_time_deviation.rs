//! Source-bound single-axis time-window analysis, without noise-profile fitting.
use anyhow::{ensure, Context, Result};
use rne_mobility_benchmark::recorded_ipin::analyze_ipin_axis;
use rne_sensor::allan::timed::TimeWindowPlan;
use serde_json::json;
use std::{
    env,
    fs::File,
    io::{self, BufRead, BufReader},
};

fn main() -> Result<()> {
    let a: Vec<_> = env::args().skip(1).collect();
    ensure!(a.len()==6,"usage: ipin_time_deviation <path|-> <axis> <window_us> <first_endpoint_us> <endpoint_period_us> <endpoints>");
    let ticks = |i: usize| -> Result<u64> {
        a[i].parse::<u64>()?
            .checked_mul(1000)
            .context("tick overflow")
    };
    let plan = TimeWindowPlan {
        window_ticks: ticks(2)?,
        first_endpoint_ticks: ticks(3)?,
        endpoint_period_ticks: ticks(4)?,
        endpoints: a[5].parse()?,
    };
    let input: Box<dyn BufRead> = if a[0] == "-" {
        Box::new(BufReader::new(io::stdin()))
    } else {
        Box::new(BufReader::new(File::open(&a[0])?))
    };
    let result = analyze_ipin_axis(input, a[1].parse()?, plan)?;
    println!(
        "{}",
        serde_json::to_string_pretty(&json!({
            "schema_version":1,"algorithm":"sample_count_weighted_adjacent_time_windows_v1",
            "source":result.source,"axis":result.axis,
            "plan":{"window_ticks":plan.window_ticks,"first_endpoint_ticks":plan.first_endpoint_ticks,
                "endpoint_period_ticks":plan.endpoint_period_ticks,"endpoints":plan.endpoints},
            "statistic":{"source_samples":result.statistic.source_samples,"valid_pairs":result.statistic.valid_pairs,
                "empty_pairs":result.statistic.empty_pairs,"total_weight":result.statistic.total_weight,
                "variance_source_units_squared":result.statistic.variance,"deviation_source_units":result.statistic.deviation},
            "physical_calibration":false,"capture_timing_qualified":false
        }))?
    );
    Ok(())
}
