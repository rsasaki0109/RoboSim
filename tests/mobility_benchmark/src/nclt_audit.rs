//! Read-only CLI for auditing external NCLT wheel or IMU CSV files.

use anyhow::{bail, ensure, Result};
use rne_mobility_benchmark::recorded_nclt::{read_imu_samples, read_wheel_samples, NcltSeries};
use std::{fs::File, path::PathBuf};

fn main() -> Result<()> {
    let args: Vec<_> = std::env::args_os().skip(1).collect();
    ensure!(
        args.len() == 2,
        "usage: rne-nclt-audit <wheels|imu> <csv-path>"
    );
    let kind = args[0].to_str().unwrap_or_default();
    ensure!(matches!(kind, "wheels" | "imu"), "expected wheels or imu");
    let input = File::open(PathBuf::from(&args[1]))?;
    ensure!(input.metadata()?.is_file(), "input must be a regular file");
    let report = match kind {
        "wheels" => summary(read_wheel_samples(input)?, kind, |s| s.timestamp_us),
        "imu" => summary(read_imu_samples(input)?, kind, |s| s.timestamp_us),
        _ => bail!("unsupported input"),
    };
    println!("{}", serde_json::to_string_pretty(&report)?);
    Ok(())
}

fn summary<T>(
    series: NcltSeries<T>,
    channel: &str,
    timestamp: impl Fn(&T) -> u64,
) -> serde_json::Value {
    let deltas: Vec<_> = series
        .samples
        .windows(2)
        .map(|w| timestamp(&w[1]) - timestamp(&w[0]))
        .collect();
    serde_json::json!({
        "kind": "rne_nclt_source_audit", "schema_version": 1,
        "channel": channel, "source_sha256": series.source_sha256,
        "source_bytes": series.source_bytes, "sample_count": series.samples.len(),
        "first_timestamp_us": series.samples.first().map(&timestamp),
        "last_timestamp_us": series.samples.last().map(&timestamp),
        "minimum_interval_us": deltas.iter().min(), "maximum_interval_us": deltas.iter().max(),
        "timestamp_interpretation": "NCLT Unix microseconds; wheel convention inherited from dataset",
        "receipt_time_known": false, "physical_accuracy_validated": false,
        "resampled": false, "source_frame_preserved": true
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn single_sample_has_no_invented_interval() {
        let series = read_wheel_samples(&b"1350000000000000,0,0\n"[..]).unwrap();
        let result = summary(series, "wheels", |s| s.timestamp_us);
        assert!(result["minimum_interval_us"].is_null());
        assert!(result["maximum_interval_us"].is_null());
        assert_eq!(result["physical_accuracy_validated"], false);
    }
}
