//! Read-only CLI for auditing external NCLT wheel or IMU CSV files.

use anyhow::{bail, ensure, Result};
use rne_mobility_benchmark::recorded_nclt::replay::{NcltReplay, NcltReplayPolicy};
use rne_mobility_benchmark::recorded_nclt::{read_imu_samples, read_wheel_samples, NcltSeries};
use sha2::{Digest, Sha256};
use std::{fs::File, path::PathBuf};

fn main() -> Result<()> {
    let args: Vec<_> = std::env::args_os().skip(1).collect();
    if args.first().is_some_and(|a| a == "replay") {
        ensure!(
            args.len() == 4,
            "usage: rne-nclt-audit replay <wheels.csv> <ms25.csv> <delay-us>"
        );
        let delay_us = args[3]
            .to_str()
            .ok_or_else(|| anyhow::anyhow!("delay must be UTF-8 integer"))?
            .parse::<u64>()?;
        let wheels = File::open(PathBuf::from(&args[1]))?;
        let imu = File::open(PathBuf::from(&args[2]))?;
        ensure!(
            wheels.metadata()?.is_file() && imu.metadata()?.is_file(),
            "inputs must be regular files"
        );
        let mut world = rne_ecs::World::new();
        let entity = rne_ecs::spawn_named(&mut world, "recorded NCLT source");
        let replay = NcltReplay::new(
            wheels,
            imu,
            NcltReplayPolicy {
                wheel_delay_us: delay_us,
                imu_delay_us: delay_us,
            },
            entity,
        )?;
        println!(
            "{}",
            serde_json::to_string_pretty(&replay_summary(replay)?)?
        );
        return Ok(());
    }
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

fn replay_summary(mut replay: NcltReplay) -> Result<serde_json::Value> {
    let mut hasher = Sha256::new();
    hasher.update(b"rne_nclt_replay_observations_v1\0");
    // Bind explicit scheduling policy and original bytes; entity allocation is not
    // part of this source-observation digest. This is not a physics state hash.
    hasher.update(serde_json::to_vec(&(
        replay.origin_us(),
        replay.policy(),
        replay.source_sha256(),
    ))?);
    let mut boundaries = 0_u64;
    while let Some(time) = replay.next_delivery_time() {
        let observation = replay.advance_to(time)?;
        let wheels = observation.wheels.as_ref().map(|f| {
            (
                f.sequence,
                f.capture_time.ticks(),
                f.available_time.ticks(),
                &f.payload,
            )
        });
        let imu = observation.imu.as_ref().map(|f| {
            (
                f.sequence,
                f.capture_time.ticks(),
                f.available_time.ticks(),
                &f.payload,
            )
        });
        let record = serde_json::to_vec(&(time.ticks(), wheels, imu))?;
        hasher.update((record.len() as u64).to_le_bytes());
        hasher.update(record);
        boundaries += 1;
    }
    let final_observation = replay.observation();
    Ok(serde_json::json!({
        "kind": "rne_nclt_replay_audit", "schema_version": 1,
        "origin_us": replay.origin_us(), "policy": replay.policy(),
        "source_sha256": replay.source_sha256(), "delivery_boundaries": boundaries,
        "wheel_samples": final_observation.wheels.as_ref().map(|f| f.sequence + 1),
        "imu_samples": final_observation.imu.as_ref().map(|f| f.sequence + 1),
        "final_decision_ticks": final_observation.decision_time.ticks(),
        "observation_sha256": format!("{:x}", hasher.finalize()),
        "physical_accuracy_validated": false, "measured_latency_known": false,
        "source_frame_preserved": true, "resampled": false
    }))
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
    fn replay_digest_is_repeatable_and_binds_policy() {
        let run = |delay_us| {
            let mut world = rne_ecs::World::new();
            let entity = rne_ecs::spawn_named(&mut world, "source");
            replay_summary(
                NcltReplay::new(
                    &b"1350000000000000,1,2\n"[..],
                    &b"1350000000000001,1,2,3,4,5,6,7,8,9\n"[..],
                    NcltReplayPolicy {
                        wheel_delay_us: delay_us,
                        imu_delay_us: delay_us,
                    },
                    entity,
                )
                .unwrap(),
            )
            .unwrap()
        };
        let a = run(0);
        assert_eq!(a, run(0));
        assert_eq!(a["delivery_boundaries"], 2);
        assert_eq!(a["wheel_samples"], 1);
        assert_eq!(a["imu_samples"], 1);
        assert_ne!(a["observation_sha256"], run(10)["observation_sha256"]);
    }

    #[test]
    fn single_sample_has_no_invented_interval() {
        let series = read_wheel_samples(&b"1350000000000000,0,0\n"[..]).unwrap();
        let result = summary(series, "wheels", |s| s.timestamp_us);
        assert!(result["minimum_interval_us"].is_null());
        assert!(result["maximum_interval_us"].is_null());
        assert_eq!(result["physical_accuracy_validated"], false);
    }
}
