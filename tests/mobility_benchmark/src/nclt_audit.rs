//! Read-only CLI for auditing external NCLT wheel or IMU CSV files.

use anyhow::{bail, ensure, Result};
use rne_mobility_benchmark::recorded_nclt::estimation::{
    MeasuredVelocityEstimator, VelocityEstimatorConfig,
};
use rne_mobility_benchmark::recorded_nclt::replay::{NcltReplay, NcltReplayPolicy};
use rne_mobility_benchmark::recorded_nclt::{read_imu_samples, read_wheel_samples, NcltSeries};
use sha2::{Digest, Sha256};
use std::{fs::File, path::PathBuf};

fn main() -> Result<()> {
    let args: Vec<_> = std::env::args_os().skip(1).collect();
    if args
        .first()
        .is_some_and(|a| a == "replay" || a == "estimate")
    {
        let estimate = args[0] == "estimate";
        ensure!(
            args.len() == if estimate { 6 } else { 4 },
            "usage: rne-nclt-audit replay <wheels.csv> <ms25.csv> <delay-us>; or estimate <wheels.csv> <ms25.csv> <delay-us> <wheel-hold-us> <imu-hold-us>"
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
        let report = if estimate {
            let integer = |index: usize| -> Result<u64> {
                Ok(args[index]
                    .to_str()
                    .ok_or_else(|| anyhow::anyhow!("hold must be UTF-8 integer"))?
                    .parse()?)
            };
            estimate_summary(
                replay,
                VelocityEstimatorConfig {
                    wheel_hold_us: integer(4)?,
                    imu_hold_us: integer(5)?,
                    wheel_scale: 1.0,
                    gyro_z_bias_rad_s: 0.0,
                    segment_origin_m: [0.0, 0.0],
                    segment_heading_rad: 0.0,
                    queue_capacity: 4096,
                },
            )?
        } else {
            replay_summary(replay)?
        };
        println!("{}", serde_json::to_string_pretty(&report)?);
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

fn estimate_summary(
    mut replay: NcltReplay,
    config: VelocityEstimatorConfig,
) -> Result<serde_json::Value> {
    let mut estimator =
        MeasuredVelocityEstimator::new(replay.origin_us(), replay.policy(), config)?;
    let mut hasher = Sha256::new();
    hasher.update(b"rne_nclt_velocity_estimates_v1\0");
    hasher.update(serde_json::to_vec(&(
        replay.origin_us(),
        replay.policy(),
        replay.source_sha256(),
        config,
    ))?);
    let mut boundaries = 0_u64;
    let mut segments = 0_u64;
    while let Some(time) = replay.next_delivery_time() {
        let estimate = estimator.update(&replay.advance_to(time)?)?;
        segments = segments.max(estimate.pose.map_or(0, |p| p.segment_id));
        let record = serde_json::to_vec(&estimate)?;
        hasher.update((record.len() as u64).to_le_bytes());
        hasher.update(record);
        boundaries += 1;
    }
    let observation = replay.observation();
    let last_capture = observation
        .wheels
        .as_ref()
        .map_or(0, |f| f.capture_time.ticks())
        .max(
            observation
                .imu
                .as_ref()
                .map_or(0, |f| f.capture_time.ticks()),
        );
    let lag = replay
        .policy()
        .wheel_delay_us
        .max(replay.policy().imu_delay_us)
        * 1000;
    let flush = last_capture
        .checked_add(lag)
        .ok_or_else(|| anyhow::anyhow!("terminal flush overflow"))?;
    let final_estimate =
        estimator.update(&replay.advance_to(rne_core::SimTime::from_ticks(flush))?)?;
    segments = segments.max(final_estimate.pose.map_or(0, |p| p.segment_id));
    let record = serde_json::to_vec(&final_estimate)?;
    hasher.update((record.len() as u64).to_le_bytes());
    hasher.update(record);
    Ok(serde_json::json!({
        "kind": "rne_nclt_velocity_estimate_audit", "schema_version": 1,
        "origin_us": replay.origin_us(), "policy": replay.policy(), "config": config,
        "source_sha256": replay.source_sha256(), "delivery_boundaries": boundaries,
        "observed_segment_count": segments, "final_estimate": final_estimate,
        "estimate_sha256": format!("{:x}", hasher.finalize()),
        "physical_accuracy_validated": false, "measured_latency_known": false,
        "calibration": "uncalibrated unit wheel scale and zero gyro bias; local segment origins only",
        "model": "source-frame planar no-slip wheel-average / gyro-z baseline"
    }))
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
    fn estimate_digest_repeats_binds_config_and_flushes_unequal_delays() {
        let run = |hold| {
            let entity = rne_ecs::spawn_named(&mut rne_ecs::World::new(), "source");
            estimate_summary(
                NcltReplay::new(
                    &b"1350000000000000,1,1\n1350000001000000,1,1\n"[..],
                    &b"1350000000000000,0,0,0,0,0,0,0,0,0\n"[..],
                    NcltReplayPolicy {
                        wheel_delay_us: 0,
                        imu_delay_us: 100_000,
                    },
                    entity,
                )
                .unwrap(),
                VelocityEstimatorConfig {
                    wheel_hold_us: hold,
                    imu_hold_us: hold,
                    wheel_scale: 1.0,
                    gyro_z_bias_rad_s: 0.0,
                    segment_origin_m: [0.0, 0.0],
                    segment_heading_rad: 0.0,
                    queue_capacity: 16,
                },
            )
            .unwrap()
        };
        let a = run(2_000_000);
        assert_eq!(a, run(2_000_000));
        assert_eq!(
            a["final_estimate"]["estimate_time_ticks"],
            1_000_000_000_u64
        );
        assert_eq!(
            a["final_estimate"]["decision_time_ticks"],
            1_100_000_000_u64
        );
        assert_eq!(a["final_estimate"]["pose"]["position_m"][0], 1.0);
        assert_eq!(a["final_estimate"]["unobserved_ticks"], 0);
        assert_ne!(a["estimate_sha256"], run(3_000_000)["estimate_sha256"]);
        let gap = run(200_000);
        assert_eq!(gap["final_estimate"]["unobserved_ticks"], 800_000_000_u64);
        assert!(gap["final_estimate"]["pose"].is_null());
    }

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
