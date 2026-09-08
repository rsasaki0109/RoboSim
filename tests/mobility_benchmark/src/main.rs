use anyhow::{bail, ensure, Context, Result};
use rne_mobility_benchmark::ackermann_observed::{
    run_ackermann_observed_failure_capsule, run_ackermann_observed_trace, AckermannObservedFault,
};
use rne_mobility_benchmark::ackermann_suspension::run_ackermann_suspension_trace;
use rne_mobility_benchmark::backend::run_backend_mobility_trace;
use rne_mobility_benchmark::diff_caster::run_differential_caster_trace;
#[cfg(feature = "mujoco")]
use rne_mobility_benchmark::identified_suspension_road::run_identified_suspension_road_evidence;
use rne_mobility_benchmark::mobility_randomization::{
    run_mobility_randomized_backend_trace, run_mobility_randomized_batch,
};
use rne_mobility_benchmark::observed::run_sensor_observed_trace;
use rne_mobility_benchmark::per_wheel::run_per_wheel_skid_trace;
use rne_mobility_benchmark::per_wheel_observed::{
    run_per_wheel_observed_failure_capsule, run_per_wheel_observed_trace, PerWheelObservedFault,
};
use rne_mobility_benchmark::road_excitation::run_road_excitation_trace;
use rne_mobility_benchmark::run_mobility_benchmark;
use rne_mobility_benchmark::suspension_acquisition::{
    decode_suspension_acquisition_manifest, MAX_SUSPENSION_ACQUISITION_MANIFEST_BYTES,
};
use rne_mobility_benchmark::suspension_identification::{
    decode_suspension_identification_dataset, identify_suspension_dataset,
    synthetic_suspension_identification_dataset, MAX_SUSPENSION_IDENTIFICATION_DATASET_BYTES,
};
use rne_physics_rapier::RapierBackend;
use std::path::PathBuf;

mod fixed_cli;

fn main() -> Result<()> {
    let mut args = std::env::args().skip(1);
    let mut output = None;
    let mut failure_replay = None;
    let mut fault = None;
    let mut input = None;
    let mut interval_tolerance_s = None;
    let mut acquisition_manifest = None;
    let mut evidence_root = None;
    let mut num_envs = None;
    let mut num_workers = None;
    let mut root_seed = None;
    let mut noise_root_seed = None;
    let mut episode_index = None;
    let mut lane_id = None;
    let mut backend = "analytic".to_string();
    while let Some(argument) = args.next() {
        match argument.as_str() {
            "--backend" => {
                backend = args.next().context("--backend requires a value")?;
            }
            "--output" => {
                output = Some(PathBuf::from(
                    args.next().context("--output requires a path")?,
                ));
            }
            "--failure-replay" => {
                failure_replay = Some(PathBuf::from(
                    args.next().context("--failure-replay requires a path")?,
                ));
            }
            "--fault" => {
                fault = Some(args.next().context("--fault requires a value")?);
            }
            "--input" => {
                input = Some(PathBuf::from(
                    args.next().context("--input requires a path")?,
                ));
            }
            "--interval-tolerance-s" => {
                let value = args
                    .next()
                    .context("--interval-tolerance-s requires a value")?
                    .parse::<f64>()
                    .context("interval tolerance must be a number")?;
                ensure!(
                    value.is_finite() && value >= 0.0,
                    "interval tolerance must be finite and nonnegative"
                );
                interval_tolerance_s = Some(value);
            }
            "--acquisition-manifest" => {
                acquisition_manifest = Some(PathBuf::from(
                    args.next()
                        .context("--acquisition-manifest requires a path")?,
                ));
            }
            "--evidence-root" => {
                evidence_root = Some(PathBuf::from(
                    args.next().context("--evidence-root requires a path")?,
                ));
            }
            "--num-envs" => {
                num_envs = Some(
                    args.next()
                        .context("--num-envs requires a value")?
                        .parse::<usize>()
                        .context("--num-envs must be an integer")?,
                );
            }
            "--seed" => {
                root_seed = Some(
                    args.next()
                        .context("--seed requires a value")?
                        .parse::<u64>()
                        .context("--seed must be an unsigned integer")?,
                );
            }
            "--workers" => {
                num_workers = Some(
                    args.next()
                        .context("--workers requires a value")?
                        .parse::<usize>()
                        .context("--workers must be an unsigned integer")?,
                );
            }
            "--noise-root-seed" => {
                ensure!(noise_root_seed.is_none(), "duplicate --noise-root-seed");
                noise_root_seed = Some(
                    args.next()
                        .context("--noise-root-seed requires a value")?
                        .parse::<u64>()
                        .context("--noise-root-seed must be an unsigned integer")?,
                );
            }
            "--episode-index" => {
                episode_index = Some(
                    args.next()
                        .context("--episode-index requires a value")?
                        .parse::<u64>()
                        .context("--episode-index must be an unsigned integer")?,
                );
            }
            "--lane-id" => {
                lane_id = Some(
                    args.next()
                        .context("--lane-id requires a value")?
                        .parse::<u64>()
                        .context("--lane-id must be an unsigned integer")?,
                );
            }
            other => bail!("unknown argument: {other}"),
        }
    }
    if backend.starts_with("fixed-") {
        ensure!(
            failure_replay.is_none()
                && fault.is_none()
                && acquisition_manifest.is_none()
                && evidence_root.is_none()
                && episode_index.is_none()
                && lane_id.is_none(),
            "fixed replay commands reject unrelated flags"
        );
        return fixed_cli::run(
            &backend,
            input
                .as_deref()
                .context("fixed replay command requires --input")?,
            output.as_deref(),
            root_seed,
            num_envs,
            num_workers.unwrap_or(1),
            noise_root_seed,
        );
    }
    ensure!(
        noise_root_seed.is_none(),
        "--noise-root-seed requires fixed replay recording"
    );
    let sensor_episode_batch = matches!(
        backend.as_str(),
        "sensor-episode-batch-rapier" | "sensor-episode-batch-mujoco"
    );
    ensure!(
        sensor_episode_batch || num_workers.is_none(),
        "--workers requires a sensor episode batch"
    );
    if backend.starts_with("sensor-capsule-") {
        ensure!(
            failure_replay.is_none()
                && fault.is_none()
                && acquisition_manifest.is_none()
                && evidence_root.is_none()
                && num_envs.is_none()
                && root_seed.is_none()
                && episode_index.is_none()
                && lane_id.is_none(),
            "capsule commands accept only --input and --output"
        );
        return run_sensor_capsule(
            &backend,
            input
                .as_deref()
                .context("capsule command requires --input")?,
            output.as_deref(),
        );
    }
    let (json, label) = match backend.as_str() {
        "analytic" => {
            let report = run_mobility_benchmark()?;
            ensure!(report.passed, "mobility benchmark verdict failed");
            (serde_json::to_string_pretty(&report)? + "\n", "analytic")
        }
        "rapier" => {
            let trace =
                run_backend_mobility_trace(RapierBackend::new(), RapierBackend::manifest())?;
            ensure!(trace.passed, "Rapier mobility benchmark verdict failed");
            (serde_json::to_string_pretty(&trace)? + "\n", "rapier")
        }
        "sensor-rapier" => {
            let trace = run_sensor_observed_trace(RapierBackend::new(), RapierBackend::manifest())?;
            ensure!(trace.passed, "Rapier sensor-observed verdict failed");
            (
                serde_json::to_string_pretty(&trace)? + "\n",
                "sensor-rapier",
            )
        }
        "skid-rapier" => {
            let trace = run_per_wheel_skid_trace(RapierBackend::new(), RapierBackend::manifest())?;
            ensure!(trace.passed, "Rapier per-wheel skid verdict failed");
            (serde_json::to_string_pretty(&trace)? + "\n", "skid-rapier")
        }
        "skid-sensor-rapier" => {
            let trace =
                run_per_wheel_observed_trace(RapierBackend::new(), RapierBackend::manifest())?;
            ensure!(trace.passed, "Rapier per-wheel sensor verdict failed");
            (
                serde_json::to_string_pretty(&trace)? + "\n",
                "skid-sensor-rapier",
            )
        }
        "skid-sensor-failure-rapier" => {
            let fault = parse_fatal_sensor_fault(
                fault
                    .as_deref()
                    .context("--backend skid-sensor-failure-rapier requires --fault")?,
            )?;
            let capsule = run_per_wheel_observed_failure_capsule(
                RapierBackend::new(),
                RapierBackend::manifest(),
                fault,
            )?;
            capsule.validate()?;
            (
                serde_json::to_string_pretty(&capsule)? + "\n",
                "skid-sensor-failure-rapier",
            )
        }
        "mujoco" => run_mujoco()?,
        "compare" => run_comparison(failure_replay.as_deref())?,
        "sensor-mujoco" => run_sensor_mujoco()?,
        "sensor-episode-batch-rapier" => {
            let report = rne_mobility_benchmark::observed_batch::run_sensor_episode_batch(
                || Ok((RapierBackend::new(), RapierBackend::manifest())),
                root_seed.context("sensor episode batch requires --seed")?,
                episode_index.unwrap_or(0),
                num_envs.context("sensor episode batch requires --num-envs")?,
                num_workers.unwrap_or(1),
            )?;
            eprintln!(
                "episode batch: failed_lanes={} total={}",
                report.failed_lanes,
                report.lanes.len()
            );
            (
                serde_json::to_string_pretty(&report)? + "\n",
                "sensor-episode-batch-rapier",
            )
        }
        "sensor-episode-batch-mujoco" => run_sensor_episode_batch_mujoco(
            root_seed.context("sensor episode batch requires --seed")?,
            episode_index.unwrap_or(0),
            num_envs.context("sensor episode batch requires --num-envs")?,
            num_workers.unwrap_or(1),
        )?,
        "sensor-replay-rapier" => {
            let source = rne_mobility_benchmark::observed::read_sensor_observed_trace(
                input.as_deref().context("sensor replay requires --input")?,
            )?;
            let replay = rne_mobility_benchmark::observed::replay_sensor_observed_trace(
                RapierBackend::new(),
                RapierBackend::manifest(),
                &source,
            )?;
            eprintln!("voltage replay exact; task_passed={}", replay.passed);
            (
                serde_json::to_string_pretty(&replay)? + "\n",
                "sensor-voltage-replay-rapier",
            )
        }
        "sensor-replay-mujoco" => {
            run_sensor_replay_mujoco(input.as_deref().context("sensor replay requires --input")?)?
        }
        "mobility-sensor-randomized-rapier" => {
            let trace = rne_mobility_benchmark::observed::run_randomized_mobility_sensor_trace(
                RapierBackend::new(),
                RapierBackend::manifest(),
                root_seed.context("joint reset requires --seed (episode seed)")?,
            )?;
            eprintln!("joint reset: task_passed={}", trace.passed);
            (
                serde_json::to_string_pretty(&trace)? + "\n",
                "mobility-sensor-randomized-rapier",
            )
        }
        "sensor-compare" => run_sensor_comparison(None, false)?,
        "sensor-randomized-compare" => run_sensor_comparison(
            Some(root_seed.context("sensor-randomized-compare requires --seed (episode seed)")?),
            false,
        )?,
        "mobility-sensor-randomized-compare" => run_sensor_comparison(
            Some(
                root_seed
                    .context("mobility-sensor-randomized-compare requires --seed (episode seed)")?,
            ),
            true,
        )?,
        "skid-mujoco" => run_skid_mujoco()?,
        "skid-compare" => run_skid_comparison()?,
        "skid-sensor-mujoco" => run_skid_sensor_mujoco()?,
        "skid-sensor-compare" => run_skid_sensor_comparison()?,
        "diff-caster-rapier" => {
            let trace =
                run_differential_caster_trace(RapierBackend::new(), RapierBackend::manifest())?;
            ensure!(trace.passed, "Rapier differential-caster verdict failed");
            (
                serde_json::to_string_pretty(&trace)? + "\n",
                "diff-caster-rapier",
            )
        }
        "diff-caster-mujoco" => run_diff_caster_mujoco()?,
        "diff-caster-compare" => run_diff_caster_comparison()?,
        "ackermann-suspension-rapier" => {
            let trace =
                run_ackermann_suspension_trace(RapierBackend::new(), RapierBackend::manifest())?;
            ensure!(
                trace.passed,
                "Rapier Ackermann-suspension verdict failed: {:#?}",
                trace.metrics
            );
            (
                serde_json::to_string_pretty(&trace)? + "\n",
                "ackermann-suspension-rapier",
            )
        }
        "ackermann-suspension-mujoco" => run_ackermann_suspension_mujoco()?,
        "ackermann-suspension-compare" => run_ackermann_suspension_comparison()?,
        "ackermann-sensor-rapier" => {
            let trace =
                run_ackermann_observed_trace(RapierBackend::new(), RapierBackend::manifest())?;
            ensure!(
                trace.passed,
                "Rapier sensor-only Ackermann verdict failed: {:#?}",
                trace.metrics
            );
            (
                serde_json::to_string_pretty(&trace)? + "\n",
                "ackermann-sensor-rapier",
            )
        }
        "ackermann-sensor-failure-rapier" => {
            let fault = parse_ackermann_fatal_sensor_fault(
                fault
                    .as_deref()
                    .context("--backend ackermann-sensor-failure-rapier requires --fault")?,
            )?;
            let capsule = run_ackermann_observed_failure_capsule(
                RapierBackend::new(),
                RapierBackend::manifest(),
                fault,
            )?;
            capsule.validate()?;
            (
                serde_json::to_string_pretty(&capsule)? + "\n",
                "ackermann-sensor-failure-rapier",
            )
        }
        "ackermann-sensor-mujoco" => run_ackermann_observed_mujoco()?,
        "ackermann-sensor-failure-mujoco" => {
            run_ackermann_observed_failure_mujoco(fault.as_deref())?
        }
        "ackermann-sensor-compare" => run_ackermann_observed_comparison()?,
        "road-excitation-rapier" => {
            let trace = run_road_excitation_trace(RapierBackend::new(), RapierBackend::manifest())?;
            ensure!(
                trace.passed,
                "Rapier road-excitation verdict failed: {:#?}",
                trace.metrics
            );
            (
                serde_json::to_string_pretty(&trace)? + "\n",
                "road-excitation-rapier",
            )
        }
        "road-excitation-mujoco" => run_road_excitation_mujoco()?,
        "road-excitation-compare" => run_road_excitation_comparison()?,
        "suspension-identification-fixture" => {
            let dataset = synthetic_suspension_identification_dataset()?;
            (
                serde_json::to_string_pretty(&dataset)? + "\n",
                "suspension-identification-fixture",
            )
        }
        "suspension-uncertainty"
        | "suspension-uncertainty-verify"
        | "suspension-derived-errors"
        | "suspension-derived-errors-verify" => {
            ensure!(interval_tolerance_s.is_none() && root_seed.is_none()
                && noise_root_seed.is_none() && episode_index.is_none() && lane_id.is_none()
                && num_envs.is_none() && num_workers.is_none() && acquisition_manifest.is_none()
                && failure_replay.is_none() && fault.is_none(),
                "uncertainty backend accepts only --backend, --input, --evidence-root and --output; assumptions must be embedded in the request");
            use rne_mobility_benchmark::suspension_runs::MAX_SUSPENSION_RUN_BYTES;
            use rne_mobility_benchmark::suspension_uncertainty::{
                decode_suspension_uncertainty, encode_suspension_uncertainty,
                SuspensionUncertaintyRequest,
            };
            use std::io::Read;
            let source = input
                .as_deref()
                .context("suspension uncertainty requires --input")?;
            let root = evidence_root
                .as_deref()
                .context("suspension uncertainty requires --evidence-root")?;
            let file = std::fs::File::open(source)?;
            ensure!(
                file.metadata()?.is_file(),
                "uncertainty input must be a regular file"
            );
            let mut bytes = Vec::new();
            file.take(MAX_SUSPENSION_RUN_BYTES as u64 + 1)
                .read_to_end(&mut bytes)?;
            ensure!(
                bytes.len() <= MAX_SUSPENSION_RUN_BYTES,
                "uncertainty input too large"
            );
            if matches!(
                backend.as_str(),
                "suspension-derived-errors" | "suspension-derived-errors-verify"
            ) {
                use rne_mobility_benchmark::suspension_derivative::{
                    decode_suspension_derived_errors, encode_suspension_derived_errors,
                    SuspensionDerivedErrorRequest,
                };
                let evidence = if backend == "suspension-derived-errors-verify" {
                    decode_suspension_derived_errors(&bytes, root)?
                } else {
                    let request: SuspensionDerivedErrorRequest = serde_json::from_slice(&bytes)?;
                    request.evaluate(root)?
                };
                (
                    String::from_utf8(encode_suspension_derived_errors(&evidence, root)?)?,
                    "suspension-derived-error-evidence",
                )
            } else {
                let evidence = if backend == "suspension-uncertainty-verify" {
                    decode_suspension_uncertainty(&bytes, root)?
                } else {
                    let request: SuspensionUncertaintyRequest = serde_json::from_slice(&bytes)?;
                    request.evaluate(root)?
                };
                (
                    String::from_utf8(encode_suspension_uncertainty(&evidence, root)?)?,
                    "suspension-uncertainty-evidence",
                )
            }
        }
        "suspension-acquired" | "suspension-acquired-verify" => {
            use rne_mobility_benchmark::suspension_runs::{
                decode_suspension_acquired_evidence, decode_suspension_acquired_request,
                encode_suspension_acquired_evidence, MAX_SUSPENSION_RUN_BYTES,
            };
            use std::io::Read;
            let input = input
                .as_deref()
                .context("acquired suspension requires --input")?;
            let root = evidence_root
                .as_deref()
                .context("acquired suspension requires --evidence-root")?;
            let file = std::fs::File::open(input)?;
            ensure!(
                file.metadata()?.is_file(),
                "acquired input must be a regular file"
            );
            let mut bytes = Vec::new();
            file.take(MAX_SUSPENSION_RUN_BYTES as u64 + 1)
                .read_to_end(&mut bytes)?;
            let evidence = if backend == "suspension-acquired-verify" {
                ensure!(
                    interval_tolerance_s.is_none(),
                    "verification uses the embedded tolerance; do not override it"
                );
                decode_suspension_acquired_evidence(&bytes, root)?
            } else {
                decode_suspension_acquired_request(&bytes)?.identify(
                    root,
                    interval_tolerance_s.context(
                        "acquired suspension requires --interval-tolerance-s from clock evidence",
                    )?,
                )?
            };
            (
                String::from_utf8(encode_suspension_acquired_evidence(&evidence, root)?)?,
                "suspension-acquired-evidence",
            )
        }
        "suspension-timing" | "suspension-timing-verify" => {
            use rne_mobility_benchmark::suspension_runs::{
                decode_suspension_run_request, decode_suspension_timing, encode_suspension_timing,
                identify_suspension_timing, MAX_SUSPENSION_RUN_BYTES,
            };
            use std::io::Read;
            let input = input
                .as_deref()
                .context("suspension timing requires --input")?;
            let file = std::fs::File::open(input)?;
            ensure!(
                file.metadata()?.is_file(),
                "timing input must be a regular file"
            );
            let mut bytes = Vec::new();
            file.take(MAX_SUSPENSION_RUN_BYTES as u64 + 1)
                .read_to_end(&mut bytes)?;
            let evidence = if backend == "suspension-timing-verify" {
                ensure!(
                    interval_tolerance_s.is_none(),
                    "verification uses the embedded tolerance; do not override it"
                );
                decode_suspension_timing(&bytes)?
            } else {
                identify_suspension_timing(
                    &decode_suspension_run_request(&bytes)?,
                    interval_tolerance_s.context(
                        "suspension timing requires --interval-tolerance-s from clock evidence",
                    )?,
                )?
            };
            (
                String::from_utf8(encode_suspension_timing(&evidence)?)?,
                "suspension-timing-evidence",
            )
        }
        "suspension-influence" | "suspension-influence-verify" => {
            use rne_mobility_benchmark::suspension_runs::{
                decode_suspension_influence, decode_suspension_run_request,
                encode_suspension_influence, identify_suspension_influence,
                MAX_SUSPENSION_RUN_BYTES,
            };
            use std::io::Read;
            let input = input
                .as_deref()
                .context("suspension influence requires --input")?;
            let file = std::fs::File::open(input)?;
            ensure!(
                file.metadata()?.is_file(),
                "influence input must be a regular file"
            );
            let mut bytes = Vec::new();
            file.take(MAX_SUSPENSION_RUN_BYTES as u64 + 1)
                .read_to_end(&mut bytes)?;
            let evidence = if backend == "suspension-influence-verify" {
                decode_suspension_influence(&bytes)?
            } else {
                identify_suspension_influence(&decode_suspension_run_request(&bytes)?)?
            };
            (
                String::from_utf8(encode_suspension_influence(&evidence)?)?,
                "suspension-influence-evidence",
            )
        }
        "suspension-excitation" | "suspension-excitation-verify" => {
            use rne_mobility_benchmark::suspension_runs::{
                decode_suspension_excitation, decode_suspension_run_request,
                encode_suspension_excitation, identify_suspension_excitation,
                MAX_SUSPENSION_RUN_BYTES,
            };
            use std::io::Read;
            let input = input
                .as_deref()
                .context("suspension excitation requires --input")?;
            let file = std::fs::File::open(input)?;
            ensure!(
                file.metadata()?.is_file(),
                "excitation input must be a regular file"
            );
            let mut bytes = Vec::new();
            file.take(MAX_SUSPENSION_RUN_BYTES as u64 + 1)
                .read_to_end(&mut bytes)?;
            let evidence = if backend == "suspension-excitation-verify" {
                decode_suspension_excitation(&bytes)?
            } else {
                identify_suspension_excitation(&decode_suspension_run_request(&bytes)?)?
            };
            (
                String::from_utf8(encode_suspension_excitation(&evidence)?)?,
                "suspension-excitation-evidence",
            )
        }
        "suspension-run-identification" | "suspension-run-verify" => {
            use rne_mobility_benchmark::suspension_runs::{
                decode_suspension_run_evidence, decode_suspension_run_request,
                encode_suspension_run_evidence, identify_suspension_runs, MAX_SUSPENSION_RUN_BYTES,
            };
            use std::io::Read;
            let input = input
                .as_deref()
                .context("suspension run operation requires --input")?;
            let file = std::fs::File::open(input)?;
            ensure!(
                file.metadata()?.is_file(),
                "run input must be a regular file"
            );
            let mut bytes = Vec::new();
            file.take(MAX_SUSPENSION_RUN_BYTES as u64 + 1)
                .read_to_end(&mut bytes)?;
            let evidence = if backend == "suspension-run-verify" {
                decode_suspension_run_evidence(&bytes)?
            } else {
                identify_suspension_runs(&decode_suspension_run_request(&bytes)?)?
            };
            (
                String::from_utf8(encode_suspension_run_evidence(&evidence)?)?,
                "suspension-run-evidence",
            )
        }
        "suspension-identification" => {
            let input = input
                .as_deref()
                .context("--backend suspension-identification requires --input")?;
            let dataset = read_suspension_identification_dataset(input)?;
            let evidence = identify_suspension_dataset(&dataset)?;
            (
                serde_json::to_string_pretty(&evidence)? + "\n",
                "suspension-identification",
            )
        }
        "identified-road-compare" => {
            let input = input
                .as_deref()
                .context("--backend identified-road-compare requires --input")?;
            run_identified_road_comparison(input)?
        }
        "suspension-acquisition-verify" => {
            let input = input
                .as_deref()
                .context("--backend suspension-acquisition-verify requires --input")?;
            let manifest_path = acquisition_manifest.as_deref().context(
                "--backend suspension-acquisition-verify requires --acquisition-manifest",
            )?;
            let evidence_root = evidence_root
                .as_deref()
                .context("--backend suspension-acquisition-verify requires --evidence-root")?;
            let dataset = read_suspension_identification_dataset(input)?;
            use std::io::Read;
            let file = std::fs::File::open(manifest_path)
                .with_context(|| format!("open {}", manifest_path.display()))?;
            let metadata = file
                .metadata()
                .with_context(|| format!("inspect {}", manifest_path.display()))?;
            ensure!(
                metadata.is_file()
                    && metadata.len() <= MAX_SUSPENSION_ACQUISITION_MANIFEST_BYTES as u64,
                "suspension acquisition manifest is not a bounded regular file"
            );
            let mut bytes = Vec::new();
            file.take(MAX_SUSPENSION_ACQUISITION_MANIFEST_BYTES as u64 + 1)
                .read_to_end(&mut bytes)
                .with_context(|| format!("read {}", manifest_path.display()))?;
            let manifest = decode_suspension_acquisition_manifest(&bytes, &dataset)?;
            manifest.verify_files(&dataset, evidence_root)?;
            (
                serde_json::to_string_pretty(&manifest)? + "\n",
                "suspension-acquisition-verified",
            )
        }
        "mobility-randomized-batch" => {
            let report = run_mobility_randomized_batch(
                root_seed.context("--backend mobility-randomized-batch requires --seed")?,
                episode_index.unwrap_or(0),
                num_envs.context("--backend mobility-randomized-batch requires --num-envs")?,
            )?;
            ensure!(report.passed, "randomized Mobility batch failed");
            (
                serde_json::to_string_pretty(&report)? + "\n",
                "mobility-randomized-batch",
            )
        }
        "mobility-randomized-backend-rapier" => {
            let trace = run_mobility_randomized_backend_trace(
                RapierBackend::new(),
                RapierBackend::manifest(),
                root_seed
                    .context("--backend mobility-randomized-backend-rapier requires --seed")?,
                lane_id.unwrap_or(0),
                episode_index.unwrap_or(0),
            )?;
            ensure!(
                trace.trace.passed,
                "randomized Rapier Mobility trace failed"
            );
            (
                serde_json::to_string_pretty(&trace)? + "\n",
                "mobility-randomized-backend-rapier",
            )
        }
        "mobility-randomized-backend-mujoco" => run_mobility_randomized_backend_mujoco(
            root_seed.context("--backend mobility-randomized-backend-mujoco requires --seed")?,
            lane_id.unwrap_or(0),
            episode_index.unwrap_or(0),
        )?,
        "mobility-randomized-backend-compare" => run_mobility_randomized_backend_comparison(
            root_seed.context("--backend mobility-randomized-backend-compare requires --seed")?,
            lane_id.unwrap_or(0),
            episode_index.unwrap_or(0),
        )?,
        other => bail!("unknown backend: {other}"),
    };
    ensure!(
        backend == "compare" || failure_replay.is_none(),
        "--failure-replay is valid only with --backend compare"
    );
    ensure!(
        matches!(
            backend.as_str(),
            "skid-sensor-failure-rapier"
                | "ackermann-sensor-failure-rapier"
                | "ackermann-sensor-failure-mujoco"
        ) || fault.is_none(),
        "--fault is valid only with a sensor-failure backend"
    );
    ensure!(
        matches!(
            backend.as_str(),
            "suspension-identification"
                | "identified-road-compare"
                | "suspension-acquisition-verify"
                | "suspension-acquired"
                | "suspension-uncertainty"
                | "suspension-uncertainty-verify"
                | "suspension-derived-errors"
                | "suspension-derived-errors-verify"
                | "suspension-acquired-verify"
                | "suspension-timing"
                | "suspension-timing-verify"
                | "suspension-excitation"
                | "suspension-influence"
                | "suspension-influence-verify"
                | "suspension-excitation-verify"
                | "suspension-run-identification"
                | "suspension-run-verify"
                | "sensor-replay-rapier"
                | "sensor-replay-mujoco"
        ) || input.is_none(),
        "--input requires an identification or sensor-replay backend"
    );
    ensure!(
        backend == "suspension-acquisition-verify" || acquisition_manifest.is_none(),
        "--acquisition-manifest is valid only with suspension-acquisition-verify"
    );
    ensure!(
        matches!(
            backend.as_str(),
            "suspension-acquisition-verify"
                | "suspension-acquired"
                | "suspension-acquired-verify"
                | "suspension-uncertainty"
                | "suspension-uncertainty-verify"
                | "suspension-derived-errors"
                | "suspension-derived-errors-verify"
        ) || evidence_root.is_none(),
        "--evidence-root requires an acquisition verification or acquired suspension backend"
    );
    let randomized_backend = matches!(
        backend.as_str(),
        "mobility-randomized-backend-rapier"
            | "mobility-randomized-backend-mujoco"
            | "mobility-randomized-backend-compare"
    );
    ensure!(
        !matches!(
            backend.as_str(),
            "sensor-randomized-compare"
                | "mobility-sensor-randomized-compare"
                | "mobility-sensor-randomized-rapier"
        ) || episode_index.is_none(),
        "sensor reset comparison takes an episode seed directly, without --episode-index"
    );
    ensure!(
        backend == "mobility-randomized-batch" || sensor_episode_batch || num_envs.is_none(),
        "--num-envs requires a batch backend"
    );
    ensure!(
        backend == "mobility-randomized-batch"
            || sensor_episode_batch
            || backend == "sensor-randomized-compare"
            || backend == "mobility-sensor-randomized-compare"
            || backend == "mobility-sensor-randomized-rapier"
            || randomized_backend
            || (root_seed.is_none() && episode_index.is_none()),
        "--seed and --episode-index require a randomized Mobility backend"
    );
    ensure!(
        randomized_backend || lane_id.is_none(),
        "--lane-id requires a randomized physics backend"
    );
    if let Some(path) = output {
        if let Some(parent) = path.parent() {
            std::fs::create_dir_all(parent)
                .with_context(|| format!("create {}", parent.display()))?;
        }
        std::fs::write(&path, json).with_context(|| format!("write {}", path.display()))?;
        println!("{label} mobility evidence written: {}", path.display());
    } else {
        print!("{json}");
    }
    Ok(())
}

fn run_sensor_capsule(
    mode: &str,
    input: &std::path::Path,
    output: Option<&std::path::Path>,
) -> Result<()> {
    match mode {
        "sensor-capsule-create-rapier" | "sensor-capsule-verify-rapier" => capsule_operation(
            RapierBackend::new(),
            RapierBackend::manifest(),
            mode == "sensor-capsule-create-rapier",
            input,
            output,
        ),
        #[cfg(feature = "mujoco")]
        "sensor-capsule-create-mujoco" | "sensor-capsule-verify-mujoco" => {
            use rne_physics_mujoco::MuJoCoBackend;
            capsule_operation(
                MuJoCoBackend::new(rne_core::SimDuration::from_ticks(
                    rne_mobility_benchmark::observed::SENSOR_OBSERVED_FIXED_DELTA_TICKS,
                ))?,
                MuJoCoBackend::manifest(),
                mode == "sensor-capsule-create-mujoco",
                input,
                output,
            )
        }
        _ => bail!("unsupported capsule command {mode}; MuJoCo commands require --features mujoco"),
    }
}

fn capsule_operation<B: rne_physics::PhysicsBackend>(
    backend: B,
    manifest: rne_physics::PhysicsBackendManifest,
    create: bool,
    input: &std::path::Path,
    output: Option<&std::path::Path>,
) -> Result<()> {
    use rne_mobility_benchmark::observed_capsule::{
        compiled_capsule_build, SensorObservedFailureBundle,
    };
    let build = compiled_capsule_build()?;
    if create {
        let destination = output.context("capsule creation requires --output (new directory)")?;
        let source = rne_mobility_benchmark::observed::read_sensor_observed_trace(input)?;
        let bundle = SensorObservedFailureBundle::create(backend, manifest, &source, build)?;
        bundle.write_new(destination)?;
        println!(
            "verified failed voltage replay packaged: {}",
            destination.display()
        );
    } else {
        ensure!(
            output.is_none(),
            "capsule verification does not accept --output"
        );
        let bundle = SensorObservedFailureBundle::read(input)?;
        bundle.verify_replay(backend, manifest, &build)?;
        println!(
            "failure capsule replay verified; task remains failed: {}",
            input.display()
        );
    }
    Ok(())
}

fn read_suspension_identification_dataset(
    input: &std::path::Path,
) -> Result<rne_mobility_benchmark::suspension_identification::SuspensionIdentificationDataset> {
    let metadata =
        std::fs::metadata(input).with_context(|| format!("inspect {}", input.display()))?;
    ensure!(
        metadata.is_file() && metadata.len() <= MAX_SUSPENSION_IDENTIFICATION_DATASET_BYTES as u64,
        "suspension identification input is not a bounded regular file"
    );
    let bytes = std::fs::read(input).with_context(|| format!("read {}", input.display()))?;
    decode_suspension_identification_dataset(&bytes)
}

#[cfg(feature = "mujoco")]
fn run_identified_road_comparison(input: &std::path::Path) -> Result<(String, &'static str)> {
    use rne_core::SimDuration;
    use rne_mobility_benchmark::road_excitation::ROAD_EXCITATION_FIXED_DELTA_TICKS;
    use rne_physics_mujoco::MuJoCoBackend;

    let dataset = read_suspension_identification_dataset(input)?;
    let evidence = run_identified_suspension_road_evidence(
        dataset,
        RapierBackend::new(),
        RapierBackend::manifest(),
        MuJoCoBackend::new(SimDuration::from_ticks(ROAD_EXCITATION_FIXED_DELTA_TICKS))?,
        MuJoCoBackend::manifest(),
    )?;
    ensure!(
        evidence.passed,
        "identified suspension road comparison failed: {:#?}",
        evidence.road_comparison.metrics
    );
    Ok((
        serde_json::to_string_pretty(&evidence)? + "\n",
        "identified-suspension-road-rapier-vs-mujoco",
    ))
}

#[cfg(not(feature = "mujoco"))]
fn run_identified_road_comparison(_input: &std::path::Path) -> Result<(String, &'static str)> {
    bail!("identified suspension road comparison requires --features mujoco")
}

#[cfg(feature = "mujoco")]
fn run_ackermann_observed_mujoco() -> Result<(String, &'static str)> {
    use rne_core::SimDuration;
    use rne_mobility_benchmark::ackermann_observed::ACKERMANN_OBSERVED_FIXED_DELTA_TICKS;
    use rne_physics_mujoco::MuJoCoBackend;

    let trace = run_ackermann_observed_trace(
        MuJoCoBackend::new(SimDuration::from_ticks(
            ACKERMANN_OBSERVED_FIXED_DELTA_TICKS,
        ))?,
        MuJoCoBackend::manifest(),
    )?;
    ensure!(
        trace.passed,
        "MuJoCo sensor-only Ackermann verdict failed: {:#?}",
        trace.metrics
    );
    Ok((
        serde_json::to_string_pretty(&trace)? + "\n",
        "ackermann-sensor-mujoco",
    ))
}

#[cfg(feature = "mujoco")]
fn run_ackermann_observed_failure_mujoco(fault: Option<&str>) -> Result<(String, &'static str)> {
    use rne_core::SimDuration;
    use rne_mobility_benchmark::ackermann_observed::ACKERMANN_OBSERVED_FIXED_DELTA_TICKS;
    use rne_physics_mujoco::MuJoCoBackend;

    let fault = parse_ackermann_fatal_sensor_fault(
        fault.context("--backend ackermann-sensor-failure-mujoco requires --fault")?,
    )?;
    let capsule = run_ackermann_observed_failure_capsule(
        MuJoCoBackend::new(SimDuration::from_ticks(
            ACKERMANN_OBSERVED_FIXED_DELTA_TICKS,
        ))?,
        MuJoCoBackend::manifest(),
        fault,
    )?;
    capsule.validate()?;
    Ok((
        serde_json::to_string_pretty(&capsule)? + "\n",
        "ackermann-sensor-failure-mujoco",
    ))
}

#[cfg(feature = "mujoco")]
fn run_ackermann_observed_comparison() -> Result<(String, &'static str)> {
    use rne_core::SimDuration;
    use rne_mobility_benchmark::ackermann_observed::{
        compare_ackermann_observed_traces, ACKERMANN_OBSERVED_FIXED_DELTA_TICKS,
    };
    use rne_physics_mujoco::MuJoCoBackend;

    let rapier = run_ackermann_observed_trace(RapierBackend::new(), RapierBackend::manifest())?;
    let mujoco = run_ackermann_observed_trace(
        MuJoCoBackend::new(SimDuration::from_ticks(
            ACKERMANN_OBSERVED_FIXED_DELTA_TICKS,
        ))?,
        MuJoCoBackend::manifest(),
    )?;
    let comparison = compare_ackermann_observed_traces(rapier, mujoco)?;
    ensure!(
        comparison.passed,
        "sensor-only Ackermann comparison failed: {:#?}",
        comparison.metrics
    );
    Ok((
        serde_json::to_string_pretty(&comparison)? + "\n",
        "ackermann-sensor-rapier-vs-mujoco",
    ))
}

#[cfg(feature = "mujoco")]
fn run_road_excitation_mujoco() -> Result<(String, &'static str)> {
    use rne_core::SimDuration;
    use rne_mobility_benchmark::road_excitation::ROAD_EXCITATION_FIXED_DELTA_TICKS;
    use rne_physics_mujoco::MuJoCoBackend;

    let trace = run_road_excitation_trace(
        MuJoCoBackend::new(SimDuration::from_ticks(ROAD_EXCITATION_FIXED_DELTA_TICKS))?,
        MuJoCoBackend::manifest(),
    )?;
    ensure!(
        trace.passed,
        "MuJoCo road-excitation verdict failed: {:#?}",
        trace.metrics
    );
    Ok((
        serde_json::to_string_pretty(&trace)? + "\n",
        "road-excitation-mujoco",
    ))
}

#[cfg(feature = "mujoco")]
fn run_road_excitation_comparison() -> Result<(String, &'static str)> {
    use rne_core::SimDuration;
    use rne_mobility_benchmark::road_excitation::{
        compare_road_excitation_traces, ROAD_EXCITATION_FIXED_DELTA_TICKS,
    };
    use rne_physics_mujoco::MuJoCoBackend;

    let rapier = run_road_excitation_trace(RapierBackend::new(), RapierBackend::manifest())?;
    let mujoco = run_road_excitation_trace(
        MuJoCoBackend::new(SimDuration::from_ticks(ROAD_EXCITATION_FIXED_DELTA_TICKS))?,
        MuJoCoBackend::manifest(),
    )?;
    let comparison = compare_road_excitation_traces(rapier, mujoco)?;
    ensure!(
        comparison.passed,
        "road-excitation comparison failed: {:#?}",
        comparison.metrics
    );
    Ok((
        serde_json::to_string_pretty(&comparison)? + "\n",
        "road-excitation-rapier-vs-mujoco",
    ))
}

#[cfg(not(feature = "mujoco"))]
fn run_ackermann_observed_mujoco() -> Result<(String, &'static str)> {
    bail!("sensor-only Ackermann mujoco requires --features mujoco")
}

#[cfg(not(feature = "mujoco"))]
fn run_ackermann_observed_failure_mujoco(_fault: Option<&str>) -> Result<(String, &'static str)> {
    bail!("sensor-only Ackermann failure mujoco requires --features mujoco")
}

#[cfg(not(feature = "mujoco"))]
fn run_ackermann_observed_comparison() -> Result<(String, &'static str)> {
    bail!("sensor-only Ackermann comparison requires --features mujoco")
}

#[cfg(not(feature = "mujoco"))]
fn run_road_excitation_mujoco() -> Result<(String, &'static str)> {
    bail!("road-excitation mujoco requires --features mujoco")
}

#[cfg(not(feature = "mujoco"))]
fn run_road_excitation_comparison() -> Result<(String, &'static str)> {
    bail!("road-excitation comparison requires --features mujoco")
}

#[cfg(feature = "mujoco")]
fn run_ackermann_suspension_mujoco() -> Result<(String, &'static str)> {
    use rne_core::SimDuration;
    use rne_mobility_benchmark::ackermann_suspension::ACKERMANN_SUSPENSION_FIXED_DELTA_TICKS;
    use rne_physics_mujoco::MuJoCoBackend;

    let trace = run_ackermann_suspension_trace(
        MuJoCoBackend::new(SimDuration::from_ticks(
            ACKERMANN_SUSPENSION_FIXED_DELTA_TICKS,
        ))?,
        MuJoCoBackend::manifest(),
    )?;
    ensure!(
        trace.passed,
        "MuJoCo Ackermann-suspension verdict failed: {:#?}",
        trace.metrics
    );
    Ok((
        serde_json::to_string_pretty(&trace)? + "\n",
        "ackermann-suspension-mujoco",
    ))
}

#[cfg(feature = "mujoco")]
fn run_ackermann_suspension_comparison() -> Result<(String, &'static str)> {
    use rne_core::SimDuration;
    use rne_mobility_benchmark::ackermann_suspension::{
        compare_ackermann_suspension_traces, ACKERMANN_SUSPENSION_FIXED_DELTA_TICKS,
    };
    use rne_physics_mujoco::MuJoCoBackend;

    let rapier = run_ackermann_suspension_trace(RapierBackend::new(), RapierBackend::manifest())?;
    let mujoco = run_ackermann_suspension_trace(
        MuJoCoBackend::new(SimDuration::from_ticks(
            ACKERMANN_SUSPENSION_FIXED_DELTA_TICKS,
        ))?,
        MuJoCoBackend::manifest(),
    )?;
    let comparison = compare_ackermann_suspension_traces(rapier, mujoco)?;
    ensure!(comparison.passed, "Ackermann-suspension comparison failed");
    Ok((
        serde_json::to_string_pretty(&comparison)? + "\n",
        "ackermann-suspension-rapier-vs-mujoco",
    ))
}

#[cfg(not(feature = "mujoco"))]
fn run_ackermann_suspension_mujoco() -> Result<(String, &'static str)> {
    bail!("Ackermann-suspension mujoco requires --features mujoco")
}

#[cfg(not(feature = "mujoco"))]
fn run_ackermann_suspension_comparison() -> Result<(String, &'static str)> {
    bail!("Ackermann-suspension comparison requires --features mujoco")
}

#[cfg(feature = "mujoco")]
fn run_diff_caster_mujoco() -> Result<(String, &'static str)> {
    use rne_core::SimDuration;
    use rne_mobility_benchmark::diff_caster::DIFF_CASTER_FIXED_DELTA_TICKS;
    use rne_physics_mujoco::MuJoCoBackend;

    let trace = run_differential_caster_trace(
        MuJoCoBackend::new(SimDuration::from_ticks(DIFF_CASTER_FIXED_DELTA_TICKS))?,
        MuJoCoBackend::manifest(),
    )?;
    ensure!(trace.passed, "MuJoCo differential-caster verdict failed");
    Ok((
        serde_json::to_string_pretty(&trace)? + "\n",
        "diff-caster-mujoco",
    ))
}

#[cfg(feature = "mujoco")]
fn run_diff_caster_comparison() -> Result<(String, &'static str)> {
    use rne_core::SimDuration;
    use rne_mobility_benchmark::diff_caster::{
        compare_differential_caster_traces, DIFF_CASTER_FIXED_DELTA_TICKS,
    };
    use rne_physics_mujoco::MuJoCoBackend;

    let rapier = run_differential_caster_trace(RapierBackend::new(), RapierBackend::manifest())?;
    let mujoco = run_differential_caster_trace(
        MuJoCoBackend::new(SimDuration::from_ticks(DIFF_CASTER_FIXED_DELTA_TICKS))?,
        MuJoCoBackend::manifest(),
    )?;
    let comparison = compare_differential_caster_traces(rapier, mujoco)?;
    ensure!(comparison.passed, "differential-caster comparison failed");
    Ok((
        serde_json::to_string_pretty(&comparison)? + "\n",
        "diff-caster-rapier-vs-mujoco",
    ))
}

#[cfg(not(feature = "mujoco"))]
fn run_diff_caster_mujoco() -> Result<(String, &'static str)> {
    bail!("differential-caster mujoco requires --features mujoco")
}

#[cfg(not(feature = "mujoco"))]
fn run_diff_caster_comparison() -> Result<(String, &'static str)> {
    bail!("differential-caster comparison requires --features mujoco")
}

fn parse_fatal_sensor_fault(value: &str) -> Result<PerWheelObservedFault> {
    match value {
        "encoder-stuck" => Ok(PerWheelObservedFault::FrontLeftEncoderStuck { sequence: 30 }),
        "encoder-saturated" => Ok(PerWheelObservedFault::FrontLeftEncoderSaturate {
            counter_bits: 4,
        }),
        "imu-stuck" => Ok(PerWheelObservedFault::ImuStuck { sequence: 30 }),
        other => bail!(
            "unknown fatal sensor fault {other}; expected encoder-stuck, encoder-saturated, or imu-stuck"
        ),
    }
}

fn parse_ackermann_fatal_sensor_fault(value: &str) -> Result<AckermannObservedFault> {
    match value {
        "wheel-stuck" => Ok(AckermannObservedFault::FrontLeftWheelStuck { sequence: 200 }),
        "wheel-saturated" => Ok(AckermannObservedFault::FrontLeftWheelSaturate {
            counter_bits: 4,
        }),
        "steering-stuck" => Ok(AckermannObservedFault::FrontRightSteeringStuck {
            sequence: 200,
        }),
        "imu-stuck" => Ok(AckermannObservedFault::ImuStuck { sequence: 200 }),
        other => bail!(
            "unknown fatal Ackermann sensor fault {other}; expected wheel-stuck, wheel-saturated, steering-stuck, or imu-stuck"
        ),
    }
}

#[cfg(feature = "mujoco")]
fn run_skid_sensor_mujoco() -> Result<(String, &'static str)> {
    use rne_core::SimDuration;
    use rne_mobility_benchmark::per_wheel_observed::PER_WHEEL_OBSERVED_FIXED_DELTA_TICKS;
    use rne_physics_mujoco::MuJoCoBackend;

    let trace = run_per_wheel_observed_trace(
        MuJoCoBackend::new(SimDuration::from_ticks(
            PER_WHEEL_OBSERVED_FIXED_DELTA_TICKS,
        ))?,
        MuJoCoBackend::manifest(),
    )?;
    ensure!(trace.passed, "MuJoCo per-wheel sensor verdict failed");
    Ok((
        serde_json::to_string_pretty(&trace)? + "\n",
        "skid-sensor-mujoco",
    ))
}

#[cfg(feature = "mujoco")]
fn run_skid_sensor_comparison() -> Result<(String, &'static str)> {
    use rne_core::SimDuration;
    use rne_mobility_benchmark::per_wheel_observed::{
        compare_per_wheel_observed_traces, PER_WHEEL_OBSERVED_FIXED_DELTA_TICKS,
    };
    use rne_physics_mujoco::MuJoCoBackend;

    let rapier = run_per_wheel_observed_trace(RapierBackend::new(), RapierBackend::manifest())?;
    let mujoco = run_per_wheel_observed_trace(
        MuJoCoBackend::new(SimDuration::from_ticks(
            PER_WHEEL_OBSERVED_FIXED_DELTA_TICKS,
        ))?,
        MuJoCoBackend::manifest(),
    )?;
    let comparison = compare_per_wheel_observed_traces(rapier, mujoco)?;
    ensure!(comparison.passed, "per-wheel sensor comparison failed");
    Ok((
        serde_json::to_string_pretty(&comparison)? + "\n",
        "skid-sensor-rapier-vs-mujoco",
    ))
}

#[cfg(not(feature = "mujoco"))]
fn run_skid_sensor_mujoco() -> Result<(String, &'static str)> {
    bail!("per-wheel sensor mujoco requires --features mujoco")
}

#[cfg(not(feature = "mujoco"))]
fn run_skid_sensor_comparison() -> Result<(String, &'static str)> {
    bail!("per-wheel sensor comparison requires --features mujoco")
}

#[cfg(feature = "mujoco")]
fn run_skid_mujoco() -> Result<(String, &'static str)> {
    use rne_core::SimDuration;
    use rne_mobility_benchmark::per_wheel::PER_WHEEL_SKID_FIXED_DELTA_TICKS;
    use rne_physics_mujoco::MuJoCoBackend;

    let backend = MuJoCoBackend::new(SimDuration::from_ticks(PER_WHEEL_SKID_FIXED_DELTA_TICKS))?;
    let trace = run_per_wheel_skid_trace(backend, MuJoCoBackend::manifest())?;
    ensure!(trace.passed, "MuJoCo per-wheel skid verdict failed");
    Ok((serde_json::to_string_pretty(&trace)? + "\n", "skid-mujoco"))
}

#[cfg(feature = "mujoco")]
fn run_skid_comparison() -> Result<(String, &'static str)> {
    use rne_core::SimDuration;
    use rne_mobility_benchmark::per_wheel::{
        compare_per_wheel_skid_traces, PER_WHEEL_SKID_FIXED_DELTA_TICKS,
    };
    use rne_physics_mujoco::MuJoCoBackend;

    let rapier = run_per_wheel_skid_trace(RapierBackend::new(), RapierBackend::manifest())?;
    let mujoco = run_per_wheel_skid_trace(
        MuJoCoBackend::new(SimDuration::from_ticks(PER_WHEEL_SKID_FIXED_DELTA_TICKS))?,
        MuJoCoBackend::manifest(),
    )?;
    let comparison = compare_per_wheel_skid_traces(rapier, mujoco)?;
    ensure!(comparison.passed, "per-wheel skid comparison failed");
    Ok((
        serde_json::to_string_pretty(&comparison)? + "\n",
        "skid-rapier-vs-mujoco",
    ))
}

#[cfg(not(feature = "mujoco"))]
fn run_skid_mujoco() -> Result<(String, &'static str)> {
    bail!("per-wheel skid mujoco backend requires --features mujoco")
}

#[cfg(not(feature = "mujoco"))]
fn run_skid_comparison() -> Result<(String, &'static str)> {
    bail!("per-wheel skid comparison requires --features mujoco")
}

#[cfg(feature = "mujoco")]
fn run_sensor_mujoco() -> Result<(String, &'static str)> {
    use rne_core::SimDuration;
    use rne_mobility_benchmark::observed::SENSOR_OBSERVED_FIXED_DELTA_TICKS;
    use rne_physics_mujoco::MuJoCoBackend;

    let backend = MuJoCoBackend::new(SimDuration::from_ticks(SENSOR_OBSERVED_FIXED_DELTA_TICKS))?;
    let trace = run_sensor_observed_trace(backend, MuJoCoBackend::manifest())?;
    ensure!(trace.passed, "MuJoCo sensor-observed verdict failed");
    Ok((
        serde_json::to_string_pretty(&trace)? + "\n",
        "sensor-mujoco",
    ))
}

#[cfg(feature = "mujoco")]
fn run_sensor_comparison(
    episode_seed: Option<u64>,
    physical_reset: bool,
) -> Result<(String, &'static str)> {
    use rne_core::SimDuration;
    use rne_mobility_benchmark::observed::{
        compare_sensor_observed_traces, run_randomized_mobility_sensor_trace,
        run_randomized_sensor_observed_trace, SENSOR_OBSERVED_FIXED_DELTA_TICKS,
    };
    use rne_physics_mujoco::MuJoCoBackend;

    let mujoco_backend =
        MuJoCoBackend::new(SimDuration::from_ticks(SENSOR_OBSERVED_FIXED_DELTA_TICKS))?;
    let (rapier, mujoco) = if let Some(seed) = episode_seed {
        if physical_reset {
            (
                run_randomized_mobility_sensor_trace(
                    RapierBackend::new(),
                    RapierBackend::manifest(),
                    seed,
                )?,
                run_randomized_mobility_sensor_trace(
                    mujoco_backend,
                    MuJoCoBackend::manifest(),
                    seed,
                )?,
            )
        } else {
            (
                run_randomized_sensor_observed_trace(
                    RapierBackend::new(),
                    RapierBackend::manifest(),
                    seed,
                )?,
                run_randomized_sensor_observed_trace(
                    mujoco_backend,
                    MuJoCoBackend::manifest(),
                    seed,
                )?,
            )
        }
    } else {
        (
            run_sensor_observed_trace(RapierBackend::new(), RapierBackend::manifest())?,
            run_sensor_observed_trace(mujoco_backend, MuJoCoBackend::manifest())?,
        )
    };
    let comparison = compare_sensor_observed_traces(rapier, mujoco)?;
    eprintln!(
        "sensor comparison: backend_agreement={} first_task_passed={} second_task_passed={}",
        comparison.passed, comparison.first.passed, comparison.second.passed
    );
    ensure!(
        comparison.passed,
        "sensor-observed cross-backend verdict failed"
    );
    Ok((
        serde_json::to_string_pretty(&comparison)? + "\n",
        "sensor-rapier-vs-mujoco",
    ))
}

#[cfg(feature = "mujoco")]
fn run_sensor_episode_batch_mujoco(
    root_seed: u64,
    episode_index: u64,
    num_envs: usize,
    num_workers: usize,
) -> Result<(String, &'static str)> {
    use rne_physics_mujoco::MuJoCoBackend;
    let report = rne_mobility_benchmark::observed_batch::run_sensor_episode_batch(
        || {
            Ok((
                MuJoCoBackend::new(rne_core::SimDuration::from_ticks(
                    rne_mobility_benchmark::observed::SENSOR_OBSERVED_FIXED_DELTA_TICKS,
                ))?,
                MuJoCoBackend::manifest(),
            ))
        },
        root_seed,
        episode_index,
        num_envs,
        num_workers,
    )?;
    eprintln!(
        "sensor episode batch: failed_lanes={} total={}",
        report.failed_lanes,
        report.lanes.len()
    );
    Ok((
        serde_json::to_string_pretty(&report)? + "\n",
        "sensor-episode-batch-mujoco",
    ))
}

#[cfg(not(feature = "mujoco"))]
fn run_sensor_episode_batch_mujoco(
    _root_seed: u64,
    _episode_index: u64,
    _num_envs: usize,
    _num_workers: usize,
) -> Result<(String, &'static str)> {
    bail!("MuJoCo sensor episode batch requires --features mujoco")
}

#[cfg(feature = "mujoco")]
fn run_sensor_replay_mujoco(input: &std::path::Path) -> Result<(String, &'static str)> {
    use rne_core::SimDuration;
    use rne_mobility_benchmark::observed::{
        read_sensor_observed_trace, replay_sensor_observed_trace, SENSOR_OBSERVED_FIXED_DELTA_TICKS,
    };
    use rne_physics_mujoco::MuJoCoBackend;
    let source = read_sensor_observed_trace(input)?;
    let replay = replay_sensor_observed_trace(
        MuJoCoBackend::new(SimDuration::from_ticks(SENSOR_OBSERVED_FIXED_DELTA_TICKS))?,
        MuJoCoBackend::manifest(),
        &source,
    )?;
    eprintln!("voltage replay exact; task_passed={}", replay.passed);
    Ok((
        serde_json::to_string_pretty(&replay)? + "\n",
        "sensor-voltage-replay-mujoco",
    ))
}

#[cfg(not(feature = "mujoco"))]
fn run_sensor_replay_mujoco(_input: &std::path::Path) -> Result<(String, &'static str)> {
    bail!("MuJoCo voltage replay requires --features mujoco")
}

#[cfg(not(feature = "mujoco"))]
fn run_sensor_mujoco() -> Result<(String, &'static str)> {
    bail!("sensor-observed mujoco backend requires --features mujoco")
}

#[cfg(not(feature = "mujoco"))]
fn run_sensor_comparison(
    _episode_seed: Option<u64>,
    _physical_reset: bool,
) -> Result<(String, &'static str)> {
    bail!("sensor-observed backend comparison requires --features mujoco")
}

#[cfg(feature = "mujoco")]
fn run_mujoco() -> Result<(String, &'static str)> {
    use rne_core::SimDuration;
    use rne_mobility_benchmark::backend::BACKEND_MOBILITY_FIXED_DELTA_TICKS;
    use rne_physics_mujoco::MuJoCoBackend;

    let backend = MuJoCoBackend::new(SimDuration::from_ticks(BACKEND_MOBILITY_FIXED_DELTA_TICKS))?;
    let trace = run_backend_mobility_trace(backend, MuJoCoBackend::manifest())?;
    ensure!(trace.passed, "MuJoCo mobility benchmark verdict failed");
    Ok((serde_json::to_string_pretty(&trace)? + "\n", "mujoco"))
}

#[cfg(feature = "mujoco")]
fn run_mobility_randomized_backend_mujoco(
    root_seed: u64,
    lane_id: u64,
    episode_index: u64,
) -> Result<(String, &'static str)> {
    use rne_core::SimDuration;
    use rne_mobility_benchmark::backend::BACKEND_MOBILITY_FIXED_DELTA_TICKS;
    use rne_physics_mujoco::MuJoCoBackend;

    let trace = run_mobility_randomized_backend_trace(
        MuJoCoBackend::new(SimDuration::from_ticks(BACKEND_MOBILITY_FIXED_DELTA_TICKS))?,
        MuJoCoBackend::manifest(),
        root_seed,
        lane_id,
        episode_index,
    )?;
    ensure!(
        trace.trace.passed,
        "randomized MuJoCo Mobility trace failed"
    );
    Ok((
        serde_json::to_string_pretty(&trace)? + "\n",
        "mobility-randomized-backend-mujoco",
    ))
}

#[cfg(feature = "mujoco")]
fn run_mobility_randomized_backend_comparison(
    root_seed: u64,
    lane_id: u64,
    episode_index: u64,
) -> Result<(String, &'static str)> {
    use rne_core::SimDuration;
    use rne_mobility_benchmark::backend::BACKEND_MOBILITY_FIXED_DELTA_TICKS;
    use rne_mobility_benchmark::mobility_randomization::compare_mobility_randomized_backend_traces;
    use rne_physics_mujoco::MuJoCoBackend;

    let rapier = run_mobility_randomized_backend_trace(
        RapierBackend::new(),
        RapierBackend::manifest(),
        root_seed,
        lane_id,
        episode_index,
    )?;
    let mujoco = run_mobility_randomized_backend_trace(
        MuJoCoBackend::new(SimDuration::from_ticks(BACKEND_MOBILITY_FIXED_DELTA_TICKS))?,
        MuJoCoBackend::manifest(),
        root_seed,
        lane_id,
        episode_index,
    )?;
    let comparison = compare_mobility_randomized_backend_traces(rapier, mujoco)?;
    ensure!(
        comparison.passed,
        "randomized cross-backend Mobility verdict failed: {:#?}",
        comparison.metrics
    );
    Ok((
        serde_json::to_string_pretty(&comparison)? + "\n",
        "mobility-randomized-backend-compare",
    ))
}

#[cfg(not(feature = "mujoco"))]
fn run_mobility_randomized_backend_mujoco(
    _root_seed: u64,
    _lane_id: u64,
    _episode_index: u64,
) -> Result<(String, &'static str)> {
    bail!("randomized MuJoCo backend requires --features mujoco")
}

#[cfg(not(feature = "mujoco"))]
fn run_mobility_randomized_backend_comparison(
    _root_seed: u64,
    _lane_id: u64,
    _episode_index: u64,
) -> Result<(String, &'static str)> {
    bail!("randomized backend comparison requires --features mujoco")
}

#[cfg(feature = "mujoco")]
fn run_comparison(failure_replay: Option<&std::path::Path>) -> Result<(String, &'static str)> {
    use rne_core::SimDuration;
    use rne_mobility_benchmark::backend::{
        backend_mobility_divergence_replay, compare_backend_mobility_traces,
        BACKEND_MOBILITY_FIXED_DELTA_TICKS,
    };
    use rne_physics_mujoco::MuJoCoBackend;

    let rapier = run_backend_mobility_trace(RapierBackend::new(), RapierBackend::manifest())?;
    let mujoco = run_backend_mobility_trace(
        MuJoCoBackend::new(SimDuration::from_ticks(BACKEND_MOBILITY_FIXED_DELTA_TICKS))?,
        MuJoCoBackend::manifest(),
    )?;
    let comparison = compare_backend_mobility_traces(rapier, mujoco)?;
    ensure!(comparison.passed, "cross-backend mobility verdict failed");
    if let Some(path) = failure_replay {
        let replay =
            backend_mobility_divergence_replay(&comparison.first, &comparison.second, 0.001)?;
        replay.write_json(path)?;
    }
    Ok((
        serde_json::to_string_pretty(&comparison)? + "\n",
        "rapier-vs-mujoco",
    ))
}

#[cfg(not(feature = "mujoco"))]
fn run_mujoco() -> Result<(String, &'static str)> {
    bail!("mujoco backend requires --features mujoco")
}

#[cfg(not(feature = "mujoco"))]
fn run_comparison(_failure_replay: Option<&std::path::Path>) -> Result<(String, &'static str)> {
    bail!("backend comparison requires --features mujoco")
}
