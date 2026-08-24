//! Headless, deterministic IMU measurement and estimator validation.
//!
//! The fixture keeps raw gyroscope/accelerometer feedback separate from truth,
//! exercises stationary and prescribed roll motion, and localizes dropout and
//! stuck-value failures at the observation boundary.

use anyhow::{bail, Context, Result};
use rne_core::SimTime;
use rne_data::{DataBus, ImuFeedback, ImuFeedbackStatus, InMemoryDataBus, StreamId};
use rne_ecs::{spawn_named, Entity, World};
use rne_math::{Quat, Vec3};
use rne_physics::RigidBody;
use rne_sensor::{
    sample_imu_feedback_sensors, ImuAxisErrors, ImuFeedbackFault, ImuFeedbackSensor,
    ImuFeedbackSensorState, ImuMount, ImuSpec,
};
use rne_world::{Transform3, WorldRandom};
use serde::Serialize;
use sha2::{Digest, Sha256};
use std::f64::consts::TAU;
use std::fs;
use std::path::{Path, PathBuf};

const SCHEMA_VERSION: u32 = 1;
const SAMPLE_HZ: u64 = 100;
const SAMPLE_PERIOD_TICKS: u64 = 1_000_000_000 / SAMPLE_HZ;
const LATENCY_TICKS: u64 = 2_000_000;
const DURATION_S: u64 = 12;
const SAMPLE_COUNT: u64 = DURATION_S * SAMPLE_HZ + 1;
const MOTION_START_S: f64 = 4.0;
const MOTION_AMPLITUDE_RAD: f64 = 0.35;
const MOTION_FREQUENCY_HZ: f64 = 0.25;
const FAULT_SEQUENCE: u64 = 650;
const STREAM_ID: StreamId = StreamId::new(91_001);
const COMPLEMENTARY_GAIN: f64 = 0.02;
const INNOVATION_SIGMA_RAD: f64 = 0.015;

#[derive(Clone, Copy, Debug)]
enum Scenario {
    Nominal,
    Dropout,
    Stuck,
}

impl Scenario {
    fn name(self) -> &'static str {
        match self {
            Self::Nominal => "nominal",
            Self::Dropout => "dropout",
            Self::Stuck => "stuck",
        }
    }

    fn fault(self) -> ImuFeedbackFault {
        match self {
            Self::Nominal => ImuFeedbackFault::None,
            Self::Dropout => ImuFeedbackFault::DropSequence {
                sequence: FAULT_SEQUENCE,
            },
            Self::Stuck => ImuFeedbackFault::StuckFromSequence {
                sequence: FAULT_SEQUENCE,
            },
        }
    }
}

#[derive(Clone, Debug, Serialize)]
struct TraceSample {
    sequence: u64,
    capture_ticks: u64,
    available_ticks: u64,
    truth_roll_rad: f64,
    estimated_roll_rad: f64,
    error_rad: f64,
    innovation_rad: f64,
    status: ImuFeedbackStatus,
}

#[derive(Clone, Debug, Serialize)]
struct PhaseMetrics {
    sample_count: usize,
    rmse_rad: f64,
    max_abs_error_rad: f64,
    mean_normalized_innovation_squared: f64,
    innovation_within_3sigma_fraction: f64,
}

#[derive(Clone, Debug, Serialize)]
struct ScenarioReport {
    scenario: &'static str,
    verdict: &'static str,
    attempted_samples: u64,
    emitted_samples: u64,
    first_violation_sequence: Option<u64>,
    first_violation_kind: Option<&'static str>,
    timestamp_mismatches: u64,
    stationary: PhaseMetrics,
    prescribed_motion: PhaseMetrics,
    trace_sha256: String,
    trace: Vec<TraceSample>,
}

#[derive(Debug, Serialize)]
struct ValidationReport {
    kind: &'static str,
    schema_version: u32,
    verdict: &'static str,
    fixture: FixtureContract,
    estimator: EstimatorContract,
    scenarios: Vec<ScenarioReport>,
}

#[derive(Debug, Serialize)]
struct FixtureContract {
    sample_hz: u64,
    sample_period_ticks: u64,
    latency_ticks: u64,
    duration_s: u64,
    stationary_duration_s: f64,
    prescribed_roll_amplitude_rad: f64,
    prescribed_roll_frequency_hz: f64,
    mount_translation_m: [f64; 3],
    mount_rotation_xyzw: [f64; 4],
    fault_sequence: u64,
}

#[derive(Debug, Serialize)]
struct EstimatorContract {
    kind: &'static str,
    correction_gain: f64,
    innovation_sigma_rad: f64,
    nominal_stationary_rmse_limit_rad: f64,
    nominal_motion_rmse_limit_rad: f64,
}

#[derive(Clone, Copy, Debug, Default)]
struct ComplementaryEstimator {
    roll_rad: f64,
    initialized: bool,
}

impl ComplementaryEstimator {
    fn update(&mut self, feedback: &ImuFeedback) -> f64 {
        let accel_roll_rad = feedback
            .specific_force_m_s2
            .x
            .atan2(feedback.specific_force_m_s2.y);
        if !self.initialized {
            self.roll_rad = accel_roll_rad;
            self.initialized = true;
            return 0.0;
        }
        let predicted = self.roll_rad + feedback.angular_velocity_rad_s.z / SAMPLE_HZ as f64;
        let innovation = wrap_angle(accel_roll_rad - predicted);
        self.roll_rad = wrap_angle(predicted + COMPLEMENTARY_GAIN * innovation);
        innovation
    }
}

fn main() {
    if let Err(error) = run() {
        eprintln!("IMU validation failed: {error:#}");
        std::process::exit(1);
    }
}

fn run() -> Result<()> {
    let output = output_dir()?;
    fs::create_dir_all(&output)
        .with_context(|| format!("create IMU validation output {}", output.display()))?;

    let nominal = run_scenario(Scenario::Nominal)?;
    let nominal_repeat = run_scenario(Scenario::Nominal)?;
    if nominal.trace_sha256 != nominal_repeat.trace_sha256 {
        bail!("nominal IMU trace is not deterministic");
    }
    let dropout = run_scenario(Scenario::Dropout)?;
    let stuck = run_scenario(Scenario::Stuck)?;
    let nominal_pass = nominal.stationary.rmse_rad <= 0.01
        && nominal.prescribed_motion.rmse_rad <= 0.025
        && nominal.timestamp_mismatches == 0
        && nominal.first_violation_sequence.is_none();
    let faults_localized = dropout.first_violation_sequence == Some(FAULT_SEQUENCE)
        && dropout.first_violation_kind == Some("missing_sequence")
        && stuck.first_violation_sequence == Some(FAULT_SEQUENCE)
        && stuck.first_violation_kind == Some("stuck_value");
    let verdict = if nominal_pass && faults_localized {
        "pass"
    } else {
        "fail"
    };
    let report = ValidationReport {
        kind: "rne_imu_validation_report",
        schema_version: SCHEMA_VERSION,
        verdict,
        fixture: FixtureContract {
            sample_hz: SAMPLE_HZ,
            sample_period_ticks: SAMPLE_PERIOD_TICKS,
            latency_ticks: LATENCY_TICKS,
            duration_s: DURATION_S,
            stationary_duration_s: MOTION_START_S,
            prescribed_roll_amplitude_rad: MOTION_AMPLITUDE_RAD,
            prescribed_roll_frequency_hz: MOTION_FREQUENCY_HZ,
            mount_translation_m: [0.0; 3],
            mount_rotation_xyzw: [0.0, 0.0, 0.0, 1.0],
            fault_sequence: FAULT_SEQUENCE,
        },
        estimator: EstimatorContract {
            kind: "scalar_roll_complementary_filter",
            correction_gain: COMPLEMENTARY_GAIN,
            innovation_sigma_rad: INNOVATION_SIGMA_RAD,
            nominal_stationary_rmse_limit_rad: 0.01,
            nominal_motion_rmse_limit_rad: 0.025,
        },
        scenarios: vec![nominal, dropout, stuck],
    };
    write_report(&output, &report)?;
    if verdict != "pass" {
        bail!("IMU validation report failed its registered gates");
    }
    println!(
        "IMU validation pass: report={} nominal_trace_sha256={}",
        output.join("imu-validation-report.json").display(),
        report.scenarios[0].trace_sha256
    );
    Ok(())
}

fn run_scenario(scenario: Scenario) -> Result<ScenarioReport> {
    let (mut world, body, sensor, mut bus) = fixture(scenario);
    let mut estimator = ComplementaryEstimator::default();
    let mut trace = Vec::with_capacity(SAMPLE_COUNT as usize);
    let mut first_violation_sequence = None;
    let mut first_violation_kind = None;
    let mut timestamp_mismatches = 0;

    for sample_index in 0..SAMPLE_COUNT {
        let sequence = sample_index + 1;
        let capture_ticks = sample_index * SAMPLE_PERIOD_TICKS;
        let time_s = capture_ticks as f64 * 1.0e-9;
        apply_truth(&mut world, body, time_s);
        let published =
            sample_imu_feedback_sensors(&mut world, SimTime::from_ticks(capture_ticks), &mut bus)?;
        if published == 0 {
            if first_violation_sequence.is_none() {
                first_violation_sequence = Some(sequence);
                first_violation_kind = Some("missing_sequence");
            }
            continue;
        }
        let available_ticks = capture_ticks + LATENCY_TICKS;
        let frame = bus
            .latest_available::<ImuFeedback>(STREAM_ID, SimTime::from_ticks(available_ticks))
            .context("published IMU frame was unavailable at its declared latency")?;
        if frame.sequence != sequence
            || frame.capture_time.ticks() != capture_ticks
            || frame.available_time.ticks() != available_ticks
            || frame.payload.scheduled_capture_ticks != capture_ticks
            || frame.payload.sample_phase_error_ticks != 0
        {
            timestamp_mismatches += 1;
        }
        if frame.payload.status == ImuFeedbackStatus::StuckValue
            && first_violation_sequence.is_none()
        {
            first_violation_sequence = Some(sequence);
            first_violation_kind = Some("stuck_value");
        }
        let innovation_rad = estimator.update(&frame.payload);
        let truth_roll_rad = prescribed_roll(time_s).0;
        let error_rad = wrap_angle(estimator.roll_rad - truth_roll_rad);
        trace.push(TraceSample {
            sequence,
            capture_ticks,
            available_ticks,
            truth_roll_rad,
            estimated_roll_rad: estimator.roll_rad,
            error_rad,
            innovation_rad,
            status: frame.payload.status,
        });
    }

    let state = world
        .get::<ImuFeedbackSensorState>(sensor)
        .context("IMU sensor state missing after validation")?;
    let trace_bytes = serde_json::to_vec(&trace)?;
    let stationary = metrics(
        trace
            .iter()
            .filter(|sample| sample.capture_ticks < (MOTION_START_S * 1.0e9) as u64),
    );
    let prescribed_motion = metrics(
        trace
            .iter()
            .filter(|sample| sample.capture_ticks >= (MOTION_START_S * 1.0e9) as u64),
    );
    let expected_failure = !matches!(scenario, Scenario::Nominal);
    let localized = first_violation_sequence == Some(FAULT_SEQUENCE);
    Ok(ScenarioReport {
        scenario: scenario.name(),
        verdict: if expected_failure == localized {
            "pass"
        } else {
            "fail"
        },
        attempted_samples: state.attempted_sequence,
        emitted_samples: state.emitted_frames,
        first_violation_sequence,
        first_violation_kind,
        timestamp_mismatches,
        stationary,
        prescribed_motion,
        trace_sha256: hex_digest(Sha256::digest(trace_bytes)),
        trace,
    })
}

fn fixture(scenario: Scenario) -> (World, Entity, Entity, InMemoryDataBus) {
    let mut world = World::new();
    world.insert_resource(WorldRandom::new(0x494d_555f_5641_4c31));
    let body = spawn_named(&mut world, "imu_validation_body");
    world
        .entity_mut(body)
        .insert((Transform3::IDENTITY, RigidBody::default()));
    let sensor = spawn_named(&mut world, "imu_validation_sensor");
    world.entity_mut(sensor).insert((
        ImuMount {
            body_entity: body,
            body_from_sensor: Transform3::IDENTITY,
        },
        ImuFeedbackSensor {
            spec: validation_imu_spec(),
            update_rate_hz: SAMPLE_HZ as f64,
            sample_period_ticks: Some(SAMPLE_PERIOD_TICKS),
            phase_offset_ticks: 0,
            latency_ticks: LATENCY_TICKS,
            enabled: true,
            stream_id: STREAM_ID,
            fault: scenario.fault(),
        },
        ImuFeedbackSensorState::default(),
    ));
    (world, body, sensor, InMemoryDataBus::new())
}

fn validation_imu_spec() -> ImuSpec {
    ImuSpec {
        seed: 0x5641_4c49_4441_5445,
        gyro: ImuAxisErrors {
            random_walk: 0.0002,
            bias_instability: 0.0003,
            bias_correlation_time_s: 30.0,
            rate_random_walk: 0.00002,
            turn_on_bias: Vec3::new(0.0001, -0.0002, 0.0003),
            scale_factor_error: Vec3::splat(0.0002),
            misalignment_rad: Vec3::new(0.0001, -0.0001, 0.0001),
        },
        accel: ImuAxisErrors {
            random_walk: 0.003,
            bias_instability: 0.005,
            bias_correlation_time_s: 60.0,
            rate_random_walk: 0.0002,
            turn_on_bias: Vec3::new(0.005, -0.004, 0.003),
            scale_factor_error: Vec3::splat(0.0005),
            misalignment_rad: Vec3::new(0.0002, -0.0001, 0.0002),
        },
        gyro_range_rad_s: 4.0,
        accel_range_m_s2: 40.0,
        gyro_resolution_rad_s: 0.0001,
        accel_resolution_m_s2: 0.001,
        ..ImuSpec::default()
    }
}

fn apply_truth(world: &mut World, body: Entity, time_s: f64) {
    let (roll_rad, roll_rate_rad_s) = prescribed_roll(time_s);
    world.get_mut::<Transform3>(body).unwrap().rotation = Quat::from_rotation_z(roll_rad);
    world
        .get_mut::<RigidBody>(body)
        .unwrap()
        .angular_velocity_rad_s = Vec3::new(0.0, 0.0, roll_rate_rad_s);
}

fn prescribed_roll(time_s: f64) -> (f64, f64) {
    if time_s < MOTION_START_S {
        return (0.0, 0.0);
    }
    let phase = TAU * MOTION_FREQUENCY_HZ * (time_s - MOTION_START_S);
    (
        MOTION_AMPLITUDE_RAD * phase.sin(),
        MOTION_AMPLITUDE_RAD * TAU * MOTION_FREQUENCY_HZ * phase.cos(),
    )
}

fn metrics<'a>(samples: impl Iterator<Item = &'a TraceSample>) -> PhaseMetrics {
    let samples: Vec<_> = samples.collect();
    let count = samples.len().max(1) as f64;
    let error_sum_sq = samples
        .iter()
        .map(|sample| sample.error_rad.powi(2))
        .sum::<f64>();
    let max_abs_error_rad = samples
        .iter()
        .map(|sample| sample.error_rad.abs())
        .fold(0.0, f64::max);
    let nis_sum = samples
        .iter()
        .map(|sample| (sample.innovation_rad / INNOVATION_SIGMA_RAD).powi(2))
        .sum::<f64>();
    let within = samples
        .iter()
        .filter(|sample| sample.innovation_rad.abs() <= 3.0 * INNOVATION_SIGMA_RAD)
        .count() as f64;
    PhaseMetrics {
        sample_count: samples.len(),
        rmse_rad: (error_sum_sq / count).sqrt(),
        max_abs_error_rad,
        mean_normalized_innovation_squared: nis_sum / count,
        innovation_within_3sigma_fraction: within / count,
    }
}

fn wrap_angle(angle_rad: f64) -> f64 {
    (angle_rad + std::f64::consts::PI).rem_euclid(TAU) - std::f64::consts::PI
}

fn output_dir() -> Result<PathBuf> {
    let mut args = std::env::args().skip(1);
    let mut output = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../..")
        .join("target/imu-validation");
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--output" => {
                output = PathBuf::from(args.next().context("--output requires a directory")?);
            }
            _ => bail!("unknown argument {arg}; expected --output <directory>"),
        }
    }
    Ok(output)
}

fn write_report(output: &Path, report: &ValidationReport) -> Result<()> {
    let mut json = serde_json::to_vec_pretty(report)?;
    json.push(b'\n');
    fs::write(output.join("imu-validation-report.json"), &json)?;
    let escaped = String::from_utf8(json)?
        .replace('&', "&amp;")
        .replace('<', "&lt;")
        .replace('>', "&gt;");
    let html = format!(
        "<!doctype html><meta charset=\"utf-8\"><title>RNE IMU validation</title>\
<style>body{{font:15px system-ui;max-width:1100px;margin:2rem auto;padding:0 1rem;background:#10151d;color:#e9f0f7}}\
h1{{color:#79d7ff}}pre{{white-space:pre-wrap;background:#17202b;padding:1rem;border-radius:8px}}</style>\
<h1>RNE IMU validation — {}</h1><p>Stationary and prescribed-motion estimator evidence, with localized dropout and stuck-value cases.</p><pre>{escaped}</pre>",
        report.verdict
    );
    fs::write(output.join("imu-validation-report.html"), html)?;
    Ok(())
}

fn hex_digest(digest: impl AsRef<[u8]>) -> String {
    digest
        .as_ref()
        .iter()
        .map(|byte| format!("{byte:02x}"))
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn nominal_fixture_is_deterministic_and_meets_estimator_limits() {
        let first = run_scenario(Scenario::Nominal).unwrap();
        let second = run_scenario(Scenario::Nominal).unwrap();
        assert_eq!(first.trace_sha256, second.trace_sha256);
        assert!(first.stationary.rmse_rad <= 0.01);
        assert!(first.prescribed_motion.rmse_rad <= 0.025);
        assert_eq!(first.timestamp_mismatches, 0);
        assert_eq!(first.first_violation_sequence, None);
    }

    #[test]
    fn deliberate_faults_are_localized_at_the_contract_boundary() {
        let dropout = run_scenario(Scenario::Dropout).unwrap();
        let stuck = run_scenario(Scenario::Stuck).unwrap();
        assert_eq!(dropout.first_violation_sequence, Some(FAULT_SEQUENCE));
        assert_eq!(dropout.first_violation_kind, Some("missing_sequence"));
        assert_eq!(stuck.first_violation_sequence, Some(FAULT_SEQUENCE));
        assert_eq!(stuck.first_violation_kind, Some("stuck_value"));
    }
}
