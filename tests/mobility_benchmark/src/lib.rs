//! Stable Mobility Physical AI longitudinal benchmark producer.

pub mod backend;

use anyhow::{ensure, Result};
use rne_robot::{
    evaluate_longitudinal_mobility_plant, CombinedSlipTireSpec, DcMotorFailureMode, DcMotorSpec,
    LongitudinalMobilityPlantSpec, LongitudinalMobilityPlantState, TransmissionSpec,
    WheelAssemblySpec,
};
use serde::{Deserialize, Serialize};

/// Stable artifact discriminator for the longitudinal benchmark.
pub const MOBILITY_BENCHMARK_KIND: &str = "rne_mobility_longitudinal_benchmark";
/// Current benchmark report schema version.
pub const MOBILITY_BENCHMARK_SCHEMA_VERSION: u32 = 1;
/// Fixed integration step used by benchmark cases, in nanosecond ticks.
pub const MOBILITY_BENCHMARK_FIXED_DELTA_TICKS: u64 = 1_000_000;

const FIXED_DELTA_S: f64 = MOBILITY_BENCHMARK_FIXED_DELTA_TICKS as f64 / 1_000_000_000.0;

/// One SI-unit metric and its inclusive acceptance interval.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MobilityBenchmarkMetric {
    /// Stable metric identity.
    pub id: String,
    /// Unit symbol, or `1` when dimensionless.
    pub unit: String,
    /// Observed scalar value.
    pub value: f64,
    /// Inclusive lower acceptance bound.
    pub minimum: f64,
    /// Inclusive upper acceptance bound.
    pub maximum: f64,
    /// Whether the observed value falls inside the interval.
    pub passed: bool,
}

/// One deterministic benchmark maneuver.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MobilityBenchmarkCase {
    /// Stable case identity.
    pub id: String,
    /// Human-readable plant or fault purpose.
    pub purpose: String,
    /// Number of fixed plant steps.
    pub steps: u64,
    /// Ordered metrics.
    pub metrics: Vec<MobilityBenchmarkMetric>,
    /// Whether every metric passed.
    pub passed: bool,
}

/// Timing-free deterministic benchmark report.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MobilityBenchmarkReport {
    /// Stable artifact discriminator.
    pub kind: String,
    /// Report schema version.
    pub schema_version: u32,
    /// Engine package version that produced the report.
    pub engine_version: String,
    /// Fixed step in simulation nanosecond ticks.
    pub fixed_delta_ticks: u64,
    /// Cases in strict identity order.
    pub cases: Vec<MobilityBenchmarkCase>,
    /// Whether every case passed.
    pub passed: bool,
    /// FNV-1a digest of the same report with this field empty.
    pub content_digest: String,
}

impl MobilityBenchmarkReport {
    /// Recomputes and verifies ordering, metrics, verdicts, and content digest.
    pub fn validate(&self) -> Result<()> {
        ensure!(self.kind == MOBILITY_BENCHMARK_KIND, "kind mismatch");
        ensure!(
            self.schema_version == MOBILITY_BENCHMARK_SCHEMA_VERSION,
            "schema mismatch"
        );
        ensure!(
            self.fixed_delta_ticks == MOBILITY_BENCHMARK_FIXED_DELTA_TICKS,
            "fixed step mismatch"
        );
        ensure!(!self.cases.is_empty(), "benchmark omitted cases");
        ensure!(
            self.cases.windows(2).all(|pair| pair[0].id < pair[1].id),
            "cases are not strictly sorted"
        );
        for case in &self.cases {
            ensure!(case.steps > 0, "case {} has zero steps", case.id);
            ensure!(!case.metrics.is_empty(), "case {} omitted metrics", case.id);
            ensure!(
                case.metrics.windows(2).all(|pair| pair[0].id < pair[1].id),
                "case {} metrics are not strictly sorted",
                case.id
            );
            for metric in &case.metrics {
                ensure!(
                    metric.value.is_finite(),
                    "metric {} is non-finite",
                    metric.id
                );
                ensure!(metric.minimum.is_finite(), "metric minimum is non-finite");
                ensure!(metric.maximum.is_finite(), "metric maximum is non-finite");
                ensure!(
                    metric.minimum <= metric.maximum,
                    "metric interval is inverted"
                );
                ensure!(
                    metric.passed
                        == (metric.value >= metric.minimum && metric.value <= metric.maximum),
                    "metric {} verdict mismatch",
                    metric.id
                );
            }
            ensure!(
                case.passed == case.metrics.iter().all(|metric| metric.passed),
                "case {} verdict mismatch",
                case.id
            );
        }
        ensure!(
            self.passed == self.cases.iter().all(|case| case.passed),
            "report verdict mismatch"
        );
        ensure!(
            self.content_digest == report_digest(self)?,
            "content digest mismatch"
        );
        Ok(())
    }
}

/// Runs the deterministic longitudinal benchmark matrix.
pub fn run_mobility_benchmark() -> Result<MobilityBenchmarkReport> {
    let nominal = plant_spec(1.0, DcMotorFailureMode::Nominal);
    let first = evaluate_longitudinal_mobility_plant(
        nominal,
        LongitudinalMobilityPlantState::default(),
        24.0,
        FIXED_DELTA_S,
    )?;
    let locked_rotor = case(
        "01_locked_rotor",
        "supply and current limits are explicit at zero rotor speed",
        1,
        vec![
            metric(
                "current_a",
                "A",
                first.motor.state.current_a,
                nominal.motor.current_limit_a,
                nominal.motor.current_limit_a,
            ),
            metric(
                "terminal_voltage_v",
                "V",
                first.motor.terminal_voltage_v,
                24.0,
                24.0,
            ),
        ],
    );

    let high = rollout(
        nominal,
        LongitudinalMobilityPlantState::default(),
        24.0,
        2_000,
    )?;
    // Ice-like road scaling deliberately puts this plant into its
    // traction-limited regime; 0.2 remains motor-limited at steady state.
    let low_spec = plant_spec(0.05, DcMotorFailureMode::Nominal);
    let low = rollout(
        low_spec,
        LongitudinalMobilityPlantState::default(),
        24.0,
        2_000,
    )?;
    let high_slip_speed_m_s =
        high.state.wheel_velocity_rad_s * nominal.wheel.radius_m - high.state.velocity_m_s;
    let low_slip_speed_m_s =
        low.state.wheel_velocity_rad_s * low_spec.wheel.radius_m - low.state.velocity_m_s;
    let acceleration = case(
        "02_high_mu_acceleration",
        "motor, transmission, wheel inertia, transient slip, and chassis force are coupled",
        2_000,
        vec![
            metric("distance_m", "m", high.state.position_m, 1.0, 30.0),
            metric(
                "maximum_friction_utilization",
                "1",
                high.maximum_utilization,
                0.1,
                1.0,
            ),
            metric("velocity_m_s", "m/s", high.state.velocity_m_s, 1.0, 20.0),
        ],
    );
    let split_friction = case(
        "03_low_mu_traction",
        "lower road friction reduces chassis speed and increases wheel spin",
        2_000,
        vec![
            metric(
                "high_minus_low_velocity_m_s",
                "m/s",
                high.state.velocity_m_s - low.state.velocity_m_s,
                0.1,
                20.0,
            ),
            metric(
                "low_minus_high_slip_speed_m_s",
                "m/s",
                low_slip_speed_m_s - high_slip_speed_m_s,
                0.1,
                100.0,
            ),
            metric(
                "low_mu_maximum_friction_utilization",
                "1",
                low.maximum_utilization,
                0.99,
                1.0,
            ),
        ],
    );

    let accelerated = rollout(
        nominal,
        LongitudinalMobilityPlantState::default(),
        24.0,
        1_500,
    )?;
    let braked = rollout(nominal, accelerated.state, -24.0, 500)?;
    let braking = case(
        "04_regenerative_braking",
        "negative voltage produces negative current and reduces forward speed",
        500,
        vec![
            metric("final_current_a", "A", braked.last_current_a, -20.0, 0.0),
            metric(
                "speed_reduction_m_s",
                "m/s",
                accelerated.state.velocity_m_s - braked.state.velocity_m_s,
                0.1,
                20.0,
            ),
        ],
    );

    let open = rollout(
        plant_spec(1.0, DcMotorFailureMode::OpenCircuit),
        LongitudinalMobilityPlantState::default(),
        24.0,
        2_000,
    )?;
    let open_circuit = case(
        "05_open_circuit",
        "open motor terminals cannot create current or vehicle motion",
        2_000,
        vec![
            metric(
                "absolute_current_a",
                "A",
                open.last_current_a.abs(),
                0.0,
                0.0,
            ),
            metric(
                "absolute_distance_m",
                "m",
                open.state.position_m.abs(),
                0.0,
                0.0,
            ),
        ],
    );

    let coarse = rollout(
        nominal,
        LongitudinalMobilityPlantState::default(),
        12.0,
        2_000,
    )?;
    let fine = rollout_with_dt(
        nominal,
        LongitudinalMobilityPlantState::default(),
        12.0,
        FIXED_DELTA_S / 2.0,
        4_000,
    )?;
    let convergence = case(
        "06_step_convergence",
        "halving the fixed step preserves the two-second trajectory envelope",
        4_000,
        vec![
            metric(
                "position_delta_m",
                "m",
                (coarse.state.position_m - fine.state.position_m).abs(),
                0.0,
                0.15,
            ),
            metric(
                "velocity_delta_m_s",
                "m/s",
                (coarse.state.velocity_m_s - fine.state.velocity_m_s).abs(),
                0.0,
                0.15,
            ),
        ],
    );

    let mut report = MobilityBenchmarkReport {
        kind: MOBILITY_BENCHMARK_KIND.to_string(),
        schema_version: MOBILITY_BENCHMARK_SCHEMA_VERSION,
        engine_version: env!("CARGO_PKG_VERSION").to_string(),
        fixed_delta_ticks: MOBILITY_BENCHMARK_FIXED_DELTA_TICKS,
        cases: vec![
            locked_rotor,
            acceleration,
            split_friction,
            braking,
            open_circuit,
            convergence,
        ],
        passed: false,
        content_digest: String::new(),
    };
    report.passed = report.cases.iter().all(|case| case.passed);
    report.content_digest = report_digest(&report)?;
    report.validate()?;
    Ok(report)
}

#[derive(Clone, Copy, Debug)]
struct Rollout {
    state: LongitudinalMobilityPlantState,
    maximum_utilization: f64,
    last_current_a: f64,
}

fn rollout(
    spec: LongitudinalMobilityPlantSpec,
    state: LongitudinalMobilityPlantState,
    voltage_v: f64,
    steps: usize,
) -> Result<Rollout> {
    rollout_with_dt(spec, state, voltage_v, FIXED_DELTA_S, steps)
}

fn rollout_with_dt(
    spec: LongitudinalMobilityPlantSpec,
    mut state: LongitudinalMobilityPlantState,
    voltage_v: f64,
    dt_s: f64,
    steps: usize,
) -> Result<Rollout> {
    let mut maximum_utilization = 0.0_f64;
    let mut last_current_a = state.motor_state.current_a;
    for _ in 0..steps {
        let evaluation = evaluate_longitudinal_mobility_plant(spec, state, voltage_v, dt_s)?;
        state = evaluation.state;
        last_current_a = evaluation.motor.state.current_a;
        maximum_utilization = maximum_utilization.max(evaluation.tire.friction_utilization);
    }
    Ok(Rollout {
        state,
        maximum_utilization,
        last_current_a,
    })
}

fn plant_spec(
    road_friction_scale: f64,
    failure_mode: DcMotorFailureMode,
) -> LongitudinalMobilityPlantSpec {
    LongitudinalMobilityPlantSpec {
        vehicle_mass_kg: 100.0,
        driven_wheel_count: 2,
        normal_load_per_driven_wheel_n: 490.3325,
        road_grade_rad: 0.0,
        aerodynamic_drag_n_s2_m2: 0.4,
        road_friction_scale,
        motor: DcMotorSpec {
            failure_mode,
            ..DcMotorSpec::default()
        },
        transmission: TransmissionSpec::default(),
        wheel: WheelAssemblySpec::default(),
        tire: CombinedSlipTireSpec {
            reference_load_n: 490.3325,
            ..CombinedSlipTireSpec::default()
        },
    }
}

fn metric(id: &str, unit: &str, value: f64, minimum: f64, maximum: f64) -> MobilityBenchmarkMetric {
    MobilityBenchmarkMetric {
        id: id.to_string(),
        unit: unit.to_string(),
        value,
        minimum,
        maximum,
        passed: value >= minimum && value <= maximum,
    }
}

fn case(
    id: &str,
    purpose: &str,
    steps: u64,
    mut metrics: Vec<MobilityBenchmarkMetric>,
) -> MobilityBenchmarkCase {
    metrics.sort_by(|left, right| left.id.cmp(&right.id));
    let passed = metrics.iter().all(|metric| metric.passed);
    MobilityBenchmarkCase {
        id: id.to_string(),
        purpose: purpose.to_string(),
        steps,
        metrics,
        passed,
    }
}

fn report_digest(report: &MobilityBenchmarkReport) -> Result<String> {
    let mut canonical = report.clone();
    canonical.content_digest.clear();
    let bytes = serde_json::to_vec(&canonical)?;
    let mut digest = 0xcbf29ce484222325_u64;
    for byte in bytes {
        digest ^= u64::from(byte);
        digest = digest.wrapping_mul(0x100000001b3);
    }
    Ok(format!("fnv1a64:{digest:016x}"))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn report_is_passing_sorted_self_verifying_and_deterministic() {
        let first = run_mobility_benchmark().unwrap();
        let second = run_mobility_benchmark().unwrap();

        assert!(first.passed);
        assert_eq!(first, second);
        first.validate().unwrap();
        assert_eq!(first.cases.len(), 6);
        assert_eq!(
            serde_json::to_vec(&first).unwrap(),
            serde_json::to_vec(&second).unwrap()
        );
    }

    #[test]
    fn tampering_is_detected() {
        let mut report = run_mobility_benchmark().unwrap();
        report.cases[0].metrics[0].value += 1.0;
        assert!(report.validate().is_err());
    }
}
