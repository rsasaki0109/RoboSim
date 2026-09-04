//! Deterministic, batch-width-independent Mobility plant domain randomization.

use crate::ackermann_suspension::suspension_spec;
use crate::plant_spec;
use anyhow::{ensure, Result};
use rne_ai::derive_episode_seed;
use rne_core::KeyedRandom;
use rne_robot::{
    evaluate_longitudinal_mobility_plant, DcMotorFailureMode, LongitudinalMobilityPlantSpec,
    LongitudinalMobilityPlantState, SuspensionStrutSpec,
};
use serde::{Deserialize, Serialize};

/// Stable randomized-batch artifact kind.
pub const MOBILITY_RANDOMIZED_BATCH_KIND: &str = "rne_mobility_randomized_batch";
/// Randomized-batch schema.
pub const MOBILITY_RANDOMIZED_BATCH_SCHEMA_VERSION: u32 = 1;
/// Fixed analytic integration step in seconds.
pub const MOBILITY_RANDOMIZED_FIXED_DELTA_S: f64 = 0.001;
/// Fixed rollout length per lane.
pub const MOBILITY_RANDOMIZED_STEPS: u64 = 2_000;
const RANDOM_DOMAIN: u64 = 0x524e_452d_4d4f_4249;

/// Inclusive-lower, exclusive-upper uniform range.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct UniformRange {
    /// Inclusive lower bound.
    pub minimum: f64,
    /// Exclusive upper bound.
    pub maximum: f64,
}

impl UniformRange {
    fn valid(self) -> bool {
        self.minimum.is_finite() && self.maximum.is_finite() && self.minimum < self.maximum
    }

    fn sample(self, random: &KeyedRandom, channel: u64) -> f64 {
        random.sample_f64(0, 0, channel, self.minimum, self.maximum)
    }
}

/// Frozen v1 randomization ranges for identifiable Mobility physics.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MobilityRandomizationSpec {
    /// Vehicle mass multiplier.
    pub vehicle_mass_scale: UniformRange,
    /// Armature resistance multiplier.
    pub motor_resistance_scale: UniformRange,
    /// Shared SI torque/back-EMF constant multiplier.
    pub motor_constant_scale: UniformRange,
    /// Motor rotor inertia multiplier.
    pub rotor_inertia_scale: UniformRange,
    /// Motor-to-wheel ratio multiplier.
    pub transmission_ratio_scale: UniformRange,
    /// Drive and backdrive efficiency multiplier.
    pub transmission_efficiency_scale: UniformRange,
    /// Rolling radius multiplier.
    pub wheel_radius_scale: UniformRange,
    /// Wheel rotational inertia multiplier.
    pub wheel_inertia_scale: UniformRange,
    /// Rolling resistance multiplier.
    pub rolling_resistance_scale: UniformRange,
    /// Longitudinal and lateral small-slip stiffness multiplier.
    pub tire_stiffness_scale: UniformRange,
    /// Longitudinal and lateral peak-friction multiplier.
    pub tire_peak_friction_scale: UniformRange,
    /// Road friction multiplier.
    pub road_friction_scale: UniformRange,
    /// Road grade in radians.
    pub road_grade_rad: UniformRange,
    /// Suspension stiffness multiplier.
    pub suspension_stiffness_scale: UniformRange,
    /// Suspension damping multiplier.
    pub suspension_damping_scale: UniformRange,
}

impl MobilityRandomizationSpec {
    /// Returns the bounded v1 training distribution.
    pub fn training_v1() -> Self {
        Self {
            vehicle_mass_scale: range(0.8, 1.2),
            motor_resistance_scale: range(0.85, 1.15),
            motor_constant_scale: range(0.9, 1.1),
            rotor_inertia_scale: range(0.75, 1.25),
            transmission_ratio_scale: range(0.95, 1.05),
            transmission_efficiency_scale: range(0.9, 1.05),
            wheel_radius_scale: range(0.95, 1.05),
            wheel_inertia_scale: range(0.8, 1.2),
            rolling_resistance_scale: range(0.7, 1.3),
            tire_stiffness_scale: range(0.7, 1.3),
            tire_peak_friction_scale: range(0.7, 1.1),
            road_friction_scale: range(0.5, 1.2),
            road_grade_rad: range(-0.08, 0.08),
            suspension_stiffness_scale: range(0.75, 1.25),
            suspension_damping_scale: range(0.75, 1.25),
        }
    }

    /// Checks every range and the conservative all-lower/all-upper profile corners.
    pub fn validate(self) -> Result<()> {
        let ranges = [
            self.vehicle_mass_scale,
            self.motor_resistance_scale,
            self.motor_constant_scale,
            self.rotor_inertia_scale,
            self.transmission_ratio_scale,
            self.transmission_efficiency_scale,
            self.wheel_radius_scale,
            self.wheel_inertia_scale,
            self.rolling_resistance_scale,
            self.tire_stiffness_scale,
            self.tire_peak_friction_scale,
            self.road_friction_scale,
            self.road_grade_rad,
            self.suspension_stiffness_scale,
            self.suspension_damping_scale,
        ];
        ensure!(
            ranges.into_iter().all(UniformRange::valid),
            "invalid randomization range"
        );
        for sample in [self.boundary_sample(false), self.boundary_sample(true)] {
            let profile = apply(sample);
            ensure!(
                profile.plant.is_valid() && profile.suspension.is_valid(),
                "range creates invalid profile"
            );
        }
        Ok(())
    }

    fn boundary_sample(self, upper: bool) -> MobilityRandomizationSample {
        let select = |range: UniformRange| {
            if upper {
                range.maximum
            } else {
                range.minimum
            }
        };
        MobilityRandomizationSample {
            vehicle_mass_scale: select(self.vehicle_mass_scale),
            motor_resistance_scale: select(self.motor_resistance_scale),
            motor_constant_scale: select(self.motor_constant_scale),
            rotor_inertia_scale: select(self.rotor_inertia_scale),
            transmission_ratio_scale: select(self.transmission_ratio_scale),
            transmission_efficiency_scale: select(self.transmission_efficiency_scale),
            wheel_radius_scale: select(self.wheel_radius_scale),
            wheel_inertia_scale: select(self.wheel_inertia_scale),
            rolling_resistance_scale: select(self.rolling_resistance_scale),
            tire_stiffness_scale: select(self.tire_stiffness_scale),
            tire_peak_friction_scale: select(self.tire_peak_friction_scale),
            road_friction_scale: select(self.road_friction_scale),
            road_grade_rad: select(self.road_grade_rad),
            suspension_stiffness_scale: select(self.suspension_stiffness_scale),
            suspension_damping_scale: select(self.suspension_damping_scale),
        }
    }

    /// Samples one profile solely from its episode seed.
    pub fn sample(self, episode_seed: u64) -> RandomizedMobilityProfile {
        let random = KeyedRandom::new(episode_seed, RANDOM_DOMAIN);
        let values = MobilityRandomizationSample {
            vehicle_mass_scale: self.vehicle_mass_scale.sample(&random, 0),
            motor_resistance_scale: self.motor_resistance_scale.sample(&random, 1),
            motor_constant_scale: self.motor_constant_scale.sample(&random, 2),
            rotor_inertia_scale: self.rotor_inertia_scale.sample(&random, 3),
            transmission_ratio_scale: self.transmission_ratio_scale.sample(&random, 4),
            transmission_efficiency_scale: self.transmission_efficiency_scale.sample(&random, 5),
            wheel_radius_scale: self.wheel_radius_scale.sample(&random, 6),
            wheel_inertia_scale: self.wheel_inertia_scale.sample(&random, 7),
            rolling_resistance_scale: self.rolling_resistance_scale.sample(&random, 8),
            tire_stiffness_scale: self.tire_stiffness_scale.sample(&random, 9),
            tire_peak_friction_scale: self.tire_peak_friction_scale.sample(&random, 10),
            road_friction_scale: self.road_friction_scale.sample(&random, 11),
            road_grade_rad: self.road_grade_rad.sample(&random, 12),
            suspension_stiffness_scale: self.suspension_stiffness_scale.sample(&random, 13),
            suspension_damping_scale: self.suspension_damping_scale.sample(&random, 14),
        };
        apply(values)
    }
}

/// Exact sampled values, retained separately from the applied physical structs.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MobilityRandomizationSample {
    /// Vehicle mass scale.
    pub vehicle_mass_scale: f64,
    /// Motor resistance scale.
    pub motor_resistance_scale: f64,
    /// Coupled torque/back-EMF constant scale.
    pub motor_constant_scale: f64,
    /// Rotor inertia scale.
    pub rotor_inertia_scale: f64,
    /// Transmission ratio scale.
    pub transmission_ratio_scale: f64,
    /// Transmission efficiency scale.
    pub transmission_efficiency_scale: f64,
    /// Wheel radius scale.
    pub wheel_radius_scale: f64,
    /// Wheel inertia scale.
    pub wheel_inertia_scale: f64,
    /// Rolling resistance scale.
    pub rolling_resistance_scale: f64,
    /// Tire stiffness scale.
    pub tire_stiffness_scale: f64,
    /// Tire peak-friction scale.
    pub tire_peak_friction_scale: f64,
    /// Road friction scale.
    pub road_friction_scale: f64,
    /// Road grade in radians.
    pub road_grade_rad: f64,
    /// Suspension stiffness scale.
    pub suspension_stiffness_scale: f64,
    /// Suspension damping scale.
    pub suspension_damping_scale: f64,
}

/// One fully applied backend-neutral physical profile.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RandomizedMobilityProfile {
    /// Sample retained for audit and replay.
    pub sample: MobilityRandomizationSample,
    /// Applied motor/transmission/wheel/tire/road plant.
    pub plant: LongitudinalMobilityPlantSpec,
    /// Applied suspension force law.
    pub suspension: SuspensionStrutSpec,
}

/// Result and stable digest for one vectorized lane.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MobilityRandomizedLane {
    /// Stable zero-based lane identity.
    pub lane_id: u64,
    /// Lane-local episode index.
    pub episode_index: u64,
    /// Width-independent derived episode seed.
    pub episode_seed: u64,
    /// Exact randomized physics.
    pub profile: RandomizedMobilityProfile,
    /// Final longitudinal position.
    pub final_position_m: f64,
    /// Final longitudinal velocity.
    pub final_velocity_m_s: f64,
    /// Final wheel angular velocity.
    pub final_wheel_velocity_rad_s: f64,
    /// Final measured plant current.
    pub final_current_a: f64,
    /// Maximum tire friction utilization.
    pub maximum_friction_utilization: f64,
    /// Lane acceptance verdict.
    pub passed: bool,
    /// FNV-1a integrity digest with this field empty.
    pub content_digest: String,
}

/// Ordered CPU reference batch for Physical AI training integration.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MobilityRandomizedBatchReport {
    /// Artifact discriminator.
    pub kind: String,
    /// Artifact schema.
    pub schema_version: u32,
    /// Root seed before lane/episode derivation.
    pub root_seed: u64,
    /// Shared episode index.
    pub episode_index: u64,
    /// Exact randomization distribution.
    pub randomization_spec: MobilityRandomizationSpec,
    /// Fixed integration step.
    pub fixed_delta_s: f64,
    /// Steps per lane.
    pub steps: u64,
    /// Results in strict lane order.
    pub lanes: Vec<MobilityRandomizedLane>,
    /// Aggregate verdict.
    pub passed: bool,
    /// FNV-1a integrity digest with this field empty.
    pub content_digest: String,
}

impl MobilityRandomizedBatchReport {
    /// Re-samples and re-runs every lane, then verifies ordering and digests.
    pub fn validate(&self) -> Result<()> {
        ensure!(
            self.kind == MOBILITY_RANDOMIZED_BATCH_KIND
                && self.schema_version == MOBILITY_RANDOMIZED_BATCH_SCHEMA_VERSION,
            "randomized batch kind/schema drift"
        );
        self.randomization_spec.validate()?;
        ensure!(
            self.fixed_delta_s == MOBILITY_RANDOMIZED_FIXED_DELTA_S
                && self.steps == MOBILITY_RANDOMIZED_STEPS,
            "randomized runtime drift"
        );
        ensure!(
            !self.lanes.is_empty() && self.lanes.len() <= 4_096,
            "invalid randomized batch width"
        );
        for (index, lane) in self.lanes.iter().enumerate() {
            ensure!(lane.lane_id == index as u64, "randomized lane order drift");
            ensure!(
                lane.episode_index == self.episode_index,
                "episode index drift"
            );
            let expected = run_lane(
                self.randomization_spec,
                self.root_seed,
                lane.lane_id,
                self.episode_index,
            )?;
            ensure!(*lane == expected, "randomized lane replay drift");
        }
        ensure!(
            self.passed == self.lanes.iter().all(|lane| lane.passed),
            "randomized batch verdict drift"
        );
        ensure!(
            self.content_digest == batch_digest(self)?,
            "randomized batch digest drift"
        );
        Ok(())
    }
}

/// Runs a bounded deterministic batch in stable lane order.
pub fn run_mobility_randomized_batch(
    root_seed: u64,
    episode_index: u64,
    num_envs: usize,
) -> Result<MobilityRandomizedBatchReport> {
    ensure!(
        (1..=4_096).contains(&num_envs),
        "randomized batch width must be 1..=4096"
    );
    let randomization_spec = MobilityRandomizationSpec::training_v1();
    randomization_spec.validate()?;
    let lanes = (0..num_envs)
        .map(|lane| run_lane(randomization_spec, root_seed, lane as u64, episode_index))
        .collect::<Result<Vec<_>>>()?;
    let mut report = MobilityRandomizedBatchReport {
        kind: MOBILITY_RANDOMIZED_BATCH_KIND.into(),
        schema_version: MOBILITY_RANDOMIZED_BATCH_SCHEMA_VERSION,
        root_seed,
        episode_index,
        randomization_spec,
        fixed_delta_s: MOBILITY_RANDOMIZED_FIXED_DELTA_S,
        steps: MOBILITY_RANDOMIZED_STEPS,
        passed: lanes.iter().all(|lane| lane.passed),
        lanes,
        content_digest: String::new(),
    };
    report.content_digest = batch_digest(&report)?;
    report.validate()?;
    Ok(report)
}

fn apply(sample: MobilityRandomizationSample) -> RandomizedMobilityProfile {
    let mut plant = plant_spec(1.0, DcMotorFailureMode::Nominal);
    plant.vehicle_mass_kg *= sample.vehicle_mass_scale;
    plant.normal_load_per_driven_wheel_n *= sample.vehicle_mass_scale;
    plant.motor.resistance_ohm *= sample.motor_resistance_scale;
    plant.motor.torque_constant_nm_a *= sample.motor_constant_scale;
    plant.motor.back_emf_constant_v_s_rad *= sample.motor_constant_scale;
    plant.motor.rotor_inertia_kg_m2 *= sample.rotor_inertia_scale;
    plant.transmission.ratio_motor_rad_per_wheel_rad *= sample.transmission_ratio_scale;
    plant.transmission.drive_efficiency_ratio *= sample.transmission_efficiency_scale;
    plant.transmission.backdrive_efficiency_ratio *= sample.transmission_efficiency_scale;
    plant.wheel.radius_m *= sample.wheel_radius_scale;
    plant.wheel.inertia_kg_m2 *= sample.wheel_inertia_scale;
    plant.wheel.rolling_resistance_coefficient *= sample.rolling_resistance_scale;
    plant.tire.reference_load_n *= sample.vehicle_mass_scale;
    plant.tire.longitudinal_stiffness_n *= sample.tire_stiffness_scale;
    plant.tire.lateral_stiffness_n *= sample.tire_stiffness_scale;
    plant.tire.longitudinal_peak_friction *= sample.tire_peak_friction_scale;
    plant.tire.lateral_peak_friction *= sample.tire_peak_friction_scale;
    plant.road_friction_scale *= sample.road_friction_scale;
    plant.road_grade_rad = sample.road_grade_rad;
    let mut suspension = suspension_spec();
    suspension.stiffness_n_per_m *= sample.suspension_stiffness_scale;
    suspension.damping_n_s_per_m *= sample.suspension_damping_scale;
    RandomizedMobilityProfile {
        sample,
        plant,
        suspension,
    }
}

fn run_lane(
    spec: MobilityRandomizationSpec,
    root_seed: u64,
    lane_id: u64,
    episode_index: u64,
) -> Result<MobilityRandomizedLane> {
    let episode_seed = derive_episode_seed(root_seed, lane_id, episode_index);
    let profile = spec.sample(episode_seed);
    ensure!(
        profile.plant.is_valid() && profile.suspension.is_valid(),
        "invalid applied randomized profile"
    );
    let mut state = LongitudinalMobilityPlantState::default();
    let mut maximum_friction_utilization = 0.0_f64;
    let mut final_current_a = 0.0;
    for _ in 0..MOBILITY_RANDOMIZED_STEPS {
        let evaluation = evaluate_longitudinal_mobility_plant(
            profile.plant,
            state,
            18.0,
            MOBILITY_RANDOMIZED_FIXED_DELTA_S,
        )?;
        state = evaluation.state;
        final_current_a = evaluation.motor.state.current_a;
        maximum_friction_utilization =
            maximum_friction_utilization.max(evaluation.tire.friction_utilization);
    }
    let passed = state.position_m.is_finite()
        && state.velocity_m_s.is_finite()
        && state.wheel_velocity_rad_s.is_finite()
        && final_current_a.is_finite()
        && state.position_m > 0.0
        && (0.0..=1.0).contains(&maximum_friction_utilization);
    let mut lane = MobilityRandomizedLane {
        lane_id,
        episode_index,
        episode_seed,
        profile,
        final_position_m: state.position_m,
        final_velocity_m_s: state.velocity_m_s,
        final_wheel_velocity_rad_s: state.wheel_velocity_rad_s,
        final_current_a,
        maximum_friction_utilization,
        passed,
        content_digest: String::new(),
    };
    lane.content_digest = lane_digest(&lane)?;
    Ok(lane)
}

fn range(minimum: f64, maximum: f64) -> UniformRange {
    UniformRange { minimum, maximum }
}

fn lane_digest(lane: &MobilityRandomizedLane) -> Result<String> {
    let mut canonical = lane.clone();
    canonical.content_digest.clear();
    digest(&canonical)
}
fn batch_digest(report: &MobilityRandomizedBatchReport) -> Result<String> {
    let mut canonical = report.clone();
    canonical.content_digest.clear();
    digest(&canonical)
}
fn digest<T: Serialize>(value: &T) -> Result<String> {
    let mut hash = 0xcbf29ce484222325_u64;
    for byte in serde_json::to_vec(value)? {
        hash ^= u64::from(byte);
        hash = hash.wrapping_mul(0x100000001b3);
    }
    Ok(format!("fnv1a64:{hash:016x}"))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn randomized_batch_is_repeatable_diverse_and_width_independent() {
        let narrow = run_mobility_randomized_batch(42, 3, 2).unwrap();
        let wide = run_mobility_randomized_batch(42, 3, 8).unwrap();
        assert!(narrow.passed && wide.passed);
        assert_eq!(narrow.lanes, wide.lanes[..2]);
        assert_ne!(wide.lanes[0].profile, wide.lanes[1].profile);
        assert_eq!(wide, run_mobility_randomized_batch(42, 3, 8).unwrap());
        wide.validate().unwrap();
    }

    #[test]
    fn sample_and_result_tampering_fail_replay_validation() {
        let mut report = run_mobility_randomized_batch(7, 0, 3).unwrap();
        report.lanes[1].profile.sample.vehicle_mass_scale += 0.01;
        assert!(report.validate().is_err());
        let mut report = run_mobility_randomized_batch(7, 0, 3).unwrap();
        report.lanes[1].final_velocity_m_s += 0.01;
        assert!(report.validate().is_err());
    }

    #[test]
    fn invalid_range_corner_is_rejected_before_sampling() {
        let mut spec = MobilityRandomizationSpec::training_v1();
        spec.transmission_efficiency_scale = range(0.9, 2.0);
        assert!(spec.validate().is_err());
    }
}
