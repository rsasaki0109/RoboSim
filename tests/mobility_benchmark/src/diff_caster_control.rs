//! Sensor-only voltage control boundary for the differential-caster fixture.
//!
//! The planar convention is forward +x, left +y and counterclockwise yaw. With
//! the fixture's world axes this is world (x, z), with yaw opposite world +Y.
//! Inputs contain estimates, not a physics world, contact loads or caster truth.

use anyhow::{ensure, Result};
use rne_ai::{WheelImuOdometryEstimate, WheelImuOdometryHealth};
use rne_core::SimTime;
use serde::{Deserialize, Serialize};

/// Fixed controller gains and sensor-freshness deadline.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct DifferentialCasterControlSpec {
    /// Maximum age of the oldest measurement underlying an accepted estimate.
    pub maximum_sensor_age_ticks: u64,
    /// Symmetric terminal-voltage limit per driven wheel.
    pub maximum_voltage_v: f64,
    /// Forward-speed proportional gain in volt seconds per meter.
    pub speed_kp_v_s_m: f64,
    /// Forward-speed integral gain in volts per meter.
    pub speed_ki_v_m: f64,
    /// Yaw-rate proportional gain in volt seconds per radian.
    pub yaw_kp_v_s_rad: f64,
    /// Yaw-rate integral gain in volts per radian.
    pub yaw_ki_v_rad: f64,
    /// Nominal steady-state voltage per forward reference speed; zero disables it.
    #[serde(default)]
    pub speed_feedforward_v_s_m: f64,
    /// Nominal differential voltage per yaw-rate reference; zero disables it.
    #[serde(default)]
    pub yaw_feedforward_v_s_rad: f64,
}

impl Default for DifferentialCasterControlSpec {
    fn default() -> Self {
        Self {
            maximum_sensor_age_ticks: 32_000_000,
            maximum_voltage_v: 12.0,
            speed_kp_v_s_m: 12.0,
            speed_ki_v_m: 3.0,
            yaw_kp_v_s_rad: 4.0,
            yaw_ki_v_rad: 1.0,
            speed_feedforward_v_s_m: 0.0,
            yaw_feedforward_v_s_rad: 0.0,
        }
    }
}

/// Reason for the voltage returned on a simulation step.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DifferentialCasterControlStatus {
    /// No usable estimator update has arrived yet.
    AwaitingEstimate,
    /// The first counter pair establishes a baseline and does not drive.
    Initializing,
    /// A new sensor estimate produced a voltage command.
    Tracking,
    /// No new estimate; the existing command is still inside its age deadline.
    Holding,
    /// Measurement age exceeded its deadline; terminal voltage is zero.
    Expired,
}

/// A command for the next drive-path interval, not a completed physical stop.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DifferentialCasterControlOutput {
    /// Left and right terminal voltages. Zero voltage does not mean zero torque.
    pub voltage_v: [f64; 2],
    /// Why this command was selected.
    pub status: DifferentialCasterControlStatus,
}

#[derive(Clone, Copy, Debug)]
struct AcceptedEstimate {
    capture_ticks: u64,
    oldest_capture_ticks: u64,
    sequences: [u64; 3],
}

/// Deterministic PI controller with bounded sensor-age command holding.
///
/// Repeated estimates never renew the deadline or integrate the PI state. Missing
/// input holds only until the oldest source measurement exceeds its age limit.
/// Malformed input returns an error and clears the held voltage and integrators.
/// A subsequent fresh estimate may recover; this is not a latched emergency stop.
#[derive(Clone, Debug)]
pub struct DifferentialCasterController {
    spec: DifferentialCasterControlSpec,
    accepted: Option<AcceptedEstimate>,
    last_decision_ticks: Option<u64>,
    integral: [f64; 2],
    voltage_v: [f64; 2],
}

impl DifferentialCasterController {
    /// Creates an initially unpowered controller, rejecting invalid gains/limits.
    pub fn new(spec: DifferentialCasterControlSpec) -> Result<Self> {
        ensure!(spec.maximum_sensor_age_ticks > 0, "zero sensor-age limit");
        ensure!(
            spec.maximum_voltage_v.is_finite() && spec.maximum_voltage_v > 0.0,
            "invalid voltage limit"
        );
        ensure!(
            [
                spec.speed_kp_v_s_m,
                spec.speed_ki_v_m,
                spec.yaw_kp_v_s_rad,
                spec.yaw_ki_v_rad,
                spec.speed_feedforward_v_s_m,
                spec.yaw_feedforward_v_s_rad
            ]
            .iter()
            .all(|value| value.is_finite() && *value >= 0.0),
            "invalid controller gain"
        );
        Ok(Self {
            spec,
            accepted: None,
            last_decision_ticks: None,
            integral: [0.0; 2],
            voltage_v: [0.0; 2],
        })
    }

    /// Consumes only a sensor-derived estimate and a task velocity reference.
    ///
    /// Call once per fixed simulation step. The reference is `[forward m/s, yaw
    /// rad/s]`; the caller owns its command-delivery contract. Sensor freshness is
    /// independent of whether the reference was refreshed. IMU saturation is
    /// rejected here rather than silently running the closed loop on wheel yaw.
    pub fn update(
        &mut self,
        now: SimTime,
        estimate: Option<&WheelImuOdometryEstimate>,
        target_velocity: [f64; 2],
    ) -> Result<DifferentialCasterControlOutput> {
        let result = self.update_checked(now, estimate, target_velocity);
        if result.is_err() {
            self.voltage_v = [0.0; 2];
            self.integral = [0.0; 2];
        }
        result
    }

    fn update_checked(
        &mut self,
        now: SimTime,
        estimate: Option<&WheelImuOdometryEstimate>,
        target: [f64; 2],
    ) -> Result<DifferentialCasterControlOutput> {
        let now = now.ticks();
        ensure!(
            self.last_decision_ticks.is_none_or(|last| now > last),
            "nonadvancing controller time"
        );
        self.last_decision_ticks = Some(now);
        ensure!(
            target.iter().all(|value| value.is_finite()),
            "nonfinite target"
        );
        let expired = self.accepted.is_some_and(|old| {
            now.saturating_sub(old.oldest_capture_ticks) > self.spec.maximum_sensor_age_ticks
        });
        if expired {
            self.voltage_v = [0.0; 2];
            self.integral = [0.0; 2];
        }
        let mut status = if expired {
            DifferentialCasterControlStatus::Expired
        } else if self.accepted.is_some() {
            DifferentialCasterControlStatus::Holding
        } else {
            DifferentialCasterControlStatus::AwaitingEstimate
        };
        if let Some(estimate) = estimate {
            let p = estimate.provenance;
            ensure!(
                p.capture_ticks <= p.decision_ticks && p.decision_ticks <= now,
                "future estimate"
            );
            let oldest = p
                .decision_ticks
                .checked_sub(p.max_age_ticks)
                .ok_or_else(|| anyhow::anyhow!("invalid source age"))?;
            ensure!(oldest <= p.capture_ticks, "source age contradicts capture");
            ensure!(
                estimate.linear_velocity_m_s.is_finite()
                    && estimate.angular_velocity_rad_s.is_finite(),
                "nonfinite estimate"
            );
            ensure!(
                estimate.health != WheelImuOdometryHealth::ImuSaturated,
                "saturated IMU estimate"
            );
            let sequences = [p.left_sequence, p.right_sequence, p.imu_sequence];
            ensure!(
                sequences.iter().all(|sequence| *sequence > 0),
                "zero source sequence"
            );
            if let Some(previous) = self.accepted {
                if sequences == previous.sequences {
                    ensure!(
                        p.capture_ticks == previous.capture_ticks
                            && oldest == previous.oldest_capture_ticks,
                        "repeated sequence changed timing"
                    );
                    return Ok(DifferentialCasterControlOutput {
                        voltage_v: self.voltage_v,
                        status,
                    });
                }
                ensure!(
                    sequences
                        .iter()
                        .zip(previous.sequences)
                        .all(|(new, old)| *new > old)
                        && p.capture_ticks > previous.capture_ticks,
                    "nonadvancing source provenance"
                );
            }
            if now - oldest > self.spec.maximum_sensor_age_ticks {
                // Even a newly delivered sequence cannot refresh a stale command.
                self.voltage_v = [0.0; 2];
                self.integral = [0.0; 2];
                status = DifferentialCasterControlStatus::Expired;
            } else {
                let dt_s = if expired {
                    0.0
                } else {
                    self.accepted.map_or(0.0, |old| {
                        (p.capture_ticks - old.capture_ticks) as f64 / 1.0e9
                    })
                };
                self.accepted = Some(AcceptedEstimate {
                    capture_ticks: p.capture_ticks,
                    oldest_capture_ticks: oldest,
                    sequences,
                });
                if estimate.health == WheelImuOdometryHealth::Initializing {
                    self.voltage_v = [0.0; 2];
                    self.integral = [0.0; 2];
                    status = DifferentialCasterControlStatus::Initializing;
                } else {
                    let error = [
                        target[0] - estimate.linear_velocity_m_s,
                        target[1] - estimate.angular_velocity_rad_s,
                    ];
                    let candidate =
                        std::array::from_fn::<_, 2, _>(|i| self.integral[i] + error[i] * dt_s);
                    let common = self.spec.speed_feedforward_v_s_m * target[0]
                        + self.spec.speed_kp_v_s_m * error[0]
                        + self.spec.speed_ki_v_m * candidate[0];
                    let differential = self.spec.yaw_feedforward_v_s_rad * target[1]
                        + self.spec.yaw_kp_v_s_rad * error[1]
                        + self.spec.yaw_ki_v_rad * candidate[1];
                    let unconstrained = [common - differential, common + differential];
                    ensure!(
                        candidate
                            .iter()
                            .chain(unconstrained.iter())
                            .all(|v| v.is_finite()),
                        "controller arithmetic overflow"
                    );
                    // Freeze both integrators under coupled wheel-voltage saturation.
                    if unconstrained
                        .iter()
                        .all(|v| v.abs() <= self.spec.maximum_voltage_v)
                    {
                        self.integral = candidate;
                    }
                    self.voltage_v = unconstrained.map(|v| {
                        v.clamp(-self.spec.maximum_voltage_v, self.spec.maximum_voltage_v)
                    });
                    status = DifferentialCasterControlStatus::Tracking;
                }
            }
        }
        Ok(DifferentialCasterControlOutput {
            voltage_v: self.voltage_v,
            status,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rne_ai::WheelImuOdometryProvenance;
    use rne_data::PoseSample;

    fn time(ms: u64) -> SimTime {
        SimTime::from_ticks(ms * 1_000_000)
    }

    fn estimate(sequence: u64, capture_ms: u64) -> WheelImuOdometryEstimate {
        WheelImuOdometryEstimate {
            pose: PoseSample::default(),
            linear_velocity_m_s: 0.0,
            angular_velocity_rad_s: 0.0,
            encoder_delta_yaw_rad: 0.0,
            gyro_delta_yaw_rad: 0.0,
            yaw_innovation_rad: 0.0,
            health: WheelImuOdometryHealth::Nominal,
            provenance: WheelImuOdometryProvenance {
                left_sequence: sequence,
                right_sequence: sequence,
                imu_sequence: sequence,
                capture_ticks: time(capture_ms).ticks(),
                decision_ticks: time(capture_ms + 2).ticks(),
                max_age_ticks: time(2).ticks(),
                skipped_sequences: 0,
            },
            pose_covariance: [[0.0; 3]; 3],
        }
    }

    fn controller() -> DifferentialCasterController {
        DifferentialCasterController::new(DifferentialCasterControlSpec::default()).unwrap()
    }

    #[test]
    fn planar_forward_reverse_and_left_turn_have_consistent_voltage_signs() {
        let input = estimate(1, 10);
        let forward = controller()
            .update(time(12), Some(&input), [0.2, 0.0])
            .unwrap();
        assert_eq!(forward.voltage_v, [2.4000000000000004; 2]);
        let reverse = controller()
            .update(time(12), Some(&input), [-0.2, 0.0])
            .unwrap();
        assert_eq!(reverse.voltage_v, forward.voltage_v.map(|v| -v));
        let left = controller()
            .update(time(12), Some(&input), [0.0, 0.1])
            .unwrap();
        assert_eq!(left.voltage_v, [-0.4, 0.4]);
        let right = controller()
            .update(time(12), Some(&input), [0.0, -0.1])
            .unwrap();
        assert_eq!(right.voltage_v, [0.4, -0.4]);
    }

    #[test]
    fn oldest_source_time_bounds_hold_and_duplicate_does_not_renew_it() {
        let mut control = controller();
        assert_eq!(
            control.update(time(1), None, [0.2, 0.0]).unwrap().status,
            DifferentialCasterControlStatus::AwaitingEstimate
        );
        let mut input = estimate(1, 10);
        input.provenance.max_age_ticks = time(3).ticks(); // Oldest source at 9 ms.
        let first = control.update(time(12), Some(&input), [0.2, 0.0]).unwrap();
        assert_eq!(
            control
                .update(time(30), Some(&input), [0.9, 0.0])
                .unwrap()
                .voltage_v,
            first.voltage_v
        );
        assert_eq!(
            control
                .update(time(41), None, [0.9, 0.0])
                .unwrap()
                .voltage_v,
            first.voltage_v
        );
        let expired = control.update(time(42), Some(&input), [0.9, 0.0]).unwrap();
        assert_eq!(expired.status, DifferentialCasterControlStatus::Expired);
        assert_eq!(expired.voltage_v, [0.0; 2]);
        let fresh = estimate(2, 100);
        assert_eq!(
            control
                .update(time(102), Some(&fresh), [0.2, 0.0])
                .unwrap()
                .voltage_v,
            first.voltage_v
        );
    }

    #[test]
    fn initialization_and_duplicate_updates_never_integrate() {
        let mut control = controller();
        let mut initial = estimate(1, 10);
        initial.health = WheelImuOdometryHealth::Initializing;
        assert_eq!(
            control
                .update(time(12), Some(&initial), [0.2, 0.0])
                .unwrap()
                .voltage_v,
            [0.0; 2]
        );
        let next = estimate(2, 20);
        control.update(time(22), Some(&next), [0.2, 0.0]).unwrap();
        let mut without_duplicate = control.clone();
        let integral = control.integral;
        control.update(time(23), Some(&next), [0.2, 0.0]).unwrap();
        assert_eq!(control.integral, integral);
        let next = estimate(3, 30);
        assert_eq!(
            control.update(time(32), Some(&next), [0.2, 0.0]).unwrap(),
            without_duplicate
                .update(time(32), Some(&next), [0.2, 0.0])
                .unwrap()
        );
    }

    #[test]
    fn saturation_freezes_integrators_and_invalid_inputs_clear_voltage() {
        let mut control = controller();
        for sequence in 1..=20 {
            let input = estimate(sequence, sequence * 10);
            let output = control
                .update(time(sequence * 10 + 2), Some(&input), [100.0, 100.0])
                .unwrap();
            assert!(output.voltage_v.iter().all(|value| value.abs() <= 12.0));
            assert_eq!(control.integral, [0.0; 2]);
        }
        let mut bad = estimate(21, 210);
        bad.health = WheelImuOdometryHealth::ImuSaturated;
        assert!(control.update(time(212), Some(&bad), [0.2, 0.0]).is_err());
        assert_eq!(
            control
                .update(time(213), None, [0.2, 0.0])
                .unwrap()
                .voltage_v,
            [0.0; 2]
        );
        assert!(control.update(time(214), None, [f64::NAN, 0.0]).is_err());
        assert!(control.update(time(213), None, [0.0; 2]).is_err());
    }

    #[test]
    fn future_stale_and_mixed_provenance_cannot_drive() {
        let mut control = controller();
        let input = estimate(1, 10);
        assert!(control.update(time(11), Some(&input), [0.2, 0.0]).is_err());
        assert_eq!(
            control
                .update(time(50), Some(&input), [0.2, 0.0])
                .unwrap()
                .voltage_v,
            [0.0; 2]
        );
        let input = estimate(2, 60);
        control.update(time(62), Some(&input), [0.2, 0.0]).unwrap();
        let mut mixed = estimate(3, 70);
        mixed.provenance.imu_sequence = 2;
        assert!(control.update(time(72), Some(&mixed), [0.2, 0.0]).is_err());
        assert_eq!(control.voltage_v, [0.0; 2]);
        let mut changed = input;
        changed.provenance.capture_ticks += 1;
        assert!(control
            .update(time(73), Some(&changed), [0.2, 0.0])
            .is_err());
        assert!(
            DifferentialCasterController::new(DifferentialCasterControlSpec {
                maximum_sensor_age_ticks: 0,
                ..Default::default()
            })
            .is_err()
        );
    }

    #[test]
    fn feedforward_uses_reference_units_but_still_expires_without_sensors() {
        let mut control = DifferentialCasterController::new(DifferentialCasterControlSpec {
            speed_feedforward_v_s_m: 14.375,
            yaw_feedforward_v_s_rad: 4.3125,
            ..Default::default()
        })
        .unwrap();
        let mut input = estimate(1, 10);
        input.linear_velocity_m_s = 0.2;
        input.angular_velocity_rad_s = 0.1;
        let output = control.update(time(12), Some(&input), [0.2, 0.1]).unwrap();
        assert!((output.voltage_v[0] - 2.44375).abs() < 1.0e-12);
        assert!((output.voltage_v[1] - 3.30625).abs() < 1.0e-12);
        assert_eq!(control.integral, [0.0; 2]);
        let expired = control.update(time(43), None, [0.2, 0.1]).unwrap();
        assert_eq!(expired.status, DifferentialCasterControlStatus::Expired);
        assert_eq!(expired.voltage_v, [0.0; 2]);
        assert!(
            DifferentialCasterController::new(DifferentialCasterControlSpec {
                speed_feedforward_v_s_m: f64::NAN,
                ..Default::default()
            })
            .is_err()
        );
    }
}
