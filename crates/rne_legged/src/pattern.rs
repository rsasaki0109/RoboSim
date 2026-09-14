//! Assembly of a full walking pattern from footsteps and ZMP preview control.

use crate::error::LeggedError;
use crate::footstep::{plan_straight_walk, FootstepPlan, StraightWalkRequest};
use crate::horizontal::Horizontal;
use crate::lipm::{capture_point, LimpParams, LimpState};
use crate::preview::ZmpPreviewController;
use serde::{Deserialize, Serialize};
use std::ops::Sub;

/// A sampled center-of-mass and ZMP trajectory for a footstep plan.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct WalkingPattern {
    /// Sampling period in seconds.
    pub sample_time_s: f64,
    /// Constant center-of-mass height used by the LIPM, in meters.
    pub com_height_m: f64,
    /// Center-of-mass horizontal samples in meters.
    pub com_m: Vec<Horizontal>,
    /// Center-of-mass horizontal velocity samples in meters per second.
    pub com_velocity_m_s: Vec<Horizontal>,
    /// Realized Zero Moment Point samples in meters.
    pub zmp_m: Vec<Horizontal>,
    /// Reference Zero Moment Point samples in meters.
    pub reference_zmp_m: Vec<Horizontal>,
    /// Footstep plan the trajectory follows.
    pub plan: FootstepPlan,
}

impl WalkingPattern {
    /// Number of trajectory samples.
    pub fn sample_count(&self) -> usize {
        self.com_m.len()
    }

    /// Total pattern duration in seconds.
    pub fn duration_s(&self) -> f64 {
        self.plan.duration_s
    }

    /// Maximum horizontal distance between the realized and reference ZMP.
    pub fn max_zmp_tracking_error_m(&self) -> f64 {
        self.zmp_m
            .iter()
            .zip(&self.reference_zmp_m)
            .map(|(realized, reference)| realized.sub(*reference).norm())
            .fold(0.0_f64, f64::max)
    }

    /// Maximum ZMP tracking error after `time_s`, which isolates the steady
    /// gait from the initial weight-shift transient.
    pub fn max_zmp_tracking_error_after_m(&self, time_s: f64) -> f64 {
        self.zmp_m
            .iter()
            .zip(&self.reference_zmp_m)
            .enumerate()
            .filter(|(index, _)| *index as f64 * self.sample_time_s > time_s)
            .map(|(_, (realized, reference))| realized.sub(*reference).norm())
            .fold(0.0_f64, f64::max)
    }

    /// Maximum center-of-mass speed over the pattern, in meters per second.
    pub fn max_com_speed_m_s(&self) -> f64 {
        self.com_velocity_m_s
            .iter()
            .map(|velocity| velocity.norm())
            .fold(0.0_f64, f64::max)
    }

    /// Maximum Divergent Component of Motion magnitude relative to the start,
    /// which bounds the walking excursion.
    pub fn max_dcm_offset_m(&self, params: &LimpParams) -> f64 {
        let origin = self.com_m.first().copied().unwrap_or_default();
        self.com_m
            .iter()
            .zip(&self.com_velocity_m_s)
            .map(|(com, velocity)| {
                let dcm = capture_point(params, &LimpState::new(*com, *velocity));
                dcm.sub(origin).norm()
            })
            .fold(0.0_f64, f64::max)
    }
}

/// Plans a straight walk and generates a center-of-mass trajectory that tracks
/// its Zero Moment Point reference with the given preview controller.
pub fn plan_walking_pattern(
    params: &LimpParams,
    controller: &ZmpPreviewController,
    request: &StraightWalkRequest,
) -> Result<WalkingPattern, LeggedError> {
    let plan = plan_straight_walk(params, request)?;
    let sample_time_s = controller.sample_time_s();
    let reference = plan.zmp_reference(sample_time_s);
    if reference.is_empty() {
        return Err(LeggedError::EmptyPlan);
    }

    let omega_squared = params.gravity_m_s2 / params.com_height_m;
    let start = plan.initial_com_m;
    let first = reference[0];
    let initial_x = [start.x_m, 0.0, omega_squared * (start.x_m - first.x_m)];
    let initial_z = [start.z_m, 0.0, omega_squared * (start.z_m - first.z_m)];

    let x_reference: Vec<f64> = reference.iter().map(|value| value.x_m).collect();
    let z_reference: Vec<f64> = reference.iter().map(|value| value.z_m).collect();
    let x_trajectory = controller.track_axis(initial_x, &x_reference);
    let z_trajectory = controller.track_axis(initial_z, &z_reference);

    let sample_count = reference.len();
    let mut com_m = Vec::with_capacity(sample_count);
    let mut com_velocity_m_s = Vec::with_capacity(sample_count);
    let mut zmp_m = Vec::with_capacity(sample_count);
    for index in 0..sample_count {
        let com = Horizontal::new(x_trajectory.com_m[index], z_trajectory.com_m[index]);
        let velocity =
            Horizontal::new(x_trajectory.states[index][1], z_trajectory.states[index][1]);
        let zmp = Horizontal::new(x_trajectory.zmp_m[index], z_trajectory.zmp_m[index]);
        if !com.is_finite() || !velocity.is_finite() || !zmp.is_finite() {
            return Err(LeggedError::NonFiniteTrajectory);
        }
        com_m.push(com);
        com_velocity_m_s.push(velocity);
        zmp_m.push(zmp);
    }

    Ok(WalkingPattern {
        sample_time_s,
        com_height_m: params.com_height_m,
        com_m,
        com_velocity_m_s,
        zmp_m,
        reference_zmp_m: reference,
        plan,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::footstep::GaitSchedule;

    fn request() -> StraightWalkRequest {
        StraightWalkRequest {
            direction: Horizontal::new(1.0, 0.0),
            start_com_m: Horizontal::ZERO,
            steps: 4,
            step_length_m: 0.2,
            step_width_m: 0.2,
            schedule: GaitSchedule::default(),
        }
    }

    fn pattern() -> WalkingPattern {
        let params = LimpParams::new(0.30, 9.81);
        let controller =
            ZmpPreviewController::new(params, 0.005, 1.0, 1.0e-7, 400).expect("controller");
        plan_walking_pattern(&params, &controller, &request()).expect("pattern")
    }

    #[test]
    fn pattern_samples_cover_the_plan_and_advance_forward() {
        let pattern = pattern();
        assert!(pattern.sample_count() > 0);
        assert!(pattern.com_m.iter().all(|value| value.is_finite()));
        // The CoM advances along the walking direction.
        let first = pattern.com_m.first().copied().unwrap_or_default();
        let last = pattern.com_m.last().copied().unwrap_or_default();
        assert!(last.x_m > first.x_m + 0.2);
    }

    #[test]
    fn zmp_tracking_error_is_bounded() {
        let pattern = pattern();
        // The initial weight shift carries a transient; the steady gait should
        // track the reference ZMP within a centimeter.
        assert!(pattern.max_zmp_tracking_error_m() < 0.08);
        let steady = pattern.max_zmp_tracking_error_after_m(1.0);
        assert!(steady < 0.02, "steady zmp tracking error {steady} m");
    }

    #[test]
    fn pattern_is_deterministic_and_bounded() {
        let first = pattern();
        let second = pattern();
        assert_eq!(first.com_m, second.com_m);
        assert_eq!(first.zmp_m, second.zmp_m);
        assert!(first.max_dcm_offset_m(&LimpParams::new(0.30, 9.81)) < 1.0);
    }
}
