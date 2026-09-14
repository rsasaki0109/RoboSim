//! Linear Inverted Pendulum Model, Divergent Component of Motion, and capture
//! point foot placement.

use crate::horizontal::Horizontal;
use serde::{Deserialize, Serialize};
use std::ops::{Add, Sub};

/// Linear Inverted Pendulum Model parameters for a constant center-of-mass
/// height.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct LimpParams {
    /// Constant center-of-mass height above the floor, in meters.
    pub com_height_m: f64,
    /// Gravitational acceleration magnitude, in meters per second squared.
    pub gravity_m_s2: f64,
}

impl Default for LimpParams {
    fn default() -> Self {
        Self {
            com_height_m: 0.30,
            gravity_m_s2: 9.81,
        }
    }
}

impl LimpParams {
    /// Creates LIPM parameters.
    pub fn new(com_height_m: f64, gravity_m_s2: f64) -> Self {
        Self {
            com_height_m,
            gravity_m_s2,
        }
    }

    /// Natural pendulum frequency `sqrt(g / h)` in radians per second.
    pub fn omega_rad_s(&self) -> f64 {
        (self.gravity_m_s2 / self.com_height_m).sqrt()
    }

    /// Returns whether the height and gravity are positive and finite.
    pub fn is_valid(&self) -> bool {
        self.com_height_m.is_finite()
            && self.com_height_m > 0.0
            && self.gravity_m_s2.is_finite()
            && self.gravity_m_s2 > 0.0
    }
}

/// Planar center-of-mass state of the LIPM.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct LimpState {
    /// Center-of-mass horizontal position in meters.
    pub com_m: Horizontal,
    /// Center-of-mass horizontal velocity in meters per second.
    pub com_velocity_m_s: Horizontal,
}

impl LimpState {
    /// Creates a LIPM state.
    pub fn new(com_m: Horizontal, com_velocity_m_s: Horizontal) -> Self {
        Self {
            com_m,
            com_velocity_m_s,
        }
    }
}

/// Divergent Component of Motion (capture point) `xi = com + com_vel / omega`.
pub fn capture_point(params: &LimpParams, state: &LimpState) -> Horizontal {
    let omega = params.omega_rad_s();
    state.com_m.add(state.com_velocity_m_s.scale(1.0 / omega))
}

/// Recovers the center-of-mass velocity from a DCM: `com_vel = omega * (xi - com)`.
pub fn com_velocity_from_dcm(
    params: &LimpParams,
    com_m: Horizontal,
    dcm_m: Horizontal,
) -> Horizontal {
    dcm_m.sub(com_m).scale(params.omega_rad_s())
}

/// Advances a DCM under a constant Zero Moment Point over `duration_s`.
///
/// Uses the closed-form solution `xi(t) = zmp + (xi0 - zmp) * exp(omega * t)`.
pub fn dcm_step(
    params: &LimpParams,
    dcm_m: Horizontal,
    zmp_m: Horizontal,
    duration_s: f64,
) -> Horizontal {
    let decay = (params.omega_rad_s() * duration_s).exp();
    zmp_m.add(dcm_m.sub(zmp_m).scale(decay))
}

/// Advances the full LIPM state under a constant Zero Moment Point.
///
/// Returns the state and the resulting DCM at the end of the interval.
pub fn propagate_constant_zmp(
    params: &LimpParams,
    state: &LimpState,
    zmp_m: Horizontal,
    duration_s: f64,
) -> (LimpState, Horizontal) {
    let omega = params.omega_rad_s();
    let angle = omega * duration_s;
    let (sinh, cosh) = (angle.sinh(), angle.cosh());

    let position = zmp_m.add(
        state
            .com_m
            .sub(zmp_m)
            .scale(cosh)
            .add(state.com_velocity_m_s.scale(sinh / omega)),
    );
    let velocity = state
        .com_m
        .sub(zmp_m)
        .scale(omega * sinh)
        .add(state.com_velocity_m_s.scale(cosh));
    let end = LimpState::new(position, velocity);
    let dcm = capture_point(params, &end);
    (end, dcm)
}

/// Computes the footstep that drives the current DCM to `desired_dcm_end_m`
/// over a constant-ZMP single-support interval.
///
/// Solving `xi(T) = p + (xi0 - p) * exp(omega T)` for the ZMP `p` gives the
/// capture-point foot placement. The result is the foot position, not a ZMP:
/// during single support the ZMP coincides with the stance foot.
pub fn footstep_from_dcm(
    params: &LimpParams,
    dcm_m: Horizontal,
    desired_dcm_end_m: Horizontal,
    duration_s: f64,
) -> Horizontal {
    let decay = (params.omega_rad_s() * duration_s).exp();
    let denominator = decay - 1.0;
    if denominator.abs() <= 1.0e-12 {
        return desired_dcm_end_m;
    }
    Horizontal::new(
        (dcm_m.x_m * decay - desired_dcm_end_m.x_m) / denominator,
        (dcm_m.z_m * decay - desired_dcm_end_m.z_m) / denominator,
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn capture_point_is_com_plus_velocity_over_omega() {
        let params = LimpParams::new(0.25, 10.0);
        // omega = sqrt(10 / 0.25) = sqrt(40)
        let state = LimpState::new(Horizontal::new(0.1, 0.0), Horizontal::new(0.2, 0.0));
        let dcm = capture_point(&params, &state);
        assert_relative_eq!(dcm.x_m, 0.1 + 0.2 / 40.0_f64.sqrt(), epsilon = 1.0e-12);
    }

    #[test]
    fn dcm_step_matches_analytic_solution() {
        let params = LimpParams::new(0.3, 9.81);
        let dcm = Horizontal::new(0.05, -0.02);
        let zmp = Horizontal::new(0.1, 0.01);
        let duration = 0.3;
        let decay = (params.omega_rad_s() * duration).exp();
        let expected = zmp.add(dcm.sub(zmp).scale(decay));
        let actual = dcm_step(&params, dcm, zmp, duration);
        assert_relative_eq!(actual.x_m, expected.x_m, epsilon = 1.0e-12);
        assert_relative_eq!(actual.z_m, expected.z_m, epsilon = 1.0e-12);
    }

    #[test]
    fn placing_the_capture_point_brings_the_com_to_rest() {
        let params = LimpParams::new(0.3, 9.81);
        let state = LimpState::new(Horizontal::new(0.0, 0.0), Horizontal::new(0.25, 0.0));
        let dcm = capture_point(&params, &state);
        // Place the foot exactly at the capture point.
        let foot = dcm;
        let (end, _) = propagate_constant_zmp(&params, &state, foot, 2.0);
        // The CoM converges on the foot and the velocity decays.
        assert_relative_eq!(end.com_m.x_m, foot.x_m, epsilon = 2.0e-3);
        assert!(end.com_velocity_m_s.norm() < 2.0e-2);
    }

    #[test]
    fn footstep_from_dcm_inverts_dcm_step() {
        let params = LimpParams::new(0.3, 9.81);
        let dcm0 = Horizontal::new(-0.05, 0.02);
        let desired = Horizontal::new(0.12, -0.03);
        let duration = 0.25;
        let foot = footstep_from_dcm(&params, dcm0, desired, duration);
        let realized = dcm_step(&params, dcm0, foot, duration);
        assert_relative_eq!(realized.x_m, desired.x_m, epsilon = 1.0e-10);
        assert_relative_eq!(realized.z_m, desired.z_m, epsilon = 1.0e-10);
    }
}
