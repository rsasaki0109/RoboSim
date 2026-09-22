//! Standard microscopic car-following models.
//!
//! These are deterministic, unit-explicit implementations of the Intelligent
//! Driver Model (IDM) and the Krauss model, provided as pure functions so the
//! traffic runtime (which keeps its kinematic default for reproducibility) and
//! external consumers can select a standard longitudinal model. Every parameter
//! and observable carries its SI unit in the name.

/// Intelligent Driver Model parameters.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct IdmParams {
    /// Desired free-flow speed in meters per second.
    pub desired_speed_m_s: f64,
    /// Desired time headway in seconds.
    pub time_headway_s: f64,
    /// Minimum bumper-to-bumper gap in meters.
    pub minimum_gap_m: f64,
    /// Maximum acceleration in meters per second squared.
    pub max_acceleration_m_s2: f64,
    /// Comfortable deceleration in meters per second squared.
    pub comfortable_braking_m_s2: f64,
    /// Free-flow acceleration exponent (typically 4).
    pub exponent: f64,
}

impl Default for IdmParams {
    fn default() -> Self {
        Self {
            desired_speed_m_s: 13.89,
            time_headway_s: 1.5,
            minimum_gap_m: 2.0,
            max_acceleration_m_s2: 1.5,
            comfortable_braking_m_s2: 2.0,
            exponent: 4.0,
        }
    }
}

impl IdmParams {
    /// Returns true when every parameter is finite and physically valid.
    pub fn is_valid(&self) -> bool {
        self.desired_speed_m_s.is_finite()
            && self.desired_speed_m_s > 0.0
            && self.time_headway_s.is_finite()
            && self.time_headway_s >= 0.0
            && self.minimum_gap_m.is_finite()
            && self.minimum_gap_m >= 0.0
            && self.max_acceleration_m_s2.is_finite()
            && self.max_acceleration_m_s2 > 0.0
            && self.comfortable_braking_m_s2.is_finite()
            && self.comfortable_braking_m_s2 > 0.0
            && self.exponent.is_finite()
            && self.exponent > 0.0
    }
}

/// Longitudinal model selected by the traffic runtime.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub enum CarFollowingModel {
    /// The original deterministic kinematic target-speed rule.
    #[default]
    Kinematic,
    /// The Intelligent Driver Model.
    Idm(IdmParams),
}

impl CarFollowingModel {
    /// Returns true when the selected model is valid.
    pub fn is_valid(&self) -> bool {
        match self {
            Self::Kinematic => true,
            Self::Idm(params) => params.is_valid(),
        }
    }
}

/// IDM acceleration in meters per second squared.
///
/// `gap_m` is the bumper-to-bumper distance to the leader; `leader_speed_m_s` is
/// the leader's speed. A negative result brakes.
pub fn idm_acceleration(
    params: &IdmParams,
    speed_m_s: f64,
    leader_speed_m_s: f64,
    gap_m: f64,
) -> f64 {
    let speed = speed_m_s.max(0.0);
    let gap = gap_m.max(1.0e-3);
    let approach = speed - leader_speed_m_s.max(0.0);
    let braking_term = speed * params.time_headway_s
        + speed * approach
            / (2.0 * (params.max_acceleration_m_s2 * params.comfortable_braking_m_s2).sqrt());
    let desired_gap = params.minimum_gap_m + braking_term.max(0.0);
    let free_term = (speed / params.desired_speed_m_s).powf(params.exponent);
    let interaction_term = (desired_gap / gap).powi(2);
    params.max_acceleration_m_s2 * (1.0 - free_term - interaction_term)
}

/// Krauss model parameters.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct KraussParams {
    /// Maximum acceleration in meters per second squared.
    pub max_acceleration_m_s2: f64,
    /// Maximum safe braking in meters per second squared.
    pub max_braking_m_s2: f64,
    /// Driver reaction time in seconds.
    pub reaction_time_s: f64,
    /// Desired speed in meters per second.
    pub desired_speed_m_s: f64,
    /// Minimum bumper-to-bumper gap in meters.
    pub minimum_gap_m: f64,
}

impl Default for KraussParams {
    fn default() -> Self {
        Self {
            max_acceleration_m_s2: 1.5,
            max_braking_m_s2: 3.0,
            reaction_time_s: 1.0,
            desired_speed_m_s: 13.89,
            minimum_gap_m: 2.0,
        }
    }
}

impl KraussParams {
    /// Returns true when every parameter is finite and physically valid.
    pub fn is_valid(&self) -> bool {
        self.max_acceleration_m_s2.is_finite()
            && self.max_acceleration_m_s2 > 0.0
            && self.max_braking_m_s2.is_finite()
            && self.max_braking_m_s2 > 0.0
            && self.reaction_time_s.is_finite()
            && self.reaction_time_s >= 0.0
            && self.desired_speed_m_s.is_finite()
            && self.desired_speed_m_s > 0.0
            && self.minimum_gap_m.is_finite()
            && self.minimum_gap_m >= 0.0
    }
}

/// Maximum speed that still avoids colliding with a leader, in meters per second.
pub fn krauss_safe_speed(params: &KraussParams, leader_speed_m_s: f64, gap_m: f64) -> f64 {
    let brake_time = params.max_braking_m_s2 * params.reaction_time_s;
    let leader = leader_speed_m_s.max(0.0);
    let gap = gap_m.max(0.0);
    let safe = -brake_time
        + (brake_time * brake_time + leader * leader + 2.0 * params.max_braking_m_s2 * gap).sqrt();
    safe.max(0.0)
}

/// Next speed under the Krauss model, in meters per second.
pub fn krauss_new_speed(
    params: &KraussParams,
    speed_m_s: f64,
    leader_speed_m_s: f64,
    gap_m: f64,
    dt_s: f64,
) -> f64 {
    let safe = krauss_safe_speed(params, leader_speed_m_s, gap_m);
    let accelerate =
        (speed_m_s + params.max_acceleration_m_s2 * dt_s.max(0.0)).min(params.desired_speed_m_s);
    accelerate.min(safe).max(0.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn idm_accelerates_in_free_flow() {
        let params = IdmParams::default();
        let acceleration = idm_acceleration(&params, 0.0, 0.0, 1000.0);
        assert!(
            (acceleration - params.max_acceleration_m_s2).abs() < 1e-3,
            "acceleration {acceleration}"
        );
    }

    #[test]
    fn idm_brakes_for_a_slower_leader() {
        let params = IdmParams::default();
        let acceleration = idm_acceleration(&params, 10.0, 2.0, 5.0);
        assert!(acceleration < -1.0, "acceleration {acceleration}");
    }

    #[test]
    fn idm_is_near_equilibrium_at_the_desired_gap() {
        let params = IdmParams::default();
        let speed = 10.0;
        let gap = params.minimum_gap_m + speed * params.time_headway_s;
        let acceleration = idm_acceleration(&params, speed, speed, gap);
        // At the interaction equilibrium the free-flow term is the only residual.
        assert!(acceleration < 0.0 && acceleration > -params.max_acceleration_m_s2);
    }

    #[test]
    fn idm_overspeed_brakes() {
        let params = IdmParams::default();
        let acceleration = idm_acceleration(&params, 20.0, 20.0, 1000.0);
        assert!(acceleration < 0.0, "acceleration {acceleration}");
    }

    #[test]
    fn krauss_safe_speed_is_zero_for_a_wall() {
        let params = KraussParams::default();
        let safe = krauss_safe_speed(&params, 0.0, 0.0);
        assert!(safe.abs() < 1e-9, "safe {safe}");
    }

    #[test]
    fn krauss_new_speed_respects_the_safe_bound() {
        let params = KraussParams::default();
        let next = krauss_new_speed(&params, 20.0, 0.0, 1.0, 0.1);
        let safe = krauss_safe_speed(&params, 0.0, 1.0);
        assert!(next <= safe + 1e-9);
    }
}
