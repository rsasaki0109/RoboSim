//! Domain randomization helpers for built-in environments.

use crate::rng::DeterministicRng;
use rne_math::Vec3;

/// Optional uniform ranges applied on each episode reset.
#[derive(Clone, Debug, PartialEq)]
pub struct DiffDriveDomainRandomization {
    /// Uniform range for initial base X in meters.
    pub initial_x_m: Option<(f64, f64)>,
    /// Uniform range for initial base Y in meters.
    pub initial_y_m: Option<(f64, f64)>,
    /// Uniform range for goal X in meters.
    pub goal_x_m: Option<(f64, f64)>,
}

impl DiffDriveDomainRandomization {
    /// Randomizes start pose and goal distance for forward-drive training.
    pub fn forward_goal_training() -> Self {
        Self {
            initial_x_m: Some((-0.2, 0.2)),
            initial_y_m: None,
            goal_x_m: Some((1.5, 2.5)),
        }
    }

    /// Applies configured ranges using the given RNG.
    pub fn apply(
        &self,
        rng: &mut DeterministicRng,
        initial_translation_m: &mut Vec3,
        goal_x_m: &mut f64,
    ) {
        if let Some((min, max)) = self.initial_x_m {
            initial_translation_m.x = rng.uniform_f64(min, max);
        }
        if let Some((min, max)) = self.initial_y_m {
            initial_translation_m.y = rng.uniform_f64(min, max);
        }
        if let Some((min, max)) = self.goal_x_m {
            *goal_x_m = rng.uniform_f64(min, max);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn domain_randomization_is_repeatable() {
        let dr = DiffDriveDomainRandomization::forward_goal_training();
        let mut rng_a = DeterministicRng::new(7);
        let mut rng_b = DeterministicRng::new(7);
        let mut initial_a = Vec3::new(0.0, 0.25, 0.0);
        let mut initial_b = initial_a;
        let mut goal_a = 2.0;
        let mut goal_b = 2.0;

        dr.apply(&mut rng_a, &mut initial_a, &mut goal_a);
        dr.apply(&mut rng_b, &mut initial_b, &mut goal_b);

        assert_eq!(initial_a, initial_b);
        assert_eq!(goal_a, goal_b);
    }
}
