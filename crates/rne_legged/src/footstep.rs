//! Footstep plans, gait schedules, and their Zero Moment Point reference.

use crate::error::LeggedError;
use crate::horizontal::Horizontal;
use crate::lipm::LimpParams;
use serde::{Deserialize, Serialize};
use std::ops::{Add, Sub};

/// Which leg a footstep belongs to.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum FootSide {
    /// Left foot.
    Left,
    /// Right foot.
    Right,
}

/// A single planted footstep.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Footstep {
    /// Foot position in the horizontal plane, in meters.
    pub position_m: Horizontal,
    /// Which foot lands here.
    pub side: FootSide,
}

/// Timing of the walking gait.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct GaitSchedule {
    /// Duration of one single-support interval in seconds.
    pub single_support_s: f64,
    /// Duration of each intermediate double-support transition in seconds.
    pub double_support_s: f64,
    /// Duration of the initial and final weight-shift transitions in seconds.
    pub weight_shift_s: f64,
    /// Extra double-support settle time appended after the last step, in seconds.
    pub settle_s: f64,
}

impl Default for GaitSchedule {
    fn default() -> Self {
        Self {
            single_support_s: 0.30,
            double_support_s: 0.12,
            weight_shift_s: 0.40,
            settle_s: 0.40,
        }
    }
}

impl GaitSchedule {
    /// Returns whether every duration is finite and non-negative and the
    /// single-support interval is positive.
    pub fn is_valid(&self) -> bool {
        self.single_support_s.is_finite()
            && self.single_support_s > 0.0
            && self.double_support_s.is_finite()
            && self.double_support_s >= 0.0
            && self.weight_shift_s.is_finite()
            && self.weight_shift_s >= 0.0
            && self.settle_s.is_finite()
            && self.settle_s >= 0.0
    }
}

/// Request for a straight-line walking plan.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct StraightWalkRequest {
    /// Horizontal walking direction; normalized internally.
    pub direction: Horizontal,
    /// Starting center-of-mass position in meters.
    pub start_com_m: Horizontal,
    /// Number of steps to take.
    pub steps: usize,
    /// Forward distance advanced by each step in meters.
    pub step_length_m: f64,
    /// Lateral distance between the two feet in meters.
    pub step_width_m: f64,
    /// Gait timing.
    pub schedule: GaitSchedule,
}

/// A Zero Moment Point reference segment.
///
/// During single support the segment holds one stance foot. During double
/// support it moves smoothly between the two consecutive stance feet using a
/// cubic smoothstep, so the reference is continuous and differentiable at the
/// segment boundaries and therefore trackable by preview control.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct ZmpSegment {
    /// ZMP at the start of the segment, in meters.
    pub start_m: Horizontal,
    /// ZMP at the end of the segment, in meters.
    pub end_m: Horizontal,
    /// Segment duration in seconds.
    pub duration_s: f64,
}

impl ZmpSegment {
    /// A constant (hold) segment.
    pub fn hold(zmp_m: Horizontal, duration_s: f64) -> Self {
        Self {
            start_m: zmp_m,
            end_m: zmp_m,
            duration_s,
        }
    }

    /// A smooth transition between two ZMP locations.
    pub fn transition(start_m: Horizontal, end_m: Horizontal, duration_s: f64) -> Self {
        Self {
            start_m,
            end_m,
            duration_s,
        }
    }

    /// Samples the segment at `time_s` from its start.
    pub fn sample(&self, time_s: f64) -> Horizontal {
        if self.duration_s <= 0.0 {
            return self.end_m;
        }
        let u = (time_s / self.duration_s).clamp(0.0, 1.0);
        let smooth = u * u * (3.0 - 2.0 * u);
        self.start_m.lerp(self.end_m, smooth)
    }
}

/// A planned sequence of footsteps and its ZMP reference.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct FootstepPlan {
    /// Landing order, starting with the two initial feet.
    pub footsteps: Vec<Footstep>,
    /// ZMP reference segments in time order.
    pub zmp_segments: Vec<ZmpSegment>,
    /// Initial center-of-mass position in meters.
    pub initial_com_m: Horizontal,
    /// Total plan duration in seconds.
    pub duration_s: f64,
}

impl FootstepPlan {
    /// Samples the ZMP reference at `sample_time_s`.
    ///
    /// The returned length is `floor(duration / sample_time) + 1`. The final
    /// sample holds the last segment's ZMP.
    pub fn zmp_reference(&self, sample_time_s: f64) -> Vec<Horizontal> {
        if self.zmp_segments.is_empty() || !sample_time_s.is_finite() || sample_time_s <= 0.0 {
            return Vec::new();
        }
        let sample_count = (self.duration_s / sample_time_s).floor() as usize + 1;
        let mut reference = Vec::with_capacity(sample_count);
        let mut segment_index = 0;
        let mut segment_start = 0.0;
        for index in 0..sample_count {
            let time = index as f64 * sample_time_s;
            while segment_index + 1 < self.zmp_segments.len()
                && time >= segment_start + self.zmp_segments[segment_index].duration_s
            {
                segment_start += self.zmp_segments[segment_index].duration_s;
                segment_index += 1;
            }
            reference.push(self.zmp_segments[segment_index].sample(time - segment_start));
        }
        reference
    }
}

/// Plans a nominal straight-line walk.
///
/// The feet alternate half a step apart around the walking direction. Each step
/// is a single-support interval on the planted stance foot, followed by a short
/// double-support transition whose ZMP moves smoothly to the next stance foot.
pub fn plan_straight_walk(
    params: &LimpParams,
    request: &StraightWalkRequest,
) -> Result<FootstepPlan, LeggedError> {
    if !params.is_valid() {
        return Err(LeggedError::InvalidLimp {
            com_height_m: params.com_height_m,
            gravity_m_s2: params.gravity_m_s2,
        });
    }
    if request.steps == 0 {
        return Err(LeggedError::EmptyPlan);
    }
    if !request.direction.is_finite() || request.direction.norm() <= 1.0e-9 {
        return Err(LeggedError::InvalidRequest(
            "walking direction must be non-zero and finite",
        ));
    }
    if !request.step_length_m.is_finite() || request.step_length_m <= 0.0 {
        return Err(LeggedError::InvalidRequest(
            "step length must be positive and finite",
        ));
    }
    if !request.step_width_m.is_finite() || request.step_width_m <= 0.0 {
        return Err(LeggedError::InvalidRequest(
            "step width must be positive and finite",
        ));
    }
    if !request.schedule.is_valid() {
        return Err(LeggedError::InvalidRequest("gait schedule is invalid"));
    }
    if !request.start_com_m.is_finite() {
        return Err(LeggedError::InvalidRequest(
            "start center of mass must be finite",
        ));
    }

    let direction = request.direction.normalize_or_zero();
    let lateral = direction.perpendicular_left();
    let half_width = lateral.scale(request.step_width_m * 0.5);

    let mut left: Vec<Horizontal> = vec![request.start_com_m.add(half_width)];
    let mut right: Vec<Horizontal> = vec![request.start_com_m.sub(half_width)];
    let mut footsteps = vec![
        Footstep {
            position_m: left[0],
            side: FootSide::Left,
        },
        Footstep {
            position_m: right[0],
            side: FootSide::Right,
        },
    ];
    for index in 1..=request.steps {
        if index % 2 == 1 {
            let next = right[index - 1].add(direction.scale(request.step_length_m));
            right.push(next);
            left.push(left[index - 1]);
            footsteps.push(Footstep {
                position_m: next,
                side: FootSide::Right,
            });
        } else {
            let next = left[index - 1].add(direction.scale(request.step_length_m));
            left.push(next);
            right.push(right[index - 1]);
            footsteps.push(Footstep {
                position_m: next,
                side: FootSide::Left,
            });
        }
    }

    let schedule = request.schedule;
    let midpoint = |a: Horizontal, b: Horizontal| a.lerp(b, 0.5);
    // The stance foot during step `index` (1-based) is the foot that stays
    // planted while the other one swings.
    let stance = |index: usize| {
        if index % 2 == 1 {
            left[index - 1]
        } else {
            right[index - 1]
        }
    };
    let start_midpoint = midpoint(left[0], right[0]);
    let final_midpoint = midpoint(left[request.steps], right[request.steps]);

    let mut zmp_segments = vec![ZmpSegment::transition(
        start_midpoint,
        stance(1),
        schedule.weight_shift_s,
    )];
    let mut duration = zmp_segments[0].duration_s;
    for index in 1..=request.steps {
        zmp_segments.push(ZmpSegment::hold(stance(index), schedule.single_support_s));
        duration += schedule.single_support_s;
        let last_step = index == request.steps;
        let next = if last_step {
            final_midpoint
        } else {
            stance(index + 1)
        };
        let transition_s = if last_step {
            schedule.weight_shift_s
        } else {
            schedule.double_support_s
        };
        zmp_segments.push(ZmpSegment::transition(stance(index), next, transition_s));
        duration += transition_s;
    }
    zmp_segments.push(ZmpSegment::hold(final_midpoint, schedule.settle_s));
    duration += schedule.settle_s;

    if duration <= 0.0 {
        return Err(LeggedError::InvalidRequest(
            "plan duration must be positive",
        ));
    }

    Ok(FootstepPlan {
        footsteps,
        zmp_segments,
        initial_com_m: request.start_com_m,
        duration_s: duration,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

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

    #[test]
    fn straight_walk_advances_alternating_feet() {
        let plan = plan_straight_walk(&LimpParams::default(), &request()).expect("plan");
        // Two initial feet plus one landing per step.
        assert_eq!(plan.footsteps.len(), 6);
        let left_max = plan
            .footsteps
            .iter()
            .filter(|step| step.side == FootSide::Left)
            .map(|step| step.position_m.x_m)
            .fold(f64::MIN, f64::max);
        let right_max = plan
            .footsteps
            .iter()
            .filter(|step| step.side == FootSide::Right)
            .map(|step| step.position_m.x_m)
            .fold(f64::MIN, f64::max);
        assert!(left_max > 0.0 && right_max > 0.0);
    }

    #[test]
    fn plan_is_deterministic() {
        let first = plan_straight_walk(&LimpParams::default(), &request()).expect("plan");
        let second = plan_straight_walk(&LimpParams::default(), &request()).expect("plan");
        assert_eq!(first, second);
    }

    #[test]
    fn zmp_reference_is_continuous() {
        let plan = plan_straight_walk(&LimpParams::default(), &request()).expect("plan");
        let reference = plan.zmp_reference(0.001);
        assert_eq!(
            reference.len(),
            (plan.duration_s / 0.001).floor() as usize + 1
        );
        // Adjacent samples of a smoothstep transition differ by a bounded step.
        for pair in reference.windows(2) {
            assert!(pair[0].sub(pair[1]).norm() < 0.01);
        }
        assert_relative_eq!(reference[0].x_m, 0.0, epsilon = 1.0e-12);
        assert_relative_eq!(reference[0].z_m, 0.0, epsilon = 1.0e-12);
    }

    #[test]
    fn rejects_invalid_requests() {
        let params = LimpParams::default();
        let mut invalid = request();
        invalid.steps = 0;
        assert_eq!(
            plan_straight_walk(&params, &invalid),
            Err(LeggedError::EmptyPlan)
        );
        let mut invalid = request();
        invalid.direction = Horizontal::ZERO;
        assert!(plan_straight_walk(&params, &invalid).is_err());
        let mut invalid = request();
        invalid.step_length_m = -0.1;
        assert!(plan_straight_walk(&params, &invalid).is_err());
    }
}
