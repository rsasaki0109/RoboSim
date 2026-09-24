//! Keyframe joint trajectories (the RNE analogue of Choreonoid's `BodyMotion`).
//!
//! A [`BodyMotion`] is a deterministic, backend-neutral recording of joint
//! position keyframes. It can be sampled at any simulation time, optionally
//! looping, and evaluated with linear, smooth-step, or cubic Hermite
//! interpolation. No wall-clock time or random state is involved.

use bevy_ecs::prelude::World;
use rne_ecs::Entity;
use thiserror::Error;

/// Interpolation applied between adjacent keyframes.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum JointInterpolation {
    /// Piecewise-linear interpolation.
    #[default]
    Linear,
    /// Smooth-step (`3u² - 2u³`) interpolation with zero end slopes.
    SmoothStep,
    /// Cubic Hermite interpolation using finite-difference tangents.
    Cubic,
}

/// A single joint position keyframe.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct JointKeyframe {
    /// Time in seconds.
    pub time_s: f64,
    /// Joint position in radians or meters.
    pub position: f64,
}

impl JointKeyframe {
    /// Creates a keyframe.
    pub fn new(time_s: f64, position: f64) -> Self {
        Self { time_s, position }
    }
}

/// A keyframed trajectory for one joint.
#[derive(Clone, Debug, PartialEq)]
pub struct JointTrack {
    /// Joint entity.
    pub joint: Entity,
    /// Joint name captured at authoring time.
    pub joint_name: String,
    /// Interpolation between keyframes.
    pub interpolation: JointInterpolation,
    /// Keyframes sorted by strictly increasing time.
    pub keyframes: Vec<JointKeyframe>,
}

impl JointTrack {
    /// Creates a track with the given keyframes and default interpolation.
    pub fn new(
        joint: Entity,
        joint_name: impl Into<String>,
        keyframes: Vec<JointKeyframe>,
    ) -> Self {
        Self {
            joint,
            joint_name: joint_name.into(),
            interpolation: JointInterpolation::default(),
            keyframes,
        }
    }

    /// Samples the track position at `time_s`, clamped to the keyframe range.
    pub fn sample(&self, time_s: f64) -> f64 {
        let keys = &self.keyframes;
        if keys.is_empty() {
            return 0.0;
        }
        let first = keys[0];
        let last = keys[keys.len() - 1];
        if time_s <= first.time_s {
            return first.position;
        }
        if time_s >= last.time_s {
            return last.position;
        }

        let segment = keys
            .windows(2)
            .position(|window| time_s >= window[0].time_s && time_s <= window[1].time_s)
            .unwrap_or(0);
        let start = keys[segment];
        let end = keys[segment + 1];
        let span = end.time_s - start.time_s;
        let u = if span > 0.0 {
            ((time_s - start.time_s) / span).clamp(0.0, 1.0)
        } else {
            0.0
        };
        match self.interpolation {
            JointInterpolation::Linear => start.position + (end.position - start.position) * u,
            JointInterpolation::SmoothStep => {
                let s = u * u * (3.0 - 2.0 * u);
                start.position + (end.position - start.position) * s
            }
            JointInterpolation::Cubic => {
                let previous = if segment > 0 {
                    keys[segment - 1]
                } else {
                    start
                };
                let next = if segment + 2 < keys.len() {
                    keys[segment + 2]
                } else {
                    end
                };
                let incoming = safe_slope(previous, end);
                let outgoing = safe_slope(start, next);
                let m0 = if segment > 0 { incoming } else { outgoing };
                let m1 = if segment + 2 < keys.len() {
                    outgoing
                } else {
                    incoming
                };
                hermite(start.position, m0, end.position, m1, span, u)
            }
        }
    }
}

/// A set of joint tracks evaluated together.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct BodyMotion {
    /// Tracks in authoring order.
    pub tracks: Vec<JointTrack>,
    /// When set, sampling wraps time into `[0, loop_duration_s)`.
    pub loop_duration_s: Option<f64>,
}

/// Result of sampling a [`BodyMotion`].
#[derive(Clone, Debug, PartialEq)]
pub struct BodyMotionSample {
    /// Resolved (possibly wrapped) time in seconds.
    pub time_s: f64,
    /// Joint positions in track order.
    pub positions: Vec<(Entity, f64)>,
}

impl BodyMotionSample {
    /// Position of a joint in the sample.
    pub fn position(&self, joint: Entity) -> Option<f64> {
        self.positions
            .iter()
            .find(|(candidate, _)| *candidate == joint)
            .map(|(_, position)| *position)
    }
}

/// Error returned when authoring or sampling a motion.
#[derive(Clone, Debug, PartialEq, Eq, Error)]
pub enum MotionError {
    /// The motion has no tracks.
    #[error("body motion has no tracks")]
    EmptyMotion,
    /// A track has no keyframes.
    #[error("joint track '{0}' has no keyframes")]
    EmptyTrack(String),
    /// A keyframe time is not finite.
    #[error("joint track '{0}' has a non-finite keyframe time")]
    NonFiniteTime(String),
    /// Keyframe times are not strictly increasing.
    #[error("joint track '{0}' keyframes are not strictly increasing at index {1}")]
    KeyframesNotSorted(String, usize),
    /// A joint appears in more than one track.
    #[error("joint {0:?} appears in more than one track")]
    DuplicateJoint(Entity),
    /// The loop duration is not finite and positive.
    #[error("loop duration must be finite and positive")]
    InvalidLoopDuration,
    /// The requested sample time is not finite.
    #[error("sample time must be finite")]
    NonFiniteSample,
}

impl BodyMotion {
    /// Creates an empty motion.
    pub fn new() -> Self {
        Self::default()
    }

    /// Adds a track after validating it, returning the track index.
    pub fn add_track(&mut self, track: JointTrack) -> Result<usize, MotionError> {
        validate_track(&track)?;
        if self
            .tracks
            .iter()
            .any(|existing| existing.joint == track.joint)
        {
            return Err(MotionError::DuplicateJoint(track.joint));
        }
        self.tracks.push(track);
        Ok(self.tracks.len() - 1)
    }

    /// Duration in seconds, or zero for an empty motion.
    pub fn duration_s(&self) -> f64 {
        self.tracks
            .iter()
            .filter_map(|track| track.keyframes.last().map(|key| key.time_s))
            .fold(0.0_f64, f64::max)
    }

    /// Validates the whole motion.
    pub fn validate(&self) -> Result<(), MotionError> {
        if self.tracks.is_empty() {
            return Err(MotionError::EmptyMotion);
        }
        if let Some(duration) = self.loop_duration_s {
            if !duration.is_finite() || duration <= 0.0 {
                return Err(MotionError::InvalidLoopDuration);
            }
        }
        let mut seen: Vec<Entity> = Vec::new();
        for track in &self.tracks {
            validate_track(track)?;
            if seen.contains(&track.joint) {
                return Err(MotionError::DuplicateJoint(track.joint));
            }
            seen.push(track.joint);
        }
        Ok(())
    }

    /// Samples every track at `time_s`, wrapping if a loop duration is set.
    pub fn sample(&self, time_s: f64) -> Result<BodyMotionSample, MotionError> {
        if !time_s.is_finite() {
            return Err(MotionError::NonFiniteSample);
        }
        self.validate()?;
        let resolved = match self.loop_duration_s {
            Some(duration) => time_s.rem_euclid(duration),
            None => time_s,
        };
        let positions = self
            .tracks
            .iter()
            .map(|track| (track.joint, track.sample(resolved)))
            .collect();
        Ok(BodyMotionSample {
            time_s: resolved,
            positions,
        })
    }
}

/// Builds a motion by reading the current joint positions of a robot once.
///
/// Every movable joint known to the world for `robot` becomes a single-keyframe
/// track at `time_s`. This is a convenient seed for authoring; callers can push
/// further keyframes onto the returned tracks.
pub fn body_motion_from_world(world: &World, robot: Entity, time_s: f64) -> BodyMotion {
    let mut motion = BodyMotion::new();
    for entity_ref in world.iter_entities() {
        let Some(joint) = entity_ref.get::<crate::components::Joint>() else {
            continue;
        };
        if joint.robot != robot || joint.kind == crate::components::JointKind::Fixed {
            continue;
        }
        let name = entity_ref
            .get::<rne_ecs::Name>()
            .map(|name| name.0.clone())
            .unwrap_or_else(|| format!("joint_{}", entity_ref.id().index()));
        motion.tracks.push(JointTrack::new(
            entity_ref.id(),
            name,
            vec![JointKeyframe::new(time_s, joint.position)],
        ));
    }
    motion
}

fn validate_track(track: &JointTrack) -> Result<(), MotionError> {
    if track.keyframes.is_empty() {
        return Err(MotionError::EmptyTrack(track.joint_name.clone()));
    }
    let mut previous = f64::NEG_INFINITY;
    for (index, key) in track.keyframes.iter().enumerate() {
        if !key.time_s.is_finite() || !key.position.is_finite() {
            return Err(MotionError::NonFiniteTime(track.joint_name.clone()));
        }
        if key.time_s <= previous {
            return Err(MotionError::KeyframesNotSorted(
                track.joint_name.clone(),
                index,
            ));
        }
        previous = key.time_s;
    }
    Ok(())
}

fn safe_slope(a: JointKeyframe, b: JointKeyframe) -> f64 {
    let span = b.time_s - a.time_s;
    if span > 0.0 {
        (b.position - a.position) / span
    } else {
        0.0
    }
}

fn hermite(p0: f64, m0: f64, p1: f64, m1: f64, span: f64, u: f64) -> f64 {
    let u2 = u * u;
    let u3 = u2 * u;
    let h00 = 2.0 * u3 - 3.0 * u2 + 1.0;
    let h10 = u3 - 2.0 * u2 + u;
    let h01 = -2.0 * u3 + 3.0 * u2;
    let h11 = u3 - u2;
    h00 * p0 + h10 * span * m0 + h01 * p1 + h11 * span * m1
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use rne_ecs::{spawn_named, World};

    fn joint_in_world() -> (World, Entity) {
        let mut world = World::new();
        let joint = spawn_named(&mut world, "joint");
        (world, joint)
    }

    #[test]
    fn linear_sampling_interpolates() {
        let (_world, joint) = joint_in_world();
        let track = JointTrack::new(
            joint,
            "joint",
            vec![JointKeyframe::new(0.0, 0.0), JointKeyframe::new(1.0, 2.0)],
        );
        assert_relative_eq!(track.sample(0.5), 1.0, epsilon = 1e-12);
        assert_relative_eq!(track.sample(-1.0), 0.0, epsilon = 1e-12);
        assert_relative_eq!(track.sample(2.0), 2.0, epsilon = 1e-12);
    }

    #[test]
    fn smooth_step_has_zero_end_slopes() {
        let (_world, joint) = joint_in_world();
        let mut track = JointTrack::new(
            joint,
            "joint",
            vec![JointKeyframe::new(0.0, 0.0), JointKeyframe::new(1.0, 1.0)],
        );
        track.interpolation = JointInterpolation::SmoothStep;
        assert_relative_eq!(track.sample(0.5), 0.5, epsilon = 1e-12);
        let near_start = track.sample(1.0e-3);
        assert!(near_start < 1.0e-5, "near_start={near_start}");
    }

    #[test]
    fn body_motion_wraps_when_looping() {
        let (_world, joint) = joint_in_world();
        let mut motion = BodyMotion::new();
        motion.loop_duration_s = Some(1.0);
        motion
            .add_track(JointTrack::new(
                joint,
                "joint",
                vec![JointKeyframe::new(0.0, 0.0), JointKeyframe::new(1.0, 4.0)],
            ))
            .unwrap();
        let sample = motion.sample(1.25).unwrap();
        assert_relative_eq!(sample.time_s, 0.25, epsilon = 1e-12);
        assert_relative_eq!(sample.position(joint).unwrap(), 1.0, epsilon = 1e-12);
    }

    #[test]
    fn rejects_unsorted_keyframes() {
        let (_world, joint) = joint_in_world();
        let track = JointTrack::new(
            joint,
            "joint",
            vec![JointKeyframe::new(1.0, 0.0), JointKeyframe::new(0.5, 1.0)],
        );
        let mut motion = BodyMotion::new();
        assert!(matches!(
            motion.add_track(track),
            Err(MotionError::KeyframesNotSorted(_, 1))
        ));
    }
}
