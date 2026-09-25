//! Press-sensitive call button for elevator boarding.
//!
//! A robot that shares a building with people has to operate the building's own
//! controls, and the elevator call button is the first of them. Simulating the
//! press honestly matters: a robot that summons the elevator by calling an API
//! has not demonstrated anything, whereas a robot that has to reach a 2 cm
//! target with enough force, and no more, has.
//!
//! This module owns the button as a device. It holds no physics handles: the
//! caller passes the contacts its backend reported as plain values, exactly as
//! [`crate::elevator::Elevator`] takes plain time steps. A press is detected
//! from force with hysteresis, so a fingertip resting on the button does not
//! chatter between states, and [`CallButton::just_pressed`] is an edge, so one
//! physical press produces exactly one elevator call.

use rne_math::Vec3;
use serde::{Deserialize, Serialize};

/// One contact reported against the button, in world coordinates.
///
/// This is the backend-neutral projection of a solved contact point: where it
/// touched and how hard. Callers building it from a physics backend supply the
/// contact position and its normal force.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct ButtonContact {
    /// Contact position in world coordinates, in meters.
    pub point_world_m: Vec3,
    /// Normal force magnitude at the contact, in newtons.
    pub normal_force_n: f64,
}

/// Error raised when configuring a call button.
#[derive(Clone, Copy, Debug, PartialEq, Eq, thiserror::Error)]
pub enum CallButtonError {
    /// The face geometry was not finite, or the radius was not positive.
    #[error("button face must be finite with a positive radius")]
    InvalidFace,
    /// The face normal had no direction.
    #[error("button face normal must have a non-zero length")]
    DegenerateNormal,
    /// The force thresholds were not finite with press above release.
    #[error("button press force must be finite and above the release force")]
    InvalidThresholds,
}

/// Geometry and actuation thresholds of one call button.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct CallButtonSpec {
    /// Centre of the button face in world coordinates, in meters.
    pub center_world_m: Vec3,
    /// Outward face normal; normalized internally.
    pub normal_world: Vec3,
    /// Face radius in meters. A contact outside it is a touch on the wall.
    pub radius_m: f64,
    /// Depth behind the face within which a contact still counts, in meters.
    ///
    /// A plunger travels as it is pressed, so a contact slightly behind the
    /// nominal face is still on the button rather than through it.
    pub travel_m: f64,
    /// Force at which the button actuates, in newtons.
    pub press_force_n: f64,
    /// Force below which an actuated button releases, in newtons.
    ///
    /// Strictly below [`Self::press_force_n`], so a fingertip holding near the
    /// threshold does not oscillate between pressed and released.
    pub release_force_n: f64,
    /// Floor this button calls.
    pub floor: usize,
}

impl CallButtonSpec {
    /// Validates the specification.
    pub fn validate(&self) -> Result<(), CallButtonError> {
        if !self.center_world_m.is_finite()
            || !self.radius_m.is_finite()
            || self.radius_m <= 0.0
            || !self.travel_m.is_finite()
            || self.travel_m < 0.0
        {
            return Err(CallButtonError::InvalidFace);
        }
        if !self.normal_world.is_finite() || self.normal_world.length() <= f64::EPSILON {
            return Err(CallButtonError::DegenerateNormal);
        }
        if !self.press_force_n.is_finite()
            || !self.release_force_n.is_finite()
            || self.release_force_n < 0.0
            || self.press_force_n <= self.release_force_n
        {
            return Err(CallButtonError::InvalidThresholds);
        }
        Ok(())
    }
}

/// Whether the button is currently actuated.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum CallButtonState {
    /// Not actuated.
    Released,
    /// Actuated.
    Pressed,
}

/// A call button that actuates on contact force.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct CallButton {
    spec: CallButtonSpec,
    state: CallButtonState,
    just_pressed: bool,
    press_count: usize,
    applied_force_n: f64,
}

impl CallButton {
    /// Creates a released button.
    pub fn new(spec: CallButtonSpec) -> Result<Self, CallButtonError> {
        spec.validate()?;
        Ok(Self {
            spec,
            state: CallButtonState::Released,
            just_pressed: false,
            press_count: 0,
            applied_force_n: 0.0,
        })
    }

    /// Returns the specification.
    pub fn spec(&self) -> &CallButtonSpec {
        &self.spec
    }

    /// Returns the current state.
    pub fn state(&self) -> CallButtonState {
        self.state
    }

    /// Returns whether the most recent update actuated the button.
    ///
    /// This is an edge, not a level: holding the button down leaves it false on
    /// every update after the first, so a caller can wire it straight to
    /// [`crate::elevator::Elevator::call`] without summoning the car repeatedly.
    pub fn just_pressed(&self) -> bool {
        self.just_pressed
    }

    /// Returns how many times the button has been actuated.
    pub fn press_count(&self) -> usize {
        self.press_count
    }

    /// Returns the force currently applied to the button face, in newtons.
    pub fn applied_force_n(&self) -> f64 {
        self.applied_force_n
    }

    /// Returns whether a contact lies on the button face.
    ///
    /// The contact must be inside the face radius measured in the face plane,
    /// and between the face and [`CallButtonSpec::travel_m`] behind it. A touch
    /// on the surrounding wall, or a contact reported from behind the panel, is
    /// not a press.
    pub fn accepts(&self, contact: ButtonContact) -> bool {
        if !contact.point_world_m.is_finite() || !contact.normal_force_n.is_finite() {
            return false;
        }
        let normal = self.spec.normal_world.normalize();
        let offset = contact.point_world_m - self.spec.center_world_m;
        let depth_m = offset.dot(normal);
        if depth_m > 0.0 || depth_m < -self.spec.travel_m {
            return false;
        }
        let in_plane = offset - normal * depth_m;
        in_plane.length() <= self.spec.radius_m
    }

    /// Updates the button from the contacts reported this step.
    ///
    /// Forces from every accepted contact are summed, so a broad fingertip
    /// pressing across the face actuates as one press rather than several.
    pub fn update(&mut self, contacts: &[ButtonContact]) -> CallButtonState {
        let applied_force_n: f64 = contacts
            .iter()
            .filter(|contact| self.accepts(**contact))
            .map(|contact| contact.normal_force_n.max(0.0))
            .sum();
        self.applied_force_n = applied_force_n;

        self.just_pressed = false;
        match self.state {
            CallButtonState::Released => {
                if applied_force_n >= self.spec.press_force_n {
                    self.state = CallButtonState::Pressed;
                    self.just_pressed = true;
                    self.press_count += 1;
                }
            }
            CallButtonState::Pressed => {
                if applied_force_n <= self.spec.release_force_n {
                    self.state = CallButtonState::Released;
                }
            }
        }
        self.state
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn spec() -> CallButtonSpec {
        CallButtonSpec {
            center_world_m: Vec3::new(1.0, 1.1, 0.0),
            normal_world: Vec3::new(0.0, 0.0, 1.0),
            radius_m: 0.02,
            travel_m: 0.004,
            press_force_n: 3.0,
            release_force_n: 1.0,
            floor: 2,
        }
    }

    /// A contact on the face centre, just in front of the plunger.
    fn contact_on_face(force_n: f64) -> ButtonContact {
        ButtonContact {
            point_world_m: Vec3::new(1.0, 1.1, -0.001),
            normal_force_n: force_n,
        }
    }

    #[test]
    fn specification_rejects_degenerate_faces_normals_and_thresholds() {
        assert_eq!(
            CallButtonSpec {
                radius_m: 0.0,
                ..spec()
            }
            .validate(),
            Err(CallButtonError::InvalidFace)
        );
        assert_eq!(
            CallButtonSpec {
                travel_m: -0.001,
                ..spec()
            }
            .validate(),
            Err(CallButtonError::InvalidFace)
        );
        assert_eq!(
            CallButtonSpec {
                center_world_m: Vec3::new(f64::NAN, 0.0, 0.0),
                ..spec()
            }
            .validate(),
            Err(CallButtonError::InvalidFace)
        );
        assert_eq!(
            CallButtonSpec {
                normal_world: Vec3::ZERO,
                ..spec()
            }
            .validate(),
            Err(CallButtonError::DegenerateNormal)
        );
        // Press must be strictly above release, or the hysteresis is not one.
        assert_eq!(
            CallButtonSpec {
                press_force_n: 1.0,
                release_force_n: 1.0,
                ..spec()
            }
            .validate(),
            Err(CallButtonError::InvalidThresholds)
        );
        assert_eq!(
            CallButtonSpec {
                release_force_n: -0.1,
                ..spec()
            }
            .validate(),
            Err(CallButtonError::InvalidThresholds)
        );
    }

    #[test]
    fn only_contacts_on_the_face_count_as_a_press() {
        let button = CallButton::new(spec()).expect("button");

        assert!(button.accepts(contact_on_face(5.0)));
        // Flush with the face.
        assert!(button.accepts(ButtonContact {
            point_world_m: Vec3::new(1.0, 1.1, 0.0),
            normal_force_n: 5.0,
        }));
        // Fully depressed, still on the plunger.
        assert!(button.accepts(ButtonContact {
            point_world_m: Vec3::new(1.0, 1.1, -0.004),
            normal_force_n: 5.0,
        }));
        // Beyond the plunger travel: that is through the panel, not a press.
        assert!(!button.accepts(ButtonContact {
            point_world_m: Vec3::new(1.0, 1.1, -0.01),
            normal_force_n: 5.0,
        }));
        // In front of the face: not touching yet.
        assert!(!button.accepts(ButtonContact {
            point_world_m: Vec3::new(1.0, 1.1, 0.002),
            normal_force_n: 5.0,
        }));
        // On the wall beside the button.
        assert!(!button.accepts(ButtonContact {
            point_world_m: Vec3::new(1.05, 1.1, -0.001),
            normal_force_n: 50.0,
        }));
        // Non-finite input is rejected rather than propagated.
        assert!(!button.accepts(ButtonContact {
            point_world_m: Vec3::new(f64::NAN, 1.1, 0.0),
            normal_force_n: 5.0,
        }));
        assert!(!button.accepts(ButtonContact {
            point_world_m: Vec3::new(1.0, 1.1, -0.001),
            normal_force_n: f64::NAN,
        }));
    }

    #[test]
    fn a_held_press_actuates_once_and_re_arms_only_after_release() {
        let mut button = CallButton::new(spec()).expect("button");
        assert_eq!(button.state(), CallButtonState::Released);

        // Approaching without enough force does not actuate.
        button.update(&[contact_on_face(2.0)]);
        assert_eq!(button.state(), CallButtonState::Released);
        assert!(!button.just_pressed());
        assert_eq!(button.applied_force_n(), 2.0);

        // Crossing the threshold actuates exactly once.
        button.update(&[contact_on_face(4.0)]);
        assert_eq!(button.state(), CallButtonState::Pressed);
        assert!(button.just_pressed());
        assert_eq!(button.press_count(), 1);

        // Holding it down must not summon the car again and again.
        for _ in 0..10 {
            button.update(&[contact_on_face(6.0)]);
            assert_eq!(button.state(), CallButtonState::Pressed);
            assert!(!button.just_pressed());
        }
        assert_eq!(button.press_count(), 1);

        // Easing off within the hysteresis band keeps it pressed.
        button.update(&[contact_on_face(2.0)]);
        assert_eq!(button.state(), CallButtonState::Pressed);
        assert_eq!(button.press_count(), 1);

        // Releasing re-arms it.
        button.update(&[]);
        assert_eq!(button.state(), CallButtonState::Released);
        assert!(!button.just_pressed());
        assert_eq!(button.applied_force_n(), 0.0);

        button.update(&[contact_on_face(4.0)]);
        assert_eq!(button.press_count(), 2);
        assert!(button.just_pressed());
    }

    #[test]
    fn force_from_several_contacts_on_the_face_sums_into_one_press() {
        let mut button = CallButton::new(spec()).expect("button");
        // A fingertip reports several contact points; each alone is below the
        // threshold, and together they are one press, not three.
        let spread = [
            ButtonContact {
                point_world_m: Vec3::new(0.995, 1.1, -0.001),
                normal_force_n: 1.2,
            },
            ButtonContact {
                point_world_m: Vec3::new(1.0, 1.105, -0.002),
                normal_force_n: 1.2,
            },
            ButtonContact {
                point_world_m: Vec3::new(1.004, 1.098, -0.001),
                normal_force_n: 1.2,
            },
            // A simultaneous shove on the wall must not help.
            ButtonContact {
                point_world_m: Vec3::new(1.2, 1.1, -0.001),
                normal_force_n: 100.0,
            },
        ];
        button.update(&spread);
        assert!((button.applied_force_n() - 3.6).abs() < 1.0e-9);
        assert_eq!(button.state(), CallButtonState::Pressed);
        assert_eq!(button.press_count(), 1);
    }
}
