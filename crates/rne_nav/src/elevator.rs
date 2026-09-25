//! Deterministic elevator device for multi-floor navigation.
//!
//! An elevator is the thing that stops an indoor service robot from being a
//! single-floor robot, and simulating it is mostly a timing problem rather than
//! a physics one: the car takes a known time to travel, the doors take a known
//! time to open, and they stay open for a bounded dwell. What the robot has to
//! get right is *when* it is allowed to drive in and out.
//!
//! This module owns that timing as a pure state machine. It holds no physics
//! handles and reads no wall-clock time: given the same calls and the same
//! integration steps it always produces the same car height and door opening,
//! so a boarding sequence replays exactly.
//!
//! A caller drives it with [`Elevator::update`] and applies
//! [`Elevator::car_height_m`] and [`Elevator::door_opening_m`] to whatever
//! bodies represent the car and doors. Riding itself needs no special support:
//! a body standing on the car is carried by ordinary normal contact.

use serde::{Deserialize, Serialize};

/// Error raised when configuring or commanding an elevator.
#[derive(Clone, Copy, Debug, PartialEq, Eq, thiserror::Error)]
pub enum ElevatorError {
    /// Fewer than two floors were declared.
    #[error("an elevator needs at least two floors")]
    TooFewFloors,
    /// Floor heights were not finite and strictly ascending.
    #[error("floor heights must be finite and strictly ascending")]
    UnorderedFloors,
    /// A speed or duration was not finite and positive.
    #[error("elevator speeds and durations must be finite and positive")]
    InvalidTiming,
    /// A call named a floor the shaft does not serve.
    #[error("floor {0} is not served by this elevator")]
    UnknownFloor(usize),
    /// A non-finite integration step was supplied.
    #[error("elevator update requires a finite non-negative step")]
    InvalidStep,
}

/// Physical and timing description of one elevator shaft.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ElevatorSpec {
    /// Car floor heights in meters, strictly ascending.
    pub floor_heights_m: Vec<f64>,
    /// Car cruise speed in meters per second.
    pub car_speed_m_s: f64,
    /// Car acceleration and braking magnitude in meters per second squared.
    ///
    /// A car whose speed changes instantly throws whatever is standing on it:
    /// stopping from `v` launches an unattached rider `v^2 / 2g` into the air,
    /// which is 5 cm at 1 m/s. Real elevators ramp, and so does this one.
    pub car_acceleration_m_s2: f64,
    /// Door leaf travel in meters from closed to fully open.
    pub door_travel_m: f64,
    /// Door leaf speed in meters per second.
    pub door_speed_m_s: f64,
    /// Time in seconds the doors stay fully open before closing.
    pub door_hold_s: f64,
}

impl ElevatorSpec {
    /// Validates the specification.
    pub fn validate(&self) -> Result<(), ElevatorError> {
        if self.floor_heights_m.len() < 2 {
            return Err(ElevatorError::TooFewFloors);
        }
        if self
            .floor_heights_m
            .windows(2)
            .any(|pair| !pair[0].is_finite() || !pair[1].is_finite() || pair[1] <= pair[0])
        {
            return Err(ElevatorError::UnorderedFloors);
        }
        let positive = |value: f64| value.is_finite() && value > 0.0;
        if !positive(self.car_speed_m_s)
            || !positive(self.car_acceleration_m_s2)
            || !positive(self.door_travel_m)
            || !positive(self.door_speed_m_s)
            || !positive(self.door_hold_s)
        {
            return Err(ElevatorError::InvalidTiming);
        }
        Ok(())
    }

    /// Returns the number of floors served.
    pub fn floor_count(&self) -> usize {
        self.floor_heights_m.len()
    }
}

/// What the elevator is currently doing.
///
/// The car only moves with the doors fully closed, and a robot may only cross
/// the threshold in [`Self::DoorsOpen`].
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum ElevatorState {
    /// Stopped at a floor with the doors closed.
    Idle {
        /// Floor the car is parked at.
        floor: usize,
    },
    /// Doors travelling open at a floor.
    Opening {
        /// Floor the car is parked at.
        floor: usize,
    },
    /// Doors fully open and dwelling.
    DoorsOpen {
        /// Floor the car is parked at.
        floor: usize,
    },
    /// Doors travelling closed at a floor.
    Closing {
        /// Floor the car is parked at.
        floor: usize,
    },
    /// Car travelling between floors with the doors closed.
    Moving {
        /// Floor the car left.
        from: usize,
        /// Floor the car is travelling to.
        to: usize,
    },
}

impl ElevatorState {
    /// Returns the floor the car is parked at, or `None` while travelling.
    pub fn parked_floor(self) -> Option<usize> {
        match self {
            Self::Idle { floor }
            | Self::Opening { floor }
            | Self::DoorsOpen { floor }
            | Self::Closing { floor } => Some(floor),
            Self::Moving { .. } => None,
        }
    }
}

/// A deterministic elevator serving a fixed set of floors.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Elevator {
    spec: ElevatorSpec,
    state: ElevatorState,
    car_height_m: f64,
    car_velocity_m_s: f64,
    door_opening_m: f64,
    dwell_remaining_s: f64,
    /// Outstanding calls in ascending floor order, so service is deterministic.
    pending_calls: Vec<usize>,
}

impl Elevator {
    /// Creates an elevator parked at `start_floor` with its doors closed.
    pub fn new(spec: ElevatorSpec, start_floor: usize) -> Result<Self, ElevatorError> {
        spec.validate()?;
        if start_floor >= spec.floor_count() {
            return Err(ElevatorError::UnknownFloor(start_floor));
        }
        let car_height_m = spec.floor_heights_m[start_floor];
        Ok(Self {
            spec,
            state: ElevatorState::Idle { floor: start_floor },
            car_height_m,
            car_velocity_m_s: 0.0,
            door_opening_m: 0.0,
            dwell_remaining_s: 0.0,
            pending_calls: Vec::new(),
        })
    }

    /// Returns the specification.
    pub fn spec(&self) -> &ElevatorSpec {
        &self.spec
    }

    /// Returns the current state.
    pub fn state(&self) -> ElevatorState {
        self.state
    }

    /// Returns the car's current height in meters.
    pub fn car_height_m(&self) -> f64 {
        self.car_height_m
    }

    /// Returns the car's signed vertical velocity in meters per second.
    pub fn car_velocity_m_s(&self) -> f64 {
        self.car_velocity_m_s
    }

    /// Returns how far each door leaf is open, in meters.
    pub fn door_opening_m(&self) -> f64 {
        self.door_opening_m
    }

    /// Returns the outstanding calls in service order.
    pub fn pending_calls(&self) -> &[usize] {
        &self.pending_calls
    }

    /// Returns whether a robot may cross the threshold at `floor` right now.
    ///
    /// This is the question a boarding behaviour actually asks: the car must be
    /// at that floor **and** the doors fully open. Being parked with the doors
    /// still moving is not boardable, which is what keeps a robot from driving
    /// into a closing door.
    pub fn is_boardable(&self, floor: usize) -> bool {
        matches!(self.state, ElevatorState::DoorsOpen { floor: at } if at == floor)
    }

    /// Requests service at `floor`.
    ///
    /// Duplicate calls are idempotent. Calling the floor the car is already
    /// parked at opens the doors.
    pub fn call(&mut self, floor: usize) -> Result<(), ElevatorError> {
        if floor >= self.spec.floor_count() {
            return Err(ElevatorError::UnknownFloor(floor));
        }
        if !self.pending_calls.contains(&floor) {
            self.pending_calls.push(floor);
            self.pending_calls.sort_unstable();
        }
        Ok(())
    }

    /// Advances the elevator by one integration step.
    ///
    /// The step is in seconds and must be finite and non-negative. Progress is
    /// exact for the supplied step: no wall-clock time is read, so the same
    /// sequence of steps always produces the same trajectory.
    pub fn update(&mut self, dt_s: f64) -> Result<ElevatorState, ElevatorError> {
        if !dt_s.is_finite() || dt_s < 0.0 {
            return Err(ElevatorError::InvalidStep);
        }
        match self.state {
            ElevatorState::Idle { floor } => {
                if self.pending_calls.first() == Some(&floor) {
                    self.pending_calls.remove(0);
                    self.state = ElevatorState::Opening { floor };
                } else if let Some(&target) = self.pending_calls.first() {
                    self.state = ElevatorState::Moving {
                        from: floor,
                        to: target,
                    };
                }
            }
            ElevatorState::Opening { floor } => {
                self.door_opening_m = (self.door_opening_m + self.spec.door_speed_m_s * dt_s)
                    .min(self.spec.door_travel_m);
                if self.door_opening_m >= self.spec.door_travel_m {
                    self.dwell_remaining_s = self.spec.door_hold_s;
                    self.state = ElevatorState::DoorsOpen { floor };
                }
            }
            ElevatorState::DoorsOpen { floor } => {
                self.dwell_remaining_s -= dt_s;
                if self.dwell_remaining_s <= 0.0 {
                    self.dwell_remaining_s = 0.0;
                    self.state = ElevatorState::Closing { floor };
                }
            }
            ElevatorState::Closing { floor } => {
                self.door_opening_m =
                    (self.door_opening_m - self.spec.door_speed_m_s * dt_s).max(0.0);
                if self.door_opening_m <= 0.0 {
                    self.state = ElevatorState::Idle { floor };
                }
            }
            ElevatorState::Moving { from, to } => {
                let target_m = self.spec.floor_heights_m[to];
                let remaining_m = target_m - self.car_height_m;
                let direction = if remaining_m >= 0.0 { 1.0 } else { -1.0 };
                let acceleration = self.spec.car_acceleration_m_s2;
                let speed = self.car_velocity_m_s.abs();

                // Trapezoidal profile: brake once the remaining distance is no
                // more than what stopping from the current speed needs.
                let braking_distance_m = speed * speed / (2.0 * acceleration);
                let target_speed = if remaining_m.abs() <= braking_distance_m {
                    (speed - acceleration * dt_s).max(0.0)
                } else {
                    (speed + acceleration * dt_s).min(self.spec.car_speed_m_s)
                };
                self.car_velocity_m_s = target_speed * direction;

                let step_m = self.car_velocity_m_s * dt_s;
                if remaining_m.abs() <= step_m.abs() || remaining_m == 0.0 {
                    self.car_height_m = target_m;
                    self.car_velocity_m_s = 0.0;
                    // Arriving consumes the call and opens up for boarding.
                    self.pending_calls.retain(|floor| *floor != to);
                    self.state = ElevatorState::Opening { floor: to };
                } else {
                    self.car_height_m += step_m;
                    self.state = ElevatorState::Moving { from, to };
                }
            }
        }
        Ok(self.state)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn spec() -> ElevatorSpec {
        ElevatorSpec {
            floor_heights_m: vec![0.0, 3.5, 7.0],
            car_speed_m_s: 1.0,
            car_acceleration_m_s2: 0.8,
            door_travel_m: 0.6,
            door_speed_m_s: 0.6,
            door_hold_s: 2.0,
        }
    }

    /// Runs the elevator until `predicate` holds, returning the elapsed seconds.
    fn run_until(
        elevator: &mut Elevator,
        dt_s: f64,
        max_s: f64,
        mut predicate: impl FnMut(&Elevator) -> bool,
    ) -> Option<f64> {
        let mut elapsed_s = 0.0;
        while elapsed_s <= max_s {
            if predicate(elevator) {
                return Some(elapsed_s);
            }
            elevator.update(dt_s).expect("update");
            elapsed_s += dt_s;
        }
        None
    }

    #[test]
    fn specification_rejects_degenerate_shafts_and_timings() {
        assert_eq!(
            ElevatorSpec {
                floor_heights_m: vec![0.0],
                ..spec()
            }
            .validate(),
            Err(ElevatorError::TooFewFloors)
        );
        assert_eq!(
            ElevatorSpec {
                floor_heights_m: vec![0.0, 0.0],
                ..spec()
            }
            .validate(),
            Err(ElevatorError::UnorderedFloors)
        );
        assert_eq!(
            ElevatorSpec {
                floor_heights_m: vec![3.0, 0.0],
                ..spec()
            }
            .validate(),
            Err(ElevatorError::UnorderedFloors)
        );
        assert_eq!(
            ElevatorSpec {
                floor_heights_m: vec![0.0, f64::NAN],
                ..spec()
            }
            .validate(),
            Err(ElevatorError::UnorderedFloors)
        );
        for broken in [
            ElevatorSpec {
                car_speed_m_s: 0.0,
                ..spec()
            },
            ElevatorSpec {
                car_acceleration_m_s2: 0.0,
                ..spec()
            },
            ElevatorSpec {
                door_travel_m: -0.1,
                ..spec()
            },
            ElevatorSpec {
                door_speed_m_s: f64::INFINITY,
                ..spec()
            },
            ElevatorSpec {
                door_hold_s: 0.0,
                ..spec()
            },
        ] {
            assert_eq!(broken.validate(), Err(ElevatorError::InvalidTiming));
        }

        assert_eq!(
            Elevator::new(spec(), 3).unwrap_err(),
            ElevatorError::UnknownFloor(3)
        );
        let mut elevator = Elevator::new(spec(), 0).expect("elevator");
        assert_eq!(elevator.call(9), Err(ElevatorError::UnknownFloor(9)));
        assert_eq!(elevator.update(-1.0), Err(ElevatorError::InvalidStep));
        assert_eq!(elevator.update(f64::NAN), Err(ElevatorError::InvalidStep));
    }

    #[test]
    fn calling_the_parked_floor_opens_the_doors_and_they_close_after_the_dwell() {
        let dt_s = 1.0 / 60.0;
        let mut elevator = Elevator::new(spec(), 0).expect("elevator");
        assert_eq!(elevator.state(), ElevatorState::Idle { floor: 0 });
        assert!(!elevator.is_boardable(0));

        elevator.call(0).expect("call floor 0");
        let open_s =
            run_until(&mut elevator, dt_s, 10.0, |e| e.is_boardable(0)).expect("doors open");
        // 0.6 m of travel at 0.6 m/s.
        assert!(
            (open_s - 1.0).abs() < 2.0 * dt_s,
            "doors should open in about 1 s, took {open_s}"
        );
        assert!((elevator.door_opening_m() - 0.6).abs() < 1.0e-9);
        assert_eq!(elevator.car_height_m(), 0.0);

        // The dwell is bounded, then the doors close and boarding ends.
        let closed_s = run_until(&mut elevator, dt_s, 20.0, |e| {
            e.state() == ElevatorState::Idle { floor: 0 }
        })
        .expect("doors close");
        assert!(
            (closed_s - 3.0).abs() < 3.0 * dt_s,
            "dwell plus close should take about 3 s, took {closed_s}"
        );
        assert!(!elevator.is_boardable(0));
        assert_eq!(elevator.door_opening_m(), 0.0);
        assert!(elevator.pending_calls().is_empty());
    }

    #[test]
    fn the_car_travels_with_the_doors_shut_and_opens_on_arrival() {
        let dt_s = 1.0 / 60.0;
        let mut elevator = Elevator::new(spec(), 0).expect("elevator");
        elevator.call(2).expect("call floor 2");

        let mut saw_moving = false;
        let arrived_s = run_until(&mut elevator, dt_s, 30.0, |e| {
            if matches!(e.state(), ElevatorState::Moving { .. }) {
                saw_moving = true;
                // Travelling with an open door would be the classic simulation
                // cheat; assert it never happens instead of assuming it.
                assert_eq!(e.door_opening_m(), 0.0);
            }
            e.is_boardable(2)
        })
        .expect("arrives at floor 2");
        assert!(saw_moving, "the car must actually travel between floors");
        // The car comes to rest rather than stopping instantly, which is what
        // keeps it from throwing whatever is standing on it.
        assert_eq!(elevator.car_velocity_m_s(), 0.0);

        // Trapezoidal profile over 7 m: 1.25 s to reach 1 m/s covering 0.625 m,
        // the same again to brake, and 5.75 m of cruise at 1 m/s, so 8.25 s of
        // travel; then 0.6 m of door travel at 0.6 m/s.
        assert!(
            (arrived_s - 9.25).abs() < 4.0 * dt_s,
            "travel plus opening should take about 9.25 s, took {arrived_s}"
        );
        assert!((elevator.car_height_m() - 7.0).abs() < 1.0e-9);
        assert!(!elevator.is_boardable(0));
        assert!(!elevator.is_boardable(1));
    }

    #[test]
    fn calls_are_served_in_a_deterministic_order_and_are_idempotent() {
        let dt_s = 1.0 / 60.0;
        let mut elevator = Elevator::new(spec(), 0).expect("elevator");
        elevator.call(2).expect("call 2");
        elevator.call(1).expect("call 1");
        elevator.call(2).expect("duplicate call 2");
        assert_eq!(elevator.pending_calls(), &[1, 2]);

        run_until(&mut elevator, dt_s, 30.0, |e| e.is_boardable(1)).expect("serves floor 1 first");
        assert_eq!(elevator.pending_calls(), &[2]);
        run_until(&mut elevator, dt_s, 30.0, |e| e.is_boardable(2)).expect("then floor 2");
        assert!(elevator.pending_calls().is_empty());
    }

    #[test]
    fn the_same_calls_and_steps_replay_to_the_same_trajectory() {
        let dt_s = 1.0 / 60.0;
        let trajectory = |()| {
            let mut elevator = Elevator::new(spec(), 0).expect("elevator");
            elevator.call(2).expect("call");
            let mut samples = Vec::new();
            for _ in 0..1_200 {
                elevator.update(dt_s).expect("update");
                samples.push((
                    elevator.car_height_m().to_bits(),
                    elevator.door_opening_m().to_bits(),
                ));
            }
            samples
        };
        assert_eq!(trajectory(()), trajectory(()));
    }
}
