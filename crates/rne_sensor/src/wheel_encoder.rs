//! Wheel encoder sensor specification and sampling.

use rne_data::WheelEncoderSample;
use rne_ecs::{Entity, World};
use rne_robot::Actuator;

/// Wheel encoder parameters.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct WheelEncoderSpec {
    /// Actuator entity providing wheel velocity.
    pub actuator: Entity,
}

/// Samples wheel position and velocity from an actuator.
pub fn sample_wheel_encoder(world: &World, spec: &WheelEncoderSpec) -> WheelEncoderSample {
    let actuator = world
        .get::<Actuator>(spec.actuator)
        .expect("wheel encoder actuator");

    WheelEncoderSample {
        position_rad: actuator.target.position_rad,
        velocity_rad_s: actuator.target.velocity_rad_s,
    }
}
