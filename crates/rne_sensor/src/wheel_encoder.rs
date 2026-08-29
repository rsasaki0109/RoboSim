//! Wheel encoder sensor specification and sampling.

use rne_data::WheelEncoderSample;
use rne_ecs::{Entity, World};
use rne_physics::JointState;
use rne_robot::{Actuator, Joint};

/// Wheel encoder parameters.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct WheelEncoderSpec {
    /// Actuator whose associated revolute joint is measured.
    pub actuator: Entity,
}

/// Samples the completed wheel coordinate associated with an actuator.
///
/// Backend-synchronized [`JointState`] is authoritative when present. The
/// backend-free kinematic drive stores its realized coordinate on [`Joint`],
/// which is used only when no physics-backend state exists. Actuator targets
/// are commands and are never reported as encoder measurements.
pub fn sample_wheel_encoder(world: &World, spec: &WheelEncoderSpec) -> WheelEncoderSample {
    let actuator = world
        .get::<Actuator>(spec.actuator)
        .expect("wheel encoder actuator");
    let joint_entity = actuator.joint.expect("wheel encoder actuator joint");

    if let Some(state) = world.get::<JointState>(joint_entity).copied() {
        return match state {
            JointState::Revolute {
                position_rad,
                velocity_rad_s,
            } => WheelEncoderSample {
                position_rad,
                velocity_rad_s,
            },
            JointState::Prismatic { .. } | JointState::Fixed => {
                panic!("wheel encoder requires revolute joint state")
            }
        };
    }

    let joint = world
        .get::<Joint>(joint_entity)
        .expect("wheel encoder joint");

    WheelEncoderSample {
        position_rad: joint.position,
        velocity_rad_s: joint.velocity,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rne_math::Vec3;
    use rne_robot::{ActuatorLimits, ActuatorTarget, ControlMode, JointKind, JointLimits};

    fn spawn_fixture(world: &mut World) -> (Entity, Entity) {
        let robot = world.spawn_empty().id();
        let parent = world.spawn_empty().id();
        let wheel = world
            .spawn(Joint {
                robot,
                parent_link: parent,
                child_link: Entity::PLACEHOLDER,
                kind: JointKind::Continuous,
                limits: JointLimits::default(),
                axis: Vec3::Y,
                position: 0.2,
                velocity: 0.4,
            })
            .id();
        world.get_mut::<Joint>(wheel).unwrap().child_link = wheel;
        let actuator = world
            .spawn(Actuator {
                robot,
                joint: Some(wheel),
                name: "wheel".to_owned(),
                mode: ControlMode::Velocity,
                target: ActuatorTarget {
                    position_rad: 3.0,
                    velocity_rad_s: 6.0,
                    effort_nm: 0.0,
                },
                limits: ActuatorLimits::default(),
            })
            .id();
        (wheel, actuator)
    }

    #[test]
    fn backend_joint_state_is_measured_instead_of_command_target() {
        let mut world = World::new();
        let (wheel, actuator) = spawn_fixture(&mut world);
        world.entity_mut(wheel).insert(JointState::Revolute {
            position_rad: 0.25,
            velocity_rad_s: 0.0,
        });

        let sample = sample_wheel_encoder(&world, &WheelEncoderSpec { actuator });

        assert_eq!(sample.position_rad, 0.25);
        assert_eq!(sample.velocity_rad_s, 0.0);
    }

    #[test]
    fn backend_free_joint_coordinate_is_measured_without_target_fallback() {
        let mut world = World::new();
        let (_, actuator) = spawn_fixture(&mut world);

        let sample = sample_wheel_encoder(&world, &WheelEncoderSpec { actuator });

        assert_eq!(sample.position_rad, 0.2);
        assert_eq!(sample.velocity_rad_s, 0.4);
    }
}
