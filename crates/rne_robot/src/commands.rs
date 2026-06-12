//! Actuator command types and deferred command buffer.

use bevy_ecs::prelude::Resource;
use rne_core::{SimDuration, SimTime};
use rne_ecs::Entity;
use rne_math::Vec3;
use std::collections::VecDeque;

/// Robot-native actuator command.
#[derive(Clone, Debug, PartialEq)]
pub enum ActuatorCommand {
    /// Position command for a revolute joint.
    JointPosition {
        /// Target joint entity.
        joint: Entity,
        /// Target position in radians.
        position_rad: f64,
    },
    /// Velocity command for a revolute joint.
    JointVelocity {
        /// Target joint entity.
        joint: Entity,
        /// Target velocity in radians per second.
        velocity_rad_s: f64,
    },
    /// Effort command for a revolute joint.
    JointEffort {
        /// Target joint entity.
        joint: Entity,
        /// Target effort in newton-meters.
        effort_nm: f64,
    },
    /// Wheel velocity command in radians per second.
    WheelVelocity {
        /// Wheel actuator entity.
        wheel: Entity,
        /// Target wheel angular velocity in radians per second.
        velocity_rad_s: f64,
    },
    /// Gripper width command in meters.
    GripperWidth {
        /// Gripper actuator entity.
        actuator: Entity,
        /// Target opening width in meters.
        width_m: f64,
    },
    /// Body wrench applied to a link.
    BodyWrench {
        /// Target link entity.
        link: Entity,
        /// Force in newtons.
        force_n: Vec3,
        /// Torque in newton-meters.
        torque_nm: Vec3,
    },
    /// Ackermann steering command.
    Ackermann {
        /// Forward speed in meters per second.
        speed_m_s: f64,
        /// Steering angle in radians.
        steering_rad: f64,
    },
}

/// One queued actuator command with timing metadata.
#[derive(Clone, Debug, PartialEq)]
pub struct ActuatorCommandEntry {
    /// Command payload.
    pub command: ActuatorCommand,
    /// Simulation time when the command was issued.
    pub sim_time: SimTime,
    /// Monotonic sequence number.
    pub sequence: u64,
}

/// Deferred actuator command queue.
#[derive(Resource, Debug, Default)]
pub struct ActuatorCommandBuffer {
    commands: VecDeque<ActuatorCommandEntry>,
    next_sequence: u64,
    max_age: Option<SimDuration>,
}

impl ActuatorCommandBuffer {
    /// Creates an empty command buffer.
    pub fn new() -> Self {
        Self::default()
    }

    /// Sets the maximum age after which commands are ignored.
    pub fn with_max_age(mut self, max_age: SimDuration) -> Self {
        self.max_age = Some(max_age);
        self
    }

    /// Queues a command for the next application step.
    pub fn push(&mut self, command: ActuatorCommand, sim_time: SimTime) -> u64 {
        let sequence = self.next_sequence;
        self.next_sequence += 1;
        self.commands.push_back(ActuatorCommandEntry {
            command,
            sim_time,
            sequence,
        });
        sequence
    }

    /// Returns pending commands without removing them.
    pub fn pending(&self) -> impl DoubleEndedIterator<Item = &ActuatorCommandEntry> {
        self.commands.iter()
    }

    /// Removes stale commands relative to the current simulation time.
    pub fn discard_stale(&mut self, current_time: SimTime) {
        let Some(max_age) = self.max_age else {
            return;
        };

        while let Some(entry) = self.commands.front() {
            let age_ticks = current_time.ticks().saturating_sub(entry.sim_time.ticks());
            if age_ticks > max_age.ticks() {
                self.commands.pop_front();
            } else {
                break;
            }
        }
    }

    /// Drains all pending commands in FIFO order.
    pub fn drain(&mut self) -> impl Iterator<Item = ActuatorCommandEntry> + '_ {
        self.commands.drain(..)
    }

    /// Returns the number of pending commands.
    pub fn len(&self) -> usize {
        self.commands.len()
    }

    /// Returns whether the buffer is empty.
    pub fn is_empty(&self) -> bool {
        self.commands.is_empty()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rne_math::Seconds;

    #[test]
    fn command_sequence_order() {
        let mut buffer = ActuatorCommandBuffer::new();
        let t0 = SimTime::from_seconds(Seconds::new(0.0));
        let seq_a = buffer.push(
            ActuatorCommand::Ackermann {
                speed_m_s: 1.0,
                steering_rad: 0.0,
            },
            t0,
        );
        let seq_b = buffer.push(
            ActuatorCommand::Ackermann {
                speed_m_s: 2.0,
                steering_rad: 0.0,
            },
            t0,
        );
        assert_eq!(seq_a, 0);
        assert_eq!(seq_b, 1);
    }
}
