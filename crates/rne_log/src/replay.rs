//! Log replay helpers.

use crate::record::{LogRecord, ReplayCompatibility, ReplayCompatibilityError, SimulationLog};
use rne_core::SimTime;
use rne_ecs::Entity;
use rne_robot::{ActuatorCommand, ActuatorCommandBuffer};

/// Replays actuator commands from a log into a command buffer.
pub fn replay_commands(log: &SimulationLog, buffer: &mut ActuatorCommandBuffer) {
    for record in log.records() {
        let LogRecord::ActuatorCommand {
            sim_ticks,
            wheel_index,
            velocity_rad_s,
            ..
        } = record
        else {
            continue;
        };

        let (Some(wheel_index), Some(velocity_rad_s)) = (wheel_index, velocity_rad_s) else {
            continue;
        };

        buffer.push(
            ActuatorCommand::WheelVelocity {
                wheel: Entity::from_raw(*wheel_index),
                velocity_rad_s: *velocity_rad_s,
            },
            SimTime::from_ticks(*sim_ticks),
        );
    }
}

/// Validates replay compatibility, then replays actuator commands into a command buffer.
pub fn replay_commands_checked(
    log: &SimulationLog,
    expected: &ReplayCompatibility,
    buffer: &mut ActuatorCommandBuffer,
) -> Result<(), ReplayCompatibilityError> {
    log.validate_compatibility(expected)?;
    replay_commands(log, buffer);
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::record::{ReplayHeader, SimulationLog};
    use rne_core::SimDuration;
    use rne_math::Seconds;
    use rne_robot::{ActuatorCommand, ActuatorCommandEntry};
    use tempfile::NamedTempFile;

    #[test]
    fn replay_reproduces_command_sequence() {
        let mut log = SimulationLog::new();
        log.record_actuator_command(&ActuatorCommandEntry {
            command: ActuatorCommand::WheelVelocity {
                wheel: Entity::from_raw(10),
                velocity_rad_s: 2.0,
            },
            sim_time: SimTime::from_seconds(Seconds::new(0.0)),
            sequence: 0,
        });

        let file = NamedTempFile::new().unwrap();
        log.write_jsonl(file.path()).unwrap();
        let loaded = SimulationLog::read_jsonl(file.path()).unwrap();

        let mut buffer = ActuatorCommandBuffer::new();
        replay_commands(&loaded, &mut buffer);
        assert_eq!(buffer.len(), 1);
    }

    #[test]
    fn checked_replay_rejects_incompatible_header_before_commands() {
        let mut log = SimulationLog::new_with_header(ReplayHeader::new(
            42,
            1,
            SimDuration::from_ticks(16_666_666),
        ));
        log.record_actuator_command(&ActuatorCommandEntry {
            command: ActuatorCommand::WheelVelocity {
                wheel: Entity::from_raw(10),
                velocity_rad_s: 2.0,
            },
            sim_time: SimTime::from_seconds(Seconds::new(0.0)),
            sequence: 0,
        });
        let expected = ReplayCompatibility::current(7, 1, SimDuration::from_ticks(16_666_666));
        let mut buffer = ActuatorCommandBuffer::new();

        let error = replay_commands_checked(&log, &expected, &mut buffer).unwrap_err();

        assert!(matches!(
            error,
            ReplayCompatibilityError::Mismatch {
                field: "world_seed",
                ..
            }
        ));
        assert_eq!(buffer.len(), 0);
    }
}
