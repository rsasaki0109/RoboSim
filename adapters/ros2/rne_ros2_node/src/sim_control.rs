//! Headless diff-drive bridge state for simulation_interfaces control.

use rne_ai::{DiffDriveObservation, DiffDriveSim};
use simulation_interfaces::{
    msg::{Result as SimResult, SimulationState},
    srv::ResetSimulation_Request,
};

pub const SIM_DT_NS: u64 = 1_000_000_000 / 60;

/// Shared simulation playback and reset state for the ROS bridge loop.
pub struct BridgeSim {
    sim: DiffDriveSim,
    obs: DiffDriveObservation,
    sim_ticks: u64,
    playback: u8,
}

impl BridgeSim {
    /// Creates a new bridge simulation in the playing state.
    pub fn new() -> Self {
        let mut sim = DiffDriveSim::new();
        let obs = sim.reset();
        Self {
            sim,
            obs,
            sim_ticks: 0,
            playback: SimulationState::STATE_PLAYING,
        }
    }

    /// Latest observation from the diff-drive simulation.
    pub fn observation(&self) -> &DiffDriveObservation {
        &self.obs
    }

    /// Current simulation clock in nanosecond ticks.
    pub fn sim_ticks(&self) -> u64 {
        self.sim_ticks
    }

    /// Current playback state (`SimulationState::STATE_*`).
    pub fn playback(&self) -> u8 {
        self.playback
    }

    /// Resets simulation scope per `simulation_interfaces/ResetSimulation`.
    pub fn reset(&mut self, scope: u8) -> SimResult {
        let scope = normalize_reset_scope(scope);
        if scope & ResetSimulation_Request::SCOPE_TIME != 0 {
            self.sim_ticks = 0;
        }
        if scope & ResetSimulation_Request::SCOPE_STATE != 0 {
            self.obs = self.sim.reset();
        }
        ok_result()
    }

    /// Sets playback state per `simulation_interfaces/SetSimulationState`.
    pub fn set_playback(&mut self, target: u8) -> SimResult {
        if target == self.playback {
            return result_code(
                simulation_interfaces::srv::SetSimulationState_Response::ALREADY_IN_TARGET_STATE,
                String::new(),
            );
        }

        match target {
            SimulationState::STATE_PLAYING => {
                if self.playback == SimulationState::STATE_STOPPED {
                    self.obs = self.sim.reset();
                    self.sim_ticks = 0;
                }
                self.playback = SimulationState::STATE_PLAYING;
                ok_result()
            }
            SimulationState::STATE_PAUSED => {
                if self.playback == SimulationState::STATE_STOPPED {
                    return result_code(
                        simulation_interfaces::srv::SetSimulationState_Response::INCORRECT_TRANSITION,
                        "cannot pause while simulation is stopped".into(),
                    );
                }
                self.playback = SimulationState::STATE_PAUSED;
                ok_result()
            }
            SimulationState::STATE_STOPPED => {
                self.playback = SimulationState::STATE_STOPPED;
                self.reset(ResetSimulation_Request::SCOPE_ALL)
            }
            _ => fail_operation("unsupported simulation state"),
        }
    }

    /// Returns the current playback state message.
    pub fn playback_state(&self) -> SimulationState {
        SimulationState {
            state: self.playback,
        }
    }

    /// Advances one tick when playback is active.
    pub fn step_if_playing(&mut self, wheel_velocity_rad_s: f64) -> bool {
        if self.playback != SimulationState::STATE_PLAYING {
            return false;
        }
        self.step_once(wheel_velocity_rad_s);
        true
    }

    /// Steps the simulation while paused, as required by step/action interfaces.
    pub fn step_while_paused(
        &mut self,
        steps: u64,
        wheel_velocity_rad_s: f64,
    ) -> Result<(), SimResult> {
        if self.playback != SimulationState::STATE_PAUSED {
            return Err(incorrect_state("stepping requires paused simulation"));
        }
        for _ in 0..steps {
            self.step_once(wheel_velocity_rad_s);
        }
        Ok(())
    }

    fn step_once(&mut self, wheel_velocity_rad_s: f64) {
        self.obs = self.sim.step(wheel_velocity_rad_s, wheel_velocity_rad_s);
        self.sim_ticks = self.sim_ticks.saturating_add(SIM_DT_NS);
    }
}

fn normalize_reset_scope(scope: u8) -> u8 {
    if scope == ResetSimulation_Request::SCOPE_DEFAULT {
        ResetSimulation_Request::SCOPE_ALL
    } else {
        scope
    }
}

fn ok_result() -> SimResult {
    result_code(SimResult::RESULT_OK, String::new())
}

fn incorrect_state(message: impl Into<String>) -> SimResult {
    result_code(SimResult::RESULT_INCORRECT_STATE, message.into())
}

fn fail_operation(message: impl Into<String>) -> SimResult {
    result_code(SimResult::RESULT_OPERATION_FAILED, message.into())
}

fn result_code(code: u8, message: String) -> SimResult {
    SimResult {
        result: code,
        error_message: message,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reset_scope_all_restarts_pose_and_time() {
        let mut bridge = BridgeSim::new();
        bridge.step_if_playing(6.0);
        assert!(bridge.sim_ticks() > 0);
        assert!(bridge.observation().base_x_m > 0.0);

        bridge.reset(ResetSimulation_Request::SCOPE_ALL);
        assert_eq!(bridge.sim_ticks(), 0);
        assert!(bridge.observation().base_x_m.abs() < 0.01);
    }

    #[test]
    fn paused_stepping_advances_pose() {
        let mut bridge = BridgeSim::new();
        bridge.set_playback(SimulationState::STATE_PAUSED);
        bridge
            .step_while_paused(30, 6.0)
            .expect("paused stepping should succeed");
        assert!(bridge.observation().base_x_m > 0.05);
    }

    #[test]
    fn step_while_playing_rejects_when_not_paused() {
        let mut bridge = BridgeSim::new();
        let err = bridge
            .step_while_paused(1, 6.0)
            .expect_err("playing sim should reject paused stepping");
        assert_eq!(err.result, SimResult::RESULT_INCORRECT_STATE);
    }
}
