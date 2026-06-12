"""Headless diff-drive bridge state for simulation_interfaces control."""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

from simulation_interfaces.msg import Result, SimulationState
from simulation_interfaces.srv import ResetSimulation

if TYPE_CHECKING:
    import rne_py

SIM_DT_NS = 1_000_000_000 // 60


@dataclass
class BridgeSim:
    """Shared simulation playback and reset state for the ROS bridge loop."""

    sim: rne_py.DiffDriveSim
    obs: object
    sim_ticks: int = 0
    playback: int = SimulationState.STATE_PLAYING

    @classmethod
    def new(cls, sim: rne_py.DiffDriveSim) -> BridgeSim:
        return cls(sim=sim, obs=sim.reset())

    @property
    def observation(self) -> object:
        return self.obs

    def reset(self, scope: int) -> Result:
        scope = _normalize_reset_scope(scope)
        if scope & ResetSimulation.Request.SCOPE_TIME:
            self.sim_ticks = 0
        if scope & ResetSimulation.Request.SCOPE_STATE:
            self.obs = self.sim.reset()
        return _ok_result()

    def set_playback(self, target: int) -> Result:
        from simulation_interfaces.srv import SetSimulationState

        if target == self.playback:
            return _result_code(
                SetSimulationState.Response.ALREADY_IN_TARGET_STATE, ""
            )

        if target == SimulationState.STATE_PLAYING:
            if self.playback == SimulationState.STATE_STOPPED:
                self.obs = self.sim.reset()
                self.sim_ticks = 0
            self.playback = SimulationState.STATE_PLAYING
            return _ok_result()

        if target == SimulationState.STATE_PAUSED:
            if self.playback == SimulationState.STATE_STOPPED:
                return _result_code(
                    SetSimulationState.Response.INCORRECT_TRANSITION,
                    "cannot pause while simulation is stopped",
                )
            self.playback = SimulationState.STATE_PAUSED
            return _ok_result()

        if target == SimulationState.STATE_STOPPED:
            self.playback = SimulationState.STATE_STOPPED
            return self.reset(ResetSimulation.Request.SCOPE_ALL)

        return _fail_operation("unsupported simulation state")

    def playback_state(self) -> SimulationState:
        return SimulationState(state=self.playback)

    def step_if_playing(self, wheel_velocity_rad_s: float) -> bool:
        if self.playback != SimulationState.STATE_PLAYING:
            return False
        self._step_once(wheel_velocity_rad_s)
        return True

    def step_while_paused(
        self, steps: int, wheel_velocity_rad_s: float
    ) -> Result | None:
        if self.playback != SimulationState.STATE_PAUSED:
            return _incorrect_state("stepping requires paused simulation")
        for _ in range(steps):
            self._step_once(wheel_velocity_rad_s)
        return None

    def _step_once(self, wheel_velocity_rad_s: float) -> None:
        self.obs = self.sim.step(wheel_velocity_rad_s, wheel_velocity_rad_s)
        self.sim_ticks += SIM_DT_NS


def _normalize_reset_scope(scope: int) -> int:
    if scope == ResetSimulation.Request.SCOPE_DEFAULT:
        return ResetSimulation.Request.SCOPE_ALL
    return scope


def _ok_result() -> Result:
    return _result_code(Result.RESULT_OK, "")


def _incorrect_state(message: str) -> Result:
    return _result_code(Result.RESULT_INCORRECT_STATE, message)


def _fail_operation(message: str) -> Result:
    return _result_code(Result.RESULT_OPERATION_FAILED, message)


def _result_code(code: int, message: str) -> Result:
    return Result(result=code, error_message=message)
