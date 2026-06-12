#!/usr/bin/env python3
"""Unit tests for bridge simulation control helpers."""

from __future__ import annotations

import unittest

try:
    import rne_py
except ImportError:
    rne_py = None

from simulation_interfaces.msg import Result, SimulationState
from simulation_interfaces.srv import ResetSimulation

from sim_control import BridgeSim


@unittest.skipUnless(rne_py is not None, "rne_py not installed")
class BridgeSimTests(unittest.TestCase):
    def test_reset_scope_all_restarts_pose_and_time(self) -> None:
        bridge = BridgeSim.new(rne_py.DiffDriveSim())
        bridge.step_if_playing(6.0)
        self.assertGreater(bridge.sim_ticks, 0)
        self.assertGreater(bridge.observation.base_x, 0.0)

        bridge.reset(ResetSimulation.Request.SCOPE_ALL)
        self.assertEqual(bridge.sim_ticks, 0)
        self.assertLess(abs(bridge.observation.base_x), 0.01)

    def test_paused_stepping_advances_pose(self) -> None:
        bridge = BridgeSim.new(rne_py.DiffDriveSim())
        bridge.set_playback(SimulationState.STATE_PAUSED)
        error = bridge.step_while_paused(30, 6.0)
        self.assertIsNone(error)
        self.assertGreater(bridge.observation.base_x, 0.05)

    def test_step_while_playing_rejects_when_not_paused(self) -> None:
        bridge = BridgeSim.new(rne_py.DiffDriveSim())
        error = bridge.step_while_paused(1, 6.0)
        self.assertIsNotNone(error)
        self.assertEqual(error.result, Result.RESULT_INCORRECT_STATE)


if __name__ == "__main__":
    unittest.main()
