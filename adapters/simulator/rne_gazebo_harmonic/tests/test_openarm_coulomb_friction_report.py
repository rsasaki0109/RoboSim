from __future__ import annotations

import importlib.util
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[4]
SCRIPT = (
    ROOT
    / "adapters/simulator/rne_gazebo_harmonic/build_openarm_coulomb_friction_report.py"
)
SPEC = importlib.util.spec_from_file_location("openarm_coulomb_report", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
REPORT = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(REPORT)


def checks(performance: str, structural: str, capacity: str):
    values = []
    for requirement in sorted(REPORT.PERFORMANCE_IDS):
        values.append({"requirement_id": requirement, "status": performance})
    for requirement in sorted(REPORT.STRUCTURAL_IDS):
        values.append({"requirement_id": requirement, "status": structural})
    values.append({"requirement_id": REPORT.CAPACITY_ID, "status": capacity})
    return values


class OpenArmCoulombFrictionReportTests(unittest.TestCase):
    def test_supported_failure_cannot_be_relabelled_as_boundary(self) -> None:
        self.assertEqual(REPORT.outcome_status(checks("failed", "passed", "passed"), True), "failed")

    def test_outside_performance_failure_requires_structural_evidence(self) -> None:
        self.assertEqual(
            REPORT.outcome_status(checks("failed", "passed", "failed"), False),
            "expected_boundary_failure",
        )
        self.assertEqual(
            REPORT.outcome_status(checks("failed", "failed", "failed"), False),
            "failed",
        )
        self.assertEqual(
            REPORT.outcome_status(checks("passed", "passed", "failed"), False),
            "outside_declared_envelope",
        )


if __name__ == "__main__":
    unittest.main()
