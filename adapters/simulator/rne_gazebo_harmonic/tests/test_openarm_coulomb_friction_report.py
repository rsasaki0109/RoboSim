from __future__ import annotations

import importlib.util
from pathlib import Path
import tempfile
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
    def test_measured_effort_is_preferred_over_command_model(self) -> None:
        source, peak = REPORT.bounded_actuator_peak(
            "mujoco_native",
            Path("unused.json"),
            {
                "measured_effort_peak_abs_nm": 6.5,
                "limited_effort_command_peak_abs_nm": 7.0,
            },
            4,
        )
        self.assertEqual(source, "measured_actuator_force")
        self.assertEqual(peak, 6.5)

    def test_gazebo_uses_post_clamp_adapter_diagnostic(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            trace_path = Path(directory) / "gazebo-success-trace.json"
            REPORT.write_json(
                trace_path.parent / "gazebo-actuation-diagnostics-a.json",
                {
                    "steps": [
                        {
                            "joint_applied_command_min": [0.0, -7.0],
                            "joint_applied_command_max": [0.0, 6.75],
                        }
                    ]
                },
            )
            source, peak = REPORT.bounded_actuator_peak(
                "gazebo_sim", trace_path, {}, 1
            )
        self.assertEqual(source, "adapter_clamp_diagnostic")
        self.assertEqual(peak, 7.0)

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
