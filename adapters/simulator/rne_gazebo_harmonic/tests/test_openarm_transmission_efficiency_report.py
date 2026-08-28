from __future__ import annotations

import importlib.util
from pathlib import Path
import tempfile
import unittest


SCRIPT_DIR = Path(__file__).resolve().parents[1]
SCRIPT = SCRIPT_DIR / "build_openarm_transmission_efficiency_report.py"
SPEC = importlib.util.spec_from_file_location("openarm_transmission_report", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
REPORT = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(REPORT)


def checks(performance: str, structural: str, capacity: str):
    values = [
        {"requirement_id": requirement, "status": performance}
        for requirement in sorted(REPORT.PERFORMANCE_IDS)
    ]
    values.extend(
        {"requirement_id": requirement, "status": structural}
        for requirement in sorted(REPORT.STRUCTURAL_IDS)
    )
    values.append({"requirement_id": REPORT.CAPACITY_ID, "status": capacity})
    return values


class OpenArmTransmissionEfficiencyReportTests(unittest.TestCase):
    def test_supported_performance_failure_is_not_boundary_evidence(self) -> None:
        self.assertEqual(
            REPORT.outcome_status(checks("failed", "passed", "passed"), True),
            "failed",
        )

    def test_outside_failure_requires_structural_evidence(self) -> None:
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

    def test_gazebo_diagnostics_keep_motor_and_joint_effort_separate(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            trace_path = root / "gazebo-success-trace.json"
            diagnostic = {
                "steps": [
                    {
                        "joint_applied_command_min": [-7.0],
                        "joint_applied_command_max": [6.0],
                        "joint_transmitted_effort_min_nm": [-5.25],
                        "joint_transmitted_effort_max_nm": [4.5],
                    }
                ]
            }
            REPORT.write_json(root / "gazebo-actuation-diagnostics-a.json", diagnostic)
            REPORT.write_json(root / "gazebo-actuation-diagnostics-b.json", diagnostic)
            digest = REPORT.sha256(root / "gazebo-actuation-diagnostics-a.json")
            REPORT.write_json(
                trace_path,
                {
                    "actuation_diagnostics_sha256": digest,
                    "replay_actuation_diagnostics_sha256": digest,
                    "observations": [{"actuator_realization": diagnostic["steps"][0]}],
                },
            )
            evidence = REPORT.gazebo_effort_evidence(trace_path, 0)
        self.assertEqual(evidence["motor_effort_command_peak_abs_nm"], 7.0)
        self.assertEqual(evidence["joint_transmitted_effort_peak_abs_nm"], 5.25)


if __name__ == "__main__":
    unittest.main()
