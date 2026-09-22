"""Gate regressions using a preserved full-contact native recording."""

import copy
import gzip
import json
from pathlib import Path
import unittest

from g1_native_audit import audit


class NativeAuditTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        path = Path(__file__).resolve().parents[1] / "docs/evidence/g1-contact-backflip/native-transfer/full-long-validation/step-500us.json.gz"
        cls.recording = json.loads(gzip.decompress(path.read_bytes()))

    def complete_fixture(self):
        # Synthetic telemetry is used only to exercise the gate combinations;
        # it is never written into the historical evidence recording.
        result = copy.deepcopy(self.recording)
        result.update(final_standing_error=0.005, final_second_min_base_height_m=0.78,
            effort_measurements_valid=True,
            joint_effort_audit=[dict(link_name=name, limit_nm=20,
                peak_measured_effort_nm=20, measured_steps=32000)
                for name in result["joint_link_names"]])
        return result

    def test_old_evidence_cannot_pass_with_missing_telemetry(self):
        result = audit(self.recording)
        self.assertFalse(result["passed_recorded_gates"])
        self.assertEqual(set(result["failed_gates"]), {
            "final_second_min_base_height_m", "final_standing_error", "measured_effort_limits"})
        self.assertFalse(audit({})["passed_recorded_gates"])

    def test_predictive_zero_impulse_self_pair_is_not_overlap(self):
        result = audit(self.complete_fixture())
        self.assertTrue(result["passed_recorded_gates"], result)

    def test_zero_impulse_does_not_excuse_negative_gap(self):
        result = self.complete_fixture()
        row = result["contact_separation_audit"][-1]
        self.assertNotIn("environment", (row["link_a"], row["link_b"]))
        row.update(min_solver_separation_m=-1e-9, negative_separation_steps=1)
        self.assertIn("no_nonadjacent_self_overlap", audit(result)["failed_gates"])

    def test_missing_self_pair_geometry_cannot_pass(self):
        result = self.complete_fixture()
        result["contact_separation_audit"] = [row for row in result["contact_separation_audit"]
            if "environment" in (row["link_a"], row["link_b"])]
        self.assertIn("contact_evidence", audit(result)["failed_gates"])

    def test_speed_boundary_and_nonfinite_values_fail(self):
        for speed in (1.05, float("nan"), float("inf"), None, True):
            result = self.complete_fixture()
            result["peak_joint_speed_ratio"] = speed
            self.assertIn("peak_joint_speed_ratio", audit(result)["failed_gates"])

    def test_one_missing_effort_sample_and_over_torque_fail(self):
        for change in ({"measured_steps": 31999}, {"peak_measured_effort_nm": 20.001}):
            result = self.complete_fixture()
            result["joint_effort_audit"][0].update(change)
            self.assertIn("measured_effort_limits", audit(result)["failed_gates"])

    def test_nonfoot_ground_and_malformed_gaps_fail(self):
        result = self.complete_fixture()
        result["contact_pair_audit"].append(dict(link_a="environment", link_b="torso_link", max_normal_impulse_ns=0))
        self.assertIn("no_nonfoot_ground_contact", audit(result)["failed_gates"])
        for row in (None, {"link_a": [], "link_b": "torso_link"},
                    dict(link_a="a", link_b="b", min_solver_separation_m=float("nan"))):
            result = self.complete_fixture()
            result["contact_separation_audit"].append(row)
            self.assertIn("contact_evidence", audit(result)["failed_gates"])


if __name__ == "__main__":
    unittest.main()
