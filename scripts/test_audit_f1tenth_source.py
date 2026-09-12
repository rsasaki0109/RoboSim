"""Synthetic audit-contract tests; no physical qualification is implied."""

import unittest
from pathlib import Path

from audit_f1tenth_source import ChannelAudit, SourceSpec, verify_source


class AuditTests(unittest.TestCase):
    def test_unstamped_is_unknown_not_zero_latency(self):
        audit = ChannelAudit()
        audit.add(100)
        result = audit.report()
        self.assertIsNone(result["min_bag_minus_header_ns"])
        self.assertEqual(result["unstamped_samples"], 1)

    def test_anomalies_are_retained_without_clock_repair(self):
        audit = ChannelAudit()
        audit.add(100, 200, 0.0, 255)
        audit.add(90, 200, 12.0, 0)
        audit.add(120, 180, 0.0, 7)
        result = audit.report()
        self.assertEqual(result["min_bag_minus_header_ns"], -110)
        self.assertEqual(result["max_bag_minus_header_ns"], -60)
        self.assertEqual(result["bag_time_reversals"], 1)
        self.assertEqual(result["header_time_duplicates"], 1)
        self.assertEqual(result["header_time_reversals"], 1)
        self.assertEqual(result["zero_input_voltage_samples"], 2)
        self.assertEqual(result["fault_outside_declared_samples"], 2)

    def test_invalid_values_do_not_mutate_audit(self):
        audit = ChannelAudit()
        for values in [(-1,), (1, -1), (1, 1, float("nan"))]:
            with self.assertRaises(ValueError):
                audit.add(*values)
        self.assertEqual(audit.count, 0)

    def test_non_capture_file_is_rejected_before_reader_import(self):
        with self.assertRaises(ValueError):
            verify_source(Path(__file__))

    def test_matching_size_cannot_substitute_for_hash(self):
        path = Path(__file__)
        specs = {path.name: SourceSpec(path.stat().st_size, "0" * 64)}
        with self.assertRaisesRegex(ValueError, "hash mismatch"):
            verify_source(path, specs)


if __name__ == "__main__":
    unittest.main()
