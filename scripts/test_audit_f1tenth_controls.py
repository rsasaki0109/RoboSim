"""Synthetic tests for exposed F1TENTH control-path diagnostics."""

import unittest

from audit_f1tenth_controls import affine_report, nearest_pairs, summarize


class ControlAuditTests(unittest.TestCase):
    def test_nearest_pairing_preserves_signed_delta_and_fixed_limit(self):
        source = [(10, 1.0), (30, 2.0), (50, 3.0)]
        target = [(9, 4.0), (32, 7.0), (60, 10.0)]
        self.assertEqual(
            nearest_pairs(source, target, 3),
            [(1.0, 4.0, -1), (2.0, 7.0, 2)],
        )

    def test_affine_diagnostic_recovers_mapping_without_applying_it(self):
        source = [(10, 0.0), (20, 1.0), (30, 2.0)]
        target = [(11, 1.0), (21, 3.0), (31, 5.0)]
        report = affine_report(source, target)
        self.assertEqual(report["slope"], 2.0)
        self.assertEqual(report["intercept"], 1.0)
        self.assertEqual(report["residual_rms"], 0.0)
        self.assertFalse(report["mapping_applied"])
        self.assertFalse(report["calibration_qualified"])

    def test_invalid_order_empty_and_unexcited_values_fail_closed(self):
        with self.assertRaises(ValueError):
            nearest_pairs([(2, 0.0), (1, 0.0)], [(1, 0.0)])
        with self.assertRaises(ValueError):
            summarize([])
        with self.assertRaises(ValueError):
            affine_report([(1, 1.0), (2, 1.0), (3, 1.0)], [(1, 0.0), (2, 1.0), (3, 2.0)])


if __name__ == "__main__":
    unittest.main()
