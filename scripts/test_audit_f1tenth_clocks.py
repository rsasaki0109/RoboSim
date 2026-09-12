"""Synthetic tests for the exposed F1TENTH clock audit contract."""

import unittest

from audit_f1tenth_clocks import CMD, POSE, TWIST, VESC, affine_clock_report, interval_report, report_from_rows


class ClockAuditTests(unittest.TestCase):
    def test_affine_diagnostic_centers_large_epoch_values_without_applying_mapping(self):
        epoch = 1_700_000_000_000_000_000
        rows = [
            (epoch + 50 + 2 * elapsed, epoch + elapsed)
            for elapsed in (0, 10, 20, 30)
        ]
        report = affine_clock_report(rows)
        self.assertEqual(report["bag_elapsed_from_header_elapsed_scale"], 2.0)
        self.assertEqual(report["elapsed_intercept_ns"], 0.0)
        self.assertEqual(report["residual_rms_ns"], 0.0)
        self.assertFalse(report["mapping_applied"])
        self.assertFalse(report["physical_latency_qualified"])

    def test_interval_report_preserves_reversals_and_duplicates(self):
        report = interval_report([10, 20, 20, 15, 30])
        self.assertEqual(report["reversal_count"], 1)
        self.assertEqual(report["duplicate_count"], 1)
        self.assertFalse(report["strictly_increasing"])

    def test_only_recorder_order_is_qualified_when_command_has_no_header(self):
        stamped = [(100, 10), (200, 110), (300, 210)]
        rows = {
            CMD: [(100, None), (200, None), (300, None)],
            POSE: stamped,
            TWIST: stamped,
            VESC: stamped,
        }
        report = report_from_rows(rows)
        self.assertTrue(report["deterministic_bag_time_replay_qualified"])
        self.assertFalse(report["command_capture_clock_qualified"])
        self.assertFalse(report["input_to_reference_capture_clock_qualified"])
        self.assertFalse(report["physical_latency_qualified"])
        self.assertFalse(report["clock_mapping_applied"])

    def test_nonmonotonic_recorder_time_fails_replay_qualification(self):
        stamped = [(100, 10), (200, 110), (300, 210)]
        rows = {
            CMD: [(100, None), (90, None), (300, None)],
            POSE: stamped,
            TWIST: stamped,
            VESC: stamped,
        }
        report = report_from_rows(rows)
        self.assertFalse(report["recorder_time_ordering_qualified"])
        self.assertFalse(report["deterministic_bag_time_replay_qualified"])

    def test_missing_channel_partial_stamp_and_bad_percentile_fail_closed(self):
        with self.assertRaises(ValueError):
            report_from_rows({CMD: [(1, None)]})
        rows = {
            CMD: [(100, None), (200, None), (300, None)],
            POSE: [(100, 10), (200, None), (300, 210)],
            TWIST: [(100, 10), (200, 110), (300, 210)],
            VESC: [(100, 10), (200, 110), (300, 210)],
        }
        with self.assertRaises(ValueError):
            report_from_rows(rows)
        with self.assertRaises(ValueError):
            interval_report([])


if __name__ == "__main__":
    unittest.main()
