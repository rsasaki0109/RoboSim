"""Synthetic tests of the retrospective consistency diagnostic."""

import math
import unittest

from audit_f1tenth_reference import compare, compare_integrals, yaw


class ReferenceTests(unittest.TestCase):
    def test_integral_does_not_apply_new_velocity_retroactively(self):
        poses = [(0, 0, 0, 0), (20_000_000, .03, 0, .02)]
        twists = [(0, 1, 0, 1), (10_000_000, 2, 0, 1), (20_000_000, 100, 0, 100)]
        result = compare_integrals(poses, twists)
        self.assertEqual(result["paired_intervals"], 1)
        self.assertAlmostEqual(result["planar_displacement_difference_rms_m"], 0)
        self.assertAlmostEqual(result["unscaled_angular_increment_difference_rms"], 0)

    def test_integral_rejects_interior_gap_even_if_endpoint_is_fresh(self):
        poses = [(0, 0, 0, 0), (30_000_000, 0, 0, 0)]
        result = compare_integrals(poses, [(0, 0, 0, 0), (25_000_000, 0, 0, 0)])
        self.assertEqual(result["paired_intervals"], 0)
        self.assertEqual(result["incompletely_observed_interval_count"], 1)

    def test_integral_replacement_at_expiry_has_no_positive_gap(self):
        result = compare_integrals(
            [(0, 0, 0, 0), (40_000_000, .04, 0, 0)],
            [(0, 1, 0, 0), (20_000_000, 1, 0, 0)],
        )
        self.assertEqual(result["paired_intervals"], 1)
        self.assertAlmostEqual(result["planar_displacement_difference_rms_m"], 0)

    def test_units_wrap_and_scale_are_diagnostic_only(self):
        poses = [(0, 0, 0, math.pi - .005), (10_000_000, .01, 0, -math.pi + .005)]
        result = compare(poses, [(10_000_000, 1, 0, .01)])
        self.assertEqual(result["paired_intervals"], 1)
        self.assertAlmostEqual(result["pose_heading_rate_rms_rad_s"], 1)
        self.assertAlmostEqual(result["diagnostic_through_origin_angular_scale"], 100)
        self.assertEqual(result["planar_linear_difference_rms_m_s"], 0)
        self.assertFalse(result["scale_applied"])
        self.assertFalse(result["reference_qualified"])

    def test_future_and_stale_samples_are_not_paired(self):
        poses = [(0, 0, 0, 0), (30_000_000, 0, 0, 0)]
        for twists in [[(30_000_001, 0, 0, 0)], [(0, 0, 0, 0)], []]:
            result = compare(poses, twists)
            self.assertEqual(result["paired_intervals"], 0)
            self.assertIsNone(result["diagnostic_through_origin_angular_scale"])

    def test_bad_time_and_quaternion_rejected(self):
        with self.assertRaises(ValueError):
            compare([(1, 0, 0, 0), (1, 0, 0, 0)], [])
        with self.assertRaises(ValueError):
            yaw((0, 0, 0, 0))
        self.assertAlmostEqual(yaw((0, 0, math.sin(.25), math.cos(.25))), .5)


if __name__ == "__main__":
    unittest.main()
