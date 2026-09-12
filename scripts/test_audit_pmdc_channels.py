"""Synthetic tests for PMDC retained-channel integrity and self-consistency."""

import hashlib
import json
import tempfile
import unittest
from pathlib import Path

from audit_pmdc_channels import audit_channels, read_partition
from convert_pmdc_prbs9 import CHANNELS


def canonical(value):
    return json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False)


def record(row, time_us, raw):
    values = {channel: "0" for channel in CHANNELS}
    values.update({
        "time": str(time_us),
        "rawCurrent": str(raw),
        "Current": str(2 * raw + 1),
        "rawVoltageA1": str(raw),
        "VoltageA1": str(3 * raw + 2),
        "rawVoltageB1": str(raw),
        "VoltageB1": str(4 * raw + 3),
        "MotorVoltage": str(raw + 1),
    })
    return {
        "partition": "training",
        "run_id": "prbs9_motor_a_trial_01",
        "source_row": row,
        "values": values,
    }


def write_artifact(path, records, digest_override=None):
    manifest = {
        "kind": "rne_pmdc_prbs9_partition",
        "partition": "training",
        "final_partition_read": False,
        "sample_counts": {"prbs9_motor_a_trial_01": len(records)},
        "source_sha256": "source",
    }
    lines = [canonical(value) for value in records]
    digest = hashlib.sha256()
    for line in lines:
        digest.update(line.encode())
        digest.update(b"\n")
    trailer = {
        "kind": "rne_pmdc_prbs9_partition_end",
        "record_count": len(records),
        "records_sha256": digest_override or digest.hexdigest(),
    }
    path.write_text("\n".join([canonical(manifest), *lines, canonical(trailer)]) + "\n")


class ChannelAuditTests(unittest.TestCase):
    def test_affine_and_terminal_difference_self_consistency(self):
        records = [record(3, 0, 1), record(4, 10_000, 2), record(5, 20_000, 3)]
        report = audit_channels({"source_sha256": "source"}, records)
        current = report["source_derived_affine_self_consistency"]["current_from_raw"]
        self.assertAlmostEqual(current["slope"], 2.0)
        self.assertAlmostEqual(current["intercept"], 1.0)
        self.assertAlmostEqual(current["residual_rmse"], 0.0)
        self.assertAlmostEqual(report["motor_voltage_minus_b1_minus_a1"]["residual_rmse_v"], 0.0)
        self.assertEqual(report["timing"]["prbs9_motor_a_trial_01"]["non_10000us_delta_count"], 0)
        self.assertFalse(report["physical_accuracy_validated"])

    def test_record_digest_mismatch_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "bad.jsonl"
            write_artifact(path, [record(3, 0, 1)], digest_override="bad")
            with self.assertRaisesRegex(ValueError, "digest mismatch"):
                read_partition(path)

    def test_source_row_reversal_is_rejected(self):
        records = [record(4, 0, 1), record(3, 10_000, 2)]
        with self.assertRaisesRegex(ValueError, "strictly increasing"):
            audit_channels({"source_sha256": "source"}, records)

    def test_nonnumeric_value_is_not_repaired(self):
        bad = record(3, 0, 1)
        bad["values"]["Current"] = "missing"
        with self.assertRaisesRegex(ValueError, "nonnumeric Current"):
            audit_channels({"source_sha256": "source"}, [bad, record(4, 10_000, 2)])


if __name__ == "__main__":
    unittest.main()
