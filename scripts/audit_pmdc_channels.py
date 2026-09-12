"""Audit retained PMDC training channels without claiming physical calibration."""

import argparse
import hashlib
import json
import math
import statistics
from collections import Counter, defaultdict
from pathlib import Path

from convert_pmdc_prbs9 import CHANNELS


def _canonical(value):
    return json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False)


def read_partition(path):
    """Read and verify one complete converter artifact, including its record digest."""
    with path.open("r", encoding="utf-8", newline="") as stream:
        lines = iter(stream)
        try:
            manifest_text = next(lines).rstrip("\r\n")
            pending = next(lines)
        except StopIteration as error:
            raise ValueError("partition artifact is incomplete") from error
        manifest = json.loads(manifest_text)
        records = []
        digest = hashlib.sha256()
        for line in lines:
            text = pending.rstrip("\r\n")
            value = json.loads(text)
            if value.get("kind") == "rne_pmdc_prbs9_partition_end":
                raise ValueError("partition trailer appears before end of file")
            records.append(value)
            digest.update(text.encode())
            digest.update(b"\n")
            pending = line
        trailer = json.loads(pending.rstrip("\r\n"))

    if manifest.get("kind") != "rne_pmdc_prbs9_partition":
        raise ValueError("unexpected partition manifest kind")
    if manifest.get("partition") != "training" or manifest.get("final_partition_read") is not False:
        raise ValueError("channel audit accepts only the sealed training artifact")
    if trailer.get("kind") != "rne_pmdc_prbs9_partition_end":
        raise ValueError("partition trailer is missing")
    if trailer.get("record_count") != len(records):
        raise ValueError("partition record count mismatch")
    if trailer.get("records_sha256") != digest.hexdigest():
        raise ValueError("partition record digest mismatch")
    expected_counts = Counter(manifest.get("sample_counts", {}))
    actual_counts = Counter(record.get("run_id") for record in records)
    if dict(actual_counts) != dict(expected_counts):
        raise ValueError("partition run counts differ from the manifest")
    return manifest, records, trailer


def _finite(value, channel, source_row):
    try:
        number = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError(f"nonnumeric {channel} at source row {source_row}") from error
    if not math.isfinite(number):
        raise ValueError(f"nonfinite {channel} at source row {source_row}")
    return number


def _affine_fit(xs, ys):
    mean_x = statistics.fmean(xs)
    mean_y = statistics.fmean(ys)
    centered = math.fsum((x - mean_x) ** 2 for x in xs)
    if centered == 0.0:
        raise ValueError("cannot fit an affine conversion to a constant raw channel")
    slope = math.fsum((x - mean_x) * (y - mean_y) for x, y in zip(xs, ys)) / centered
    intercept = mean_y - slope * mean_x
    residuals = [y - (slope * x + intercept) for x, y in zip(xs, ys)]
    return {
        "slope": slope,
        "intercept": intercept,
        "residual_mean": statistics.fmean(residuals),
        "residual_rmse": math.sqrt(statistics.fmean(value * value for value in residuals)),
        "residual_max_abs": max(abs(value) for value in residuals),
        "sample_count": len(xs),
    }


def audit_channels(manifest, records):
    """Quantify source self-consistency while leaving all qualification flags false."""
    numeric = {channel: [] for channel in CHANNELS}
    timestamps = defaultdict(list)
    last_row = {}
    for record in records:
        run_id = record.get("run_id")
        source_row = record.get("source_row")
        values = record.get("values")
        if not isinstance(source_row, int) or source_row <= last_row.get(run_id, 0):
            raise ValueError("source rows are not strictly increasing within a run")
        last_row[run_id] = source_row
        if not isinstance(values, dict) or set(values) != set(CHANNELS):
            raise ValueError(f"channel set differs at source row {source_row}")
        for channel in CHANNELS:
            numeric[channel].append(_finite(values[channel], channel, source_row))
        time_text = values["time"]
        if not isinstance(time_text, str) or not time_text.isdigit():
            raise ValueError(f"time is not an unsigned source-microsecond integer at row {source_row}")
        timestamps[run_id].append(int(time_text))

    timing = {}
    for run_id in sorted(timestamps):
        times = timestamps[run_id]
        deltas = [later - earlier for earlier, later in zip(times, times[1:])]
        if not deltas:
            raise ValueError(f"run {run_id} has no timestamp interval")
        timing[run_id] = {
            "sample_count": len(times),
            "first_time_us": times[0],
            "last_time_us": times[-1],
            "delta_min_us": min(deltas),
            "delta_median_us": statistics.median(deltas),
            "delta_max_us": max(deltas),
            "nonpositive_delta_count": sum(delta <= 0 for delta in deltas),
            "non_10000us_delta_count": sum(delta != 10_000 for delta in deltas),
        }

    affine_pairs = {
        "current_from_raw": ("rawCurrent", "Current"),
        "voltage_a1_from_raw": ("rawVoltageA1", "VoltageA1"),
        "voltage_b1_from_raw": ("rawVoltageB1", "VoltageB1"),
    }
    conversions = {
        name: _affine_fit(numeric[raw], numeric[converted])
        for name, (raw, converted) in affine_pairs.items()
    }
    motor_voltage_residuals = [
        motor - (voltage_b - voltage_a)
        for motor, voltage_a, voltage_b in zip(
            numeric["MotorVoltage"], numeric["VoltageA1"], numeric["VoltageB1"]
        )
    ]
    report = {
        "schema_version": 1,
        "kind": "rne_pmdc_training_channel_audit",
        "source_sha256": manifest["source_sha256"],
        "partition_records_sha256": manifest.get("records_sha256"),
        "record_count": len(records),
        "timing": timing,
        "source_derived_affine_self_consistency": conversions,
        "motor_voltage_minus_b1_minus_a1": {
            "residual_mean_v": statistics.fmean(motor_voltage_residuals),
            "residual_rmse_v": math.sqrt(
                statistics.fmean(value * value for value in motor_voltage_residuals)
            ),
            "residual_max_abs_v": max(abs(value) for value in motor_voltage_residuals),
        },
        "timestamp_adjustments_applied": False,
        "values_repaired": False,
        "physical_accuracy_validated": False,
        "independent_voltage_calibration_qualified": False,
        "independent_current_calibration_qualified": False,
        "clock_accuracy_qualified": False,
    }
    digest_input = _canonical(report)
    report["audit_sha256"] = hashlib.sha256(digest_input.encode()).hexdigest()
    return report


def audit(path):
    manifest, records, trailer = read_partition(path)
    manifest = dict(manifest)
    manifest["records_sha256"] = trailer["records_sha256"]
    return audit_channels(manifest, records)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("training_partition", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    rendered = json.dumps(audit(args.training_partition), indent=2, sort_keys=True, allow_nan=False)
    with args.output.open("x", encoding="utf-8", newline="\n") as stream:
        stream.write(rendered + "\n")
