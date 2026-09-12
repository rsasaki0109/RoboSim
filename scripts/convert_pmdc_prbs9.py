"""Convert frozen PMDC PRBS9 Motor A train/dev runs without touching final trials.

All 12 source cells are retained as lexical strings.  No timestamp alignment,
interpolation, unit conversion, calibration, missing-value repair, or formula
evaluation is performed.  The untouched final partition is intentionally not a
CLI choice and must remain sealed until the model form and acceptance metrics are
frozen elsewhere.
"""

import argparse
import hashlib
import json
import xml.etree.ElementTree as ET
import zipfile
from pathlib import Path

from audit_pmdc_source import (
    XMLNS,
    _cell_text,
    _column_number,
    _open_xml,
    inspect_archive,
    shared_strings,
    verify_source,
    workbook_sheets,
)

SHEET_NAME = "PRBS9-MotorA"
CHANNELS = (
    "time",
    "encoderCount",
    "Velocity",
    "rawCurrent",
    "Current",
    "rawVoltageA1",
    "rawVoltageB1",
    "VoltageA1",
    "VoltageB1",
    "MotorVoltage",
    "MotorStatus",
    "PWM",
)
HEADER_ROWS = (2, 2013, 4024, 6035, 8046, 10057, 12068, 14079, 16090, 18101, 20112)
PARTITION_TRIALS = {"training": tuple(range(1, 9)), "development": (9,)}
ANNOTATION_TEXT = "Trial Description: PRBS9"

CHANNEL_SEMANTICS = {
    "time": "source microseconds; clock calibration unqualified",
    "encoderCount": "source encoder count",
    "Velocity": "source-derived revolutions per minute",
    "rawCurrent": "source ADC count",
    "Current": "source-derived ampere",
    "rawVoltageA1": "source ADC count",
    "rawVoltageB1": "source ADC count",
    "VoltageA1": "source-derived volt",
    "VoltageB1": "source-derived volt",
    "MotorVoltage": "source-derived terminal-potential difference in volt",
    "MotorStatus": "source motor state",
    "PWM": "source PWM command count",
}


def _json_line(value):
    return json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False)


def _row_cells(row, strings):
    values = {}
    formulas = []
    for cell in row.findall(f"{{{XMLNS}}}c"):
        reference = cell.get("r", "")
        column = _column_number(reference)
        if column in values:
            raise ValueError(f"duplicate cell column in source row: {reference}")
        values[column] = _cell_text(cell, strings)
        formula = cell.find(f"{{{XMLNS}}}f")
        if formula is not None:
            formulas.append(reference)
    return values, formulas


def extract_partition(archive, member, strings, partition):
    """Return strict, lossless records for one authorized pre-final partition."""
    if partition not in PARTITION_TRIALS:
        raise ValueError("only frozen training or development partitions are authorized")
    selected = set(PARTITION_TRIALS[partition])
    header_to_trial = {row: index + 1 for index, row in enumerate(HEADER_ROWS)}
    annotation_to_trial = {row - 1: index + 1 for index, row in enumerate(HEADER_ROWS)}
    stop_row = HEADER_ROWS[max(selected)] - 1
    current_trial = None
    seen_headers = []
    annotations = {}
    records = []
    per_trial = {trial: 0 for trial in sorted(selected)}

    with _open_xml(archive, member) as stream:
        for event, row in ET.iterparse(stream, events=("end",)):
            if row.tag != f"{{{XMLNS}}}row":
                continue
            row_number = int(row.get("r", "0"))
            if row_number >= stop_row:
                row.clear()
                break
            values, formulas = _row_cells(row, strings)
            if row_number in annotation_to_trial:
                trial = annotation_to_trial[row_number]
                if values != {1: ANNOTATION_TEXT} or formulas:
                    raise ValueError(f"unexpected trial annotation at source row {row_number}")
                if trial in selected:
                    annotations[trial] = {"source_row": row_number, "text": values[1]}
                current_trial = None
            elif row_number in header_to_trial:
                header = tuple(values.get(column) for column in range(1, 13))
                if header != CHANNELS:
                    raise ValueError(f"unexpected channel header at source row {row_number}")
                current_trial = header_to_trial[row_number]
                seen_headers.append(row_number)
            elif current_trial in selected:
                if formulas:
                    raise ValueError(f"formula cells are forbidden in selected data row {row_number}")
                if set(values) != set(range(1, 13)):
                    raise ValueError(f"selected source row {row_number} is not a complete 12-cell record")
                records.append({
                    "partition": partition,
                    "run_id": f"prbs9_motor_a_trial_{current_trial:02d}",
                    "source_row": row_number,
                    "values": {name: values[index] for index, name in enumerate(CHANNELS, 1)},
                })
                per_trial[current_trial] += 1
            row.clear()

    required_headers = list(HEADER_ROWS[:9] if partition == "development" else HEADER_ROWS[:8])
    if seen_headers != required_headers:
        raise ValueError("pre-final trial headers differ from the frozen split")
    if not records or any(count == 0 for count in per_trial.values()):
        raise ValueError("selected partition contains an empty trial")
    if set(annotations) != selected:
        raise ValueError("selected partition trial annotations are incomplete")
    return records, per_trial, annotations


def convert(source, output, partition):
    """Verify the pinned source and exclusively create a deterministic JSONL artifact."""
    source_bytes, source_sha256 = verify_source(source)
    with zipfile.ZipFile(source) as archive:
        container = inspect_archive(archive)
        sheets = dict(workbook_sheets(archive))
        if SHEET_NAME not in sheets:
            raise ValueError("pinned PRBS9 Motor A worksheet is missing")
        strings = shared_strings(archive)
        records, per_trial, annotations = extract_partition(
            archive, sheets[SHEET_NAME], strings, partition
        )

    manifest = {
        "schema_version": 1,
        "kind": "rne_pmdc_prbs9_partition",
        "partition": partition,
        "source_sha256": source_sha256,
        "source_bytes": source_bytes,
        "source_sheet": SHEET_NAME,
        "zip": container,
        "channels": list(CHANNELS),
        "channel_semantics": CHANNEL_SEMANTICS,
        "run_ids": sorted({record["run_id"] for record in records}),
        "sample_counts": {
            f"prbs9_motor_a_trial_{trial:02d}": count
            for trial, count in sorted(per_trial.items())
        },
        "run_annotations": {
            f"prbs9_motor_a_trial_{trial:02d}": annotation
            for trial, annotation in sorted(annotations.items())
        },
        "timestamp_adjustments_applied": False,
        "values_repaired": False,
        "converted_channels_calibrated": False,
        "physical_accuracy_validated": False,
        "final_partition_read": False,
    }
    record_lines = [_json_line(record) for record in records]
    record_digest = hashlib.sha256()
    for line in record_lines:
        record_digest.update(line.encode())
        record_digest.update(b"\n")
    trailer = {
        "kind": "rne_pmdc_prbs9_partition_end",
        "record_count": len(records),
        "records_sha256": record_digest.hexdigest(),
    }

    with output.open("x", encoding="utf-8", newline="\n") as stream:
        stream.write(_json_line(manifest) + "\n")
        for line in record_lines:
            stream.write(line + "\n")
        stream.write(_json_line(trailer) + "\n")
    return manifest, trailer


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("workbook", type=Path)
    parser.add_argument("--partition", required=True, choices=tuple(PARTITION_TRIALS))
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    result = convert(args.workbook, args.output, args.partition)
    print(json.dumps({"manifest": result[0], "trailer": result[1]}, indent=2, sort_keys=True))
