"""Seal both untouched PMDC final runs without evaluating or displaying responses."""

import argparse
import hashlib
import json
import xml.etree.ElementTree as ET
import zipfile
from dataclasses import dataclass
from pathlib import Path

from audit_pmdc_source import XMLNS, _open_xml, inspect_archive, shared_strings, verify_source, workbook_sheets
from convert_pmdc_prbs9 import ANNOTATION_TEXT, CHANNELS, SHEET_NAME, _json_line, _row_cells

FINAL_PROTOCOL_SHA256 = "c8ed0ce2f34fa90fd1797b42dee02f2aa721760f504d9590c1e3b659220fca53"


@dataclass(frozen=True)
class FinalRunLayout:
    """Exact source rows for one complete final run."""

    trial: int
    annotation_row: int
    header_row: int
    first_data_row: int
    last_data_row: int


FINAL_LAYOUT = (
    FinalRunLayout(10, 18100, 18101, 18102, 20110),
    FinalRunLayout(11, 20111, 20112, 20113, 22121),
)


def extract_final_records(archive, member, strings, layout=FINAL_LAYOUT):
    """Extract the exact declared rows and reject formulas, gaps, or extra final cells."""
    by_annotation = {run.annotation_row: run for run in layout}
    by_header = {run.header_row: run for run in layout}
    by_data_row = {
        row: run
        for run in layout
        for row in range(run.first_data_row, run.last_data_row + 1)
    }
    first_final_row = min(run.annotation_row for run in layout)
    last_final_row = max(run.last_data_row for run in layout)
    annotations = {}
    headers = set()
    records = []
    counts = {run.trial: 0 for run in layout}
    with _open_xml(archive, member) as stream:
        for _, element in ET.iterparse(stream, events=("end",)):
            if element.tag != f"{{{XMLNS}}}row":
                continue
            row_number = int(element.get("r", "0"))
            if row_number < first_final_row:
                element.clear()
                continue
            values, formulas = _row_cells(element, strings)
            if row_number in by_annotation:
                run = by_annotation[row_number]
                if values != {1: ANNOTATION_TEXT} or formulas:
                    raise ValueError(f"unexpected final annotation at source row {row_number}")
                annotations[run.trial] = {"source_row": row_number, "text": values[1]}
            elif row_number in by_header:
                run = by_header[row_number]
                header = tuple(values.get(column) for column in range(1, 13))
                if header != CHANNELS or formulas:
                    raise ValueError(f"unexpected final header at source row {row_number}")
                headers.add(run.trial)
            elif row_number in by_data_row:
                run = by_data_row[row_number]
                if formulas:
                    raise ValueError(f"formula cells are forbidden in final row {row_number}")
                if set(values) != set(range(1, 13)):
                    raise ValueError(f"final source row {row_number} is not a complete 12-cell record")
                records.append({
                    "partition": "final",
                    "run_id": f"prbs9_motor_a_trial_{run.trial:02d}",
                    "source_row": row_number,
                    "values": {name: values[index] for index, name in enumerate(CHANNELS, 1)},
                })
                counts[run.trial] += 1
            elif values and row_number > last_final_row:
                raise ValueError(f"unexpected populated row after frozen final runs: {row_number}")
            elif values:
                raise ValueError(f"unexpected populated row inside frozen final layout: {row_number}")
            element.clear()
    if set(annotations) != set(counts) or headers != set(counts):
        raise ValueError("final annotations or headers are incomplete")
    for run in layout:
        expected = run.last_data_row - run.first_data_row + 1
        if counts[run.trial] != expected:
            raise ValueError(f"final trial {run.trial} sample count drift")
    return records, counts, annotations


def seal(source, output):
    """Verify the source and exclusively create the unevaluated final JSONL seal."""
    source_bytes, source_sha256 = verify_source(source)
    with zipfile.ZipFile(source) as archive:
        container = inspect_archive(archive)
        sheets = dict(workbook_sheets(archive))
        if SHEET_NAME not in sheets:
            raise ValueError("pinned PRBS9 Motor A worksheet is missing")
        records, counts, annotations = extract_final_records(
            archive, sheets[SHEET_NAME], shared_strings(archive)
        )
    manifest = {
        "schema_version": 1,
        "kind": "rne_pmdc_prbs9_final_partition",
        "partition": "final",
        "final_protocol_sha256": FINAL_PROTOCOL_SHA256,
        "source_sha256": source_sha256,
        "source_bytes": source_bytes,
        "source_sheet": SHEET_NAME,
        "channels": list(CHANNELS),
        "run_ids": [f"prbs9_motor_a_trial_{trial:02d}" for trial in sorted(counts)],
        "sample_counts": {
            f"prbs9_motor_a_trial_{trial:02d}": count
            for trial, count in sorted(counts.items())
        },
        "run_annotations": {
            f"prbs9_motor_a_trial_{trial:02d}": annotation
            for trial, annotation in sorted(annotations.items())
        },
        "zip": container,
        "development_already_exposed": True,
        "final_partition_read": True,
        "final_responses_evaluated": False,
        "timestamp_adjustments_applied": False,
        "values_repaired": False,
        "physical_accuracy_validated": False,
    }
    record_lines = [_json_line(record) for record in records]
    digest = hashlib.sha256()
    for line in record_lines:
        digest.update(line.encode())
        digest.update(b"\n")
    trailer = {
        "kind": "rne_pmdc_prbs9_final_partition_end",
        "record_count": len(records),
        "records_sha256": digest.hexdigest(),
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
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    manifest, trailer = seal(args.workbook, args.output)
    print(json.dumps({
        "kind": manifest["kind"],
        "final_protocol_sha256": manifest["final_protocol_sha256"],
        "source_sha256": manifest["source_sha256"],
        "run_ids": manifest["run_ids"],
        "sample_counts": manifest["sample_counts"],
        "final_responses_evaluated": manifest["final_responses_evaluated"],
        "record_count": trailer["record_count"],
        "records_sha256": trailer["records_sha256"],
    }, indent=2, sort_keys=True))
